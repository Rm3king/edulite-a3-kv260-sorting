#!/usr/bin/python3
"""Bind automatic C3 entry/return to current feedback, with no stored-pose gate.

Numerical planning runs in an isolated spawned process; the sole CAN owner
continues holding and draining feedback. No hardware handle enters the worker.
"""
import copy, math, multiprocessing, time
from joint_trajectory import JointPlanner, vector, Q24
from septic_trajectory import movej_c3

POLICY = 'CURRENT_FEEDBACK_REPLAN'
MEASUREMENT_MARGIN_RAD = .02


def current_reference(p, motor):
    """Keep legal commands at calibrated endpoints despite small measured offset.

    The measured pose is preserved in the report. A <=0.02 rad overrun at an
    endpoint is captured by CSP at its legal endpoint, without asking the user
    to move it manually. This does not expand the command or IK workspace.
    """
    motor = vector(motor, 'current feedback')
    robot = [(x-o)/d for x,o,d in zip(motor,p.offset,p.direction)]
    # Four Q24 codes inward avoid sub-code septic rounding excursions at an
    # exact calibrated endpoint; this is <0.000014 degrees, not a start gate.
    inset = 4/Q24
    ref = [max(lo+inset,min(hi-inset,q)) for q,lo,hi in zip(robot,p.lower,p.upper)]
    margin=p.profile.get('capture_measurement_margin_rad',MEASUREMENT_MARGIN_RAD)
    if type(margin) not in (int,float) or not math.isfinite(margin) or not 0<margin<=.05:
        raise ValueError('invalid current capture observation margin')
    if max(abs(a-b) for a,b in zip(robot,ref)) > margin:
        raise ValueError('current measurement substantially outside calibrated travel')
    return p.positions(ref), [a-b for a,b in zip(p.to_motor(ref),motor)]


def rebind_job(job, p, motor):
    from septic_job import SCHEMA
    from motion_executor import verify_job
    if job.get('start_policy') != POLICY or job.get('schema') != SCHEMA:
        raise ValueError('live binding requires explicit C3 current-feedback policy')
    bound = copy.deepcopy(job)
    ref, adjustment = current_reference(p, motor)
    first = bound['tasks'][0]
    if first['kind'] != 'MoveJ': raise ValueError('live entry must be MoveJ')
    first['segments'] = movej_c3(p,ref,first['goal_rad'])
    bound['start_motor_rad'] = list(motor)
    bound['start_rad'] = ref
    bound['start_reference_adjustment_motor_rad'] = adjustment
    # An explicit return task preserves its commanded gripper state.
    for i,t in enumerate(bound['tasks']):
        if t.get('return_to_live_start'):
            if i == 0 or t['kind'] != 'MoveJ': raise ValueError('invalid live return')
            goal = ref[:6] + [t['goal_rad'][6]]
            t['segments'] = movej_c3(p,bound['tasks'][i-1]['goal_rad'],goal)
            t['goal_rad'] = goal
            # Later gripper-only tasks must follow the rebound arm coordinates.
            for later in bound['tasks'][i+1:]:
                if not later.get('gripper_only'): raise ValueError('only gripper tasks allowed after live return')
                g = ref[:6] + [later['goal_rad'][6]]
                later['segments'] = movej_c3(p,goal,g)
                later['goal_rad'] = g
                goal = g
            break
    bound['hardware_start_verified'] = True
    prepared = verify_job(bound,p,return_prepared=True)
    return bound,prepared


def _worker(conn,job,profile,gravity_profile=None):
    try:
        from motion_executor import verify_job,job_digest
        p=JointPlanner(profile)
        model=None
        if gravity_profile is not None:
            from gravity_trajectory import GravityModel,prepare_stream,validate_records
            from motion_runtime import MotionRuntime
            model=GravityModel(MotionRuntime(),p,gravity_profile)
        conn.send(('ready',None))
        motor=conn.recv()
        import pickle
        started=time.monotonic()
        bound,prepared=rebind_job(job,p,motor)
        timing=dict(replan_seconds=time.monotonic()-started)
        if model is not None:
            if bound['model_id']!=model.r.model_id:raise ValueError('tracking model mismatch')
            for task in bound['tasks']:task['gravity_alpha']=[1.,1.]
            started=time.monotonic();prepared=prepare_stream(model,bound['tasks'])
            timing['tracking_encode_seconds']=time.monotonic()-started
            started=time.monotonic()
            validate_records(prepared,bound,p,model)
            timing['fixed_point_validation_seconds']=time.monotonic()-started
            bound['tracking_profile_id']=model.id
        header=dict(bound,tasks=[{k:v for k,v in t.items() if k!='segments'} for t in bound['tasks']])
        conn.send(('ok',len(prepared)))
        conn.send_bytes(pickle.dumps(dict(job=header,job_sha256=job_digest(bound),timing=timing),protocol=5))
        index=0
        for task_index,task in enumerate(bound['tasks']):
            for plan in task['segments']:
                payload=pickle.dumps((task_index,plan,prepared[index]),protocol=5)
                if len(payload)>65536:raise ValueError('live binding segment IPC budget exceeded')
                conn.send_bytes(payload);index+=1
    except BaseException as exc:
        try:conn.send(('error',repr(exc)))
        except (BrokenPipeError,EOFError):pass
    finally: conn.close()


def start_binding_worker(job,profile,gravity_profile=None):
    """Spawn and import numerical dependencies BEFORE any hardware is enabled."""
    ctx=multiprocessing.get_context('spawn');parent,child=ctx.Pipe()
    process=ctx.Process(target=_worker,args=(child,job,str(profile),str(gravity_profile) if gravity_profile is not None else None))
    process.start();child.close()
    if not parent.poll(120.):
        process.terminate();process.join();parent.close()
        raise TimeoutError('numerical worker startup')
    kind,result=parent.recv()
    if kind!='ready':
        process.join();parent.close();raise RuntimeError('numerical worker startup: '+str(result))
    return parent,process


def bind_while_holding(executor, job, profile, worker, timeout_s=180.):
    """Only the parent touches CAN; all polynomial work stays in the worker."""
    from motion_executor import parameter
    import pickle
    s=executor.s
    motor=[s.read(mid,0x7019) for mid in range(1,8)]
    executor.report['start_motor_rad']=motor
    ref,adjustment=current_reference(executor.p,motor)
    if hasattr(executor,'capture_tracking_hold'):
        executor.capture_tracking_hold(motor)
    else:
        for mid,value in enumerate(executor.p.to_motor(ref),1):
            s.targets[mid]=value
            s.send(parameter(mid,0x7016,value))
    s.pump(.02)
    parent,process=worker
    begin=time.monotonic();parent.send(motor)
    counters=(executor.d.read(0x42c),executor.d.read(0x424))
    def wait_packet():
        while not parent.poll():
            s.pump(.003)
            if executor.d.read(0x424)!=counters[1]:
                raise RuntimeError('expiration during live planning')
            if time.monotonic()-begin>timeout_s:raise TimeoutError('current-feedback planning timeout')
            if not process.is_alive() and not parent.poll():raise RuntimeError('current-feedback worker exited')
    try:
        wait_packet();kind,count=parent.recv()
        if kind!='ok':raise ValueError('current-feedback planning: '+str(count))
        transfer_started=time.monotonic();maximum_decode=0.
        def receive_item():
            nonlocal maximum_decode
            wait_packet();raw=parent.recv_bytes(65536)
            decoded=time.monotonic();item=pickle.loads(raw)
            maximum_decode=max(maximum_decode,time.monotonic()-decoded)
            s.pump(.001)
            return item
        header=receive_item();bound=header['job']
        if hasattr(executor,'capture_tracking_hold') and bound.get('tracking_profile_id')!=executor.model.id:
            raise ValueError('worker tracking profile differs from enabled hold configuration')
        for task in bound['tasks']:task['segments']=[]
        prepared=[]
        for _ in range(count):
            task_index,plan,record=receive_item()
            identity=record.get('original_plan',record['plan'])
            if identity is not plan:raise ValueError('worker segment identity mismatch')
            bound['tasks'][task_index]['segments'].append(plan);prepared.append(record)
        executor.report['binding_transfer_seconds']=time.monotonic()-transfer_started
        executor.report['binding_decode_seconds']=maximum_decode
        executor.report['binding_job_sha256']=header['job_sha256']
        executor.report['binding_computation']=header.get('timing',{})
        executor.transport.prepared=prepared
        executor.transport.prepared_index=0
        executor.report['current_binding_seconds']=time.monotonic()-begin
        executor.report['start_reference_adjustment_motor_rad']=adjustment
        executor.event('CURRENT_FEEDBACK_BOUND',start_motor_rad=motor,segments=len(prepared))
        return bound
    finally:
        if process.is_alive():process.terminate()
        while process.is_alive():s.pump(.003)
        process.join(timeout=0);parent.close()
