"""Bounded stationary visual correction; one hardware owner, separate planning worker.

XY correction keeps the jaw fixed. An explicit single-nut episode may continue
into the verified grasp tail without returning home. Default validates offline.
The host supplies image-bound reviewed paths; a child process independently checks
C3 polynomials and installed bounds before returning records to the owner.
"""
import argparse, base64, fcntl, gzip, hashlib, json, math, multiprocessing as mp
import queue, signal, time, uuid, pickle, io
from pathlib import Path
from urllib.request import Request, urlopen
from calibration_motion import motion_gc_scope
from continuous_transfer import WRIST_RETURN_TEMPERATURE_C, TransferDevice, expected_grasp_height_offset_m, target_classes, path_digest, verify as verify_transfer, load_bundle, TaskOutcomeError, recover_task_outcome, check_batch_release_temperature
from control_continuity import WaitStateWriter, retain_until_supported, freeze_failure_evidence
from joint_trajectory import JointPlanner, Q24
from motion_executor import job_digest
from septic_job import verify_septic_job
from septic_trajectory import extrema
from visual_pick_trial import GraspExecutor
from canctl import CanDevice
from wrist_handeye_motion import CameraWorker, stationary_capture, verify as verify_initial


MIN_INTERIOR_SEGMENT_S=.04


def require_serviceable_segments(tasks):
    """Refuse, at rest, a path whose interior segment is too short for the executor loop to chain (it submits one segment per
    pass into a two-deep PL queue; a 15 ms segment starved the queue at non-zero velocity in motion, 2026-10-28)."""
    for task in tasks:
        d=[x['duration_s'] for x in task['segments']]
        for i in range(1,len(d)-1):
            if d[i]<MIN_INTERIOR_SEGMENT_S-1e-9:
                raise ValueError(f"Task {task.get('name')}: segment {i} lasts {d[i]*1000:.0f} ms, too short to execute")


def validate_grasp_reply(reply, request, job, planner):
    if not job.get('visual_servo',{}).get('integrated_grasp'):
        raise ValueError('Noncontact episode cannot start grasping')
    error=reply.get('pixel_error',float('inf'))
    if not math.isfinite(error) or error>5:raise ValueError('Convergence required before grasp')
    candidate=reply['candidate']
    if (candidate['start_rad']!=job['start_rad'] or candidate['start_motor_rad']!=job['start_motor_rad']
        or candidate['grasp_height_offset_m']!=expected_grasp_height_offset_m(target_classes(job))
        or candidate['tasks'][0]['segments']!=request['executed_prefix']
        or candidate['tasks'][0]['goal_rad']!=request['q_rad']
        or candidate['targets'][0]['source']!=job['targets'][0]['source']
        or candidate['targets'][0]['destination_uv']!=job['targets'][0]['destination_uv']):
        raise ValueError('Grasp continuation changed source, history, height or destination')
    require_serviceable_segments(candidate['tasks'][1:])   # the first task is the already executed prefix
    delta=reply['cumulative_tool_xy_m']
    if delta!=request['cumulative_tool_xy_m'] or math.hypot(*delta)>min(.035,float(job['visual_servo'].get('max_correction_m',.025)))+1e-6:
        raise ValueError('Grasp must use observed correction only')
    audit=candidate['geometry_audit']
    if not audit.get('wrist_camera_proxy_checked') or audit.get('camera_proxy_min_floor_clearance_m',-1)<.005:
        raise ValueError('Camera housing clearance required for grasp')
    if 'reviewed_bundle_b64' in reply:
        if (candidate['motor_lower_rad']!=job['motor_lower_rad'] or candidate['motor_upper_rad']!=job['motor_upper_rad']):
            raise ValueError('Reviewed continuation must retain installed guards')
        raw=gzip.decompress(base64.b64decode(reply.pop('reviewed_bundle_b64'),validate=True))
        if len(raw)>20_000_000:raise ValueError('Oversized reviewed bundle')
        import tempfile
        with tempfile.TemporaryDirectory(prefix='wrist_review_') as directory:
            path=Path(directory)/'bundle.json';path.write_bytes(raw)
            records=load_bundle(candidate,planner,path,reply.pop('reviewed_bundle_sha256'))
        bundled=True
    else:
        records=verify_transfer(candidate,planner);bundled=False
    for group in ([] if bundled else [records]+list(records.recoveries.values())):
        for record in group:
            for i in range(7):
                lo,hi=extrema(record['words'][8*i:8*i+8])
                if lo/Q24<job['motor_lower_rad'][i] or hi/Q24>job['motor_upper_rad'][i]:
                    raise ValueError('Grasp exceeds initially installed guards')
    reply['full_records']=records
    reply['records']=records[len(request['executed_prefix']):]
    return reply


JAW_EMPTY_RAD=1.58          # at or beyond this the jaws have (almost) reached the close command
JAW_STALL_MARGIN_RAD=.008   # stopped this far short of the command ...
JAW_STALL_CURRENT_A=.6      # ... with the current at the 0.65 A limit: something thin (M6 nut 1.585 rad / 0.651 A) blocks the jaws


def jaw_holds_part(actual_rad, command_rad, iqf_a):
    """A part is held when the jaws stopped before JAW_EMPTY_RAD, or stalled short of the command at the current limit.
    An empty close runs on to the command with low current; a part gripped on a thread (1.5997 rad / 0.325 A) is not confirmed."""
    if actual_rad < JAW_EMPTY_RAD:
        return True
    return actual_rad <= command_rad-JAW_STALL_MARGIN_RAD and iqf_a >= JAW_STALL_CURRENT_A


class StallWatch:
    """Diagnostics only: when no heartbeat reaches the PL for `limit_s`, record where the main thread is.

    2026-10-30 run13: after nut_0_place completed the owner sent nothing for 0.74 s, the PL lease (100 ms) expired and
    output stopped with the arm low over the tray; the logs could not say where it was. Behaviour is unchanged: the wrapped
    heartbeat is the original one, the watcher only reads a timestamp and writes a file."""
    def __init__(self, device, path, limit_s=.06, period=.01):
        import threading
        self.path, self.limit, self.period, self.last, self.main = Path(path), limit_s, period, time.monotonic(), threading.main_thread().ident
        self.stop, self.reported, self.thread = threading.Event(), False, None
        original = getattr(device, 'heartbeat', None)
        if original is None:                 # a device without a heartbeat (test fakes): nothing to watch
            return

        def heartbeat(*a, **k):
            self.last = time.monotonic()
            self.reported = False
            return original(*a, **k)
        device.heartbeat = heartbeat
        self.thread = threading.Thread(target=self.loop, daemon=True)
        self.thread.start()

    def loop(self):
        import sys, traceback
        while not self.stop.wait(self.period):
            gap = time.monotonic() - self.last
            if gap > self.limit and not self.reported:
                self.reported = True
                frame = sys._current_frames().get(self.main)
                with self.path.open('a') as f:
                    f.write('STALL %.3f s without heartbeat at monotonic %.3f\n' % (gap, time.monotonic()))
                    f.write(''.join(traceback.format_stack(frame)) if frame else 'main thread frame unavailable\n')
                    f.write('\n')

    def close(self):
        self.stop.set()
        if self.thread:
            self.thread.join(1)


def execute_grasp_tail(executor, reply, out, report, continuous=None):
    """Continue from this hover in the SAME enabled session; no start movement.

    continuous: the wrist_continuous module when the PL correction channel holds offsets for this grasp: pose checks include
    them, the channel is switched off after the lift (offsets slew to zero in the transfer) and before any reviewed recovery."""
    candidate=reply['candidate'];records=reply['full_records']
    executor.transport.prepared=reply['records'];executor.transport.prepared_index=0
    fade=None;fade_s=float(((candidate.get('visual_servo') or {}).get('continuous') or {}).get('fade_s',0.)) if continuous else 0.
    try:
        for index,task in enumerate(candidate['tasks'][1:],1):
            name=task['name'];report['active_task']=name
            executor.event('ONLINE_GRASP_TASK',name=name)
            executor.execute_segments(task['segments']);report['last_completed_task']=name
            if name.endswith('_place') and fade:fade.finish()     # normally long done (0.8 s into a 2-6 s transfer)
            if name.endswith(('_approach','_place')):
                goal=continuous.offset_goal(executor,task['goal_rad']) if continuous else task['goal_rad']
                report.setdefault('pose_checks',[]).append(dict(task=name,**executor.settle_arm(goal,seconds=.08)))
            if name.endswith(('_close','_open')):
                executor.pump(.10);actual=executor.s.read(7,0x7019)
                iqf=executor.s.read(7,0x701a);report.setdefault('jaw_observations',[]).append(dict(task=name,actual_rad=actual,command_rad=task['goal_rad'][6],iqf_a=iqf))
                executor.event('JAW_OUTCOME',task=name,actual_rad=actual)
                if name.endswith('_close') and not jaw_holds_part(actual,task['goal_rad'][6],iqf):raise TaskOutcomeError('Empty closure: grasp not confirmed')
                if name.endswith('_open') and abs(actual-1.12)>.015:raise TaskOutcomeError('Jaw release not reached')
                chain=candidate.get('chain')
                if name.endswith('_open') and chain and chain['index']<chain['count']-1:
                    # Another nut follows: stop at this reviewed empty-jaw endpoint if the arm is hot.
                    check_batch_release_temperature(executor,dict(batch_return_temperature_c=WRIST_RETURN_TEMPERATURE_C),name)
            if name.endswith('_lift'):
                actual,iqf=executor.s.read(7,0x7019),executor.s.read(7,0x701a)
                report.setdefault('jaw_observations',[]).append(dict(task=name,actual_rad=actual,iqf_a=iqf))
                if not jaw_holds_part(actual,task['goal_rad'][6],iqf):
                    # The part slipped out while lifting (measured 1.600 rad / 0.055 A after a 1.574 / 0.651 close). There is no reviewed
                    # return from here, and carrying an empty jaw through the remaining reviewed path is harmless: record it and go on.
                    report.setdefault('parts_lost',[]).append(dict(task=name,actual_rad=actual,iqf_a=iqf))
                    executor.event('PART_LOST_AFTER_LIFT',task=name,actual_rad=actual,iqf_a=iqf)
                # lifted clear of the tray: the offsets go to zero during the transfer, smoothly when a fade time is set
                if continuous:
                    if fade_s>0:fade=continuous.Fade(executor,fade_s)
                    else:continuous.disable(executor.d)
            if name.endswith(('_approach','_lift','_open')):
                (Path(out)/(f'snapshot_{index}.json' if not candidate.get('chain') or candidate['chain']['index']==0 else f"nut{candidate['chain']['index']}_snapshot_{index}.json")).write_text(json.dumps(dict(task=name,board_monotonic=time.monotonic())))
        report['return_endpoint']=executor.settle_arm(candidate['tasks'][-1]['goal_rad'])
        return 'COMPLETED_ONLINE_GRASP_MOTION_VERIFY_IMAGES'
    except TaskOutcomeError as error:
        report['task_outcome_error']=repr(error)
        if fade:fade.finish()
        if continuous:
            continuous.disable(executor.d);continuous.wait_zero(executor)
            # Zero PL offsets are commanded positions, not proof that the joints
            # have reached them. Keep servicing control until the six arm axes
            # settle at the reviewed nominal return start (original 4 mrad gate).
            branch=candidate.get('recoveries',{}).get(task['name'])
            if branch is not None:
                report['recovery_zero_settle']=executor.settle_arm(branch['start_rad'],seconds=.2)
        recover_task_outcome(executor,candidate,records,task['name'],report)
        return 'TASK_FAILED_RETURNED_SUPPORTED'


def validate_reply(reply, request, job, planner):
    for key in ('nonce', 'job_sha256', 'frame_sha256', 'iteration'):
        if reply.get(key) != request[key]:
            raise ValueError('Stale/unbound visual reply: '+key)
    if reply.get('action') in ('RETURN', 'CONVERGED'):
        if reply['action'] == 'CONVERGED':
            error = reply.get('pixel_error', float('inf'))
            if not math.isfinite(error) or error > 5:
                raise ValueError('False image convergence')
        return reply
    if reply.get('action')=='GRASP':return validate_grasp_reply(reply,request,job,planner)
    if reply.get('action') != 'MOVE':
        raise ValueError('Unknown visual action')
    candidate = reply['candidate']
    task, branch = candidate['tasks']
    require_serviceable_segments([task, branch])
    current = request['q_rad']
    home = job.get('chain',{}).get('support_rad',job['start_rad'])
    if (candidate['start_rad'] != current or task['start_rad'] != current
            or branch['start_rad'] != task['goal_rad'] or branch['goal_rad'] != home
            or not branch.get('return_to_live_start')):
        raise ValueError('Online step/return endpoint mismatch')
    delta = reply['cumulative_tool_xy_m']
    if len(delta) != 2 or not all(math.isfinite(v) for v in delta) or math.hypot(*delta) > min(.035,float(job['visual_servo'].get('max_correction_m',.025)))+1e-6:
        raise ValueError('Online correction exceeds configured noncontact envelope')
    old = request['cumulative_tool_xy_m']
    if math.hypot(*(a-b for a,b in zip(delta,old))) > .012001:
        raise ValueError('Online step exceeds 12mm')
    if max(abs(a-b) for a,b in zip(current[:6],task['goal_rad'][:6])) > .20:
        raise ValueError('Online joint branch jump')
    audit = candidate['geometry_audit']
    if (audit.get('status') != 'PASS' or audit.get('unexpected')
            or not audit.get('wrist_camera_proxy_checked')
            or audit.get('trajectory_sha256') != path_digest(candidate)):
        raise ValueError('Exact online geometry audit required')
    jaw = home[6]
    for t in (task, branch):
        for s in t['segments']:
            row = s['coefficients_rad'][6]
            if abs(row[0]-jaw)>1e-9 or any(abs(v)>1e-12 for v in row[1:]):
                raise ValueError('Online jaw motion forbidden')
    groups = []
    for t in (task, branch):
        records = verify_septic_job(dict(schema=job['schema'],profile_id=job['profile_id'],
            start_rad=t['start_rad'],tasks=[t]),planner,return_prepared=True)
        for record in records:
            for i in range(7):
                lo,hi=extrema(record['words'][8*i:8*i+8])
                if lo/Q24 < job['motor_lower_rad'][i] or hi/Q24 > job['motor_upper_rad'][i]:
                    raise ValueError('Online path exceeds initially installed guards')
        groups.append(records)
    reply['records'],reply['return_records']=groups
    return reply


REBASE_SAME_KEYS=('schema','policy','profile_id','model_id','motion_mask','start_rad','start_motor_rad','open_rad','close_rad',
                  'grasp_height_offset_m','gripper_open_current_a','gripper_close_current_a','chain','visual_servo','hardware_start_verified')


def validate_rebase(reply, request, old, planner):
    """A re-planned job for the next part of a chain (the part was moved), received at the previous exit pose.

    It must be the same episode in every respect except the source: same start (this exit pose), chain, profile, gripper and
    servo settings, same part class and destination slot, same exit pose (the next job of the chain starts there), all
    tasks serviceable, a passed geometry audit with the camera proxy, and every record inside the guards installed at enable."""
    if reply.get('nonce')!=request['nonce'] or reply.get('action')!='REPLAN':raise ValueError('Unbound rebase reply')
    new=json.loads(gzip.decompress(base64.b64decode(reply['job_gz_b64'],validate=True)))
    def same(a,b):   # equal up to float noise (a re-run of the same IK differs by ~1e-15)
        if isinstance(a,dict) and isinstance(b,dict):return a.keys()==b.keys() and all(same(a[k],b[k]) for k in a)
        if isinstance(a,list) and isinstance(b,list):return len(a)==len(b) and all(same(x,y) for x,y in zip(a,b))
        if isinstance(a,(int,float)) and isinstance(b,(int,float)) and not isinstance(a,bool) and not isinstance(b,bool):return abs(a-b)<=1e-9
        return a==b
    for key in REBASE_SAME_KEYS:
        if not same(new.get(key),old.get(key)):raise ValueError('Rebase changed '+key)
    if len(new.get('targets',[]))!=1 or new['targets'][0]['part_class']!=old['targets'][0]['part_class']:raise ValueError('Rebase changed the part')
    if new['targets'][0]['destination_uv']!=old['targets'][0]['destination_uv']:raise ValueError('Rebase changed the destination')
    if [t['name'] for t in new['tasks']]!=[t['name'] for t in old['tasks']] or max(abs(a-b) for a,b in zip(new['tasks'][-1]['goal_rad'],old['tasks'][-1]['goal_rad']))>1e-6:
        raise ValueError('Rebase changed the task sequence or the exit pose')
    if not same(new['start_rad'],request['q_rad']):raise ValueError('Rebase does not start at this pose')
    audit=new.get('geometry_audit',{})
    if not str(audit.get('status','')).startswith('PASS') or not audit.get('wrist_camera_proxy_checked') or audit.get('camera_proxy_min_floor_clearance_m',-1)<.005:
        raise ValueError('Rebase lacks a passed geometry audit')
    lower,upper=request['guards']
    if any(a<b for a,b in zip(new['motor_lower_rad'],lower)) or any(a>b for a,b in zip(new['motor_upper_rad'],upper)):
        raise ValueError('Rebase bounds exceed the installed guards')
    require_serviceable_segments(new['tasks'])
    raw=gzip.decompress(base64.b64decode(reply['bundle_gz_b64'],validate=True))
    if len(raw)>20_000_000:raise ValueError('Oversized rebase bundle')
    import tempfile
    with tempfile.TemporaryDirectory(prefix='wrist_rebase_') as directory:
        path=Path(directory)/'bundle.json';path.write_bytes(raw)
        records=load_bundle(new,planner,path,reply['bundle_sha256'])
    for group in [records]+list(records.recoveries.values()):
        for record in group:
            for i in range(7):
                lo,hi=extrema(record['words'][8*i:8*i+8])
                if lo/Q24<lower[i] or hi/Q24>upper[i]:raise ValueError('Rebase exceeds the installed guards')
    digest=job_digest(new)
    if digest!=reply.get('job_sha256'):raise ValueError('Rebase job digest mismatch')
    return dict(job=new,validated=(records,records.recoveries),digest=digest)


def planning_worker(requests, results, job, profile, out, rebase_results=None):
    planner=JointPlanner(profile);pending=None
    while True:
        request=requests.get()
        if request is None:return
        if request.get('kind')=='adopt':
            # the owner took the re-planned job: later visual replies are bound to it
            job=pending['job'];pending=None;continue
        if request.get('kind')=='rebase':
            try:
                started=time.monotonic()
                raw=json.dumps({k:request[k] for k in ('nonce','job_sha256','q_rad')}).encode()
                req=Request(job['visual_servo']['endpoint']+'rebase',raw,{'Content-Type':'application/json'})
                with urlopen(req,timeout=40.) as response:encoded=response.read(30_000_001)
                if len(encoded)>30_000_000:raise ValueError('Oversized rebase reply')
                reply=json.loads(encoded)
                if reply.get('nonce')!=request['nonce']:raise ValueError('Unbound rebase reply')
                if reply.get('action')=='KEEP':rebase_results.put(dict(ok=True,action='KEEP',reason=reply.get('reason'),host_s=time.monotonic()-started));continue
                received=time.monotonic();pending=validate_rebase(reply,request,job,planner)
                prepared=Path(out)/f"rebase_{request['nonce']}.pkl"
                with prepared.open('wb') as stream:pickle.dump(pending,stream,protocol=4)
                rebase_results.put(dict(ok=True,action='REPLAN',reply_file=str(prepared),moved_px=reply.get('moved_px'),host_s=received-started,validation_s=time.monotonic()-received))
            except Exception as error:
                pending=None;rebase_results.put(dict(ok=False,error=repr(error)))
            continue
        try:
            started=time.monotonic()
            payload=dict(request)
            payload['images']=[base64.b64encode((Path(out)/request['capture_name']/f'frame_{i}.jpg').read_bytes()).decode() for i in range(3)]
            raw=json.dumps(payload).encode()
            req=Request(job['visual_servo']['endpoint'],raw,{'Content-Type':'application/json'})
            with urlopen(req,timeout=14.) as response:
                encoded=response.read(8_000_001)
            if len(encoded)>8_000_000:raise ValueError('Oversized reply')
            received=time.monotonic()
            reply=validate_reply(json.loads(encoded),request,job,planner)
            reply['board_timing_s']=dict(host_roundtrip=received-started,validation=time.monotonic()-received)
            prepared=Path(out)/(request['capture_name']+'_prepared.pkl')
            with prepared.open('wb') as stream:pickle.dump(reply,stream,protocol=4)
            results.put(dict(ok=True,reply_file=str(prepared)))
        except Exception as error:
            results.put(dict(ok=False,error=repr(error)))


def read_prepared_reply(executor,path):
    """Service the controller between pickle frames, not after a large decode."""
    class Reader:
        def __init__(self,stream):self.stream=stream
        def read(self,n=-1):
            executor.pump(.001);executor.diagnostics()
            return self.stream.read(n)
        def readline(self):
            executor.pump(.001);executor.diagnostics()
            return self.stream.readline()
    with open(path,'rb') as stream:return pickle.Unpickler(Reader(stream)).load()


def bounded_reply(executor, results, timeout=32.):
    deadline=time.monotonic()+timeout
    while time.monotonic()<deadline:
        executor.pump(.003);executor.diagnostics()
        try:
            answer=results.get_nowait()
            if answer.get('ok') and 'reply_file' in answer:
                answer=dict({k:v for k,v in answer.items() if k!='reply_file'},ok=True,reply=read_prepared_reply(executor,answer['reply_file']))
            return answer
        except queue.Empty:pass
    return dict(ok=False,error='Visual/planning deadline exceeded')


def servo_nut(index, job, digest, returns, executor, camera, requests, results, out, report, state):
    """Hover, correct, grasp and place one nut. Returns the nut's status; the arm is then
    at the exit pose (more nuts follow in the chain), at support (last nut / any return)."""
    task=job['tasks'][0];executor.execute_segments(task['segments'])
    q=task['goal_rad'];before=executor.settle_arm(q,seconds=.2)
    branch=job['recoveries'][task['name']];return_records=returns[task['name']]
    cumulative=[0.,0.];started=time.monotonic();prefix=list(task['segments']);status=None
    for iteration in range(5):
        if time.monotonic()-started>35 or max(executor.s.require(i,2)['temperature'] for i in range(1,8))>=WRIST_RETURN_TEMPERATURE_C:
            status='RETURN_BUDGET_OR_TEMPERATURE';break
        name=f'servo_{iteration:02d}' if index==0 else f'nut{index}_servo_{iteration:02d}'
        capture=stationary_capture(executor,camera,name,before)
        report.setdefault('captures',[]).append(capture)
        for _ in range(2):  # a joint still creeping into place (J3 under load, 3 mrad seen) is waited out, not returned for
            if capture.get('error')!='Joint motion during image interval':break
            before=executor.settle_arm(q,seconds=.3)
            capture=stationary_capture(executor,camera,name,before)
            report['captures'].append(capture)
        if capture['status']!='CAPTURED':
            status='RETURN_CAPTURE_REJECTED';break
        session=capture['frames'][0]['headers']['X-Camera-Session']
        if state['camera_session'] is not None and session!=state['camera_session']:
            status='RETURN_CAMERA_RESTARTED';break
        state['camera_session']=session
        request=dict(nonce=uuid.uuid4().hex,job_sha256=digest,iteration=iteration,
            frame_sha256=[f['sha256'] for f in capture['frames']],camera_session=session,
            q_rad=q,cumulative_tool_xy_m=cumulative,capture_name=name,executed_prefix=list(prefix))
        (out/f'{name}_request.json').write_text(json.dumps(request))
        requests.put(request);answer=bounded_reply(executor,results)
        # One rejected image set is not a reason for a whole return trip.
        # Retry only an answered vision rejection, never a timed-out worker
        # or a motor/control error (which could leave a stale reply queued).
        if not answer['ok'] and 'HTTPError 422' in answer.get('error',''):
            executor.event('VISION_RETRY_SAME_POSE',iteration=iteration)
            retry_name=name+'_retry';before=executor.settle_arm(q,seconds=.15)
            retry=stationary_capture(executor,camera,retry_name,before)
            report.setdefault('captures',[]).append(retry)
            if retry['status']=='CAPTURED' and retry['frames'][0]['headers']['X-Camera-Session']==state['camera_session']:
                request=dict(request,nonce=uuid.uuid4().hex,capture_name=retry_name,
                    frame_sha256=[f['sha256'] for f in retry['frames']])
                (out/f'{retry_name}_request.json').write_text(json.dumps(request))
                requests.put(request);answer=bounded_reply(executor,results)
        if not answer['ok']:
            report.update(visual_error=answer['error']);status='RETURN_VISION_REJECTED';break
        reply=answer['reply']
        report.setdefault('visual_replies',[]).append(dict({k:v for k,v in reply.items() if k not in ('records','return_records','full_records','candidate')},nut=index))
        executor.event('VISUAL_SERVO_REPLY',nut=index,iteration=iteration,action=reply['action'],pixel_error=reply.get('pixel_error'))
        if reply['action']=='GRASP':
            return execute_grasp_tail(executor,reply,out,report)
        if reply['action']!='MOVE':
            status='RETURN_'+reply['action'];break
        # Plans and records arrive in one pickle, preserving plan identity.
        move,new_return=reply['candidate']['tasks']
        executor.transport.prepared=reply['records'];executor.transport.prepared_index=0
        executor.execute_segments(move['segments'])
        q=move['goal_rad'];before=executor.settle_arm(q,seconds=.2)
        branch=new_return;return_records=reply['return_records'];cumulative=reply['cumulative_tool_xy_m'];prefix.extend(move['segments'])
    else:status='RETURN_ITERATION_LIMIT'
    executor.transport.prepared=return_records;executor.transport.prepared_index=0
    executor.execute_segments(branch['segments'])
    report['return_endpoint']=executor.settle_arm(job['chain']['support_rad'] if job.get('chain') else job['start_rad'])
    return status


def needs_planning_worker(job):
    """The stop-and-look servo and a re-plan are answered through the job's planning worker process; a continuous part is not
    (servo_continuous never touches it) and each idle worker costs ~4 s of imports on the board's 4 cores."""
    servo=job.get('visual_servo',{})
    return not servo.get('continuous') or bool(servo.get('rebase'))


def execute(jobs, planner, profile, out, validated, vision_loop=None):
    """Run one nut, or a continuous chain of nuts in a single enabled session.

    jobs[k+1] starts at jobs[k]'s exit pose and only the last job returns to support;
    any failure still returns to support through the reviewed recovery of the active job.
    """
    job=jobs[0];initial=validated[0][0]
    # Hashing a whole job takes ~0.3 s on the board, longer than the 100 ms controller
    # lease: every digest is computed here, before anything is enabled, never in the loop.
    digests=[job_digest(j) for j in jobs]
    out=Path(out);out.mkdir(parents=True,exist_ok=False)
    report=dict(status='RUNNING',commands=[],frames=[],feedback=[],readbacks={},reset_confirmed=[],stop_errors={},job_sha256=digests[0],flash_writes=0,zero_writes=0,nut_statuses=[])
    (out/'job.json').write_text(json.dumps(job))
    ctx=mp.get_context('spawn');channels=[(ctx.Queue(),ctx.Queue(),ctx.Queue()) for _ in jobs]
    processes=[ctx.Process(target=planning_worker,args=(requests,results,j,str(profile),str(out),rebases),daemon=True)
               for j,(requests,results,rebases) in zip(jobs,channels) if needs_planning_worker(j)]
    for process in processes:process.start()
    # One guard installation must cover every nut of the chain (each job's own bounds
    # remain enforced when its replies are validated).
    guards=job if len(jobs)==1 else dict(job,motor_lower_rad=[min(j['motor_lower_rad'][i] for j in jobs) for i in range(7)],
                                          motor_upper_rad=[max(j['motor_upper_rad'][i] for j in jobs) for i in range(7)])
    supported=False;state=dict(camera_session=None)
    import wrist_continuous as wc
    correction=wc.merged_config(jobs)
    if correction:wc.check_before_enable()            # slow (bitstream hash): never between enable and the first pump
    # started and ready before anything is enabled (main() may have started it already, to overlap its start-up with job loading)
    if vision_loop is not None and not correction:vision_loop.close();vision_loop=None
    loop=(vision_loop if vision_loop is not None else wc.VisionLoop(out,wait=False)) if correction else None
    if loop is not None:loop.wait_ready()
    with motion_gc_scope(),open('/tmp/kv260_gripper_commission.lock','a') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        device=TransferDevice(CanDevice())
        from continuous_transfer import jaw_ceiling
        device.jaw_ceiling=max(jaw_ceiling(j) for j in jobs)   # validated per job above; 1.67 only with an M6 nut close
        stall_watch=StallWatch(device,Path(out)/'stalls.txt')
        if not job.get('visual_servo',{}).get('integrated_grasp'):device.marker_jaw_hold=job['start_motor_rad'][6]
        executor=GraspExecutor(planner,device,report,initial)
        executor.wait_state_writer=WaitStateWriter(out);camera=CameraWorker(out)
        try:
            executor.prepare(guards)
            if correction:wc.configure(device,correction);report['correction_config']=correction
            executor.settle_arm(job['start_rad'],seconds=.15);executor.release_host_hold()
            for index,(current,(records,returns)) in enumerate(zip(jobs,validated)):
                if index>0 and current.get('visual_servo',{}).get('rebase'):
                    # The host looked at the scene again while the previous part was placed. If this part was moved it sends a
                    # re-planned job from this exit pose; any refusal, error or timeout keeps the job validated before enable.
                    nonce=uuid.uuid4().hex;started=time.monotonic()
                    channels[index][0].put(dict(kind='rebase',nonce=nonce,job_sha256=digests[index],q_rad=current['start_rad'],
                                                guards=(guards['motor_lower_rad'],guards['motor_upper_rad'])))
                    answer=bounded_reply(executor,channels[index][2],timeout=45.)
                    if answer.get('ok') and answer.get('action')=='REPLAN':
                        rebased=answer['reply'];current=rebased['job'];records,returns=rebased['validated'];digests[index]=rebased['digest'];jobs[index]=current
                        channels[index][0].put(dict(kind='adopt'))
                    report.setdefault('rebases',[]).append(dict(nut=index,action=answer.get('action'),error=answer.get('error'),moved_px=answer.get('moved_px'),wait_s=time.monotonic()-started))
                    executor.event('REBASE',nut=index,action=answer.get('action','KEPT_AFTER_ERROR'),error=answer.get('error'),wait_s=round(time.monotonic()-started,2))
                if index>0:
                    # Handover: the arm must be where the reviewed plan of this nut begins.
                    executor.settle_arm(current['start_rad'],seconds=.15)
                    executor.transport.prepared=records;executor.transport.prepared_index=0
                    executor.event('CHAIN_NEXT_NUT',nut=index)
                requests,results,_=channels[index]
                if current.get('visual_servo',{}).get('continuous'):
                    status=wc.servo_continuous(index,current,digests[index],records,returns,executor,loop,out,report)
                else:status=servo_nut(index,current,digests[index],returns,executor,camera,requests,results,out,report,state)
                report['nut_statuses'].append(status);report['status']=status
                if status!='COMPLETED_ONLINE_GRASP_MOTION_VERIFY_IMAGES' or index==len(jobs)-1:break
            # Standing user authorization applies ONLY after this verified return.
            for i in range(1,8):executor.s.require(i,2)
            supported=True;executor.event('VERIFIED_START_RETURN',status=report['status'])
            if len(report['nut_statuses'])==len(jobs) and set(report['nut_statuses'])=={'COMPLETED_ONLINE_GRASP_MOTION_VERIFY_IMAGES'}:
                report['status']='COMPLETED_ONLINE_GRASP_MOTION_VERIFY_IMAGES'
        except (Exception,KeyboardInterrupt) as error:
            report.update(status='FAILED',error=repr(error));executor.event('VISUAL_SERVO_FAILED',error=repr(error))
            if report.get('enable_attempted'):
                # Persist the fault window before a possibly long operator wait trims the live buffers.
                freeze_failure_evidence(executor,out,report,error=repr(error),task=report.get('active_task'))
                retain_until_supported(executor,out,report);supported=True
        finally:
            if report.get('owned'):
                if report.get('enable_attempted'):
                    if not supported:retain_until_supported(executor,out,report)
                    try:executor.s.stop_all()
                    except (Exception,KeyboardInterrupt) as error:
                        report['stop_errors']['fatal']=repr(error);retain_until_supported(executor,out,report)
                else:
                    device.stop();device.write(0x5a0,0)
                    for i in range(1,8):device.write(device.pp_base(i),0)
            if report.get('enable_attempted') and (sorted(report['reset_confirmed'])!=list(range(1,8)) or report['stop_errors']):
                report['status']='FAIL_DISABLE_CONFIRMATION'
            camera.requests.put(None);executor.wait_state_writer.close()
            if loop is not None:loop.close()
            stall_watch.close()
            report.update(final_controller=device.status(),final_active_mask=device.read(0x5a8),heartbeat_max_gap_s=device.max_gap)
            device.close()
            summary={k:report.get(k) for k in ('status','nut_statuses','error','task_outcome_error','jaw_observations','visual_error','visual_replies','reset_confirmed','stop_errors','final_controller','final_active_mask','heartbeat_max_gap_s')}
            (out/'runtime_summary.json').write_text(json.dumps(summary))
            (out/'result.json.gz').write_bytes(gzip.compress(json.dumps(report).encode()))
    for process in processes:process.terminate()
    for process in processes:process.join(timeout=1)
    return report


def load_job(job_path, bundle_path, bundle_sha, planner):
    job=json.loads(Path(job_path).read_text())
    servo=job.get('visual_servo',{})
    if servo.get('normal_return_disable_authorized') is not True or not job.get('hardware_start_verified'):
        raise ValueError('Explicit normal-return disable authorization and measured start required')
    if servo.get('integrated_grasp'):
        if len(job.get('targets',[]))!=1 or job['targets'][0]['part_class'] not in ('nut','screw') or abs(job['start_rad'][6]-1.12)>1e-6:
            raise ValueError('Integrated trial requires one nut or screw and working-open start')
        if servo.get('continuous'):
            import wrist_continuous;wrist_continuous.validate_config(job)
        if bundle_path is not None:
            if not bundle_sha:raise ValueError('Reviewed bundle digest required')
            records=load_bundle(job,planner,bundle_path,bundle_sha)
        else:records=verify_transfer(job,planner)
        validated=(records,records.recoveries)
    else:
        if len(job['tasks'])!=2 or not servo.get('noncontact_only'):raise ValueError('Noncontact episode required')
        validated=verify_initial(job,planner)
    return job,validated


def check_chain(jobs):
    """Static cross-job binding of a continuous chain (checked before anything is enabled)."""
    if len(jobs)==1:
        if jobs[0].get('chain') is not None and jobs[0]['chain']['count']!=1:raise ValueError('Chain job needs its followers')
        return
    first=jobs[0]
    for index,job in enumerate(jobs):
        chain=job.get('chain')
        if (chain is None or chain['index']!=index or chain['count']!=len(jobs) or chain['support_rad']!=first['chain']['support_rad']
                or job['profile_id']!=first['profile_id'] or not job.get('visual_servo',{}).get('integrated_grasp')):
            raise ValueError('Chain jobs are not one bound sequence')
        if index>0:
            exit_pose=jobs[index-1]['tasks'][-1]['goal_rad']
            if max(abs(a-b) for a,b in zip(job['start_rad'],exit_pose))>1e-9 or jobs[index-1]['tasks'][-1]['name']!=jobs[index-1]['targets'][0]['target_id']+'_exit':
                raise ValueError('Chain job does not start at the previous exit pose')
        if jobs[index]['visual_servo']['endpoint'] in [j['visual_servo']['endpoint'] for j in jobs[:index]]:
            raise ValueError('Each chain job needs its own vision endpoint')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    for key in ('job','profile','out'):parser.add_argument('--'+key,type=Path,required=True)
    parser.add_argument('--reviewed-bundle',type=Path);parser.add_argument('--bundle-sha256')
    parser.add_argument('--chain',type=Path,help='JSON list of {job,reviewed_bundle,bundle_sha256} run after --job in the same session')
    parser.add_argument('--execute-attended',action='store_true')
    parser.add_argument('--prestart-vision',action='store_true',help='start the wrist vision process before the jobs are loaded (a chain with continuous parts)')
    args=parser.parse_args()
    vision=None
    if args.prestart_vision and args.execute_attended:
        import wrist_continuous as wc
        vision=wc.VisionLoop(None,wait=False,log=args.out.with_name(args.out.name+'_vision_loop.log'))
    try:
        _run(args,vision)
    finally:
        if vision is not None and vision.process.poll() is None:vision.close()   # also when loading or validating a job fails


def _run(args,vision):
    planner=JointPlanner(args.profile)
    job,validated=load_job(args.job,args.reviewed_bundle,args.bundle_sha256,planner)
    jobs=[job];validations=[validated]
    if args.chain is not None:
        for entry in json.loads(args.chain.read_text()):
            followed,checked=load_job(entry['job'],Path(entry['reviewed_bundle']),entry['bundle_sha256'],planner)
            jobs.append(followed);validations.append(checked)
    check_chain(jobs)
    if not args.execute_attended:print(json.dumps(dict(status='OFFLINE_VALIDATED',segments=[len(v[0]) for v in validations])));return
    def interrupt(*unused):raise KeyboardInterrupt('Retain until supported')
    signal.signal(signal.SIGTERM,interrupt);signal.signal(signal.SIGINT,interrupt)
    execute(jobs,planner,args.profile,args.out,validations,vision)

if __name__=='__main__':main()
