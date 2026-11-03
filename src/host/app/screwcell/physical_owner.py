"""Explicit physical backend for the registered empty-pick skill only.

One persistent process owns CAN. Network loss selects a precomputed C3 brake;
it never calls reset. Reset is allowed only after the cached supported return
and endpoint check. Heavy work and evidence compression happen disabled.
"""
import contextlib
import ctypes
import fcntl
import gzip
import gc
import io
import json
import os
import queue
import sys
import time
from collections import deque
from pathlib import Path

SKILL = 'empty_pick_return_v1'


class BoundedEvidence(list):
    """An offline observer must not grow the CAN owner's memory indefinitely."""
    def __init__(self,limit,trim):
        super().__init__();self.limit=limit;self.trim=min(trim,limit);self.dropped=0
    def append(self,value):
        if len(self)>=self.limit:
            del self[:self.trim];self.dropped+=self.trim
        super().append(value)


def worker(commands, results, runtime_root, profile, tracking, evidence_root):
    sys.path.insert(0, runtime_root)
    sys.path.append(str(Path(runtime_root).parents[1]/'kv260_can_deploy'))
    from canctl import CanDevice
    from read_arm import main as read_arm_main
    from control_benchmark import build, initialize_disabled_csp, native_library, following_guard, timing_read
    from joint_trajectory import JointPlanner
    from gravity_trajectory import GravityModel
    from motion_runtime import MotionRuntime
    from autotune import TuningExecutor
    from gravity_transport import GravityTransport
    from motion_executor import MoveJExecutor
    from .motion_paths import prepare_branches,restart_entries
    from .offline_fields import FieldValidator
    from .clocks import PLClock
    from .wire import digest
    from .archives import DeferredArchives
    from .preparation import NumericalPreparation
    p = JointPlanner(profile)
    model = GravityModel(MotionRuntime(), p, tracking)
    clock = PLClock()
    state, task_id, executions, lease = 'IDLE', None, 0, 0.
    e = None
    bank = branches = None
    pending, restart_pending = deque(), deque()
    committed = []
    checkpoint = None
    pause = False
    returning = False
    last_original = -1
    report = {}
    lock = None
    lib = native_library()
    arrays = {}
    archives=DeferredArchives()
    gc_previous = True
    root = Path(evidence_root)
    root.mkdir(parents=True, exist_ok=True)

    class Discard(list):
        def append(self, value):
            pass

    class Transport(GravityTransport):
        def submit_record(self, record):
            rc = lib.native_upload(self.dev.base, arrays[id(record)])
            if rc:
                raise RuntimeError('native segment upload '+str(rc))
            self.prepared.append(record)
            self.prepared_index += 1
            self.tail = record['ends']
            self.tail_velocity = record['end_velocity']
            self.last_encoded_plan = record['plan']

    def send(value):
        try:
            results.put_nowait(value)
        except queue.Full:
            # A failed observer cannot interrupt torque. The durable motion
            # report remains with this owner; request timeouts are explicit.
            pass

    def event(name, **extra):
        value = dict(event=name, task_id=task_id, backend='kv260-motion',
                     simulated=False, time_ns=time.monotonic_ns(), **extra)
        report.setdefault('network_events', []).append(value)
        send(value)

    def snapshot():
        d = e.d if e is not None else CanDevice()
        try:
            before = time.monotonic_ns()
            value = d.status()
            after = time.monotonic_ns()
            return dict(status=value, abi=d.read(0x470), motion_mask=d.read(0x5a0),
                        active_motors=d.read(0x5a8), mit_flags=d.read(0xb40),
                        timing_signature=d.read(0x100)), clock.sample(value['time_us'], before, after)
        finally:
            if e is None:
                d.close()

    def status():
        raw, mapping = snapshot()
        feedback={}
        if e:
            for mid,f in e.s.feedback.items():
                feedback[str(mid)]=dict(motor_id=mid,position_motor_rad=f['position'],
                    position_joint_rad=(f['position']-p.offset[mid-1])/p.direction[mid-1],
                    velocity_motor_rad_s=f['velocity'],velocity_joint_rad_s=f['velocity']/p.direction[mid-1],
                    torque_feedback_nm=f['torque'],temperature_c=f['temperature'],drive_state=f['mode'],
                    drive_fault=f['fault'],kv_received_monotonic_s=f['time'],pl_rx_us=f.get('pl_rx_us'))
        return dict(state=state, task_id=task_id, executions=executions,
                    physical_execution_supported=True, motor_commands_sent=len(report.get('commands', [])),
                    pl_read_only=raw, pl_clock_mapping=mapping,
                    feedback=feedback,feedback_schema='joint_feedback.v1',
                    evidence_dropped={k:v.dropped for k,v in report.items() if isinstance(v,BoundedEvidence) and v.dropped},
                    submitted_segments=len(committed), completed_segments=e.transport.status()['completed'] if e else 0,
                    pause_reason=report.get('pause_reason'), return_policy='FINISH_PREVALIDATED_ROUTE_TO_CAPTURED_START',
                    preparation_progress=report.get('preparation_progress'),
                    last_result=report.get('result_summary'))

    def capture():
        nonlocal lock
        if lock is None:
            lock = open('/tmp/kv260_gripper_commission.lock', 'a')
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        raw, _ = snapshot()
        if any(raw[k] for k in ('motion_mask', 'active_motors', 'mit_flags')) or raw['status']['control']:
            raise ValueError('ANOTHER_HARDWARE_OWNER_ACTIVE')
        stream = io.StringIO()
        with contextlib.redirect_stdout(stream), contextlib.redirect_stderr(stream):
            rc = read_arm_main()  # argv temporarily bounded below
        value = json.loads(stream.getvalue())
        if rc or value['status'] != 'PASS_READ_ONLY_ALL_7':
            raise ValueError('SEVEN_MOTOR_FEEDBACK_REQUIRED')
        return value

    def prepare(payload):
        nonlocal bank, branches, arrays, report, task_id, state, lease
        if state not in ('IDLE', 'MOTION_DONE') or e is not None:
            raise ValueError('OWNER_BUSY')
        if payload['skill_id'] != SKILL:
            raise ValueError('UNKNOWN_PHYSICAL_SKILL')
        task_id = payload['task_id']
        report = dict(task_id=task_id, status='PREPARING_DISABLED', commands=BoundedEvidence(12000,512), frames=Discard(),
                      feedback=BoundedEvidence(120000,2048), readbacks={}, reset_confirmed=[], stop_errors={}, flash_writes=False,
                      events=BoundedEvidence(2048,64),network_events=BoundedEvidence(2048,64),pl_samples=BoundedEvidence(8000,256),
                      velocity_estimator_excursions=BoundedEvidence(8000,256),observation_gaps=BoundedEvidence(1024,64),
                      velocity_observation_gaps=BoundedEvidence(1024,64),segment_submissions=BoundedEvidence(4096,128))
        begin = time.monotonic()
        prior = sys.argv
        try:
            sys.argv = ['read_arm.py', '--rounds', '1']
            measured = capture()
        finally:
            sys.argv = prior
        capture_s=time.monotonic()-begin
        preparation_workers=min(4,os.cpu_count() or 1)
        def service_waiting():
            nonlocal lease
            for _ in range(8):
                try:waiting_token,op,_payload=commands.get_nowait()
                except queue.Empty:break
                try:
                    if op=='STATUS':send(dict(token=waiting_token,value=status()))
                    elif op=='HEARTBEAT':lease=time.monotonic()+.5;send(dict(token=waiting_token,value={}))
                    else:send(dict(token=waiting_token,error='PREPARATION_IN_PROGRESS'))
                except Exception as exc:send(dict(token=waiting_token,error=str(exc)))
        state='PREPARING'
        try:
            job=NumericalPreparation(runtime=runtime_root,profile=profile,tracking=tracking,capture=measured,
                cache_path=str(root/'encoding_cache.json.gz'),workers=preparation_workers)
            while True:
                result=job.poll(.01)
                if result and 'progress' in result:
                    report['preparation_progress']=result['progress'];event('PREPARATION_PROGRESS',**result['progress'])
                if result and 'error' in result:
                    (root/(task_id+'.prepare.failure.json')).write_text(json.dumps(result,indent=2)+'\n')
                    raise ValueError(result['error'])
                if result and 'result' in result:
                    compiled=result['result'];break
                # The compiler has no devices. The sole owner remains free to
                # answer telemetry/status and refresh the application lease.
                service_waiting()
            bank,branches,edges=compiled['bank'],compiled['branches'],compiled['edges']
        finally:state='IDLE'
        numeric=compiled['metrics'];main_s=numeric['main_s'];branches_s=numeric['branches_s'];stage=time.monotonic()
        report['preparation_progress']=dict(stage='PL_FK_AUDIT');event('PREPARATION_PROGRESS',stage='PL_FK_AUDIT')
        # Actual PL audit of the original dense trajectory, while disabled.
        # This is sampled model TCP containment, never whole-arm clearance.
        fk_root=Path(runtime_root).parents[1]/'kv260_fk_batch/source'
        sys.path.insert(0,str(fk_root))
        from client import BatchFK
        import numpy as np
        fk=BatchFK(fk_root/'libfk_batch.so',True)
        try:
            audits=[]
            for r in bank['records']:
                audits.append(fk.audit(np.asarray(r['original_plan']['coefficients_rad'])[:6],
                    [-1,1,-1,1,-1,1],min(4097,r['original_plan']['ticks']*8+1),'pl'))
                service_waiting()
        finally:fk.close()
        if any(x['status'] or x['violations'] for x in audits):raise ValueError('SAMPLED_PL_FK_REPORT_FAILED')
        all_records = list(bank['records'])
        for branch in branches.values():
            for name in ('brake', 'resume', 'suffix'):
                all_records.extend(branch[name])
        encoded_arrays={}
        for r in all_records:
            words=tuple(r['words'])
            if words not in encoded_arrays:encoded_arrays[words]=(ctypes.c_int32*220)(*words)
            if len(encoded_arrays)%64==0:service_waiting()
        arrays = {id(r):encoded_arrays[tuple(r['words'])] for r in all_records}
        checkpoint_times = [edges[i+1] for i in branches]
        summary = dict(skill_id=SKILL, model_id=model.r.model_id, profile_id=p.profile_id,
                       tracking_profile_id=model.id, reference_sha256=bank['reference_sha256'],
                       duration_s=bank['duration_s'], segments=len(bank['records']),
                       checkpoint_count=len(branches), max_checkpoint_gap_s=max(b-a for a,b in zip([0.]+checkpoint_times,checkpoint_times)),
                       maximum_brake_s=max(x['ramp_s'] for x in branches.values()),
                       time_warp_sample_error_rad=max(x['fit_error_rad'] for x in branches.values()),
                       start_policy='CURRENT_FEEDBACK_REPLAN', physical_execution_supported=True,
                       scope='REGISTERED_EMPTY_AIR_PICK_NOT_CALIBRATED_VISUAL_GRASP',
                       grasp_pose=bank['audit']['grasp_pose'],
                       start_joint_rad=bank['job']['start_rad'], preparation_s=time.monotonic()-begin,
                       preparation_workers=preparation_workers,
                       numerical_preparation=numeric,unique_native_arrays=len(encoded_arrays),
                       preparation_phases_s=dict(capture=capture_s,main=main_s,branches=branches_s,pl_fk=time.monotonic()-stage),
                       collision_checked=False, scene_calibration='UNSET')
        summary['pl_fk_audit_samples']=sum(x['samples'] for x in audits)
        summary['exhaustively_validated_branch_phases']=numeric['checked_phases']
        report['preparation'] = summary
        report['capture'] = measured
        # Includes actual encoded brake/resume bytes, not just request fields.
        summary['prepared_digest'] = compiled['prepared_digest']
        (root/(task_id+'.prepare.json')).write_text(json.dumps(summary,indent=2)+'\n')
        report['preparation_progress']=dict(stage='READY',preparation_s=summary['preparation_s'])
        event('PREPARATION_PROGRESS',**report['preparation_progress'])
        return summary

    def start(payload):
        nonlocal state, e, executions, lease, pending, restart_pending, committed
        nonlocal pause, returning, checkpoint, last_original
        nonlocal gc_previous
        if state not in ('IDLE','MOTION_DONE') or e is not None or not bank or payload['task_id'] != task_id:
            raise ValueError('OWNER_BUSY_OR_UNPREPARED_TASK')
        if payload['prepared_digest'] != report['preparation']['prepared_digest']:
            raise ValueError('PREPARED_DIGEST_MISMATCH')
        # Refresh the disabled origin. A changed pose is replanned, never
        # rejected against an old stored start, and all work stays disabled.
        prior = sys.argv
        try:
            sys.argv = ['read_arm.py','--rounds','1']
            fresh = capture()
        finally:
            sys.argv = prior
        old = [x['decoded']['position_motor_rad'] for x in report['capture']['transactions']]
        new = [x['decoded']['position_motor_rad'] for x in fresh['transactions']]
        if max(abs(a-b) for a,b in zip(old,new)) > .001:
            prepare(dict(task_id=task_id,skill_id=SKILL))
            event('CURRENT_ORIGIN_REPLANNED',prepared_digest=report['preparation']['prepared_digest'])
        report['status']='STARTING'
        # Collect only while disabled; runtime observations are bounded above.
        gc_previous=gc.isenabled();gc.collect();gc.disable()
        report['disabled_boot_mode_initialization'] = initialize_disabled_csp(p,bank['job'])
        d = CanDevice()
        e = TuningExecutor(p,d,report,bank['records'],model)
        e.prepare(bank['job'],current_tracking=True)
        e.transport = Transport(d,p,[])
        e.release_host_hold()
        d.write(0x104,3)
        pending = deque((r,i) for i,r in enumerate(bank['records']))
        restart_pending = deque()
        committed = []
        pause = returning = False
        checkpoint = None
        last_original = -1
        executions += 1
        state = 'RUNNING'
        lease = time.monotonic()+.5
        report['started_ns']=time.monotonic_ns()
        event('STARTED', reference_sha256=bank['reference_sha256'],prepared_digest=report['preparation']['prepared_digest'])
        return dict(reference_sha256=bank['reference_sha256'],prepared_digest=report['preparation']['prepared_digest'],
                    start_joint_rad=bank['job']['start_rad'],start_policy='CURRENT_FEEDBACK_REPLAN')

    def request_pause(reason):
        nonlocal pause, state
        if state not in ('RUNNING','BRAKING','HOLDING'):
            raise ValueError('TASK_NOT_RUNNING')
        if state == 'RUNNING':
            pause = True
            state = 'BRAKING'
            report['pause_requested_ns']=time.monotonic_ns()
            report['pause_reason']=reason
            event('BRAKING',reason=reason)

    def resume(return_request=False):
        nonlocal state, pending, pause, checkpoint, returning, lease, last_original
        if state != 'HOLDING':
            raise ValueError('NOT_HOLDING')
        pending = deque(restart_pending)
        pause = False
        checkpoint = None
        # A new pause during restart must wait for the next ORIGINAL path
        # checkpoint. Reusing the old pre-brake tail would jump backwards.
        last_original = -1
        returning = return_request
        state = 'RETURNING' if return_request else 'RUNNING'
        lease = time.monotonic()+.5
        event(state if return_request else 'RESUMED')

    def finish():
        nonlocal state, e, lock
        e.s.pump(.15)
        goal=p.to_motor(bank['job']['tasks'][-1]['goal_rad'])
        errors=[abs(e.s.require(i+1,2)['position']-goal[i]) for i in range(7)]
        if max(errors)>model.config['endpoint_tolerance_rad']:
            raise RuntimeError('SUPPORTED_ENDPOINT_NOT_REACHED_RETAINING_TORQUE')
        report['supported_return_errors_rad']=errors
        report['timing']=timing_read(e.d)
        event('SUPPORTED_RETURN_REACHED',maximum_error_rad=max(errors))
        e.s.stop_all()
        if sorted(report['reset_confirmed'])!=list(range(1,8)) or report['stop_errors']:
            raise RuntimeError('SUPPORTED_RESET_NOT_CONFIRMED')
        e.restore_csp_configuration()
        final, _ = snapshot()
        report['final_hardware']=final
        report['status']='PASS_RETURNED_SUPPORTED_AND_DISABLED'
        report['finished_ns']=time.monotonic_ns()
        report['evidence_dropped']={k:v.dropped for k,v in report.items() if isinstance(v,BoundedEvidence)}
        report['result_summary']=dict(status=report['status'],reset_confirmed=report['reset_confirmed'],
            maximum_return_error_rad=max(errors),motion_wall_s=(report['finished_ns']-report['started_ns'])/1e9,
            evidence_file=task_id+'.motion.json.gz',motion_done_is_grasp_success=False)
        e.d.close()
        e = None
        if gc_previous:gc.enable()
        if lock is not None:
            lock.close();lock=None
        state='MOTION_DONE'
        report['result_summary'].update(evidence_pending=True)
        event('MOTION_DONE',**report['result_summary'])
        try:archives.submit(task_id,report,root/(task_id+'.motion.json.gz'))
        except Exception as exc:
            report['result_summary'].update(evidence_pending=False,evidence_saved=False,evidence_error=repr(exc))
            event('ARCHIVE_COMPLETE',**report['result_summary'])

    try:
        while True:
            for archived_task,value in archives.poll():
                if archived_task==task_id:report.get('result_summary',{}).update(evidence_pending=False,**value)
                send(dict(event='ARCHIVE_COMPLETE',task_id=archived_task,time_ns=time.monotonic_ns(),backend='kv260-motion',simulated=False,**value))
            for _ in range(4):
                try:
                    token, op, payload = commands.get_nowait()
                except queue.Empty:
                    break
                try:
                    value = {}
                    if op == 'SHUTDOWN':
                        if e is not None:
                            raise ValueError('ACTIVE_OWNER_CANNOT_SHUTDOWN')
                        if lock is not None:lock.close();lock=None
                        archives.close()
                        send(dict(token=token,value={'stopped':True}));return
                    if op == 'PREPARE_SKILL':value=prepare(payload)
                    elif op == 'EXECUTE':value=start(payload)
                    elif op == 'HEARTBEAT':lease=time.monotonic()+.5
                    elif op == 'STATUS':value=status()
                    elif op == 'PAUSE':request_pause('REQUESTED')
                    elif op == 'RESUME':resume()
                    elif op == 'RETURN_SUPPORTED':resume(True)
                    else:
                        if op!='PREPARE_SKILL':raise ValueError('UNKNOWN_PHYSICAL_OPERATION')
                    send(dict(token=token,value=value))
                except Exception as exc:
                    send(dict(token=token,error=str(exc)))
                    if e is None and gc_previous:gc.enable()
                    if e is not None and report.get('enable_attempted'):
                        event('OWNER_OPERATION_ERROR_RETAINING_TORQUE',error=repr(exc))
            if e is None:
                time.sleep(.002)
                continue
            try:
                s=MoveJExecutor.pump(e,.0015)
                if state=='RUNNING' and time.monotonic()>lease:
                    request_pause('LINK_HEARTBEAT_EXPIRED')
                if pause and checkpoint is None and last_original in branches:
                    checkpoint=branches[last_original]
                    pending=deque((r,None) for r in checkpoint['brake'])
                    restart_pending=deque(restart_entries(checkpoint))
                    event('BRAKE_COMMITTED',source_end_s=checkpoint['end_s'],stop_s=checkpoint['stop_s'],brake_s=checkpoint['ramp_s'])
                if not s['flags']&2 and pending:
                    record, original = pending.popleft()
                    e.transport.submit_record(record)
                    committed.append((record,original));e.submitted+=1
                    report['segment_submissions'].append(dict(ordinal=e.submitted,time=time.monotonic(),
                        original_index=original,task=record['task'],plan=record['plan'],words=record['words']))
                    if original is not None:last_original=original
                    if not e.started:
                        e.transport.start();e.started=True
                now=time.monotonic()
                if e.started and s['completed']<len(committed) and not report.get('runtime_error'):
                    following_guard(e,committed[s['completed']][0],s['tick'])
                if s['holding'] and s['completed']>=len(committed) and not pending:
                    if pause and checkpoint is not None:
                        if state!='HOLDING':
                            state='HOLDING'
                            event('HOLDING',reason=report.get('pause_reason'),
                                  request_to_hold_s=(time.monotonic_ns()-report['pause_requested_ns'])/1e9,
                                  active_motors=e.d.read(0x5a8))
                    elif state in ('RUNNING','RETURNING'):
                        finish()
            except Exception as exc:
                # Never fall through to reset for a Python/telemetry exception.
                # Keep the original owner and PL heartbeat alive for diagnosis.
                if not report.get('runtime_error'):
                    report['runtime_error']=repr(exc)
                    event('OWNER_RUNTIME_ERROR_RETAINING_TORQUE',error=repr(exc))
                    try:
                        hardware_fault=e.d.read(0x418) or e.transport.status()['fault'] or e.d.read(0x5a8)!=127
                    except Exception:hardware_fault=True
                    if hardware_fault:
                        state='HOLDING_FAULT';pending.clear()
                        event('HOLDING_FAULT',error=repr(exc))
                    else:
                        if state=='RETURNING':state='RUNNING'
                        request_pause('SOFTWARE_DIAGNOSTIC_RETAINING_TORQUE')
                try:e.s.pump(.0015)
                except Exception:
                    with contextlib.suppress(Exception):e.d.heartbeat()
                    time.sleep(.001)
    finally:
        # Only disabled exits reach here during normal service shutdown.
        if e is None and lock is not None:lock.close()
