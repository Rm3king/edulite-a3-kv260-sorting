"""Shared software-failure policy. No device access at import time."""
import json,time,queue,threading
from pathlib import Path


class WaitStateWriter:
    """Publish telemetry without putting filesystem latency on CAN servicing.

    The worker never accesses a device. Superseded observations are dropped;
    their timestamps remain in the published record so readers can detect age.
    """
    def __init__(self,out):
        self.out=Path(out);self.pending=queue.Queue(maxsize=1)
        self.stopped=threading.Event();self.error=None
        self.thread=threading.Thread(target=self._run,daemon=True)
        self.thread.start()

    def publish(self,observation):
        try:self.pending.put_nowait(observation)
        except queue.Full:
            try:self.pending.get_nowait()
            except queue.Empty:pass
            try:self.pending.put_nowait(observation)
            except queue.Full:pass

    def _run(self):
        tmp=self.out/('operator_wait_state.'+str(id(self))+'.tmp')
        while not self.stopped.is_set():
            try:observation=self.pending.get(timeout=.05)
            except queue.Empty:continue
            try:
                tmp.write_text(json.dumps(observation))
                if not self.stopped.is_set():tmp.replace(self.out/'operator_wait_state.json')
            except (OSError,ValueError,TypeError) as error:self.error=repr(error)
        try:tmp.unlink(missing_ok=True)
        except OSError:pass

    def close(self):
        # Never wait for a slow filesystem while this thread owns heartbeat.
        self.stopped.set()


def freeze_failure_evidence(e,out,summary,error=None,task=None):
    """Copy a bounded window of the live telemetry and write it from a worker thread.

    Called before a long operator wait: the hold loop later trims the live buffers, and an owner that is
    terminated while holding would otherwise leave no trace of what led to the fault. The copy is cheap and
    runs on the servicing thread; the slow file write never does. Idempotent, never touches the device.
    """
    if summary.get('failure_evidence_frozen'):return
    report=getattr(e,'report',None) or {}
    tail=lambda name,n:[dict(x) if isinstance(x,dict) else x for x in report.get(name,[])[-n:]]
    payload=dict(error=error,task=task,status=summary.get('status'),frozen_monotonic=time.monotonic(),
                 last_completed_task=report.get('last_completed_task'),active_task=report.get('active_task'),
                 feedback_tail=tail('feedback',512),tracking_diagnostics=tail('tracking_diagnostics',128),events=tail('events',200),
                 velocity_monitor_failures=list(report.get('velocity_monitor_failures',[])),jaw_observations=list(report.get('jaw_observations',[])),
                 hold_service_errors=list(summary.get('hold_service_errors',[])))
    summary['failure_evidence_frozen']=True;path=Path(out)/'failure_context.json'
    def write():
        tmp=path.with_suffix('.tmp')
        try:tmp.write_text(json.dumps(payload,default=str));tmp.replace(path);summary['failure_evidence_written']=True
        except (OSError,ValueError,TypeError) as exc:summary['failure_evidence_write_error']=repr(exc)
    threading.Thread(target=write,daemon=True).start()


def support_alarm(now,waiting_since,next_alarm,hottest_c):
    """(event name or None, next alarm time). Escalates with temperature; never decides to remove torque."""
    if now<next_alarm:return None,next_alarm
    critical=hottest_c is not None and hottest_c>=65
    return ('OPERATOR_SUPPORT_CRITICAL_TEMPERATURE' if critical else 'OPERATOR_SUPPORT_REQUIRED'),now+(15. if critical else 30.)


def emit(payload):
    """A closed console is not a request to remove motor torque."""
    try:print(json.dumps(payload),flush=True)
    except (OSError,TypeError,ValueError):pass


def retain_until_supported(e,out,summary,service=None,waiting_status='RECOVERY_FAILED_RETAINING_CONTROL'):
    """A failed software recovery must not fall through to blanket reset.

    Stay in the hardware-owning process and keep servicing feedback/heartbeat.
    The marker explicitly means the operator has supported the arm for reset;
    it is never written by the numerical worker or an exception handler.
    This cannot override a drive's internal protection or a broken CAN link.
    """
    marker=Path(out)/'SUPPORTED_DISABLE'
    summary['status']=waiting_status
    freeze_failure_evidence(e,out,summary)
    e.event('RETAINING_CONTROL_NO_AUTOMATIC_DISABLE',supported_disable_marker=str(marker))
    faults=set();last_observation=0.;last_state=None;waiting_since=time.monotonic();next_alarm=waiting_since+5.
    writer=getattr(e,'wait_state_writer',None)
    borrowed_writer=writer is not None
    if writer is None:writer=WaitStateWriter(out)
    while True:
        try:
            if marker.exists():break
            now=time.monotonic()
            if now-last_observation>=.25:
                last_observation=now
                try:
                    observation=dict(time=now,control=e.d.read(0x404),fault=e.d.read(0x418),active_mask=e.d.read(0x5a8))
                    if observation['fault']:
                        state='HARDWARE_OUTPUT_STOPPED_REQUIRES_SUPPORT' if observation['active_mask']==0 and observation['control']==0 else 'HARDWARE_FAULT_REQUIRES_SUPPORT'
                    else:state='AWAITING_OPERATOR_CONFIRMATION_HOLD_NOT_GUARANTEED'
                except Exception as error:
                    observation=dict(time=now,error=repr(error));state='HARDWARE_STATE_UNKNOWN_REQUIRES_SUPPORT'
                observation['status']=state
                observation['waiting_seconds']=now-waiting_since
                cache=getattr(e.s,'feedback',{})
                observation['motor_feedback']=[dict(motor=mid,age_s=now-f['time'],
                    fresh=0<=now-f['time']<=.08,temperature_c=f.get('temperature'),
                    mode=f.get('mode'),fault=f.get('fault'))
                    for mid,f in cache.items()] if isinstance(cache,dict) else []
                # These are controller registers, not fresh per-drive mode or
                # temperature acknowledgements. Never claim torque is retained.
                summary['hold_controller_observation']=observation
                hottest=max((f.get('temperature_c') for f in observation['motor_feedback'] if isinstance(f.get('temperature_c'),(int,float))),default=None)
                alarm,next_alarm=support_alarm(now,waiting_since,next_alarm,hottest)
                if alarm:
                    summary['operator_alarms']=summary.get('operator_alarms',0)+1
                    e.event(alarm,waiting_seconds=round(now-waiting_since,1),hottest_c=hottest,state=state,
                            note='arm is held without a guarantee; support it, then create the SUPPORTED_DISABLE marker')
                writer.publish(observation)
                if writer.error:summary['hold_state_write_error']=writer.error
                if observation.get('fault'):
                    summary.setdefault('hold_hardware_fault',dict(observation))
                if state!=last_state:
                    e.event(state,observation=observation);last_state=state
            try:(service or (lambda:e.s.pump(.003)))()
            except Exception as exc:
                message=repr(exc)
                if message not in faults and len(faults)<32:
                    faults.add(message)
                    summary.setdefault('hold_service_errors',[]).append(message)
                    e.event('HOLD_SERVICE_OBSERVATION',error=message)
                # Do not configure(), reset, clear faults, or re-enable here.
                try:e.d.heartbeat()
                except Exception as heartbeat_error:summary['hold_heartbeat_error']=repr(heartbeat_error)
                time.sleep(.001)
            # This fallback may run for a long time; bounded telemetry prevents
            # a later OOM kill from removing the owner of the heartbeat.
            for name in ('commands','frames','feedback','velocity_estimator_excursions'):
                evidence=e.report.get(name,[])
                if len(evidence)>4096:del evidence[:64]
        except KeyboardInterrupt:
            # A terminal interruption is not evidence that the arm is supported.
            continue
    if not borrowed_writer:writer.close()
    summary['supported_disable_requested']=True


def await_disable_approval(e,out,summary):
    """Returning to the support pose is not permission to remove torque."""
    if summary.get('supported_disable_requested'):return
    terminal_status=summary['status']
    retain_until_supported(e,out,summary,waiting_status='SUPPORTED_AWAITING_DISABLE_APPROVAL')
    summary['status']=terminal_status
    if summary.get('hold_hardware_fault'):
        summary['motion_status_before_wait_fault']=terminal_status
        summary['status']='OPERATOR_WAIT_HARDWARE_FAULT'


def finish_failed_job(e,job,out,report,service=None):
    """Only finish a prevalidated return-to-start/zero job; otherwise hold."""
    try:
        tasks=(job or {}).get('tasks',[])
        if not any(t.get('return_to_live_start') or t.get('final_zero') for t in tasks):
            raise RuntimeError('no declared cached supported return')
        prepared=getattr(e.transport,'prepared',None)
        if prepared is None or any(t.get('pause_after_s') is not None for t in tasks):
            raise RuntimeError('no unambiguous prevalidated return records')
        if e.d.read(0x418) or e.transport.status()['fault'] or e.d.read(0x5a8)!=127:
            raise RuntimeError('controller cannot finish supported return')
        e.s.pump(.03)
        for mid in range(1,8):e.s.require(mid,2)
        rest=prepared[e.transport.prepared_index:];e.state='HOLDING'
        if rest:e.execute_segments([r.get('original_plan',r['plan']) for r in rest])
        else:
            deadline=time.monotonic()+15
            while not e.pump(.002)['holding']:
                if time.monotonic()>deadline:raise TimeoutError('cached return completion')
        e.s.pump(.3);report['completed_cached_return_before_disable']=True
    except (Exception,KeyboardInterrupt) as exc:
        report['cached_return_error']=repr(exc)
        retain_until_supported(e,out,report,service)
