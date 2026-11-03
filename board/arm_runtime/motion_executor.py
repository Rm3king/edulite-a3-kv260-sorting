#!/usr/bin/python3
"""Attended reusable v5 execution backend. Imports never open hardware.

Segments are prepared and hashed before enable. Pause has a bounded lookahead:
finish <=2 already committed <=250 ms chunks, then a prevalidated braking cubic.
It is not an emergency stop. Holding retains torque; faults use existing reset.
"""
import argparse,fcntl,gc,hashlib,json,math,signal,sys,time
from collections import deque
from pathlib import Path
from joint_trajectory import JointPlanner,evaluate
from continuous_trajectory import split,brake_at,cubic_words
from continuous_transport import ContinuousTransport

DRIVER=Path(__file__).resolve().parents[1]/'kv260_can/software'
if not DRIVER.is_dir():DRIVER=Path.home()/'kv260_can_deploy'
if str(DRIVER) not in sys.path:sys.path.append(str(DRIVER))
from canctl import CanDevice
from arm_commission import Session,parameter
from a3_protocol import enable,disable
from csp_commission import bits,SettlingWindow
from control_continuity import emit,finish_failed_job


def job_digest(job):
    return hashlib.sha256(json.dumps(job,sort_keys=True,separators=(',',':'),allow_nan=False).encode()).hexdigest()


def job_and_path_digests(job):
    """(job_digest, path_digest) with identical values, encoding the large tasks/recoveries only once.

    On the board each of the two encodings costs ~0.3 s while the arm waits for validation.
    path_digest hashes the encoding of tasks (+ recoveries when present); the same text appears
    verbatim inside the sorted-key encoding of the whole job.
    """
    def encode(value):return json.dumps(value,sort_keys=True,separators=(',',':'),allow_nan=False)
    tasks=encode(job['tasks']);recoveries=encode(job['recoveries']) if job.get('recoveries') else None
    pieces=[]
    for key in sorted(job):
        text=tasks if key=='tasks' else (recoveries if key=='recoveries' and recoveries is not None else encode(job[key]))
        pieces.append(json.dumps(key)+':'+text)
    path_text=tasks if recoveries is None else '{"recoveries":'+recoveries+',"tasks":'+tasks+'}'
    digest=lambda text:hashlib.sha256(text.encode()).hexdigest()
    return digest('{'+','.join(pieces)+'}'),digest(path_text)


class PathVelocityMonitor:
    """Per-axis speed diagnostics scaled to explicit reviewed servo ceilings."""
    def __init__(self,p,report=None):
        self.p=p;self.history={};self.high_since={};self.stamps={};self.clocks={}
        self.report=report if report is not None else {}
        # Contact-loaded jaws may correct their position after lifting a part.
        # This is a feedback diagnostic margin, not a motor command speed.
        self.gripper_factor=getattr(p,'profile',{}).get('gripper_encoder_window_factor',1.05)
        if type(self.gripper_factor) not in (int,float) or not math.isfinite(self.gripper_factor) or not 1.05<=self.gripper_factor<=1.5:
            raise ValueError('gripper encoder window factor must be 1.05..1.5')
    def fail(self,reason,f,**details):
        records=self.report.setdefault('velocity_monitor_failures',[])
        if len(records)<32:
            records.append(dict(reason=reason,feedback=dict(f),csp_limit_rad_s=self.p.csp_velocity[f['motor']-1],
                                window=list(self.history.get(f['motor'],[])),**details))
        raise RuntimeError(f"Axis {f['motor']}: {reason}")
    def observe(self,f):
        mid=f['motor'];i=mid-1;now=f['time'];v=abs(f['velocity']);limit=self.p.csp_velocity[i]
        if 'pl_rx_us' in f:
            stamp=f['pl_rx_us'];last=self.stamps.get(mid);self.stamps[mid]=stamp
            dt=0. if last is None else ((stamp-last+(1<<31))%(1<<32)-(1<<31))*1e-6
            if last is not None and not 0<dt<=.08:
                self.report.setdefault('velocity_observation_gaps',[]).append(dict(motor=mid,dt=dt,time=f['time']))
                self.history.pop(mid,None);self.high_since.pop(mid,None);dt=.005
            self.clocks[mid]=self.clocks.get(mid,0.)+dt;now=self.clocks[mid]
        h=self.history.setdefault(mid,deque())
        if h and not 0<now-h[-1][0]<=.08:
            h.clear();self.high_since.pop(mid,None)
        if v>max(1.2,3*limit):self.fail('gross velocity',f)
        excursion=v>max(.4,1.8*limit)
        if excursion:
            self.high_since.setdefault(mid,now)
            if now-self.high_since[mid]>=.1:self.fail('sustained velocity',f)
        else:self.high_since.pop(mid,None)
        h.append((now,f['position']))
        while len(h)>1 and now-h[1][0]>=.08:h.popleft()
        dt=now-h[0][0]
        factor=self.gripper_factor if mid==7 else 1.05
        allowed=max(.12,factor*limit)*dt+2*25.14/65535
        displacement=abs(f['position']-h[0][1])
        if dt>=.08 and displacement>allowed:
            self.fail('encoder window velocity',f,window_s=dt,displacement_rad=displacement,
                      allowed_displacement_rad=allowed,margin_factor=factor)
        return excursion


class MoveJExecutor:
    def __init__(self,p,dev,report,transport_class=ContinuousTransport,prepared_segments=None):
        self.p,self.d,self.report=p,dev,report;self.s=Session(dev,report)
        self.s.velocity_monitor=PathVelocityMonitor(p,report)
        self.transport=transport_class(dev,p,prepared_segments) if prepared_segments is not None else transport_class(dev,p)
        self.state='DISABLED';self.started=False;self.submitted=0;self.last_snapshot=0.;self.expected=None
        self.following_since={};self.last_diagnostic=0.

    def event(self,event_code,**fields):
        self.report.setdefault('events',[]).append(dict(event=event_code,time=time.monotonic(),**fields))
        emit(dict(event=event_code,**fields))

    def prepare(self,job):
        if job.get('start_policy')=='CURRENT_FEEDBACK_REPLAN':
            return self.prepare_current(job)
        p,d,s=self.p,self.d,self.s
        if d.read(0x470) not in (0x50000,0x50001,0x60000,0x60001,0x60002,0x60003) or d.read(0x404)!=0 or d.read(0x5a8) or d.read(0x800)&1:raise RuntimeError('v5 stopped controller required')
        d.configure('normal');self.report['owned']=True
        for mid in range(1,8):
            s.send(disable(mid));s.pump(.025);s.require(mid,0)
        actual=[s.read(mid,0x7019) for mid in range(1,8)]
        if max(abs(a-b) for a,b in zip(actual,job['start_motor_rad']))>.002:raise RuntimeError('posture changed since job review')
        self.report['start_motor_rad']=actual
        d.stop();d.write(0x5a0,127);d.write(0x5a4,100000)
        for mid,(lo,hi) in enumerate(zip(job['motor_lower_rad'],job['motor_upper_rad']),1):
            s.plans[mid]=dict(lower=lo,upper=hi,speed=p.csp_velocity[mid-1],current=p.current[mid-1])
            base=d.pp_base(mid)
            for off,val in [(0,1),(4,bits(lo)),(8,bits(hi)),(12,bits(p.csp_velocity[mid-1])),(16,bits(p.acceleration[mid-1])),(20,bits(p.current[mid-1])),(28,1)]:
                d.write(base+off,val)
                if d.read(base+off)!=val:raise RuntimeError('guard config readback mismatch')
        d.configure('normal')
        for mid in range(1,8):
            fresh=s.read(mid,0x7019)
            if abs(fresh-actual[mid-1])>.002:raise RuntimeError('disabled posture moved before enable')
            for idx,val,integer in [(0x7005,5,True),(0x7017,p.csp_velocity[mid-1],False),(0x7018,p.current[mid-1],False),(0x7028,6000,True),(0x7016,fresh,False)]:
                s.write_verified(mid,idx,val,integer)
            self.report.setdefault('initial_hold_targets',{})[str(mid)]=fresh
            self.report['enable_attempted']=True;s.targets[mid]=fresh
            stamp=time.monotonic();s.send(enable(mid));s.pump(.02);f=s.require(mid,2)
            if f['time']<stamp or abs(f['position']-actual[mid-1])>.01:raise RuntimeError('unexpected CSP enable displacement')
            s.active.add(mid);s.pump(.1)
            if abs(s.read(mid,0x7019)-actual[mid-1])>.005:raise RuntimeError('initial CSP hold drift')
        s.pump(.2);s.target_owner='pl'
        deadline=time.monotonic()+.05
        while d.read(0x41c)&65535 or d.read(0x464):
            if time.monotonic()>deadline:raise RuntimeError('host queue did not drain')
            s.pump(.001)
        self.report['stream_initial_counters']=dict(rx_overflow=d.read(0x42c),expired=d.read(0x424))
        self.state='HOLDING';self.event('ENABLED_HOLDING')

    def prepare_current(self,job):
        """Configure all disabled axes, capture fresh targets, enable as a group.

        No comparison with a historical pose. The caller replans entry and
        return from the held feedback before releasing any trajectory to PL.
        """
        from live_motion import current_reference,MEASUREMENT_MARGIN_RAD
        p,d,s=self.p,self.d,self.s
        if d.read(0x470) not in (0x50001,0x60000,0x60001,0x60002,0x60003) or d.read(0x404) or d.read(0x5a8) or d.read(0x800)&1:
            raise RuntimeError('stopped C3 controller required')
        d.configure('normal');self.report['owned']=True
        for mid in range(1,8):
            s.send(disable(mid));s.pump(.015);s.require(mid,0)
        d.stop();d.write(0x5a0,127);d.write(0x5a4,100000)
        for mid,(lo,hi) in enumerate(zip(job['motor_lower_rad'],job['motor_upper_rad']),1):
            i=mid-1;base=d.pp_base(mid)
            s.plans[mid]=dict(lower=lo-MEASUREMENT_MARGIN_RAD,upper=hi+MEASUREMENT_MARGIN_RAD)
            for off,val in [(0,1),(4,bits(lo)),(8,bits(hi)),(12,bits(p.csp_velocity[i])),(16,bits(p.acceleration[i])),(20,bits(p.current[i])),(28,1)]:
                d.write(base+off,val)
                if d.read(base+off)!=val:raise RuntimeError('current-entry guard readback mismatch')
        d.configure('normal')
        for mid in range(1,8):
            i=mid-1
            for idx,val,integer in [(0x7005,5,True),(0x7017,p.csp_velocity[i],False),(0x7018,p.current[i],False),(0x7028,6000,True)]:
                s.write_verified(mid,idx,val,integer)
        actual=[s.read(mid,0x7019) for mid in range(1,8)]
        reference,adjustment=current_reference(p,actual)
        self.report['enable_capture_motor_rad']=actual
        self.report['enable_reference_adjustment_motor_rad']=adjustment
        for mid,value in enumerate(p.to_motor(reference),1):
            # Type17 position reads do not renew the Type2 guard table. Refresh
            # this disabled axis immediately before its guarded LOC_REF write.
            s.send(parameter(mid,0x7028,6000,True));s.pump(.003);s.require(mid,0)
            # Disabled firmware can mirror measured position in loc_ref,
            # including a small calibrated endpoint offset. The legal packet
            # is checked by PL; a disabled loc_ref read is not an echo register.
            s.send(parameter(mid,0x7016,value));s.pump(.003)
            s.targets[mid]=value
        for mid in range(1,8):
            s.send(parameter(mid,0x7028,6000,True));s.pump(.003);s.require(mid,0)
            s.send(parameter(mid,0x7016,s.targets[mid]))
            if d.read(d.pp_base(mid)+24)!=63:raise RuntimeError('current-entry CSP preparation incomplete')
        self.report['enable_attempted']=True;stamps={}
        for mid in range(1,8):
            stamps[mid]=time.monotonic();s.send(enable(mid));s.send(parameter(mid,0x7016,s.targets[mid]))
        s.pump(.02)
        for mid in range(1,8):
            if s.require(mid,2)['time']<stamps[mid]:raise RuntimeError('missing fresh grouped enable acknowledgement')
        s.active.update(range(1,8));s.pump(.1)
        if d.read(0x5a8)!=127:raise RuntimeError('seven enabled axes required')
        self.state='HOLDING';self.event('CURRENT_POSE_ENABLED_HOLDING')

    def release_host_hold(self):
        self.s.pump(.01);self.s.target_owner='pl'
        deadline=time.monotonic()+.05
        while self.d.read(0x41c)&65535 or self.d.read(0x464):
            if time.monotonic()>deadline:raise RuntimeError('current-entry hold queue did not drain')
            self.s.pump(.001)
        self.report['stream_initial_counters']=dict(rx_overflow=self.d.read(0x42c),expired=self.d.read(0x424))

    def pump(self,seconds=.003):
        self.s.pump(seconds);d=self.d;status=self.transport.status();now=time.monotonic()
        if status['fault'] or d.read(0x418):raise RuntimeError('PL trajectory/controller fault')
        initial=self.report['stream_initial_counters']
        if d.read(0x424)!=initial['expired']:raise RuntimeError('expired target')
        overflow=d.read(0x42c)
        if overflow!=initial['rx_overflow']:
            self.report.setdefault('observation_gaps',[]).append(dict(time=now,rx_lost=overflow-initial['rx_overflow']))
            initial['rx_overflow']=overflow
        if now-self.last_snapshot>.025:
            self.last_snapshot=now
            phase=getattr(self.transport,'phase_register_base',0x800)
            completed=d.read(phase+12);tick=d.read(phase+16);due=d.read(phase+24);holding=d.read(phase+28)
            if d.read(phase+12)==completed:
                self.report.setdefault('pl_samples',[]).append(dict(time=now,**dict(status,completed=completed,tick=tick,next_due_us=due,holding=bool(holding))))
        if self.started:
            if status['completed']>self.submitted:raise RuntimeError('PL completed count exceeds submitted')
            if status['holding'] and status['completed']==self.submitted and self.transport.tail_velocity is not None and max(map(abs,self.transport.tail_velocity))>2e-4:
                raise RuntimeError('nonzero-velocity queue underrun')
        return status

    def diagnostics(self):
        now=time.monotonic()
        if now-self.last_diagnostic<.15:return
        self.last_diagnostic=now
        for mid in (2,3):
            sample=dict(motor=mid,time=time.monotonic(),loc_ref=self.s.read(mid,0x7016),iqf=self.s.read(mid,0x701a))
            feedback=self.s.require(mid,2)
            sample['feedback_position']=feedback['position'];sample['following_error']=abs(sample['loc_ref']-sample['feedback_position'])
            sample.update(task=self.report.get('active_task'),temperature_c=feedback.get('temperature'),
                          current_limit_a=self.p.current[mid-1],following_limit_rad=.04,following_duration_s=.3)
            self.report.setdefault('tracking_diagnostics',[]).append(sample)
            if sample['following_error']>.04:
                self.following_since.setdefault(mid,sample['time'])
                if sample['time']-self.following_since[mid]>.3:raise RuntimeError(f'Axis {mid}: sustained following error')
            else:self.following_since.pop(mid,None)

    def execute_segments(self,plans,pause_after=None,brakes=None):
        """Pause commits brake after the queued tail; never cancels a live cubic."""
        if self.state!='HOLDING':raise ValueError('execution requires HOLDING')
        if not plans:raise ValueError('empty path')
        local_count=0;last_plan=None;brake_pending=None;pause_requested=False;begin=time.monotonic()
        self.state='MOVING';first_count=self.submitted;self.event('PATH_START',segments=len(plans))
        deadline=begin+sum(x['duration_s'] for x in plans)+10.
        while time.monotonic()<deadline:
            status=self.pump()
            if pause_after is not None and not pause_requested and time.monotonic()-begin>=pause_after and local_count:
                pause_requested=True;self.state='BRAKING'
                brake_pending=brakes[local_count-1] if brakes else brake_at(self.p,last_plan)
                self.event('PAUSE_REQUEST',committed_segments=local_count,active_tick=status['tick'],active_ticks=status['ticks'],
                    brake_duration_s=brake_pending['duration_s'],max_lookahead_s=.5)
            if not status['flags']&2:
                next_plan=None
                if brake_pending is not None:next_plan=brake_pending;brake_pending=None
                elif not pause_requested and local_count<len(plans):next_plan=plans[local_count];local_count+=1;last_plan=next_plan
                if next_plan is not None:
                    self.transport.submit(next_plan);self.submitted+=1
                    self.report.setdefault('segment_submissions',[]).append(dict(time=time.monotonic(),ordinal=self.submitted,plan=next_plan,encoded_plan=getattr(self.transport,'last_encoded_plan',next_plan)))
                    if not self.started:
                        self.transport.start();self.started=True;self.report['pl_started_monotonic']=time.monotonic()
                    continue
            # Holding is only expected after the final zero-velocity segment.
            expected=first_count+local_count+(1 if pause_requested else 0)
            if status['holding'] and status['completed']>=expected and (pause_requested or local_count==len(plans)):
                self.state='HOLDING';self.expected=[v/(1<<24) for v in self.transport.tail]
                self.event('PAUSED_HOLDING' if pause_requested else 'PATH_COMPLETE',completed=status['completed'],target_motor_rad=self.expected)
                return pause_requested,last_plan
            self.diagnostics()
        raise TimeoutError('path execution deadline')

    def hold(self,seconds=1.):
        if self.state!='HOLDING':raise ValueError('hold requires rest')
        deadline=time.monotonic()+seconds;peak=0.
        while time.monotonic()<deadline:
            self.pump(.005)
            for mid in range(1,8):peak=max(peak,abs(self.s.require(mid,2)['position']-self.expected[mid-1]))
            if peak>.01:raise RuntimeError('holding position drift')
        self.event('HOLD_VERIFIED',seconds=seconds,max_position_error_rad=peak)

    def settle(self,goal,seconds=.3,tolerance=.004):
        motor=self.p.to_motor(goal);window=SettlingWindow(tolerance,seconds);deadline=time.monotonic()+5
        while time.monotonic()<deadline:
            self.pump(.01);errors=[abs(self.s.require(mid,2)['position']-motor[mid-1]) for mid in range(1,8)]
            if window.observe(time.monotonic(),errors):
                actual=[self.s.read(mid,0x7019) for mid in range(1,8)]
                if max(abs(a-b) for a,b in zip(actual,motor))>tolerance:raise RuntimeError('Type17 endpoint residual')
                return dict(feedback_errors_rad=errors,actual_motor_rad=actual)
        raise RuntimeError('endpoint did not settle')

    def movej(self,plans,goal,pause_after=None,brakes=None,resume_plans=None,hold_seconds=1.):
        paused,last=self.execute_segments(plans,pause_after,brakes)
        if paused:
            self.hold(hold_seconds)
            if resume_plans is None:raise ValueError('reviewed resume plans required')
            # Resume was precomputed for each possible committed tail; select the
            # exact validated branch, never solve IK or allocate a fresh path live.
            index=next(i for i,x in enumerate(plans) if x is last)
            self.event('RESUME',from_tail_index=index)
            self.execute_segments(resume_plans[index])
        return self.settle(goal)


def verify_job(job,p,return_prepared=False):
    if job.get('schema')=='kv260.reviewed_septic_job.v1':
        from septic_job import verify_septic_job
        return verify_septic_job(job,p,return_prepared)
    if job.get('schema')!='kv260.reviewed_motion_job.v1' or job['profile_id']!=p.profile_id:raise ValueError('job/profile mismatch')
    if not job['tasks']:raise ValueError('empty job')
    # Revalidate every segment and all possible brake/resume branches before I/O.
    tail=job['start_rad'];expected_model=None
    for task in job['tasks']:
        plans=task['segments']
        if max(abs(a-b) for a,b in zip(plans[0]['start_rad'],tail))>2e-6:raise ValueError('discontinuous task boundary')
        for seq in [plans]+task.get('resume_plans',[]):
            previous=None
            for plan in seq:
                w,end,v0,v1=cubic_words(plan,p)
                if previous and (max(abs(w[i*4]-previous[0][i]) for i in range(7))>2 or max(abs(a-b) for a,b in zip(v0,previous[1]))>2e-4):raise ValueError('job cubic discontinuity')
                previous=(end,v1)
            if max(map(abs,previous[1]))>2e-4:raise ValueError('path must finish at rest')
        for plan,brake,resume in zip(plans,task.get('brakes',[]),task.get('resume_plans',[])):
            cubic_words(brake,p)
            a=evaluate(plan,plan['duration_s']);b=evaluate(brake,0)
            if max(abs(x-y) for x,y in zip(a['q'],b['q']))>2e-6 or max(abs(x-y) for x,y in zip(a['v'],b['v']))>2e-5:raise ValueError('invalid brake continuity')
            if max(abs(x-y) for x,y in zip(brake['goal_rad'],resume[0]['start_rad']))>2e-6:raise ValueError('invalid resume start')
        if task.get('pause_after_s') is not None and (len(task.get('brakes',[]))!=len(plans) or len(task.get('resume_plans',[]))!=len(plans)):raise ValueError('missing reviewed pause branches')
        tail=task['goal_rad']
    # Exercise the actual sample-encoding transport before opening UIO,
    # including every possible bounded-lookahead brake and resume branch.
    class ValidationSink:
        def read(self,a):return {0x470:0x50000,0x404:7,0x800:1}.get(a,0)
        def write(self,a,v):pass
    for task in job['tasks']:
        sequences=[task['segments']]
        for i,(brake,resume) in enumerate(zip(task.get('brakes',[]),task.get('resume_plans',[]))):
            sequences.append(task['segments'][:i+1]+[brake]+resume)
        for sequence in sequences:
            sink=ContinuousTransport(ValidationSink(),p)
            for plan in sequence:sink.submit(plan)
    return True


def execute_job(job,p,output,profile_path=None):
    validation_started=time.monotonic()
    prepared=verify_job(job,p,return_prepared=True)
    validation_seconds=time.monotonic()-validation_started
    if not job.get('hardware_start_verified'):raise ValueError('synthetic start cannot execute on hardware')
    live=job.get('start_policy')=='CURRENT_FEEDBACK_REPLAN'
    if live and profile_path is None:raise ValueError('current binding requires explicit profile path')
    report=dict(status='RUNNING',commands=[],frames=[],feedback=[],readbacks={},reset_confirmed=[],stop_errors={},
      flash_writes=False,job_sha256=job_digest(job),profile_id=p.profile_id,model_id=job['model_id'],started_monotonic=time.monotonic(),
      prevalidation_seconds=validation_seconds)
    lock=open('/tmp/kv260_gripper_commission.lock','w');fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    (Path(output).parent/'SUPPORTED_DISABLE').unlink(missing_ok=True)
    transport_class=ContinuousTransport
    if job['schema']=='kv260.reviewed_septic_job.v1':
        from septic_transport import SepticTransport
        transport_class=SepticTransport
    prepared=prepared if job['schema']=='kv260.reviewed_septic_job.v1' else None
    report['precomputed_septic_segments']=len(prepared) if prepared is not None else 0
    worker=None
    if live:
        from live_motion import start_binding_worker
        worker=start_binding_worker(job,profile_path)
    try:
        d=CanDevice();executor=MoveJExecutor(p,d,report,transport_class,prepared)
    except BaseException:
        if worker is not None:
            connection,process=worker
            if process.is_alive():process.terminate()
            process.join(timeout=2);connection.close()
        lock.close()
        raise
    gc_before=gc.isenabled();gc.collect();gc.disable()
    try:
        executor.prepare(job)
        if live:
            from live_motion import bind_while_holding
            job=bind_while_holding(executor,job,profile_path,worker)
            report['bound_job_sha256']=report['binding_job_sha256']
            report['bound_start_motor_rad']=job['start_motor_rad']
            executor.release_host_hold()
        for task in job['tasks']:
            executor.event('TASK',name=task['name'],kind=task['kind'])
            begin=time.monotonic()
            endpoint=executor.movej(task['segments'],task['goal_rad'],task.get('pause_after_s'),task.get('brakes'),task.get('resume_plans'),task.get('hold_seconds',1.))
            if task.get('final_zero') or task.get('final_hold_s'):endpoint=executor.settle(task['goal_rad'],task.get('final_hold_s',2.),.002)
            report.setdefault('task_results',[]).append(dict(name=task['name'],kind=task['kind'],begin=begin,end=time.monotonic(),**endpoint))
        report['status']='PASS_REVIEWED_MOTION_JOB'
    except (Exception,KeyboardInterrupt) as exc:
        executor.state='FAULT';report['status']='FAIL';report['error']=repr(exc)
        report['failure_controller']=d.status();report['failure_trajectory']=executor.transport.status()
    finally:
        if report.get('owned'):
            if report.get('enable_attempted'):
                if not report['status'].startswith('PASS'):
                    finish_failed_job(executor,job,Path(output).parent,report)
                try:executor.s.stop_all()
                except Exception as exc:report['stop_errors']['fatal']=repr(exc)
                if sorted(report['reset_confirmed'])!=list(range(1,8)) or report['stop_errors']:report['status']='FAIL_STOP'
            else:
                d.stop();d.write(0x5a0,0)
                for mid in range(1,8):d.write(d.pp_base(mid),0)
            report['final_controller']=d.status();report['final_motion_mask']=d.read(0x5a0)
        started=report.get('pl_started_monotonic',float('inf'))
        report['host_loc_ref_after_pl_start']=sum(c['id']>>24==18 and c['data'].startswith('1670') and c['time']>=started for c in report['commands'])
        report['finished_monotonic']=time.monotonic();Path(output).write_text(json.dumps(report,indent=2)+'\n')
        if live:Path(output).with_suffix('.bound_job.json').write_text(json.dumps(job,indent=2)+'\n')
        d.close();lock.close()
        if worker is not None:
            connection,process=worker
            if process.is_alive():process.terminate()
            process.join(timeout=2);connection.close()
        if gc_before:gc.enable()
    print(json.dumps({k:report.get(k) for k in ['status','error','reset_confirmed','stop_errors']},indent=2),flush=True)
    return report

if __name__=='__main__':
    ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('--job',required=True);ap.add_argument('--profile',required=True);ap.add_argument('--output',required=True);ap.add_argument('--execute',action='store_true');a=ap.parse_args()
    job=json.loads(Path(a.job).read_text());p=JointPlanner(a.profile)
    if a.execute:
        def interrupt(*args):raise KeyboardInterrupt('operator interrupt')
        signal.signal(signal.SIGTERM,interrupt)
        result=execute_job(job,p,a.output,a.profile);sys.exit(0 if result['status'].startswith('PASS') else 1)
    verify_job(job,p)
    print(json.dumps({'status':'PASS_JOB_VALIDATION_NO_HARDWARE','job_sha256':job_digest(job)}))
