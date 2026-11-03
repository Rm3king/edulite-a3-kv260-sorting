#!/usr/bin/python3
"""Operator-ended hand guiding from current feedback; no stored starting pose.

Six joints use Kp=0, measured-pose gravity and vendor-range damping. The gripper
keeps its current CSP opening. The ARM updates gravity at nominal 100 Hz; the
FPGA CAN engine enforces torque/packet limits, feedback freshness and heartbeat.
There is no timed reset, prepositioning or automatic return motion.
"""
import argparse,fcntl,gc,hashlib,json,math,signal,struct,time
from pathlib import Path
import numpy as np
from gravity_executor import GravityExecutor,CanDevice,parameter,tracking_packet
from gravity_trajectory import GravityModel
from joint_trajectory import JointPlanner
from motion_runtime import MotionRuntime
from control_continuity import emit,finish_failed_job

CONFIG=dict(period_s=.01,kp=[0.]*6,kd=[.15,.15,.15,.10,.05,.05],gravity_ema_alpha=.1,
            feedback_age_s=.08,trace_window_limit=20000,automatic_stop=False,automatic_motion=False)


class DirectModel(GravityModel):
    def __init__(self,runtime,p):
        super().__init__(runtime,p)
        self.parent_id=self.id;self.kp=np.zeros(6);self.kd=np.array(CONFIG['kd'])
        self.guard_kp=self.kp.copy();self.guard_kd=self.kd.copy()
        self.id=hashlib.sha256(json.dumps(dict(parent=self.parent_id,freedrive=CONFIG),sort_keys=True).encode()).hexdigest()
        self.last_clipped=[False]*6

    def reference_motor(self,mid,value):
        # This is only the protocol's position field: Kp is identically zero.
        # Clip that inactive reference, while gravity uses the actual position.
        i=mid-1;lo,hi=sorted((self.p.lower[i]*self.p.direction[i]+self.p.offset[i],self.p.upper[i]*self.p.direction[i]+self.p.offset[i]))
        return min(hi,max(lo,value))

    def measured_gravity(self,motor):
        if len(motor)!=7 or not np.isfinite(motor).all():raise ValueError('finite seven-axis feedback required')
        q=[(float(x)-o)/d for x,o,d in zip(motor,self.p.offset,self.p.direction)]
        raw=np.array(self.r.kin.compute_gravity(q[:6])[:6])*np.array(self.p.direction[:6])*self.scale
        if not np.isfinite(raw).all():raise ValueError('nonfinite gravity')
        self.last_clipped=(np.abs(raw)>self.caps).tolist()
        # Saturation produces partial compensation, not a surprise motor reset.
        return np.clip(raw,-self.caps,self.caps)


class Policy:
    def __init__(self,p,m):
        self.p,self.m=p,m;self.filtered=None;self.previous=None;self.finish_begin=None;self.steady=[]

    def step(self,feedback,now,command=None):
        if len(feedback)!=7:raise RuntimeError('seven-axis feedback required')
        for i,f in enumerate(feedback):
            if f['motor']!=i+1 or f['mode']!=2 or f['fault'] or not 0<=now-f['time']<=CONFIG['feedback_age_s']:
                raise RuntimeError('stale/faulted/non-operating feedback')
        if self.previous is not None and now<=self.previous:raise RuntimeError('feedback time order')
        dt=CONFIG['period_s'] if self.previous is None else now-self.previous;self.previous=now
        actual=np.array([f['position'] for f in feedback],float)
        if not np.isfinite(actual).all():raise RuntimeError('nonfinite actual position')
        alpha=1-(1-CONFIG['gravity_ema_alpha'])**(dt/CONFIG['period_s'])
        self.filtered=actual.copy() if self.filtered is None else alpha*actual+(1-alpha)*self.filtered
        tau=self.m.measured_gravity(self.filtered.tolist())
        commands=[tracking_packet(i+1,self.m.reference_motor(i+1,actual[i]),tau[i],0.,self.m.kd[i]) for i in range(6)]
        if command=='finish_supported':self.finish_begin=now;self.steady=[]
        elif command is not None:raise ValueError('operator-supported finish is the only normal stop')
        done=False
        if self.finish_begin is not None:
            self.steady.append((now,actual.copy()));self.steady=[x for x in self.steady if now-x[0]<=.6]
            done=now-self.steady[0][0]>=.5 and np.max(np.ptp([x[1][:6] for x in self.steady],axis=0))<.005
        return dict(phase='DONE' if done else 'DRAG',packets=commands,kp=[0.]*6,kd=self.m.kd.tolist(),gravity_motor_nm=tau.tolist(),
                    torque_saturated=self.m.last_clipped,actual_motor_rad=actual.tolist(),finish_pending=self.finish_begin is not None,cycle_interval_s=dt)


class ManualVelocityTelemetry:
    def observe(self,feedback):
        if not math.isfinite(feedback['velocity']):raise RuntimeError('nonfinite manual velocity')
        return False


def execute(p,output,control,live):
    m=DirectModel(MotionRuntime(),p)
    lo=[];hi=[]
    for a,b,d,o in zip(p.lower,p.upper,p.direction,p.offset):
        lower,upper=sorted((a*d+o,b*d+o));lo.append(lower);hi.append(upper)
    job=dict(motor_lower_rad=lo,motor_upper_rad=hi)
    report=dict(status='RUNNING',commands=[],frames=[],feedback=[],readbacks={},reset_confirmed=[],stop_errors={},
        flash_writes=False,config=CONFIG,model_id=m.r.model_id,gravity_profile_id=m.parent_id,freedrive_profile_id=m.id,
        pl_trajectory_used=False,live_start=True,samples=[],trace_dropped_counts={})
    lock=open('/tmp/kv260_gripper_commission.lock','w');fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    (Path(output).parent/'SUPPORTED_DISABLE').unlink(missing_ok=True)
    d=CanDevice();e=GravityExecutor(p,d,report,[],m);e.s.velocity_monitor=ManualVelocityTelemetry()
    old_gc=gc.isenabled();gc.collect();gc.disable();last_command_id=None;last_live=0.;policy=Policy(p,m);started=time.monotonic()
    try:
        Path(control).unlink(missing_ok=True)
        e.prepare(job,live_start=True);e.s.target_owner='freedrive';started=time.monotonic();next_tick=started
        report['gripper_hold_motor_rad']=e.s.targets[7]
        emit(dict(event='FREEDRIVE_ACTIVE',kp=CONFIG['kp'],kd=CONFIG['kd'],automatic_stop=False,automatic_motion=False))
        while True:
            now=time.monotonic()
            if now<next_tick:e.s.pump(min(.002,next_tick-now));continue
            next_tick=now+CONFIG['period_s'];command=None
            if Path(control).exists():
                try:
                    request=json.loads(Path(control).read_text())
                    if request['id']!=last_command_id:last_command_id=request['id'];command=request['command']
                except (OSError,ValueError,KeyError,TypeError) as exc:
                    report['control_file_error']=repr(exc)
            state=policy.step([e.s.require(mid,2) for mid in range(1,8)],now,command)
            for msg in state.pop('packets'):e.s.send(msg)
            e.s.send(parameter(7,0x7016,e.s.targets[7]))
            if d.read(0x424)!=report['stream_initial_counters']['expired']:raise RuntimeError('expired command')
            overflow=d.read(0x42c)
            if overflow!=report['stream_initial_counters']['rx_overflow']:
                report['stream_initial_counters']['rx_overflow']=overflow
                report['observation_overflow_count']=overflow
            report['maximum_cycle_interval_s']=max(report.get('maximum_cycle_interval_s',0),state['cycle_interval_s'])
            if now-last_live>=.1:
                last_live=now;state.update(time=now,elapsed_s=now-started);report['samples'].append(state)
                try:
                    tmp=Path(str(live)+'.tmp');tmp.write_text(json.dumps(state)+'\n');tmp.replace(live)
                except OSError as exc:report['live_file_error']=repr(exc)
                for key in ('commands','frames','feedback','samples'):
                    excess=len(report[key])-CONFIG['trace_window_limit']
                    if excess>0:
                        del report[key][:excess];report['trace_dropped_counts'][key]=report['trace_dropped_counts'].get(key,0)+excess
            if state['phase']=='DONE':report['status']='PASS_ATTENDED_FREEDRIVE_SUPPORTED_STOP';break
    except (Exception,KeyboardInterrupt) as exc:report.update(status='FAIL',error=repr(exc))
    finally:
        if report.get('owned'):
            if report.get('enable_attempted'):
                if not report['status'].startswith('PASS'):
                    def maintain_manual_compensation():
                        e.s.pump(.003)
                        state=policy.step([e.s.require(mid,2) for mid in range(1,8)],time.monotonic())
                        for packet in state['packets']:e.s.send(packet)
                        e.s.send(parameter(7,0x7016,e.s.targets[7]))
                    finish_failed_job(e,None,Path(output).parent,report,maintain_manual_compensation)
                try:e.s.stop_all()
                except Exception as exc:report['stop_errors']['fatal']=repr(exc)
                if sorted(report['reset_confirmed'])!=list(range(1,8)) or report['stop_errors']:report['status']='FAIL_STOP'
            else:
                d.stop();d.write(0x5a0,0)
                for mid in range(1,8):d.write(d.pp_base(mid),0)
            reset_ok=(sorted(report['reset_confirmed'])==list(range(1,8)) and not report['stop_errors']) if report.get('enable_attempted') else report.get('initial_reset_confirmed')==list(range(1,8))
            if report.get('volatile_configuration_changed') and reset_ok:
                try:e.restore_csp_configuration()
                except Exception as exc:report.update(status='FAIL_RESTORE',restore_error=repr(exc))
            report['final_controller']=d.status();report['final_motion_mask']=d.read(0x5a0)
        report['finished_monotonic']=time.monotonic();Path(output).write_text(json.dumps(report,indent=2)+'\n')
        d.close();lock.close()
        if old_gc:gc.enable()
    print(json.dumps({k:report.get(k) for k in ('status','error','reset_confirmed','stop_errors')}),flush=True)
    return report


if __name__=='__main__':
    ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('--profile',required=True);ap.add_argument('--output',required=True)
    ap.add_argument('--control',required=True);ap.add_argument('--live',required=True);ap.add_argument('--hands-supporting-attended',action='store_true');a=ap.parse_args()
    if not a.hands_supporting_attended:ap.error('operator must support the arm during manual guiding and a fault stop')
    def interrupt(*args):raise KeyboardInterrupt('operator interrupt')
    signal.signal(signal.SIGTERM,interrupt)
    result=execute(JointPlanner(a.profile),a.output,a.control,a.live);raise SystemExit(result['status']!='PASS_ATTENDED_FREEDRIVE_SUPPORTED_STOP')
