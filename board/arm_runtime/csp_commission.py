#!/usr/bin/python3
"""Attended supported-arm CSP/PL MoveJ validation; no Flash/zero writes.

Default plans only. Execution explicitly selects small or visible. An immutable
review file pins starts/goals/profile; positions are rechecked before enabling.
No automatic retry/current increase, and no unattended keep-enabled process.
"""
from control_continuity import retain_until_supported,emit
from types import SimpleNamespace
import argparse
import fcntl
import gc
import json
from pathlib import Path
import signal
import struct
import sys
import time
from joint_trajectory import JointPlanner,PROFILE
from pl_transport import PlTransport,segment_words
# Explicit project driver path, preserving the system Python environment.
DRIVER=Path(__file__).resolve().parents[1]/'kv260_can/software'
if not DRIVER.is_dir():DRIVER=Path.home()/'kv260_can_deploy'
if str(DRIVER) not in sys.path:sys.path.append(str(DRIVER))
from canctl import CanDevice
from arm_commission import Session,parameter
from a3_protocol import enable,disable

DELTAS={'small':[.008,.003,-.003,-.008,.008,.008,.015],
        'visible':[.174532925,.034906585,-.034906585,-.174532925,.087266463,.087266463,.15],
        'axis3':[0.,0.,-.03,0.,0.,0.,0.]}

def bits(v):return struct.unpack('<I',struct.pack('<f',v))[0]


class SettlingWindow:
    """Require continuous in-tolerance samples, with no stale sample gap."""
    def __init__(self,tolerance=.002,duration=2.):
        self.tolerance=tolerance;self.duration=duration;self.since=None;self.last=None
    def observe(self,now,errors):
        if self.last is not None and not 0<now-self.last<=.08:self.since=None
        self.last=now
        if max(errors)>self.tolerance:self.since=None
        elif self.since is None:self.since=now
        return self.since is not None and now-self.since>=self.duration


def plans_for(start_motor,planner,amplitude,finish='start'):
    robot=[(v-o)/d for v,o,d in zip(start_motor,planner.offset,planner.direction)]
    planner.positions(robot)
    goal=planner.positions([q+x for q,x in zip(robot,DELTAS[amplitude])])
    if finish not in ('start','zero'):raise ValueError('finish must be start or zero')
    final=planner.positions([0.]*7 if finish=='zero' else robot)
    forward=planner.movej(robot,goal);back=planner.movej(goal,final)
    segment_words(forward,planner);segment_words(back,planner)
    return dict(start_motor_rad=start_motor,start_robot_rad=robot,goal_robot_rad=goal,
                forward_plan=forward,return_plan=back,amplitude=amplitude,
                profile_id=planner.profile_id,finish=finish,final_robot_rad=final)


def bind_review(start_motor,original,planner,amplitude,finish='start'):
    if original['profile_id']!=planner.profile_id or original['amplitude']!=amplitude:
        raise ValueError('reviewed profile/amplitude mismatch')
    if finish not in ('start','zero') or original.get('finish','start')!=finish:
        raise ValueError('reviewed finish mismatch')
    if len(start_motor)!=7 or max(abs(a-b) for a,b in zip(start_motor,original['start_motor_rad']))>.002:
        raise ValueError('posture changed >2 mrad since review; replan required')
    robot=planner.positions([(v-o)/d for v,o,d in zip(start_motor,planner.offset,planner.direction)])
    goal=planner.positions(original['goal_robot_rad'])
    final=planner.positions([0.]*7 if finish=='zero' else robot)
    if finish=='zero' and original.get('final_robot_rad')!=final:
        raise ValueError('zero finish must be explicitly reviewed for all seven channels')
    # Preserve an explicitly reviewed slow duration when rebinding fresh starts.
    floor=original['forward_plan']['duration_s']
    forward=planner.movej(robot,goal,minimum_duration_s=floor)
    back_floor=original['return_plan']['duration_s']
    if finish=='start':back_floor=max(forward['duration_s'],back_floor)
    back=planner.movej(goal,final,minimum_duration_s=back_floor)
    segment_words(forward,planner);segment_words(back,planner)
    return dict(start_motor_rad=list(start_motor),start_robot_rad=robot,goal_robot_rad=goal,
                reviewed_start_motor_rad=original['start_motor_rad'],
                forward_plan=forward,return_plan=back,amplitude=amplitude,profile_id=planner.profile_id,
                finish=finish,final_robot_rad=final)


def run(args):
    p=JointPlanner(getattr(args,'profile',PROFILE))
    if any(v > .15 for v in p.csp_velocity):
        raise ValueError('Commissioning CSP servo ceiling exceeds 0.15 rad/s')
    report=dict(status='RUNNING',commands=[],frames=[],feedback=[],
        readbacks={},reset_confirmed=[],stop_errors={},action=args.action,
        flash_writes=False,profile_id=p.profile_id,started_monotonic=time.monotonic(),
        planner_velocity_rad_s=p.velocity,csp_velocity_limit_rad_s=p.csp_velocity)
    out=Path(args.output);out.parent.mkdir(parents=True,exist_ok=True)
    lock=open('/tmp/kv260_gripper_commission.lock','w');fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    (out.parent/'SUPPORTED_DISABLE').unlink(missing_ok=True)
    d=CanDevice();s=Session(d,report);owned=False;enable_attempted=False
    gc_was_enabled=gc.isenabled()
    try:
        if d.read(0x470) not in (0x50000,0x50001,0x60000,0x60001,0x60002,0x60003):raise RuntimeError('v5 required')
        if d.read(0x404)!=0 or d.read(0x5a8) or d.read(0x800)&1:
            raise RuntimeError('controller must be stopped; another owner may exist')
        # A cyclic heap scan can outlast the 16-frame RX FIFO. Reference-count
        # reclamation remains active; this bounded report contains no cycles.
        # This removes GC pauses, not Linux scheduling or FIFO capacity limits.
        gc.collect();gc.disable();report['cyclic_gc_disabled_during_capture']=True
        d.configure('normal');owned=True
        if args.action=='execute':
            report['initial_reset_confirmed']=[]
            for mid in range(1,8):
                s.send(disable(mid));s.pump(.06);s.require(mid,0)
                report['initial_reset_confirmed'].append(mid)
        starts=[s.read(mid,0x7019) for mid in range(1,8)]
        if args.action=='execute':
            if not args.review:raise ValueError('immutable reviewed plan file required')
            original=json.loads(Path(args.review).read_text())
            reviewed=bind_review(starts,original,p,args.amplitude,args.finish)
        else:reviewed=plans_for(starts,p,args.amplitude,args.finish)
        report.update(reviewed)
        if args.action=='plan':report['status']='PASS_PLAN_ONLY'
        else:
            starts=reviewed['start_motor_rad'];plan=reviewed['forward_plan'];back=reviewed['return_plan']
            segment_words(plan,p);segment_words(back,p)
            d.stop();d.write(0x5a0,127);d.write(0x5a4,100000)
            motor_goal=p.to_motor(reviewed['goal_robot_rad'])
            motor_final=p.to_motor(reviewed['final_robot_rad'])
            for mid,(start,end) in enumerate(zip(starts,motor_goal),1):
                lo,hi=sorted((p.lower[mid-1]*p.direction[mid-1]+p.offset[mid-1],
                              p.upper[mid-1]*p.direction[mid-1]+p.offset[mid-1]))
                lo=max(lo,min(start,end,motor_final[mid-1])-.015)
                hi=min(hi,max(start,end,motor_final[mid-1])+.015)
                s.plans[mid]=dict(lower=lo,upper=hi,speed=p.csp_velocity[mid-1],current=p.current[mid-1])
                b=d.pp_base(mid)
                for off,val in [(0,1),(4,bits(lo)),(8,bits(hi)),(12,bits(p.csp_velocity[mid-1])),
                                (16,bits(p.acceleration[mid-1])),(20,bits(p.current[mid-1])),(28,1)]:
                    d.write(b+off,val)
                    if d.read(b+off)!=val:raise RuntimeError('guard readback mismatch')
            d.configure('normal')
            for mid in range(1,8):
                current=s.read(mid,0x7019)
                if abs(current-starts[mid-1])>.002:raise RuntimeError('disabled posture changed before enable')
                for index,value,integer in [(0x7005,5,True),(0x7017,p.csp_velocity[mid-1],False),
                              (0x7018,p.current[mid-1],False),(0x7028,6000,True),(0x7016,starts[mid-1],False)]:
                    s.write_verified(mid,index,value,integer)
                s.require(mid,0)
                s.targets[mid]=starts[mid-1];enable_attempted=True
                stamp=time.monotonic();s.send(enable(mid));s.pump(.02);feedback=s.require(mid,2)
                if feedback['time']<stamp or abs(feedback['position']-starts[mid-1])>.01:
                    raise RuntimeError('unexpected displacement on CSP enable')
                s.active.add(mid);s.pump(.1)
                actual=s.read(mid,0x7019)
                if abs(actual-starts[mid-1])>.005:raise RuntimeError('CSP single-axis hold drift')
                report.setdefault('csp_enable_hold',[]).append(dict(motor=mid,actual=actual))
            s.pump(.5)
            report['csp_hold_motor_rad']=[s.read(mid,0x7019) for mid in range(1,8)]
            if max(abs(a-b) for a,b in zip(starts,report['csp_hold_motor_rad']))>.01:
                raise RuntimeError('CSP initial hold tracking error')
            transport=PlTransport(d,p)
            s.target_owner='pl'
            drain_deadline=time.monotonic()+.05
            while d.read(0x41c)&65535 or d.read(0x464):
                if time.monotonic()>drain_deadline:raise TimeoutError('host target queue did not drain before PL ownership')
                s.pump(.001)
            transport.submit(plan)
            report['stream_initial_counters']=dict(expired=d.read(0x424),rx_overflow=d.read(0x42c))
            transport.start();transport.submit(back)
            report['pl_started_monotonic']=time.monotonic()
            deadline=time.monotonic()+plan['duration_s']+back['duration_s']+5
            paused=False;forward_verified=False;last_diagnostic=0.;following_since={}
            while time.monotonic()<deadline:
                s.pump(.005)
                if args.amplitude in ('visible','axis3') and time.monotonic()-last_diagnostic>=.1:
                    last_diagnostic=time.monotonic()
                    for mid in (2,3):
                        sample=dict(motor=mid,time=time.monotonic(),loc_ref=s.read(mid,0x7016),iqf=s.read(mid,0x701a))
                        sample['feedback_position']=s.require(mid,2)['position']
                        sample['following_error']=abs(sample['loc_ref']-sample['feedback_position'])
                        report.setdefault('tracking_diagnostics',[]).append(sample)
                        if args.finish=='zero' and sample['following_error']>.03:
                            following_since.setdefault(mid,sample['time'])
                            if sample['time']-following_since[mid]>.5:
                                raise RuntimeError(f'Axis {mid}: sustained CSP following error >0.03 rad')
                        else:following_since.pop(mid,None)
                status=transport.status()
                status.update(submitted=d.read(0x420),expired=d.read(0x424),rx_overflow=d.read(0x42c),
                              last_submit_us=d.read(0x430),last_lateness_us=d.read(0x434))
                if status['expired']!=report['stream_initial_counters']['expired'] or status['rx_overflow']!=report['stream_initial_counters']['rx_overflow']:
                    raise RuntimeError('expired target or feedback FIFO overflow')
                if status['fault']:raise RuntimeError(f'PL trajectory fault {status}')
                report.setdefault('pl_samples',[]).append(dict(time=time.monotonic(),**status))
                if status['completed']>=1 and not forward_verified:
                    forward_verified=True
                    report['forward_feedback']={str(mid):dict(s.require(mid,2)) for mid in range(1,8)}
                    errs=[abs(report['forward_feedback'][str(mid)]['position']-motor_goal[mid-1]) for mid in range(1,8)]
                    report['forward_errors_rad']=errs
                    if max(errs)>.01:raise RuntimeError(f'CSP forward tracking {errs}')
                if status['completed']>=1 and status['tick']>0 and not paused:
                    transport.pause_at_end();paused=True
                if status['completed']>=2 and status['holding']:
                    if args.finish=='zero':
                        window=SettlingWindow();settle_deadline=time.monotonic()+6.
                        while time.monotonic()<settle_deadline:
                            s.pump(.01)
                            errs=[abs(s.require(mid,2)['position']-motor_final[mid-1]) for mid in range(1,8)]
                            if window.observe(time.monotonic(),errs):
                                report['zero_settling']=dict(duration_s=2.,tolerance_rad=.002,final_errors_rad=errs);break
                        else:raise RuntimeError('Zero did not remain within 0.002 rad for 2 seconds')
                    else:s.pump(1.)
                    final=[s.read(mid,0x7019) for mid in range(1,8)]
                    report['final_enabled_motor_rad']=final
                    errors=[abs(a-b) for a,b in zip(final,motor_final)];report['return_errors_rad']=errors
                    if max(errors)>(.002 if args.finish=='zero' else .005):raise RuntimeError(f'CSP return tracking {errors}')
                    report['pl_final']=transport.status()
                    report['status']='PASS_CSP_PL_MOVE_AND_ZERO' if args.finish=='zero' else 'PASS_CSP_PL_OUT_AND_BACK'
                    break
            else:raise TimeoutError('PL move/hold deadline')
    except (Exception,KeyboardInterrupt) as exc:
        report['status']='FAIL';report['error']=repr(exc)
        if owned:
            report['failure_controller']=d.status()
            report['failure_trajectory']=PlTransport(d,p).status()
    finally:
        if owned:
            if enable_attempted:
                if not report['status'].startswith('PASS'):
                    owner=SimpleNamespace(d=d,s=s,report=report,event=lambda event,**kw:emit(dict(event=event,**kw)))
                    retain_until_supported(owner,out.parent,report)
                try:s.stop_all()
                except Exception as exc:report['stop_errors']['fatal']=repr(exc)
                if len(report['reset_confirmed'])!=7 or report['stop_errors']:report['status']='FAIL_STOP'
            else:
                d.stop();d.write(0x5a0,0)
                for mid in range(1,8):d.write(d.pp_base(mid),0)
            report['final_controller']=d.status()
        report['enable_commands_sent']=sum(c['id']>>24==3 for c in report['commands'])
        if 'pl_started_monotonic' in report:
            report['host_loc_ref_after_pl_start']=sum(c['id']>>24==18 and c['data'].startswith('1670') and c['time']>=report['pl_started_monotonic'] for c in report['commands'])
        report['finished_monotonic']=time.monotonic();out.write_text(json.dumps(report,indent=2)+'\n')
        d.close();lock.close()
        if gc_was_enabled:gc.enable()
    print(json.dumps({k:report.get(k) for k in ('status','error','reset_confirmed','stop_errors','return_errors_rad','pl_final')},indent=2))
    return report

if __name__=='__main__':
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--action',choices=['plan','execute'],default='plan')
    ap.add_argument('--amplitude',choices=tuple(DELTAS),default='small')
    ap.add_argument('--profile',type=Path,default=PROFILE)
    ap.add_argument('--finish',choices=['start','zero'],default='start')
    ap.add_argument('--review');ap.add_argument('--output',required=True);args=ap.parse_args()
    def interrupted(*unused):raise KeyboardInterrupt('operator/process interrupt')
    signal.signal(signal.SIGTERM,interrupted)
    result=run(args)
    if not result['status'].startswith('PASS'):sys.exit(1)
