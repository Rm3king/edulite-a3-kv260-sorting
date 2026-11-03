"""Attended single-screw gentle grip, lift 30mm, replace, release and return.
Default is offline. New independent seven-axis owner; calibration gate unchanged.
"""
import argparse,copy,fcntl,hashlib,json,math,signal,time
from pathlib import Path
from calibration_motion import CalibrationExecutor,motion_gc_scope
from arm_commission import Session,parameter
from a3_protocol import enable,disable
from canctl import CanDevice
from csp_commission import bits
from motion_executor import MoveJExecutor,job_digest
from septic_transport import SepticTransport
from septic_job import verify_septic_job
from septic_trajectory import movej_c3,extrema
from joint_trajectory import JointPlanner,Q24
from control_continuity import retain_until_supported
ALL_AXES=tuple(range(1,8));ALL_MASK=127
POLICY='attended_single_screw_gentle_pick_replace_v1'

class TrialDevice:
    def __init__(self,device):self.device=device;self.last_heartbeat=None;self.max_gap=0.
    def __getattr__(self,name):return getattr(self.device,name)
    def heartbeat(self):
        now=time.monotonic()
        if self.last_heartbeat is not None:self.max_gap=max(self.max_gap,now-self.last_heartbeat)
        self.last_heartbeat=now;return self.device.heartbeat()
    def enqueue(self,identifier,payload,*args,**kwargs):
        import struct
        mid,kind=identifier&255,identifier>>24
        if mid not in ALL_AXES or kind not in (3,4,17,18):raise ValueError('Fixed trial command set')
        if kind==18:
            index=int.from_bytes(bytes(payload)[:2],'little')
            if index not in (0x7005,0x7017,0x7018,0x7028,0x7016):raise ValueError('Fixed CSP parameter set')
            if mid==7:
                value=struct.unpack('<f',bytes(payload)[4:])[0]
                if index==0x7018 and not 0<value<=.300001:raise ValueError('Gripper current cap')
                if index==0x7016 and not 1.20<=value<=1.620001:raise ValueError('Bounded gripper opening/closing')
        return self.device.enqueue(identifier,payload,*args,**kwargs)
    def write(self,address,value):
        if address==0x5a0 and value not in (0,127):raise ValueError('Seven-axis permit required')
        if 0xb40<=address<0xd00:raise ValueError('No MIT control')
        return self.device.write(address,value)

class GraspExecutor(MoveJExecutor):
    def __init__(self,p,d,report,prepared):super().__init__(p,d,report,SepticTransport,prepared)
    refresh_disabled_feedback=CalibrationExecutor.refresh_disabled_feedback
    settle_arm=CalibrationExecutor.settle
    def pump(self,seconds=.003):
        state=super().pump(seconds)
        if self.d.read(0x5a0)!=127 or self.d.read(0x5a8)!=127:raise RuntimeError('Seven-axis mask changed')
        if self.started and state['mask']!=127:raise RuntimeError('PL mask changed')
        return state
    def prepare(self, job):
        p, d, s = self.p, self.d, self.s
        if (d.read(0x470) not in (0x60002,0x60003) or d.read(0x404) or d.read(0x5a0) or
                d.read(0x5a8) or d.read(0x800)&1 or d.read(0xb40)&1):
            raise RuntimeError('stopped released controller with no active owner required')
        d.configure('normal')
        self.report['owned'] = True
        if not job.get('startup_no_reset'):
            for mid in ALL_AXES:
                s.send(disable(mid))
                s.pump(.015)
                s.require(mid, 0)
        actual = [s.read(mid, 0x7019) for mid in ALL_AXES]
        if max(abs(a-b) for a, b in zip(actual, job['start_motor_rad'])) > .002:
            raise RuntimeError('arm moved since review: recapture/replan while disabled')
        d.stop()
        d.write(0x5a0, ALL_MASK)
        d.write(0x5a4, 100000)
        for mid in ALL_AXES:
            i = mid-1
            lo, hi = job['motor_lower_rad'][i], job['motor_upper_rad'][i]
            s.plans[mid] = dict(lower=lo-.02, upper=hi+.02)
            for off, val in [(0, 1), (4, bits(lo)), (8, bits(hi)), (12, bits(p.csp_velocity[i])),
                             (16, bits(p.acceleration[i])), (20, bits(p.current[i])), (28, 1)]:
                address = d.pp_base(mid)+off
                d.write(address, val)
                if d.read(address) != val:
                    raise RuntimeError('seven-axis guard readback mismatch')
        d.configure('normal')
        if job.get('startup_no_reset'):
            # Obtain fresh reset-mode feedback through bounded watchdog setup,
            # not a Type4 command. Guard banks must be installed first.
            for mid in ALL_AXES:self.refresh_disabled_feedback(mid)
            self.report['startup_reset_skipped']=True
        for mid in ALL_AXES:
            for idx, value, integer in [(0x7005, 5, True), (0x7017, p.csp_velocity[mid-1], False),
                                        (0x7018, p.current[mid-1], False), (0x7028, 6000, True)]:
                s.write_verified(mid, idx, value, integer)
        fresh = [s.read(mid, 0x7019) for mid in ALL_AXES]
        if max(abs(a-b) for a, b in zip(fresh, actual)) > .002:
            raise RuntimeError('disabled arm moved during preparation')
        self.report['enable_capture_motor_rad'] = fresh
        for mid, value in zip(ALL_AXES, p.to_motor(job['start_rad'])):
            self.refresh_disabled_feedback(mid)
            s.send(parameter(mid, 0x7016, value))
            s.pump(.003)
            s.targets[mid] = value
            if d.read(d.pp_base(mid)+24) != 63:
                raise RuntimeError('seven-axis CSP preparation incomplete')
        # Refresh all disabled feedback in one batch before grouped enable.
        # No assumption that a reply must arrive within a fixed three ms sleep.
        refresh_stamps = {}
        for mid in ALL_AXES:
            refresh_stamps[mid] = time.monotonic()
            s.send(parameter(mid, 0x7028, 6000, True))
        deadline = time.monotonic()+.04
        while True:
            s.pump(.002)
            if all(s.feedback.get(mid, {}).get('time', -1) >= refresh_stamps[mid] for mid in ALL_AXES):
                for mid in ALL_AXES: s.require(mid, 0)
                break
            if time.monotonic() > deadline:
                raise TimeoutError('missing fresh disabled feedback before grouped enable')
        self.report['enable_attempted'] = True
        stamps = {}
        for mid in ALL_AXES:
            stamps[mid] = time.monotonic()
            s.send(enable(mid))
            s.send(parameter(mid, 0x7016, s.targets[mid]))
        s.pump(.02)
        for mid in ALL_AXES:
            if s.require(mid, 2)['time'] < stamps[mid]:
                raise RuntimeError('missing fresh seven-axis enable acknowledgement')
        s.active.update(ALL_AXES)
        s.pump(.1)
        if d.read(0x5a8) != ALL_MASK:
            raise RuntimeError('seven-axis active mask mismatch')
        self.state = 'HOLDING'
        self.event('SEVEN_AXES_HOLDING_LIMITED_GRIPPER')
        # Host must keep refreshing targets during the initial settling window.
        # PL cannot own this hold until its first segment has been started.
        self.report['stream_initial_counters'] = dict(rx_overflow=d.read(0x42c), expired=d.read(0x424))


def segments(p,qs):
    rows=[]
    for a,b in zip(qs,qs[1:]):rows.extend(movej_c3(p,a,b,1.5))
    return rows


def verify(job,p):
    if job.get('policy')!=POLICY or job.get('motion_mask')!=127 or not job.get('user_confirmed_existing_aperture_sufficient'):
        raise ValueError('Explicit attended trial and existing aperture acceptance required')
    if len(job['start_motor_rad'])!=7 or not 1.20<=job['start_rad'][6]<=1.27:raise ValueError('Reviewed moderate opening required')
    if p.current[6]>.300001 or p.velocity[6]>.080001 or p.csp_velocity[6]>.120001:raise ValueError('Gentle gripper limits required')
    if job['close_target_rad']!=1.60 or job['lift_m']!=.030:raise ValueError('Fixed bounded grip/lift')
    raw=job['route_file_text'];route=json.loads(raw)['waypoints']
    if hashlib.sha256(raw.encode()).hexdigest()!=job['route_sha256'] or route!=job['route']:raise ValueError('Route binding')
    audit=job['geometry_audit']
    if audit['status']!='PASS_SAMPLED_WITH_MONOTONIC_EXISTING_START_CONTACT_DEPARTURE' or audit.get('unexpected') or audit['route_sha256']!=job['route_sha256']:
        raise ValueError('Collision audit binding')
    qs=[x['q_rad'] for x in route]
    if len(qs)!=18 or qs[0]!=job['start_rad']:raise ValueError('Complete reviewed route required')
    elbow=list(qs[0]);elbow[2]-=math.radians(30);shoulder=list(elbow);shoulder[1]+=math.radians(20)
    if max(abs(a-b) for a,b in zip(elbow,qs[1]))>1e-10 or max(abs(a-b) for a,b in zip(shoulder,qs[2]))>1e-10:raise ValueError('Departure sequence')
    for i in range(3,16):
        expected=.12-.01*(i-3) if i<15 else .002
        if abs(route[i]['height_above_surface_m']-expected)>1e-9:raise ValueError('Descent height nodes')
    if abs(route[16]['height_above_surface_m']-.032)>1e-9 or qs[17]!=qs[15]:raise ValueError('Lift/place geometry')
    tasks=job['tasks'];last=0;expected_tasks=[]
    for i in [9,11,13,14,15]:
        expected_tasks.append(segments(p,qs[last:i+1]));last=i
    closed=qs[15][:6]+[job['close_target_rad']];lifted=qs[16][:6]+[job['close_target_rad']]
    expected_tasks.extend([segments(p,[qs[15],closed]),segments(p,[closed,lifted]),segments(p,[lifted,closed]),segments(p,[closed,qs[15]]),segments(p,qs[15::-1])])
    if len(tasks)!=10 or any(t['segments']!=expected for t,expected in zip(tasks,expected_tasks)) or tasks[-1]['goal_rad']!=qs[0] or not tasks[-1].get('return_to_live_start'):
        raise ValueError('Trial sequence differs from reviewed plan')
    records=verify_septic_job(job,p,return_prepared=True);retreats={}
    for stage,index in enumerate([9,11,13,14,15]):
        task=job['retreats'][str(stage)]
        if task['segments']!=segments(p,qs[index::-1]) or task['goal_rad']!=qs[0]:raise ValueError('Retreat mismatch')
        branch=copy.copy(job);branch['tasks']=tasks[:stage+1]+[task]
        record=verify_septic_job(branch,p,return_prepared=True);retreats[stage]=(task,record[sum(len(t['segments']) for t in tasks[:stage+1]):])
    for rs in [records]+[r[1] for r in retreats.values()]:
        for i in range(7):
            lo,hi=job['motor_lower_rad'][i],job['motor_upper_rad'][i];legal=sorted((p.lower[i]*p.direction[i]+p.offset[i],p.upper[i]*p.direction[i]+p.offset[i]))
            if not legal[0]<=lo<hi<=legal[1]:raise ValueError('Illegal guard bounds')
            if i==6 and not 1.20<=lo<hi<=1.62:raise ValueError('Gripper guard envelope')
            for record in rs:
                a,b=extrema(record['words'][8*i:8*i+8])
                if a/Q24<lo or b/Q24>hi:raise ValueError('Quantized guard violation')
    return records,retreats


def inspection(e,job,out,stage):
    ready=dict(stage=stage,job_sha256=e.report['job_sha256'],timeout_s=45)
    f=out/f'stage_{stage}_ready.json';f.write_text(json.dumps(ready));e.event('PICK_INSPECTION_READY',stage=stage)
    deadline=time.monotonic()+45
    while time.monotonic()<deadline:
        e.pump(.003);e.diagnostics();ack=out/f'stage_{stage}_ack.json'
        if ack.exists():
            data=json.loads(ack.read_text())
            if data.get('stage')!=stage or data.get('job_sha256')!=ready['job_sha256'] or data.get('action') not in ('CONTINUE','RETURN'):raise ValueError('Bad inspection ACK')
            if data['action']=='CONTINUE' and (not data.get('camera_points_path') or type(data.get('frame_number')) is not int):raise ValueError('Camera capture required')
            return data
    return dict(action='RETURN',reason='inspection_timeout')


def execute(job,p,out,*,validated=None):
    prepared,retreats=verify(job,p) if validated is None else validated
    if not job.get('hardware_start_verified'):raise ValueError('Fresh physical start required')
    out=Path(out);out.mkdir(exist_ok=False,parents=True)
    report=dict(status='RUNNING',commands=[],frames=[],feedback=[],readbacks={},reset_confirmed=[],stop_errors={},job_sha256=job_digest(job),grasp_attempted=False,physical_grasp_verified=False,flash_writes=0,zero_writes=0)
    (out/'job.json').write_text(json.dumps(job,indent=2)+'\n');settled=None;moving=False;safe=False
    with motion_gc_scope(),open('/tmp/kv260_gripper_commission.lock','a') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB);d=TrialDevice(CanDevice());e=GraspExecutor(p,d,report,prepared)
        def retreat(stage):
            task,records=retreats[stage];e.transport.prepared=records;e.transport.prepared_index=0
            e.execute_segments(task['segments']);report['return_endpoint']=e.settle_arm(job['start_rad'])
        try:
            e.prepare(job);e.settle_arm(job['start_rad']);e.release_host_hold()
            for i,task in enumerate(job['tasks']):
                moving=True;e.execute_segments(task['segments']);endpoint=e.settle_arm(task['goal_rad']);moving=False;settled=i
                gripper=e.s.read(7,0x7019);endpoint['gripper_actual_rad']=gripper
                report.setdefault('endpoints',[]).append(dict(stage=i,name=task['name'],**endpoint))
                if i<=4 or i==8:
                    if abs(gripper-task['goal_rad'][6])>.01:raise RuntimeError('Open aperture tracking/retention failed')
                if i==5:
                    report['grasp_attempted']=True;report['closed_motor_rad']=gripper
                    report['contact_candidate_from_following_error']=job['close_target_rad']-gripper>.015
                if i in (0,1,2,3,4,6):
                    decision=inspection(e,job,out,i);report.setdefault('inspections',[]).append(dict(decision,stage=i))
                    if decision['action']=='RETURN' and i<=4:
                        moving=True;retreat(i);moving=False;safe=True;break
                    # At lifted inspection, either action proceeds with the
                    # already reviewed lower-to-original-place/release/return.
            else:safe=True
            report['status']='COMPLETED_PICK_REPLACE_MOTION_GRASP_UNVERIFIED' if report['grasp_attempted'] else 'COMPLETED_INSPECTION_RETURN_NO_GRASP'
        except (Exception,KeyboardInterrupt) as error:
            report.update(status='FAIL',error=repr(error));e.event('PICK_TRIAL_FAILED',error=repr(error))
            if report.get('enable_attempted'):
                try:
                    if moving or settled is None or settled>4 or d.read(0x418) or e.transport.status()['fault']:raise RuntimeError('Retain at unverified recovery origin')
                    retreat(settled);safe=True
                except (Exception,KeyboardInterrupt) as recovery:
                    report['recovery_error']=repr(recovery);retain_until_supported(e,out,report);safe=True
        finally:
            if report.get('owned'):
                if report.get('enable_attempted'):
                    if not safe:retain_until_supported(e,out,report)
                    try:e.s.stop_all()
                    except (Exception,KeyboardInterrupt) as error:
                        report['stop_errors']['fatal']=repr(error);retain_until_supported(e,out,report)
                    if sorted(report['reset_confirmed'])!=list(ALL_AXES) or report['stop_errors']:report['status']='FAIL_DISABLE_CONFIRMATION'
                else:
                    d.stop();d.write(0x5a0,0)
                    for mid in ALL_AXES:d.write(d.pp_base(mid),0)
            report['final_controller']=d.status();report['final_motion_mask']=d.read(0x5a0);report['final_active_mask']=d.read(0x5a8);report['host_heartbeat_max_gap_s']=d.max_gap
            d.close()
            # Publish the small, stopped-hardware result before serializing the
            # full evidence. Pollers must not repeatedly transfer all CAN frames.
            summary={k:report.get(k) for k in ('status','error','grasp_attempted','physical_grasp_verified',
                     'reset_confirmed','stop_errors','final_controller','final_motion_mask','final_active_mask',
                     'host_heartbeat_max_gap_s')}
            temp=out/'runtime_summary.tmp';temp.write_text(json.dumps(summary));temp.replace(out/'runtime_summary.json')
            temp=out/'result.tmp';temp.write_text(json.dumps(report,indent=2)+'\n');temp.replace(out/'result.json')
    return report


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    for key in ('job','profile','out'):ap.add_argument('--'+key,type=Path,required=True)
    ap.add_argument('--execute-attended',action='store_true');a=ap.parse_args();job=json.loads(a.job.read_text());p=JointPlanner(a.profile);records,branches=verify(job,p)
    if not a.execute_attended:print(json.dumps(dict(status='VALIDATED_NO_HARDWARE',segments=len(records),retreats=len(branches))));return
    def interrupt(*args):raise KeyboardInterrupt('Return or retain')
    signal.signal(signal.SIGTERM,interrupt);r=execute(job,p,a.out,validated=(records,branches));print(json.dumps({k:r.get(k) for k in ['status','error','grasp_attempted','reset_confirmed']}))

if __name__=='__main__':main()
