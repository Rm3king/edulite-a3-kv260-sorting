"""Attended corrected local screw pick. Continuous upper approach, low stop.
Independent retry policy; original calibration and first trial gates unchanged.
"""
import argparse,fcntl,hashlib,json,signal,time,copy,math
from pathlib import Path
from visual_pick_trial import GraspExecutor,TrialDevice,inspection
from calibration_motion import motion_gc_scope
from motion_executor import job_digest
from joint_trajectory import JointPlanner,Q24
from septic_job import verify_septic_job
from septic_trajectory import extrema
from canctl import CanDevice
from control_continuity import retain_until_supported
ALL_AXES=tuple(range(1,8))
POLICY='attended_local_corrected_continuous_pick_v1'
class RetryDevice(TrialDevice):
    def enqueue(self,identifier,payload,*args,**kwargs):
        import struct
        if identifier&255==7 and identifier>>24==18 and int.from_bytes(bytes(payload)[:2],'little')==0x7016:
            value=struct.unpack('<f',bytes(payload)[4:])[0]
            if not 1.08<=value<=1.620001:raise ValueError('Moderately wider bounded aperture only')
            return self.device.enqueue(identifier,payload,*args,**kwargs)
        return super().enqueue(identifier,payload,*args,**kwargs)

def trajectory_digest(job):
    return hashlib.sha256(json.dumps({'tasks':job['tasks'],'retreats':job['retreats']},sort_keys=True,separators=(',',':'),allow_nan=False).encode()).hexdigest()

def verify(job,p):
    if job.get('policy')!=POLICY or job.get('motion_mask')!=127:raise ValueError('Explicit retry policy')
    if len(job['start_motor_rad'])!=7 or not 1.20<=job['start_rad'][6]<=1.27:raise ValueError('Observed starting aperture')
    if job['open_target_rad']!=1.12 or job['close_target_rad']!=1.6 or job['lift_m']!=.03:raise ValueError('Fixed small opening and lift')
    if p.current[6]>.300001 or p.velocity[6]>.100001 or p.csp_velocity[6]>.150001:raise ValueError('Gentle jaw limits')
    if job['minimum_height_above_surface_m']!=.017 or not job['marker_removed'] or not job['fresh_one_screw_one_nut']:raise ValueError('Observed correction/scene contract')
    shift=job['local_compensation_base_m']
    if len(shift)!=3 or not all(math.isfinite(x) for x in shift) or sum(x*x for x in shift)>.013**2:raise ValueError('Bounded local correction')
    audit=job['geometry_audit']
    if audit['status']!='PASS_SAMPLED_WITH_MONOTONIC_EXISTING_START_CONTACT_DEPARTURE' or audit.get('unexpected') or audit['trajectory_sha256']!=trajectory_digest(job):raise ValueError('Exact trajectory audit binding')
    tasks=job['tasks'];names=['moderate_open','elbow_shoulder_high','continuous_upper','slow_final','gentle_close','lift_30mm','replace','release','return_supported']
    if [t['name'] for t in tasks]!=names or not tasks[-1].get('return_to_live_start'):raise ValueError('Fixed pick/replace sequence')
    opened=job['start_rad'][:6]+[1.12]
    if any(max(abs(a-b) for a,b in zip(tasks[i]['goal_rad'],opened))>1e-10 for i in (0,-1)):raise ValueError('Supported return with open jaw')
    for i in [0,4,7]:
        if any(max(abs(s['goal_rad'][k]-s['start_rad'][k]) for k in range(6))>1e-10 for s in tasks[i]['segments']):raise ValueError('Arm must remain still during jaw changes')
    for i in [1,2,3,5,6,8]:
        expected=1.6 if i in [5,6] else 1.12
        for s in tasks[i]['segments']:
            row=s['coefficients_rad'][6]
            if abs(row[0]-expected)>1e-10 or any(abs(v)>1e-10 for v in row[1:]):raise ValueError('Jaw state along arm motion')
    records=verify_septic_job(job,p,return_prepared=True);retreats={}
    for stage in range(4):
        task=job['retreats'][str(stage)]
        if task['goal_rad']!=opened:raise ValueError('Retreat must end supported')
        branch=copy.copy(job);branch['tasks']=tasks[:stage+1]+[task]
        rs=verify_septic_job(branch,p,return_prepared=True);retreats[stage]=(task,rs[sum(len(t['segments']) for t in tasks[:stage+1]):])
    for rs in [records]+[r[1] for r in retreats.values()]:
        for i in range(7):
            lo,hi=job['motor_lower_rad'][i],job['motor_upper_rad'][i];legal=sorted((p.lower[i]*p.direction[i]+p.offset[i],p.upper[i]*p.direction[i]+p.offset[i]))
            if not legal[0]<=lo<hi<=legal[1]:raise ValueError('Joint guard envelope')
            if i==6 and not 1.08<=lo<hi<=1.62:raise ValueError('Jaw guard envelope')
            for record in rs:
                a,b=extrema(record['words'][8*i:8*i+8])
                if a/Q24<lo or b/Q24>hi:raise ValueError('Quantized trajectory guard')
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
        fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB);d=RetryDevice(CanDevice());e=GraspExecutor(p,d,report,prepared)
        def retreat(stage):
            task,records=retreats[stage];e.transport.prepared=records;e.transport.prepared_index=0
            e.execute_segments(task['segments']);report['return_endpoint']=e.settle_arm(job['start_rad'])
        try:
            e.prepare(job);e.settle_arm(job['start_rad']);e.release_host_hold()
            for i,task in enumerate(job['tasks']):
                moving=True;e.execute_segments(task['segments']);endpoint=e.settle_arm(task['goal_rad']);moving=False;settled=i
                gripper=e.s.read(7,0x7019);endpoint['gripper_actual_rad']=gripper
                report.setdefault('endpoints',[]).append(dict(stage=i,name=task['name'],**endpoint))
                if i<=3 or i==7:
                    if abs(gripper-task['goal_rad'][6])>.01:raise RuntimeError('Open aperture tracking/retention failed')
                if i==4:
                    report['grasp_attempted']=True;report['closed_motor_rad']=gripper
                    report['contact_candidate_from_following_error']=job['close_target_rad']-gripper>.015
                if i in (3,5):
                    decision=inspection(e,job,out,i);report.setdefault('inspections',[]).append(dict(decision,stage=i))
                    if decision['action']=='RETURN' and i<=3:
                        moving=True;retreat(i);moving=False;safe=True;break
                    # At lifted inspection, either action proceeds with the
                    # already reviewed lower-to-original-place/release/return.
            else:safe=True
            report['status']='COMPLETED_PICK_REPLACE_MOTION_GRASP_UNVERIFIED' if report['grasp_attempted'] else 'COMPLETED_INSPECTION_RETURN_NO_GRASP'
        except (Exception,KeyboardInterrupt) as error:
            report.update(status='FAIL',error=repr(error));e.event('PICK_TRIAL_FAILED',error=repr(error))
            if report.get('enable_attempted'):
                try:
                    if moving or settled is None or settled>3 or d.read(0x418) or e.transport.status()['fault']:raise RuntimeError('Retain at unverified recovery origin')
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
