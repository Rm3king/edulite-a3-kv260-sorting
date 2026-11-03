"""Attended arm-only staged descent with a cached retreat at every stop.

Default validates offline. Open gripper stays disabled. No contact or grasp.
Missing/rejected image acknowledgement retreats from the last settled stage;
an in-motion fault retains control rather than executing deeper waypoints.
"""
import argparse
import copy
import fcntl
import hashlib
import json
import math
from pathlib import Path
import signal
import time
from calibration_motion import (ArmOnlyDevice, CalibrationExecutor, motion_gc_scope,
                                AXES, MASK)
from canctl import CanDevice
from control_continuity import retain_until_supported
from joint_trajectory import JointPlanner, Q24
from motion_executor import job_digest
from septic_job import verify_septic_job
from septic_trajectory import movej_c3, extrema

POLICY='visual_open_jaws_staged_descent_v1'
HEIGHTS=(.06,.04,.02,.01)


def route_segments(p, route, minimum):
    result=[]
    for a,b in zip(route,route[1:]):result.extend(movej_c3(p,a,b,minimum))
    return result


def verify(job,p):
    if job.get('descent_policy')!=POLICY or job.get('motion_mask')!=MASK:
        raise ValueError('Explicit six-axis descent policy required')
    if job.get('unused_channel_7_rad')!=0 or len(job['start_motor_rad'])!=6:
        raise ValueError('Gripper excluded')
    if not 1.5<=job['minimum_leg_s']<=10:raise ValueError('Leg speed gate')
    raw=job['route_file_text'];route=json.loads(raw)['waypoints'];qs=[r['q_rad'] for r in route]
    if hashlib.sha256(raw.encode()).hexdigest()!=job['route_file_sha256'] or route!=job['route']:
        raise ValueError('Route provenance mismatch')
    if len(qs)!=15 or qs[0]!=job['start_rad']:raise ValueError('Reviewed 12 height nodes required')
    elbow=list(qs[0]);elbow[2]-=math.radians(30)
    lift=list(elbow);lift[1]+=math.radians(20)
    if max(abs(a-b) for a,b in zip(qs[1],elbow))>1e-10 or max(abs(a-b) for a,b in zip(qs[2],lift))>1e-10:
        raise ValueError('Elbow-first supported departure required')
    for i,q in enumerate(qs):
        p.positions(q)
        if q[6]!=0:raise ValueError('No gripper trajectory')
        if i>=3 and abs(route[i]['height_above_surface_m']-(.12-.01*(i-3)))>1e-9:
            raise ValueError('No descent below surface+10mm; 10mm nodes required')
    audit=job['geometry_audit']
    if audit.get('status')!='PASS_SAMPLED_WITH_MONOTONIC_EXISTING_START_CONTACT_DEPARTURE' or audit.get('unexpected'):
        raise ValueError('Collision audit failed')
    if audit['route_sha256']!=job['route_file_sha256'] or not audit.get('box_walls_checked'):
        raise ValueError('Collision provenance mismatch')
    aperture=job['aperture_evidence']
    if aperture['status']!='PASS_MODERATE_OPEN_ONLY_WIDTH_UNMEASURED' or not aperture['disabled_confirmed']:
        raise ValueError('Successful stopped aperture probe required')
    if abs(aperture['final_motor_rad']-job['observed_gripper_motor_rad'])>.002:
        raise ValueError('Opened gripper position changed')
    if not 18<=job['measured_aperture_mm']<=35 or not job.get('measurement_complete_hands_clear'):
        raise ValueError('Actual moderate aperture and hands-clear observation required')
    tasks=job['tasks']
    if len(tasks)!=5 or tasks[-1]['goal_rad']!=qs[0] or not tasks[-1].get('return_to_live_start'):
        raise ValueError('Four stages and supported return required')
    previous=0;prefix=[];retreats={}
    for stage,(height,index) in enumerate(zip(HEIGHTS,(9,11,13,14))):
        task=tasks[stage]
        if task['route_index']!=index or task['height_above_surface_m']!=height or task['goal_rad']!=qs[index]:
            raise ValueError('Stage identity mismatch')
        expected=route_segments(p,qs[previous:index+1],job['minimum_leg_s'])
        if task['segments']!=expected:raise ValueError('Unreviewed stage trajectory')
        prefix.append(task)
        retreat=job['retreats'][str(stage)]
        if retreat['goal_rad']!=qs[0] or retreat['segments']!=route_segments(p,qs[index::-1],job['minimum_leg_s']):
            raise ValueError('Retreat differs from exact reverse path')
        branch=copy.copy(job);branch['tasks']=prefix+[retreat]
        records=verify_septic_job(branch,p,return_prepared=True)
        retreat_records=records[sum(len(t['segments']) for t in prefix):]
        retreats[stage]=(retreat,retreat_records)
        previous=index
    if tasks[-1]!=job['retreats']['3']:raise ValueError('Final retreat mismatch')
    prepared=verify_septic_job(job,p,return_prepared=True)
    # Audit every alternative return as well as the normal full sequence.
    for records in [prepared]+[r[1] for r in retreats.values()]:
        for i in range(6):
            legal=sorted((p.lower[i]*p.direction[i]+p.offset[i],p.upper[i]*p.direction[i]+p.offset[i]))
            lo,hi=job['motor_lower_rad'][i],job['motor_upper_rad'][i]
            if not legal[0]<=lo<hi<=legal[1]:raise ValueError('Guard outside legal limits')
            for record in records:
                low,high=extrema(record['words'][8*i:8*i+8])
                if low/Q24<lo or high/Q24>hi:raise ValueError('Quantized trajectory outside guard')
    return prepared,retreats


def inspect_stage(e,job,out,stage,before):
    ready=dict(job_sha256=e.report['job_sha256'],stage=stage,height_above_surface_m=HEIGHTS[stage],
               endpoint=before,timeout_s=45.)
    f=out/f'stage_{stage}_ready.json';tmp=f.with_suffix('.tmp')
    tmp.write_text(json.dumps(ready));tmp.replace(f)
    e.event('STAGED_DESCENT_READY',stage=stage,height_above_surface_m=HEIGHTS[stage])
    deadline=time.monotonic()+45
    while time.monotonic()<deadline:
        e.pump(.003);e.diagnostics()
        ack=out/f'stage_{stage}_ack.json'
        if ack.exists():
            data=json.loads(ack.read_text())
            if data.get('job_sha256')!=ready['job_sha256'] or data.get('stage')!=stage:
                raise ValueError('Stage acknowledgement mismatch')
            if data.get('action') not in ('CONTINUE','RETURN'):raise ValueError('Unknown acknowledgement')
            if data['action']=='CONTINUE' and (not data.get('camera_points_path') or type(data.get('frame_number')) is not int):
                raise ValueError('Captured camera evidence required before further descent')
            after=e.settle(job['tasks'][stage]['goal_rad'])
            drift=max(abs(a-b) for a,b in zip(before['actual_motor_rad'],after['actual_motor_rad']))
            if drift>.001:raise ValueError('Stopped capture bracket moved')
            gripper=e.s.read(7,0x7019)
            if abs(gripper-job['observed_gripper_motor_rad'])>.002:raise ValueError('Disabled jaw aperture drift')
            return dict(action=data['action'],ack=data,after=after,gripper_motor_rad=gripper,max_bracket_drift_rad=drift)
    return dict(action='RETURN',reason='inspection_timeout_no_further_descent')


def return_from_stage(e,job,stage,retreats,report):
    task,records=retreats[stage]
    # Only at a verified rest endpoint. Quantization records include the
    # already-run prefix, so the first return tick matches the held PL tail.
    e.transport.prepared=records;e.transport.prepared_index=0
    e.execute_segments(task['segments'])
    report['return_endpoint']=e.settle(job['start_rad'])
    report['safe_to_disable_after_return_or_support']=True


def execute(job,p,out):
    prepared,retreats=verify(job,p)
    if not job.get('hardware_start_verified') or job.get('aperture_measurement',{}).get('synthetic_for_offline_validation'):
        raise ValueError('Actual reviewed start and measured aperture required')
    out=Path(out);out.mkdir(parents=True,exist_ok=False)
    report=dict(status='RUNNING',commands=[],frames=[],feedback=[],readbacks={},reset_confirmed=[],stop_errors={},
                job_sha256=job_digest(job),motion_mask=MASK,grasp_attempted=False,stages=[],flash_writes=0,zero_writes=0)
    (out/'job.json').write_text(json.dumps(job,indent=2)+'\n')
    settled_stage=None;in_motion=False
    with motion_gc_scope(),open('/tmp/kv260_gripper_commission.lock','a') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        d=ArmOnlyDevice(CanDevice());e=CalibrationExecutor(p,d,report,prepared)
        try:
            e.prepare(job);report['initial_hold_endpoint']=e.settle(job['start_rad']);e.release_host_hold()
            for stage,task in enumerate(job['tasks'][:-1]):
                in_motion=True;e.execute_segments(task['segments'])
                before=e.settle(task['goal_rad']);in_motion=False;settled_stage=stage
                inspection=inspect_stage(e,job,out,stage,before)
                report['stages'].append(dict(stage=stage,before=before,inspection=inspection))
                if inspection['action']=='RETURN':break
            in_motion=True;return_from_stage(e,job,settled_stage,retreats,report);in_motion=False
            report['status']='PASS_STAGED_DESCENT_RETURN_NO_GRASP'
        except (Exception,KeyboardInterrupt) as error:
            report.update(status='FAIL',error=repr(error));e.event('STAGED_DESCENT_FAILED',error=repr(error))
            if report.get('enable_attempted'):
                try:
                    if in_motion or settled_stage is None or d.read(0x418) or e.transport.status()['fault']:
                        raise RuntimeError('No verified stopped retreat origin')
                    return_from_stage(e,job,settled_stage,retreats,report)
                except (Exception,KeyboardInterrupt) as recovery:
                    report['recovery_error']=repr(recovery);retain_until_supported(e,out,report)
                    report['safe_to_disable_after_return_or_support']=True
        finally:
            if report.get('owned'):
                if report.get('enable_attempted'):
                    if not report.get('safe_to_disable_after_return_or_support'):retain_until_supported(e,out,report)
                    try:e.s.stop_all()
                    except (Exception,KeyboardInterrupt) as error:
                        report['stop_errors']['fatal']=repr(error);retain_until_supported(e,out,report)
                    if sorted(report['reset_confirmed'])!=list(AXES) or report['stop_errors']:report['status']='FAIL_DISABLE_CONFIRMATION'
                else:
                    d.stop();d.write(0x5a0,0)
                    for mid in AXES:d.write(d.pp_base(mid),0)
            report['final_controller']=d.status();report['final_motion_mask']=d.read(0x5a0)
            report['gripper_control_commands']=sum(c['id']&255==7 and c['id']>>24!=17 for c in report['commands'])
            report['host_heartbeat_max_gap_s']=d.max_heartbeat_gap_s
            d.close();(out/'result.json').write_text(json.dumps(report,indent=2)+'\n')
    return report


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ('job','profile','out'):parser.add_argument('--'+name,type=Path,required=True)
    parser.add_argument('--execute-attended',action='store_true');a=parser.parse_args()
    job=json.loads(a.job.read_text());p=JointPlanner(a.profile)
    prepared,retreats=verify(job,p)
    if not a.execute_attended:
        print(json.dumps(dict(status='VALIDATED_NO_HARDWARE',segments=len(prepared),retreat_branches=len(retreats))));return
    def interrupt(*args):raise KeyboardInterrupt('Retain or reviewed retreat')
    signal.signal(signal.SIGTERM,interrupt)
    result=execute(job,p,a.out)
    print(json.dumps({k:result.get(k) for k in ('status','error','reset_confirmed','gripper_control_commands')}))


if __name__=='__main__':main()
