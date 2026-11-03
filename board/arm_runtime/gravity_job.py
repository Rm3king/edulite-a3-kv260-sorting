#!/usr/bin/python3
"""Build a matched, reviewed gravity-only comparison from a stable capture."""
import argparse,copy,hashlib,json,math
from pathlib import Path
import numpy as np
from joint_trajectory import JointPlanner
from motion_runtime import MotionRuntime
from gravity_trajectory import GravityModel,hold_segment,prepare_stream,validate_records,measured_start_reference
from septic_trajectory import movej_c3,extrema
from motion_executor import job_digest


def prepare(capture,p,runtime,circle_job,static_only=False):
    if capture['status']!='PASS_READ_ONLY_ALL_7':raise ValueError('all seven axes must respond')
    by={mid:[t['decoded']['position_motor_rad'] for t in capture['transactions'] if t['motor']==mid and t.get('decoded',{}).get('valid')] for mid in range(1,8)}
    if any(len(v)!=3 or max(v)-min(v)>.002 for v in by.values()):raise ValueError('stable 21-read capture required')
    motor=[by[i][-1] for i in range(1,8)];start,adjustment=measured_start_reference(p,motor)
    if circle_job['model_id']!=runtime.model_id or circle_job['profile_id']!=p.profile_id:raise ValueError('circle model/profile mismatch')
    reference=next(t for t in circle_job['tasks'] if t['kind']=='MoveC')
    anchor=reference['segments'][0]['start_rad'];m=GravityModel(runtime,p)
    # IK closes the circle within microradians, not bit-exact joint angles.
    # Add a C3 rest-to-rest closure so the two comparison runs share the
    # identical joint start, without hiding a discontinuity in a hold segment.
    circle=copy.deepcopy(reference['segments'])+movej_c3(p,reference['goal_rad'],anchor,minimum_duration_s=.2)
    tasks=[dict(name='MIT_C3_approach_compensated',gravity_alpha=[1.,1.],segments=movej_c3(p,start,anchor),goal_rad=anchor)]
    def hold(name,a,b,seconds):
        tasks.append(dict(name=name,gravity_alpha=[a,b],segments=[hold_segment(p,anchor,seconds)],goal_rad=anchor))
    hold('static_gravity_on_before',1.,1.,2.)
    hold('gravity_ramp_down',1.,0.,2.)
    hold('static_gravity_off',0.,0.,2.)
    if not static_only:tasks.append(dict(name='A_C3_XZ_circle_gravity_off',gravity_alpha=[0.,0.],segments=copy.deepcopy(circle),goal_rad=anchor,geometry=copy.deepcopy(reference['geometry'])))
    hold('gravity_ramp_up',0.,1.,2.)
    hold('static_gravity_on_after',1.,1.,2.)
    if not static_only:tasks.append(dict(name='B_C3_XZ_circle_gravity_on',gravity_alpha=[1.,1.],segments=copy.deepcopy(circle),goal_rad=anchor,geometry=copy.deepcopy(reference['geometry'])))
    hold('compensated_final_hold',1.,1.,2.)
    # Stay enabled across the complete experiment. Return to the measured
    # supported start before reset; an intermediate reset can drop the arm.
    tasks.append(dict(name='MIT_C3_return_to_supported_start',gravity_alpha=[1.,1.],segments=movej_c3(p,anchor,start),goal_rad=start))
    lower=np.full(7,np.inf);upper=-lower.copy()
    for task in tasks:
        for s in task['segments']:
            for i,row in enumerate(s['motor_coefficients_rad']):
                lo,hi=extrema(row);lower[i]=min(lower[i],lo);upper[i]=max(upper[i],hi)
    for i in range(7):
        lo,hi=sorted((p.lower[i]*p.direction[i]+p.offset[i],p.upper[i]*p.direction[i]+p.offset[i]))
        margin=m.config['following_envelope_rad'] if i<6 else .02
        lower[i]=max(lo,float(lower[i])-margin);upper[i]=min(hi,float(upper[i])+margin)
    job=dict(schema='kv260.reviewed_gravity_job.v1',hardware_start_verified=not capture.get('synthetic_for_planning_only',False),
        profile_id=p.profile_id,model_id=runtime.model_id,gravity_profile_id=m.id,start_rad=start,start_motor_rad=motor,
        start_reference_adjustment_motor_rad=adjustment,
        initial_gravity_motor_nm=m.gravity(start).tolist(),motor_lower_rad=lower.tolist(),motor_upper_rad=upper.tolist(),tasks=tasks,
        circle_reference_sha256=job_digest(circle_job),comparison='Same C3 q/v, kp/kd, torque caps; only gravity alpha changes',static_only=static_only,return_to_supported_start=True)
    records=prepare_stream(m,tasks);validate_records(records,job,p,m)
    if not static_only:
        a=[r for r in records if r['task']=='A_C3_XZ_circle_gravity_off'];b=[r for r in records if r['task']=='B_C3_XZ_circle_gravity_on']
        if len(a)!=len(b) or any(x['words'][:112]!=y['words'][:112] or x['words'][210:]!=y['words'][210:] for x,y in zip(a,b)):
            raise ValueError('A/B q/v/gains/time are not identical')
    info=dict(job_sha256=job_digest(job),model_id=runtime.model_id,profile_id=p.profile_id,gravity_profile_id=m.id,
        total_seconds=sum(s['duration_s'] for t in tasks for s in t['segments']),segments=len(records),
        maximum_gravity_fit_error_nm=max(r['sampled_gravity_fit_error_nm'] for r in records),
        torque_caps_nm=m.caps.tolist(),kp=m.kp.tolist(),kd=m.kd.tolist(),
        motor_lower_deg=np.rad2deg(lower).tolist(),motor_upper_deg=np.rad2deg(upper).tolist(),
        matched_ab_position_velocity_gain_timing_identical=not static_only,static_only=static_only)
    return job,info


if __name__=='__main__':
    ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('--capture',required=True);ap.add_argument('--profile',required=True)
    ap.add_argument('--circle-job',required=True);ap.add_argument('--output',required=True);ap.add_argument('--static-only',action='store_true');a=ap.parse_args()
    j,info=prepare(json.loads(Path(a.capture).read_text()),JointPlanner(a.profile),MotionRuntime(),json.loads(Path(a.circle_job).read_text()),a.static_only)
    out=Path(a.output);out.write_text(json.dumps(j,indent=2)+'\n');out.with_suffix('.review.json').write_text(json.dumps(info,indent=2)+'\n');print(json.dumps(info,indent=2))
