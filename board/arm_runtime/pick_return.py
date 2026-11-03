#!/usr/bin/python3
"""C3 fixed-point empty pick, lift, return and release; no vision dependency."""
import argparse,json,math
from pathlib import Path
import numpy as np
from joint_trajectory import JointPlanner
from motion_runtime import MotionRuntime,pose_values
from septic_trajectory import movej_c3,smooth_cartesian_c3
from septic_job import SCHEMA,verify_septic_job
from motion_executor import job_digest
from live_motion import POLICY,current_reference


def prepare(capture,p,runtime,grasp_z=.13,approach_height=.05,tcp_speed=.025):
    if capture['status']!='PASS_READ_ONLY_ALL_7':raise ValueError('all seven motors must respond')
    by={i:[x['decoded']['position_motor_rad'] for x in capture['transactions'] if x['motor']==i and x.get('decoded',{}).get('valid')] for i in range(1,8)}
    if any(not v for v in by.values()):raise ValueError('missing joint feedback')
    motor=[by[i][-1] for i in range(1,8)]
    start,adjustment=current_reference(p,motor)
    # Base -X is nominal forward at zero; +Y is right when looking along -X.
    # Tool extends along local -Z according to the fixed TCP joint in the URDF.
    if not all(math.isfinite(x) for x in [grasp_z,approach_height,tcp_speed]) or not .05<=grasp_z<=.3 or not .01<=approach_height<=.1:
        raise ValueError('finite grasp height and 10..100mm approach required')
    target=[-.17,.12,float(grasp_z),0.,0.,math.pi/4]
    seed=np.deg2rad([-35,91,-67,66,0,-10]).tolist()
    grasp=runtime.ik(target,seed).tolist()+[.35]
    pre_pose=target.copy();pre_pose[2]+=approach_height
    pre=runtime.ik(pre_pose,grasp[:6]).tolist()+[.35]
    down=smooth_cartesian_c3(runtime,p,pre,kind='line',delta=(0,0,-approach_height),tcp_speed=tcp_speed)
    closed=down['goal_rad'][:6]+[1.6]
    up=smooth_cartesian_c3(runtime,p,closed,kind='line',delta=(0,0,approach_height),tcp_speed=tcp_speed)
    home=start[:6]+[1.6];released=start[:6]+[.35]
    tasks=[dict(name='Approach_right_front_open',kind='MoveJ',segments=movej_c3(p,start,pre),goal_rad=pre),
           dict(name=f'Descend_{approach_height*1000:g}mm',kind='MoveL',segments=down['segments'],goal_rad=down['goal_rad'],geometry={k:v for k,v in down.items() if k!='segments'}),
           dict(name='Close_empty_gripper',kind='Gripper',segments=movej_c3(p,down['goal_rad'],closed),goal_rad=closed),
           dict(name=f'Lift_{approach_height*1000:g}mm',kind='MoveL',segments=up['segments'],goal_rad=up['goal_rad'],geometry={k:v for k,v in up.items() if k!='segments'}),
           dict(name='Return_to_current_start',kind='MoveJ',segments=movej_c3(p,up['goal_rad'],home),goal_rad=home,return_to_live_start=True),
           dict(name='Release_at_start',kind='Gripper',segments=movej_c3(p,home,released),goal_rad=released,gripper_only=True,final_hold_s=1.)]
    # Current entry is rebound at enable, so guards cover calibrated legal
    # travel. This is not a collision-clearance assertion.
    bounds=[sorted((a*d+o,b*d+o)) for a,b,d,o in zip(p.lower,p.upper,p.direction,p.offset)]
    job=dict(schema=SCHEMA,start_policy=POLICY,hardware_start_verified=not capture.get('synthetic_for_planning_only',False),profile_id=p.profile_id,model_id=runtime.model_id,start_rad=start,start_motor_rad=motor,
             start_reference_adjustment_motor_rad=adjustment,tasks=tasks,motor_lower_rad=[b[0] for b in bounds],motor_upper_rad=[b[1] for b in bounds])
    verify_septic_job(job,p)
    points=[];joints=[]
    from septic_trajectory import derivative
    for t in tasks:
        for part in t['segments']:
            for u in (0.,.25,.5,.75,1.):
                q=[derivative(row,u) for row in part['coefficients_rad']]
                # Remove floating endpoint noise after analytic checks.
                q=[max(a,min(b,x)) for x,a,b in zip(q,p.lower,p.upper)]
                points.append(runtime.fk(q[:6]).translation.tolist());joints.append(q)
    q=np.array(joints);xyz=np.array(points)
    review=dict(job_sha256=job_digest(job),task_names=[t['name'] for t in tasks],grasp_pose=target,pregrasp_pose=pre_pose,
                nominal_seconds=sum(s['duration_s'] for t in tasks for s in t['segments']),segments=sum(len(t['segments']) for t in tasks),
                robot_lower_deg=np.rad2deg(q[:,:6].min(axis=0)).tolist(),robot_upper_deg=np.rad2deg(q[:,:6].max(axis=0)).tolist(),
                tcp_sample_lower_m=xyz.min(axis=0).tolist(),tcp_sample_upper_m=xyz.max(axis=0).tolist(),
                start_pose=pose_values(runtime.fk(start[:6])),start_policy=POLICY,task_seconds={t['name']:sum(s['duration_s'] for s in t['segments']) for t in tasks},
                path_audits={t['name']:t['geometry'] for t in tasks if 'geometry'in t},physical_table_tcp_calibrated=False,
                scope='EMPTY_AIR_PICK_NOT_REAL_SCREW_GRASP',collision_clearance_verified=False)
    return job,review

if __name__=='__main__':
    a=argparse.ArgumentParser(description=__doc__);a.add_argument('--capture',required=True);a.add_argument('--profile',required=True);a.add_argument('--output',required=True)
    a.add_argument('--grasp-z',type=float,default=.13);a.add_argument('--approach-height',type=float,default=.05);a.add_argument('--tcp-speed',type=float,default=.025);x=a.parse_args()
    job,review=prepare(json.loads(Path(x.capture).read_text()),JointPlanner(x.profile),MotionRuntime(),x.grasp_z,x.approach_height,x.tcp_speed)
    Path(x.output).write_text(json.dumps(job,indent=2)+'\n');Path(x.output).with_suffix('.review.json').write_text(json.dumps(review,indent=2)+'\n')
    print(json.dumps({k:v for k,v in review.items() if k!='path_audits'},indent=2))
