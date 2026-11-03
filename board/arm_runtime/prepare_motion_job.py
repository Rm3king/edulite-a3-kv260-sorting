#!/usr/bin/python3
"""Build concrete attended MoveJ/pause, line, circle, zero test from read-only capture."""
import argparse,json,math,sys
from pathlib import Path
import numpy as np
from motion_runtime import MotionRuntime,pose_values
from joint_trajectory import JointPlanner
from continuous_trajectory import CartesianPlanner,split,brake_at,cubic_words,extrema,smooth_cartesian_path
from motion_executor import verify_job,job_digest

def prepare(capture,p,circle_radius=.01,circle_only=False,gripper_target=.2,keep_gripper=False,smooth_c2=False,plane='XY'):
    if capture['status']!='PASS_READ_ONLY_ALL_7':raise ValueError('all seven motors must respond')
    reads=capture['transactions'];by={i:[x['decoded']['position_motor_rad'] for x in reads if x['motor']==i and x.get('decoded',{}).get('valid')] for i in range(1,8)}
    if any(not v or max(v)-min(v)>.002 for v in by.values()):raise ValueError('unstable read-only posture')
    motor=[by[i][-1] for i in range(1,8)];start=p.positions([(a-o)/d for a,o,d in zip(motor,p.offset,p.direction)])
    runtime=MotionRuntime();cart=CartesianPlanner(runtime,p)
    axes={'XY':(0,1),'XZ':(0,2),'YZ':(1,2)}[plane]
    anchor=np.deg2rad([-60,10,-15,15,20,-10]).tolist()+[float(gripper_target)]
    first=split(p,p.movej(start,anchor));brakes=[];resumes=[]
    if not circle_only:
        for part in first:
            b=brake_at(p,part);brakes.append(b);resumes.append(split(p,p.movej(b['goal_rad'],anchor)))
    if circle_only:
        tasks=[dict(name='MoveJ_approach',kind='MoveJ',segments=first,goal_rad=anchor)]
    else:
        tasks=[dict(name='MoveJ_pause_hold_resume',kind='MoveJ',segments=first,goal_rad=anchor,pause_after_s=.6,hold_seconds=1.5,brakes=brakes,resume_plans=resumes)]
        line=cart.path(anchor,'line',delta=(0,0,.02),tcp_speed=.025)
        for name,path in [('MoveL_up_20mm',line),('MoveL_down_20mm',cart.path(line['goal_rad'],'line',delta=(0,0,-.02),tcp_speed=.025))]:
            tasks.append(dict(name=name,kind='MoveL',segments=[x for s in path['segments'] for x in split(p,s)],goal_rad=path['goal_rad'],geometry={k:v for k,v in path.items() if k!='segments'}))
    circle=smooth_cartesian_path(runtime,p,tasks[-1]['goal_rad'],'circle',radius=circle_radius,axes=axes,tcp_speed=.025) if smooth_c2 else cart.path(tasks[-1]['goal_rad'],'circle',radius=circle_radius,axes=axes,tcp_speed=.025)
    tasks.append(dict(name=f'MoveC_{plane}_radius{circle_radius*1000:g}mm',kind='MoveC',segments=[x for s in circle['segments'] for x in split(p,s)],goal_rad=circle['goal_rad'],geometry={k:v for k,v in circle.items() if k!='segments'}))
    final_goal=[0.]*6+[anchor[6] if keep_gripper else 0.]
    zero=p.movej(circle['goal_rad'],final_goal)
    tasks.append(dict(name='MoveJ_arm_zero_gripper_hold' if keep_gripper else 'MoveJ_zero',kind='MoveJ',segments=split(p,zero),goal_rad=final_goal,final_zero=not keep_gripper,final_hold_s=2.))
    # Includes every permitted brake/resume branch, not only nominal waypoints.
    lo=[float('inf')]*7;hi=[-float('inf')]*7;count=0
    for task in tasks:
        sequences=[task['segments'],task.get('brakes',[])]+task.get('resume_plans',[])
        for seq in sequences:
            for part in seq:
                words,_,_,_=cubic_words(part,p);count+=1
                for i in range(7):
                    a,b=extrema(words[i*4:i*4+4]);lo[i]=min(lo[i],a/(1<<24));hi[i]=max(hi[i],b/(1<<24))
    # Limit to SDK range, add a local follow envelope for feedback and LOC_REF.
    lower=[];upper=[]
    for i in range(7):
        a,b=sorted((p.lower[i]*p.direction[i]+p.offset[i],p.upper[i]*p.direction[i]+p.offset[i]))
        lower.append(max(a,lo[i]-.02));upper.append(min(b,hi[i]+.02))
    job=dict(schema='kv260.reviewed_motion_job.v1',hardware_start_verified=not capture.get('synthetic_for_planning_only',False),profile_id=p.profile_id,model_id=runtime.model_id,start_motor_rad=motor,start_rad=start,tasks=tasks,motor_lower_rad=lower,motor_upper_rad=upper)
    verify_job(job,p)
    info=dict(job_sha256=job_digest(job),model_id=runtime.model_id,profile_id=p.profile_id,anchor_robot_deg=np.rad2deg(anchor[:6]).tolist(),anchor_pose=pose_values(runtime.fk(anchor[:6])),
      nominal_seconds=sum(sum(x['duration_s'] for x in t['segments']) for t in tasks),validated_cubics=count,
      robot_lower_deg=[math.degrees(min((a-o)/d,(b-o)/d)) for a,b,o,d in zip(lo,hi,p.offset,p.direction)],
      robot_upper_deg=[math.degrees(max((a-o)/d,(b-o)/d)) for a,b,o,d in zip(lo,hi,p.offset,p.direction)],
      path_audits={t['name']:t['geometry'] for t in tasks if 'geometry' in t})
    return job,info

if __name__=='__main__':
    ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('--capture',required=True);ap.add_argument('--profile',required=True);ap.add_argument('--output',required=True);ap.add_argument('--circle-radius',type=float,default=.01);ap.add_argument('--circle-only',action='store_true');ap.add_argument('--gripper-target',type=float,default=.2);ap.add_argument('--keep-gripper',action='store_true');ap.add_argument('--smooth-c2',action='store_true');ap.add_argument('--plane',choices=['XY','XZ','YZ'],default='XY');a=ap.parse_args()
    job,info=prepare(json.loads(Path(a.capture).read_text()),JointPlanner(a.profile),a.circle_radius,a.circle_only,a.gripper_target,a.keep_gripper,a.smooth_c2,a.plane)
    out=Path(a.output);out.write_text(json.dumps(job,indent=2)+'\n');out.with_suffix('.review.json').write_text(json.dumps(info,indent=2)+'\n')
    print(json.dumps({k:v for k,v in info.items() if k!='path_audits'},indent=2))
