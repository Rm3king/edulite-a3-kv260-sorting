#!/usr/bin/python3
"""Immutable reviewed C3 jobs; reject unsupported pause branches before I/O."""
import json,math
import numpy as np
from septic_trajectory import movej_c3,smooth_cartesian_c3,septic_words,extrema,boundary,continuity_metrics
from septic_transport import SepticTransport
from joint_trajectory import Q24

SCHEMA='kv260.reviewed_septic_job.v1'


def verify_septic_job(job,p,return_prepared=False):
    if job.get('schema')!=SCHEMA or job['profile_id']!=p.profile_id:raise ValueError('C3 job/profile mismatch')
    if not job['tasks']:raise ValueError('empty C3 job')
    tail=p.positions(job['start_rad'])
    class Sink:
        def read(self,a):return {0x470:0x50001,0x828:0x70008,0x404:7,0x800:1}.get(a,0)
        def write(self,a,v):pass
    transport=SepticTransport(Sink(),p)
    for task in job['tasks']:
        if task.get('pause_after_s') is not None or task.get('brakes') or task.get('resume_plans'):
            raise ValueError('C3 braking/resume not commissioned; no hardware opened')
        plans=task['segments']
        if not plans or max(abs(a-b) for a,b in zip(plans[0]['start_rad'],tail))>2e-6:raise ValueError('C3 task position discontinuity')
        previous=None
        for plan in plans:
            # transport.submit below validates the original and encoded forms;
            # repeating the same coefficient checks here adds no coverage.
            if max(abs(a-b) for a,b in zip(boundary(plan),plan['start_rad']))>1e-10 or max(abs(a-b) for a,b in zip(boundary(plan,end=True),plan['goal_rad']))>1e-10:
                raise ValueError('C3 polynomial/endpoint metadata mismatch')
            begin=np.array([boundary(plan,k) for k in range(4)])
            end=np.array([boundary(plan,k,True) for k in range(4)])
            if previous is not None and np.max(np.abs(begin-previous))>1e-7:raise ValueError('reference is not C3')
            if previous is None and np.max(np.abs(begin[1:]))>1e-7:raise ValueError('C3 task must start at rest through jerk')
            transport.submit(plan);previous=end
        if np.max(np.abs(previous[1:]))>1e-7:raise ValueError('C3 task must end at rest through jerk')
        if max(abs(a-b) for a,b in zip(plans[-1]['goal_rad'],task['goal_rad']))>2e-6:raise ValueError('C3 task goal mismatch')
        tail=task['goal_rad']
    return transport.records if return_prepared else True


def prepare(capture,p,runtime,plane='XZ',radius=.04,gripper_target=1.6):
    from motion_executor import job_digest
    from motion_runtime import pose_values
    if capture['status']!='PASS_READ_ONLY_ALL_7':raise ValueError('all seven motors must respond')
    by={mid:[x['decoded']['position_motor_rad'] for x in capture['transactions'] if x['motor']==mid and x.get('decoded',{}).get('valid')] for mid in range(1,8)}
    if any(len(v)!=3 or max(v)-min(v)>.002 for v in by.values()):raise ValueError('stable 21-read capture required')
    motor=[by[i][-1] for i in range(1,8)];start=p.positions([(a-o)/d for a,o,d in zip(motor,p.offset,p.direction)])
    anchor=np.deg2rad([-60,10,-15,15,20,-10]).tolist()+[gripper_target]
    path=smooth_cartesian_c3(runtime,p,anchor,radius=radius,axes={'XY':(0,1),'XZ':(0,2),'YZ':(1,2)}[plane])
    final=[0.]*6+[gripper_target]
    tasks=[dict(name='C3_MoveJ_approach',kind='MoveJ',segments=movej_c3(p,start,anchor),goal_rad=anchor),
        dict(name=f'C3_MoveC_{plane}_diameter{radius*2000:g}mm',kind='MoveC',segments=path['segments'],goal_rad=path['goal_rad'],geometry={k:v for k,v in path.items() if k!='segments'}),
        dict(name='C3_MoveJ_arm_zero_gripper_hold',kind='MoveJ',segments=movej_c3(p,path['goal_rad'],final),goal_rad=final,final_hold_s=2.)]
    lo=np.full(7,np.inf);hi=-lo.copy()
    for task in tasks:
        for part in task['segments']:
            w,_,_,_=septic_words(part,p)
            for i in range(7):
                a,b=extrema(w[i*8:i*8+8]);lo[i]=min(lo[i],a/Q24);hi[i]=max(hi[i],b/Q24)
    lower=[];upper=[]
    for i in range(7):
        a,b=sorted((p.lower[i]*p.direction[i]+p.offset[i],p.upper[i]*p.direction[i]+p.offset[i]))
        lower.append(max(a,float(lo[i])-.02));upper.append(min(b,float(hi[i])+.02))
    job=dict(schema=SCHEMA,hardware_start_verified=not capture.get('synthetic_for_planning_only',False),profile_id=p.profile_id,model_id=runtime.model_id,
        start_rad=start,start_motor_rad=motor,tasks=tasks,motor_lower_rad=lower,motor_upper_rad=upper)
    verify_septic_job(job,p)
    info=dict(job_sha256=job_digest(job),model_id=runtime.model_id,profile_id=p.profile_id,anchor_pose=pose_values(runtime.fk(anchor[:6])),
        nominal_seconds=sum(s['duration_s'] for t in tasks for s in t['segments']),segments=sum(len(t['segments']) for t in tasks),
        robot_lower_deg=[math.degrees(min((a-o)/d,(b-o)/d)) for a,b,o,d in zip(lo,hi,p.offset,p.direction)],
        robot_upper_deg=[math.degrees(max((a-o)/d,(b-o)/d)) for a,b,o,d in zip(lo,hi,p.offset,p.direction)],
        path_audit={k:v for k,v in path.items() if k!='segments'},
        task_continuity={t['name']:{'reference':continuity_metrics(t['segments'],p),'quantized':continuity_metrics(t['segments'],p,True)} for t in tasks})
    return job,info


if __name__=='__main__':
    import argparse
    from pathlib import Path
    from joint_trajectory import JointPlanner
    from motion_runtime import MotionRuntime
    ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('--capture',required=True);ap.add_argument('--profile',required=True);ap.add_argument('--output',required=True)
    ap.add_argument('--plane',choices=['XY','XZ','YZ'],default='XZ');a=ap.parse_args()
    job,info=prepare(json.loads(Path(a.capture).read_text()),JointPlanner(a.profile),MotionRuntime(),a.plane)
    out=Path(a.output);out.write_text(json.dumps(job,indent=2)+'\n');out.with_suffix('.review.json').write_text(json.dumps(info,indent=2)+'\n')
    print(json.dumps({k:v for k,v in info.items() if k not in ('path_audit','task_continuity')},indent=2))
