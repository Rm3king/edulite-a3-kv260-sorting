"""Offline bounded image-response probe above selected nuts; no grasp commands."""
import json
from pathlib import Path
import numpy as np
from joint_trajectory import JointPlanner,Q24
from live_motion import current_reference
from motion_runtime import MotionRuntime,pose_values,pin
from septic_trajectory import smooth_cartesian_c3,movej_c3,septic_words,extrema,derivative
from septic_job import SCHEMA
from continuous_transfer import path_digest
from wrist_handeye_motion import POLICY


def plan(out,config):
    out=Path(out);p=JointPlanner(out/'profile.json');kin=MotionRuntime();cap=json.loads((out/'board_capture.json').read_text())
    values=[[x['decoded']['position_motor_rad'] for x in cap['transactions'] if x['motor']==i and x.get('decoded',{}).get('valid')] for i in range(1,8)]
    if any(len(v)!=3 or max(v)-min(v)>.002 for v in values):raise ValueError('Unstable actual joints')
    actual=[v[-1] for v in values];start,adjustment=current_reference(p,actual)
    nuts=[o for o in json.loads((out/'target_geometry/surface_points.json').read_text())['objects'] if o['part_class']=='nut' and o['inside_support_fit_patch']]
    selected=config.get('probe_detection_indices')
    if selected is not None:
        if not 1<=len(selected)<=4 or len(set(selected))!=len(selected):raise ValueError('Invalid probe target selection')
        nuts=[next(o for o in nuts if o['detection_index']==i) for i in selected]
    elif len(nuts)!=1:raise ValueError('Exactly one visible nut required without explicit selection')
    obj=nuts[0];n=np.array(obj['support_normal_base']);a=np.array(next(o for o in config['reference_objects'] if o['part_class']=='screw' and o['inside_support_fit_patch'])['axis_base'])
    R=np.column_stack([-np.cross(n,a),a,n]);shift=np.array([.00863568637816367,-.007186523132585276,-.001232162508185381])
    correction=np.array(config.get('probe_correction_base_m',[0.,0.,0.]))
    if correction.shape!=(3,) or not np.isfinite(correction).all() or np.linalg.norm(correction)>.020 or abs(correction@n)>1e-5:
        raise ValueError('Probe correction must stay within 20mm on the support plane')
    height=float(config.get('probe_end_height_above_surface_m',.178))
    if not .145<=height<=.178:raise ValueError('Noncontact observation height outside reviewed range')
    q=list(start);tasks=[];returns={};views=[]
    specs=[(nut,offset) for k,nut in enumerate(nuts) for offset in
           ([[0.,0.]] if config.get('probe_verify_only') or k else [[0.,0.],[.004,0.],[0.,.004],[0.,0.]])]
    for i,(obj,offset) in enumerate(specs):
        local_correction=np.array(config.get('probe_target_corrections',{}).get(str(obj['detection_index']),correction))
        if (local_correction.shape!=(3,) or not np.isfinite(local_correction).all()
            or np.linalg.norm(local_correction)>.025 or abs(local_correction@n)>1e-5):raise ValueError('Invalid local target correction')
        end=np.array(obj['base_surface_xyz_m'])+shift+height*n+local_correction
        point=end+R[:,:2]@np.array(offset);solutions=[]
        for guess in [q[:6]]+config['ik_seeds']+[config['ik_seed']]:
            try:solutions.append(kin.ik(pose_values(pin.SE3(R,point)),np.array(guess)).tolist()+[start[6]])
            except ValueError:pass
        if not solutions:raise ValueError('No probe IK')
        goal=min(solutions,key=lambda v:np.linalg.norm(np.array(v[:6])-q[:6]));name=f'pose_{i:02d}'
        high=None
        if height<.178:
            high=kin.ik(pose_values(pin.SE3(R,point+(.178-height)*n)),np.array(goal[:6])).tolist()+[start[6]]
            descend=smooth_cartesian_c3(kin,p,high,kind='line',delta=-(.178-height)*n,tcp_speed=.035)
            goal=descend['segments'][-1]['goal_rad']
            approach=movej_c3(p,q,high,1.0)+descend['segments']
        else:approach=movej_c3(p,q,goal,1.0)
        tasks.append(dict(name=name,capture=True,split='image_response_probe',goal_rad=goal,segments=approach))
        route=([goal,high,start] if high is not None else ([goal,tasks[0]['goal_rad'],start] if i else [goal,start]));segments=[]
        for qa,qb in zip(route,route[1:]):segments.extend(movej_c3(p,qa,qb,1.0))
        returns[name]=dict(name=name+'_return',start_rad=goal,goal_rad=start,return_to_live_start=True,segments=segments)
        views.append(dict(name=name,offset_tool_xy_m=offset,source=obj,correction_base_m=local_correction.tolist(),T_base_end=kin.fk(goal[:6]).homogeneous.tolist()));q=goal
    tasks.append(dict(name='return_supported',capture=False,goal_rad=start,return_to_live_start=True,segments=returns[tasks[-1]['name']]['segments']))
    job=dict(schema=SCHEMA,policy=POLICY,profile_id=p.profile_id,model_id=kin.model_id,motion_mask=127,start_rad=start,start_motor_rad=actual,start_reference_adjustment_motor_rad=adjustment,startup_no_reset=True,hardware_start_verified=True,tasks=tasks,recoveries=returns,geometry_audit=dict(status='PENDING'),scope='Explicitly selected nut noncontact observations; fixed jaw, no grasp')
    lo=np.full(7,np.inf);hi=np.full(7,-np.inf);rows=[]
    for branch,seq in [('main',tasks)]+[(name,[b]) for name,b in returns.items()]:
        for task in seq:
            for s in task['segments']:
                words=septic_words(s,p)[0]
                for k in range(7):
                    low,high=extrema(words[8*k:8*k+8]);lo[k]=min(lo[k],low/Q24);hi[k]=max(hi[k],high/Q24)
                coefficients=np.array(words[:56]).reshape(7,8)/Q24;coefficients*=np.array(p.direction)[:,None];coefficients[:,0]-=np.array(p.offset)*p.direction
                for u in np.linspace(0,1,max(3,int(np.ceil(s['duration_s']/.05))+1)):
                    qr=[derivative(row,float(u)) for row in coefficients];rows.append(dict(branch=branch,name=task['name'],q_rad=qr,T_base_end=kin.fk(qr[:6]).homogeneous.tolist()))
    job['motor_lower_rad']=[];job['motor_upper_rad']=[]
    for k in range(7):
        low,high=sorted((p.lower[k]*p.direction[k]+p.offset[k],p.upper[k]*p.direction[k]+p.offset[k]));job['motor_lower_rad'].append(max(low,float(lo[k])-.02));job['motor_upper_rad'].append(min(high,float(hi[k])+.02))
    (out/'job_pending_geometry.json').write_text(json.dumps(job,indent=2));(out/'collision_samples.json').write_text(json.dumps(dict(start_rad=start,trajectory_sha256=path_digest(job),samples=rows)))
    (out/'probe_plan.json').write_text(json.dumps(dict(views=views,source=obj,rotation=R.tolist(),legacy_surface_shift=shift.tolist(),correction_base_m=correction.tolist(),nominal_tip_clearance_from_ruler_m=height-.1335,scope='Provisional geometry; no descent'),indent=2))
    print(json.dumps(dict(views=len(views),seconds=sum(s['duration_s'] for t in tasks for s in t['segments']),samples=len(rows))))
