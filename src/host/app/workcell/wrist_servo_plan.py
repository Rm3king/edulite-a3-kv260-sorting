"""Offline calculation of one fixed-height visual step and its supported return."""
import json,sys
from pathlib import Path
import numpy as np
from joint_trajectory import JointPlanner,Q24
from motion_runtime import MotionRuntime,pose_values,pin
from septic_trajectory import movej_c3,movej_cruise_c3,septic_words,derivative
from septic_job import SCHEMA,verify_septic_job
from continuous_transfer import path_digest


def plan(root,request,delta,out):
    root=Path(root);out=Path(out);out.mkdir(parents=True,exist_ok=False)
    episode=json.loads((root/'job.json').read_text());view=json.loads((root/'probe_plan.json').read_text())['views'][0]
    R=np.array(view['T_base_end'])[:3,:3];origin=np.array(view['T_base_end'])[:3,3]
    p=JointPlanner(root/'profile.json');kin=MotionRuntime();q=request['q_rad'];home=episode.get('chain',{}).get('support_rad',episode['start_rad'])
    config=json.loads((root/'workflow_config.json').read_text());joint_move=movej_cruise_c3 if config.get('joint_time_law')=='cruise_c3' else movej_c3
    point=origin+R[:,:2]@np.array(delta)
    goal=kin.ik(pose_values(pin.SE3(R,point)),np.array(q[:6])).tolist()+[home[6]]
    if max(abs(a-b) for a,b in zip(goal[:6],q[:6]))>.20:raise ValueError('IK branch jump')
    move=dict(name='visual_xy',start_rad=q,goal_rad=goal,segments=joint_move(p,q,goal,.5))
    hover=episode['tasks'][0]['goal_rad']
    segments=joint_move(p,goal,hover,.5)+episode['recoveries'][episode['tasks'][0]['name']]['segments']
    branch=dict(name='return_supported',start_rad=goal,goal_rad=home,return_to_live_start=True,segments=segments)
    candidate=dict(schema=SCHEMA,profile_id=p.profile_id,start_rad=q,tasks=[move,branch],geometry_audit={'status':'PENDING'},start_motor_rad=episode['start_motor_rad'])
    # Verify exact C3 encoding before sampled CAD review. Board child repeats this.
    verify_septic_job(candidate,p)
    rows=[dict(branch='initial',name='supported_start',q_rad=home,T_base_end=kin.fk(home[:6]).homogeneous.tolist())]
    for t in candidate['tasks']:
        for s in t['segments']:
            words=septic_words(s,p)[0];coeff=np.array(words[:56]).reshape(7,8)/Q24
            coeff*=np.array(p.direction)[:,None];coeff[:,0]-=np.array(p.offset)*p.direction
            for u in np.linspace(0,1,max(3,int(np.ceil(s['duration_s']/.05))+1)):
                qr=[derivative(row,float(u)) for row in coeff]
                rows.append(dict(branch='online',name=t['name'],q_rad=qr,T_base_end=kin.fk(qr[:6]).homogeneous.tolist()))
    (out/'job_pending_geometry.json').write_text(json.dumps(candidate))
    (out/'collision_samples.json').write_text(json.dumps(dict(start_rad=home,trajectory_sha256=path_digest(candidate),samples=rows)))
    return out



def plan_grasp(root,request,out):
    """Replan the remaining grasp at this actual commanded hover, without reset."""
    import shutil,hashlib
    from .live_planner import prepare
    root=Path(root);out=Path(out);out.mkdir(parents=True,exist_ok=False)
    for name in ('profile.json','board_capture.json','scene_review.json','destinations.json','input.json'):
        shutil.copy2(root/name,out/name)
    shutil.copytree(root/'target_geometry',out/'target_geometry')
    episode=json.loads((root/'job.json').read_text());probe=json.loads((root/'probe_plan.json').read_text())
    source=episode['targets'][0]['source'];delta=np.array(probe['rotation'])[:,:2]@(np.array(request['cumulative_tool_xy_m'])+np.array(episode.get('wrist_hover_bias_tool_xy_m',[0.,0.])))
    evidence=dict(status='HOVER_CORRECTION_OBSERVED_CANDIDATE_REFERENCE_ONLY',online_closed_loop=True,
        delta_base_m=delta.tolist(),source_base_surface_xyz_m=source['base_surface_xyz_m'],
        pixel_error_after=request['pixel_error'],pixel_error_basis=request.get('pixel_error_basis','MEASURED'),scope='Same-owner live image correction; original grasp height')
    evidencefile=out/'online_evidence.json';evidencefile.write_text(json.dumps(evidence))
    config=json.loads((root/'workflow_config.json').read_text());config['online_hover_q_rad']=request['q_rad'];config['ik_seed']=request['q_rad'][:6]
    config['wrist_nut_trial']=dict(evidence_path=str(evidencefile),evidence_sha256=hashlib.sha256(evidencefile.read_bytes()).hexdigest())
    (out/'workflow_config.json').write_text(json.dumps(config))
    prepare(out,config,continuation=dict(executed_prefix=request['executed_prefix'],
        motor_lower_rad=episode['motor_lower_rad'],motor_upper_rad=episode['motor_upper_rad'],
        destination_q_rad=next(t['goal_rad'] for t in episode['tasks'] if t['name'].endswith('_transfer')),
        destination_rotation=episode['targets'][0]['destination_rotation']))



if __name__=='__main__':
    root,requestfile,out=sys.argv[1:4];data=json.loads(Path(requestfile).read_text())
    if len(sys.argv)>4 and sys.argv[4]=='--grasp':plan_grasp(root,data['request'],out)
    else:plan(root,data['request'],data['delta'],out)
