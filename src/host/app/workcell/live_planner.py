"""Reusable attended planner; no camera, network or motor I/O."""
import json,numpy as np
from pathlib import Path
from joint_trajectory import JointPlanner,Q24
from live_motion import current_reference
from motion_runtime import MotionRuntime,pose_values,pin
from septic_trajectory import movej_c3,smooth_cartesian_c3,septic_words,derivative,extrema,septic
from septic_job import SCHEMA,verify_septic_job
from continuous_transfer import POLICY,path_digest,grasp_raise_m,expected_grasp_height_offset_m
import copy
from direct_departure import direct_departure

def close_command(part_class, detected_class):
    """Jaw close command (rad) for a part. The M6 nut (~10 mm across flats) is about the fingertip gap at 1.60: grasps read 1.574-1.586
    (barely touching) and two of five closed on nothing (1.600); the M6 screw is gripped by its ~10 mm head and showed the same:
    held readings 1.572 and 1.5997, indistinguishable from an empty close at 1.60 (2026-11-01 run 021535: a held screw was returned as
    an empty closure). Both close to 1.66, inside the commissioned fully-closed 1.678: a held part stalls the jaws short of the command
    (the 0.65 A current limit, not the position, sets the squeeze), an empty close runs on to 1.66 and is still refused at 1.64.
    Every other part keeps 1.60. (M6 nut approved 2026-10-30; M6 screw 2026-11-01.)"""
    return 1.66 if (part_class, detected_class) in (('nut', 'nut_m6'), ('screw', 'screw_m6')) else 1.6


def prepare(out, config, runtime=None, continuation=None):
    out=Path(out);p=JointPlanner(out/'profile.json');kin=runtime or MotionRuntime()
    cruise=config.get('joint_time_law')=='cruise_c3'
    # The first leg out of the support pose carries J3 into the extended pose with a cold motor: at full speed it reached the
    # 4.5 A limit, lagged 53 mrad and the catch-up tripped the velocity monitor (run 40); the way back from the extended exit pose did
    # the same on J2 (5.4 A of 5.5 A, 35 mrad, run 45). The long free legs get slower caps; grasp, lift and transfer keep theirs.
    scale=config.get('departure_speed_scale',.8);slow=copy.copy(p);slow.velocity=[v*scale for v in p.velocity];slow.acceleration=[x*scale**2 for x in p.acceleration]
    from septic_trajectory import movej_cruise_c3
    joint_move=movej_cruise_c3 if cruise else movej_c3
    cap=json.loads((out/'board_capture.json').read_text());by={i:[x['decoded']['position_motor_rad'] for x in cap['transactions'] if x['motor']==i and x['decoded']['valid']] for i in range(1,8)};assert all(len(v)==3 and max(v)-min(v)<.002 for v in by.values());actual=[by[i][-1] for i in range(1,8)];start,adjustment=current_reference(p,actual)
    # Continuous multi-nut session: later nuts start at the previous exit pose and every
    # recovery still returns to the supported pose (`chain`, see continuous_transfer.chain_of).
    chain=config.get('chain');chain_index=chain['index'] if chain else 0;chain_count=chain['count'] if chain else 1
    if chain_index>0:
        start=list(chain['start_rad']);actual=p.to_motor(start)
    if abs(start[6]-1.12)<.005:start[6]=1.12   # the working-open restore settles within 5 mrad
    assert 0.<=start[6]<=1.70;adjustment=[a-b for a,b in zip(p.to_motor(start),actual)];home=list(chain['support_rad']) if chain_index>0 else start[:6]+[1.12]
    if chain_index>0 and (abs(start[6]-1.12)>1e-9 or abs(home[6]-1.12)>1e-9):raise ValueError('Chain jobs start and return working-open')
    accepted=set(json.loads((out/'scene_review.json').read_text())['accepted_detection_indices']);objects=[o for o in json.loads((out/'target_geometry/surface_points.json').read_text())['objects'] if (o.get('inside_support_fit_patch') or o['detection_index'] in json.loads((out/'scene_review.json').read_text())['allow_plane_extrapolation_indices']) and o['detection_index'] in accepted];assert 1<=len(objects)<=5 and all(o['part_class'] in ('screw','nut') for o in objects);objects.sort(key=lambda o:(o['part_class']!='screw',o['detection_index']))
    n=np.array(objects[0]['support_normal_base']);shift=np.array([.00863568637816367,-.007186523132585276,-.001232162508185381]);tool=np.array([0,0,-.098]);reference=config['reference_objects'];axis=np.array(next(o for o in reference if o['part_class']=='screw' and o['inside_support_fit_patch'])['axis_base']);Rref=np.column_stack([-np.cross(n,axis),axis,n]);Rd=Rref@np.array([[0.,-1,0],[1,0,0],[0,0,1]])
    trial=config.get('wrist_nut_trial');correction=np.zeros(3)
    # Measured systematic offset of the predicted wrist hover (tool-frame x,y metres); an online
    # continuation already carries it inside the observed correction, so it only shapes the first hover.
    bias=np.array(config.get('wrist_hover_bias_tool_xy_m',[0.,0.]),float)
    if bias.shape!=(2,) or not np.isfinite(bias).all() or np.linalg.norm(bias)>.025:raise ValueError('Invalid wrist hover bias')
    if trial is not None:
     import hashlib
     evidence=Path(trial['evidence_path'])
     if hashlib.sha256(evidence.read_bytes()).hexdigest()!=trial['evidence_sha256']:raise ValueError('Wrist evidence changed')
     reviewed=json.loads(evidence.read_text())
     correction=np.array(reviewed['delta_base_m'],float)
     allowed=reviewed.get('status')=='HOVER_CORRECTION_OBSERVED_CANDIDATE_REFERENCE_ONLY' and reviewed['pixel_error_after']<=5.
     # A separately recorded attended attempt may use an observed improvement
     # without claiming convergence or accepted hand-eye/TCP accuracy.
     partial=(reviewed.get('status')=='HOVER_CORRECTION_IMPROVED_NOT_CONVERGED'
              and reviewed.get('attended_trial_reviewed') is True
              and reviewed['pixel_error_after']<=10.
              and reviewed['pixel_error_after']<=.2*reviewed['pixel_error_before'])
     online=(reviewed.get('online_closed_loop') is True and allowed)
     if trial.get('raised_grasp_trial'):raise ValueError('Raised-height trial withdrawn by user; retain original grasp height')
     if (len(objects)!=1 or objects[0]['part_class'] not in ('nut','screw') or not (allowed or partial)
         or correction.shape!=(3,) or not np.isfinite(correction).all()
         or np.linalg.norm(correction)>(.035 if online else .020) or abs(correction@n)>1e-5
         or np.linalg.norm(np.array(objects[0]['base_surface_xyz_m'])-reviewed['source_base_surface_xyz_m'])>.003):
      raise ValueError('Wrist single-nut correction outside reviewed local trial')
    floor=config.get('destination_floor',config['left_floor']);K=json.loads(Path(config['workplane']).read_text())['intrinsics'];T=np.array(json.loads(Path(config['calibration']).read_text())['T_base_camera'])
    def floor_at(uv):
     ray=np.array([(uv[0]-K['cx'])/K['fx'],(uv[1]-K['cy'])/K['fy'],1.]);pc=-floor['offset_m']*ray/np.dot(floor['normal_camera'],ray);return T[:3,:3]@pc+T[:3,3]
    Rd=np.array(json.loads((out/'destinations.json').read_text())['rotation']);dest_n=Rd[:,2]
    seed=config['ik_seed']
    ik_seeds=config['ik_seeds']
    def ik(tcp,rot,seed,jaw):
     for guess in [seed[:6]]+ik_seeds:
      try:return kin.ik(pose_values(pin.SE3(rot,tcp-rot@tool)),np.array(guess)).tolist()+[jaw]
      except ValueError:pass
     raise ValueError('No IK solution from current and previously executed placement seeds')
    def reverse(seq,jaw):
     result=[];transform=np.polynomial.Polynomial([1.,-1.])
     for s in seq[::-1]:
      rows=[]
      for row in s['coefficients_rad'][:6]:
       c=np.polynomial.Polynomial(row)(transform).coef;rows.append(np.pad(c,(0,8-len(c))).tolist())
      rows.append([jaw]+[0.]*7);result.append(septic(p,rows,s['duration_s']))
     return result
    q=start;tasks=[];recoveries={};targets=[];info={};slots=json.loads((out/'destinations.json').read_text())['pixels'];counts={'screw':0,'nut':0}
    def task(name,seq):
     nonlocal q
     q=seq[-1]['goal_rad'];tasks.append(dict(name=name,segments=seq,goal_rad=q,return_to_live_start=name=='return_supported'));print(name,round(sum(s['duration_s'] for s in seq),3),flush=True)
    contact_speed=float(config.get('contact_tcp_speed_m_s',.05));retreat_speed=float(config.get('retreat_tcp_speed_m_s',.05))
    if not .01<=contact_speed<=.05 or not .01<=retreat_speed<=.10:raise ValueError('TCP line speed outside reviewed range')
    def line(name,delta):
     # Moving away from the part/tray (lift, exit) may run faster than moving toward it.
     # A lifted screw is gripped farther out than a nut: at 0.08 m/s J3 saturated its current (4.1 of 4.5 A), lagged 47 mrad and
     # then caught up at 0.64 rad/s, tripping the encoder-window velocity monitor (run34, 2026-10-26). Lift screws at the old 0.05.
     lift_speed=min(retreat_speed,float(config.get('screw_lift_tcp_speed_m_s',.05))) if name.startswith('screw') and name.endswith('_lift') else retreat_speed
     x=smooth_cartesian_c3(kin,p,q,kind='line',delta=delta,tcp_speed=lift_speed if name.endswith('_lift') else retreat_speed if name.endswith('_exit') else contact_speed);task(name,x['segments']);return x['segments']
    close=1.6
    # the surface-geometry objects keep only detection_index: the detector's class (nut_m6 ...) is in the snapshot this job used
    try:detected={o.get('detection_index'):o.get('detected_class') for o in json.loads(Path(json.loads((out/'input.json').read_text())['points']).read_text())['objects']}
    except (OSError,KeyError,ValueError):detected={}
    for i,obj in enumerate(objects):
     cls=obj['part_class'];tid=cls+'_'+str(counts[cls]);counts[cls]+=1;surface=np.array(obj['base_surface_xyz_m'])+shift+correction
     # The M6 nut (~10 mm across flats) is about the fingertip gap at 1.60: grasps read 1.574-1.586 (barely touching) and two
     # of five closed on nothing (1.600). Close it to 1.66, inside the commissioned fully-closed 1.678; the 0.65 A current
     # limit, not the position, sets the squeeze. Every other part keeps 1.60. (User approved, 2026-10-30.)
     close=close_command(cls,detected.get(obj.get('detection_index')))
     if cls=='screw':a=np.array(obj['axis_base']);Rs=np.column_stack([-np.cross(n,a),a,n])
     else:
      ang=np.deg2rad(0.);Rs=Rref@np.array([[np.cos(ang),-np.sin(ang),0],[np.sin(ang),np.cos(ang),0],[0,0,1.]])
      if trial is None:surface=surface+Rs[:,:2]@bias
     descent=.063-grasp_raise_m(o['part_class'] for o in objects)  # screws are gripped higher (continuous_transfer.SCREW_GRASP_RAISE_M)
     # The jaw is symmetric, so the grasp may be taken with either end of the closing axis. Keep the first orientation whose hover pose
     # AND vertical descent solve (a screw at ~12 deg yaw failed only on the descent line, 2026-10-28).
     for flipped in (False,True):
      Rc=Rs@np.diag([-1.,-1.,1.]) if flipped else Rs
      try:
       high=ik(surface+.08*n,Rc,seed,1.12)
       smooth_cartesian_c3(kin,p,high,kind='line',delta=-descent*n,tcp_speed=contact_speed)
       Rs=Rc;break
      except ValueError as error:last_error=error
     else:raise last_error
     if config.get('online_hover_q_rad') is not None:
      if len(objects)!=1 or trial is None:raise ValueError('Online continuation is one reviewed nut')
      observed=list(config['online_hover_q_rad']);tf=kin.fk(observed[:6])
      wanted=surface+.178*n
      if (len(observed)!=7 or abs(observed[6]-1.12)>1e-6 or np.linalg.norm(tf.translation-wanted)>.001
          or np.linalg.norm(tf.rotation-Rs)>.01):raise ValueError('Current online hover is not the corrected source pose')
      high=observed
     if i==0 and chain_index>0:
      seq,info['departure']=direct_departure(slow,q,high,cruise=cruise);task('departure',seq)
      back,info['departure_return']=direct_departure(slow,list(q),home,cruise=cruise)
      recoveries['departure']=dict(name='departure_return',start_rad=list(q),segments=back,goal_rad=home,return_to_live_start=True)
     elif i==0:
      seq,info['departure']=direct_departure(slow,q,high,open_during_departure=True,cruise=cruise,observed_closed_start=actual[6] if actual[6]>1.62 else None,observed_wide_start=actual[6] if actual[6]<1.08 else None);task('departure',seq)
      recoveries['departure']=dict(name='departure_return',start_rad=list(q),segments=reverse(seq,1.12),goal_rad=home,return_to_live_start=True)
     else:task(tid+'_transfer_empty',joint_move(p,q,high))
     approach=line(tid+'_approach',-descent*n);task(tid+'_close',joint_move(p,q,q[:6]+[close]));close_start=list(q)
     close_return=reverse(approach,close)+joint_move(p,high[:6]+[close],home[:6]+[close])
     recoveries[tid+'_close']=dict(name=tid+'_empty_return',start_rad=close_start,segments=close_return,goal_rad=home[:6]+[close],return_to_live_start=True)
     line(tid+'_lift',descent*n)
     destination=floor_at(slots[i])
     if continuation is not None and 'destination_q_rad' in continuation:
      # Fixed destination: keep the already reviewed placement endpoint exactly,
      # avoiding a new IK solution and rechecking identical downstream geometry.
      Rd=np.array(continuation['destination_rotation']);dh=continuation['destination_q_rad'][:]
      tf=kin.fk(dh[:6]);wanted=destination+shift+.12*dest_n-Rd@tool
      if np.linalg.norm(tf.translation-wanted)>1e-5 or np.linalg.norm(tf.rotation-Rd)>.0001 or abs(dh[6]-close)>1e-6:
       raise ValueError('Cached destination no longer matches this task')
     else:
      try:dh=ik(destination+shift+.12*dest_n,Rd,q,close)
      except ValueError:
       Rd=Rd@np.diag([-1.,-1.,1.]);dh=ik(destination+shift+.12*dest_n,Rd,q,close)
     task(tid+'_transfer',joint_move(p,q,dh));h=obj['surface_height_m']+.027
     line(tid+'_place',(h-.12)*dest_n);task(tid+'_open',joint_move(p,q,q[:6]+[1.12]));open_start=list(q);exitpath=line(tid+'_exit',(.12-h)*dest_n)
     open_return=exitpath+joint_move(p,q,home);recoveries[tid+'_open']=dict(name=tid+'_release_return',start_rad=open_start,segments=open_return,goal_rad=home,return_to_live_start=True)
     if chain_index<chain_count-1:
      # A later nut follows this exit; if it cannot proceed (heat, ...) the arm still has a reviewed way to support.
      back,info['exit_return']=direct_departure(slow,list(q),home,cruise=cruise);recoveries[tid+'_exit']=dict(name=tid+'_exit_return',start_rad=list(q),segments=back,goal_rad=home,return_to_live_start=True)
     targets.append(dict(target_id=tid,part_class=cls,source=obj,destination_uv=slots[i],destination_floor_base_m=destination.tolist(),source_rotation=Rs.tolist(),destination_rotation=Rd.tolist()))
    if chain_index==chain_count-1:
     seq,info['return']=direct_departure(slow,q,home,cruise=cruise);task('return_supported',seq);tasks[-1]['goal_rad']=home
    j=dict(schema=SCHEMA,policy=POLICY,profile_id=p.profile_id,model_id=kin.model_id,motion_mask=127,start_rad=start,start_motor_rad=actual,start_reference_adjustment_motor_rad=adjustment,open_rad=1.12,close_rad=close,grasp_height_offset_m=expected_grasp_height_offset_m(o['part_class'] for o in objects),gripper_open_current_a=p.current[6],gripper_close_current_a=p.current[6],automatic_task_failure_return=True,batch_return_temperature_c=75.,initial_open_during_departure=chain_index==0,cruise=cruise,initial_open_from_observed_closed=actual[6]>1.62,initial_normalize_from_observed_wide=actual[6]<1.08,targets=targets,recoveries=recoveries,local_compensation_base_m=shift.tolist(),hardware_start_verified=True,startup_no_reset=True,tasks=tasks,geometry_audit={'status':'PENDING'},departure_mode='SYNCHRONIZED_DIRECT_NO_SINGLE_JOINT_PREAMBLE',wrist_hover_bias_tool_xy_m=bias.tolist() if all(o['part_class']=='nut' for o in objects) else [0.,0.])
    if chain:j['chain']=dict(index=chain_index,count=chain_count,support_rad=list(home))
    if continuation is not None:
     if config.get('online_hover_q_rad') is None:raise ValueError('Continuation requires observed hover')
     tasks[0]['segments']=continuation['executed_prefix'];tasks[0]['goal_rad']=config['online_hover_q_rad']
    lo=np.full(7,np.inf);hi=-lo.copy();rows=[]
    if trial is not None:j['wrist_nut_trial']=dict(trial,applied_correction_base_m=correction.tolist(),z_grasp_policy='Existing empirical height retained; no new TCP length deployed')
    for branch,seq in [('main',tasks)]+[(name,[b]) for name,b in recoveries.items()]:
     for ti,t in enumerate(seq):
      for s in t['segments']:
       w=septic_words(s,p)[0]
       for k in range(7):
        a,b=extrema(w[8*k:8*k+8]);lo[k]=min(lo[k],a/Q24);hi[k]=max(hi[k],b/Q24)
       c=np.array(w[:56]).reshape(7,8)/Q24;c*=np.array(p.direction)[:,None];c[:,0]-=np.array(p.offset)*p.direction
       for u in np.linspace(0,1,max(3,int(np.ceil(s['duration_s']/.04))+1)):
        qr=[derivative(r,float(u)) for r in c];tf=kin.fk(qr[:6]);rows.append(dict(branch=branch,task=ti,name=t['name'],q_rad=qr,tcp_m=(tf.translation+tf.rotation@tool).tolist(),rotation=tf.rotation.tolist()))
    j['motor_lower_rad']=[];j['motor_upper_rad']=[]
    for i in range(7):
     a,b=sorted((p.lower[i]*p.direction[i]+p.offset[i],p.upper[i]*p.direction[i]+p.offset[i]));j['motor_lower_rad'].append(max(a,float(lo[i])-.02));j['motor_upper_rad'].append(min(b,float(hi[i])+.02))
    if continuation is not None:
     j['motor_lower_rad']=continuation['motor_lower_rad'][:];j['motor_upper_rad']=continuation['motor_upper_rad'][:]
    j['motor_lower_rad'][6]=max(actual[6]-.002 if actual[6]<1.08 else 1.08,j['motor_lower_rad'][6]);j['motor_upper_rad'][6]=min(max(1.62 if close<1.61 else 1.67,actual[6]+.002),j['motor_upper_rad'][6]);records=[s for t in tasks for s in t['segments']] # Single full verification occurs during bundle export.
    audit_start=start
    if chain_index>0:
     # The known rest-pose self contact belongs to the supported pose, not to this mid-air start:
     # measure it there first so recovery arrivals at support are judged against it.
     tf=kin.fk(home[:6]);audit_start=home;rows.insert(0,dict(branch='main',task=0,name='departure',q_rad=list(home),tcp_m=(tf.translation+tf.rotation@tool).tolist(),rotation=tf.rotation.tolist()))
    (out/'job_pending_geometry.json').write_text(json.dumps(j,indent=2));(out/'collision_samples.json').write_text(json.dumps(dict(start_rad=audit_start,trajectory_sha256=path_digest(j),samples=rows)))
    summary=dict(nominal_seconds=sum(s['duration_s'] for t in tasks for s in t['segments']),segments=len(records),recoveries=len(recoveries),task_seconds={t['name']:sum(s['duration_s'] for s in t['segments']) for t in tasks},manual_inspection_wait_s=0,physical_execution=False)
    (out/'path_summary.json').write_text(json.dumps(summary,indent=2));print('TOTAL',summary['nominal_seconds'],'SAMPLES',len(rows))
    return summary
