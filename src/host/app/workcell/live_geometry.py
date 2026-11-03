import json,time,sys,hashlib
from pathlib import Path
import numpy as np,mujoco as mj
from scipy.spatial.transform import Rotation
import xml.etree.ElementTree as ET

CONTACT_GROWTH_M = .0005  # growth of the folded-arm CAD overlap tolerated during departure
CONTACT_CAP_M = .0015  # and the overlap may never exceed this


def check(out, config):
    out=Path(out)
    from workcell.tool_geometry import width_to_slide
    job=json.loads((out/'job_pending_geometry.json').read_text());x=json.loads((out/'collision_samples.json').read_text())
    xml=ET.parse(config['geometry_xml']);wb=xml.find('worldbody');asset=xml.find('asset')
    if config.get('box_wall_check') is not False:raise ValueError('This fixture requires explicit operator-managed tray clearance')
    for element in list(wb.findall('geom')):
     if element.get('name','').startswith(('observed_tray_envelope_', 'observed_nut_envelope')):wb.remove(element)
    for element in list(asset.findall('mesh')):
     if element.get('name','').startswith('observed_tray_envelope_'):asset.remove(element)
    pb=ET.SubElement(wb,'body',name='payload_proxy',mocap='true',pos='0 0 5');ET.SubElement(pb,'geom',name='carried_part_envelope',type='box',size='.012 .040 .012',contype='4',conaffinity='1')
    for target in job['targets']:ET.SubElement(wb,'geom',name='part_'+target['target_id'],type='box',pos='0 0 5',size='.012 .040 .008',contype='1',conaffinity='3')
    stand=config.get('head_camera_stand')
    if stand:
     # The D435i and its vertical stand (struck by a re-planned departure on 2026-10-29): a cylinder from the table to the camera
     # and a box around the camera body, both static obstacles like any unselected part.
     (sx,sy),r,z0,z1=stand['xy_base_m'],stand['radius_m'],stand['z_bottom_m'],stand['z_top_m']
     ET.SubElement(wb,'geom',name='head_camera_stand',type='cylinder',pos=f'{sx} {sy} {(z0+z1)/2}',size=f'{r} {(z1-z0)/2}',contype='1',conaffinity='3')
     if stand.get('camera_box_half_m'):
      cx,cy,cz=stand['camera_xyz_base_m'];hx,hy,hz=stand['camera_box_half_m']
      ET.SubElement(wb,'geom',name='head_camera_body',type='box',pos=f'{cx} {cy} {cz}',size=f'{hx} {hy} {hz}',contype='1',conaffinity='3')
    from .live_session import inside
    selected={t['source']['detection_index'] for t in job['targets']}
    frame=json.loads(Path(json.loads((out/'input.json').read_text())['points']).read_text())
    by_index={o['detection_index']:o for o in frame['objects']}
    surfaces=json.loads((out/'target_geometry/surface_points.json').read_text())['objects']
    static_ids=[]
    for obj in surfaces:
     raw=by_index[obj['detection_index']]
     if obj['detection_index'] in selected:continue
     if raw.get('center_uv_px') is None:
      # A detected part whose position could not be measured (nut on edge, mask at the image border...) cannot be modelled as an
      # obstacle. It blocks planning only near what the arm touches (a target or a destination slot, by its detection box); elsewhere
      # the arm passes high above the tray and it is ignored, with a record.
      box=raw.get('box_xyxy_px')
      if box is None:raise ValueError('Detected part without a localised centre or box in the scene (geometry rejected): move or remove it')
      c=np.array([(box[0]+box[2])/2,(box[1]+box[3])/2]);reach=float(config.get('unlocalised_part_clearance_px',260.))
      touched=[np.array(by_index[t['source']['detection_index']]['center_uv_px'],float) for t in job['targets'] if by_index.get(t['source']['detection_index'],{}).get('center_uv_px') is not None]+[np.array(u,float) for u in json.loads((out/'destinations.json').read_text())['pixels']]
      nearest=min(float(np.linalg.norm(c-u)) for u in touched)
      if nearest<reach:raise ValueError(f'Unlocalised {raw["part_class"]} (geometry rejected) {nearest:.0f} px from a target/slot: move or remove it')
      print('UNLOCALISED_PART_IGNORED',dict(detection_index=obj['detection_index'],part_class=raw['part_class'],nearest_target_or_slot_px=round(nearest)),flush=True);continue
     if not config.get('goal_direction') and not inside(raw['center_uv_px'],config['source_roi_uv']):continue
     if 'base_surface_xyz_m' not in obj and raw['part_class'] not in ('screw','nut') and not raw.get('base_xyz_m'):
      # other-size part without a depth point: same rule as an unlocalised part (blocks only near what the arm touches)
      c=np.array(raw['center_uv_px'],float);reach=float(config.get('unlocalised_part_clearance_px',260.))
      touched=[np.array(by_index[t['source']['detection_index']]['center_uv_px'],float) for t in job['targets'] if by_index.get(t['source']['detection_index'],{}).get('center_uv_px') is not None]+[np.array(u,float) for u in json.loads((out/'destinations.json').read_text())['pixels']]
      nearest=min(float(np.linalg.norm(c-u)) for u in touched)
      if nearest<reach:raise ValueError(f'{raw["part_class"]} without depth {nearest:.0f} px from a target/slot: move it or retry')
      print('OTHER_SIZE_PART_IGNORED',dict(detection_index=obj['detection_index'],part_class=raw['part_class'],nearest_target_or_slot_px=round(nearest)),flush=True);continue
     if 'base_surface_xyz_m' not in obj and raw['part_class'] not in ('screw','nut') and raw.get('base_xyz_m'):
      # A part of the other size (M6 while picking M12, or the reverse) has no surface fit in this run: model it as a yaw-free box
      # around its detected point (M6 screw 24x24 mm, M6 nut 16x16 mm, M12 screw 70x70 mm, M12 nut 24x24 mm) so the arm still avoids it.
      half={'screw_m6':[.012,.012,.005],'nut_m6':[.008,.008,.005],'screw_m12':[.035,.035,.008],'nut_m12':[.012,.012,.006]}.get(raw['part_class'],[.035,.035,.008])
      ET.SubElement(wb,'geom',name='unselected_'+str(obj['detection_index']),type='box',pos=' '.join(map(str,np.array(raw['base_xyz_m'])-[0,0,half[2]])),size=' '.join(map(str,half)),contype='1',conaffinity='3')
      static_ids.append(obj['detection_index']);print('OTHER_SIZE_PART_AS_BOX',dict(detection_index=obj['detection_index'],part_class=raw['part_class']),flush=True);continue
     if 'base_surface_xyz_m' not in obj:raise ValueError('Non-target source object lacks geometry')
     n=np.array(obj['support_normal_base']);a=np.array((obj.get('axis_base') or [1.,0.,0.]));a=a-n*np.dot(a,n);a/=np.linalg.norm(a)
     R=np.column_stack([-np.cross(n,a),a,n]);q=Rotation.from_matrix(R).as_quat()[[3,0,1,2]];h=obj['surface_height_m']
     size=[.012,.038,h/2] if obj['part_class']=='screw' else [.014,.014,h/2]
     ET.SubElement(wb,'geom',name='unselected_'+str(obj['detection_index']),type='box',pos=' '.join(map(str,np.array(obj['base_surface_xyz_m'])-h/2*n)),quat=' '.join(map(str,q)),size=' '.join(map(str,size)),contype='1',conaffinity='3')
     static_ids.append(obj['detection_index'])
    from .geometry_cache import load_model,contact_store,save_contacts
    m,model_cache_hit=load_model(mj,ET.tostring(xml.getroot(),encoding='unicode'));d=mj.MjData(m);qids=[m.jnt_qposadr[m.joint('L'+str(i)+'_joint').id] for i in range(1,7)];jaws=[m.jnt_qposadr[m.joint('jaw_'+s).id] for s in ['left','right']];payload=m.geom('carried_part_envelope').id;mp=m.body_mocapid[m.body('payload_proxy').id];start=np.array(x['start_rad'][:6]);initial={};unexpected=[];previous={};counts={};beg=time.monotonic();known='l2_l3_urdf_asm|l5_l6_urdf_asm';maxdepth=0;names=[t['name'] for t in job['tasks']];target_map={t['target_id']:t for t in job['targets']}
    dynamic_geom_ids=[payload]+[m.geom('part_'+t['target_id']).id for t in job['targets']]
    contact_cache,contact_path=contact_store(ET.tostring(xml.getroot(),encoding='unicode'),mj.__version__);contact_cache_hits=0;collision_evaluations=0
    geom_names=[m.geom(i).name for i in range(m.ngeom)]
    body_names=[m.body(m.geom_bodyid[i]).name for i in range(m.ngeom)]
    def quat(R):return Rotation.from_matrix(R).as_quat()[[3,0,1,2]]
    for ci,row in enumerate(x['samples']):
     branch=row['branch'];name=row['name'];main=branch=='main';phase=name if main else branch;idx=row['task'] if main else names.index(branch);q=np.array(row['q_rad'][:6]);d.qpos[qids]=q;counts[branch]=counts.get(branch,0)+1
     current=next((t for t in target_map if phase.startswith(t+'_')),None)
     carrying=(main and name.endswith(('_lift','_transfer','_place')) and not name.endswith('_transfer_empty')) or (not main and branch.endswith('_open'))
     if carrying and current:
      t=target_map[current];R=np.array(row['rotation']);d.mocap_pos[mp]=np.array(row['tcp_m'])-job.get('grasp_height_offset_m',.017)*R[:,2];d.mocap_quat[mp]=quat(R);m.geom_size[payload]=[.012,.04,.012] if t['part_class']=='screw' else [.015,.015,.009]
     else:d.mocap_pos[mp]=[0,0,5]
     for tid,t in target_map.items():
      gid=m.geom('part_'+tid).id;approach=names.index(tid+'_approach');opened=names.index(tid+'_open');normal=np.array(t['source']['support_normal_base']);height=t['source']['surface_height_m'];m.geom_size[gid]=[.012,.038,height/2] if t['part_class']=='screw' else [.014,.014,height/2]
      if idx<approach:
       m.geom_pos[gid]=np.array(t['source']['base_surface_xyz_m'])-height/2*normal;m.geom_quat[gid]=quat(np.array(t['source_rotation']))
      elif idx>opened:
       m.geom_pos[gid]=np.array(t['destination_floor_base_m'])+height/2*np.array(t['destination_rotation'])[:,2];m.geom_quat[gid]=quat(np.array(t['destination_rotation']))
      else:m.geom_pos[gid]=[0,0,5]
     widths=[.002,.030,.040]
     if job.get('initial_normalize_from_observed_wide') and job['start_motor_rad'][6]<.90 and name=='departure':widths.append(.0934)
     for width in widths:
      d.qpos[jaws]=width_to_slide(width)
      # Exact state only: no rounding, no skipped samples or reused PASS result.
      key=(d.qpos.tobytes(),d.mocap_pos.tobytes(),d.mocap_quat.tobytes(),m.geom_pos[dynamic_geom_ids].tobytes(),m.geom_quat[dynamic_geom_ids].tobytes(),m.geom_size[dynamic_geom_ids].tobytes())
      key=hashlib.sha256(b''.join(key)).hexdigest()
      contacts=contact_cache.get(key)
      if contacts is None:
       mj.mj_kinematics(m,d);mj.mj_comPos(m,d);mj.mj_collision(m,d);collision_evaluations+=1
       contacts=tuple((int(c.geom1),int(c.geom2),float(c.dist)) for c in d.contact[:d.ncon] if c.dist < -1e-6)
       contact_cache[key]=contacts
      else:contact_cache_hits+=1
      depth0=0
      for g1,g2,dist in contacts:
       bodies='|'.join(sorted((body_names[g1],body_names[g2])));geoms=[geom_names[g1],geom_names[g2]];depth=-dist;maxdepth=max(maxdepth,depth)
       if ci==0:initial[bodies]=max(initial.get(bodies,0),depth)
       if bodies==known:depth0=max(depth0,depth)
       close_to_start=np.max(np.abs(q-start))<.10
       allow_start=close_to_start and bodies==known and depth<=initial.get(known,0)+.00002 and (not main or name in ['departure','return_supported'])
       allow_pickup=('carried_part_envelope' in geoms and 'measured_support_plane' in geoms and name.endswith('_lift') and depth<.003)
       if not (allow_start or allow_pickup):unexpected.append(dict(sample=ci,branch=branch,task=name,bodies=bodies,geoms=geoms,depth_m=depth,width_m=width))
      if main and name=='departure':
       # The folded arm rests with two links overlapping about 1 mm in the CAD (a modelling artefact: the real links touch). Whether the
       # overlap grows during the first samples of the departure flips with 1 mrad of start pose (runs 144909 pass, 144930 and 145004 at
       # the same pose fail by 0.4 mm), so the growth allowance is the model/pose uncertainty (0.5 mm), with an absolute cap.
       if depth0>previous.get(width,initial.get(known,0))+CONTACT_GROWTH_M or depth0>CONTACT_CAP_M:unexpected.append(dict(sample=ci,reason='initial_contact_increased',depth_m=depth0))
       previous[width]=depth0
     if len(unexpected)>25:break
    save_contacts(contact_cache,contact_path)
    r=dict(model_cache_hit=model_cache_hit,collision_cache_hits=contact_cache_hits,collision_evaluations=collision_evaluations,nonselected_obstacle_indices=static_ids,box_wall_check=False,box_wall_policy=config['box_wall_policy'],status='PASS_SAMPLED_WITH_MONOTONIC_EXISTING_START_CONTACT_DEPARTURE' if not unexpected else 'REJECTED_COLLISION',trajectory_sha256=x['trajectory_sha256'],samples=(ci+1)*3,unexpected=unexpected,initial_contact_pairs=initial,maximum_penetration_m=maxdepth,branch_samples=counts,scope='Quantized main + all declared rest-point return branches; fullCAD; jaw2/30/40mm; tray walls intentionally NOT checked (operator-managed); remaining and deposited parts; separate mocap-body carried envelope against environment. No claim of continuous proof or exact physical dimensions.',continuous_collision_proof=False,elapsed_s=time.monotonic()-beg)
    (out/'route_geometry_report.json').write_text(json.dumps(r,indent=2));job['geometry_audit']=r;(out/'job.json').write_text(json.dumps(job,indent=2));print(json.dumps(r))
    return r

if __name__=='__main__':
 import argparse
 ap=argparse.ArgumentParser();ap.add_argument('--out',required=True);ap.add_argument('--config',required=True);a=ap.parse_args()
 result=check(a.out,json.loads(Path(a.config).read_text()))
 if result['unexpected']:raise SystemExit(2)
