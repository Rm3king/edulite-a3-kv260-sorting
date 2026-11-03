"""A3 URDF-derived MuJoCo workcell. No CAN, UIO or physical network access.

Robot origins/axes/limits come from the released legacy kinematic URDF.
Matching vendor meshes drive display and decomposed collision geometry.
Vendor mass/COM overrides are applied when supplied; jaw/contact and motor dynamics need
physical identification and must not be called a calibrated digital twin.
Screws are free bodies: no attachment/weld, pose reset or success oracle while
executing. Frictional grip and destination occupancy are measured in simulation.
"""
import argparse
import hashlib
import json
import math
import os
import time
import xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np
from .tool_geometry import (TCP_IN_END,TOOL_ID,MAX_SLIDE,OPEN_INNER_HALF,OPEN_WIDTH,CLOSED_WIDTH,
                            moving_side,width_to_slide,split_wrist_inertia)

def fmt(x):return ' '.join(f'{v:.12g}' for v in x)
def vector(e,key,default='0 0 0'):return [float(x) for x in e.get(key,default).split()]

def servo_profile(urdf):
    path=Path(urdf).parent/'simulation_tracking.json'
    if path.is_file():return json.loads(path.read_text())
    return dict(kp=[100.]*6,kd=[8.]*6,motor_torque_limit_nm=[14.]*3+[6.]*3,source='legacy synthetic gains')

def model_identity(urdf):
    urdf=Path(urdf).resolve();items=[urdf,urdf.with_name('inertia_params.yaml'),urdf.with_name('simulation_tracking.json'),Path(__file__),Path(__file__).with_name('tool_geometry.py')]
    assets=urdf.parents[3]/'assets/a3_legacy_collisions/manifest.json'
    if assets.is_file():items.append(assets)
    payload='|'.join(hashlib.sha256(p.read_bytes()).hexdigest() for p in items if p.is_file())
    payload+='|insert_mm='+os.environ.get('A3_SIM_INSERT_MM','0')
    return 'SIM_'+hashlib.sha256(payload.encode()).hexdigest()

def rotation(rpy):
    r,p,y=rpy;cr,sr,cp,sp,cy,sy=math.cos(r),math.sin(r),math.cos(p),math.sin(p),math.cos(y),math.sin(y)
    return np.array([[cy*cp,cy*sp*sr-sy*cr,cy*sp*cr+sy*sr],[sy*cp,sy*sp*sr+cy*cr,sy*sp*cr-cy*sr],[-sp,cp*sr,cp*cr]])

def build(urdf, mesh_dir=None, inertia_config=None):
    u=ET.parse(urdf).getroot();links={x.get('name'):x for x in u.findall('link')}
    mesh_dir=Path(mesh_dir) if mesh_dir else None
    calibration={}
    if inertia_config:
        import yaml
        cfg=yaml.safe_load(Path(inertia_config).read_text())
        if cfg.get('use_calibrated_params'):calibration=cfg['inertia_params']
    overrides={}
    for key,value in calibration.items():
        joint=u.find("joint[@name='%s_joint']"%key)
        if joint is None:raise ValueError('INERTIA_JOINT_NOT_FOUND '+key)
        overrides[joint.find('child').get('link')]=value
    joints=u.findall('joint');children={}
    for j in joints:children.setdefault(j.find('parent').get('link'),[]).append(j)
    root=ET.Element('mujoco',model='a3_workcell_approx_v1')
    ET.SubElement(root,'compiler',angle='radian',eulerseq='xyz',inertiafromgeom='auto')
    ET.SubElement(root,'option',timestep='0.001',integrator='implicitfast',gravity='0 0 -9.81',iterations='80')
    ET.SubElement(ET.SubElement(root,'visual'),'global',offwidth='1024',offheight='768')
    default=ET.SubElement(root,'default');ET.SubElement(default,'geom',friction='1.5 .02 .002',solref='.004 1',solimp='.95 .99 .001')
    ET.SubElement(default,'joint',damping='0',armature='0')
    assets=ET.SubElement(root,'asset');mesh_names={}
    collision_names={}
    if mesh_dir:
        for m in u.findall('.//visual/geometry/mesh'):
            name=Path(m.get('filename')).name
            if name in mesh_names:continue
            path=mesh_dir/name
            if not path.is_file():raise FileNotFoundError(path)
            mesh_names[name]='vendor_'+str(len(mesh_names))
            ET.SubElement(assets,'mesh',name=mesh_names[name],file=str(path.resolve()),scale=m.get('scale','1 1 1'))
        collision_dir=Path(os.environ.get('A3_LEGACY_COLLISION_DIR',str(mesh_dir.parent/'a3_legacy_collisions')))
        if (collision_dir/'manifest.json').is_file():
            manifest=json.loads((collision_dir/'manifest.json').read_text())
            if set(mesh_names)-set(manifest['files']):raise ValueError('INCOMPLETE_VENDOR_COLLISION_ASSETS')
            for filename,row in manifest['files'].items():
                if hashlib.sha256((mesh_dir/filename).read_bytes()).hexdigest()!=row['source_sha256']:raise ValueError('COLLISION_SOURCE_MISMATCH '+filename)
                collision_names[filename]=[]
                for part in row['parts']:
                    name='convex_'+str(len(list(assets)))
                    path=collision_dir/part['file']
                    if hashlib.sha256(path.read_bytes()).hexdigest()!=part['sha256']:raise ValueError('COLLISION_PART_MISMATCH')
                    ET.SubElement(assets,'mesh',name=name,file=str(path.resolve()))
                    collision_names[filename].append(name)
    def mesh_geom(body,name,filename,position,quaternion,color):
        ET.SubElement(body,'geom',name='display_'+name,type='mesh',mesh=mesh_names[filename],pos=position,quat=quaternion,rgba=color,contype='0',conaffinity='0',mass='0',group='2')
        for index,mesh in enumerate(collision_names.get(filename,[mesh_names[filename]])):
            ET.SubElement(body,'geom',name=name+'__'+str(index),type='mesh',mesh=mesh,pos=position,quat=quaternion,rgba='0 0 0 0',contype='2',conaffinity='3',mass='0',group='3',friction='2 .02 .002')
    materials={m.get('name'):m.find('color').get('rgba') for m in u.findall('material') if m.find('color') is not None}
    world=ET.SubElement(root,'worldbody');ET.SubElement(world,'light',pos='0 -.2 1',dir='0 0 -1',diffuse='.9 .9 .9')
    ET.SubElement(world,'camera',name='scene',pos='-.10 0 .78',quat='1 0 0 0',fovy='48')
    table_z=-.054
    insert=float(os.environ.get('A3_SIM_INSERT_MM','0'))/1000
    if not 0<=insert<=.04:raise ValueError('SIM_TRAY_INSERT_RANGE')
    ET.SubElement(world,'geom',name='table',type='plane',pos=fmt([0,0,table_z]),size='1 1 .02',rgba='.13 .17 .21 1')
    bodies={};arm_joints=[];finger_inertias=[]
    def link(parent,name,incoming=None):
        origin=incoming.find('origin') if incoming is not None else None
        attrs=dict(name=name)
        if origin is not None:
            # URDF fixed-axis RPY is Rz(yaw)Ry(pitch)Rx(roll). MuJoCo's default
            # lower-case xyz Euler is intrinsic; quaternions avoid that mismatch.
            from scipy.spatial.transform import Rotation
            quat=Rotation.from_matrix(rotation(vector(origin,'rpy'))).as_quat()
            attrs.update(pos=origin.get('xyz','0 0 0'),quat=fmt([quat[3],*quat[:3]]))
        body=ET.SubElement(parent,'body',**attrs);bodies[name]=body
        iner=links[name].find('inertial')
        if iner is not None:
            o=iner.find('origin');it=iner.find('inertia');r=rotation(vector(o,'rpy')) if o is not None else np.eye(3)
            if it is not None:
                xx,yy,zz,xy,xz,yz=[float(it.get(k,'0')) for k in ['ixx','iyy','izz','ixy','ixz','iyz']]
                inertia=r@np.array([[xx,xy,xz],[xy,yy,yz],[xz,yz,zz]])@r.T
                override=overrides.get(name,{})
                mass=float(override.get('mass',iner.find('mass').get('value')))
                com=np.array(override.get('com',vector(o,'xyz') if o is not None else [0,0,0]))
                if name=='l5_l6_urdf_asm':
                    mass,com,inertia,fingers=split_wrist_inertia(mass,com,inertia);finger_inertias.extend(fingers)
                ET.SubElement(body,'inertial',pos=fmt(com),mass=str(mass),fullinertia=fmt([inertia[0,0],inertia[1,1],inertia[2,2],inertia[0,1],inertia[0,2],inertia[1,2]]))
        if mesh_dir:
            for i,v in enumerate(links[name].findall('visual')):
                m=v.find('geometry/mesh')
                if m is None:continue
                if name=='l5_l6_urdf_asm' and moving_side(m.get('filename')):continue
                o=v.find('origin');mat=v.find('material')
                color=materials.get(mat.get('name'),'.7 .7 .7 1') if mat is not None else '.7 .7 .7 1'
                if mat is not None and mat.find('color') is not None:color=mat.find('color').get('rgba')
                from scipy.spatial.transform import Rotation
                quat=Rotation.from_matrix(rotation(vector(o,'rpy')) if o is not None else np.eye(3)).as_quat()
                mesh_geom(body,'visual_'+name+'_'+str(i),Path(m.get('filename')).name,o.get('xyz','0 0 0') if o is not None else '0 0 0',fmt([quat[3],*quat[:3]]),color)
        if incoming is not None and incoming.get('type')!='fixed':
            n=incoming.get('name');lim=incoming.find('limit');arm_joints.append(n)
            ET.SubElement(body,'joint',name=n,type='hinge',axis=incoming.find('axis').get('xyz'),limited='true',range=lim.get('lower')+' '+lim.get('upper'))
        for j in children.get(name,[]):
            if j.get('name')=='L7_joint':continue # fictitious SDK joint is not the actual parallel gripper
            o=j.find('origin');end=vector(o,'xyz') if o is not None else [0,0,0]
            if not mesh_dir and np.linalg.norm(end)>.015:
                # The TCP offset is not a solid rod ending at the grasp point.
                # Leave the fingertip volume free; model jaws separately below.
                if j.get('name')=='end_effector_joint':end=[x*.5 for x in end]
                ET.SubElement(body,'geom',name='arm_'+j.get('name'),type='capsule',fromto=fmt([0,0,0]+end),size='.012',rgba='.78 .82 .86 0' if mesh_dir else '.78 .82 .86 1',contype='2',conaffinity='1')
            link(body,j.find('child').get('link'),j)
        if name=='end_effector':
            ET.SubElement(body,'site',name='legacy_end',size='.002',rgba='.8 .3 .2 .5')
            ET.SubElement(body,'site',name='tcp',pos=fmt(TCP_IN_END),size='.003',rgba='0 1 .4 1')
            for (sign,side),(mass,com,inertia) in zip([(1,'right'),(-1,'left')],finger_inertias):
                finger=ET.SubElement(body,'body',name='jaw_'+side)
                ET.SubElement(finger,'inertial',mass=str(mass),pos=fmt(com+[0,0,.074]),fullinertia=fmt([inertia[0,0],inertia[1,1],inertia[2,2],inertia[0,1],inertia[0,2],inertia[1,2]]))
                ET.SubElement(finger,'joint',name='jaw_'+side,type='slide',axis=fmt([sign,0,0]),range=fmt([-MAX_SLIDE,0]),limited='true',damping='.2',armature='0')
                if mesh_dir:
                    for i,v in enumerate(links['l5_l6_urdf_asm'].findall('visual')):
                        m=v.find('geometry/mesh');filename=Path(m.get('filename')).name
                        if moving_side(filename)!=sign:continue
                        o=v.find('origin');position=np.array(vector(o,'xyz'))+[0,0,.074]
                        quat=Rotation.from_matrix(rotation(vector(o,'rpy'))).as_quat()
                        geom_name='pad_'+side if filename in ('hand_r.stl','part_15.stl') else 'moving_'+str(i)
                        color=materials[v.find('material').get('name')]
                        mesh_geom(finger,geom_name,filename,fmt(position),fmt([quat[3],*quat[:3]]),color)
                else:
                    ET.SubElement(finger,'geom',name='pad_'+side,type='box',pos=fmt([sign*(OPEN_INNER_HALF+.003),0,TCP_IN_END[2]]),size='.003 .006 .006',mass='0',rgba='.9 .3 .16 1',friction='2.5 .02 .002')
        return body
    link(world,'base_link')
    contacts=ET.SubElement(root,'contact')
    # URDF neighbours are mechanically assembled joints. Static base meshes
    # require this explicit adjacent-pair filter in MuJoCo as well.
    for j in joints:
        if j.get('name')=='L7_joint':continue
        ET.SubElement(contacts,'exclude',body1=j.find('parent').get('link'),body2=j.find('child').get('link'))
    # Layout is synthetic and must never be used as real calibration.
    trays={'A':[-.22,.12],'B':[-.22,-.12]}
    floor=table_z+.004+insert
    for name,(x,y) in trays.items():
        ET.SubElement(world,'geom',name='tray_'+name,type='box',pos=fmt([x,y,table_z+.002]),size='.100 .075 .002',rgba='.17 .37 .43 1',contype='1',conaffinity='3')
        if insert:ET.SubElement(world,'geom',name='insert_'+name,type='box',pos=fmt([x,y,floor-insert/2]),size=fmt([.096,.071,insert/2]),rgba='.25 .32 .38 1',contype='1',conaffinity='3')
        for axis in [0,1]:
            for sign in [-1,1]:
                center=[x,y,table_z+.040];center[axis]+=sign*(.098 if axis==0 else .073)
                size=[.100,.075,.040];size[axis]=.002
                ET.SubElement(world,'geom',name=f'wall_{name}_{axis}_{sign}',type='box',pos=fmt(center),size=fmt(size),rgba='.22 .48 .53 1')
    for i,(x,y,angle) in enumerate([(-.22,.12,0.),(-.26,.12,.2),(-.18,.12,-.2)]):
        body=ET.SubElement(world,'body',name='screw_'+str(i),pos=fmt([x,y,floor+.011]),euler=fmt([0,0,angle]))
        ET.SubElement(body,'freejoint',name='screw_free_'+str(i))
        ET.SubElement(body,'geom',name='shaft_'+str(i),type='capsule',fromto='0 -.023 0 0 .023 0',size='.006',mass='.05',rgba='.10 .11 .12 1',contype='1',conaffinity='3')
        ET.SubElement(body,'geom',name='head_'+str(i),type='cylinder',pos='0 .028 0',euler='1.57079632679 0 0',size='.010 .004',mass='.01',rgba='.18 .19 .20 1')
    for i,x in enumerate([-.18,-.26]):
        body=ET.SubElement(world,'body',name='nut_'+str(i),pos=fmt([x,.07,floor+.007]))
        ET.SubElement(body,'freejoint',name='nut_free_'+str(i));ET.SubElement(body,'geom',name='nut_shape_'+str(i),type='cylinder',size='.010 .005',mass='.015',rgba='.30 .31 .32 1')
    actuators=ET.SubElement(root,'actuator')
    tracking=servo_profile(urdf)
    for i,n in enumerate(arm_joints):
        # The MIT PD law is represented as an affine actuator so implicitfast
        # includes its velocity derivative. Explicit torque feedback at 1 ms
        # can oscillate for the light wrist when invented armature is removed.
        kp,kd,limit=tracking['kp'][i],tracking['kd'][i],tracking['motor_torque_limit_nm'][i]
        ET.SubElement(actuators,'general',name='drive_'+n,joint=n,biastype='affine',
                      gainprm='1',biasprm=fmt([0,-kp,-kd]),ctrllimited='false',
                      forcelimited='true',forcerange=fmt([-limit,limit]))
    for side in ['left','right']:ET.SubElement(actuators,'position',name='grip_'+side,joint='jaw_'+side,kp='500',kv='4',ctrllimited='true',ctrlrange=fmt([-MAX_SLIDE,0]),forcelimited='true',forcerange='-25 25')
    return ET.tostring(root,encoding='unicode')

class Simulation:
    def __init__(self, urdf, output, mesh_dir=None, inertia_config=None):
        import mujoco
        urdf=Path(urdf).resolve()
        self.mj=mujoco;self.output=Path(output);self.output.mkdir(parents=True,exist_ok=True)
        defaults=Path(urdf).parent
        candidate=Path(urdf).parents[3]/'assets/a3_legacy_meshes'
        mesh_dir=mesh_dir or os.environ.get('A3_LEGACY_MESH_DIR') or (candidate if candidate.is_dir() else None)
        collision_dir=Path(os.environ.get('A3_LEGACY_COLLISION_DIR',str(Path(mesh_dir).parent/'a3_legacy_collisions'))) if mesh_dir else None
        if mesh_dir and not (collision_dir/'manifest.json').is_file():raise ValueError('VENDOR_COLLISION_ASSETS_REQUIRED: bake matching CAD convex parts first')
        inertia_config=inertia_config or (defaults/'inertia_params.yaml' if (defaults/'inertia_params.yaml').is_file() else None)
        self.model_scope=dict(urdf_sha256=hashlib.sha256(Path(urdf).read_bytes()).hexdigest(),vendor_meshes=bool(mesh_dir),inertia_config_sha256=hashlib.sha256(Path(inertia_config).read_bytes()).hexdigest() if inertia_config else None,gripper_inertia='vendor aggregate matched at CAD-open; moving CAD-finger mass distribution estimated',collision='per-vendor-part convex collision including fingers; contact audit required',motor_dynamics='ideal torque actuators; latency/friction/compliance NOT identified',tool_id=TOOL_ID,T_end_grasp_translation_m=TCP_IN_END.tolist(),box_outer_size_m=[.2,.15,.08],tray_insert_m=float(os.environ.get('A3_SIM_INSERT_MM','0'))/1000)
        xml=build(urdf,mesh_dir,inertia_config);self.xml=xml;self.defer_render=False
        (self.output/'scene.xml').write_text(xml)
        (self.output/'model_scope.json').write_text(json.dumps(self.model_scope,indent=2)+'\n')
        self.model=mujoco.MjModel.from_xml_string(xml);self.data=mujoco.MjData(self.model)
        self.model_scope.update(collision_manifest_sha256=hashlib.sha256((collision_dir/'manifest.json').read_bytes()).hexdigest() if collision_dir else None,collision_geom_count=int(np.count_nonzero(self.model.geom_contype)),display_geom_count=int(np.count_nonzero(self.model.geom_group==2)))
        self.gravity_data=mujoco.MjData(self.model);self.gravity_force=np.zeros(self.model.nv)
        self.tracking=servo_profile(urdf);self.kp=np.asarray(self.tracking['kp']);self.kd=np.asarray(self.tracking['kd'])
        self.model_scope.update(servo_source=self.tracking.get('source'),kp=self.kp.tolist(),kd=self.kd.tolist(),torque_limits_nm=self.tracking['motor_torque_limit_nm'],feedforward='gravity only; no perfect Coriolis or acceleration cancellation',rotor_armature='0; unmeasured, not invented')
        (self.output/'model_scope.json').write_text(json.dumps(self.model_scope,indent=2)+'\n')
        self.joints=[self.model.joint('L'+str(i)+'_joint').id for i in range(1,7)]
        self.qids=[int(self.model.jnt_qposadr[i]) for i in self.joints];self.dids=[int(self.model.jnt_dofadr[i]) for i in self.joints]
        self.tcp=self.model.site('tcp').id;self.rows=[];self.frames=[];self.contact_violations={}
        self.jaw_qids={side:int(self.model.jnt_qposadr[self.model.joint('jaw_'+side).id]) for side in ['left','right']}
        self.data.qpos[self.qids]=np.deg2rad([-60,10,-15,15,20,-10]);self.forward()
        self.grip=width_to_slide(OPEN_WIDTH)
        for index in self.jaw_qids.values():self.data.qpos[index]=self.grip
        self.forward();self.reference=self.data.qpos[self.qids].copy();self.reference_v=np.zeros(6)
        self.hold(.5,'INITIAL_SETTLE');self.rows=[];self.frames=[]
    def forward(self):self.mj.mj_forward(self.model,self.data)
    def pose(self):return self.data.site_xpos[self.tcp].copy(),self.data.site_xmat[self.tcp].reshape(3,3).copy()
    def ik(self,pos,rot,seed=None):
        from scipy.spatial.transform import Rotation
        saved=self.data.qpos.copy();vel=self.data.qvel.copy()
        guess=self.data.qpos[self.qids].copy() if seed is None else np.asarray(seed,float).copy()
        jp=np.zeros((3,self.model.nv));jr=jp.copy();best=1e10
        seeds=[guess,np.deg2rad([-35,91,-67,66,0,-10]),np.deg2rad([-50,110,-110,30,0,0])]
        try:
            for q in seeds:
                for _ in range(200):
                    self.data.qpos[self.qids]=q;self.forward();p,r=self.pose()
                    er=Rotation.from_matrix(rot@r.T).as_rotvec()
                    error=np.r_[np.asarray(pos)-p,er]
                    if np.linalg.norm(error[:3])<1e-5 and np.linalg.norm(er)<1e-4:return q.copy()
                    self.mj.mj_jacSite(self.model,self.data,jp,jr,self.tcp);j=np.vstack([jp[:,self.dids],jr[:,self.dids]])
                    # Reduce damping near convergence instead of stalling at
                    # a small residual at a straight-wrist singularity.
                    damping=1e-7 if np.linalg.norm(error)<.01 else 1e-5
                    dq=j.T@np.linalg.solve(j@j.T+damping*np.eye(6),error)
                    q=q+np.clip(dq,-.08,.08)
                    q=np.clip(q,self.model.jnt_range[self.joints,0],self.model.jnt_range[self.joints,1])
                    best=min(best,float(np.linalg.norm(error)))
            raise ValueError('SIM_IK_UNREACHABLE '+str(best))
        finally:self.data.qpos[:]=saved;self.data.qvel[:]=vel;self.forward()
    def control(self, q, v, grip, phase):
        if getattr(self,'poll_hook',None) and not getattr(self,'inside_poll',False):self.poll_hook()
        self.reference=np.asarray(q);self.reference_v=np.asarray(v);self.grip=grip
        for _ in range(5):
            self.gravity_data.qpos[:]=self.data.qpos
            self.mj.mj_kinematics(self.model,self.gravity_data);self.mj.mj_comPos(self.model,self.gravity_data)
            self.mj.mj_comVel(self.model,self.gravity_data)
            self.mj.mj_rne(self.model,self.gravity_data,0,self.gravity_force)
            self.data.ctrl[:6]=self.kp*q+self.kd*v+self.gravity_force[self.dids]
            self.data.ctrl[6:]=grip;self.mj.mj_step(self.model,self.data)
        self.forward();p,_=self.pose()
        self.audit_contacts(phase)
        if len(self.rows)%10==0:
            frame=self.snapshot(phase);self.frames.append(frame)
        self.rows.append(dict(time_s=float(self.data.time),phase=phase,q_ref_rad=list(map(float,q)),q_rad=self.data.qpos[self.qids].tolist(),tcp_m=p.tolist(),contacts=int(self.data.ncon)))
    def movej(self,goal,seconds=2.,phase='TRANSIT',grip=None):
        start=self.reference.copy();delta=np.asarray(goal)-start;grip=self.grip if grip is None else grip
        for u in np.linspace(0,1,round(seconds/.005)+1):
            s=35*u**4-84*u**5+70*u**6-20*u**7;ds=(140*u**3-420*u**4+420*u**5-140*u**6)/seconds
            self.control(start+s*delta,ds*delta,grip,phase)
    def movel(self,pos,rot,seconds=2.,phase='DESCEND'):
        start,_=self.pose();seed=self.reference.copy();previous=seed.copy()
        for u in np.linspace(0,1,round(seconds/.005)+1):
            s=35*u**4-84*u**5+70*u**6-20*u**7
            q=self.ik(start+(np.asarray(pos)-start)*s,rot,seed)
            self.control(q,(q-previous)/.005,self.grip,phase);seed=previous=q
    def hold(self,seconds,phase,grip=None):
        start=self.grip;end=start if grip is None else grip
        for u in np.linspace(0,1,round(seconds/.005)+1):self.control(self.reference,np.zeros(6),start+(end-start)*u,phase)
    def snapshot(self,phase='IDLE'):
        objects={}
        for name in ['screw_0','screw_1','screw_2','nut_0','nut_1']:objects[name]=self.data.xpos[self.model.body(name).id].tolist()
        yaws={name:float(math.atan2(self.data.xmat[self.model.body(name).id][3],self.data.xmat[self.model.body(name).id][0])) for name in objects}
        pads={name:self.data.xpos[self.model.body('jaw_'+name).id].tolist() for name in ['left','right']}
        arm=[self.data.xpos[self.model.body(n).id].tolist() for n in ['base_link','l1_link_urdf_asm','l2_l3_urdf_asm','l3_lnik_urdf_asm','l4_l5_urdf_asm','part_9','l5_l6_urdf_asm','end_effector']]
        def geom(gid):
            t=np.eye(4);t[:3,:3]=self.data.geom_xmat[gid].reshape(3,3);t[:3,3]=self.data.geom_xpos[gid]
            return dict(name=self.model.geom(gid).name,type=int(self.model.geom_type[gid]),size=self.model.geom_size[gid].tolist(),pose=t.tolist(),rgba=self.model.geom_rgba[gid].tolist())
        environment=[];object_geoms=[]
        for gid in range(self.model.ngeom):
            bid=int(self.model.geom_bodyid[gid]);name=self.model.body(bid).name
            if bid==0:environment.append(geom(gid))
            elif name in objects:object_geoms.append(geom(gid))
        slides={side:float(self.data.qpos[index]) for side,index in self.jaw_qids.items()}
        return dict(schema='workcell.physics-frame.v2',tool_id=TOOL_ID,time_s=float(self.data.time),phase=phase,q_rad=self.data.qpos[self.qids].tolist(),tcp_m=self.pose()[0].tolist(),legacy_end_m=self.data.site_xpos[self.model.site('legacy_end').id].tolist(),jaw_slide_m=slides,jaw_width_m=2*OPEN_INNER_HALF+sum(slides.values()),objects=objects,object_yaw_rad=yaws,arm_points_m=arm,pads_m=pads,environment_geoms=environment,object_geoms=object_geoms,simulated=True)

    def audit_contacts(self,phase):
        for c in self.data.contact[:self.data.ncon]:
            if c.dist>=-1e-6:continue # ignore sub-micron numerical tangency
            names=[self.model.geom(int(g)).name for g in [c.geom1,c.geom2]]
            bids=[int(self.model.geom_bodyid[int(g)]) for g in [c.geom1,c.geom2]]
            bodies=[self.model.body(b).name for b in bids]
            robot=[b!=0 and not (n.startswith('screw_') or n.startswith('nut_')) for b,n in zip(bids,bodies)]
            if not any(robot):continue
            # Finger/object contact is the intended frictional grasp. All other
            # robot/environment, robot/object and nonadjacent self contacts
            # remain visible in the audit and may invalidate the simulated run.
            intended=any(n.startswith(('pad_left','pad_right')) for n in names) and any(n.startswith(('screw_','nut_')) for n in bodies)
            if intended:continue
            key='|'.join(sorted(names));depth=float(-c.dist)
            if depth>self.contact_violations.get(key,{}).get('penetration_m',0):
                self.contact_violations[key]=dict(geoms=names,bodies=bodies,penetration_m=depth,phase=phase,time_s=float(self.data.time))
    def render(self,name='scene.png'):
        if self.defer_render:
            (self.output/'render_scene.xml').write_text(self.xml)
            np.savez_compressed(self.output/(Path(name).stem+'.render_state.npz'),qpos=self.data.qpos,time_s=self.data.time)
            return
        from PIL import Image
        with self.mj.Renderer(self.model,height=540,width=800) as renderer:
            renderer.update_scene(self.data,camera='scene');Image.fromarray(renderer.render()).save(self.output/name)
    def save_failure(self,error):
        (self.output/'failure.json').write_text(json.dumps(dict(status='SIMULATION_ERROR',error=repr(error),model_scope=self.model_scope,forbidden_contacts=list(self.contact_violations.values()),motor_commands_sent=0),indent=2)+'\n')
        (self.output/'trajectory.json').write_text(json.dumps(self.rows,separators=(',',':'))+'\n')
        (self.output/'frames.json').write_text(json.dumps(self.frames,separators=(',',':'))+'\n')
    def preflight_pick(self,object_name,place,rot,place_rot):
        """Sample nominal IK and whole-CAD contacts before moving the simulator.

        Restores every free-body/joint state. This is a sampled rigid-geometry
        check, not a continuous collision proof or grasp-physics prediction.
        """
        saved=self.data.qpos.copy();vel=self.data.qvel.copy();old=self.contact_violations
        self.contact_violations={};samples=0;phase='START'
        home=self.reference.copy();target=self.data.xpos[self.model.body(object_name).id].copy()
        pre=target+[0,0,.10-self.model_scope['tray_insert_m']];place=np.asarray(place);above=place+[0,0,.06]
        try:
            self.data.qpos[self.qids]=home;self.forward()
            for phase,kind,pos,orient in [('APPROACH','J',pre,rot),('DESCEND','L',target,rot),('LIFT','L',pre,rot),('TRANSFER','J',above,place_rot),('PLACE','L',place,place_rot),('RETREAT','L',above,place_rot),('RETURN','J',None,None)]:
                start=self.data.qpos[self.qids].copy();start_pos=self.pose()[0]
                goal=home if pos is None else self.ik(pos,orient,start)
                count=max(40,int(np.max(np.abs(goal-start))/.015)+1)
                previous=start
                for u in np.linspace(0,1,count):
                    s=35*u**4-84*u**5+70*u**6-20*u**7
                    q=start+s*(goal-start) if kind=='J' else self.ik(start_pos+s*(pos-start_pos),orient,previous)
                    self.data.qpos[self.qids]=q;self.forward();previous=q;samples+=1
                    self.audit_contacts(phase)
                    if self.contact_violations:
                        c=max(self.contact_violations.values(),key=lambda x:x['penetration_m'])
                        raise ValueError('SIM_PATH_COLLISION '+phase+' '+'/'.join(c['geoms']))
            return dict(status='PASS_SAMPLED_CAD_PATH',samples=samples,joint_sample_step_target_rad=.015,scope='Nominal rigid path, current open jaws; no grasp/dynamic guarantee')
        except ValueError as error:
            raise ValueError('SIM_PREFLIGHT_'+phase+': '+str(error)) from error
        finally:
            self.data.qpos[:]=saved;self.data.qvel[:]=vel;self.forward();self.contact_violations=old
    def run_pick(self,object_name='screw_0',place_position=None,rotation_matrix=None,place_rotation_matrix=None):
        self.contact_violations={}
        initial=self.snapshot();home=self.reference.copy();target=np.array(initial['objects'][object_name])
        rot=np.eye(3) if rotation_matrix is None else np.asarray(rotation_matrix);pre=target+[0,0,.10-self.model_scope['tray_insert_m']]
        self.render('initial.png')
        self.movej(self.ik(pre,rot),phase='APPROACH');self.movel(target,rot)
        self.hold(.7,'CLOSE',width_to_slide(CLOSED_WIDTH))
        self.movel(pre,rot,phase='LIFT')
        lift=self.snapshot('LIFTED');self.render('lifted.png')
        place=target.copy() if place_position is None else np.asarray(place_position).copy()
        if place_position is None:
            place[1]=-.12;place[2]+=.04 # release clearance; never push the held part into the floor
        place_rot=rotation([0,0,math.pi/2]) if place_rotation_matrix is None and place[1]<0 else (rot if place_rotation_matrix is None else place_rotation_matrix)
        self.movej(self.ik(place+[0,0,.06],place_rot),phase='TRANSFER')
        self.movel(place,place_rot,phase='PLACE');self.hold(.7,'OPEN',width_to_slide(OPEN_WIDTH))
        self.movel(place+[0,0,.06],place_rot,phase='RETREAT');self.movej(home,phase='RETURN');self.hold(.5,'SETTLE')
        final=self.snapshot('DONE');self.render('final.png')
        lifted=bool(lift['objects'][object_name][2]>initial['objects'][object_name][2]+.05)
        # A 60mm screw may stand on its head after release. Test the actual
        # primitive bounds, resting speed and released contact, not body-origin
        # height <30mm (which incorrectly rejected a stable upright placement).
        body=self.model.body(object_name).id;low=[];high=[];object_geoms=[]
        for gid in range(self.model.ngeom):
            if self.model.geom_bodyid[gid]!=body:continue
            object_geoms.append(gid);axis=self.data.geom_xmat[gid].reshape(3,3)[:,2];size=self.model.geom_size[gid]
            if self.model.geom_type[gid]==self.mj.mjtGeom.mjGEOM_CAPSULE:extent=size[0]+size[1]*np.abs(axis)
            else:extent=size[0]*np.sqrt(np.maximum(0,1-axis**2))+size[1]*np.abs(axis)
            low.append(self.data.geom_xpos[gid]-extent);high.append(self.data.geom_xpos[gid]+extent)
        lo=np.min(low,axis=0);hi=np.max(high,axis=0);velocity=np.zeros(6)
        self.mj.mj_objectVelocity(self.model,self.data,self.mj.mjtObj.mjOBJ_BODY,body,velocity,0)
        pads={gid for gid in range(self.model.ngeom) if self.model.geom(gid).name.startswith(('pad_left','pad_right'))}
        touching_pad=any((c.geom1 in object_geoms and c.geom2 in pads) or (c.geom2 in object_geoms and c.geom1 in pads) for c in self.data.contact[:self.data.ncon])
        center=np.array([-.22,-.12 if place[1]<0 else .12]);half=np.array([.095,.070]);floor=-.050+self.model_scope['tray_insert_m']
        placed=bool(np.all(lo[:2]>=center-half) and np.all(hi[:2]<=center+half) and lo[2]>floor-.001 and hi[2]<floor+.09 and np.linalg.norm(velocity[3:])<.02 and not touching_pad)
        errors=np.asarray([np.asarray(x['q_ref_rad'])-x['q_rad'] for x in self.rows])
        maximum_penetration=max([v['penetration_m'] for v in self.contact_violations.values()] or [0.])
        geometry_pass=not self.contact_violations
        r=dict(status='PASS_SIM_CONTACT_PICK_RETURN' if lifted and placed and geometry_pass else 'FAIL_SIM_GEOMETRY_OR_GRASP',
               initial=initial,lifted=lift,final=final,lift_detected=lifted,destination_occupied=placed,
               rmse_joint_rad=np.sqrt(np.mean(errors**2,axis=0)).tolist(),max_joint_error_rad=np.max(np.abs(errors),axis=0).tolist(),
               model_scope=self.model_scope,
               geometry_pass=geometry_pass,maximum_forbidden_penetration_m=maximum_penetration,forbidden_contacts=list(self.contact_violations.values()),attachment_constraints_used=False,object_teleports_during_execution=0,motor_commands_sent=0)
        r['placement_evidence']=dict(object_bounds_min_m=lo.tolist(),object_bounds_max_m=hi.tolist(),linear_speed_m_s=float(np.linalg.norm(velocity[3:])),touching_gripper=bool(touching_pad),criterion='Full horizontal primitive bounds inside tray, stable after release; upright screws allowed')
        (self.output/'trajectory.json').write_text(json.dumps(self.rows,separators=(',',':'))+'\n')
        (self.output/'frames.json').write_text(json.dumps(self.frames,separators=(',',':'))+'\n')
        (self.output/'report.json').write_text(json.dumps(r,indent=2)+'\n');return r

def render_saved(directory):
    """Render snapshots after the protocol owner has stopped, outside its lease."""
    import mujoco as mj
    from PIL import Image
    directory=Path(directory);model=mj.MjModel.from_xml_path(str(directory/'render_scene.xml'));data=mj.MjData(model)
    with mj.Renderer(model,height=540,width=800) as renderer:
        for name in ['initial','lifted','final']:
            with np.load(directory/(name+'.render_state.npz')) as state:
                data.qpos[:]=state['qpos'];data.time=float(state['time_s'])
            mj.mj_forward(model,data);renderer.update_scene(data,camera='scene')
            Image.fromarray(renderer.render()).save(directory/(name+'.png'))

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--urdf',required=True);p.add_argument('--output',required=True)
    p.add_argument('--mode',choices=['pick','render','fk'],default='pick');p.add_argument('--fk-input');a=p.parse_args()
    s=Simulation(a.urdf,a.output)
    if a.mode=='pick':
        try:report=s.run_pick()
        except Exception as error:s.save_failure(error);raise
        print(json.dumps(report,indent=2))
        if not report['status'].startswith('PASS'):raise SystemExit(1)
    elif a.mode=='render':s.render();print(json.dumps(s.snapshot()))
    else:
        data=json.loads(Path(a.fk_input).read_text());out=[]
        for q in data['q_rad']:
            s.data.qpos[s.qids]=q;s.forward();pos,rot=s.pose();out.append(dict(position=pos.tolist(),rotation=rot.tolist()))
        print(json.dumps(dict(poses=out,motor_commands_sent=0)))
