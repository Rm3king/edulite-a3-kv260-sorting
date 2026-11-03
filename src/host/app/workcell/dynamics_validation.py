"""Cross-engine model checks; never accesses robot devices or a network.

Generate a Pinocchio reference in the motion runtime environment, then check
the same configurations in MuJoCo's isolated environment.
"""
import argparse
import hashlib
import json
from pathlib import Path
import sys
import numpy as np


def reference(runtime, count):
    sys.path.insert(0,str(runtime))
    import pinocchio as pin
    from motion_runtime import MotionRuntime
    r=MotionRuntime();kin=r.kin;model=kin.model;rng=np.random.default_rng(20261022)
    lower=model.lowerPositionLimit[:6];upper=model.upperPositionLimit[:6]
    rows=[]
    for q in rng.uniform(lower+.1*(upper-lower),upper-.1*(upper-lower),(count,6)):
        qp=kin._to_pinocchio_q(q.tolist());v=np.r_[rng.uniform(-.3,.3,6),0.];a=np.r_[rng.uniform(-.6,.6,6),0.]
        mass=np.array(pin.crba(model,model.createData(),qp));mass=np.triu(mass)+np.triu(mass,1).T
        pose=kin.forward_kinematics_se3(q.tolist())
        rows.append(dict(q=q.tolist(),v=v[:6].tolist(),a=a[:6].tolist(),mass=mass[:6,:6].tolist(),
                         gravity=np.asarray(pin.computeGeneralizedGravity(model,model.createData(),qp))[:6].tolist(),
                         rnea=np.asarray(pin.rnea(model,model.createData(),qp,v,a))[:6].tolist(),
                         position=pose.translation.tolist(),rotation=pose.rotation.tolist()))
    return dict(schema='a3.dynamics.reference.v1',model_id=r.model_id,
                urdf_sha256=hashlib.sha256((Path(runtime)/'model/el_a3_legacy.urdf').read_bytes()).hexdigest(),
                inertia_sha256=hashlib.sha256((Path(runtime)/'model/inertia_params.yaml').read_bytes()).hexdigest(),
                scope='Vendor legacy model plus supplied mass/COM overrides; not physical identification',samples=rows)


def check(ref,urdf,inertia,meshes):
    import mujoco as mj
    from .simulation import build
    from .urdf_scene import VisualModel
    from .tool_geometry import TCP_IN_END,visual_transform,moving_side
    refs=json.loads(Path(ref).read_text());errors={k:[] for k in ['fk_m','fk_rotation','mass_kg_m2','gravity_nm','rnea_nm','visual_fk_m','visual_fk_rotation','visual_world_bounds_m']}
    assert hashlib.sha256(Path(urdf).read_bytes()).hexdigest()==refs['urdf_sha256'],'URDF_HASH_MISMATCH'
    assert hashlib.sha256(Path(inertia).read_bytes()).hexdigest()==refs['inertia_sha256'],'INERTIA_HASH_MISMATCH'
    visual=VisualModel(urdf,meshes) if meshes else None
    baselines=[];visual_delta=[]
    for mesh_path in ([None,meshes] if meshes else [None]):
        m=mj.MjModel.from_xml_string(build(urdf,mesh_path,inertia));d=mj.MjData(m)
        # Contacts are audited by the trajectory test, not this algebra check.
        m.opt.disableflags|=int(mj.mjtDisableBit.mjDSBL_CONTACT)
        ids=[m.joint('L%d_joint'%i).id for i in range(1,7)]
        qids=m.jnt_qposadr[ids];dids=m.jnt_dofadr[ids];tcp=m.site('tcp').id
        for index,row in enumerate(refs['samples']):
            d.qpos[qids]=row['q'];d.qvel[:]=0;d.qvel[dids]=row['v'];mj.mj_forward(m,d)
            mass=np.zeros((m.nv,m.nv));mj.mj_fullM(m,d,mass);mass=mass[np.ix_(dids,dids)]
            rnea=mass@row['a']+d.qfrc_bias[dids]
            d.qvel[:]=0;mj.mj_forward(m,d);gravity=d.qfrc_bias[dids].copy()
            p=d.site_xpos[tcp].copy();rot=d.site_xmat[tcp].reshape(3,3).copy()
            expected_tcp=np.asarray(row['position'])+np.asarray(row['rotation'])@TCP_IN_END
            for key,value in [('fk_m',np.max(np.abs(p-expected_tcp))),('fk_rotation',np.max(np.abs(rot-row['rotation']))),
                              ('mass_kg_m2',np.max(np.abs(mass-row['mass']))),('gravity_nm',np.max(np.abs(gravity-row['gravity']))),
                              ('rnea_nm',np.max(np.abs(rnea-row['rnea'])))]:errors[key].append(float(value))
            if visual:
                poses=visual.poses(row['q']);t=poses['end_effector']
                errors['visual_fk_m'].append(float(np.max(np.abs(t[:3,3]-row['position']))))
                errors['visual_fk_rotation'].append(float(np.max(np.abs(t[:3,:3]-row['rotation']))))
                for name,t in poses.items():
                    if name=='gripper_link':continue
                    bid=m.body(name).id
                    errors['visual_fk_m'].append(float(np.max(np.abs(t[:3,3]-d.xpos[bid]))))
                    errors['visual_fk_rotation'].append(float(np.max(np.abs(t[:3,:3]-d.xmat[bid].reshape(3,3)))))
            state=np.r_[mass.ravel(),gravity,rnea,p,rot.ravel()]
            if mesh_path:visual_delta.append(float(np.max(np.abs(state-baselines[index]))))
            else:baselines.append(state)
        if visual and mesh_path:
            vertices={}
            for v in visual.visuals:
                raw=v['path'].read_bytes();count=int.from_bytes(raw[80:84],'little')
                dtype=np.dtype([('n','<f4',3),('v','<f4',(3,3)),('a','<u2')])
                vertices[v['path']]=np.unique(np.frombuffer(raw,offset=84,count=count,dtype=dtype)['v'].reshape(-1,3),axis=0)*v['scale']
            for row in refs['samples'][:8]:
                d.qpos[qids]=row['q'];poses=visual.poses(row['q'])
                for slide in [0.,-.02,-.04]:
                    for side in ['right','left']:d.qpos[m.jnt_qposadr[m.joint('jaw_'+side).id]]=slide
                    mj.mj_forward(m,d);indices={}
                    for v in visual.visuals:
                        link=v['link'];i=indices.get(link,0);indices[link]=i+1
                        if link=='gripper_link':continue
                        side=moving_side(v['path'].name) if link=='l5_l6_urdf_asm' else 0
                        if side:
                            name='pad_'+('right' if side==1 else 'left') if v['path'].name in ('hand_r.stl','part_15.stl') else 'moving_'+str(i)
                        else:name='visual_'+link+'_'+str(i)
                        gid=m.geom('display_'+name).id;mid=m.geom_dataid[gid]
                        a=int(m.mesh_vertadr[mid]);b=a+int(m.mesh_vertnum[mid])
                        actual=m.mesh_vert[a:b]@d.geom_xmat[gid].reshape(3,3).T+d.geom_xpos[gid]
                        t=visual_transform(v,poses,dict(right=slide,left=slide))
                        expected=vertices[v['path']]@t[:3,:3].T+t[:3,3]
                        errors['visual_world_bounds_m'].append(float(max(np.max(np.abs(actual.min(0)-expected.min(0))),np.max(np.abs(actual.max(0)-expected.max(0))))))
    maxima={k:max(v) if v else None for k,v in errors.items()}
    passed=all(x is None or x<(1e-7 if k=='visual_world_bounds_m' else 1e-8) for k,x in maxima.items()) and max(visual_delta or [0])<1e-12
    return dict(status='PASS_CROSS_ENGINE_MODEL_CONSISTENCY' if passed else 'FAIL_CROSS_ENGINE_MODEL_CONSISTENCY',
                samples=len(refs['samples']),max_errors=maxima,visual_mesh_physics_max_delta=max(visual_delta or [0]),
                vendor_visual_parts=len(visual.visuals) if visual else 0,model_id=refs['model_id'],inertia_sha256=refs['inertia_sha256'],
                visual_jaw_openings_tested_m=[.0934,.0534,.0134],motor_commands_sent=0,scope='Dynamics at CAD-open; independent world mesh bounds at three openings; not physical identification',
                unvalidated=['PLA stiffness/backlash','motor torque constant and latency','friction','payload','jaw geometry/contact and closed inertia'])


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('mode',choices=['reference','check'])
    p.add_argument('--runtime',type=Path);p.add_argument('--samples',type=int,default=32)
    p.add_argument('--reference',type=Path);p.add_argument('--urdf',type=Path);p.add_argument('--inertia',type=Path);p.add_argument('--meshes',type=Path)
    p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    if a.output.exists():raise SystemExit('PRESERVE_EXISTING_OUTPUT')
    result=reference(a.runtime,a.samples) if a.mode=='reference' else check(a.reference,a.urdf,a.inertia,a.meshes)
    a.output.parent.mkdir(parents=True,exist_ok=True);a.output.write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps({k:v for k,v in result.items() if k!='samples'},indent=2) if a.mode=='check' else 'Saved independent Pinocchio reference')
    if result.get('status','PASS').startswith('FAIL'):raise SystemExit(1)


if __name__=='__main__':main()
