#!/usr/bin/python3
"""Offline dynamics audit. Never imports a CAN driver or enables a motor.

Checks model consistency, not the correctness of the real robot's mass/COM,
torque calibration, payload, friction or mounting orientation.
"""
import argparse,hashlib,json,time
from pathlib import Path
import numpy as np
import pinocchio as pin
import yaml
import xml.etree.ElementTree as ET
from motion_runtime import MotionRuntime
from joint_trajectory import JointPlanner

def audit(profile,job_path):
    runtime=MotionRuntime();kin=runtime.kin;model=kin.model;p=JointPlanner(profile)
    cfgpath=Path(__file__).with_name('model')/'inertia_params.yaml'
    cfg=yaml.safe_load(cfgpath.read_text());mapping=[]
    urdf=ET.parse(cfgpath.parent/'el_a3_legacy.urdf').getroot()
    joint_links={j.attrib['name']:j.find('child').attrib['link'] for j in urdf.findall('joint')}
    for link,entry in cfg['inertia_params'].items():
        urdf_link=joint_links[link+'_joint']
        frame=model.frames[model.getFrameId(urdf_link)];index=frame.parent;body=model.inertias[index]
        expected=model.getJointId(link+'_joint')
        if index!=expected or abs(body.mass-entry['mass'])>1e-12 or np.max(np.abs(body.lever-entry['com']))>1e-12:
            raise ValueError('calibrated inertia mapping mismatch: '+link)
        mapping.append(dict(calibration_link=link,urdf_link=urdf_link,joint_index=int(index),joint_name=model.names[index],mass_kg=body.mass,com_m=body.lever.tolist()))
    job=json.loads(Path(job_path).read_text());samples=[]
    for task in job['tasks']:
        for segment in task['segments'][::max(1,len(task['segments'])//16)]:samples.append(segment['start_rad'][:6])
    residuals=[];timings=[];torques=[];rnea_static=[];power=[]
    for q in samples:
        qp=kin._to_pinocchio_q(q);zero=np.zeros(model.nv)
        stamp=time.perf_counter_ns();g=np.array(kin.compute_gravity(q));timings.append((time.perf_counter_ns()-stamp)/1000)
        tau=pin.rnea(model,model.createData(),qp,zero,zero)
        rnea_static.append(float(np.max(np.abs(tau-g))))
        if not np.isfinite(g).all():raise ValueError('nonfinite gravity')
        gradient=[]
        for axis in range(6):
            dv=zero.copy();dv[axis]=1e-6
            plus=pin.integrate(model,qp,dv);minus=pin.integrate(model,qp,-dv)
            gradient.append((pin.computePotentialEnergy(model,model.createData(),plus)-pin.computePotentialEnergy(model,model.createData(),minus))/2e-6)
        residuals.append(float(np.max(np.abs(np.array(gradient)-g[:6]))))
        speed=np.linspace(-.1,.1,6);direction=np.array(p.direction[:6])
        power.append(abs(float(g[:6]@speed-(g[:6]*direction)@(speed*direction))))
        torques.append(g[:6].tolist())
    # The vendor wrapper pads q to 7 but directly passes v/a. Explicitly report
    # whether its apparent six-axis signature also accepts six velocity values.
    try:
        kin.inverse_dynamics(samples[0],[0.]*6,[0.]*6);six_axis_inverse_dynamics='ACCEPTED'
    except ValueError as exc:six_axis_inverse_dynamics='REQUIRES_EXPLICIT_7_DOF_V_A_ADAPTER: '+str(exc).split('\n')[0]
    if max(residuals)>1e-6 or max(rnea_static)>1e-12 or max(power)>1e-12:raise ValueError('dynamics consistency check failed')
    return dict(status='PASS_OFFLINE_MODEL_CONSISTENCY_ONLY',physical_compensation='NOT_RUN',motion_commands_sent=0,
        model_id=runtime.model_id,inertia_sha256=hashlib.sha256(cfgpath.read_bytes()).hexdigest(),
        inertia_calibration_metadata=cfg['calibration_info'],calibration_scope='vendor supplied; not newly measured on this robot',
        inertia_mapping=mapping,samples=len(samples),potential_gradient_max_error_nm=max(residuals),
        static_rnea_max_error_nm=max(rnea_static),motor_robot_power_max_error_w=max(power),
        maximum_abs_gravity_nm=np.max(np.abs(torques),axis=0).tolist(),
        gravity_runtime_us={k:float(np.percentile(timings,v)) for k,v in [('p50',50),('p95',95),('p99',99),('max',100)]},
        vendor_six_axis_inverse_dynamics=six_axis_inverse_dynamics,
        gripper_model_state='vendor q6->q7 padding uses 0; actual jaw/mass effect not validated',
        next_required=['motor torque units/sign and Kt/current mapping','real static torque comparison across supported poses',
            'mounting gravity direction and payload verification','new Type1 PL mode/limits/lease validation before physical use'])

if __name__=='__main__':
    ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('--profile',required=True);ap.add_argument('--job',required=True);ap.add_argument('--output',required=True);a=ap.parse_args()
    result=audit(a.profile,a.job);Path(a.output).write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
