"""Offline full-CAD sampled audit of wrist observation and recovery paths."""
import argparse
import json
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
import mujoco as mj
from .tool_geometry import width_to_slide


def check(out, config):
    out = Path(out)
    job = json.loads((out/'job_pending_geometry.json').read_text())
    data = json.loads((out/'collision_samples.json').read_text())
    xml = ET.parse(config['geometry_xml'])
    world = xml.find('worldbody')
    for geom in list(world.findall('geom')):
        if geom.get('name', '').startswith(('observed_tray_envelope_', 'observed_nut_envelope')):
            world.remove(geom)
    from .geometry_cache import load_model
    model, model_cache_hit = load_model(mj, ET.tostring(xml.getroot(), encoding='unicode'))
    state = mj.MjData(model)
    joints = [model.jnt_qposadr[model.joint(f'L{i}_joint').id] for i in range(1,7)]
    jaws = [model.jnt_qposadr[model.joint('jaw_'+s).id] for s in ('left', 'right')]
    jaw_width = .0934 if job['start_motor_rad'][6] < .01 else (.040 if job['start_motor_rad'][6] < 1.62 else .002)
    state.qpos[jaws] = width_to_slide(jaw_width)
    n = np.array(config['trays']['A']['plane']['normal_base'])
    offset = config['trays']['A']['plane']['offset_base_m']
    start = np.array(data['start_rad'][:6])
    known = 'l2_l3_urdf_asm|l5_l6_urdf_asm'
    initial = {}
    unexpected = []
    camera_clearance = float('inf')
    for index, row in enumerate(data['samples']):
        q = np.array(row['q_rad'][:6])
        state.qpos[joints] = q
        mj.mj_kinematics(model, state)
        mj.mj_comPos(model, state)
        mj.mj_collision(model, state)
        for contact in state.contact[:state.ncon]:
            if contact.dist >= -1e-6:
                continue
            bodies = '|'.join(sorted(model.body(model.geom_bodyid[int(g)]).name
                                    for g in (contact.geom1, contact.geom2)))
            depth = float(-contact.dist)
            if index == 0:
                initial[bodies] = max(initial.get(bodies, 0), depth)
            allow = (np.max(np.abs(q-start)) < .10 and bodies == known
                     and depth <= initial.get(known, 0)+.00002)
            if not allow:
                unexpected.append(dict(sample=index, branch=row['branch'], task=row['name'],
                                       bodies=bodies, depth_m=depth))
        # A deliberately conservative 75mm sphere around the end bounds the
        # new camera housing against the support plane. Mount self-clearance is
        # not inferred from this approximation; arm CAD self contacts stay on.
        end = np.array(row['T_base_end'])[:3,3]
        clearance = float(n@end+offset-.075)
        camera_clearance = min(camera_clearance, clearance)
        if clearance < .005:
            unexpected.append(dict(sample=index, reason='camera_proxy_floor_clearance', value_m=clearance))
        if len(unexpected) > 20:
            break
    result = dict(model_cache_hit=model_cache_hit, status='PASS' if not unexpected else 'FAIL', unexpected=unexpected,
                  trajectory_sha256=data['trajectory_sha256'], samples_checked=index+1,
                  initial_contacts=initial, wrist_camera_proxy_checked=True,
                  camera_proxy_end_radius_m=.075, camera_proxy_min_floor_clearance_m=camera_clearance,
                  box_walls_checked=False, box_wall_policy=config['box_wall_policy'],
                  jaw_width_geometry_m=jaw_width,
                  scope='Sampled full arm CAD self/table check at observed endpoint aperture geometry; 75mm end sphere versus floor for new camera. Camera-specific self-collision/cable geometry not measured. No continuous proof.')
    (out/'route_geometry_report.json').write_text(json.dumps(result, indent=2))
    job['geometry_audit'] = result
    (out/'job.json').write_text(json.dumps(job, indent=2))
    print(json.dumps(result))
    if unexpected:
        raise SystemExit(2)


if __name__ == '__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out',required=True)
    parser.add_argument('--config',required=True)
    args=parser.parse_args()
    check(args.out,json.loads(Path(args.config).read_text()))
