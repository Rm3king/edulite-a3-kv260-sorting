"""Offline wrist-camera observation poses; SDK environment, no hardware I/O."""
import argparse
import json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from joint_trajectory import JointPlanner, Q24
from live_motion import current_reference
from motion_runtime import MotionRuntime, pin, pose_values
from septic_job import SCHEMA
from septic_trajectory import movej_c3, septic_words, extrema, derivative
from continuous_transfer import path_digest
from wrist_handeye_motion import POLICY


def plan(out, config, staging=None):
    out = Path(out)
    p = JointPlanner(out/'profile.json')
    kin = MotionRuntime()
    cap = json.loads((out/'board_capture.json').read_text())
    values = [[x['decoded']['position_motor_rad'] for x in cap['transactions']
               if x['motor'] == i and x.get('decoded', {}).get('valid')] for i in range(1, 8)]
    if any(len(v) != 3 or max(v)-min(v) > .002 for v in values):
        raise ValueError('Fresh stable stopped feedback required')
    actual = [v[-1] for v in values]
    start, adjustment = current_reference(p, actual)
    center = np.array(json.loads((out/'board_pose_d435.json').read_text())['board_center_base_m'])
    hint = json.loads(Path(staging).read_text()) if staging else None
    if hint:
        center = np.array(hint['board_center_base_m'])
        camera = np.array(hint['T_end_camera_staging_only'])
    n = np.array(config['trays']['A']['plane']['normal_base'])
    n /= np.linalg.norm(n)
    x = kin.fk(start[:6]).rotation[:, 0]
    x -= n*np.dot(x, n)
    x /= np.linalg.norm(x)
    reference = np.column_stack([x, np.cross(n, x), n])@Rotation.from_euler('z', -45, degrees=True).as_matrix()
    if hint:
        # Aim the estimated optical axis at the board normal before varying poses.
        axis = camera[:3,2]
        turn, _ = Rotation.align_vectors(np.array([[0.,0.,-1.]]), axis[None,:])
        reference = reference@turn.as_matrix()
    # Twelve fit views followed by five held-out views, fixed before acquisition.
    poses = [(0,0,0,0), (12,0,0,0), (-12,0,0,0), (0,12,0,0), (0,-12,0,0),
             (0,0,15,0), (0,0,-15,0), (9,9,0,.015), (-9,9,0,0),
             (9,-9,0,0), (-9,-9,0,.015), (0,0,0,.025),
             (6,-11,7,.01), (-7,10,-8,.01), (11,5,-10,.005),
             (-10,-6,10,.015), (0,0,8,.02)]
    current = list(start)
    tasks = []
    recoveries = {}
    planned = []
    for index, (rx, ry, rz, dz) in enumerate(poses):
        candidates = []
        for scale in (1., .75, .5):
            angles = [v*scale for v in (rx, ry, rz)]
            rotation = reference@Rotation.from_euler('xyz', angles, degrees=True).as_matrix()
            end = center+rotation[:, 2]*(.19+dz*scale)
            if hint:
                end = center-rotation@(camera[:3,3]+camera[:3,:3]@np.array([0.,0.,.24+dz*scale]))
            target = pose_values(pin.SE3(rotation, end))
            for guess in [current[:6]]+config['ik_seeds']+[config['ik_seed']]:
                try:
                    q = kin.ik(target, np.array(guess)).tolist()+[start[6]]
                    candidates.append(q)
                except ValueError:
                    pass
            if candidates:
                break
        if not candidates:
            raise ValueError('No IK for calibration pose '+str(index))
        q = min(candidates, key=lambda v: np.linalg.norm(np.array(v[:6])-current[:6]))
        name = f'pose_{index:02d}'
        task = dict(name=name, capture=True, split='fit' if index < 12 else 'holdout',
                    goal_rad=q, segments=movej_c3(p, current, q, 1.5))
        tasks.append(task)
        # Every endpoint-to-central-hover return is separately geometry checked.
        route = [q, tasks[0]['goal_rad'], start] if index else [q, start]
        segments = []
        for qa, qb in zip(route, route[1:]):
            segments.extend(movej_c3(p, qa, qb, 1.5))
        recoveries[name] = dict(name=name+'_return', start_rad=q, goal_rad=start,
                                return_to_live_start=True, segments=segments)
        planned.append(dict(name=name, split=task['split'], angles_deg=angles, requested_angles_deg=[rx,ry,rz],
                            T_base_end=kin.fk(q[:6]).homogeneous.tolist(), q_rad=q))
        current = q
    # Return via the central hover and the reverse of the departure.
    home = recoveries[tasks[-1]['name']]
    tasks.append(dict(name='return_supported', capture=False, goal_rad=start,
                      return_to_live_start=True, segments=home['segments']))
    job = dict(schema=SCHEMA, policy=POLICY, profile_id=p.profile_id, model_id=kin.model_id,
               motion_mask=127, start_rad=start, start_motor_rad=actual,
               start_reference_adjustment_motor_rad=adjustment,
               startup_no_reset=True, hardware_start_verified=True, tasks=tasks,
               recoveries=recoveries, geometry_audit=dict(status='PENDING'))
    if hint:
        job['initial_capture_review'] = True
    lo = np.full(7, np.inf)
    hi = np.full(7, -np.inf)
    rows = []
    for branch, seq in [('main', tasks)]+[(name, [b]) for name, b in recoveries.items()]:
        for task in seq:
            for segment in task['segments']:
                words = septic_words(segment, p)[0]
                for i in range(7):
                    low, high = extrema(words[8*i:8*i+8])
                    lo[i] = min(lo[i], low/Q24)
                    hi[i] = max(hi[i], high/Q24)
                coefficients = np.array(words[:56]).reshape(7, 8)/Q24
                coefficients *= np.array(p.direction)[:, None]
                coefficients[:, 0] -= np.array(p.offset)*p.direction
                for u in np.linspace(0, 1, max(3, int(np.ceil(segment['duration_s']/.05))+1)):
                    qr = [derivative(row, float(u)) for row in coefficients]
                    tf = kin.fk(qr[:6])
                    rows.append(dict(branch=branch, name=task['name'], q_rad=qr,
                                     T_base_end=tf.homogeneous.tolist()))
    job['motor_lower_rad'] = []
    job['motor_upper_rad'] = []
    for i in range(7):
        low, high = sorted((p.lower[i]*p.direction[i]+p.offset[i],p.upper[i]*p.direction[i]+p.offset[i]))
        job['motor_lower_rad'].append(max(low, float(lo[i])-.02))
        job['motor_upper_rad'].append(min(high, float(hi[i])+.02))
    (out/'job_pending_geometry.json').write_text(json.dumps(job, indent=2))
    (out/'planned_views.json').write_text(json.dumps(dict(poses=planned,board_center_base_m=center.tolist(),
        camera_mount_guess_only='Partial-grid staging estimate, not calibration' if hint else 'optical axis approximately -end Z; camera offset not yet calibrated'),indent=2))
    (out/'collision_samples.json').write_text(json.dumps(dict(start_rad=start,trajectory_sha256=path_digest(job),samples=rows)))
    print(json.dumps(dict(poses=len(poses),samples=len(rows),nominal_motion_s=sum(s['duration_s'] for t in tasks for s in t['segments']))))


if __name__ == '__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out',required=True)
    parser.add_argument('--config',required=True)
    parser.add_argument('--staging',help='Unverified camera mounting estimate for framing only')
    args=parser.parse_args()
    plan(args.out,json.loads(Path(args.config).read_text()),args.staging)
