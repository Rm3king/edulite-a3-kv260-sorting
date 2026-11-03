"""Attended six-axis visual hover/return. Default validates without hardware.

Uses the commissioned arm-only owner and its cached-return/retain-until-supported
policy. Never enables or writes the gripper. Separate from automatic picking.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
from joint_trajectory import JointPlanner, Q24
from septic_trajectory import movej_c3, extrema
from septic_job import verify_septic_job
from calibration_motion import execute

POLICY = 'visual_single_screw_hover_return_v1'


def verify_hover(job, p):
    if job.get('hover_policy') != POLICY or job.get('motion_mask') != 63:
        raise ValueError('Six-axis hover policy required')
    if job.get('unused_channel_7_rad') != 0 or len(job.get('start_motor_rad', [])) != 6:
        raise ValueError('Gripper must be excluded')
    if not .06 <= job.get('hover_clearance_m', 0) <= .15:
        raise ValueError('Hover must remain >=60mm above the estimated surface')
    route = job['route']; waypoints = [r['q_rad'] for r in route]
    if not 4 <= len(route) <= 20 or waypoints[0] != job['start_rad']:
        raise ValueError('Bounded route starting at observed pose required')
    for q in waypoints:
        p.positions(q)
        if q[6] != 0:
            raise ValueError('No gripper trajectory permitted')
    start = waypoints[0]
    elbow = list(start); elbow[2] -= math.radians(30)
    lifted = list(elbow); lifted[1] += math.radians(20)
    if any(abs(a-b)>1e-10 for a,b in zip(waypoints[1], elbow)) or any(abs(a-b)>1e-10 for a,b in zip(waypoints[2], lifted)):
        raise ValueError('Reviewed elbow-first departure required')
    if len(job['tasks']) != 2 or job['tasks'][1]['goal_rad'] != start or not job['tasks'][1].get('return_to_live_start'):
        raise ValueError('Exactly one out-and-supported-return pair required')
    if not job.get('sample_at_out'):
        raise ValueError('Stopped camera capture handshake required')
    for task, route in [(job['tasks'][0],waypoints),(job['tasks'][1],waypoints[::-1])]:
        expected = []
        for a,b in zip(route,route[1:]):
            expected.extend(movej_c3(p,a,b,job['minimum_leg_s']))
        if task['segments'] != expected or task['goal_rad'] != route[-1]:
            raise ValueError('Trajectory differs from collision-reviewed route')
    if not 1.5 <= job['minimum_leg_s'] <= 10:
        raise ValueError('Invalid minimum leg duration')
    audit = job['geometry_audit']
    if audit.get('status') != 'PASS_SAMPLED_WITH_MONOTONIC_EXISTING_START_CONTACT_DEPARTURE' or audit.get('unexpected'):
        raise ValueError('Geometry audit rejected')
    if audit['route_sha256'] != job['route_file_sha256'] or not audit.get('box_walls_checked'):
        raise ValueError('Geometry provenance mismatch')
    # Bind embedded route to the exact file that the geometry checker inspected.
    raw = job['route_file_text']
    if hashlib.sha256(raw.encode()).hexdigest() != job['route_file_sha256'] or json.loads(raw)['waypoints'] != job['route']:
        raise ValueError('Route changed after geometry review')
    prepared = verify_septic_job(job,p,return_prepared=True)
    for i in range(6):
        legal = sorted((p.lower[i]*p.direction[i]+p.offset[i],p.upper[i]*p.direction[i]+p.offset[i]))
        lo,hi = job['motor_lower_rad'][i],job['motor_upper_rad'][i]
        if not legal[0] <= lo < hi <= legal[1]:
            raise ValueError('Guard bounds outside joint limits')
        for record in prepared:
            a,b = extrema(record['words'][8*i:8*i+8])
            if a/Q24 < lo or b/Q24 > hi:
                raise ValueError('Quantized segment escapes guard bounds')
    return prepared


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--job',type=Path,required=True)
    ap.add_argument('--profile',type=Path,required=True)
    ap.add_argument('--out',type=Path,required=True)
    ap.add_argument('--execute-attended',action='store_true')
    a=ap.parse_args();job=json.loads(a.job.read_text());p=JointPlanner(a.profile)
    prepared=verify_hover(job,p)
    if not a.execute_attended:
        print(json.dumps(dict(status='VALIDATED_NO_HARDWARE',segments=len(prepared))))
        return
    result=execute(job,p,a.out,job_verifier=verify_hover)
    if result['status']=='PASS_SIX_AXIS_WRIST_RETURN_ONLY':
        result['status']='PASS_VISUAL_HOVER_RETURN_JOINT_EXECUTION_ONLY'
    result['physical_target_alignment_verified']=False
    (a.out/'result.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps({k:result.get(k) for k in ['status','reset_confirmed','stop_errors','gripper_control_commands']}))


if __name__=='__main__': main()
