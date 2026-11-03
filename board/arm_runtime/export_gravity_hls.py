#!/usr/bin/python3
"""Export every encoded q/v/g field at its actual 5 ms phase grid."""
import argparse,json
from pathlib import Path
from joint_trajectory import JointPlanner,Q24
from motion_runtime import MotionRuntime
from gravity_trajectory import GravityModel,prepare_stream,validate_records

if __name__=='__main__':
    ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('--job',required=True);ap.add_argument('--profile',required=True);ap.add_argument('--output',required=True);ap.add_argument('--tracking-profile');a=ap.parse_args()
    j=json.loads(Path(a.job).read_text());p=JointPlanner(a.profile);m=GravityModel(MotionRuntime(),p,a.tracking_profile)
    records=prepare_stream(m,j['tasks']);validate_records(records,j,p,m)
    cases=[(r['words'][field*56:field*56+56],r['words'][217]) for r in records for field in range(3)]
    rows=['// Encoded gravity q/v/tau: normalized first six, raw radian gripper q.',f'const int wide_case_count={len(cases)};',
        'const int wide_ticks[]={'+','.join(str(n) for _,n in cases)+'};',
        'const double wide_lower[7]={0,0,0,0,0,0,0};','const double wide_upper[7]={1,1,1,1,1,1,2};',
        'const double wide_coefficients[][7][8]={']
    for w,_ in cases:rows.append('{'+','.join('{'+','.join(format(x/Q24,'.17g') for x in w[i*8:i*8+8])+'}' for i in range(7))+'},')
    rows.append('};');Path(a.output).write_text('\n'.join(rows)+'\n')
    print('Exported',len(cases),'fields;',sum(n+1 for _,n in cases),'actual 5 ms samples')
