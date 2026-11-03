#!/usr/bin/python3
"""Export original and transmitted reviewed C3 segments to HLS C/RTL tests."""
import argparse,sys
from pathlib import Path
root=Path(__file__).resolve().parents[2];sys.path.insert(0,str(root/'contest/arm_runtime'))
import json
from joint_trajectory import JointPlanner,Q24
from septic_trajectory import septic_words
from septic_transport import SepticTransport
from septic_job import verify_septic_job

ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('--job',required=True);ap.add_argument('--profile',required=True);ap.add_argument('--output',required=True);a=ap.parse_args()
job=json.loads(Path(a.job).read_text());p=JointPlanner(a.profile);verify_septic_job(job,p)
class Sink:
    def read(self,a):return {0x470:0x50001,0x828:0x70008,0x404:7,0x800:1}.get(a,0)
    def write(self,a,v):pass
t=SepticTransport(Sink(),p);cases=[]
for task in job['tasks']:
    for plan in task['segments']:
        t.submit(plan)
        for segment in (plan,t.last_encoded_plan):
            w,_,_,_=septic_words(segment,p);cases.append(w)
def doubles(values):return ','.join(format(x/Q24,'.17g') for x in values)
out=Path(a.output)
rows=['// Generated from a reviewed ARM C3 job; original and endpoint-deduplicated segments.',
    f'const int wide_case_count={len(cases)};',
    'const int wide_ticks[] = {'+','.join(str(w[70]) for w in cases)+'};',
    'const double wide_lower[7]={'+doubles(cases[0][56:63])+'};',
    'const double wide_upper[7]={'+doubles(cases[0][63:70])+'};',
    'const double wide_coefficients[][7][8]={']
for w in cases:rows.append('{'+','.join('{'+doubles(w[i*8:i*8+8])+'}' for i in range(7))+'},')
rows.append('};');out.write_text('\n'.join(rows)+'\n')
print('Exported',len(cases),'cases,',sum(w[70]+1 for w in cases),'actual 5 ms phase samples')
