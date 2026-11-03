#!/usr/bin/python3
"""Compute-only hand-guiding benchmark; never opens hardware."""
import argparse,json,time
from pathlib import Path
import numpy as np
from freedrive import Policy,DirectModel
from joint_trajectory import JointPlanner
from motion_runtime import MotionRuntime

if __name__=='__main__':
 ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('--capture',required=True);ap.add_argument('--profile',required=True);ap.add_argument('--output',required=True);a=ap.parse_args()
 capture=json.loads(Path(a.capture).read_text());p=JointPlanner(a.profile);m=DirectModel(MotionRuntime(),p);policy=Policy(p,m)
 raw={t['motor']:t['decoded']['position_motor_rad'] for t in capture['transactions'] if t.get('decoded',{}).get('valid')}
 if sorted(raw)!=list(range(1,8)):raise ValueError('all seven captured positions required')
 timings=[]
 for k in range(3000):
  now=10.+k*.01;fs=[dict(motor=i,time=now,mode=2,fault=0,position=raw[i]) for i in range(1,8)]
  begin=time.perf_counter_ns();state=policy.step(fs,now);timings.append((time.perf_counter_ns()-begin)/1000)
 r=dict(status='PASS_COMPUTE_ONLY_NOT_PHYSICAL_FREEDRIVE',hardware_opened=False,enable_commands_sent=0,
  scope='Captured actual positions, synthetic fresh Mode2/timing; no hardware enable or real-time guarantee',samples=len(timings),
  compute_us=dict(p50=float(np.percentile(timings,50)),p99=float(np.percentile(timings,99)),maximum=float(max(timings))),kp=state['kp'])
 Path(a.output).write_text(json.dumps(r,indent=2)+'\n');print(json.dumps(r,indent=2))
