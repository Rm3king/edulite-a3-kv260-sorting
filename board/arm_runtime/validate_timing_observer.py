"""Actual PL statistics vs independently read timestamps; physical TX inhibited."""
import argparse,json,time
from pathlib import Path
from motion_executor import CanDevice
from control_benchmark import timing_read,BINS

if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--output',required=True);a=p.parse_args()
 d=CanDevice();stamps=[]
 try:
  assert d.read(0x404)==0 and d.read(0x5a8)==0
  assert d.read(0x100)==0x54494d31
  d.configure('internal');d.write(0x104,3);first=(d.read(0x408)+5000)&0xffffffff
  for k in range(20):
   d.heartbeat();d.enqueue(0x01800001,bytes.fromhex('8000800012345678'),due_us=(first+k*5000)&0xffffffff,ttl_us=4500)
   end=time.monotonic()+.1
   while time.monotonic()<end:
    d.heartbeat();f=d.receive()
    if f:
     assert f['id']==0x01800001
     stamps.append(d.read(0x430));break
    time.sleep(.0002)
   else:raise TimeoutError('internal timing frame')
  result=timing_read(d);row=result['axes'][0]
  dt=[(b-a)&0xffffffff for a,b in zip(stamps,stamps[1:])];err=[abs(t-5000) for t in dt]
  assert row['frames']==20 and row['intervals']==19
  assert row['min_interval_us']==min(dt) and row['max_interval_us']==max(dt)
  assert row['max_abs_period_error_us']==max(err)
  hist=[0]*16
  for value in err:
   for i,lim in enumerate(BINS):
    if lim is None or value<=lim:hist[i]+=1;break
  assert row['histogram']==hist
  assert all(x['frames']==0 for x in result['axes'][1:])
  assert d.read(0x404)==15 and d.read(0x5a8)==0 and d.read(0x418)==0
  result.update(status='PASS_ACTUAL_PL_INTERNAL_ONLY',physical_tx_inhibited=True,motor_enable_commands=0,
    independent_command_request_timestamps_us=stamps)
 finally:d.stop();d.close()
 Path(a.output).write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
