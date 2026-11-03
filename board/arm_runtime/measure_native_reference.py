"""Numerical baseline timing only: no device opened or motor command sent."""
import argparse,ctypes,json,pickle,time
from pathlib import Path
import numpy as np

if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--bank',required=True);p.add_argument('--output',required=True);a=p.parse_args()
 bank=pickle.loads(Path(a.bank).read_bytes());lib=ctypes.CDLL(str(Path(__file__).with_name('libnative_reference.so')))
 lib.native_packets.argtypes=[ctypes.POINTER(ctypes.c_int32),ctypes.c_uint32,ctypes.POINTER(ctypes.c_uint32)]
 records=bank['records'];arrays=[(ctypes.c_int32*220)(*r['words']) for r in records];out=(ctypes.c_uint32*21)();times=[]
 for k in range(10200):
  i=k%len(records);tick=(k//len(records))%(records[i]['words'][217]+1)
  begin=time.perf_counter_ns();code=lib.native_packets(arrays[i],tick,out);duration=time.perf_counter_ns()-begin
  if code:raise RuntimeError(code)
  if k>=200:times.append(duration/1000)
 result=dict(status='PASS_NUMERICAL_ONLY_NO_MMIO',samples=len(times),
   native_21_channel_evaluate_and_pack_with_ffi_us={k:float(np.percentile(times,v)) for k,v in [('p50',50),('p95',95),('p99',99),('max',100)]},
   reference_sha256=bank['reference_sha256'],scope='Optimized exact-integer C++ plus Python FFI call; no MMIO, scheduling, CAN or motor time')
 Path(a.output).write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
