"""Exhaustive native vs independent fixed-point checks on actual motion banks."""
import argparse,ctypes,json,pickle,time
from pathlib import Path
from gravity_trajectory import fixed_sample

if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--library',required=True);p.add_argument('--banks',nargs='+',required=True);p.add_argument('--output',required=True);a=p.parse_args()
 lib=ctypes.CDLL(str(Path(a.library).resolve()));lib.native_fields.argtypes=[ctypes.POINTER(ctypes.c_int32),ctypes.c_uint32,ctypes.POINTER(ctypes.c_int32)]
 rows=[]
 for name in a.banks:
  bank=pickle.loads(Path(name).read_bytes());checks=0;begin=time.monotonic()
  for record in bank['records']:
   w=record['words'];arr=(ctypes.c_int32*220)(*w);out=(ctypes.c_int32*21)();n=w[217]
   for tick in range(n+1):
    assert lib.native_fields(arr,tick,out)==0
    phase=tick*16777216//n if tick<n else 16777216
    for field in range(21):
     assert out[field]==fixed_sample(w[8*field:8*field+8],phase),(name,tick,field)
     checks+=1
  rows.append(dict(path=bank['path'],reference_sha256=bank['reference_sha256'],field_checks=checks,seconds=time.monotonic()-begin,status='PASS'))
 result=dict(status='PASS',banks=rows,scope='Actual 200Hz phases, native C++ vs independent Python integer HLS arithmetic reference; no motor access')
 Path(a.output).write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
