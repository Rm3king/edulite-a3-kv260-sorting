"""Native whole-segment guard versus independent Python integer phases."""
import ctypes
import random
import subprocess
import tempfile
import unittest
from pathlib import Path
from gravity_trajectory import fixed_sample

class NativeValidationTests(unittest.TestCase):
 @classmethod
 def setUpClass(cls):
  cls.temp=tempfile.TemporaryDirectory();p=Path(cls.temp.name)/'arithmetic.so'
  subprocess.run(['g++','-O3','-Wall','-Wextra','-Werror','-shared','-fPIC',str(Path(__file__).with_name('native_reference.cpp')),'-o',str(p)],check=True)
  cls.lib=ctypes.CDLL(str(p));cls.f=cls.lib.native_validate_record;cls.f.argtypes=[ctypes.POINTER(ctypes.c_int32),ctypes.POINTER(ctypes.c_int32),ctypes.c_double,ctypes.c_double,ctypes.c_int,ctypes.POINTER(ctypes.c_uint32)]
 @classmethod
 def tearDownClass(cls):cls.temp.cleanup()
 def expected(self,w,b,clo,chi,rest):
  if not 2<=w[217]<=120000 or not 0<=w[216]<=512:return -5,0
  n=w[217];checked=0
  for tick in range(n+1):
   phase=tick*(1<<24)//n;v=[fixed_sample(w[i*8:i*8+8],phase) for i in range(21)]
   if any(not w[168+i//7*14+i%7]<=x<=w[175+i//7*14+i%7] for i,x in enumerate(v)):return -1,checked
   for f in range(3):
    for i in range(6):
     code=(v[7*f+i]*65535+(1<<23))>>24;j=2*(6*f+i)
     if not b[j]<=code<=b[j+1]:return -2,checked
   if not clo<=v[6]/(1<<24)<=chi:return -3,checked
   checked+=1
  if rest and any((v[7+i]*65535+(1<<23))>>24!=32768 for i in range(6)):return -4,checked
  return 0,checked
 def test_randomized_valid_and_each_failure_match_python(self):
  rng=random.Random(261020)
  for _ in range(240):
   w=[0]*220
   for i in range(21):w[i*8:i*8+8]=[1<<23]+[rng.randrange(-10000,10000) for _ in range(7)]
   for f in range(3):
    for i in range(7):w[175+14*f+i]=1<<24
   w[217]=rng.randrange(2,18);b=[x for _ in range(18) for x in (0,65535)];lo,hi=0.,2.;rest=rng.randrange(2)
   kind=rng.randrange(7)
   if kind==1:w[175]=0
   elif kind==2:b[:2]=[0,30000]
   elif kind==3:hi=.1
   elif kind==4:w[217]=1
   elif kind==6:lo=float('nan')
   checked=ctypes.c_uint32();rc=self.f((ctypes.c_int32*220)(*w),(ctypes.c_int32*36)(*b),lo,hi,rest,ctypes.byref(checked))
   self.assertEqual((rc,checked.value),self.expected(w,b,lo,hi,rest))

if __name__=='__main__':unittest.main()
