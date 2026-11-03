"""Native/independent integer agreement, including signed halfway rounding."""
import ctypes,random,subprocess,tempfile,unittest
from pathlib import Path
from gravity_trajectory import fixed_sample

class NativeReferenceTests(unittest.TestCase):
 @classmethod
 def setUpClass(cls):
  cls.tmp=tempfile.TemporaryDirectory();lib=Path(cls.tmp.name)/'native.so'
  subprocess.run(['g++','-O3','-Wall','-Wextra','-Werror','-shared','-fPIC',str(Path(__file__).with_name('native_reference.cpp')),'-o',str(lib)],check=True)
  cls.lib=ctypes.CDLL(str(lib));cls.lib.native_horner.argtypes=[ctypes.POINTER(ctypes.c_int32),ctypes.c_uint32];cls.lib.native_horner.restype=ctypes.c_int32
  cls.lib.native_fields.argtypes=[ctypes.POINTER(ctypes.c_int32),ctypes.c_uint32,ctypes.POINTER(ctypes.c_int32)]
  cls.lib.native_packets.argtypes=[ctypes.POINTER(ctypes.c_int32),ctypes.c_uint32,ctypes.POINTER(ctypes.c_uint32)]
  cls.lib.native_submit.argtypes=[ctypes.c_void_p,ctypes.POINTER(ctypes.c_int32),ctypes.c_uint32,ctypes.c_uint32]
  cls.lib.native_upload.argtypes=[ctypes.c_void_p,ctypes.POINTER(ctypes.c_int32)]
 @classmethod
 def tearDownClass(cls):cls.tmp.cleanup()
 def test_signed_horner(self):
  rng=random.Random(26019)
  for k in range(2000):
   row=[rng.randrange(-100000000,100000000) for _ in range(8)];phase=rng.choice([0,1,1<<23,(1<<24)-1,1<<24,rng.randrange(1<<24)])
   self.assertEqual(self.lib.native_horner((ctypes.c_int32*8)(*row),phase),fixed_sample(row,phase))
 def test_field_bounds_and_endpoints(self):
  w=[0]*220
  for i in range(21):w[8*i]=1<<23
  for f in range(3):
   for a in range(7):w[175+f*14+a]=1<<24
  w[217]=49;arr=(ctypes.c_int32*220)(*w);out=(ctypes.c_int32*21)()
  for tick in range(50):
   self.assertEqual(self.lib.native_fields(arr,tick,out),0);self.assertEqual(list(out),[1<<23]*21)
  self.assertNotEqual(self.lib.native_fields(arr,50,out),0)
  arr[175]=1;self.assertNotEqual(self.lib.native_fields(arr,0,out),0)
 def test_packet_bytes(self):
  w=[0]*220
  for i in range(21):w[8*i]=1<<23
  for f in range(3):
   for a in range(7):w[175+f*14+a]=1<<24
  w[217]=10
  for a in range(6):w[210+a]=0x12345678
  out=(ctypes.c_uint32*21)();self.assertEqual(self.lib.native_packets((ctypes.c_int32*220)(*w),0,out),0)
  for a in range(6):self.assertEqual(list(out)[3*a:3*a+3],[0x1800001+a,0x80008000,0x12345678])
  self.assertEqual(list(out)[18:],[0x1200fd07,0x16700000,0x0000003f])
 def test_submission_preflight_before_any_target_write(self):
  w=[0]*220
  for i in range(21):w[8*i]=1<<23
  for f in range(3):
   for a in range(7):w[175+f*14+a]=1<<24
  w[217]=10;arr=(ctypes.c_int32*220)(*w);memory=(ctypes.c_uint32*1024)();base=ctypes.addressof(memory)
  memory[0x404//4]=7;memory[0x5a8//4]=127;memory[0x408//4]=20000
  for axis in range(6):
   for k in range(4):memory[(0x600+16*axis+4*k)//4]=0xffff0000 if k<3 else 0xffffffff
  memory[0x448//4]=0xdeadbeef
  memory[0x41c//4]=10
  self.assertEqual(self.lib.native_submit(base,arr,0,21000),-4);self.assertEqual(memory[0x448//4],0xdeadbeef)
  memory[0x41c//4]=0
  self.assertEqual(self.lib.native_submit(base,arr,0,16000),-5);self.assertEqual(memory[0x448//4],0xdeadbeef)
  memory[0x418//4]=1
  self.assertEqual(self.lib.native_submit(base,arr,0,21000),-3);self.assertEqual(memory[0x448//4],0xdeadbeef)
  memory[0x418//4]=0
  self.assertEqual(self.lib.native_submit(base,arr,0,21000),0)
  self.assertEqual(memory[0x448//4],0x1200fd07);self.assertEqual(memory[0x450//4],0x16700000)
 def test_bulk_segment_preflight_and_exact_words(self):
  memory=(ctypes.c_uint32*1024)();base=ctypes.addressof(memory);w=(ctypes.c_int32*220)(*[i-110 for i in range(220)])
  memory[0x404//4]=7;memory[0xb40//4]=2;memory[0xc00//4]=0xdeadbeef
  self.assertEqual(self.lib.native_upload(base,w),-10);self.assertEqual(memory[0xc00//4],0xdeadbeef)
  memory[0xb40//4]=0;memory[0xb60//4]=1
  self.assertEqual(self.lib.native_upload(base,w),0)
  self.assertEqual(list(memory)[0xc00//4:0xc00//4+220],[x&0xffffffff for x in w])

if __name__=='__main__':unittest.main()
