import struct,unittest
from canctl import CanDevice,version_mit_running
from arm_commission import parameter
from a3_protocol import enable,motion


class MitGuardTests(unittest.TestCase):
 def make(self,mid=3):
  b=CanDevice.pp_base(mid)
  regs={0x470:0x60000,0x404:7,0x5a0:127,0x49c:127,0x408:1000,0x5a4:10000,b:1,b+24:31,b+28:2,
    b+16:struct.unpack('<I',struct.pack('<f',5.5))[0],0x500+(mid-1)*16:0x020000fd+(mid<<8),0x50c+(mid-1)*16:500}
  for k,w in enumerate([0xffff0000,0xffff0000,0xffff0000,0xcccc28f5]):regs[0x600+(mid-1)*16+k*4]=w
  dev=object.__new__(CanDevice);dev.read=lambda a:regs.get(a,0);return dev,regs,b
 def test_preparation_gates_type1_and_enable_independently(self):
  d,r,b=self.make();packet=motion(3,.2,0,80,4,3)
  r[b+24]=1
  with self.assertRaises(ValueError):d.check_motion_guard(*packet)
  r[b+24]=31;d.check_motion_guard(*packet)
  with self.assertRaises(ValueError):d.check_motion_guard(*enable(3))
  r[b+24]=63;d.check_motion_guard(*enable(3))
 def test_torque_mode_whitelist_cap_and_active_lock(self):
  d,r,b=self.make()
  for index,val,integer in [(0x7005,0,True),(0x700b,5.5,False),(0x7028,6000,True)]:d.check_motion_guard(*parameter(3,index,val,integer))
  for index,val,integer in [(0x7005,5,True),(0x700b,5.51,False),(0x700b,0,False),(0x700b,float('nan'),False),(0x7016,.2,False),(0x7018,4.5,False),(0x7017,.3,False)]:
   with self.assertRaises(ValueError):d.check_motion_guard(*parameter(3,index,val,integer))
  r[0x5a8]=4
  with self.assertRaises(ValueError):d.check_motion_guard(*parameter(3,0x700b,5.5))
 def test_running_pl_owns_type1_and_csp(self):
  d,r,b=self.make();r[0xb40]=1
  with self.assertRaisesRegex(ValueError,'PL owns'):d.check_motion_guard(*motion(3,.2,0,80,4,3))
  with self.assertRaisesRegex(ValueError,'PL owns'):d.check_motion_guard(*parameter(3,0x7016,.2))
 def test_old_version_never_probes_unmapped_register(self):
  class Old:
   def read(self,a):
    if a!=0x470:raise AssertionError('unmapped MMIO')
    return 0x50001
  self.assertFalse(version_mit_running(Old()))


if __name__=='__main__':unittest.main()
