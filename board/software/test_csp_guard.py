import struct
import unittest
from canctl import CanDevice
from arm_commission import parameter
from a3_protocol import enable

class CspGuardTests(unittest.TestCase):
    def test_independent_modes_and_active_lock(self):
        for version in (0x50000,0x50001,0x60000,0x60001):
            with self.subTest(version=hex(version)):self.check_version(version)

    def check_version(self,version):
        for mid in range(1,8):
            b=CanDevice.pp_base(mid)
            regs={0x470:version,0x404:7,0x800:0,0x5a0:127,0x5a8:0,0x49c:127,
                  0x408:1000,0x5a4:10000,b:1,b+24:63,b+28:1,
                  0x500+(mid-1)*16:0x020000fd+(mid<<8),0x50c+(mid-1)*16:500}
            for off,v in [(4,-1),(8,1),(12,.1),(16,.2),(20,1)]:regs[b+off]=struct.unpack('<I',struct.pack('<f',v))[0]
            dev=object.__new__(CanDevice);dev.read=lambda a:regs.get(a,0)
            for idx,v,integer in [(0x7005,5,True),(0x7017,.1,False),(0x7018,1,False),(0x7028,6000,True),(0x7016,0,False)]:
                dev.check_motion_guard(*parameter(mid,idx,v,integer))
            dev.check_motion_guard(*enable(mid))
            for idx,v,integer in [(0x7005,1,True),(0x7024,.1,False),(0x7025,.2,False),(0x7017,.11,False),(0x7017,float('inf'),False)]:
                with self.assertRaises(ValueError):dev.check_motion_guard(*parameter(mid,idx,v,integer))
            regs[0x5a8]=1<<(mid-1)
            with self.assertRaises(ValueError):dev.check_motion_guard(*parameter(mid,0x7017,.1))
            regs[0x800]=1
            with self.assertRaises(ValueError):dev.check_motion_guard(*parameter(mid,0x7016,0))

    def test_joint3_gain_is_bounded_disabled_fresh_and_revision_specific(self):
        b=CanDevice.pp_base(3)
        regs={0x470:0x60001,0x404:7,0x5a0:127,0x5a8:0,0x49c:127,0x408:1000,0x5a4:10000,
              b:1,b+28:1,0x520:0x020003fd,0x52c:500}
        dev=object.__new__(CanDevice);dev.read=lambda a:regs.get(a,0)
        for v in (20.,25.,30.):dev.check_motion_guard(*parameter(3,0x701e,v))
        for v in (19.9,30.1,float('nan'),float('inf'),-20.):
            with self.assertRaises(ValueError):dev.check_motion_guard(*parameter(3,0x701e,v))
        for address,value in [(0x470,0x60000),(0x5a8,1),(0x49c,0),(0x520,0x028003fd),(b+28,0),(0x408,20000)]:
            old=regs[address];regs[address]=value
            with self.assertRaises(ValueError):dev.check_motion_guard(*parameter(3,0x701e,20.))
            regs[address]=old
        for mid in (1,2,4,5,6,7):
            bb=CanDevice.pp_base(mid);regs[bb]=1;regs[bb+28]=1
            with self.assertRaises(ValueError):dev.check_motion_guard(*parameter(mid,0x701e,20.))

    def test_speed_loop_parameters_are_not_writable(self):
        # Writing 0x7021 in the prepared state hung the KV260 twice (2026-10-26): keep it off the whitelist.
        b=CanDevice.pp_base(2)
        regs={0x470:0x60001,0x404:7,0x5a0:127,0x5a8:0,0x49c:127,0x408:1000,0x5a4:10000,b:1,b+28:1,0x510:0x020002fd,0x51c:500}
        dev=object.__new__(CanDevice);dev.read=lambda a:regs.get(a,0)
        for index in (0x701f,0x7020,0x7021):
            with self.assertRaises(ValueError):dev.check_motion_guard(*parameter(2,index,.1))

if __name__=='__main__':unittest.main()
