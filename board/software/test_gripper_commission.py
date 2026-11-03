#!/usr/bin/python3
import json
import math
from pathlib import Path
import struct
import unittest
from canctl import CanDevice
from gripper_commission import parameter, targets, repeat_targets, Session
from a3_protocol import enable


class GripperTests(unittest.TestCase):
    def setUp(self):
        self.regs={0x470:0x30000,0x404:7,0x5a0:64,0x680:1,0x698:63,0x5a8:0,
                   0x49c:64,0x560:0x020007fd,0x56c:100,0x408:200,0x5a4:100000}
        for address,value in [(0x684,-.01),(0x688,.06),(0x68c,.08),(0x690,.2),(0x694,.3)]:
            self.regs[address]=struct.unpack('<I',struct.pack('<f',value))[0]
        self.dev=object.__new__(CanDevice);self.dev.read=lambda a:self.regs.get(a,0)

    def test_whitelist_and_encoding(self):
        for index,val,integer in [(0x7005,1,True),(0x7028,6000,True),(0x7024,.08,False),
                                  (0x7025,.2,False),(0x7018,.3,False),(0x7016,.05,False),(0x7016,-.005,False)]:
            packet=parameter(index,val,integer)
            self.assertEqual(packet[0],0x1200fd07)
            self.dev.check_motion_guard(*packet)
        self.assertEqual(parameter(0x7028,6000,True)[1].hex(),'2870000070170000')
        invalid=[parameter(0x7005,1.),parameter(0x7005,0,True),parameter(0x7028,0,True),
                 parameter(0x7028,6000.),parameter(0x7028,20001,True),parameter(0x7016,-.02),
                 parameter(0x7016,.07),parameter(0x7024,-.01),parameter(0x7024,.09),
                 parameter(0x7025,.21),parameter(0x7018,.31),parameter(0x7010,.1)]
        for v in [0x7f800000,0xff800000,0x7fc00000]:
            invalid.append((0x1200fd07,bytes.fromhex('16700000')+struct.pack('<I',v)))
        for packet in invalid:
            with self.assertRaises(ValueError):self.dev.check_motion_guard(*packet)
        for mid in range(1,7):
            with self.assertRaises(ValueError):self.dev.check_motion_guard(0x1200fd00|mid,parameter(0x7016,.01)[1])

    def test_enable_gates_and_freshness(self):
        self.dev.check_motion_guard(*enable(7))
        for address,bad in [(0x5a0,0),(0x680,0),(0x698,31),(0x49c,0),(0x560,0x020107fd),(0x408,100100)]:
            old=self.regs[address];self.regs[address]=bad
            with self.assertRaises(ValueError):self.dev.check_motion_guard(*enable(7))
            self.regs[address]=old
        for mid in range(1,7):
            with self.assertRaises(ValueError):self.dev.check_motion_guard(*enable(mid))
        self.regs[0x470]=0x20000
        with self.assertRaises(ValueError):self.dev.check_motion_guard(*enable(7))
        with self.assertRaises(ValueError):self.dev.check_motion_guard(*parameter(0x7016,.01))

    def test_active_no_reconfiguration(self):
        self.regs[0x5a8]=64
        self.dev.check_motion_guard(*parameter(0x7016,.05))
        for index,val,integer in [(0x7005,1,True),(0x7028,6000,True),(0x7024,.08,False),
                                  (0x7025,.2,False),(0x7018,.3,False)]:
            with self.assertRaises(ValueError):self.dev.check_motion_guard(*parameter(index,val,integer))

    def test_measured_travel_and_failed_readback(self):
        g=json.loads((Path(__file__).parents[1]/'config/a3_commissioning.json').read_text())['gripper']
        lo,hi=sorted([g['open_motor_rad'],g['closed_motor_rad']])
        for i in range(101):
            p=lo+(hi-lo)*i/100;end,lower,upper=targets(p,hi,lo)
            self.assertAlmostEqual(abs(end-p),.05)
            self.assertTrue(lo<=lower<=p<=upper<=hi and lower<=end<=upper)
            self.assertLess(abs(end-(lo+hi)/2),abs(p-(lo+hi)/2)+.051)
        for p in [lo-.001,hi+.001,float('nan')]:
            with self.assertRaises(ValueError):targets(p,hi,lo)
        session=Session(None,{})
        session.send=lambda p:None;session.pump=lambda t:None;session.read=lambda *unused:11.
        with self.assertRaises(RuntimeError):session.write_verified(0x7018,.3)

    def test_repeat_sequence_stays_inside_and_returns(self):
        lo,hi=-.005880276,1.677834153
        for position in [.04423,.4,.85,1.6]:
            sequence,lower,upper=repeat_targets(position,hi,lo)
            self.assertEqual(len(sequence),6)
            self.assertEqual(sequence[-1],position)
            self.assertTrue(all(lower<=q<=upper for q in sequence))
            self.assertTrue(all(lo+.01<=q<=hi-.01 for q in sequence))
            self.assertAlmostEqual(abs(sequence[0]-position),.05)
        for position in [lo,hi,lo-.1,hi+.1,float('nan')]:
            with self.assertRaises(ValueError):repeat_targets(position,hi,lo)


if __name__=='__main__':unittest.main()
