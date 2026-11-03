#!/usr/bin/python3
import json
from pathlib import Path
import struct
import unittest
from a3_protocol import enable, motion
from arm_commission import make_plan, parameter, Session, readback_matches, zero_plan, ZERO_ORDER, VelocityMonitor
from canctl import CanDevice


class ArmTests(unittest.TestCase):
    def setUp(self):
        self.config=json.loads((Path(__file__).parents[1]/'config/a3_commissioning.json').read_text())
        self.regs={0x470:0x40000,0x404:7,0x5a0:127,0x5a8:0,0x49c:127,0x408:200,0x5a4:100000}
        for mid in range(1,8):
            b=CanDevice.pp_base(mid)
            self.regs.update({b:1,b+24:63,0x500+(mid-1)*16:0x020000fd+(mid<<8),0x50c+(mid-1)*16:100})
            for offset,v in ((4,-.1),(8,.1),(12,.02),(16,.1),(20,.5)):
                self.regs[b+offset]=struct.unpack('<I',struct.pack('<f',v))[0]
        self.dev=object.__new__(CanDevice);self.dev.read=lambda a:self.regs.get(a,0)

    def test_each_axis_preparation_independent(self):
        for mid in range(1,8):
            self.dev.check_motion_guard(*enable(mid))
            b=self.dev.pp_base(mid)
            for off,value in ((b,0),(b+24,31),(0x49c,127^(1<<(mid-1))),
                              (0x500+(mid-1)*16,0x020100fd+(mid<<8)),(0x50c+(mid-1)*16,0xfffc0000)):
                saved=self.regs[off];self.regs[off]=value
                with self.assertRaises(ValueError):self.dev.check_motion_guard(*enable(mid))
                for other in set(range(1,8))-{mid}:self.dev.check_motion_guard(*enable(other))
                self.regs[off]=saved

    def test_limits_and_active_reconfiguration_all_axes(self):
        for mid in range(1,8):
            for idx,v,integer in ((0x7005,1,True),(0x7028,6000,True),(0x7024,.02,False),
                                  (0x7025,.1,False),(0x7018,.5,False),(0x7016,.05,False)):
                self.dev.check_motion_guard(*parameter(mid,idx,v,integer))
            for idx,v,integer in ((0x7005,0,True),(0x7028,0,True),(0x7028,6000.,False),
                                  (0x7024,.021,False),(0x7025,.11,False),(0x7018,.51,False),
                                  (0x7016,.101,False),(0x7016,-.101,False),(0x7010,.1,False)):
                with self.assertRaises(ValueError):self.dev.check_motion_guard(*parameter(mid,idx,v,integer))
            self.regs[0x5a8]=1<<(mid-1)
            with self.assertRaises(ValueError):self.dev.check_motion_guard(*parameter(mid,0x7028,6000,True))
            self.dev.check_motion_guard(*parameter(mid,0x7016,.02))
            self.regs[0x5a8]=0
            with self.assertRaises(ValueError):self.dev.check_motion_guard(*motion(mid,0,0,0,0))

    def test_malformed_and_nonfinite_rejected(self):
        for mid in range(1,8):
            for bits in (0x7f800000,0xff800000,0x7fc00000):
                with self.assertRaises(ValueError):self.dev.check_motion_guard(0x1200fd00|mid,bytes.fromhex('16700000')+struct.pack('<I',bits))
            for ident,payload in ((0x1200fe00|mid,parameter(mid,0x7016,.01)[1]),
                                  (0x1200fd00|mid,bytes.fromhex('1670010000000000')),
                                  (0x1200fd00|mid,bytes(7))):
                with self.assertRaises(ValueError):self.dev.check_motion_guard(ident,payload)
        for mid in (0,8,255):
            with self.assertRaises(ValueError):self.dev.check_motion_guard(0x1200fd00|mid,bytes(8))

    def test_inward_plans_and_zero_unchanged(self):
        from joint_safety import LIMITS,joint_to_motor
        for mid in range(1,8):
            limits=sorted(joint_to_motor(mid,x) for x in LIMITS[mid]) if mid<7 else sorted((self.config['gripper']['open_motor_rad'],self.config['gripper']['closed_motor_rad']))
            lo,hi=limits
            for i in range(101):
                start=hi if i==100 else lo+(hi-lo)*i/100;p=make_plan(mid,start,self.config)
                self.assertAlmostEqual(abs(p['target']-start),.01)
                self.assertTrue(lo<=p['lower']<=start<=p['upper']<=hi)
                self.assertTrue(p['lower']<=p['target']<=p['upper'])
            for start in (lo-.01,hi+.01,float('nan')):
                with self.assertRaises(ValueError):make_plan(mid,start,self.config)

    def test_position_readback_tolerance_is_narrow_and_parameter_specific(self):
        self.assertTrue(readback_matches(0x7016,1.5036463737487793,1.5036077499389648))
        for actual in (1.5034,1.504,float('nan'),float('inf')):
            self.assertFalse(readback_matches(0x7016,1.5036463737487793,actual))
        self.assertFalse(readback_matches(0x7018,.5,.50005))
        self.assertFalse(readback_matches(0x7028,6000,6001,True))

    def test_position_readback_wait_is_bounded(self):
        r={'readbacks':{}};s=Session(None,r);s.send=lambda packet:None;s.pump=lambda seconds:None
        sequence=iter([1.499,1.4995,1.5]);s.read=lambda *args:next(sequence)
        s.write_verified(1,0x7016,1.5)
        self.assertEqual(len(r['readback_samples'][0]['observed']),3)
        s.read=lambda *args:1.499
        with self.assertRaises(RuntimeError):s.write_verified(1,0x7016,1.5)
        self.assertEqual(len(r['readback_samples'][-1]['observed']),8)

    def test_zero_plan_uses_calibrated_zero_and_fixed_limits(self):
        self.assertEqual(sorted(ZERO_ORDER),list(range(1,8)))
        self.assertEqual(ZERO_ORDER[-2:],(2,3))
        starts=[1.5,.0065,.0047,.43,-.034,.068,.038]
        for mid,start in enumerate(starts,1):
            p=zero_plan(mid,start,self.config)
            self.assertEqual(p['target'],0.)
            self.assertTrue(p['lower']<=0<=p['upper'] and p['lower']<=start<=p['upper'])
            self.assertEqual(p['current'],[1.5,3,3,2.5,1,1,.5][mid-1])
            self.assertEqual(p['speed'],.01 if mid in (2,3) else .04 if mid==7 else .06)
            self.assertEqual(p['acceleration'],.03 if mid in (2,3) else .15)
            self.assertEqual(p['timeout'],6000)

    def test_velocity_transient_and_real_displacement_are_distinguished(self):
        m=VelocityMonitor()
        for i in range(30):
            self.assertEqual(m.observe(dict(motor=2,time=1+i*.02,position=.001,
                velocity=.221 if i==10 else 0)),i==10)
        m=VelocityMonitor()
        with self.assertRaisesRegex(RuntimeError,'sustained'):
            for i in range(20):m.observe(dict(motor=2,time=1+i*.02,position=0,velocity=.25))
        m=VelocityMonitor()
        with self.assertRaisesRegex(RuntimeError,'encoder window'):
            for i in range(20):m.observe(dict(motor=2,time=1+i*.02,position=i*.02*.3,velocity=0))
        with self.assertRaisesRegex(RuntimeError,'gross'):
            VelocityMonitor().observe(dict(motor=2,time=1,position=0,velocity=.81))

    def test_velocity_slow_motion_quantization_and_gaps(self):
        m=VelocityMonitor()
        for i in range(100):
            for mid in range(1,8):
                m.observe(dict(motor=mid,time=1+i*.02,position=i*.02*.06+(.0003836 if i%2 else 0),velocity=.06))
        with self.assertRaisesRegex(RuntimeError,'gap/order'):
            m.observe(dict(motor=1,time=4,position=0,velocity=0))

    def test_stop_failure_does_not_skip_other_axes(self):
        class Device:
            pp_base=staticmethod(CanDevice.pp_base)
            def read(self,a):return 7 if a==0x404 else 0
            def write(self,*unused):pass
            def stop(self):pass
            def configure(self,mode):pass
            def status(self):return {'control':0}
        r=dict(reset_confirmed=[],stop_errors={});s=Session(Device(),r);attempted=[]
        def send(packet):
            mid=packet[0]&255;attempted.append(mid)
            if mid==7:raise RuntimeError('injected first stop failure')
            import time
            s.feedback[mid]={'time':time.monotonic(),'mode':0}
        s.send=send;s.pump=lambda duration:None;s.stop_all()
        self.assertEqual(attempted[:7],list(range(7,0,-1)))
        self.assertEqual(attempted[7:],[7])
        self.assertEqual(r['reset_confirmed'],list(range(6,0,-1)))
        self.assertIn('7',r['stop_errors'])


if __name__=='__main__':unittest.main()
