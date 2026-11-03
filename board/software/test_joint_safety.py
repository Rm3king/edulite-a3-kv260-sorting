import unittest,random,math
import json
from pathlib import Path
from joint_safety import *
import test_protocol
from canctl import CanDevice
from a3_protocol import motion,enable
from el_a3_sdk.protocol import DEFAULT_JOINT_DIRECTIONS,DEFAULT_JOINT_OFFSETS,DEFAULT_JOINT_LIMITS

class SafetyTests(unittest.TestCase):
 def test_actual_gripper_profile(self):
  g=json.loads((Path(__file__).parents[1]/'config/a3_commissioning.json').read_text())['gripper']
  bounds=GripperRange(g['closed_motor_rad'],g['open_motor_rad'])
  self.assertEqual([bounds.lower,bounds.upper],g['operating_motor_limits_rad'])
  self.assertEqual(bounds.check(g['closed_motor_rad']),g['closed_motor_rad'])
  self.assertEqual(bounds.check(g['open_motor_rad']),g['open_motor_rad'])
  targets=[bounds.target(i/100) for i in range(101)]
  self.assertTrue(all(a>b for a,b in zip(targets,targets[1:])))
  for target in targets:bounds.check(target)
 def test_measured_gripper_endpoints(self):
  closed=1.677834153175354
  for opened in [-.2,2.4]:
   g=GripperRange(closed,opened)
   self.assertEqual(g.check(closed),closed);self.assertEqual(g.target(0),closed)
   self.assertAlmostEqual(g.target(1),opened)
   for fraction in [0,.1,.5,.9,1]:g.check(g.target(fraction))
   for q in [g.lower-.001,g.upper+.001,float('nan')]:
    with self.assertRaises(ValueError):g.check(q)
  for opened in [None,closed,float('nan')]:
   with self.assertRaises(ValueError):GripperRange(closed,opened)
 def test_sdk_constants_and_transforms(self):
  self.assertEqual(DIRECTIONS,DEFAULT_JOINT_DIRECTIONS);self.assertEqual(OFFSETS,DEFAULT_JOINT_OFFSETS);self.assertEqual(LIMITS,DEFAULT_JOINT_LIMITS)
  rng=random.Random(260)
  for mid in range(1,8):
   for _ in range(1000):
    q=rng.uniform(*LIMITS[mid]);offsets={mid:rng.uniform(-.2,.2)}
    self.assertAlmostEqual(motor_to_joint(mid,joint_to_motor(mid,q,offsets),offsets),q)
 def test_boundary_and_invalid(self):
  for mid,(lo,hi) in LIMITS.items():
   check_joint(mid,lo);check_joint(mid,hi)
   for q in [lo-.00001,hi+.00001,float('nan'),float('inf')]:
    with self.assertRaises(ValueError):check_joint(mid,q)
  with self.assertRaises(ValueError):check_joint(7,1.6778)
 def test_quantized_ranges(self):
  for mid in range(1,8):
   lo,hi=LIMITS[mid];words=guard_words(mid,lo,hi,.1,.2,5,.5,33 if mid<4 else 50,14 if mid<4 else 6)
   lower,upper=words[0]&65535,words[0]>>16
   for raw in range(lower,upper+1):
    decoded=raw*25.14/65535-12.57
    self.assertTrue(lo-1e-12<=motor_to_joint(mid,decoded)<=hi+1e-12)
 def test_session_fail_closed(self):
  ages=dict.fromkeys(range(1,8),1);g=SessionGuard()
  g.accept(1,100,90,200,ages,True)
  with self.assertRaisesRegex(ValueError,'REPLAY'):g.accept(1,110,100,200,ages,True)
  with self.assertRaises(ValueError):g.accept(2,110,100,200,ages,True)
  g.recover();g.accept(2,110,100,200,ages,True)
  for args in [(3,200,100,200,ages,True),(3,110,100,200,{1:1},True),(3,110,100,200,dict.fromkeys(range(1,8),100000),True),(3,110,100,200,ages,False),
               (float('nan'),110,100,200,ages,True),(3,float('nan'),100,200,ages,True),
               (3,110,100,200,dict.fromkeys(range(1,8),'1'),True),(3,110,100,200,dict.fromkeys(range(1,8),float('nan')),True),
               (3,110,100,200,ages,'false'),(3,110,100,1000101,ages,True)]:
   g.recover()
   with self.assertRaises(ValueError):g.accept(*args)
   self.assertIsNotNone(g.fault)
 def test_driver_rejects_before_mmio_write(self):
  dev=CanDevice.__new__(CanDevice)
  regs={0x404:7,0x418:0,0x470:0x20000,0x5a0:0,0x49c:1,
        0x500:0x020001fd,0x50c:50,0x408:100,0x5a4:1000}
  words=guard_words(1,-.1,.1,.1,.2,5,.5,33,14)
  regs.update({0x600+i*4:w for i,w in enumerate(words)})
  dev.read=lambda off:regs[off]
  writes=[];dev.write=lambda *args:writes.append(args)
  good=motion(1,0,0,1,.1,0)
  for packet in [good,enable(1)]:
   with self.assertRaises(ValueError):dev.enqueue(*packet)
  self.assertEqual(writes,[])
  regs[0x5a0]=1;dev.check_motion_guard(*good);dev.check_motion_guard(*enable(1))
  for packet in [motion(1,.2,0,1,.1,0),motion(1,0,.2,1,.1,0),motion(1,0,0,6,.1,0),motion(1,0,0,1,.6,0),motion(1,0,0,1,.1,.3)]:
   with self.assertRaises(ValueError):dev.enqueue(*packet)
  for off,value in [(0x49c,0),(0x500,0x020101fd),(0x408,1050)]:
   saved=regs[off];regs[off]=value
   with self.assertRaises(ValueError):dev.enqueue(*good)
   regs[off]=saved
  self.assertEqual(writes,[])
  with self.assertRaisesRegex(ValueError,'alias'):dev.enqueue((26<<24)|(0xfd<<8)|1,bytes(8))
  regs[0x404]=15;dev.check_motion_guard(*good) # internal selftest has physical TX inhibited
if __name__=='__main__':unittest.main()
