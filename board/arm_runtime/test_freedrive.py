import unittest
import struct
from pathlib import Path
from unittest.mock import patch
import numpy as np
from freedrive import DirectModel,Policy,CONFIG
from gravity_executor import GravityExecutor
from gravity_trajectory import GravityModel
from joint_trajectory import JointPlanner
from motion_runtime import MotionRuntime


class DirectFreeDriveTests(unittest.TestCase):
 def setUp(self):
  self.p=JointPlanner(Path(__file__).with_name('profile_grasp_candidate.json'));self.r=MotionRuntime();self.m=DirectModel(self.r,self.p)
  self.motor=self.p.to_motor([-.8,0.,0.,.3,0.,.1,1.6]);self.policy=Policy(self.p,self.m);self.now=10.
 def step(self,command=None,dt=.01):
  self.now+=dt;fs=[dict(motor=i+1,mode=2,fault=0,time=self.now,position=q) for i,q in enumerate(self.motor)]
  return self.policy.step(fs,self.now,command)
 def test_zero_and_small_feedback_past_zero_need_no_preposition(self):
  self.motor[2]=-.001
  x=self.step();self.assertEqual(x['phase'],'DRAG');self.assertEqual(x['kp'],[0.]*6)
  self.assertTrue(all(struct.unpack('>HHHH',msg[1])[2]==0 for msg in x['packets']))
  self.assertAlmostEqual(self.m.reference_motor(3,-.001),0.)
 def test_actual_movement_updates_gravity_without_matching_old_snapshot(self):
  a=self.step();self.motor[1]+=.2;self.motor[2]+=.2
  for _ in range(200):b=self.step()
  self.assertGreater(max(abs(np.array(a['gravity_motor_nm'])-b['gravity_motor_nm'])),.01)
  np.testing.assert_allclose(b['gravity_motor_nm'],self.m.measured_gravity(self.motor),atol=1e-7)
 def test_no_short_timer_or_extra_40ms_cycle_cutoff(self):
  for _ in range(2000):x=self.step()
  self.assertEqual(x['phase'],'DRAG');self.assertEqual(self.step(dt=.05)['phase'],'DRAG')
  self.assertFalse(CONFIG['automatic_stop']);self.assertFalse(CONFIG['automatic_motion'])
 def test_real_stale_and_faulted_feedback_still_rejected(self):
  fs=[dict(motor=i+1,mode=2,fault=0,time=10.,position=q) for i,q in enumerate(self.motor)]
  with self.assertRaisesRegex(RuntimeError,'stale'):self.policy.step(fs,10.081)
  self.policy=Policy(self.p,self.m);fs[2]['fault']=1
  with self.assertRaisesRegex(RuntimeError,'faulted'):self.policy.step(fs,10.001)
 def test_torque_saturation_keeps_partial_compensation(self):
  with patch.object(self.r.kin,'compute_gravity',return_value=[30.]*7):
   x=self.step();self.assertTrue(all(x['torque_saturated']))
   self.assertTrue(np.all(np.abs(x['gravity_motor_nm'])<=self.m.caps));self.assertEqual(x['phase'],'DRAG')
 def test_supported_stop_waits_until_actual_pose_is_still(self):
  self.step(command='finish_supported')
  for k in range(100):
   self.motor[0]=.01*np.sin(k/10);self.assertEqual(self.step()['phase'],'DRAG')
  for _ in range(60):self.motor[0]=0.;x=self.step()
  self.assertEqual(x['phase'],'DONE')
 def test_live_rebind_forbidden_with_position_stiffness(self):
  e=GravityExecutor.__new__(GravityExecutor);e.model=GravityModel(self.r,self.p)
  with self.assertRaisesRegex(ValueError,'zero position stiffness'):e.prepare({},live_start=True)


if __name__=='__main__':unittest.main()
