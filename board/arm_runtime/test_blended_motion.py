import unittest
from pathlib import Path
import numpy as np
from joint_trajectory import JointPlanner
from blended_motion import blended_movej
from septic_trajectory import boundary

class BlendedMotionTests(unittest.TestCase):
 def test_interior_motion_and_endpoint_rest(self):
  p=JointPlanner(Path(__file__).with_name('profile_autotune.json'));a=np.deg2rad([-60,10,-15,15,20,-10]).tolist()+[.35]
  b=np.deg2rad([-50,18,-26,22,27,-4]).tolist()+[.35]
  c=np.deg2rad([-65,25,-35,10,34,-15]).tolist()+[.35]
  plans,audit=blended_movej(p,[a,b,c,a])
  self.assertTrue(all(x>.01 for x in audit['interior_speed_norm_rad_s']))
  for order in (1,2,3):
   self.assertLess(max(map(abs,boundary(plans[0],order))),1e-8)
   self.assertLess(max(map(abs,boundary(plans[-1],order,True))),1e-7)
  for x,y in zip(plans,plans[1:]):
   for order in range(4):self.assertLess(max(abs(a-b) for a,b in zip(boundary(x,order,True),boundary(y,order))),2e-7)
 def test_requires_interior_waypoint(self):
  with self.assertRaises(ValueError):blended_movej(JointPlanner(),[[0]*7,[0]*7])

if __name__=='__main__':unittest.main()
