import unittest
from types import SimpleNamespace
from motion_executor import PathVelocityMonitor

def profile(factor=None):
 d={} if factor is None else {'gripper_encoder_window_factor':factor}
 return SimpleNamespace(profile=d,csp_velocity=[.35]*7)

def stream(m,axis,speed,reported=None):
 for k in range(35):
  m.observe(dict(motor=axis,time=k*.005,position=speed*k*.005,
                 velocity=speed if reported is None else reported,pl_rx_us=100000+k*5000))

class GripperMarginTests(unittest.TestCase):
 def test_bounded_contact_correction_and_other_axes_unchanged(self):
  with self.assertRaisesRegex(RuntimeError,'encoder window velocity'):stream(PathVelocityMonitor(profile()),7,.42)
  stream(PathVelocityMonitor(profile(1.5)),7,.42)
  for axis in range(1,7):
   with self.subTest(axis=axis),self.assertRaisesRegex(RuntimeError,'encoder window velocity'):
    stream(PathVelocityMonitor(profile(1.5)),axis,.42)
 def test_excess_encoder_motion_still_trips_and_freezes_window(self):
  report={}
  with self.assertRaisesRegex(RuntimeError,'encoder window velocity'):
   stream(PathVelocityMonitor(profile(1.5),report),7,.60,reported=.1)
  event=report['velocity_monitor_failures'][0]
  self.assertGreaterEqual(event['window_s'],.08)
  self.assertGreater(event['displacement_rad'],event['allowed_displacement_rad'])
  self.assertGreater(len(event['window']),1)
 def test_reported_gross_and_sustained_speed_still_trip(self):
  for reported,reason in [(1.3,'gross velocity'),(.7,'sustained velocity')]:
   with self.subTest(reason=reason),self.assertRaisesRegex(RuntimeError,reason):
    stream(PathVelocityMonitor(profile(1.5)),7,0,reported)
 def test_invalid_margin_rejected(self):
  for x in [0,1.51,float('nan'),True,'1.5']:
   with self.subTest(value=x),self.assertRaises(ValueError):PathVelocityMonitor(profile(x))

if __name__=='__main__':unittest.main()
