import unittest
from pathlib import Path
import numpy as np
from types import SimpleNamespace
from unittest.mock import patch
from joint_trajectory import JointPlanner
from motion_runtime import MotionRuntime
from gravity_trajectory import GravityModel,prepare_stream,validate_records,encoded_interval
from tuning_worker import probe,guard_job,patch_gains
from tuning_metrics import reference,evaluate
from tuning_search import candidates,suggest
from freedrive import DirectModel
from autotune import TuningExecutor
from gravity_executor import GravityExecutor
from live_motion import current_reference
ROOT=Path(__file__).parent
class TuningTests(unittest.TestCase):
 @classmethod
 def setUpClass(cls):
  cls.p=JointPlanner(ROOT/'profile_grasp_damped_candidate.json');cls.m=GravityModel(MotionRuntime(),cls.p,ROOT/'tracking_autotune.json');cls.tasks=probe(cls.p,2);cls.records=prepare_stream(cls.m,cls.tasks)
 def test_all_field_phases_and_gain_limits(self):
  validate_records(self.records,guard_job(self.p,self.tasks),self.p,self.m)
  changed=patch_gains(self.records,[240,240,240,180,180,180],[5]*6,self.m)
  for before,after in zip(self.records,changed):
   self.assertEqual(before['words'][:210],after['words'][:210]);self.assertEqual(before['words'][217:],after['words'][217:]);self.assertIs(before['original_plan'],after['original_plan'])
  with self.assertRaises(ValueError):patch_gains(self.records,[241]*6,[5]*6,self.m)
 def test_free_drive_still_has_zero_hardware_stiffness_limit(self):
  m=DirectModel(MotionRuntime(),self.p)
  self.assertTrue(np.all(m.guard_kp==0))
  self.assertEqual(encoded_interval(guard_job(self.p,[]),self.p,m,0)[3]&65535,0)
 def synthetic(self,ripple,base=(1<<32)-123456,temperature=30.,torque=.1):
  q,v,_,_,offsets=reference(self.records);fs=[];snaps=[];dt=.005
  for i in range(len(q)):
   for mid in range(1,7):
    extra=ripple*np.sin(2*np.pi*2*i*dt) if mid==3 else 0.
    fs.append(dict(motor=mid,mode=2,fault=0,pl_rx_us=(base+i*5000)&0xffffffff,position=float(q[i,mid-1]+extra),velocity=float(v[i,mid-1]),torque=torque,temperature=temperature))
  for j,r in enumerate(self.records):
   for tick in range(0,r['plan']['ticks']+1,5):
    snaps.append(dict(completed=7+j,tick=tick,next_due_us=(base+(int(offsets[j])+tick)*5000)&0xffffffff,holding=False))
  return evaluate(fs,snaps,self.records,7,2,self.m.caps)
 def test_hardware_epoch_wrap_and_quantified_ripple(self):
  clean=self.synthetic(0);ripple=self.synthetic(.001)
  self.assertLess(clean['joints'][2]['position_rmse_rad'],1e-10)
  self.assertAlmostEqual(ripple['joints'][2]['position_rmse_rad'],.001/np.sqrt(2),delta=.00003)
  self.assertGreater(ripple['score'],clean['score']);self.assertTrue(ripple['feasible']);self.assertEqual(ripple['epoch_modal_fraction'],1)
 def test_search_is_bounded_and_does_not_repeat(self):
  for axis in range(6):
   history=[]
   for _ in range(9):
    k,d=suggest(history,axis);self.assertIn((k,d),candidates(axis));self.assertNotIn((k,d),[(r['kp'],r['kd']) for r in history])
    history.append(dict(kp=k,kd=d,score=((k-120)/120)**2+((d-3)/3)**2+.1))
 def test_temperature_and_torque_saturation_are_report_only(self):
  result=self.synthetic(0,temperature=85.,torque=10.)
  self.assertTrue(result['feasible'])
  self.assertEqual(result['joints'][0]['temperature_peak_c'],85.)
  self.assertEqual(result['joints'][0]['saturation_fraction'],1.)
 def test_logging_overrun_keeps_motor_control_and_real_expiry_still_faults(self):
  e=TuningExecutor.__new__(TuningExecutor);e.started=False;e.report={'stream_initial_counters':{'rx_overflow':12,'expired':0}}
  e.d=SimpleNamespace(read=lambda a:{0x424:0,0x418:0,0x42c:37}[a]);e.transport=SimpleNamespace(status=lambda:{'fault':0,'holding':True})
  with patch.object(GravityExecutor,'pump',side_effect=RuntimeError('RX overflow or expired target')):
   self.assertTrue(e.pump()['holding'])
  self.assertEqual(e.report['observation_gaps'][0]['rx_lost'],25)
  e.d=SimpleNamespace(read=lambda a:{0x424:1,0x418:0,0x42c:37}[a])
  with patch.object(GravityExecutor,'pump',side_effect=RuntimeError('RX overflow or expired target')):
   with self.assertRaises(RuntimeError):e.pump()
 def test_observed_disabled_zero_overrun_captured_without_expanding_command_limits(self):
  p=JointPlanner(ROOT/'profile_autotune.json');motor=p.to_motor([-.8,.001,-.01,.3,0.,0.,.35]);motor[2]=-.0237767
  ref,adjust=current_reference(p,motor)
  self.assertLessEqual(ref[2],0);self.assertLess(abs(ref[2]),1e-6);self.assertGreater(adjust[2],.023)
  self.assertEqual(p.upper[2],0)

if __name__=='__main__':unittest.main()
