import json,tempfile,unittest
from pathlib import Path
from unittest.mock import patch
import numpy as np
from joint_trajectory import JointPlanner
from motion_runtime import MotionRuntime
from septic_trajectory import movej_c3,derivative,continuity_metrics,S7
from reference_smoothing import smooth_task,settings,build_pair
from tuning_worker import pick_tasks
from gravity_trajectory import GravityModel

ROOT=Path(__file__).parent

class ReferenceTests(unittest.TestCase):
 def test_split_damping_changes_only_metadata_before_gain_patching(self):
  from gravity_trajectory import hold_segment
  p=JointPlanner(ROOT/'profile_autotune.json');r=MotionRuntime();m=GravityModel(r,p,ROOT/'tracking_grasp_diagnostic_baseline.json')
  m.config.update(full_pick_experiment=dict(method='split_damping',fast_joint3_kd=3.),pl_joint3_damping_nm_s_rad=2.)
  m.pl_damping_q8=512;q=[-.8,.1,-.2,.3,0.,0.,.35]
  tasks=[dict(name='hold',segments=[hold_segment(p,q,.2)],goal_rad=q,gravity_alpha=[1.,1.])]
  a,b,info=build_pair(m,tasks)
  self.assertEqual(a[0]['words'][:216],b[0]['words'][:216]);self.assertEqual(a[0]['words'][217:],b[0]['words'][217:]);self.assertEqual(a[0]['words'][216],0);self.assertEqual(b[0]['words'][216],512)
  self.assertEqual(info['nominal_low_frequency_damping'],5)
  m.config['full_pick_experiment']['fast_joint3_kd']=4
  with self.assertRaisesRegex(ValueError,'nominal'):settings(m.config)
 def test_fir_matches_independent_global_polynomial_and_c3(self):
  p=JointPlanner(ROOT/'profile_autotune.json');q=np.array([-.8,.1,-.2,.3,0.,0.,.35]);goal=q.copy();goal[1]+=.1;goal[2]-=.1
  plans=movej_c3(p,q.tolist(),goal.tolist(),minimum_duration_s=2.);duration=sum(s['duration_s'] for s in plans)
  shaped=smooth_task(p,dict(name='test',kind='MoveJ',segments=plans,goal_rad=goal.tolist()),25)
  t=0
  for plan in shaped['segments']:
   for u in np.linspace(0,1,13):
    seconds=t+u*plan['duration_s'];expected=np.zeros(7)
    for delay,weight in ((0,.25),(.125,.5),(.25,.25)):
     phase=np.clip((seconds-delay)/duration,0,1);expected+=weight*(q+(goal-q)*np.polynomial.polynomial.polyval(phase,S7))
    actual=[derivative(row,u) for row in plan['coefficients_rad']]
    np.testing.assert_allclose(actual,expected,atol=1e-6)
   t+=plan['duration_s']
  self.assertAlmostEqual(t,duration+.25)
  continuity=continuity_metrics(shaped['segments'],p)
  self.assertLess(max(continuity['maximum_boundary_jumps']),1e-7)
  self.assertLess(max(continuity['rest_endpoint_v_a_j']),1e-7)
  np.testing.assert_allclose(shaped['segments'][-1]['goal_rad'],goal,atol=1e-9)
 def test_full_pick_geometry_and_return_and_independent_dynamic_fields(self):
  p=JointPlanner(ROOT/'profile_autotune.json');r=MotionRuntime();q=np.deg2rad([-84,1,-3,32,2,6]).tolist()+[.35]
  capture=dict(status='PASS_READ_ONLY_ALL_7',transactions=[dict(motor=i+1,decoded=dict(valid=True,position_motor_rad=v)) for i,v in enumerate(p.to_motor(q))])
  tasks=pick_tasks(capture,p,r);m=GravityModel(r,p,ROOT/'tracking_grasp_diagnostic_baseline.json');m.config['full_pick_experiment']=dict(method='reference_smoothing',spacing_ticks=25)
  a,b,audit=build_pair(m,tasks)
  self.assertLess(audit['sampled_moveL_line_error_m'],.00025)
  np.testing.assert_allclose(b[-1]['original_plan']['goal_rad'],a[-1]['original_plan']['goal_rad'],atol=1e-9)
  # Each wire record emits phases 0..n inclusive; prepare_stream trims the
  # duplicate first phase of every record after the first.
  self.assertEqual(sum(x['plan']['ticks']+1 for x in b)-sum(x['plan']['ticks']+1 for x in a),300)
  m.config['full_pick_experiment']=dict(method='inverse_dynamics');m.config['inverse_dynamics_scale']=.35;m.dynamic_scale=.35
  a,b,_=build_pair(m,tasks)
  self.assertEqual(len(a),len(b));changed=False
  for x,y in zip(a,b):
   self.assertEqual(x['words'][:112],y['words'][:112]);self.assertEqual(x['words'][168:],y['words'][168:])
   changed|=x['words'][112:168]!=y['words'][112:168]
  self.assertTrue(changed)
 def test_invalid_config_and_missing_comparison_rejected_before_hardware(self):
  from autotune import execute
  for cfg in [dict(full_pick_experiment=dict(method='unknown')),dict(full_pick_experiment=dict(method='reference_smoothing',spacing_ticks=True))]:
   with self.assertRaises(ValueError):settings(cfg)
  cfg=json.loads((ROOT/'tracking_grasp_diagnostic_baseline.json').read_text());cfg['full_pick_experiment']=dict(method='reference_smoothing')
  with tempfile.TemporaryDirectory() as out,patch('autotune.CanDevice') as device:
   path=Path(out)/'config.json';path.write_text(json.dumps(cfg))
   with self.assertRaisesRegex(ValueError,'explicit dedicated'):execute(ROOT/'profile_autotune.json',path,Path(out)/'result',full_pick_only=True)
   device.assert_not_called()

if __name__=='__main__':unittest.main()
