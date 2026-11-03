import copy,unittest
from unittest.mock import patch
from pathlib import Path
import numpy as np
from joint_trajectory import JointPlanner,Q24
from septic_trajectory import (S7,septic,septic_words,movej_c3,hermite_rows,derivative,
    boundary,extrema,continuity_metrics,trim_first_tick,smooth_cartesian_c3)
from septic_transport import SepticTransport
from septic_job import verify_septic_job,SCHEMA

class Sink:
 def __init__(self,version=0x50001):self.version=version;self.reads=[];self.writes=[]
 def read(self,a):
  self.reads.append(a)
  return {0x470:self.version,0x828:0x70008,0x404:7,0x800:1}.get(a,0)
 def write(self,a,v):self.writes.append((a,v))

class SepticTests(unittest.TestCase):
 def setUp(self):self.p=JointPlanner(Path(__file__).with_name('profile_grasp_candidate.json'))
 def test_rest_to_rest_c3_split_and_true_time_grid(self):
  p=self.p;start=[-1.5,.2,-.3,.4,0,.1,1.6];goal=[0,.1,-.2,0,.3,0,1.6]
  plans=movej_c3(p,start,goal);metrics=continuity_metrics(plans,p)
  self.assertLess(max(metrics['maximum_boundary_jumps']),1e-9)
  self.assertLess(max(metrics['rest_endpoint_v_a_j']),1e-9)
  t=SepticTransport(Sink(),p);stream=[]
  for part in plans:
   t.submit(part);enc=t.last_encoded_plan;words,_,_,_=septic_words(enc,p);c=np.array(words[:56]).reshape(7,8)/Q24
   for tick in range(enc['ticks']+1):stream.append([derivative(row,tick/enc['ticks']) for row in c])
  ticks=sum(x['ticks'] for x in plans)
  self.assertEqual(len(stream),ticks+1)
  for index,q in enumerate(stream):
   expected=p.to_motor((np.array(start)+(np.array(goal)-start)*derivative(S7,index/ticks)).tolist())
   np.testing.assert_allclose(q,expected,atol=1e-6)
 def test_integer_boundary_constraints_and_analytic_extrema(self):
  p=self.p;rng=np.random.default_rng(2603)
  for _ in range(30):
   start=np.array([0,.3,-.3,0,0,0,.5]);end=start+rng.uniform(-.02,.02,7)
   v=rng.uniform(-.01,.01,(2,7));a=rng.uniform(-.005,.005,(2,7));j=rng.uniform(-.002,.002,(2,7))
   c=hermite_rows(start,end,*v,*a,*j,1.);plan=septic(p,c,1.)
   for k,values in enumerate((np.array([start,end]),v,a,j)):
    np.testing.assert_allclose(boundary(plan,k),values[0],atol=1e-12)
    np.testing.assert_allclose(boundary(plan,k,True),values[1],atol=1e-12)
   words,ends,sv,ev=septic_words(plan,p)
   for i,row in enumerate(np.array(words[:56]).reshape(7,8)):
    self.assertEqual(sum(row),ends[i]);lo,hi=extrema(row)
    dense=np.polynomial.polynomial.polyval(np.linspace(0,1,401),row)
    self.assertLessEqual(lo,min(dense)+1e-5);self.assertGreaterEqual(hi,max(dense)-1e-5)
 def test_rejects_interior_excursion_and_bad_profile(self):
  p=self.p;plan=movej_c3(p,[0,.2,-.2,0,0,0,.3],[.1,.3,-.3,.1,.1,.1,.4])[0]
  bad=copy.deepcopy(plan);bad['motor_coefficients_rad'][0][4]+=10
  with self.assertRaises(ValueError):septic_words(bad,p)
  c=np.zeros((7,8));c[:,0]=[0,.2,-.2,0,0,0,.3];c[1,1]=-2;c[1,2]=2
  with self.assertRaises(ValueError):septic(p,c,10.)
 def test_old_firmware_rejected_before_unmapped_capability_read(self):
  d=Sink(0x50000)
  with self.assertRaises(RuntimeError):SepticTransport(d,self.p)
  self.assertEqual(d.reads,[0x470]);self.assertEqual(d.writes,[])
 def test_live_submit_uses_prevalidated_records_without_numerical_work(self):
  p=self.p;q=[0,.2,-.2,0,0,0,.3];goal=[.1,.3,-.3,.1,.1,.1,.4];parts=movej_c3(p,q,goal)
  job=dict(schema=SCHEMA,profile_id=p.profile_id,start_rad=q,tasks=[dict(segments=parts,goal_rad=goal)])
  records=verify_septic_job(job,p,return_prepared=True);d=Sink();live=SepticTransport(d,p,records)
  with patch('septic_transport.septic_words',side_effect=AssertionError('live roots')),patch('septic_transport.trim_first_tick',side_effect=AssertionError('live trim')):
   for part in parts:live.submit(part)
  self.assertEqual(len(d.writes),74*len(parts));self.assertEqual(live.prepared_index,len(parts))
  d=Sink();live=SepticTransport(d,p,records)
  with self.assertRaisesRegex(ValueError,'prevalidated execution order'):live.submit(copy.deepcopy(parts[0]))
  self.assertEqual(d.writes,[])
 def test_job_rejects_non_c3_jerk_and_pause_before_hardware(self):
  p=self.p;q=[0,.2,-.2,0,0,0,.3];goal=[.1,.3,-.3,.1,.1,.1,.4];parts=movej_c3(p,q,goal)
  job=dict(schema=SCHEMA,profile_id=p.profile_id,start_rad=q,tasks=[dict(segments=parts,goal_rad=goal)])
  self.assertTrue(verify_septic_job(job,p))
  bad=copy.deepcopy(job);bad['tasks'][0]['pause_after_s']=.5
  with self.assertRaisesRegex(ValueError,'braking/resume'):verify_septic_job(bad,p)
  bad=copy.deepcopy(job);b=bad['tasks'][0]['segments'][1]
  c=np.array(b['coefficients_rad']);c[0,3]+=.0001;c[0,4]-=.0001
  bad['tasks'][0]['segments'][1]=septic(p,c,b['duration_s'])
  with self.assertRaisesRegex(ValueError,'not C3'):verify_septic_job(bad,p)
 def test_vertical_circle_is_c3_and_retains_geometry(self):
  from motion_runtime import MotionRuntime
  p=self.p;r=MotionRuntime();q=np.deg2rad([-60,10,-15,15,20,-10]).tolist()+[1.6]
  path=smooth_cartesian_c3(r,p,q,radius=.04,axes=(0,2))
  self.assertLess(max(path['reference_continuity']['maximum_boundary_jumps']),1e-8)
  self.assertLess(max(path['reference_continuity']['rest_endpoint_v_a_j']),1e-8)
  self.assertLess(path['sampled_position_error_m'],1e-5)
  xyz=np.array([r.fk(s['start_rad'][:6]).translation for s in path['segments']])
  self.assertLess(np.ptp(xyz[:,1]),3e-6);self.assertGreater(np.ptp(xyz[:,2]),.079)

if __name__=='__main__':unittest.main()
