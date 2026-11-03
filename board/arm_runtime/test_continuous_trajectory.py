import copy,random,unittest
from pathlib import Path
import numpy as np
from joint_trajectory import JointPlanner,evaluate,Q24
from continuous_trajectory import cubic_words,split,brake_at,hermite,CartesianPlanner,trim_first_tick,clamped_c2_segments,smooth_cartesian_path
from continuous_transport import ContinuousTransport

PROFILE=Path(__file__).with_name('profile_grasp_candidate.json')
class ContinuousTests(unittest.TestCase):
 def setUp(self):self.p=JointPlanner(PROFILE)
 def test_exact_split_and_bounded_brake(self):
  p=self.p;rng=random.Random(260)
  for _ in range(40):
   a=[rng.uniform(l+.3,h-.3) for l,h in zip(p.lower,p.upper)]
   b=[x+rng.uniform(-.1,.1) for x in a]
   original=p.movej(a,b);parts=split(p,original);elapsed=0
   for part in parts:
    for t in [0,part['duration_s']/2,part['duration_s']]:
     np.testing.assert_allclose(evaluate(part,t)['q'],evaluate(original,min(original['duration_s'],elapsed+t))['q'],atol=1e-12)
    brake=brake_at(p,part);x=evaluate(part,part['duration_s']);y=evaluate(brake,0)
    np.testing.assert_allclose(x['q'],y['q'],atol=1e-12);np.testing.assert_allclose(x['v'],y['v'],atol=1e-12)
    np.testing.assert_allclose(evaluate(brake,brake['duration_s'])['v'],0,atol=1e-12)
    cubic_words(brake,p);elapsed+=part['duration_s']
 def test_next_sample_advances_one_tick_without_duplicate_endpoint(self):
  p=self.p;a=[0,.2,-.2,0,0,0,.2];b=[.4,.3,-.3,.1,.1,.1,.3]
  original=p.movej(a,b);parts=split(p,original)
  for part in parts[1:]:
   trimmed=trim_first_tick(p,part)
   np.testing.assert_allclose(evaluate(trimmed,0)['q'],evaluate(part,.005)['q'],atol=1e-12)
   np.testing.assert_allclose(evaluate(trimmed,trimmed['duration_s'])['q'],evaluate(part,part['duration_s'])['q'],atol=1e-12)
   self.assertEqual(trimmed['ticks'],part['ticks']-1)
   cubic_words(trimmed,p)

 def test_encoded_sample_stream_matches_unsplit_time_grid(self):
  class Device:
   def read(self,a):return {0x470:0x50000,0x404:7,0x800:1}.get(a,0)
   def write(self,a,v):pass
  p=self.p;a=[0,.2,-.2,0,0,0,.2];b=[.4,.3,-.3,.1,.1,.1,.3]
  parent=p.movej(a,b);transport=ContinuousTransport(Device(),p);samples=[]
  for part in split(p,parent):
   transport.submit(part);encoded=transport.last_encoded_plan;words,_,_,_=cubic_words(encoded,p)
   c=np.array(words[:28]).reshape(7,4)/Q24
   for tick in range(encoded['ticks']+1):
    u=tick/encoded['ticks'];motor=((c[:,3]*u+c[:,2])*u+c[:,1])*u+c[:,0]
    samples.append((motor-np.array(p.offset))*np.array(p.direction))
  self.assertEqual(len(samples),parent['ticks']+1)
  for tick,q in enumerate(samples):np.testing.assert_allclose(q,evaluate(parent,min(tick*.005,parent['duration_s']))['q'],atol=1e-6)

 def test_quantized_interior_bounds_and_derivatives(self):
  p=self.p;q=[0,.2,-.2,0,0,0,.2]
  plan=hermite(p,q,q,[0]*7,[0]*7,1.)
  bad=copy.deepcopy(plan);bad['motor_coefficients_rad'][0]=[0,10,-10,0]
  with self.assertRaises(ValueError):cubic_words(bad,p)
  bad=copy.deepcopy(plan);bad['motor_coefficients_rad'][0]=[0,1,0,0]
  with self.assertRaises(ValueError):cubic_words(bad,p)
 def test_transport_rejects_velocity_jump_and_nonrest_initial(self):
  class Device:
   def __init__(self):self.regs={0x470:0x50000,0x404:7,0x800:1}
   def read(self,a):return self.regs.get(a,0)
   def write(self,a,v):pass
  p=self.p;q=[0,.2,-.2,0,0,0,.2];end=q.copy();end[0]=.02
  plan=hermite(p,q,end,[0]*7,[.02]+[0]*6,1.)
  t=ContinuousTransport(Device(),p);t.submit(plan)
  with self.assertRaisesRegex(ValueError,'velocity'):t.submit(hermite(p,end,q,[0]*7,[0]*7,2.))
  with self.assertRaisesRegex(ValueError,'rest'):ContinuousTransport(Device(),p).submit(hermite(p,q,end,[.02]+[0]*6,[0]*7,1.))
 def test_c2_spline_positions_velocities_accelerations_join(self):
  p=self.p;u=np.linspace(0,1,16);base=np.array([0,.3,-.3,0,0,0,.2])
  knots=[(base+.03*np.sin(np.pi*x)*np.array([1,1,-1,1,1,1,0])).tolist() for x in u]
  plans=clamped_c2_segments(p,knots,[100]*15)
  for a,b in zip(plans,plans[1:]):
   x=evaluate(a,a['duration_s']);y=evaluate(b,0)
   for key in ['q','v','a']:np.testing.assert_allclose(x[key],y[key],atol=1e-12)
  np.testing.assert_allclose(evaluate(plans[0],0)['v'],0,atol=1e-12)
  np.testing.assert_allclose(evaluate(plans[-1],plans[-1]['duration_s'])['v'],0,atol=1e-12)

 def test_vertical_xz_circle_geometry_and_reach(self):
  from motion_runtime import MotionRuntime
  p=self.p;r=MotionRuntime();q=np.deg2rad([-60,10,-15,15,20,-10]).tolist()+[1.6]
  path=smooth_cartesian_path(r,p,q,radius=.04,axes=(0,2))
  y0=path['origin_xyz_m'][1];z=[]
  for plan in path['segments']:
   T=r.fk(plan['start_rad'][:6]);self.assertLess(abs(T.translation[1]-y0),2e-6);z.append(T.translation[2])
  self.assertGreater(max(z)-min(z),.079)
  self.assertLess(path['sampled_position_error_m'],1e-5)

 def test_c2_circle_quantized_fk_and_acceleration_smoothness(self):
  from motion_runtime import MotionRuntime
  p=self.p;r=MotionRuntime();q=np.deg2rad([-60,10,-15,15,20,-10]).tolist()+[1.6]
  path=smooth_cartesian_path(r,p,q,radius=.04)
  self.assertLess(max(path['maximum_acceleration_jump_rad_s2']),1e-10)
  self.assertLess(max(path['maximum_jerk_rad_s3']),.5)
  self.assertLess(path['sampled_position_error_m'],.00001)

 def test_80mm_circle_with_closed_gripper_and_radius_scope(self):
  from motion_runtime import MotionRuntime
  p=self.p;c=CartesianPlanner(MotionRuntime(),p);q=np.deg2rad([-60,10,-15,15,20,-10]).tolist()+[1.6]
  path=c.path(q,'circle',radius=.04,axes=(0,1))
  self.assertLess(path['sampled_position_error_m'],.00025)
  self.assertAlmostEqual(path['goal_rad'][6],1.6)
  with self.assertRaises(ValueError):c.path(q,'circle',radius=.041)

 def test_line_circle_round_trip_and_quantized_audit(self):
  from motion_runtime import MotionRuntime
  r=MotionRuntime();p=self.p;planner=CartesianPlanner(r,p)
  q=np.deg2rad([-60,10,-15,15,20,-10]).tolist()+[.2]
  for kind,args in [('line',{'delta':(0,0,.02)}),('circle',{'radius':.01,'axes':(0,1)})]:
   path=planner.path(q,kind,**args)
   self.assertLess(path['sampled_position_error_m'],.00025)
   self.assertLess(path['sampled_orientation_error_rad'],.002)
   if kind=='circle':np.testing.assert_allclose(path['goal_rad'],q,atol=1e-4)
   previous=None
   for part in path['segments']:
    words,end,v0,v1=cubic_words(part,p)
    if previous:
     np.testing.assert_allclose(np.array(words[:28]).reshape(7,4)[:,0]/Q24,previous[0],atol=2/Q24)
     np.testing.assert_allclose(v0,previous[1],atol=2e-4)
    previous=(np.array(end)/Q24,v1)

if __name__=='__main__':unittest.main()
