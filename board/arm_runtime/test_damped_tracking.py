import copy,json,pickle,tempfile,unittest
from pathlib import Path
from unittest.mock import patch
from types import SimpleNamespace
import numpy as np
from joint_trajectory import JointPlanner,Q24
from motion_runtime import MotionRuntime
from gravity_trajectory import GravityModel,prepare_stream,validate_records,fixed_sample
from gravity_executor import GravityExecutor,tracking_packet
from live_motion import current_reference,_worker,POLICY,bind_while_holding
from septic_trajectory import movej_c3
from septic_job import SCHEMA

ROOT=Path(__file__).parent
class DampedTrackingTests(unittest.TestCase):
 def setUp(self):
  self.p=JointPlanner(ROOT/'profile_grasp_damped_candidate.json');self.r=MotionRuntime()
  self.m=GravityModel(self.r,self.p,ROOT/'tracking_damped_candidate.json')
  self.q=[-1.,.3,-.5,.4,.2,-.1,.35]
 def test_rnea_decomposition_and_motor_power(self):
  v=[.1,-.2,.15,.1,-.05,.08,3.];a=[.2,.1,-.1,.3,.1,-.2,5.]
  g=np.array(self.r.kin.compute_gravity(self.q[:6])[:6])
  total=np.array(self.r.kin.inverse_dynamics(self.q[:6],v[:6]+[0.],a[:6]+[0.])[:6])
  tau=self.m.feedforward(self.q,v,a)
  expected=g+.5*(total-g)
  np.testing.assert_allclose(tau,expected*self.p.direction[:6],atol=1e-12)
  self.assertAlmostEqual(np.dot(tau,np.array(v[:6])*self.p.direction[:6]),np.dot(expected,v[:6]),places=12)
  # The gripper motor is NOT Pinocchio model joint 7.
  v[6]=-100.;a[6]=200.
  np.testing.assert_array_equal(tau,self.m.feedforward(self.q,v,a))
 def test_static_reduces_to_gravity_and_rest_velocity(self):
  np.testing.assert_allclose(self.m.feedforward(self.q,[0.]*7,[0.]*7),self.m.gravity(self.q),atol=1e-12)
  goal=self.q.copy();goal[0]+=.05;plans=movej_c3(self.p,self.q,goal)
  task=dict(name='probe',goal_rad=goal,gravity_alpha=[1.,1.],segments=plans)
  bounds=[sorted((a*d+o,b*d+o)) for a,b,d,o in zip(self.p.lower,self.p.upper,self.p.direction,self.p.offset)]
  records=prepare_stream(self.m,[task]);validate_records(records,dict(tasks=[task],motor_lower_rad=[x[0] for x in bounds],motor_upper_rad=[x[1] for x in bounds]),self.p,self.m)
  for i in range(6):
   row=records[-1]['words'][56+i*8:64+i*8]
   self.assertEqual((fixed_sample(row,Q24)*65535+(1<<23))>>24,32768)
  self.assertLess(max(r['sampled_gravity_fit_error_nm'] for r in records),1e-4)
 def test_hold_uses_current_feedback_not_stored_pose_or_csp_arm_writes(self):
  motor=self.p.to_motor(self.q);motor[1]=-.001
  sent=[];e=GravityExecutor.__new__(GravityExecutor);e.p=self.p;e.model=self.m
  e.s=SimpleNamespace(targets={},static={},send=sent.append)
  ref,adjustment=e.capture_tracking_hold(motor)
  self.assertLess(ref[1],1e-6);self.assertGreater(adjustment[1],0)
  self.assertEqual([x[0]>>24 for x in sent],[1]*6+[18])
  tau=self.m.gravity(ref)
  self.assertEqual(sent[2],tracking_packet(3,e.s.targets[3],tau[2],80,5))
  with patch.object(self.m,'gravity',return_value=np.array([100.]*6)):
   with self.assertRaisesRegex(ValueError,'torque cap'):e.capture_tracking_hold(motor)
 def test_worker_rebinds_positive_stiffness_and_preserves_original_plan_identity(self):
  goal=self.q.copy();goal[0]+=.02
  bounds=[sorted((a*d+o,b*d+o)) for a,b,d,o in zip(self.p.lower,self.p.upper,self.p.direction,self.p.offset)]
  job=dict(schema=SCHEMA,profile_id=self.p.profile_id,model_id=self.r.model_id,hardware_start_verified=True,start_policy=POLICY,start_rad=self.q,start_motor_rad=self.p.to_motor(self.q),motor_lower_rad=[x[0] for x in bounds],motor_upper_rad=[x[1] for x in bounds],tasks=[dict(name='probe',kind='MoveJ',goal_rad=goal,segments=movej_c3(self.p,self.q,goal))])
  current=self.q.copy();current[0]-=.01
  class Pipe:
   def __init__(s):s.messages=[];s.payloads=[]
   def send(s,x):s.messages.append(x)
   def recv(s):return self.p.to_motor(current)
   def send_bytes(s,x):s.payloads.append(x)
   def close(s):pass
  conn=Pipe();_worker(conn,job,str(ROOT/'profile_grasp_damped_candidate.json'),str(ROOT/'tracking_damped_candidate.json'))
  self.assertEqual(conn.messages[0],('ready',None));self.assertEqual(conn.messages[1][0],'ok')
  header=pickle.loads(conn.payloads[0]);self.assertEqual(header['job']['start_rad'],current)
  for data in conn.payloads[1:]:
   _,plan,record=pickle.loads(data)
   self.assertIs(record['original_plan'],plan);self.assertEqual(len(record['words']),220)
 def test_invalid_dynamic_scale_rejected(self):
  config=copy.deepcopy(self.m.config)
  for invalid in (-1,2,True,float('nan')):
   config['inverse_dynamics_scale']=invalid
   with tempfile.NamedTemporaryFile(mode='w') as f:
    json.dump(config,f);f.flush()
    with self.assertRaisesRegex(ValueError,'inverse dynamics scale'):GravityModel(self.r,self.p,f.name)
 def friction_model(self,coefficient=None,speed=.05):
  config=copy.deepcopy(self.m.config);config['inverse_dynamics_scale']=0
  config['friction_coulomb_nm']=[0,.1,.05,0,0,0] if coefficient is None else coefficient
  config['friction_velocity_scale_rad_s']=speed
  with tempfile.NamedTemporaryFile(mode='w') as f:
   json.dump(config,f);f.flush();return GravityModel(self.r,self.p,f.name)
 def test_smooth_friction_uses_motor_direction_and_vanishes_at_rest(self):
  m=self.friction_model();v=np.array([.1,-.02,.03,.1,-.05,.08,100.])
  extra=m.feedforward(self.q,v,[0.]*7)-m.gravity(self.q)
  motor_v=v[:6]*np.array(self.p.direction[:6])
  np.testing.assert_allclose(extra,m.friction_coulomb*np.tanh(motor_v/.05),atol=1e-12)
  self.assertGreater(float(extra@motor_v),0)
  np.testing.assert_array_equal(m.feedforward(self.q,[0.]*7,[0.]*7),m.gravity(self.q))
  v[6]=-100.;np.testing.assert_allclose(extra,m.feedforward(self.q,v,[0.]*7)-m.gravity(self.q),atol=1e-12)
 def test_friction_changes_only_torque_stream_and_keeps_rest_exact(self):
  m=self.friction_model();off=self.friction_model([0.]*6)
  goal=self.q.copy();goal[1]+=.05;goal[2]-=.08
  for plan in movej_c3(self.p,self.q,goal,minimum_duration_s=3.):
   a,b=off.encode(plan),m.encode(plan)
   self.assertEqual(a['words'][:112],b['words'][:112])
   self.assertEqual(a['words'][168:],b['words'][168:])
   self.assertLess(b['sampled_gravity_fit_error_nm'],1e-4)
  from gravity_trajectory import hold_segment
  hold=hold_segment(self.p,self.q,.35)
  self.assertEqual(off.encode(hold)['words'],m.encode(hold)['words'])
 def test_invalid_friction_configuration_rejected_offline(self):
  for invalid in ([0.]*5,[-1.]*6,[float('nan')]*6,[100.]*6):
   with self.assertRaisesRegex(ValueError,'friction'):self.friction_model(invalid)
  for invalid in (0,-1,True,float('inf')):
   with self.assertRaisesRegex(ValueError,'friction'):self.friction_model(speed=invalid)
 def test_changed_worker_gain_config_cannot_reach_pl(self):
  class Pipe:
   def poll(s):return True
   def send(s,x):pass
   def recv(s):return 'ok',0
   def recv_bytes(s,n):return pickle.dumps({'job':{'tracking_profile_id':'different'},'job_sha256':'test'})
   def close(s):pass
  process=SimpleNamespace(is_alive=lambda:False,join=lambda **k:None)
  session=SimpleNamespace(read=lambda mid,index:self.p.to_motor(self.q)[mid-1],pump=lambda dt:None)
  e=SimpleNamespace(s=session,p=self.p,report={},d=SimpleNamespace(read=lambda addr:0),model=self.m,
      capture_tracking_hold=lambda motor:None,transport=SimpleNamespace(prepared='untouched'))
  with self.assertRaisesRegex(ValueError,'differs from enabled hold'):
   bind_while_holding(e,{},None,(Pipe(),process))
  self.assertEqual(e.transport.prepared,'untouched')

if __name__=='__main__':unittest.main()
