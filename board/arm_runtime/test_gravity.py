import copy,struct,time,unittest
from pathlib import Path
from unittest.mock import patch
import numpy as np
from joint_trajectory import JointPlanner,Q24
from motion_runtime import MotionRuntime
from gravity_trajectory import GravityModel,hold_segment,prepare_stream,validate_records,encoded_interval,fixed_sample,measured_start_reference
from gravity_transport import GravityTransport
from gravity_executor import tracking_packet,verify,GravityExecutor,CanDevice,parameter
from septic_trajectory import movej_c3,S7,derivative


class Sink:
 def __init__(self,version=0x60000):self.version=version;self.reads=[];self.writes=[]
 def read(self,a):
  self.reads.append(a);return {0x470:self.version,0xb68:0x70003,0x404:7}.get(a,0)
 def write(self,a,v):self.writes.append((a,v))


class GravityTests(unittest.TestCase):
 def setUp(self):
  self.p=JointPlanner(Path(__file__).with_name('profile_grasp_candidate.json'));self.r=MotionRuntime();self.m=GravityModel(self.r,self.p)
  self.q=np.deg2rad([-60,10,-15,15,20,-10]).tolist()+[1.6]
 def task(self,name='hold',alpha=(1.,1.)):
  return dict(name=name,gravity_alpha=list(alpha),segments=[hold_segment(self.p,self.q,.2)],goal_rad=self.q)
 def job(self,tasks):
  lower=[min(a*d+o,b*d+o) for a,b,d,o in zip(self.p.lower,self.p.upper,self.p.direction,self.p.offset)]
  upper=[max(a*d+o,b*d+o) for a,b,d,o in zip(self.p.lower,self.p.upper,self.p.direction,self.p.offset)]
  return dict(tasks=tasks,motor_lower_rad=lower,motor_upper_rad=upper)
 def test_motor_mapping_and_nominal_wrist_scale(self):
  original=np.array(self.r.kin.compute_gravity(self.q[:6]))[:6]
  np.testing.assert_allclose(self.m.gravity(self.q),original*np.array([-1,1,-1,1,-1,1]),atol=1e-12)
  self.assertAlmostEqual(self.m.gravity(self.q)[2],3.194579,places=5)
  self.assertAlmostEqual(self.m.gravity(self.q)[3],-.69212,places=4)
 def test_only_measured_start_has_explicit_one_code_projection(self):
  motor=self.p.to_motor(self.q);motor[1]=-3.83496299e-5;before=list(motor)
  q,adjustment=measured_start_reference(self.p,motor)
  self.assertEqual(motor,before);self.assertEqual(q[1],0);self.assertAlmostEqual(adjustment[1],-motor[1])
  with self.assertRaises(ValueError):self.p.positions([(a-o)/d for a,o,d in zip(motor,self.p.offset,self.p.direction)])
  motor[1]=-.002
  with self.assertRaisesRegex(ValueError,'one protocol position code'):measured_start_reference(self.p,motor)
 def test_all_ready_before_group_enable_without_per_axis_wait(self):
  actual=self.p.to_motor(self.q)
  class Device:
   pp_base=staticmethod(CanDevice.pp_base)
   bad_axis=0
   def read(self,a):return 0 if self.bad_axis and a==self.pp_base(self.bad_axis)+24 else 63
  class PreparedSession:
   def __init__(self):
    self.static={mid:tracking_packet(mid,actual[mid-1],0,80,4) for mid in range(1,7)}
    self.targets={mid:actual[mid-1] for mid in range(1,8)};self.active=set();self.calls=[];self.feedback={}
   def send(self,packet):self.calls.append(('send',packet[0]>>24,packet[0]&255))
   def pump(self,seconds):
    self.calls.append(('pump',seconds));self.feedback={mid:dict(time=time.monotonic(),position=actual[mid-1]) for mid in range(1,8)}
   def require(self,mid,mode):return self.feedback[mid]
   def read(self,mid,index):return actual[mid-1]
  e=GravityExecutor.__new__(GravityExecutor);e.d=Device();e.s=PreparedSession();e.report={}
  e.enable_prepared_axes(actual)
  self.assertEqual([x[2] for x in e.s.calls if x[:2]==('send',3)],list(range(1,8)))
  self.assertEqual(e.s.calls[14],('pump',.02));self.assertTrue(all(x[0]=='send' for x in e.s.calls[:14]))
  self.assertEqual(e.s.active,set(range(1,8)));self.assertLess(e.report['enable_batch_submit_seconds'],.02)
  e.d=Device();e.d.bad_axis=4;e.s=PreparedSession();e.report={}
  with self.assertRaisesRegex(RuntimeError,'readiness guards'):e.enable_prepared_axes(actual)
  self.assertEqual(e.s.calls,[]);self.assertFalse(e.report.get('enable_attempted'))
 def test_static_packets_match_fpga_rounding(self):
  t=self.task();records=prepare_stream(self.m,[t]);validate_records(records,self.job([t]),self.p,self.m);w=records[0]['words'];tau=self.m.gravity(self.q)
  for i,q in enumerate(self.p.to_motor(self.q)[:6]):
   ident,data=tracking_packet(i+1,q,tau[i],80,4);p,v,kp,kd=struct.unpack('>HHHH',data)
   codes=[(fixed_sample(w[f*56+i*8:f*56+i*8+8],0)*65535+(1<<23))>>24 for f in range(3)]
   self.assertEqual(codes,[p,v,(ident>>8)&65535]);self.assertEqual(w[210+i],kp<<16|kd)
 def test_ab_changes_only_torque(self):
  end=self.q.copy();end[0]+=.02;plans=movej_c3(self.p,self.q,end)
  off=[dict(name='off',gravity_alpha=[0.,0.],segments=plans,goal_rad=end)]
  on=copy.deepcopy(off);on[0].update(name='on',gravity_alpha=[1.,1.])
  a,b=prepare_stream(self.m,off),prepare_stream(self.m,on)
  for x,y in zip(a,b):
   self.assertEqual(x['words'][:112],y['words'][:112]);self.assertEqual(x['words'][210:],y['words'][210:]);self.assertNotEqual(x['words'][112:168],y['words'][112:168])
 def test_ramp_restriction_keeps_original_time(self):
  tasks=[self.task(),self.task('ramp',(1.,0.))];records=prepare_stream(self.m,tasks);r=records[1]
  self.assertEqual(r['plan']['ticks'],39)
  t=self.m.gravity(self.q)[2]*(1-derivative(S7,1/40))
  got=(r['words'][128]/Q24-.5)*28
  self.assertAlmostEqual(got,t,delta=3e-6)
  validate_records(records,self.job(tasks),self.p,self.m)
 def test_torque_cap_and_encoded_sample_violation_rejected(self):
  t=self.task();self.m.caps[2]=.1
  with self.assertRaisesRegex(ValueError,'torque cap'):prepare_stream(self.m,[t])
  self.m.caps[2]=5.5;r=prepare_stream(self.m,[t]);w=list(r[0]['words']);w[128]=Q24;r[0]['words']=tuple(w)
  with self.assertRaisesRegex(ValueError,'encoded field'):validate_records(r,self.job([t]),self.p,self.m)
 def test_version_checked_before_unmapped_mmio(self):
  d=Sink(0x50001)
  with self.assertRaises(RuntimeError):GravityTransport(d,self.p,[])
  self.assertEqual(d.reads,[0x470]);self.assertEqual(d.writes,[])
 def test_v61_retains_existing_gravity_register_abi(self):
  d=Sink(0x60001);GravityTransport(d,self.p,[])
  self.assertEqual(d.reads,[0x470,0xb68]);self.assertEqual(d.writes,[])
 def test_transport_never_solves_or_encodes_live(self):
  task=self.task();r=prepare_stream(self.m,[task]);d=Sink();t=GravityTransport(d,self.p,r)
  with patch.object(GravityModel,'encode',side_effect=AssertionError('live encode')):t.submit(task['segments'][0])
  self.assertEqual(len(d.writes),221);self.assertEqual(d.writes[-1],(0xb60,1))
  with self.assertRaises(ValueError):t.submit(task['segments'][0])
  d=Sink();t=GravityTransport(d,self.p,r)
  with self.assertRaisesRegex(ValueError,'reviewed order'):t.submit(copy.deepcopy(task['segments'][0]))
  self.assertEqual(d.writes,[])
 def test_synthetic_or_stale_model_job_rejected_before_io(self):
  j=dict(schema='kv260.reviewed_gravity_job.v1',profile_id=self.p.profile_id,model_id=self.r.model_id,gravity_profile_id=self.m.id,hardware_start_verified=False)
  with self.assertRaisesRegex(ValueError,'synthetic'):verify(j,self.p,self.m)
 def test_rest_protocol_codes_and_gain_bounds(self):
  packet=tracking_packet(3,0,0,80,4);p,v,kp,kd=struct.unpack('>HHHH',packet[1])
  self.assertEqual((p,v,packet[0]>>8&65535),(32768,32768,32768))
  self.assertEqual((kp,kd),(10485,52428))
  bounds=encoded_interval(self.job([]),self.p,self.m,2)
  self.assertGreaterEqual(p,bounds[0]&65535);self.assertLessEqual(p,bounds[0]>>16)
 def test_restore_is_disabled_configuration_only_and_fails_closed(self):
  class Device:
   pp_base=staticmethod(CanDevice.pp_base)
   def __init__(self):self.reg={0x470:0x60000};self.modes={};self.caps={}
   def read(self,a):return self.reg.get(a,0)
   def write(self,a,v):self.reg[a]=v
   def configure(self,mode):self.reg[0x404]=7
   def stop(self):self.reg[0x404]=0
   def check_motion_guard(self,ident,data):return CanDevice.check_motion_guard(self,ident,data)
   def check_pp_guard(self,ident,data):return CanDevice.check_pp_guard(self,ident,data)
  class ConfigurationSession:
   fail=False
   def __init__(self,d,r):self.d,self.r=d,r
   def write_verified(self,mid,index,value,integer=False):
    if self.fail:raise RuntimeError('readback failed')
    ident,data=parameter(mid,index,value,integer);self.d.check_motion_guard(ident,data)
    self.r['commands'].append(dict(id=ident,data=data.hex()))
    (self.d.modes if index==0x7005 else self.d.caps)[mid]=value
  e=GravityExecutor.__new__(GravityExecutor);e.p=self.p;e.d=Device()
  e.report=dict(commands=[],readbacks={},original_motor_parameters={str(mid):dict(mode=5,torque_limit_nm=14. if mid<=3 else 6.) for mid in range(1,7)})
  with patch('gravity_executor.Session',ConfigurationSession):e.restore_csp_configuration()
  self.assertTrue(e.report['restored_original_csp_and_torque_limits']);self.assertEqual(len(e.report['commands']),12)
  self.assertEqual(e.d.modes,{i:5 for i in range(1,7)});self.assertEqual(e.d.caps,{i:14. if i<=3 else 6. for i in range(1,7)})
  self.assertEqual(e.d.read(0x404),0);self.assertEqual(e.d.read(0x5a0),0)
  self.assertTrue(all(e.d.read(e.d.pp_base(i))==0 for i in range(1,8)))
  e.d=Device();ConfigurationSession.fail=True
  with patch('gravity_executor.Session',ConfigurationSession),self.assertRaisesRegex(RuntimeError,'readback failed'):e.restore_csp_configuration()
  self.assertEqual(e.d.read(0x404),0);self.assertEqual(e.d.read(0x5a0),0)
  self.assertTrue(all(e.d.read(e.d.pp_base(i))==0 for i in range(1,8)))


if __name__=='__main__':unittest.main()
