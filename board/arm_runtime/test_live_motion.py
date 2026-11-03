import copy,json,math,pickle,time,unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch
from joint_trajectory import JointPlanner
from live_motion import current_reference,rebind_job,POLICY,_worker
from septic_trajectory import movej_c3
from septic_job import SCHEMA,verify_septic_job

ROOT=Path(__file__).parent

class LiveMotionTests(unittest.TestCase):
 def setUp(self):
  self.p=JointPlanner(ROOT/'profile_grasp_candidate.json')
  self.q=[-1.5,.01,-.04,.5,.1,.2,1.6];self.anchor=[-.6,.3,-.5,.4,.1,.1,.35]
 def job(self):
  start=self.q;anchor=self.anchor;home=start[:6]+[1.6];released=start[:6]+[.35]
  bounds=[sorted((a*d+o,b*d+o)) for a,b,d,o in zip(self.p.lower,self.p.upper,self.p.direction,self.p.offset)]
  return dict(schema=SCHEMA,start_policy=POLICY,profile_id=self.p.profile_id,model_id='test',hardware_start_verified=True,start_rad=start,start_motor_rad=self.p.to_motor(start),motor_lower_rad=[b[0] for b in bounds],motor_upper_rad=[b[1] for b in bounds],tasks=[
   dict(kind='MoveJ',name='entry',segments=movej_c3(self.p,start,anchor),goal_rad=anchor),
   dict(kind='MoveJ',name='return',segments=movej_c3(self.p,anchor,home),goal_rad=home,return_to_live_start=True),
   dict(kind='Gripper',name='release',segments=movej_c3(self.p,home,released),goal_rad=released,gripper_only=True)])
 def test_shifted_current_start_replans_entry_and_return_and_keeps_grasp(self):
  job=self.job();old=copy.deepcopy(job);now=[-1.2,.1,-.15,.4,.05,.15,1.4]
  bound,prepared=rebind_job(job,self.p,self.p.to_motor(now))
  self.assertEqual(job,old)
  self.assertEqual(bound['start_rad'],now)
  self.assertEqual(bound['tasks'][0]['segments'][0]['start_rad'],now)
  self.assertEqual(bound['tasks'][0]['goal_rad'],self.anchor)
  self.assertEqual(bound['tasks'][1]['goal_rad'][:6],now[:6])
  self.assertEqual(bound['tasks'][2]['goal_rad'],now[:6]+[.35])
  verify_septic_job(bound,self.p)
  # The worker sends the job and prepared records as ONE pickle graph.
  cloned,records=pickle.loads(pickle.dumps((bound,prepared)))
  self.assertIs(records[0]['plan'],cloned['tasks'][0]['segments'][0])
  self.assertEqual(len(records),sum(len(t['segments']) for t in bound['tasks']))
 def test_endpoint_offsets_are_captured_without_widening_commands(self):
  q=self.q.copy();q[1]=-.01;q[2]=.007
  motor=[a*d+o for a,d,o in zip(q,self.p.direction,self.p.offset)]
  ref,adjust=current_reference(self.p,motor)
  self.assertLess(max(abs(x) for x in ref[1:3]),1e-6);self.assertNotEqual(adjust,[0.]*7)
  bound,_=rebind_job(self.job(),self.p,motor)
  self.assertEqual(bound['start_motor_rad'],motor)
  for t in bound['tasks']:
   self.p.positions(t['goal_rad'])
 def test_bad_feedback_and_unmarked_stale_jobs_are_rejected(self):
  with self.assertRaises(ValueError):current_reference(self.p,[math.nan]*7)
  q=self.q.copy();q[1]=-.5
  with self.assertRaises(ValueError):current_reference(self.p,[-x if d<0 else x for x,d in zip(q,self.p.direction)])
  job=self.job();job.pop('start_policy')
  with self.assertRaises(ValueError):rebind_job(job,self.p,self.p.to_motor(self.q))
 def test_current_preparation_groups_enable_and_ignores_old_snapshot(self):
  from motion_executor import MoveJExecutor
  from arm_commission import parameter
  p=self.p;job=self.job();motor=p.to_motor(self.q)
  class Device:
   def __init__(self):self.reg={0x470:0x60000};self.active=0
   def read(self,a):
    if a==0x5a8:return self.active
    if any(a==self.pp_base(i)+24 for i in range(1,8)):return 63
    return self.reg.get(a,0)
   def write(self,a,v):self.reg[a]=v
   def pp_base(self,i):return 0x680 if i==7 else 0x700+(i-1)*32
   def configure(self,mode):self.reg[0x404]=3
   def stop(self):self.reg[0x404]=0
  class Session:
   def __init__(self,d):self.d=d;self.active=set();self.targets={};self.plans={};self.ops=[];self.prepared=set();self.clock=0.;self.fresh={}
   def send(self,pkt):
    ident,data=pkt;mid=ident&255;typ=ident>>24
    self.ops.append((typ,mid))
    idx=int.from_bytes(data[:2],'little')
    if typ==4 or (typ==18 and idx==0x7028):self.fresh[mid]=self.clock
    if typ==18 and idx==0x7016 and self.clock-self.fresh.get(mid,-1)>.1:
     raise AssertionError('guarded hold target used stale disabled Type2')
    if typ==18 and idx==0x7016:self.prepared.add(mid)
    if typ==3:
     self.assert_ready=len(self.prepared)==7
     if not self.assert_ready:raise AssertionError('enabled before all configured')
     self.d.active|=1<<(mid-1)
   def pump(self,t):self.clock+=t
   def require(self,mid,mode=None):return {'time':time.monotonic(),'position':motor[mid-1],'mode':mode}
   def read(self,mid,idx):self.clock+=.003;return motor[mid-1]
   def write_verified(self,mid,idx,val,integer=False):
    if idx==0x7016:raise AssertionError('disabled loc_ref mirrors measured position; do not demand an echo')
    self.send(parameter(mid,idx,val,integer));self.clock+=.02
  d=Device();s=Session(d);e=object.__new__(MoveJExecutor)
  e.p=p;e.d=d;e.s=s;e.report={};e.event=lambda *a,**k:None
  job['start_motor_rad']=[0.]*7
  e.prepare(job)
  self.assertEqual(d.active,127);self.assertEqual(e.state,'HOLDING')
  self.assertEqual(s.targets,dict(enumerate(motor,1)))
  self.assertEqual([m for typ,m in s.ops if typ==3],list(range(1,8)))

if __name__=='__main__':unittest.main()
