import copy,unittest
from unittest.mock import patch
from pathlib import Path
from joint_trajectory import JointPlanner
from continuous_trajectory import split,brake_at
from motion_executor import MoveJExecutor,verify_job,PathVelocityMonitor

class Clock:
 def __init__(self):self.t=0.
 def now(self):return self.t
class FakeTransport:
 def __init__(self):self.active=None;self.queue=None;self.elapsed=0.;self.completed=0;self.sent=[];self.holding=False
 def submit(self,p):
  assert self.queue is None
  self.queue=p;self.sent.append(p)
  self.tail=[round(x*(1<<24)) for x in p['goal_rad']]
 def start(self):self.active=self.queue;self.queue=None;self.elapsed=0.
 def advance(self,dt):
  if self.active:
   self.elapsed+=dt
   if self.elapsed>=self.active['duration_s']:
    self.completed+=1;self.active=None;self.holding=True
  if self.active is None and self.queue:
   self.active=self.queue;self.queue=None;self.elapsed=0;self.holding=False
 def status(self):
  return dict(flags=1|(2 if self.queue else 0),completed=self.completed,holding=self.holding,
      tick=int(self.elapsed/.005),ticks=round(self.active['duration_s']/.005) if self.active else 0)
class FakeExecutor(MoveJExecutor):
 def __init__(self,p,clock):
  self.p=p;self.clock=clock;self.state='HOLDING';self.started=False;self.submitted=0;self.report={};self.transport=FakeTransport();self.events=[]
 def event(self,name,**kw):self.events.append(dict(event=name,time=self.clock.t,**kw))
 def pump(self,seconds=.003):
  self.clock.t+=.005;self.transport.advance(.005);return self.transport.status()
 def diagnostics(self):pass
 def hold(self,seconds=1.):
  start=self.clock.t;before=len(self.transport.sent)
  while self.clock.t-start<seconds:self.pump()
  assert len(self.transport.sent)==before
  self.event('HOLD_VERIFIED',seconds=seconds)
 def settle(self,goal,seconds=.3,tolerance=.004):
  assert max(abs(a-b) for a,b in zip(self.transport.sent[-1]['goal_rad'],goal))<1e-10
  return {'goal':goal}

class ExecutorTests(unittest.TestCase):
 def test_phase_logging_retains_wrap_timestamp_and_discards_crossed_segment(self):
  from types import SimpleNamespace
  for end_count,phase in ((1,0x800),(2,0x800),(1,0xb40),(2,0xb40)):
   with self.subTest(end_count=end_count,phase=phase):
    counts=iter([1,end_count]);regs={phase+16:7,phase+24:0xffffff00,phase+28:0}
    d=SimpleNamespace(read=lambda a:next(counts) if a==phase+12 else regs.get(a,0))
    e=object.__new__(MoveJExecutor);e.d=d;e.s=SimpleNamespace(pump=lambda _:None);e.started=False;e.last_snapshot=-1
    e.report={'stream_initial_counters':dict(expired=0,rx_overflow=0)}
    e.transport=SimpleNamespace(phase_register_base=phase,status=lambda:dict(fault=0,completed=1,tick=7,holding=False))
    with patch('motion_executor.time.monotonic',return_value=1.):e.pump()
    if end_count==1:
     self.assertEqual(e.report['pl_samples'][0]['next_due_us'],0xffffff00)
     self.assertEqual(e.report['pl_samples'][0]['tick'],7)
    else:self.assertNotIn('pl_samples',e.report)
 def test_pause_brakes_holds_and_resumes_without_old_path_replay(self):
  p=JointPlanner(Path(__file__).with_name('profile_grasp_candidate.json'));q=[0,.2,-.2,0,0,0,.2];goal=[.4,.3,-.3,.1,.1,.1,.25]
  plans=split(p,p.movej(q,goal));brakes=[brake_at(p,x) for x in plans]
  resumes=[split(p,p.movej(x['goal_rad'],goal)) for x in brakes]
  clock=Clock();e=FakeExecutor(p,clock)
  with patch('motion_executor.time.monotonic',clock.now):e.movej(plans,goal,.4,brakes,resumes,1.)
  names=[x['event'] for x in e.events]
  self.assertEqual(names,['PATH_START','PAUSE_REQUEST','PAUSED_HOLDING','HOLD_VERIFIED','RESUME','PATH_START','PATH_COMPLETE'])
  request=next(x for x in e.events if x['event']=='PAUSE_REQUEST')
  self.assertTrue(0<request['active_tick']<request['active_ticks'])
  brake=[x for x in e.transport.sent if x['kind']=='CONTROLLED_STOP_CANDIDATE']
  self.assertEqual(len(brake),1);self.assertLess(brake[0]['goal_rad'][0],goal[0])
  self.assertEqual(e.state,'HOLDING')
 def test_reviewed_movej_route_and_stopping_state(self):
  from arm_service import ArmService
  calls=[]
  def backend(job):
   calls.append(job);return dict(status='PASS_REVIEWED_MOTION_JOB',commands=[],reset_confirmed=list(range(1,8)),stop_errors={})
  s=ArmService(execution_backend=backend);job={'tasks':[{'kind':'MoveJ'}]}
  result=s.handle({'sequence':1,'command':'MoveJ','job':job})
  self.assertTrue(result['ok']);self.assertEqual(result['state'],'DISABLED');self.assertEqual(calls,[job])
  with self.assertRaises(ValueError):s.handle({'sequence':2,'command':'MoveL','job':job})
  with self.assertRaises(ValueError):s.handle({'sequence':1,'command':'MoveJ','job':job})

 def test_task_event_accepts_task_name(self):
  from types import SimpleNamespace
  target=SimpleNamespace(report={})
  MoveJExecutor.event(target,'TASK',name='MoveJ_pause_hold_resume',kind='MoveJ')
  self.assertEqual(target.report['events'][0]['name'],'MoveJ_pause_hold_resume')

 def test_speed_monitor_uses_reviewed_limits_but_rejects_fault_speed(self):
  p=JointPlanner(Path(__file__).with_name('profile_grasp_candidate.json'));m=PathVelocityMonitor(p)
  for i in range(30):m.observe(dict(motor=1,time=i*.005,position=.35*i*.005,velocity=.35))
  with self.assertRaises(RuntimeError):m.observe(dict(motor=1,time=.15,position=.0525,velocity=3.))
 def test_encoder_monitor_allows_servo_correction_but_rejects_excess(self):
  p=JointPlanner(Path(__file__).with_name('profile_grasp_candidate.json'));m=PathVelocityMonitor(p)
  for i in range(30):m.observe(dict(motor=3,time=i*.005,position=.29*i*.005,velocity=.29))
  bad=PathVelocityMonitor(p)
  with self.assertRaises(RuntimeError):
   for i in range(30):bad.observe(dict(motor=3,time=i*.005,position=.45*i*.005,velocity=.45))

 def test_job_rejects_brake_gap_before_hardware(self):
  p=JointPlanner(Path(__file__).with_name('profile_grasp_candidate.json'))
  q=[0,.2,-.2,0,0,0,.3];goal=[.01,.21,-.21,.01,.01,.01,.31]
  plans=split(p,p.movej(q,goal));brakes=[brake_at(p,x) for x in plans]
  resumes=[split(p,p.movej(x['goal_rad'],goal)) for x in brakes]
  job=dict(schema='kv260.reviewed_motion_job.v1',profile_id=p.profile_id,start_rad=q,
   tasks=[dict(segments=plans,brakes=brakes,resume_plans=resumes,goal_rad=goal,pause_after_s=.1)])
  verify_job(job,p)
  bad=copy.deepcopy(job);bad['tasks'][0]['brakes'][0]['coefficients_rad'][0][0]+=.01
  with self.assertRaises(ValueError):verify_job(bad,p)

if __name__=='__main__':unittest.main()
