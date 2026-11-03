"""Regress the receive-pause -> stale-recovery -> blanket-reset incident."""
import multiprocessing,threading,time,tempfile,unittest,pickle
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch
from autotune import HardwareVelocityMonitor,upload_evidence,return_to_support,retain_until_supported,receive_records,TrialEvidence,TuningExecutor


class ContinuityTests(unittest.TestCase):
 def test_optional_current_readout_and_timeout_do_not_disable(self):
  e=TuningExecutor.__new__(TuningExecutor);e.report={'collect_drive_telemetry':True}
  calls=[]
  def read(mid,index):
   calls.append((mid,index));return 49.5 if index==0x701c else 1.8
  e.s=SimpleNamespace(active={2,3},read=read,feedback={2:dict(temperature=35),3:dict(temperature=36)})
  with patch('autotune.time.monotonic',return_value=1.):e.diagnostics()
  with patch('autotune.time.monotonic',return_value=1.1):e.diagnostics()
  with patch('autotune.time.monotonic',return_value=1.3):e.diagnostics()
  self.assertEqual(calls,[(2,0x701a),(3,0x701a),(3,0x701c)])
  self.assertEqual(e.report['drive_telemetry'][1]['vbus_v'],49.5)
  e.s.read=lambda *args:(_ for _ in ()).throw(TimeoutError('optional query'))
  with patch('autotune.time.monotonic',return_value=1.6):e.diagnostics()
  self.assertEqual(e.s.active,{2,3});self.assertEqual(len(e.report['drive_telemetry_errors']),1)

 def test_trial_recording_gate_drops_analysis_hold_samples_without_changing_service(self):
  gate=[False];frames=TrialEvidence(gate)
  for n in range(1000):frames.append(n)
  self.assertEqual(frames,[])
  gate[0]=True
  for n in range(10):frames.append(n)
  gate[0]=False
  for n in range(1000):frames.append(n)
  self.assertEqual(frames,list(range(10)))

 def test_upload_preserves_recording_gate_for_next_trial(self):
  class Pipe:
   def send(self,message):pass
  replies=iter([('analysis_ready',None),('chunk_ok',None),('chunk_ok',None),('ok',{'score':1})])
  gate=[True];evidence=TrialEvidence(gate);evidence.append({'motor':1})
  report={'feedback':evidence};gate[0]=False
  result=upload_evidence(Pipe(),report,{}, {},lambda:next(replies),lambda t:report['feedback'].append({'holding':True}))
  self.assertEqual(result,{'score':1});self.assertIs(report['feedback'].gate,gate)
  self.assertEqual(report['feedback'],[])
  gate[0]=True;report['feedback'].append({'motor':2})
  self.assertEqual(report['feedback'],[{'motor':2}])

 def test_partial_record_transfer_keeps_servicing_can_during_worker_pause(self):
  a,b=multiprocessing.Pipe();obj={'payload':'x'*60000};data=pickle.dumps(obj,protocol=5);errors=[]
  def sender():
   try:
    for off in range(0,len(data),2048):
     if off==2048:time.sleep(.15)
     b.send_bytes(data[off:off+2048])
   except BaseException as exc:errors.append(exc)
   finally:b.close()
  thread=threading.Thread(target=sender,daemon=True);thread.start();calls=[time.monotonic()]
  def pump(t):calls.append(time.monotonic());time.sleep(.0005)
  try:records=receive_records(a,'records_chunked',[len(data)],pump)
  finally:a.close();thread.join(timeout=2)
  self.assertEqual(records,[obj]);self.assertEqual(errors,[])
  self.assertGreater(len(calls),100)
  self.assertLess(max(y-x for x,y in zip(calls,calls[1:])),.08)

 def test_host_pause_and_hardware_gap_do_not_raise_or_fake_velocity(self):
  report={};p=SimpleNamespace(csp_velocity=[.5]*7)
  m=HardwareVelocityMonitor(p,report)
  f=dict(motor=1,pl_rx_us=(1<<32)-3000,time=1.,position=0.,velocity=0.)
  m.observe(f)
  m.observe(dict(f,pl_rx_us=2000,time=1.5))  # Linux stalled 0.5 s; PL did not.
  self.assertAlmostEqual(m.clocks[1],.005)
  self.assertNotIn('velocity_observation_gaps',report)
  m.observe(dict(f,pl_rx_us=202000,time=1.7))
  m.observe(dict(f,pl_rx_us=201000,time=1.8))
  self.assertEqual(len(report['velocity_observation_gaps']),2)
  self.assertEqual(len(m.history[1]),1)

 def test_analysis_backpressure_keeps_parent_servicing(self):
  a,b=multiprocessing.Pipe();errors=[];seen=[]
  def worker():
   try:
    command,header=b.recv();self.assertEqual(command,'analyze')
    time.sleep(.15)  # Old-report deallocation before readiness.
    b.send(('analysis_ready',None));n=0
    while True:
     key,items=b.recv()
     if key=='end':break
     seen.append((key,len(items)));n+=1
     if n%25==0:time.sleep(.10)
     b.send(('chunk_ok',None))
    b.send(('ok',dict(score=1.)))
   except BaseException as exc:errors.append(exc)
   finally:b.close()
  thread=threading.Thread(target=worker,daemon=True);thread.start();pumps=[time.monotonic()]
  def pump(seconds):pumps.append(time.monotonic());time.sleep(.0005)
  def wait():
   until=time.monotonic()+10
   while not a.poll():
    pump(.001)
    if time.monotonic()>until:raise TimeoutError('test worker')
   return a.recv()
  report=dict(feedback=[dict(payload='x'*1024,n=i) for i in range(2400)],segment_submissions=[dict(payload='x'*30000)]*3)
  try:result=upload_evidence(a,report,{},dict(rx_lost=0),wait,pump)
  finally:a.close();thread.join(timeout=2)
  self.assertFalse(thread.is_alive());self.assertEqual(errors,[])
  self.assertEqual(result['score'],1.)
  self.assertLess(max(y-x for x,y in zip(pumps,pumps[1:])),.08)
  self.assertEqual(sum(n for key,n in seen if key=='feedback'),2400)
  self.assertTrue(all(n==1 for key,n in seen if key=='segment_submissions'))

 def test_recovery_refreshes_before_require_then_returns(self):
  calls=[];fresh=[False];summary={}
  def pump(t):fresh[0]=True;calls.append(('refresh',t))
  def require(mid,mode):self.assertTrue(fresh[0]);calls.append(('require',mid))
  e=SimpleNamespace(d=SimpleNamespace(read=lambda a:127 if a==0x5a8 else 0),
   s=SimpleNamespace(pump=pump,require=require),
   transport=SimpleNamespace(status=lambda:dict(fault=0),prepared=[],prepared_index=0),
   pump=lambda t:dict(holding=True),event=lambda *a:None,execute_segments=lambda plans:calls.append(('return',len(plans))))
  return_to_support(e,[dict(original_plan={})],'0',summary)
  self.assertTrue(summary['controlled_return_before_disable'])
  self.assertEqual(calls[0],('refresh',.03));self.assertIn(('return',1),calls)

 def test_failed_recovery_holds_until_explicit_support_no_reset(self):
  with tempfile.TemporaryDirectory() as out:
   marker=Path(out)/'SUPPORTED_DISABLE';summary={};beats=[];cycles=[0]
   def pump(t):
    cycles[0]+=1
    if cycles[0]==8:marker.touch()
    raise RuntimeError('Axis 1: missing/stale Type2')
   e=SimpleNamespace(s=SimpleNamespace(pump=pump),d=SimpleNamespace(heartbeat=lambda:beats.append(1)),report=dict(frames=list(range(4200))),event=lambda *a,**k:None)
   with patch('autotune.time.sleep'):
    retain_until_supported(e,out,summary)
   self.assertEqual(cycles[0],8);self.assertEqual(len(beats),8)
   self.assertTrue(summary['supported_disable_requested'])
   self.assertEqual(len(summary['hold_service_errors']),1)
   self.assertLessEqual(len(e.report['frames']),4096)


if __name__=='__main__':unittest.main()
