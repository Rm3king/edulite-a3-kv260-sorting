"""Software errors must not turn a supported-return attempt into torque loss."""
import unittest
from types import SimpleNamespace
from unittest.mock import patch,Mock
from control_continuity import emit,finish_failed_job
from motion_executor import MoveJExecutor,PathVelocityMonitor


class ContinuityPolicyTests(unittest.TestCase):
 def test_transfer_wait_reuses_preinitialized_writer(self):
  import tempfile
  from pathlib import Path
  from control_continuity import retain_until_supported
  with tempfile.TemporaryDirectory() as td:
   (Path(td)/'SUPPORTED_DISABLE').write_text('fresh support approval')
   writer=Mock();e=SimpleNamespace(wait_state_writer=writer,event=Mock())
   report={}
   with patch('control_continuity.WaitStateWriter',side_effect=AssertionError('No thread creation while enabled')):
    retain_until_supported(e,td,report)
   writer.close.assert_not_called()
   self.assertTrue(report['supported_disable_requested'])

 def test_closed_stdout_cannot_stop_motion(self):
  e=SimpleNamespace(report={})
  with patch('builtins.print',side_effect=BrokenPipeError('SSH console closed')):
   emit(dict(event='alive'))
   MoveJExecutor.event(e,'MOVING',tick=4)
  self.assertEqual(e.report['events'][0]['tick'],4)

 def owner(self):
  plans=[dict(plan=dict(duration_s=1,n=i)) for i in range(3)]
  e=SimpleNamespace(d=SimpleNamespace(read=lambda a:127 if a==0x5a8 else 0),
   s=SimpleNamespace(pump=Mock(),require=Mock()),
   transport=SimpleNamespace(status=lambda:dict(fault=0),prepared=plans,prepared_index=1),
   execute_segments=Mock(),pump=lambda t:dict(holding=True))
  return e

 def test_cached_return_finishes_only_unsent_prevalidated_records(self):
  e=self.owner();report={}
  finish_failed_job(e,{'tasks':[{'return_to_live_start':True}]},'/unused',report)
  e.execute_segments.assert_called_once_with([r['plan'] for r in e.transport.prepared[1:]])
  self.assertTrue(report['completed_cached_return_before_disable'])

 def test_recovery_error_requires_serviced_hold_not_reset(self):
  for scenario in ('following_error','no_declared_return','drive_fault'):
   with self.subTest(scenario=scenario):
    e=self.owner();report={};job={'tasks':[{'final_zero':True}]}
    if scenario=='following_error':e.execute_segments.side_effect=RuntimeError('following error')
    elif scenario=='no_declared_return':job=None
    else:e.d.read=lambda a:8 if a==0x418 else 127
    with patch('control_continuity.retain_until_supported') as hold:
     finish_failed_job(e,job,'/unused',report)
    hold.assert_called_once_with(e,'/unused',report,None)
    self.assertIn('cached_return_error',report)

 def test_hardware_time_gap_does_not_cancel_trajectory(self):
  p=SimpleNamespace(csp_velocity=[.5]*7);report={};m=PathVelocityMonitor(p,report)
  for stamp,host in [(1000,1.),(6000,2.),(206000,2.2),(211000,2.3)]:
   m.observe(dict(motor=2,pl_rx_us=stamp,time=host,position=0.,velocity=0.))
  self.assertEqual(len(report['velocity_observation_gaps']),1)


if __name__=='__main__':unittest.main()
