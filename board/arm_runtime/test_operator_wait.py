import json,time,threading
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock,patch

from control_continuity import await_disable_approval,WaitStateWriter
from motion_executor import MoveJExecutor


class OperatorWaitTests(unittest.TestCase):
    def wait(self,registers,service_error=False,feedback=None):
        with tempfile.TemporaryDirectory() as tmp:
            out=Path(tmp);report={'status':'COMPLETED_TRANSFER_MOTION_VERIFY_IMAGES'}
            e=SimpleNamespace(d=Mock(),s=Mock(),event=Mock(),report=report)
            e.s.feedback=feedback or {}
            e.d.read.side_effect=registers
            observations=[]
            def service(seconds):
                self.assertFalse(report.get('supported_disable_requested',False))
                observations.append(dict(report['hold_controller_observation']))
                (out/'SUPPORTED_DISABLE').touch() # Simulated explicit operator reply.
                if service_error:raise RuntimeError('PL/CAN fault')
            e.s.pump.side_effect=service
            await_disable_approval(e,out,report)
            self.assertTrue(report['supported_disable_requested'])
            self.assertEqual(report['status'],'OPERATOR_WAIT_HARDWARE_FAULT' if report.get('hold_hardware_fault') else 'COMPLETED_TRANSFER_MOTION_VERIFY_IMAGES')
            e.s.stop_all.assert_not_called();e.d.write.assert_not_called();e.d.configure.assert_not_called()
            await_disable_approval(e,out,report)
            self.assertEqual(len(observations),1)
            return observations[0]

    def test_slow_status_storage_does_not_block_control_service(self):
        entered=threading.Event();release=threading.Event();calls=[]
        with tempfile.TemporaryDirectory() as tmp:
            out=Path(tmp);report={'status':'COMPLETED_TRANSFER_MOTION_VERIFY_IMAGES'}
            e=SimpleNamespace(d=Mock(),s=Mock(),event=Mock(),report=report)
            e.s.feedback={};e.d.read.side_effect=lambda a:{0x404:7,0x418:0,0x5a8:127}[a]
            write_text=Path.write_text
            def slow_write(path,text,*args,**kwargs):
                if path.suffix=='.tmp':entered.set();release.wait(2)
                return write_text(path,text,*args,**kwargs)
            def service(seconds):
                calls.append(time.monotonic());time.sleep(.001)
                if entered.is_set() and len(calls)>=30:(out/'SUPPORTED_DISABLE').touch()
            e.s.pump.side_effect=service
            with patch.object(Path,'write_text',slow_write):
                try:
                    await_disable_approval(e,out,report)
                    self.assertTrue(entered.is_set());self.assertFalse(release.is_set())
                    self.assertGreaterEqual(len(calls),30)
                    e.s.stop_all.assert_not_called()
                finally:release.set();time.sleep(.02)

    def test_async_status_keeps_latest_timestamp(self):
        with tempfile.TemporaryDirectory() as tmp:
            writer=WaitStateWriter(tmp)
            try:
                writer.publish({'time':1.});writer.publish({'time':2.})
                deadline=time.monotonic()+1;target=Path(tmp)/'operator_wait_state.json'
                while time.monotonic()<deadline:
                    if target.exists() and json.loads(target.read_text())['time']==2.:break
                    time.sleep(.002)
                self.assertEqual(json.loads(target.read_text())['time'],2.)
            finally:writer.close();writer.thread.join(timeout=1)

    def test_supported_completion_waits_for_explicit_permission(self):
        obs=self.wait(lambda address:{0x404:1,0x418:0,0x5a8:127}[address])
        self.assertEqual(obs['status'],'AWAITING_OPERATOR_CONFIRMATION_HOLD_NOT_GUARANTEED')

    def test_drive_fault_output_stop_is_reported_without_reset(self):
        obs=self.wait(lambda address:{0x404:0,0x418:8,0x5a8:0}[address],True)
        self.assertEqual(obs['status'],'HARDWARE_OUTPUT_STOPPED_REQUIRES_SUPPORT')

    def test_read_failure_never_claims_holding(self):
        def fail(address):raise OSError('unavailable')
        obs=self.wait(fail)
        self.assertEqual(obs['status'],'HARDWARE_STATE_UNKNOWN_REQUIRES_SUPPORT')

    def test_stale_temperature_is_not_presented_as_live(self):
        obs=self.wait(lambda address:{0x404:0,0x418:8,0x5a8:0}[address],
                      feedback={2:dict(time=-100.,temperature=90.,mode=0,fault=16)})
        row=obs['motor_feedback'][0]
        self.assertFalse(row['fresh']);self.assertGreater(row['age_s'],.08)
        self.assertEqual(row['temperature_c'],90.)

    def test_following_trip_keeps_phase_current_and_temperature(self):
        e=object.__new__(MoveJExecutor)
        e.last_diagnostic=0;e.following_since={2:1.};e.report={'active_task':'screw_0_transfer'}
        e.p=SimpleNamespace(current=[1.5,4.5,4.5,2.5,1,1,.5]);e.s=Mock()
        e.s.read.side_effect=lambda mid,index:2. if index==0x7016 else -4.5
        e.s.require.return_value={'position':1.94,'temperature':65.}
        with patch('motion_executor.time.monotonic',return_value=2.):
            with self.assertRaisesRegex(RuntimeError,'Axis 2: sustained following error'):e.diagnostics()
        row=e.report['tracking_diagnostics'][-1]
        self.assertEqual(row['task'],'screw_0_transfer');self.assertEqual(row['temperature_c'],65.)
        self.assertEqual(row['current_limit_a'],4.5);self.assertEqual(row['following_limit_rad'],.04)


if __name__=='__main__':unittest.main()


class FailureEvidenceTests(unittest.TestCase):
    def test_evidence_is_frozen_before_the_wait_and_written_off_the_servicing_thread(self):
        from control_continuity import retain_until_supported
        with tempfile.TemporaryDirectory() as tmp:
            out=Path(tmp);report={'status':'FAILED','feedback':[{'motor':3,'position':i} for i in range(2000)],
                                  'tracking_diagnostics':[{'motor':3,'following_error':.05}]*300,'events':[{'event':'X','time':i} for i in range(500)]}
            e=SimpleNamespace(d=Mock(),s=Mock(),event=Mock(),report=report)
            e.s.feedback={};e.d.read.side_effect=lambda a:{0x404:7,0x418:0,0x5a8:127}[a]
            seen={}
            def service(seconds):
                seen['frozen']=report.get('failure_evidence_frozen');(out/'SUPPORTED_DISABLE').touch()
            e.s.pump.side_effect=service
            retain_until_supported(e,out,report)
            self.assertTrue(seen['frozen'])
            for _ in range(100):
                if (out/'failure_context.json').exists():break
                time.sleep(.02)
            data=json.loads((out/'failure_context.json').read_text())
            self.assertEqual(len(data['feedback_tail']),512);self.assertEqual(data['feedback_tail'][-1]['position'],1999)
            self.assertEqual(len(data['events']),200);self.assertEqual(len(data['tracking_diagnostics']),128)
            e.s.stop_all.assert_not_called();e.d.write.assert_not_called();e.d.configure.assert_not_called()

    def test_freeze_is_idempotent_and_keeps_the_first_error(self):
        from control_continuity import freeze_failure_evidence
        with tempfile.TemporaryDirectory() as tmp:
            report={'feedback':[]};e=SimpleNamespace(report=report)
            freeze_failure_evidence(e,tmp,report,error='first',task='departure')
            freeze_failure_evidence(e,tmp,report,error='second')
            for _ in range(100):
                if (Path(tmp)/'failure_context.json').exists():break
                time.sleep(.02)
            self.assertEqual(json.loads((Path(tmp)/'failure_context.json').read_text())['error'],'first')

    def test_alarm_cadence_and_temperature_escalation(self):
        from control_continuity import support_alarm
        self.assertEqual(support_alarm(1.,0.,5.,40.),(None,5.))
        self.assertEqual(support_alarm(5.,0.,5.,40.),('OPERATOR_SUPPORT_REQUIRED',35.))
        self.assertEqual(support_alarm(36.,0.,35.,65.),('OPERATOR_SUPPORT_CRITICAL_TEMPERATURE',51.))
        self.assertEqual(support_alarm(9.,0.,5.,None),('OPERATOR_SUPPORT_REQUIRED',39.))

    def test_alarms_are_events_only_never_a_stop(self):
        from control_continuity import retain_until_supported
        with tempfile.TemporaryDirectory() as tmp:
            out=Path(tmp);report={'status':'FAILED'};e=SimpleNamespace(d=Mock(),s=Mock(),event=Mock(),report=report)
            e.s.feedback={3:{'time':time.monotonic(),'temperature':70.,'mode':2,'fault':0}};e.d.read.side_effect=lambda a:{0x404:7,0x418:0,0x5a8:127}[a]
            calls=[]
            def service(seconds):
                calls.append(1)
                if len(calls)>5:(out/'SUPPORTED_DISABLE').touch()
            e.s.pump.side_effect=service
            with patch('control_continuity.support_alarm',return_value=('OPERATOR_SUPPORT_CRITICAL_TEMPERATURE',1e9)):
                retain_until_supported(e,out,report)
            names=[c.args[0] for c in e.event.call_args_list]
            self.assertIn('OPERATOR_SUPPORT_CRITICAL_TEMPERATURE',names)
            self.assertEqual(e.event.call_args_list[[c.args[0] for c in e.event.call_args_list].index('OPERATOR_SUPPORT_CRITICAL_TEMPERATURE')].kwargs['hottest_c'],70.)
            e.s.stop_all.assert_not_called();e.d.write.assert_not_called()
