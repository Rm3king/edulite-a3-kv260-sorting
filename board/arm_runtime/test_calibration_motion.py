"""Offline gates for a mounted board: no test opens UIO or sends real CAN."""
import copy
import math
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch

from calibration_motion import (AXES, MASK, ArmOnlyDevice, ArmOnlySession,
                                CalibrationExecutor, make_job, verify_job, recover_or_retain, capture_while_holding, job_digest, motion_gc_scope)
from arm_commission import parameter
from a3_protocol import enable, disable
from joint_trajectory import JointPlanner


class CalibrationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.p = JointPlanner(Path(__file__).with_name('profiles')/'release_20261019_motion.json')
        # Endpoint sensor offsets plus a gripper value beyond the old profile
        # reproduce the current case without silently clipping the gripper.
        cls.motor = [.65, -.00008, -.0005, .38, .0006, -.0358, 1.69155]
        cls.capture = dict(status='PASS_READ_ONLY_ALL_7', synthetic_for_planning_only=True,
                           transactions=[dict(motor=i+1, decoded=dict(valid=True, position_motor_rad=x))
                                         for _ in range(3) for i, x in enumerate(cls.motor)])
        cls.job = make_job(cls.capture, cls.p)

    def test_gripper_feedback_is_preserved_and_not_planned(self):
        j = self.job
        self.assertEqual(j['observed_gripper_motor_rad'], self.motor[6])
        self.assertGreater(self.motor[6], self.p.upper[6])
        self.assertEqual(j['start_rad'][6], 0.)
        self.assertEqual(j['start_motor_rad'], self.motor[:6])
        self.assertEqual(j['motion_mask'], 63)
        self.assertFalse(j['hardware_start_verified'])
        self.assertEqual(j['tasks'][-1]['goal_rad'], j['start_rad'])
        self.assertAlmostEqual(j['tasks'][0]['goal_rad'][5]-j['start_rad'][5], math.radians(3))

    def test_sampling_requires_bounded_three_wrist_axes_and_return(self):
        j = make_job(self.capture, self.p, wrist_delta_deg=[4, -4, 4])
        self.assertTrue(j['sample_at_out'])
        self.assertEqual(j['tasks'][-1]['goal_rad'], j['start_rad'])
        for i in (0, 1, 2, 6): self.assertEqual(j['tasks'][0]['goal_rad'][i], j['start_rad'][i])
        for delta in ([7, 0, 0], [0, 0, 0], [0, float('nan'), 0]):
            with self.assertRaises(ValueError): make_job(self.capture, self.p, wrist_delta_deg=delta)

    def test_camera_handshake_brackets_actual_feedback_and_rejects_drift(self):
        import tempfile, json
        for drift in (0., .01):
            before = dict(actual_motor_rad=self.motor[:6])
            after = dict(actual_motor_rad=[self.motor[0]+drift]+self.motor[1:6])
            e = SimpleNamespace(event=Mock(), pump=Mock(), diagnostics=Mock(), settle=Mock(return_value=after), report={'job_sha256':job_digest(self.job)})
            with tempfile.TemporaryDirectory() as directory:
                out = Path(directory)
                (out/'capture_ack.json').write_text(json.dumps(dict(status='CAPTURED', job_sha256=job_digest(self.job))))
                if drift:
                    with self.assertRaises(RuntimeError): capture_while_holding(e, self.job, out, before)
                else:
                    result = capture_while_holding(e, self.job, out, before)
                    self.assertEqual(result['max_bracket_drift_rad'], 0)
                    e.pump.assert_called()

    def test_expanded_review_keeps_static_axes_gripper_and_slow_return(self):
        for delta in ([30,0,0], [0,-30,0], [0,0,30], [12,-12,12]):
            j=make_job(self.capture,self.p,wrist_delta_deg=delta,expanded_wrist=True)
            verify_job(j,self.p)
            self.assertEqual(j['calibration_policy'],'six_axis_wrist_expanded_v1')
            self.assertEqual(j['tasks'][1]['goal_rad'],j['start_rad'])
            for task in j['tasks']:
                self.assertGreaterEqual(sum(s['duration_s'] for s in task['segments']),6)
                for i in (0,1,2,6): self.assertEqual(task['goal_rad'][i],j['start_rad'][i])
        with self.assertRaises(ValueError): make_job(self.capture,self.p,wrist_delta_deg=[30.1,0,0],expanded_wrist=True)
        bad=copy.deepcopy(j);bad['calibration_policy']='six_axis_wrist_sample_v1'
        with self.assertRaises(ValueError): verify_job(bad,self.p)

    def test_lift_opens_elbow_first_and_reverses_before_lowering(self):
        j=make_job(self.capture,self.p,wrist_delta_deg=[-20,15,0],lift_delta_deg=[15,-25])
        verify_job(j,self.p)
        w=j['lift_waypoints_rad']
        self.assertEqual(w[1][1],w[0][1]);self.assertLess(w[1][2],w[0][2])
        self.assertGreater(w[2][1],w[1][1]);self.assertEqual(w[2][2],w[1][2])
        for q in w:self.assertEqual(q[0],w[0][0]);self.assertEqual(q[6],0)
        for delta,lift in [([1,0,0],[15,-25]),([0,0,0],[15,25]),([0,0,0],[30,-25])]:
            with self.assertRaises(ValueError):make_job(self.capture,self.p,wrist_delta_deg=delta,lift_delta_deg=lift)
        bad=copy.deepcopy(j);bad['lift_waypoints_rad'][1][1]+=.01
        with self.assertRaises(ValueError):verify_job(bad,self.p)

    def test_gc_state_restored_even_when_motion_body_raises(self):
        for enabled in (True, False):
            with patch('calibration_motion.gc.isenabled',return_value=enabled), patch('calibration_motion.gc.collect') as collect, patch('calibration_motion.gc.disable') as disable_gc, patch('calibration_motion.gc.enable') as enable_gc:
                with self.assertRaises(RuntimeError):
                    with motion_gc_scope():
                        collect.assert_called_once()
                        disable_gc.assert_called_once()
                        raise RuntimeError('body failure')
                self.assertEqual(enable_gc.call_count, int(enabled))

    def test_host_heartbeat_gap_is_observed_without_changing_lease(self):
        raw=Mock();d=ArmOnlyDevice(raw)
        with patch('calibration_motion.time.monotonic',side_effect=[1.,1.005,1.115]):
            d.heartbeat();d.heartbeat();d.heartbeat()
        self.assertAlmostEqual(d.max_heartbeat_gap_s,.110)
        self.assertEqual(len(d.heartbeat_gaps),1)
        self.assertEqual(raw.heartbeat.call_count,3)
        raw.write.assert_not_called()

    def test_reject_actual_wrist_outside_travel_and_unstable_capture(self):
        for mutation in ('outside', 'unstable', 'missing', 'nonfinite'):
            c = copy.deepcopy(self.capture)
            if mutation == 'outside':
                for x in c['transactions']:
                    if x['motor'] == 6:
                        x['decoded']['position_motor_rad'] = math.radians(-176)
            elif mutation == 'unstable': c['transactions'][0]['decoded']['position_motor_rad'] += .02
            elif mutation == 'missing': c['transactions'].pop()
            else: c['transactions'][0]['decoded']['position_motor_rad'] = float('nan')
            with self.subTest(mutation=mutation), self.assertRaises(ValueError):
                make_job(c, self.p)

    def test_reject_unreviewed_axes_gripper_and_missing_return(self):
        for mutation in ('mask', 'gripper', 'shoulder', 'return', 'limit', 'fast'):
            j = copy.deepcopy(self.job)
            if mutation == 'mask': j['motion_mask'] = 127
            elif mutation == 'gripper': j['tasks'][0]['segments'][0]['coefficients_rad'][6][1] = .1
            elif mutation == 'shoulder': j['tasks'][0]['segments'][0]['coefficients_rad'][1][1] = .1
            elif mutation == 'return': j['tasks'][1]['goal_rad'][0] += .01
            elif mutation == 'limit': j['motor_lower_rad'][0] = -100
            else:
                for s in j['tasks'][0]['segments']: s['duration_s'] = .001
            with self.subTest(mutation=mutation), self.assertRaises(ValueError):
                verify_job(j, self.p)

    def test_all_gripper_controls_blocked_before_underlying_driver(self):
        driver = Mock()
        d = ArmOnlyDevice(driver)
        for packet in (enable(7), disable(7), parameter(7, 0x7016, 1.0), parameter(7, 0x7005, 5, True)):
            with self.assertRaises(ValueError): d.enqueue(*packet)
        driver.enqueue.assert_not_called()
        d.enqueue(*parameter(7, 0x7019))
        driver.enqueue.assert_called_once()
        for mid in AXES:
            d.enqueue(*enable(mid))
            d.enqueue(*disable(mid))

    def test_gripper_bank_mit_and_mask_cannot_bypass_packet_gate(self):
        driver = Mock()
        d = ArmOnlyDevice(driver)
        for addr, value in ((0x5a0, 127), (0x5a0, 64), (0x680, 1), (0x69c, 1), (0xb40, 1), (0xb60, 2)):
            with self.assertRaises(ValueError): d.write(addr, value)
        driver.write.assert_not_called()
        d.write(0x5a0, MASK)
        d.write(0x5a0, 0)
        d.write(0xa00, 0)

    def test_unexpected_parameter_write_rejected(self):
        d = ArmOnlyDevice(Mock())
        with self.assertRaises(ValueError): d.enqueue(*parameter(6, 0x7010, 1.))

    def test_prepare_only_enables_six_axes_and_never_clips_gripper(self):
        import time
        regs = {0x470: 0x60002, 0x828: 0x70008}
        packets = []
        class Device:
            def read(self, a):
                if a >= 0x700 and a < 0x7c0 and (a-0x700) % 32 == 24: return 63
                return regs.get(a, 0)
            def write(self, a, v): regs[a] = v
            def pp_base(self, mid): return 0x700+(mid-1)*32
            def configure(self, mode): regs[0x404] = 7
            def stop(self): regs[0x404] = 0
            def heartbeat(self): pass
            def enqueue(self, ident, payload):
                packets.append((ident, payload))
                if ident >> 24 == 3: regs[0x5a8] = regs.get(0x5a8, 0) | 1 << ((ident & 255)-1)
        motor = self.motor
        class SessionFake(ArmOnlySession):
            def send(self, packet):
                super().send(packet)
                mid=packet[0] & 255
                self.feedback[mid]=dict(time=time.monotonic(), position=motor[mid-1], mode=0)
            def pump(self, seconds): pass
            def require(self, mid, mode=None): return dict(time=time.monotonic(), position=motor[mid-1], mode=mode)
            def read(self, mid, index, integer=False):
                self.send(parameter(mid, index))
                return motor[mid-1]
            def write_verified(self, mid, index, value, integer=False):
                self.send(parameter(mid, index, value, integer))
        report = dict(commands=[], events=[])
        with patch('calibration_motion.ArmOnlySession', SessionFake):
            e = CalibrationExecutor(self.p, ArmOnlyDevice(Device()), report, verify_job(self.job, self.p))
            e.prepare(self.job)
        self.assertEqual(regs[0x5a0], MASK)
        self.assertEqual(regs[0x5a8], MASK)
        self.assertEqual(e.s.target_owner, 'host', 'initial settling must retain host target refresh')
        self.assertEqual(report['gripper_before_motor_rad'], self.motor[6])
        self.assertEqual([ident & 255 for ident, _ in packets if ident >> 24 == 3], list(AXES))
        self.assertEqual([(ident >> 24, payload.hex()) for ident, payload in packets if ident & 255 == 7],
                         [(17, '1970000000000000')])

    def test_disabled_feedback_refresh_waits_for_new_response(self):
        import time
        e=object.__new__(CalibrationExecutor)
        e.s=SimpleNamespace(send=Mock(), feedback={5:dict(time=0.)}, require=Mock())
        calls=[]
        def pump(seconds):
            calls.append(seconds)
            if len(calls)==4:e.s.feedback[5]=dict(time=time.monotonic())
        e.s.pump=pump
        e.refresh_disabled_feedback(5)
        self.assertEqual(len(calls),4)
        e.s.require.assert_called_once_with(5,0)

    def test_live_stream_mask_drift_is_rejected(self):
        for active, stream in ((127, MASK), (MASK, 127)):
            e = object.__new__(CalibrationExecutor)
            e.d = SimpleNamespace(read=lambda a: MASK if a == 0x5a0 else active)
            e.started = True
            with patch('motion_executor.MoveJExecutor.pump', return_value={'mask': stream}), self.assertRaises(RuntimeError):
                e.pump()

    def test_disable_and_bank_cleanup_never_touch_gripper(self):
        driver = Mock()
        driver.pp_base.side_effect = lambda mid: 0x680 if mid == 7 else 0x700+(mid-1)*32
        report = dict(commands=[], frames=[], feedback=[], reset_confirmed=[], stop_errors={})
        s = ArmOnlySession(ArmOnlyDevice(driver), report)
        def pump(seconds):
            import time
            s.feedback.update({mid: dict(time=time.monotonic(), mode=0, fault=0) for mid in AXES})
        s.pump = pump
        s.stop_all()
        self.assertEqual(sorted(report['reset_confirmed']), list(AXES))
        self.assertEqual([c['id'] & 255 for c in report['commands']], list(reversed(AXES)))
        self.assertTrue(all(c['id'] >> 24 == 4 for c in report['commands']))
        self.assertNotIn(0x680, [call.args[0] for call in driver.write.call_args_list])

    def recovery_owner(self):
        records = [dict(plan=dict(index=i)) for i in range(3)]
        return SimpleNamespace(d=SimpleNamespace(read=lambda a: MASK if a == 0x5a8 else 0),
                               s=SimpleNamespace(require=Mock()), started=True,
                               transport=SimpleNamespace(status=lambda: dict(fault=0), prepared=records, prepared_index=1),
                               execute_segments=Mock(), settle=Mock(return_value={'actual_motor_rad': self.motor[:6]}))

    def test_cached_return_checks_measured_endpoint_before_disable(self):
        e, report = self.recovery_owner(), {}
        recover_or_retain(e, self.job, '/unused', report)
        e.execute_segments.assert_called_once_with([r['plan'] for r in e.transport.prepared[1:]])
        e.settle.assert_called_once_with(self.job['start_rad'])
        self.assertTrue(report['safe_to_disable_after_return_or_support'])
        self.assertEqual([c.args[0] for c in e.s.require.call_args_list], list(AXES))

    def test_fault_partial_enable_and_bad_return_retain_instead_of_reset(self):
        for scenario in ('partial_enable', 'fault', 'endpoint', 'before_stream'):
            e, report = self.recovery_owner(), {}
            if scenario == 'partial_enable': e.d.read = lambda a: 7 if a == 0x5a8 else 0
            elif scenario == 'fault': e.transport.status = lambda: dict(fault=1)
            elif scenario == 'endpoint': e.settle.side_effect = RuntimeError('return endpoint error')
            else: e.started = False
            with self.subTest(scenario=scenario), patch('calibration_motion.retain_until_supported') as retain:
                recover_or_retain(e, self.job, '/unused', report)
                retain.assert_called_once_with(e, '/unused', report)
                self.assertIn('cached_return_error', report)


if __name__ == '__main__':
    unittest.main()
