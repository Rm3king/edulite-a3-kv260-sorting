"""Continuous wrist correction on the owner: PL configuration, the three outcomes, config validation. No device is opened."""
import copy, json, tempfile, time, unittest
import numpy as np
from pathlib import Path
from unittest.mock import patch
import wrist_continuous as wc

Q24 = 1 << 24


class FakePL:
    """Registers of the correction channel with the PL's behaviour: slew per tick toward the target (enabled) or zero."""
    def __init__(self):
        self.r = {wc.CTRL: 0, wc.STEP: 0, wc.LEASE: 0, wc.REJECTS: 0}
        for j in range(6):
            self.r[wc.TARGETS + 4 * j] = self.r[wc.APPLIED + 4 * j] = self.r[wc.LIMITS + 4 * j] = 0
        self.running = False

    def write(self, a, v):
        if a in (wc.STEP, wc.LEASE) or wc.LIMITS <= a < wc.LIMITS + 24:
            if self.running:
                self.r[wc.REJECTS] += 1
                return
        if a == wc.CTRL and v & 1 and not (any(self.r[wc.LIMITS + 4 * j] for j in range(6)) and self.r[wc.STEP] and self.r[wc.LEASE]):
            self.r[wc.REJECTS] += 1
            return
        if a == wc.CTRL and not v & 1:
            for j in range(6):
                self.r[wc.TARGETS + 4 * j] = 0
        self.r[a] = v & 0xffffffff

    def read(self, a):
        return self.r[a]

    def tick(self):
        step = self.r[wc.STEP]
        for j in range(6):
            t = wc.signed(self.r[wc.TARGETS + 4 * j]) if self.r[wc.CTRL] & 1 else 0
            a = wc.signed(self.r[wc.APPLIED + 4 * j])
            a = a + max(-step, min(step, t - a))
            self.r[wc.APPLIED + 4 * j] = a & 0xffffffff


class FakeExecutor:
    def __init__(self):
        self.d = FakePL()
        self.p = type('P', (), dict(direction=[1., -1., 1., -1., 1., 1., 1.]))()
        self.events, self.executed, self.settled = [], [], []
        self.transport = type('T', (), {})()
        self.s = type('S', (), dict(require=lambda _s, i, mode: dict(temperature=30.)))()

    def pump(self, seconds=.003):
        self.d.tick()

    def diagnostics(self):
        pass

    def execute_segments(self, segments):
        self.d.running = True
        self.executed.append(segments)

    def settle_arm(self, goal, seconds=.3):
        self.settled.append(goal)
        return dict(goal=goal)

    def event(self, name, **kw):
        self.events.append((name, kw))


def make_job():
    A = [[1.8, 3.1], [-5.1, 3.0], [-7.4, 3.9], [-2.2, 1.2], [.16, .27], [1.8, 3.1]]     # rad per m (mrad per mm), a real hover
    m = .015
    limits = [(abs(a) + abs(b)) * m * 1.15 + 5e-4 for a, b in A]
    cfg = dict(limits_rad=limits, step_rad=.0015, lease_us=200000, converge_frames=3, lost_frames=15, timeout_s=1.0, max_correction_m=m,
               vision=dict(part_class='screw', target_px=[272, 207], J_pixels_per_m=[[4973, -306], [-422, -4540]], A_motor_rad_per_tool_m=A,
                           coarse_center_px=[272, 207], radius_px=220., gain=.4, max_correction_m=m, tolerance_px=6.),
               tube_audit=dict(passed=True, corners=[dict(corner=[s * m, t * m], status='PASS_SAMPLED') for s in (-1, 1) for t in (-1, 1)]))
    tasks = [dict(name=n, segments=[dict(duration_s=.5)], goal_rad=[0.] * 7) for n in
             ('departure', 'screw_0_approach', 'screw_0_close', 'screw_0_lift', 'screw_0_transfer')]
    return dict(tasks=tasks, start_rad=[0.] * 7, recoveries=dict(departure=dict(segments=[dict(duration_s=.4)], goal_rad=[0.] * 7)),
                visual_servo=dict(continuous=cfg))


class Loop:
    """Stands in for the vision process: on 'run' it writes the scripted states into STATE."""
    engine = 'pl'

    def __init__(self, states):
        self.states, self.commands = states, []

    def command(self, mode, cid, params=None):
        self.commands.append(mode)
        if mode == 'run':
            for s in self.states:
                wc.write_json(wc.STATE, dict(s, id=cid))


class Owner(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.state = patch.object(wc, 'STATE', Path(self.tmp.name) / 'state.json')
        self.state.start()
        self.job, self.e = make_job(), FakeExecutor()
        wc.configure.__globals__  # keep flake quiet
        with patch.object(wc, 'channel_present', return_value=True):
            wc.configure(self.e.d, wc.merged_config([self.job]))

    def tearDown(self):
        self.state.stop()
        self.tmp.cleanup()

    def run_part(self, states):
        records = list(range(5))
        records = type('R', (list,), {})(records)
        records.recoveries = {}
        returns = dict(departure=['r'])
        called = {}

        def tail(executor, reply, out, report, continuous=None):
            called.update(reply=reply, continuous=continuous, applied=wc.applied_rad(executor.d))
            return 'COMPLETED_ONLINE_GRASP_MOTION_VERIFY_IMAGES'
        with patch('wrist_servo_motion.execute_grasp_tail', tail):
            report = {}
            status = wc.servo_continuous(0, self.job, 'digest', records, returns, self.e, Loop(states), self.tmp.name, report)
        return status, report, called

    def test_configuration_is_read_back_and_locked(self):
        self.assertEqual(self.e.d.read(wc.STEP), round(.0015 * Q24))
        self.e.d.running = True
        with patch.object(wc, 'channel_present', return_value=True), self.assertRaises(RuntimeError):
            wc.configure(self.e.d, wc.merged_config([self.job]))

    def test_configure_after_the_check_does_no_slow_work(self):
        # after check_before_enable the firmware hash is cached: configure (called with every axis enabled) must not hash
        wc._PRESENT[:] = [True]
        try:
            with patch('hashlib.sha256', side_effect=AssertionError('hashed after enable')):
                t = time.perf_counter()
                wc.configure(FakePL(), wc.merged_config([self.job]))
                self.assertLess(time.perf_counter() - t, .02)
        finally:
            wc._PRESENT.clear()

    def test_configuration_refused_without_the_channel(self):
        with patch.object(wc, 'channel_present', return_value=False), self.assertRaises(RuntimeError):
            wc.configure(FakePL(), wc.merged_config([self.job]))

    def test_converged_runs_the_reviewed_tail_with_the_frozen_offsets(self):
        status, report, called = self.run_part([dict(status='RUNNING', frames=6, lost=0, converged_frames=3, error_px=2.)])
        self.assertEqual(status, 'COMPLETED_ONLINE_GRASP_MOTION_VERIFY_IMAGES')
        self.assertIs(called['continuous'], wc)
        self.assertIs(called['reply']['candidate'], self.job)
        self.assertEqual(called['reply']['records'], [1, 2, 3, 4])          # departure records excluded
        self.assertTrue(self.e.d.read(wc.CTRL) & 1)                          # still enabled: offsets frozen by the PL lease
        self.assertEqual(report['continuous'][0]['status'], 'CONVERGED')

    def test_lost_part_zeroes_the_offsets_then_takes_the_reviewed_return(self):
        for j in range(6):                                                    # offsets already applied before the loss
            self.e.d.write(wc.CTRL, 1)
            self.e.d.write(wc.TARGETS + 4 * j, round(.003 * Q24))
        for _ in range(10):
            self.e.d.tick()
        self.assertTrue(any(wc.applied_rad(self.e.d)))
        status, report, called = self.run_part([dict(status='RUNNING', frames=2, lost=15, converged_frames=0)])
        self.assertEqual(status, 'RETURN_VISION_LOST')
        self.assertFalse(called)
        self.assertFalse(any(wc.applied_rad(self.e.d)))
        self.assertEqual(self.e.executed[-1], self.job['recoveries']['departure']['segments'])

    def test_timeout_returns(self):
        status, report, _ = self.run_part([dict(status='RUNNING', frames=1, lost=0, converged_frames=0)])
        self.assertEqual(status, 'RETURN_CONTINUOUS_TIMEOUT')
        self.assertFalse(self.e.d.read(wc.CTRL) & 1)

    def test_track_only_follows_then_returns_without_grasping(self):
        self.job['visual_servo']['continuous']['track_only_s'] = .3
        t = time.monotonic()
        status, report, called = self.run_part([dict(status='RUNNING', frames=9, lost=0, converged_frames=9, error_px=1.)])
        self.assertEqual(status, 'TRACK_ONLY_DONE')
        self.assertGreaterEqual(time.monotonic() - t, .3)
        self.assertFalse(called)                                              # no grasp
        self.assertFalse(any(wc.applied_rad(self.e.d)))
        self.assertEqual(self.e.executed[-1], self.job['recoveries']['departure']['segments'])

    def test_vision_error_returns(self):
        status, _, _ = self.run_part([dict(status='ERROR', error='camera')])
        self.assertEqual(status, 'RETURN_VISION_ERROR')

    def test_fade_returns_the_offsets_smoothly_then_switches_off(self):
        self.e.d.write(wc.CTRL, 1)
        for j in range(6):
            self.e.d.write(wc.TARGETS + 4 * j, round(.03 * Q24))
        for _ in range(30):
            self.e.d.tick()
        self.e.d.r[wc.STEP] = round(.01 * Q24)                 # a fast PL step: the profile must come from the fade
        fade = wc.Fade(self.e, .2)
        applied, t0 = [], time.monotonic()
        while not fade.done and time.monotonic() - t0 < 1.:
            self.e.pump()
            applied.append(wc.applied_rad(self.e.d)[0])
            time.sleep(.005)
        self.assertTrue(fade.done)
        self.assertFalse(self.e.d.read(wc.CTRL) & 1)
        for _ in range(5):
            self.e.d.tick()
        self.assertFalse(any(wc.applied_rad(self.e.d)))
        steps = np.abs(np.diff(applied))
        # raised cosine: small steps at both ends, the largest in the middle
        self.assertLess(steps[0], .3 * steps.max())
        self.assertLess(steps[len(steps) - 3:].max(), .5 * steps.max())
        self.assertIs(self.e.pump.__func__, FakeExecutor.pump)    # the pump hook is removed

    def test_offset_goal_adds_offsets_in_joint_space(self):
        self.e.d.write(wc.CTRL, 1)
        self.e.d.write(wc.TARGETS + 4, round(.001 * Q24))
        for _ in range(3):
            self.e.d.tick()
        goal = wc.offset_goal(self.e, [0.] * 7)
        self.assertAlmostEqual(goal[1], -.001, places=6)                     # joint 2 direction -1
        self.assertEqual(goal[6], 0.)


class Validation(unittest.TestCase):
    def test_reviewed_config_passes(self):
        wc.validate_config(make_job())

    def bad(self, change):
        job = make_job()
        change(job['visual_servo']['continuous'])
        with self.assertRaises(ValueError):
            wc.validate_config(job)

    def test_refusals(self):
        self.bad(lambda c: c['limits_rad'].__setitem__(2, .3))                                # above the PL ceiling
        self.bad(lambda c: c['limits_rad'].__setitem__(1, .01))                               # does not cover the box
        self.bad(lambda c: c.__setitem__('step_rad', .004))
        self.bad(lambda c: c.__setitem__('tube_audit', dict(passed=False, corners=[])))
        self.bad(lambda c: c['tube_audit']['corners'][2].__setitem__('status', 'REJECTED_COLLISION'))
        self.bad(lambda c: c['vision'].__setitem__('part_class', 'nut_m6'))
        self.bad(lambda c: c.__setitem__('timeout_s', 30))


class Startup(unittest.TestCase):
    """Board start-up (2026-10-31): the vision process starts before the jobs are loaded; idle planning workers are not started."""
    def test_planning_workers_only_where_the_stop_and_look_servo_or_a_replan_needs_them(self):
        import wrist_servo_motion as m
        self.assertTrue(m.needs_planning_worker(dict(visual_servo={})))
        self.assertTrue(m.needs_planning_worker(dict(visual_servo=dict(continuous=None))))
        self.assertFalse(m.needs_planning_worker(dict(visual_servo=dict(continuous=dict(limits_rad=[])))))
        self.assertTrue(m.needs_planning_worker(dict(visual_servo=dict(continuous=dict(limits_rad=[]), rebase=True))))

    def test_vision_loop_can_start_now_and_be_awaited_later(self):
        class P:
            def poll(self): return None
            def wait(self, timeout=None): return 0
            def terminate(self): pass
        with tempfile.TemporaryDirectory() as t, patch.object(wc, 'STATE', Path(t) / 's.json'), patch.object(wc, 'COMMAND', Path(t) / 'c.json'), \
                patch.object(wc, 'BUNDLE', Path(t)), patch('subprocess.Popen', return_value=P()) as popen:
            (Path(t) / 'wrist_vision_loop.py').write_text('')
            loop = wc.VisionLoop(None, wait=False, log=Path(t) / 'loop.log')          # returns at once, nothing is awaited
            self.assertEqual(popen.call_count, 1)
            self.assertIsNone(loop.engine)
            with self.assertRaises(RuntimeError):
                loop.wait_ready(timeout=.2)                                           # not READY yet: refuses, closes
            wc.write_json(wc.STATE, dict(status='READY', engine='pl'))
            loop2 = wc.VisionLoop(None, wait=False, log=Path(t) / 'loop2.log')
            wc.write_json(wc.STATE, dict(status='READY', engine='pl'))
            loop2.wait_ready(timeout=1.)
            self.assertEqual(loop2.engine, 'pl')

    def test_pre_started_vision_is_closed_when_a_job_fails_to_load(self):
        import wrist_servo_motion as m
        closed = []
        fake = type('V', (), dict(process=type('P', (), dict(poll=lambda s: None))(), close=lambda s: closed.append(1)))()
        args = type('A', (), dict(prestart_vision=True, execute_attended=True, out=Path('/tmp/x/execution')))()
        with patch.object(m, '_run', side_effect=ValueError('bad job')), patch.object(wc, 'VisionLoop', return_value=fake), \
                patch('argparse.ArgumentParser.parse_args', return_value=args):
            args.job = args.profile = args.out
            with self.assertRaises(ValueError):
                m.main()
        self.assertEqual(closed, [1])


if __name__ == '__main__':
    unittest.main()
