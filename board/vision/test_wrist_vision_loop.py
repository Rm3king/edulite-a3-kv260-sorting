"""wrist_vision_loop.run against fake PL registers, camera and locator: the correction it writes, and observe-only."""
import json, struct, sys, tempfile, threading, time, types, unittest
from pathlib import Path
from unittest.mock import patch
import numpy as np

for name in ('vision_pre', 'wrist_locator'):          # the PL engine and board locator are not needed here
    sys.modules.setdefault(name, types.ModuleType(name))
import wrist_vision_loop as w


class Camera:
    def frame(self):
        time.sleep(.005)
        return np.zeros((400, 640), np.uint8), 1, time.monotonic_ns()


def params(**kw):
    p = dict(part_class='nut', part_size='m12', target_px=[300., 200.], J_pixels_per_m=[[5000., 0.], [0., 5000.]],
             A_motor_rad_per_tool_m=[[1., 0.], [0., 1.], [1., 1.], [0., 0.], [0., 0.], [0., 0.]],
             coarse_center_px=[310., 200.], radius_px=160., gain=.5, max_correction_m=.015, tolerance_px=4.)
    p.update(kw)
    return p


class Loop(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.patches = [patch.object(w, 'COMMAND', Path(self.tmp.name) / 'c.json'), patch.object(w, 'STATE', Path(self.tmp.name) / 's.json')]
        for p in self.patches:
            p.start()

    def tearDown(self):
        for p in self.patches:
            p.stop()
        self.tmp.cleanup()

    def drive(self, correction, centres, seconds=.4, **kw):
        """Run the loop for `seconds` with the locator returning `centres` in turn (the last one repeats)."""
        seq = iter(centres)
        last = [None]

        def observe(_self, gray, shift=None):
            last[0] = next(seq, last[0])
            return dict(center_px=list(last[0]))
        command = dict(id='t', mode='run', params=params(**kw))
        w.write_json(w.COMMAND, command)
        with patch.object(w.Tracker, 'observe', observe):
            t = threading.Thread(target=w.run, args=(command, None, Camera(), correction))
            t.start()
            time.sleep(seconds)
            w.write_json(w.COMMAND, dict(id='t', mode='freeze'))
            t.join(2)
        self.assertFalse(t.is_alive())
        return json.loads(w.STATE.read_text())

    def test_enabled_channel_receives_the_integrated_correction(self):
        regs = bytearray(0x1000)
        struct.pack_into('<I', regs, w.CTRL, 1)
        state = self.drive(w.Correction(regs), [[310., 200.]])       # 10 px right of the target, never moves (no arm)
        targets = [struct.unpack_from('<i', regs, w.TARGETS + 4 * j)[0] / w.Q24 for j in range(6)]
        self.assertEqual(state['status'], 'FROZEN')
        self.assertGreater(state['frames'], 3)
        # the error pushes tool x negative until the 15 mm box clamps it; the motor offsets are A @ cumulative
        self.assertAlmostEqual(state['cumulative_tool_xy_m'][0], -.015, places=6)
        np.testing.assert_allclose(targets, np.asarray(params()['A_motor_rad_per_tool_m']) @ state['cumulative_tool_xy_m'], atol=1e-6)
        self.assertEqual(state['applied_rad'], [0.] * 6)               # the PL slews; nothing applied in a fake bank

    def test_converges_when_the_part_reaches_the_target(self):
        regs = bytearray(0x1000)
        struct.pack_into('<I', regs, w.CTRL, 1)
        state = self.drive(w.Correction(regs), [[310., 200.], [305., 200.], [301., 200.]])
        self.assertGreaterEqual(state['converged_frames'], 3)
        self.assertLessEqual(state['error_px'], 4.)

    def test_disabled_channel_is_never_written(self):
        regs = bytearray(0x1000)
        self.drive(w.Correction(regs), [[310., 200.]])
        self.assertEqual(bytes(regs[w.TARGETS:w.TARGETS + 24]), bytes(24))

    def test_observe_only_runs_without_registers(self):
        state = self.drive(w.ObserveOnly(), [[310., 200.]])
        self.assertGreater(state['frames'], 3)
        self.assertIsNone(state['applied_rad'])



class SimPL:
    """PL correction bank in time: applied slews toward the targets (step per 5 ms tick) while enabled."""
    def __init__(self, step_rad):
        self.regs = bytearray(0x1000)
        struct.pack_into('<I', self.regs, w.CTRL, 1)
        self.step, self.applied, self.log, self.stop = step_rad, np.zeros(6), [], threading.Event()
        threading.Thread(target=self.tick, daemon=True).start()

    def tick(self):
        while not self.stop.wait(.005):
            t = np.array([struct.unpack_from('<i', self.regs, w.TARGETS + 4 * j)[0] / w.Q24 for j in range(6)])
            self.applied = self.applied + np.clip(t - self.applied, -self.step, self.step)
            for j in range(6):
                struct.pack_into('<i', self.regs, w.APPLIED + 4 * j, int(round(self.applied[j] * w.Q24)))
            self.log.append((time.monotonic(), self.applied.copy()))

    def at(self, t):
        older = [a for s, a in self.log if s <= t]
        return older[-1] if older else np.zeros(6)


class Plant(unittest.TestCase):
    """run3-like timing: 0.19 s per frame, the image shows the arm 0.1 s earlier, 30 mm/s slew. The part needs +11 mm in x."""
    A = np.array([[1., 0.], [0., 1.], [1., 1.], [0., 0.], [0., 0.], [0., 0.]])
    J = np.array([[-4500., 0.], [0., -4500.]])            # image moves opposite to the tool, ~4.5 px/mm
    NEED = np.array([.011, 0.])

    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.patches = [patch.object(w, 'COMMAND', Path(self.tmp.name) / 'c.json'), patch.object(w, 'STATE', Path(self.tmp.name) / 's.json')]
        for p in self.patches:
            p.start()

    def tearDown(self):
        for p in self.patches:
            p.stop()
        self.tmp.cleanup()

    def fly(self, control, gain, seconds=3., **extra):
        pl = SimPL(.03 * .005)                           # 30 mm/s; A is identity-like here (rad == m)
        target = np.array([300., 200.])
        pinv = np.linalg.pinv(self.A)

        class Cam:
            def frame(_self):
                time.sleep(.19)
                return np.zeros((400, 640), np.uint8), 1, time.monotonic_ns()

        def observe(_self, gray, shift=None):
            had = pinv @ pl.at(time.monotonic() - .1 - getattr(self, 'extra_lag', 0.))
            return dict(center_px=list(target + self.J @ (had - self.NEED)))   # centred once the arm has NEED
        command = dict(id='p', mode='run', params=dict(params(), control=control, gain=gain, J_pixels_per_m=self.J.tolist(),
                                                         A_motor_rad_per_tool_m=self.A.tolist(), capture_lag_s=.08, **extra))
        w.write_json(w.COMMAND, command)
        with patch.object(w.Tracker, 'observe', observe):
            t = threading.Thread(target=w.run, args=(command, None, Cam(), w.Correction(pl.regs)))
            t.start()
            time.sleep(seconds)
            w.write_json(w.COMMAND, dict(id='p', mode='freeze'))
            t.join(2)
        pl.stop.set()
        x = np.array([(pinv @ a)[0] for _, a in pl.log])
        # stop-go: 50 ms windows with the arm (nearly) still while more than 0.5 mm short, between its start and first arrival
        ts = np.array([s for s, _ in pl.log]); ts -= ts[0]
        bins = np.arange(0., ts[-1], .05); xs = np.interp(bins, ts, x); v = np.abs(np.diff(xs)) / .05
        short = np.abs(self.NEED[0] - xs[1:]) > .0005
        moving = np.nonzero(v > .001)[0]
        arrive = np.nonzero(~short)[0]
        window = slice(moving[0] if len(moving) else 0, arrive[0] if len(arrive) else len(v))
        stops = int(np.sum(v[window] < .001))
        state = json.loads(w.STATE.read_text())
        first = next((r['t'] for r in state['trace'] if r['err'] <= 4.), None)
        return dict(stops_50ms=stops, overshoot_mm=(x.max() - self.NEED[0]) * 1e3, final_mm=x[-1] * 1e3, converged=state['converged_frames'], first_within_4px_s=first)

    def test_position_law_does_not_overshoot_where_the_integral_law_does(self):
        self.extra_lag = .2                                     # real lag 0.3 s, as run3 may have had
        old = self.fly('integral', .4)
        self.extra_lag = .05                                    # real 0.15 s; the loop assumes 0.08 s (run5 fit: 0.07 s after receipt)
        new = self.fly('position', .5)
        print('\nintegral gain .4:', old, '\nposition gain .5:', new)
        self.assertGreater(old['overshoot_mm'], 1.)           # the run3 behaviour, reproduced
        self.assertLess(new['overshoot_mm'], .5)
        self.assertAlmostEqual(new['final_mm'], 11., delta=.5)
        self.assertGreaterEqual(new['converged'], 3)


    def test_smoothing_removes_the_stop_go_staircase(self):
        self.extra_lag = .05
        steps = self.fly('position', .5)
        smooth = self.fly('position', .5, smooth_s=.12)
        print('\nno smoothing:', steps, '\nsmooth .12 s:', smooth)
        # the unsmoothed baseline stops at a goal on the AI PC (1 window) but not always: thread timing; not asserted
        self.assertEqual(smooth['stops_50ms'], 0)
        self.assertLess(smooth['overshoot_mm'], .5)
        self.assertGreaterEqual(smooth['converged'], 3)


class Jumps(unittest.TestCase):
    def tracker(self, centres):
        t = w.Tracker(None, 'screw', 'm12', [235., 239.], 220.)
        seq = iter(centres)
        t.locate = lambda gray: dict(center_px=list(next(seq)))
        return t

    def test_the_arms_own_motion_is_not_a_jump(self):
        # run5: the part moved 17 px in the image between two frames because the correction moved 3.5 mm
        t = self.tracker([[232., 239.7], [232., 239.7], [248.7, 237.2]])
        with self.assertRaises(ValueError):
            t.observe(None, [0., 0.])                          # first sighting waits for a second
        t.observe(None, [0., 0.])
        o = t.observe(None, [16.7, -2.5])                      # J @ offset at capture explains the shift
        self.assertEqual(o['center_px'], [248.7, 237.2])

    def test_a_real_jump_is_still_rejected(self):
        t = self.tracker([[232., 239.7], [232., 239.7], [248.7, 237.2]])
        with self.assertRaises(ValueError):
            t.observe(None, [0., 0.])
        t.observe(None, [0., 0.])
        with self.assertRaises(ValueError):
            t.observe(None, [0., 0.])                          # same image shift without any correction: a jump



class NutSearchRadius(unittest.TestCase):
    """The 2026-11-01 failed nut was visible just outside the locator's 65 px default."""
    def frames(self):
        import cv2
        root = Path(__file__).parent / 'fixtures' / 'm12_nut_radius'
        return [cv2.imread(str(root / ('lost_%d.png' % i)), cv2.IMREAD_GRAYSCALE) for i in range(4)]

    def test_configured_radius_recovers_tracking_after_two_frame_agreement(self):
        frames = self.frames()
        tracker = w.Tracker(None, 'nut', 'm12', [258., 215.], 160.)
        with self.assertRaisesRegex(ValueError, 'second frame'):
            tracker.observe(frames[0])
        for gray in frames[1:]:
            observation = tracker.observe(gray)
            np.testing.assert_allclose(observation['center_px'], [314.5, 179.5], atol=2.)

    def test_smaller_requested_radius_still_rejects_the_outside_rim(self):
        tracker = w.Tracker(None, 'nut', 'm12', [258., 215.], 65.)
        for gray in self.frames():
            with self.assertRaisesRegex(ValueError, 'No unique near-view nut rim'):
                tracker.observe(gray)


class Error(unittest.TestCase):
    def test_along_axis_weight(self):
        o = dict(center_px=[270.5, 221.6], axis_deg=90.)              # run8 M6 screw: 14 px along a vertical axis
        weak = w.pixel_error(o, [271.9, 207.3], 'screw')
        full = w.pixel_error(o, [271.9, 207.3], 'screw', 1.)
        self.assertLess(np.linalg.norm(weak), 5.)
        self.assertAlmostEqual(np.linalg.norm(full), np.hypot(1.4, 14.3), places=6)

if __name__ == '__main__':
    unittest.main()
