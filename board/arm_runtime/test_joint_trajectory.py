#!/usr/bin/python3
import copy
import math
import random
import unittest
from joint_trajectory import JointPlanner, MotionContract, evaluate
from arm_service import ArmService


class TrajectoryTests(unittest.TestCase):
    def setUp(self):
        self.p = JointPlanner()
        self.q = [0., 1., -1., 0., 0., 0., .5]

    def test_random_synchronized_bounds_and_endpoints(self):
        rng = random.Random(260)
        for _ in range(100):
            q0 = [rng.uniform(l, h) for l, h in zip(self.p.lower, self.p.upper)]
            q1 = [min(h, max(l, q+rng.uniform(-.2, .2))) for q, l, h in zip(q0, self.p.lower, self.p.upper)]
            plan = self.p.movej(q0, q1)
            self.assertFalse(plan['executable'])
            for frac in (0., .5, 1.):
                r = evaluate(plan, plan['duration_s']*frac)
                for i in range(7):
                    self.assertLessEqual(abs(r['v'][i]), self.p.velocity[i]+1e-12)
                    self.assertLessEqual(abs(r['a'][i]), self.p.acceleration[i]+1e-12)
                    self.assertGreaterEqual(r['q'][i], min(q0[i], q1[i])-1e-12)
                    self.assertLessEqual(r['q'][i], max(q0[i], q1[i])+1e-12)
            for a, b in zip(evaluate(plan, plan['duration_s'])['q'], q1):
                self.assertAlmostEqual(a, b, places=12)
            self.assertEqual(evaluate(plan, 0)['v'], [0.]*7)
            for v in evaluate(plan, plan['duration_s'])['v']:
                self.assertAlmostEqual(v, 0., places=12)

    def test_directions_gripper_and_hls_layout(self):
        q = [0.1, .2, -.3, .4, -.5, .6, 1.67]
        p = self.p.movej(q, q)
        expected = [-.1, .2, .3, .4, .5, .6, 1.67]
        self.assertEqual(self.p.to_motor(q), expected)
        for row, x, words in zip(p['motor_coefficients_rad'], expected, p['hls_coefficients_q8_24']):
            self.assertEqual(row[0], x)
            self.assertLessEqual(abs(words[0]/2**24-x), .5/2**24)
        self.assertFalse(p['tcp_straight_line'])

    def test_invalid_inputs_and_no_clipping(self):
        for value in [True, '0', None, math.nan, math.inf]:
            q = self.q[:]; q[0] = value
            with self.assertRaises(ValueError): self.p.movej(q, self.q)
        for q in [self.q[:6], self.q+[0], [0, -.0001, -1, 0, 0, 0, .5]]:
            with self.assertRaises(ValueError): self.p.movej(q, self.q)
        for scale in [0, -1, 1.01, True, math.nan]:
            with self.assertRaises(ValueError): self.p.movej(self.q, self.q, scale)
        with self.assertRaises(ValueError): self.p.movej(self.q, self.q, minimum_duration_s=601)

    def test_braking_continuity_acceleration_and_stop_position(self):
        v = [.04, .008, -.009, .03, -.05, 0, -.02]
        p = self.p.brake(self.q, v)
        start, end = evaluate(p, 0), evaluate(p, p['duration_s'])
        self.assertEqual(start['q'], self.q)
        for i in range(7):
            self.assertAlmostEqual(start['v'][i], v[i])
            self.assertAlmostEqual(end['v'][i], 0)
            self.assertAlmostEqual(end['q'][i], self.q[i]+v[i]*p['duration_s']/2)
            self.assertLessEqual(abs(start['a'][i]), self.p.acceleration[i]+1e-12)
        self.assertFalse(p['physically_verified'])

    def test_brake_rejects_outward_limit_and_raw_speed_spike(self):
        q = self.q[:]; q[0] = self.p.upper[0]-.0001
        with self.assertRaises(ValueError): self.p.brake(q, [.06, 0, 0, 0, 0, 0, 0])
        with self.assertRaises(ValueError): self.p.brake(self.q, [0, .21, 0, 0, 0, 0, 0])

    def test_pause_holds_after_braking_does_not_resume_old_goal(self):
        s = MotionContract(self.p); s.enable_simulated(self.q)
        goal = self.q[:]; goal[0] = .2
        m = s.movej(goal); s.advance(m['duration_s']/2)
        before = evaluate(s.plan, s.elapsed)
        with self.assertRaises(ValueError): s.hold()
        b = s.pause()
        self.assertEqual(s.state, 'BRAKING')
        after = evaluate(s.plan, 0)
        for a, z in zip(before['q']+before['v'], after['q']+after['v']):
            self.assertAlmostEqual(a, z, places=12)
        with self.assertRaises(ValueError): s.movej(goal)
        s.advance(b['duration_s']); self.assertEqual(s.state, 'HOLDING')
        self.assertLess(s.hold()[0], goal[0])
        self.assertIsNone(s.plan)
        q = s.hold(); s.advance(10); self.assertEqual(s.hold(), q)

    def test_fault_latches_and_never_automatically_reenables(self):
        s = MotionContract(self.p); s.enable_simulated(self.q)
        s.movej([.1]+self.q[1:]); s.fault()
        self.assertIsNone(s.plan)
        with self.assertRaises(ValueError): s.enable_simulated(self.q)
        with self.assertRaises(ValueError): s.movej(self.q)

    def test_sequence_replay_and_physical_commands_closed(self):
        s = ArmService()
        self.assertFalse(s.handle({'sequence': 1, 'command': 'Status'})['result']['hardware_motion_ready'])
        with self.assertRaises(ValueError): s.handle({'sequence': 1, 'command': 'SimEnable', 'position_rad': self.q})
        for seq, command in enumerate(['Enable', 'MoveJ', 'MoveL', 'Pause', 'Hold', 'Home', 'Disable'], 2):
            with self.assertRaisesRegex(ValueError, 'not commissioned'):
                s.handle({'sequence': seq, 'command': command})
        self.assertEqual(s.session.state, 'DISABLED')


if __name__ == '__main__': unittest.main()
