"""No hardware: C3 cruise references retain existing quantized bounds."""
import unittest
from pathlib import Path
import numpy as np
from joint_trajectory import JointPlanner
from septic_trajectory import movej_c3, movej_cruise_c3, continuity_metrics, segment_floor_violations
from septic_job import SCHEMA, verify_septic_job


class CruiseTests(unittest.TestCase):
    def setUp(self):
        self.p = JointPlanner(Path(__file__).with_name('profile_grasp_candidate.json'))

    def check_path(self, start, goal, minimum=.1):
        plans = movej_cruise_c3(self.p, start, goal, minimum)
        job = dict(schema=SCHEMA, profile_id=self.p.profile_id, start_rad=start,
                   tasks=[dict(segments=plans, goal_rad=goal)])
        self.assertTrue(verify_septic_job(job, self.p))
        metrics = continuity_metrics(plans, self.p)
        self.assertLess(max(metrics['maximum_boundary_jumps']), 1e-7)
        self.assertLess(max(metrics['rest_endpoint_v_a_j']), 1e-7)
        self.assertGreaterEqual(sum(s['duration_s'] for s in plans)+1e-9, minimum)
        return plans

    def test_random_signed_moves_and_joint_limits(self):
        rng = np.random.default_rng(26025)
        for _ in range(30):
            start = rng.uniform(self.p.lower, self.p.upper).tolist()
            goal = rng.uniform(self.p.lower, self.p.upper).tolist()
            self.check_path(start, goal)

    def test_short_zero_and_minimum_duration(self):
        q = [0., .3, -.3, 0., 0., 0., 1.2]
        self.check_path(q, q, .5)
        for d in (1e-7, 1e-4, .002, -.002):
            goal=q.copy();goal[0]+=d
            self.check_path(q, goal, .5)

    def test_no_short_interior_segment(self):
        """A 15 ms cruise segment between two 250 ms ramps starved the PL queue (2026-10-28): none may be planned."""
        rng = np.random.default_rng(1)
        q = [0., 1.2, -1.5, 0., 0., 0., 1.12]
        for scale in np.logspace(-3.2, -0.3, 60):
            for axes in ([0], [1, 2], [0, 1, 2, 3], [3, 4, 5]):
                goal = list(q)
                for a in axes:
                    goal[a] += float(scale * rng.choice([-1, 1]) * rng.uniform(.5, 1))
                plans = self.check_path(q, goal)
                self.assertEqual(segment_floor_violations(plans), [], (scale, axes, [x['duration_s'] for x in plans]))

    def test_floor_check_flags_a_sliver(self):
        self.assertEqual(segment_floor_violations([dict(duration_s=x) for x in (.25, .015, .25)]), [1])
        self.assertEqual(segment_floor_violations([dict(duration_s=x) for x in (.015, .25)]), [])

    def test_jaw_only_faster_without_arm_motion(self):
        q = [0., .3, -.3, 0., 0., 0., 1.12]
        goal = q[:6]+[1.6]
        plans = self.check_path(q, goal)
        self.assertLess(sum(s['duration_s'] for s in plans),
                        sum(s['duration_s'] for s in movej_c3(self.p, q, goal)))
        for plan in plans:
            np.testing.assert_allclose(np.array(plan['coefficients_rad'])[:6, 1:], 0)


if __name__ == '__main__':
    unittest.main()
