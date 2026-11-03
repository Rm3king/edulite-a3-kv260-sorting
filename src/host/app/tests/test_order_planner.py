import unittest
from workcell.order_planner import ZH, expand, assign_slots, batches, predicted_peak, unsupported_reason
from workcell.tasks import result

OBJECTS = [dict(part_class='screw')] * 3 + [dict(part_class='nut')] * 2


class Order(unittest.TestCase):
    def test_steps_expand_in_order(self):
        task = result('plan', [dict(part_class='screw', quantity=2, region='all', destination='B'), dict(part_class='nut', quantity=1, region='all', destination='B')])
        self.assertEqual(expand(task, OBJECTS), ['screw', 'screw', 'nut'])

    def test_not_enough_parts_is_refused_with_a_reason(self):
        task = result('plan', [dict(part_class='nut', quantity=3, region='all', destination='B')])
        with self.assertRaisesRegex(ValueError, '盘里只有 2'):expand(task, OBJECTS)

    def test_screws_take_the_near_slots_first(self):
        self.assertEqual(assign_slots(['screw', 'screw', 'nut']), [2, 3, 4])

    def test_cool_arm_runs_a_screw_and_a_nut_without_waiting(self):
        plan = batches({2: 30., 3: 30.}, ['screw', 'nut'])
        self.assertEqual(len(plan), 1);self.assertEqual(plan[0]['cool_before_min'], 0)

    def test_three_parts_fit_one_chain_from_a_cool_arm(self):
        plan = batches({2: 30., 3: 30.}, ['screw', 'screw', 'nut'])
        self.assertEqual([b['parts'] for b in plan], [['screw', 'screw', 'nut']]);self.assertEqual(plan[0]['cool_before_min'], 0)

    def test_warm_arm_cools_only_as_much_as_needed(self):
        plan = batches({2: 70., 3: 60.}, ['screw', 'nut'])
        self.assertEqual(len(plan), 1);self.assertGreater(plan[0]['cool_before_min'], 0);self.assertLess(plan[0]['cool_before_min'], 3.1)
        self.assertLessEqual(plan[0]['predicted_peak_c'], 93.)

    def test_long_order_is_split_into_sessions_under_the_limit(self):
        plan = batches({2: 45., 3: 45.}, ['screw'] * 4)
        self.assertGreater(len(plan), 1)
        for batch in plan:self.assertLessEqual(batch['predicted_peak_c'], 93.)
        self.assertEqual(sum(len(b['parts']) for b in plan), 4)

    def test_skip_cooling_never_waits(self):
        plan = batches({2: 50., 3: 45.}, ['screw', 'screw', 'nut'], skip_cooling=True)
        self.assertEqual((len(plan), plan[0]['cool_before_min']), (1, 0))
        self.assertIn('不等降温', plan[0]['reason'])

    def test_skip_cooling_still_refuses_a_hot_start(self):
        with self.assertRaises(ValueError):batches({2: 90., 3: 60.}, ['nut'], skip_cooling=True)

    def test_only_right_to_left_is_supported(self):
        ok = result('plan', [dict(part_class='screw', quantity=2, region='right', destination='B')])
        self.assertIsNone(unsupported_reason(ok))
        back = result('plan', [dict(part_class='nut', quantity='all', region='left', destination='A')])
        self.assertIn('反方向', unsupported_reason(back))
        into_a = result('plan', [dict(part_class='nut', quantity=1, region='all', destination='A')])
        self.assertIsNotNone(unsupported_reason(into_a))

    def test_peak_uses_the_hotter_joint(self):
        self.assertEqual(predicted_peak({2: 40., 3: 60.}, ['screw']), 72.)



class FourClasses(unittest.TestCase):
    def test_m6_parts_expand_and_take_slots_like_the_trial(self):
        task = result('plan', [dict(part_class=c, quantity='all', region='all', destination='B') for c in ('screw', 'nut', 'screw_m6', 'nut_m6')])
        parts = expand(task, [dict(part_class=c) for c in ('screw', 'nut', 'screw_m6', 'nut_m6')])
        self.assertEqual(parts, ['screw', 'nut', 'screw_m6', 'nut_m6'])
        self.assertEqual(assign_slots(parts), [2, 4, 3, 1])                     # as wrist_servo_trial's 4/4 chain of 2026-10-30 22:16
        self.assertEqual((ZH['screw_m6'], ZH['nut']), ('小螺丝', '大螺母'))
        self.assertGreater(predicted_peak({2: 40., 3: 35.}, parts), 40.)


if __name__ == '__main__':unittest.main()