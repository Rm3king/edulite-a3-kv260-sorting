"""Rule parser: a sentence that names exactly one tray (in its last clause) sends every quantity clause there; nothing else changes."""
import unittest

from .order_language import fast_parse


def steps(text):
    task = fast_parse(text)
    return None if task is None else [(s['part_class'], s['quantity'], s['destination']) for s in task['steps']]


class OrderRules(unittest.TestCase):
    def test_tray_in_last_clause_applies_to_earlier_clauses(self):
        # 2026-11-01: "...，一颗大螺母和一颗小螺母放到左盘" went to the model, which merged the two nuts: "还不够明确"
        self.assertEqual(steps('把一颗大螺丝 ，一颗大螺母和一颗小螺母放到左盘'), [('screw', 1, 'B'), ('nut', 1, 'B'), ('nut_m6', 1, 'B')])
        self.assertEqual(steps('把一颗大螺丝和一颗大螺母，一颗小螺丝放到左边托盘'), [('screw', 1, 'B'), ('nut', 1, 'B'), ('screw_m6', 1, 'B')])

    def test_two_trays_keep_each_clause_own_tray(self):
        self.assertEqual(steps('把一颗大螺丝放到右盘，一颗大螺母和一颗小螺母放到左盘'), [('screw', 1, 'A'), ('nut', 1, 'B'), ('nut_m6', 1, 'B')])
        self.assertIsNone(steps('把一颗大螺丝，一颗大螺母放到左盘，一颗小螺丝放到右盘'))

    def test_source_tray_is_not_a_destination(self):
        self.assertEqual(steps('把左盘的小螺丝放回右盘'), [('screw_m6', 'all', 'A')])
        self.assertEqual(steps('把左边盘子的螺母放回右边'), [('nut', 'all', 'A')])

    def test_no_tray_or_no_quantity_still_goes_to_the_model(self):
        self.assertIsNone(steps('把一颗大螺丝，一颗大螺母和一颗小螺母'))
        self.assertIsNone(steps('把大螺丝放到左盘'))

    def test_unchanged_basics(self):
        self.assertEqual(steps('把两颗大螺母放到左盘'), [('nut', 2, 'B')])
        self.assertEqual(len(steps('把所有零件放到左盘')), 4)


if __name__ == '__main__':
    unittest.main()
