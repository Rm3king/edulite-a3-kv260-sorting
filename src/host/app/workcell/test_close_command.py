"""Run: F=<fpga_competition_2026>/contest; aipc.sh sdk env PYTHONPATH=$PWD:$PWD/workcell:$F/arm_runtime:$F/kv260_can/software python -m unittest workcell.test_close_command  (from integration/)
M6 parts close to 1.66 (nut and screw); every other part keeps 1.60, and 1.66 is a reviewed close command on the board."""
import unittest

from live_planner import close_command   # flat import: run with the planner's environment (pinocchio), see below


class CloseCommand(unittest.TestCase):
    def test_m6_parts_close_to_1_66(self):
        self.assertEqual(close_command('nut', 'nut_m6'), 1.66)
        self.assertEqual(close_command('screw', 'screw_m6'), 1.66)

    def test_other_parts_keep_1_60(self):
        for part, detected in (('nut', 'nut'), ('screw', 'screw'), ('nut', None), ('screw', None), ('nut', 'screw_m6'), ('screw', 'nut_m6')):
            self.assertEqual(close_command(part, detected), 1.6, (part, detected))

    def test_both_commands_are_reviewed_on_the_board(self):
        import continuous_transfer as ct
        for part, detected in (('nut', 'nut_m6'), ('screw', 'screw_m6'), ('nut', 'nut')):
            self.assertIn(close_command(part, detected), ct.JAW_CEILINGS)

    def test_held_m6_screw_is_not_an_empty_closure(self):
        # TaskOutcomeError('Empty closure') fires at actual >= close_rad - .02
        for actual in (1.572, 1.5997):
            self.assertLess(actual, close_command('screw', 'screw_m6') - .02)
        self.assertGreaterEqual(1.66, close_command('screw', 'screw_m6') - .02)   # an empty close reaches the command


if __name__ == '__main__':
    unittest.main()
