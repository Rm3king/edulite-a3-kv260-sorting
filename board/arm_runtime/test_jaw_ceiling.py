"""The jaw ceiling follows the reviewed close command: 1.62 for 1.60, 1.67 only for the M6 nut's 1.66; anything else is refused."""
import unittest
import continuous_transfer as ct


class JawCeiling(unittest.TestCase):
    def test_reviewed_close_commands(self):
        self.assertEqual(ct.jaw_ceiling(dict(close_rad=1.6)), 1.62)
        self.assertEqual(ct.jaw_ceiling(dict(close_rad=1.66)), 1.67)
        self.assertEqual(ct.jaw_ceiling({}), 1.62)
        for close in (1.63, 1.67, 1.678, 1.7):
            with self.assertRaises(ValueError):
                ct.jaw_ceiling(dict(close_rad=close))

    def test_device_default_is_the_old_ceiling(self):
        self.assertEqual(ct.TransferDevice.jaw_ceiling, 1.62)

    def test_initial_upper_without_observed_closed_start(self):
        self.assertEqual(ct.initial_jaw_upper(dict(close_rad=1.66)), 1.67)
        self.assertEqual(ct.initial_jaw_upper(dict(close_rad=1.6)), 1.62)


if __name__ == '__main__':
    unittest.main()
