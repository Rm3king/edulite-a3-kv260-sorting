"""No physical access: commissioning restores even a failed acknowledged write."""
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch
import csp_tuning


class Device:
    def __init__(self, version=0x60003):
        self.regs = {0x470: version}
        self.writes = []
        self.closed = False

    def read(self, address): return self.regs.get(address, 0)
    def status(self): return {'control': self.read(0x404)}
    def write(self, address, value):
        self.regs[address] = value
        self.writes.append((address, value))
    def configure(self, mode): self.regs[0x404] = 7
    def stop(self): self.regs[0x404] = 0
    def close(self): self.closed = True
    def pp_base(self, mid): return 0x680 if mid == 7 else 0x700+32*(mid-1)


class Session:
    def __init__(self, fail=False):
        self.values = {(2, 0x7021): .05, (3, 0x7021): .05}
        self.fail = fail
        self.writes = []

    def read(self, mid, index, integer=False):
        if index == 0x7019: return 0.
        if index == 0x7005: return 5
        return self.values[mid, index]

    def write_verified(self, mid, index, value):
        self.values[mid, index] = value
        self.writes.append((mid, index, value))
        if self.fail and value == .1:
            self.fail = False
            raise RuntimeError('injected readback failure after motor accepted write')


class TuningTests(unittest.TestCase):
    def trial(self, *, fail=False, leave=False, version=0x60003):
        d, s = Device(version), Session(fail)
        with tempfile.TemporaryDirectory() as root:
            out = Path(root)/'test'
            with patch.object(csp_tuning, 'CanDevice', return_value=d), \
                 patch.object(csp_tuning, 'Session', return_value=s), \
                 patch.object(csp_tuning, 'refresh', return_value={'mode': 0, 'temperature': 30}):
                report = csp_tuning.commission('filter', .1, out, leave)
            self.assertTrue((out/'result.json').exists())
        self.assertTrue(d.closed)
        self.assertEqual(d.read(0x404), 0)
        self.assertEqual(d.read(0x5a0), 0)
        return report, s, d

    def test_default_restores_both_originals(self):
        r, s, _ = self.trial()
        self.assertEqual(r['status'], 'PASS_WRITE_READ_RESTORE_DISABLED')
        self.assertEqual(list(s.values.values()), [.05, .05])
        self.assertEqual(r['enable_commands'], 0)

    def test_readback_failure_also_restores_written_axis(self):
        r, s, _ = self.trial(fail=True, leave=True)
        self.assertEqual(r['status'], 'FAIL')
        self.assertEqual(list(s.values.values()), [.05, .05])
        self.assertEqual(s.writes[-1], (2, 0x7021, .05))

    def test_leaving_requires_explicit_opt_in_and_success(self):
        r, s, _ = self.trial(leave=True)
        self.assertEqual(r['status'], 'PASS_CANDIDATE_READBACK')
        self.assertEqual(list(s.values.values()), [.1, .1])
        self.assertEqual(r['originals'], {'2': .05, '3': .05})

    def test_old_firmware_rejected_before_any_write(self):
        r, s, d = self.trial(version=0x60002)
        self.assertEqual(r['status'], 'FAIL')
        self.assertEqual(s.writes, [])
        self.assertEqual(d.writes, [])

    def test_bounds_reject_invalid_gain(self):
        for name, value in [('filter', .16), ('filter', -.1), ('spd_kp', 7),
                            ('spd_ki', .03), ('j3_loc_kp', 31), ('filter', float('nan'))]:
            with self.assertRaises(ValueError): csp_tuning.check_candidate(name, value)


if __name__ == '__main__': unittest.main()
