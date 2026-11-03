"""No hardware: ABI boundaries, recoverable rejection, bounded volatile gains."""
import struct
import unittest
from canctl import CanDevice, CommandRejectedError
from arm_commission import parameter


class Mailbox:
    def __init__(self, version=0x60003, status=0, sequence=0):
        self.regs = {0x470: version, 0x474: 0, 0x478: sequence}
        self.status = status
        self.writes = []

    def mmio_read32(self, base, offset):
        return self.regs.get(offset, 0)

    def mmio_write32(self, base, offset, value):
        self.writes.append((offset, value))
        if offset == 0x45c:
            self.regs[0x478] = (self.regs[0x478] + 1) & 0xffffffff
            self.regs[0x474] = self.status


class SafeSubmitTests(unittest.TestCase):
    def device(self, **kwargs):
        d = object.__new__(CanDevice)
        d.base = 0
        d.io = Mailbox(**kwargs)
        return d

    def test_legacy_call_redirected_and_sequence_wrap(self):
        d = self.device(sequence=0xffffffff)
        d.write(0x458, 1)
        self.assertEqual(d.io.writes, [(0x45c, 1)])
        self.assertEqual(d.read(0x478), 0)

    def test_rejection_is_not_success(self):
        for status in (1, 2, 3):
            d = self.device(status=status)
            with self.assertRaises(CommandRejectedError):
                d.write(0x458, 1)
            self.assertEqual(d.io.writes, [(0x45c, 1)])

    def test_old_abi_never_accesses_new_registers(self):
        for version in (0x60001, 0x60002):
            d = self.device(version=version)
            with self.assertRaises(ValueError):
                d.write(0x45c, 1)
            for offset in (0x474, 0x478):
                with self.assertRaises(ValueError):
                    d.read(offset)
            self.assertEqual(d.io.writes, [])
            d.write(0x458, 1)
            self.assertEqual(d.io.writes, [(0x458, 1)])

    def test_missing_ack_is_error(self):
        d = self.device()
        d.io.mmio_write32 = lambda *args: None
        with self.assertRaisesRegex(RuntimeError, 'sequence'):
            d.write(0x458, 1)

    def test_gain_contract(self):
        for mid in (2, 3):
            b = CanDevice.pp_base(mid)
            fb = 0x500 + (mid-1)*16
            regs = {0x470: 0x60003, 0x404: 7, 0x5a0: 127, 0x5a8: 0,
                    0x49c: 127, 0x408: 1000, 0x5a4: 10000,
                    b: 1, b+28: 1, fb: 0x020000fd+(mid<<8), fb+12: 500}
            d = object.__new__(CanDevice)
            d.read = lambda a: regs.get(a, 0)
            for index, values, bad in (
                (0x701f, (4., 5., 6.), (3.99, 6.01)),
                (0x7020, (.01, .015, .02), (.009, .021)),
                (0x7021, (.05, .1, .15), (.049, .151))):
                for value in values:
                    d.check_motion_guard(*parameter(mid, index, value))
                for value in (*bad, -1., float('nan'), float('inf')):
                    with self.assertRaises(ValueError):
                        d.check_motion_guard(*parameter(mid, index, value))
                for addr, value in ((0x470, 0x60002), (0x470, 0x60001),
                                    (0x5a8, 1), (0x49c, 0), (b+28, 0),
                                    (fb, 0x028000fd+(mid<<8)), (0x408, 20000)):
                    old = regs[addr]
                    regs[addr] = value
                    with self.assertRaises(ValueError):
                        d.check_motion_guard(*parameter(mid, index, values[1]))
                    regs[addr] = old
                for other in (1, 4, 5, 6, 7):
                    bb = CanDevice.pp_base(other)
                    regs[bb] = 1
                    regs[bb+28] = 1
                    with self.assertRaises(ValueError):
                        d.check_motion_guard(*parameter(other, index, values[1]))


if __name__ == '__main__':
    unittest.main()
