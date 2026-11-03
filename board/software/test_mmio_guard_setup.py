import unittest
from unittest.mock import Mock
from canctl import CanDevice


class GuardSetupTests(unittest.TestCase):
    def device(self,control):
        d=object.__new__(CanDevice);d.base=123;d.io=Mock();d.read=Mock(return_value=control);return d

    def test_running_guard_writes_rejected_before_any_mmio_write(self):
        for control in (1,4,7,15):
            for address in (0x5a0,0x5a4,0x600,0x66c,0x680,0x69c,0x700,0x7bc):
                d=self.device(control)
                with self.assertRaisesRegex(ValueError,'Stop CAN'):d.write(address,1)
                d.io.mmio_write32.assert_not_called()

    def test_stopped_setup_and_existing_live_heartbeat_allowed(self):
        d=self.device(0);d.write(0x5a0,127);d.io.mmio_write32.assert_called_once_with(123,0x5a0,127)
        d=self.device(7)
        for address in (0x410,0x404,0x820,0xa00):d.write(address,1)
        self.assertEqual(d.io.mmio_write32.call_count,4);d.read.assert_not_called()

    def test_bad_values_and_read_only_fields_rejected_while_stopped(self):
        for address,value in [(0x5a4,0),(0x5a4,1000001),(0x698,0),(0x718,0),(0x7b8,0),(0x69c,3),(0x71c,3)]:
            d=self.device(0)
            with self.assertRaises(ValueError):d.write(address,value)
            d.io.mmio_write32.assert_not_called()

    def test_trajectory_status_and_reserved_writes_never_reach_mmio(self):
        for control in (0,7):
            for address in range(0x800,0x1000,4):
                writable=(address in (0x820,0xb60) or 0x900<=address<=0x9b0
                          or 0xa00<=address<=0xb20 or 0xc00<=address<=0xf6c)
                d=self.device(control)
                if writable:
                    d.write(address,0);d.io.mmio_write32.assert_called_once()
                else:
                    with self.assertRaisesRegex(ValueError,'Read-only'):d.write(address,0)
                    d.io.mmio_write32.assert_not_called();d.read.assert_not_called()


    def test_correction_registers_only_with_the_bundle_marker(self):
        import canctl
        addresses=[0x840,0x848,0x84c]+list(range(0x850,0x868,4))+list(range(0x8a0,0x8b8,4))
        for present in (False,True):
            canctl._CORRECTION[:]=[present]
            try:
                for address in addresses:
                    d=self.device(0)
                    if present:
                        d.write(address,0);d.io.mmio_write32.assert_called_once()
                    else:
                        with self.assertRaisesRegex(ValueError,'Read-only'):d.write(address,0)
                        d.io.mmio_write32.assert_not_called()
            finally:canctl._CORRECTION.clear()


if __name__=='__main__':unittest.main()
