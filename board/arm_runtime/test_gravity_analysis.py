import unittest
import numpy as np
from analyze_gravity import band_rms


class RippleTests(unittest.TestCase):
    def test_known_in_band_signal_with_large_slow_motion(self):
        time=np.arange(0.,30.,.005)
        signal=3*np.sin(2*np.pi*.1*time)+.02*np.sin(2*np.pi*8*time)
        self.assertAlmostEqual(band_rms(signal),.02/np.sqrt(2),delta=1e-6)
        self.assertLess(band_rms(3*np.sin(2*np.pi*.1*time)),1e-6)
        self.assertAlmostEqual(band_rms(2*signal),2*band_rms(signal),places=10)


if __name__=='__main__':unittest.main()
