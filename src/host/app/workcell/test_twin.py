"""The numpy forward kinematics of the twin equals the SDK's pinocchio forward kinematics (when pinocchio is available)."""
import unittest
from pathlib import Path

import numpy as np

from .twin import Twin, find_urdf

APP = Path(__file__).resolve().parents[2]


class TwinTests(unittest.TestCase):
    def setUp(self):
        self.twin = Twin(find_urdf(APP))

    def test_chain_has_six_revolute_joints_and_a_tool(self):
        self.assertEqual(self.twin.revolute, [f'L{i}_joint' for i in range(1, 7)])   # the gripper joint L7 is a separate branch
        self.assertEqual(len(self.twin.points([0] * 6)), len(self.twin.chain) + 1)

    def test_matches_pinocchio_tool_position(self):
        try:
            import sys
            import pinocchio  # noqa: F401
            sys.path.insert(0, str(APP.parents[1] / 'fpga_competition_2026/contest/arm_runtime/vendor'))
            from el_a3_sdk.kinematics import ELA3Kinematics
        except ImportError:
            self.skipTest('pinocchio not available in this environment')
        root = APP.parents[1] / 'fpga_competition_2026/contest/arm_runtime'
        kin = ELA3Kinematics(urdf_path=str(root / 'model/el_a3_legacy.urdf'), inertia_config_path=str(root / 'model/inertia_params.yaml'))
        rng = np.random.default_rng(1)
        for _ in range(8):
            q = rng.uniform(-1.2, 1.2, 6)
            reference = kin.forward_kinematics_se3(q.tolist()).translation
            np.testing.assert_allclose(self.twin.tool(list(q)), reference, atol=1e-6)


if __name__ == '__main__':
    unittest.main()
