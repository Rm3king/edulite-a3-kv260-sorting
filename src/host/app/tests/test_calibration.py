import unittest
import numpy as np
from workcell.calibration import eye_to_hand,tcp_pivot,support_plane
from workcell.simulation import rotation


def pose(rpy,xyz):
    t=np.eye(4);t[:3,:3]=rotation(rpy);t[:3,3]=xyz;return t


class CalibrationTests(unittest.TestCase):
    def setUp(self):self.rng=np.random.default_rng(321)
    def test_eye_to_hand_direction_and_unknown_board_mount(self):
        x=pose([.2,.1,-.3],[.02,-.03,.04]);y=pose([3.,.2,.5],[.1,.2,.5]);samples=[]
        for _ in range(20):
            g=pose(self.rng.uniform(-1,1,3),self.rng.uniform(-.3,.3,3));c=np.linalg.inv(y)@g@x
            samples.append(dict(T_base_end=g,T_camera_board=c))
        result=eye_to_hand(samples[:15],samples[15:])
        np.testing.assert_allclose(result['T_base_camera'],y,atol=1e-10)
        np.testing.assert_allclose(result['T_end_board'],x,atol=1e-10)
        self.assertLess(result['held_out_pose_consistency']['position_rmse_mm'],1e-7)
        self.assertFalse(result['verified'])
    def test_single_rotation_axis_is_rejected(self):
        rows=[dict(T_base_end=pose([0,0,t],[t,0,0]),T_camera_board=pose([0,0,t],[t,0,0])) for t in np.linspace(0,1,10)]
        with self.assertRaisesRegex(ValueError,'NONPARALLEL'):eye_to_hand(rows)
    def test_noisy_pose_fit_keeps_holdout_and_physical_accuracy_separate(self):
        x=pose([.2,.1,-.3],[.02,-.03,.04]);y=pose([3.,.2,.5],[.1,.2,.5]);rows=[]
        for _ in range(25):
            g=pose(self.rng.uniform(-1,1,3),self.rng.uniform(-.3,.3,3))
            noise=pose(self.rng.normal(0,.001,3),self.rng.normal(0,.0003,3))
            rows.append(dict(T_base_end=g,T_camera_board=noise@np.linalg.inv(y)@g@x))
        r=eye_to_hand(rows[:20],rows[20:])
        self.assertLess(np.linalg.norm(np.asarray(r['T_base_camera'])[:3,3]-y[:3,3]),.005)
        self.assertLess(r['held_out_pose_consistency']['position_rmse_mm'],2.)
        self.assertIsNone(r['independent_physical_tcp_error_mm']);self.assertFalse(r['verified'])
    def test_tcp_pivot_recovers_translation_not_orientation(self):
        tip=np.array([.01,.02,-.08]);fixed=np.array([.1,.2,.02]);poses=[]
        for _ in range(15):
            g=pose(self.rng.uniform(-1,1,3),[0,0,0]);g[:3,3]=fixed-g[:3,:3]@tip;poses.append(g)
        r=tcp_pivot(poses[:10],poses[10:]);np.testing.assert_allclose(r['tip_in_end_m'],tip,atol=1e-12)
        self.assertLess(max(r['held_out_error_mm']),1e-7);self.assertFalse(r['verified'])
    def test_fixed_tool_orientation_does_not_determine_tcp(self):
        with self.assertRaisesRegex(ValueError,'DEGENERATE'):tcp_pivot([np.eye(4)]*5)
    def test_plane_outliers_and_independent_points(self):
        xy=self.rng.uniform(-.1,.1,(200,2));z=.012+.02*xy[:,0]+self.rng.normal(0,.0002,200)
        points=np.c_[xy,z];points[:25,2]+=.03
        r=support_plane(points,holdout=[[0,0,.012],[.03,0,.0126]])
        self.assertEqual(r['inliers'],175);self.assertLess(r['fit_rmse_mm'],.3)
        self.assertLess(max(r['held_out_plane_distances_mm']),.1);self.assertFalse(r['verified'])
    def test_collinear_plane_is_rejected(self):
        with self.assertRaises(ValueError):support_plane([[x,0,0] for x in range(20)])
