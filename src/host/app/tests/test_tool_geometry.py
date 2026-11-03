import unittest
from pathlib import Path
import numpy as np
from workcell.tool_geometry import (visual_transform,grasp_transform,width_to_slide,
                                     split_wrist_inertia,parallel,OPEN_INNER_HALF)


class GripperGeometryTests(unittest.TestCase):
    def test_grasp_frame_is_near_cad_tips_not_the_sdk_end(self):
        t=grasp_transform()
        self.assertAlmostEqual(float(t[2,3]),-.098)
        self.assertAlmostEqual(float(t[2,3])-(-.104),.006)
        np.testing.assert_allclose(t[:3,:3],np.eye(3))
    def test_both_visible_hands_follow_the_measured_slide(self):
        poses={'l5_l6_urdf_asm':np.eye(4)}
        for filename,x,side in [('hand_r.stl',.0662,1),('part_15.stl',-.0662,-1)]:
            local=np.eye(4);local[0,3]=x
            visual=dict(path=Path(filename),link='l5_l6_urdf_asm',local=local)
            actual=visual_transform(visual,poses,dict(left=-.04,right=-.04))
            self.assertAlmostEqual(actual[0,3],x-side*.04)
    def test_fixed_guide_does_not_move_with_the_hands(self):
        v=dict(path=Path('rack_base.stl'),link='l5_l6_urdf_asm',local=np.eye(4))
        np.testing.assert_array_equal(visual_transform(v,{'l5_l6_urdf_asm':np.eye(4)},dict(left=-.04,right=-.04)),np.eye(4))
    def test_width_and_prismatic_state_are_consistent(self):
        q=width_to_slide(.012)
        self.assertAlmostEqual(2*OPEN_INNER_HALF+2*q,.012)
        with self.assertRaises(ValueError):width_to_slide(.15)
    def test_redistribution_preserves_vendor_open_spatial_inertia(self):
        m=.6274;c=np.array([0,-.001,-.043338391871556046]);i=np.array([[.00100405,-4.97095e-6,1.06714e-7],[-4.97095e-6,.00139953,1.65952e-5],[1.06714e-7,1.65952e-5,.00054217]])
        fm,fc,fi,jaws=split_wrist_inertia(m,c,i)
        mass=fm+sum(j[0] for j in jaws)
        first=fm*fc+sum(j[0]*j[1] for j in jaws)
        moment=fi+fm*parallel(fc)+sum(j[2]+j[0]*parallel(j[1]) for j in jaws)
        np.testing.assert_allclose(mass,m+.05,atol=1e-14)
        np.testing.assert_allclose(first,m*c+.05*np.array([0,0,-.074]),atol=1e-14)
        np.testing.assert_allclose(moment,i+m*parallel(c)+np.eye(3)*.0001+.05*parallel([0,0,-.074]),atol=1e-14)
