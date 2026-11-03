"""Screw localisation in a synthetic wrist frame: centre, axis, and refusal of nuts, clutter and cut-off bodies."""
import unittest
import cv2
import numpy as np
from workcell.wrist_screw_localization import locate_screw_near,axis_difference_deg,across_axis_error


def tray(seed=0):
    rng=np.random.default_rng(seed);img=np.full((400,640),175,np.uint8)
    for _ in range(60):cv2.circle(img,(int(rng.integers(0,640)),int(rng.integers(0,400))),int(rng.integers(2,5)),235,-1)  # specular dots
    return cv2.GaussianBlur(img,(0,0),1.0)


def draw_screw(img,center,length,width,axis_deg):
    box=cv2.boxPoints((tuple(map(float,center)),(float(length),float(width)),float(axis_deg)))
    cv2.fillConvexPoly(img,box.astype(np.int32),35)
    ux,uy=np.cos(np.deg2rad(axis_deg)),np.sin(np.deg2rad(axis_deg))
    cv2.circle(img,(int(center[0]+ux*length/2),int(center[1]+uy*length/2)),int(width*.75),35,-1)  # head
    return cv2.GaussianBlur(img,(0,0),1.2)


class Screw(unittest.TestCase):
    def test_centre_and_axis_for_several_orientations(self):
        for axis in (0,35,90,125,170):
            with self.subTest(axis=axis):
                img=draw_screw(tray(axis),(300,210),170,20,axis)
                found=locate_screw_near(img,(280,230))
                # the head sticks out 0.75 widths past the shaft end: the extent centre moves half of that toward it
                ux,uy=np.cos(np.deg2rad(axis)),np.sin(np.deg2rad(axis));expected=(300+ux*7.5,210+uy*7.5)
                self.assertLess(np.hypot(found['center_px'][0]-expected[0],found['center_px'][1]-expected[1]),3.)
                self.assertLess(axis_difference_deg(found['axis_deg'],axis),4.)

    def test_vertical_screw_matches_the_jaw_direction_convention(self):
        found=locate_screw_near(draw_screw(tray(),(280,200),150,18,90),(270,215))
        self.assertLess(axis_difference_deg(found['axis_deg'],90),3.)  # axis 90 = image y = along the fingers

    def test_nut_is_not_a_screw(self):
        img=tray();cv2.circle(img,(300,210),48,35,-1)
        with self.assertRaises(ValueError):locate_screw_near(cv2.GaussianBlur(img,(0,0),1.2),(290,220))

    def test_empty_tray_is_refused(self):
        with self.assertRaises(ValueError):locate_screw_near(tray(),(300,210))

    def test_body_cut_by_the_image_is_flagged_not_trusted(self):
        found=locate_screw_near(draw_screw(tray(),(60,210),330,20,0),(100,210))
        self.assertTrue(found['cut']);self.assertFalse(locate_screw_near(draw_screw(tray(),(300,210),170,20,0),(300,210))['cut'])

    def test_two_screws_at_the_same_distance_are_ambiguous(self):
        img=draw_screw(tray(),(300,200),150,20,90);img=draw_screw(img,(345,215),150,20,90)
        with self.assertRaises(ValueError):locate_screw_near(img,(322,208))

    def test_neighbour_beyond_the_window_does_not_move_the_target(self):
        # the real tray case: a second screw lying across the first one's search window, outline 78 px from the grasp point
        img=draw_screw(tray(),(222,266),284,22,80);img=draw_screw(img,(463,129),305,22,0)
        found=locate_screw_near(img,(272,207))
        self.assertLess(np.hypot(found['center_px'][0]-222,found['center_px'][1]-266),3.)

    def test_neighbour_too_close_for_the_jaws_is_refused(self):
        img=draw_screw(tray(),(300,260),150,20,90);img=draw_screw(img,(345,240),150,20,90)
        with self.assertRaisesRegex(ValueError,'too close'):locate_screw_near(img,(300,260))

    def test_axis_difference_wraps(self):
        self.assertAlmostEqual(axis_difference_deg(179,1),2.)
        self.assertAlmostEqual(axis_difference_deg(90,90),0.)

    def test_cut_body_corrects_only_across_the_axis(self):
        obs=[dict(center_px=[264.,261.],axis_deg=90.,cut=True)]*3
        error,_,spread=across_axis_error((272.,207.),obs)
        self.assertAlmostEqual(error[0],8.,places=6);self.assertAlmostEqual(error[1],0.,places=6);self.assertEqual(spread,0.)

    def test_complete_body_corrects_both_axes(self):
        obs=[dict(center_px=[264.,261.],axis_deg=90.,cut=False)]*3
        error,_,_=across_axis_error((272.,207.),obs)
        self.assertAlmostEqual(error[0],8.);self.assertAlmostEqual(error[1],-54.)

    def test_cut_head_end_uses_the_visible_tip_and_known_length(self):
        # tip at y=125 pointing up, body runs off the bottom edge: middle is 147.5 px below the tip
        obs=[dict(center_px=[264.,262.],axis_deg=90.,cut=True,length_px=275.)]*3
        error,middle,_=across_axis_error((272.,207.),obs,295.)
        self.assertAlmostEqual(middle[1],125.+147.5,delta=1.);self.assertAlmostEqual(error[1],207.-272.5,delta=1.);self.assertAlmostEqual(error[0],8.,delta=1e-6)

    def test_cut_at_both_ends_falls_back_to_across_only(self):
        obs=[dict(center_px=[264.,200.],axis_deg=90.,cut=True,length_px=500.)]*3
        error,_,_=across_axis_error((272.,207.),obs,295.)
        self.assertAlmostEqual(error[1],0.,places=6)


if __name__=='__main__':unittest.main()
