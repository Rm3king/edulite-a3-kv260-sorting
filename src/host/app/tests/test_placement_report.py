"""Placement acceptance: matching to slots, wrong class, missing, extra parts, and the mm estimate."""
import unittest
from workcell.placement_report import verify_placement,mm_per_pixel

CONFIG={'source_roi_uv':[[0,0],[1000,0],[1000,1000],[0,1000]]}   # source tray is the left part of the image
SLOTS=[[1400.,300.],[1500.,400.],[1600.,500.]]


def obj(cls,u,v,scale=0.25):   # 0.25 mm per pixel
    return dict(part_class=cls,center_uv_px=[u,v],base_xyz_m=[u*scale/1000,v*scale/1000,0.])


class Placement(unittest.TestCase):
    def test_all_placed_reports_worst_deviation_in_mm(self):
        pts=dict(objects=[obj('screw',1404,303),obj('nut',1500,400),obj('screw',100,100),obj('nut',300,500)])
        r=verify_placement(pts,CONFIG,[dict(part_class='screw',slot=0),dict(part_class='nut',slot=1)],SLOTS)
        self.assertEqual((r['placed'],r['expected']),(2,2));self.assertAlmostEqual(r['worst_deviation_mm'],5*.25,places=3)
        self.assertEqual(r['unexpected'],[]);self.assertIn('2/2',r['summary'])

    def test_missing_wrong_class_and_extra(self):
        pts=dict(objects=[obj('nut',1400,300),obj('screw',1580,500),obj('nut',1700,700),obj('screw',100,100),obj('nut',300,500)])
        r=verify_placement(pts,CONFIG,[dict(part_class='screw',slot=0),dict(part_class='nut',slot=1),dict(part_class='screw',slot=2)],SLOTS)
        self.assertEqual([p['status'] for p in r['parts']],['WRONG_CLASS','MISSING','PLACED'])
        self.assertEqual(len(r['unexpected']),1)

    def test_a_part_cannot_satisfy_two_slots(self):
        pts=dict(objects=[obj('screw',1430,330),obj('screw',100,100),obj('nut',300,500)])
        r=verify_placement(pts,CONFIG,[dict(part_class='screw',slot=0),dict(part_class='screw',slot=1)],SLOTS)
        self.assertEqual(sorted(p['status'] for p in r['parts']),['MISSING','PLACED'])

    def test_scale_needs_two_distant_parts(self):
        self.assertIsNone(mm_per_pixel([obj('nut',100,100)]))
        self.assertAlmostEqual(mm_per_pixel([obj('nut',100,100),obj('nut',400,100)]),.25)


if __name__=='__main__':unittest.main()
