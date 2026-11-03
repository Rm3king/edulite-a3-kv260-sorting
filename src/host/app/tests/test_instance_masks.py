import unittest
import numpy as np
from workcell.instance_masks import refine


class InstanceMaskTests(unittest.TestCase):
    def prediction(self, masks, scores=None, classes=None):
        return dict(masks=np.array(masks,np.uint8),scores=np.array(scores or [.9]*len(masks)),
                    classes=np.array(classes or [0]*len(masks)))

    def test_duplicate_removed_but_adjacent_separate_part_retained(self):
        a=np.zeros((40,40),np.uint8);a[5:15,5:20]=1
        b=np.zeros_like(a);b[17:27,5:20]=1
        p=self.prediction([a,a.copy(),b],[.9,.6,.8]);before=p['masks'].copy()
        r=refine(p)
        self.assertEqual([v[0] for v in r],[0,2])
        self.assertEqual(r[0][2]['suppressed_duplicates'][0]['index'],1)
        np.testing.assert_array_equal(p['masks'],before)

    def test_fragment_removed_only_with_independent_neighbor_ownership(self):
        a=np.zeros((40,40),np.uint8);a[5:15,5:25]=1;a[20:22,5:20]=1
        b=np.zeros_like(a);b[18:28,5:25]=1
        single=refine(self.prediction([a]))[0]
        np.testing.assert_array_equal(single[1],a)
        r=refine(self.prediction([a,b]))
        self.assertEqual(int(r[0][1].sum()),200)
        self.assertEqual(r[0][2]['removed_fragments'][0]['owner_index'],1)
        other_class=refine(self.prediction([a,b],classes=[0,1]))
        np.testing.assert_array_equal(other_class[0][1],a)

    def test_equally_fragmented_or_overlapping_bodies_not_simplified(self):
        a=np.zeros((40,40),np.uint8);a[5:15,5:15]=1;a[20:30,5:15]=1
        b=np.zeros_like(a);b[20:30,5:15]=1
        r=refine(self.prediction([a,b]));np.testing.assert_array_equal(r[0][1],a)

    def test_empty_and_cross_class_duplicate_are_preserved(self):
        a=np.ones((10,10),np.uint8)
        self.assertEqual(len(refine(self.prediction([a,a],classes=[0,1]))),2)
        self.assertEqual(len(refine(self.prediction([np.zeros_like(a)]))),1)

    def test_same_body_with_low_confidence_other_class_is_deduplicated(self):
        a=np.zeros((50,50),np.uint8);a[10:40,20:30]=1
        p=self.prediction([a,a.copy()],[.56,.30],[2,3])
        p['boxes']=np.array([[20,10,30,40],[20,10,30,40]],float)
        before=p['masks'].copy();r=refine(p)
        self.assertEqual([v[0] for v in r],[0])
        self.assertTrue(r[0][2]['suppressed_duplicates'][0]['cross_class'])
        self.assertEqual(r[0][2]['suppressed_duplicates'][0]['suppressed_class'],3)
        np.testing.assert_array_equal(p['masks'],before)

    def test_cross_class_ambiguous_scores_remain_for_geometry_check(self):
        a=np.ones((20,20),np.uint8)
        p=self.prediction([a,a],[.56,.50],[2,3]);p['boxes']=np.array([[0,0,20,20]]*2)
        self.assertEqual(len(refine(p)),2)

    def test_cross_class_distinct_bodies_in_same_box_are_not_merged(self):
        a=np.zeros((40,40),np.uint8);a[5:35,5:15]=1
        b=np.zeros_like(a);b[5:35,20:30]=1
        p=self.prediction([a,b],[.8,.3],[2,3]);p['boxes']=np.array([[0,0,40,40]]*2)
        self.assertEqual(len(refine(p)),2)

    def test_cross_class_disconnected_fragment_keeps_unknown_obstacle(self):
        a=np.zeros((50,50),np.uint8);a[10:40,20:30]=1
        b=a.copy();b[3:5,3:5]=1
        p=self.prediction([a,b],[.8,.3],[2,3]);p['boxes']=np.array([[0,0,50,50]]*2)
        self.assertEqual(len(refine(p)),2)

    def test_cross_class_disagreeing_boxes_are_not_merged(self):
        a=np.ones((20,20),np.uint8)
        p=self.prediction([a,a],[.8,.3],[2,3]);p['boxes']=np.array([[0,0,20,20],[0,0,12,12]])
        self.assertEqual(len(refine(p)),2)

    def test_reciprocal_leak_does_not_merge_two_distinct_bodies(self):
        a=np.zeros((50,50),np.uint8);a[5:15,5:35]=1;a[25:35,5:25]=1
        b=np.zeros_like(a);b[25:35,5:35]=1;b[5:15,5:25]=1
        # Raw IoU is 2/3 despite two disjoint full bodies.
        r=refine(self.prediction([a,b],[.92,.91]))
        self.assertEqual(len(r),2)
        self.assertEqual([int(v[1].sum()) for v in r],[300,300])
        self.assertTrue(all(v[2]['removed_fragments'][0]['reciprocal_ownership'] for v in r))
        single=refine(self.prediction([a]))
        np.testing.assert_array_equal(single[0][1],a)


if __name__=='__main__':unittest.main()
