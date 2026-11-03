"""Exact reuse, state invalidation and corruption recovery without any actuator."""
import gzip
import json
import tempfile
import unittest
from pathlib import Path
from screwcell.preparation_cache import EncodingCache


class EncodingCacheTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        from joint_trajectory import JointPlanner
        from gravity_trajectory import GravityModel
        from motion_runtime import MotionRuntime
        root=Path(__import__('gravity_trajectory').__file__).parent
        cls.p=JointPlanner(root/'profiles/release_20261019_motion.json')
        cls.model=GravityModel(MotionRuntime(),cls.p,root/'profiles/release_20261019_tracking.json')
        from gravity_trajectory import hold_segment
        cls.plan=hold_segment(cls.p,[-.8,.2,-.3,.3,0.,0.,.4],.1)
    def test_exact_encoding_reused_but_return_values_are_independent(self):
        c=EncodingCache(self.model);a=c.encode(self.plan);b=c.encode(self.plan)
        self.assertEqual(a['words'],self.model.encode(self.plan)['words']);self.assertEqual(a['words'],b['words'])
        self.assertEqual(c.stats()['hits'],1);a['gravity_alpha'][0]=0
        self.assertEqual(c.encode(self.plan)['gravity_alpha'],[1.,1.])
    def test_actual_gain_and_torque_state_invalidate_even_with_same_profile_id(self):
        import copy
        import numpy as np
        m=copy.copy(self.model);m.kp=self.model.kp.copy();m.caps=self.model.caps.copy();c=EncodingCache(m)
        a=c.encode(self.plan);m.kp[0]+=1;b=c.encode(self.plan)
        self.assertNotEqual(a['words'][210],b['words'][210]);self.assertEqual(c.misses,2)
        m.caps[:]=.00001
        with self.assertRaises(ValueError):c.encode(self.plan)
    def test_roundtrip_and_corruption_cannot_reuse_old_bytes(self):
        c=EncodingCache(self.model);a=c.encode(self.plan)
        with tempfile.TemporaryDirectory() as d:
            path=Path(d)/'cache.gz';c.save(path);other=EncodingCache(self.model)
            self.assertEqual(other.load(path),'LOADED');self.assertEqual(other.encode(self.plan)['words'],a['words']);self.assertEqual(other.hits,1)
            value=json.loads(gzip.decompress(path.read_bytes()));value['payload']['entries'][0][1]['words'][0]+=100
            path.write_bytes(gzip.compress(json.dumps(value).encode()));fresh=EncodingCache(self.model)
            self.assertTrue(fresh.load(path).startswith('MISS_'));self.assertEqual(len(fresh.entries),0)
            self.assertEqual(fresh.encode(self.plan)['words'],a['words']);self.assertEqual(fresh.misses,1)
    def test_changed_implementation_and_bounded_storage(self):
        c=EncodingCache(self.model,limit=2)
        with tempfile.TemporaryDirectory() as d:
            c.encode(self.plan);path=Path(d)/'cache.gz';c.save(path);other=EncodingCache(self.model);other.implementation='new_code'
            self.assertTrue(other.load(path).startswith('MISS_'));self.assertFalse(other.entries)
        for i in range(5):c.keep(str(i),dict(words=[]))
        self.assertEqual(list(c.entries),['3','4'])
    def test_septic_cache_returns_fresh_lists_and_rechecks_actual_limits(self):
        import copy
        from septic_trajectory import septic_words
        words=septic_words(self.plan,self.p)[0];expected=list(words);words[0]+=100000
        self.assertEqual(septic_words(self.plan,self.p)[0],expected)
        p=copy.copy(self.p);p.upper=self.p.upper.copy();p.upper[1]=.1
        with self.assertRaises(ValueError):septic_words(self.plan,p)

if __name__=='__main__':unittest.main()
