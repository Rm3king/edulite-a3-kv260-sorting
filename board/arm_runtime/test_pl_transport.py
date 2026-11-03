import copy
import random
import unittest
from joint_trajectory import JointPlanner,Q24
from pl_transport import segment_words,extrema

class TransportTests(unittest.TestCase):
    def test_random_quantized_extrema_derivatives_and_phase(self):
        p=JointPlanner();rng=random.Random(89)
        for _ in range(200):
            a=[rng.uniform(l+.02,h-.02) for l,h in zip(p.lower,p.upper)]
            b=[max(l,min(h,x+rng.uniform(-.02,.02))) for x,l,h in zip(a,p.lower,p.upper)]
            plan=p.movej(a,b);words,end=segment_words(plan,p)
            self.assertEqual(len(words),45)
            self.assertEqual(words[42]*words[43]+words[44],Q24)
            for i in range(7):
                c=words[i*4:i*4+4];lo,hi=extrema(c)
                self.assertGreaterEqual(lo,words[28+i]);self.assertLessEqual(hi,words[35+i])
                self.assertEqual(sum(c),end[i]);self.assertEqual(c[1]+2*c[2]+3*c[3],0)
    def test_bad_plan_not_silently_executed(self):
        p=JointPlanner();a=[0,.2,-.2,0,0,0,.2];b=[.1,.3,-.3,.1,.1,.1,.3];plan=p.movej(a,b)
        for key,value in [('tick_s',.004),('ticks',0),('kind','CONTROLLED_STOP_CANDIDATE'),('profile_id','other')]:
            bad=copy.deepcopy(plan);bad[key]=value
            with self.assertRaises(ValueError):segment_words(bad,p)
        bad=copy.deepcopy(plan);bad['ticks']=1
        with self.assertRaises(ValueError):segment_words(bad,p)
    def test_commit_consumed_before_status_read_and_discontinuity(self):
        from pl_transport import PlTransport
        class Device:
            def __init__(self):self.regs={0x470:0x50000,0x404:7,0x800:1}
            def read(self,a):return self.regs.get(a,0)
            def write(self,a,v):
                if a!=0x820:self.regs[a]=v
                # Accepted entry is already active; queued flag remains clear.
        p=JointPlanner();dev=Device();t=PlTransport(dev,p)
        a=[0,.2,-.2,0,0,0,.2];b=[.01,.21,-.21,.01,.01,.01,.21]
        t.submit(p.movej(a,b))
        with self.assertRaisesRegex(ValueError,'discontinuous'):t.submit(p.movej(a,b))
        dev.regs[0x404]=0
        with self.assertRaises(RuntimeError):t.submit(p.movej(b,a))

    def test_polynomial_between_endpoints_is_checked(self):
        lo,hi=extrema([0,4,-4,0]);self.assertEqual((lo,hi),(0.,1.))

if __name__=='__main__':unittest.main()
