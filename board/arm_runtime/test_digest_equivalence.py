import json,random,unittest
from motion_executor import job_digest,job_and_path_digests
from continuous_transfer import path_digest


def blob(rng):
    return [dict(name='n%d'%i,x=[rng.random() for _ in range(5)],y={'b':1,'a':[rng.random()]}) for i in range(4)]


class DigestEquivalence(unittest.TestCase):
    """One encoding pass must give exactly the two digests the board and host already bind to."""
    def test_matches_separate_digests_for_every_recovery_shape(self):
        rng=random.Random(1)
        for recoveries in ('present','absent','empty'):
            job=dict(schema='s',z=1,a=dict(k=[1,2],j=None),tasks=blob(rng),start_rad=[.1,2.],name='é')
            if recoveries=='present':job['recoveries']={'r2':blob(rng),'r1':blob(rng)}
            if recoveries=='empty':job['recoveries']={}
            with self.subTest(recoveries=recoveries):
                self.assertEqual(job_and_path_digests(job),(job_digest(job),path_digest(job)))

    def test_any_change_changes_both_or_the_job_digest(self):
        rng=random.Random(2);job=dict(tasks=blob(rng),recoveries={'r':blob(rng)},start_rad=[0.])
        base=job_and_path_digests(job);job['tasks'][0]['x'][0]+=1e-12
        self.assertNotEqual(job_and_path_digests(job)[0],base[0]);self.assertNotEqual(job_and_path_digests(job)[1],base[1])
        job['tasks'][0]['x'][0]-=1e-12;job['start_rad']=[1.]
        again=job_and_path_digests(job);self.assertNotEqual(again[0],base[0]);self.assertEqual(again[1],base[1])


if __name__=='__main__':unittest.main()
