import copy
import unittest
from types import SimpleNamespace
from continuous_transfer import HOVER_POLICY,contract,path_digest


class HoverContractTests(unittest.TestCase):
    def fixture(self):
        q=[0.]*6+[1.12]
        tasks=[dict(name=n,goal_rad=q[:],segments=[dict(coefficients_rad=[[v]+[0.]*7 for v in q])])
               for n in ['departure','hover_approach','hover_lift','return_supported']]
        tasks[-1]['return_to_live_start']=True
        job=dict(policy=HOVER_POLICY,startup_no_reset=True,motion_mask=127,profile_id='test',
                 start_rad=q,start_motor_rad=q,open_rad=1.12,gripper_close_current_a=.5,
                 gripper_open_current_a=.5,initial_open_during_departure=True,
                 hover_clearance_m=.06,hover_capture_seconds=2.,tasks=tasks,recoveries={})
        job['geometry_audit']=dict(status='PASS_SAMPLED_WITH_MONOTONIC_EXISTING_START_CONTACT_DEPARTURE',
                                  unexpected=[],trajectory_sha256=path_digest(job),hover_min_clearance_m=.06)
        p=SimpleNamespace(profile_id='test',current=[1.,5.5,4.5,2.5,1.,1.,.5],
                          velocity=[.3]*6+[.25],csp_velocity=[.5]*6+[.35])
        return job,p

    def test_valid_hover_contract(self):
        job,p=self.fixture();contract(job,p)

    def test_rejects_grasp_or_contact_or_unbounded_wait(self):
        base,p=self.fixture()
        for change in ['low','missing_clearance','long_wait','close','target']:
            job=copy.deepcopy(base)
            if change=='low':job['hover_clearance_m']=.017
            if change=='missing_clearance':del job['geometry_audit']['hover_min_clearance_m']
            if change=='long_wait':job['hover_capture_seconds']=60.
            if change=='close':job['tasks'][1]['goal_rad'][6]=1.6
            if change=='target':job['targets']=[{'target_id':'nut_0'}]
            job['geometry_audit']['trajectory_sha256']=path_digest(job)
            with self.subTest(change=change),self.assertRaises(ValueError):contract(job,p)


if __name__=='__main__':unittest.main()
