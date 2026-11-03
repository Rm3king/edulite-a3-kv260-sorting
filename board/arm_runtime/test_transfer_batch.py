import copy
import unittest
from types import SimpleNamespace
from continuous_transfer import POLICY,contract,path_digest,check_batch_release_temperature,TaskOutcomeError,expected_grasp_height_offset_m
from unittest.mock import Mock


class BatchContractTests(unittest.TestCase):
    def test_slip_trial_fixed_current_and_packet_envelope(self):
        import struct
        from continuous_transfer import TransferDevice
        j,p=self.fixture(['screw_0'])
        p.current[6]=.65
        j['gripper_open_current_a']=j['gripper_close_current_a']=.65
        contract(j,p)
        device=Mock(); wrapper=TransferDevice(device)
        wrapper.enqueue(0x1200fd07,struct.pack('<HHf',0x7018,0,.65))
        device.enqueue.assert_called_once()
        for value in (.7,1.,0.,float('nan'),float('inf')):
            with self.assertRaises(ValueError):
                wrapper.enqueue(0x1200fd07,struct.pack('<HHf',0x7018,0,value))
            p.current[6]=value
            j['gripper_open_current_a']=j['gripper_close_current_a']=value
            with self.assertRaises(ValueError):contract(j,p)
        p.current[6]=.65;j['gripper_open_current_a']=.5;j['gripper_close_current_a']=.65
        with self.assertRaisesRegex(ValueError,'fixed current'):contract(j,p)

    def test_warm_batch_returns_only_after_measured_release_check(self):
        e=SimpleNamespace(s=SimpleNamespace(require=lambda mid,mode:{'temperature':74. if mid==2 else 30.}),event=Mock())
        check_batch_release_temperature(e,{'batch_return_temperature_c':75.},'screw_2_open')
        e.s.require=lambda mid,mode:{'temperature':75. if mid==2 else 30.}
        with self.assertRaises(TaskOutcomeError):check_batch_release_temperature(e,{'batch_return_temperature_c':75.},'screw_2_open')

    def test_temperature_exit_cannot_raise_threshold_or_omit_return(self):
        for setting in [80.,90.,float('nan')]:
            j,p=self.fixture(['screw_0']);j['batch_return_temperature_c']=setting;j['automatic_task_failure_return']=True
            with self.assertRaises(ValueError):contract(j,p)
        j,p=self.fixture(['screw_0']);j['batch_return_temperature_c']=75.
        with self.assertRaises(ValueError):contract(j,p)

    def fixture(self,ids):
        q=[0.]*6+[1.12];tasks=[]
        for i,target in enumerate(ids):
            names=['departure' if i==0 else target+'_transfer_empty']
            names += [target+'_'+phase for phase in ['approach','close','lift','transfer','place','open','exit']]
            for name in names:
                jaw=1.6 if name.endswith(('_close','_lift','_transfer','_place')) else 1.12
                goal=q[:6]+[jaw]
                tasks.append(dict(name=name,goal_rad=goal,segments=[dict(start_rad=goal,goal_rad=goal,coefficients_rad=[[v]+[0.]*7 for v in goal])]))
        tasks.append(dict(name='return_supported',goal_rad=q,return_to_live_start=True,segments=[dict(coefficients_rad=[[v]+[0.]*7 for v in q])]))
        job=dict(policy=POLICY,profile_id='test',startup_no_reset=True,motion_mask=127,
                 start_rad=q,start_motor_rad=q,open_rad=1.12,close_rad=1.6,grasp_height_offset_m=expected_grasp_height_offset_m(i.split('_')[0] for i in ids),
                 gripper_open_current_a=.5,gripper_close_current_a=.5,
                 targets=[dict(target_id=i) for i in ids],tasks=tasks,recoveries={})
        job['geometry_audit']=dict(status='PASS_SAMPLED_WITH_MONOTONIC_EXISTING_START_CONTACT_DEPARTURE',unexpected=[],trajectory_sha256=path_digest(job))
        p=SimpleNamespace(profile_id='test',current=[1.,5.5,4.5,2.5,1.,1.,.5],velocity=[.3]*6+[.25],csp_velocity=[.5]*6+[.35])
        return job,p

    def test_four_screws_then_nut_preserves_exact_sequence(self):
        j,p=self.fixture(['screw_0','screw_1','screw_2','screw_3','nut_0']);contract(j,p)
        j['tasks'][0],j['tasks'][1]=j['tasks'][1],j['tasks'][0]
        j['geometry_audit']['trajectory_sha256']=path_digest(j)
        with self.assertRaises(ValueError):contract(j,p)

    def test_six_duplicate_and_invalid_ids_rejected(self):
        for ids in [['screw_'+str(i) for i in range(5)]+['nut_0'],['screw_0','screw_0'],['screw_5'],['nut_bad']]:
            j,p=self.fixture(ids)
            with self.subTest(ids=ids),self.assertRaises(ValueError):contract(j,p)

    def test_every_outcome_still_requires_its_return_branch(self):
        j,p=self.fixture(['screw_0','screw_1','screw_2','screw_3','nut_0']);j['automatic_task_failure_return']=True
        with self.assertRaises(ValueError):contract(j,p)


if __name__=='__main__':unittest.main()
