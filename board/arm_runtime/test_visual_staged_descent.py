"""Offline mutation and retreat tests; never opens hardware."""
import copy
import json
import os
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch
from joint_trajectory import JointPlanner
from visual_staged_descent import verify, execute, return_from_stage, inspect_stage
from gripper_aperture_probe import validate, GripperOnlyDevice


class StagedTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.job=json.loads(Path(os.environ['STAGED_DESCENT_TEST_JOB']).read_text())
        cls.planner=JointPlanner(Path(__file__).parent/'profiles/release_20261019_motion.json')

    def test_validated_route_has_four_exact_reverse_branches(self):
        records,branches=verify(self.job,self.planner)
        self.assertGreater(len(records),0)
        self.assertEqual(set(branches),{0,1,2,3})
        for stage,(task,records) in branches.items():
            self.assertEqual(records[0]['plan']['start_rad'],self.job['tasks'][stage]['goal_rad'])
            self.assertLess(max(abs(a-b) for a,b in zip(records[-1]['plan']['goal_rad'],self.job['start_rad'])),1e-10)

    def test_unmeasured_offline_job_cannot_open_hardware(self):
        job=copy.deepcopy(self.job);job['hardware_start_verified']=False
        with patch('visual_staged_descent.CanDevice') as device:
            with self.assertRaises(ValueError):execute(job,self.planner,'unused_no_creation')
            device.assert_not_called()

    def test_invalid_motion_inputs_rejected(self):
        mutations=[lambda j:j.update(motion_mask=127),
                   lambda j:j.update(measured_aperture_mm=50),
                   lambda j:j.update(measurement_complete_hands_clear=False),
                   lambda j:j.update(observed_gripper_motor_rad=1.6),
                   lambda j:j['geometry_audit'].update(status='REJECTED_COLLISION'),
                   lambda j:j['geometry_audit'].update(route_sha256='bad'),
                   lambda j:j['tasks'][0].update(height_above_surface_m=0),
                   lambda j:j['tasks'][-1].update(return_to_live_start=False),
                   lambda j:j['retreats']['0']['segments'].pop(),
                   lambda j:j['tasks'][1]['segments'][0]['coefficients_rad'][0].__setitem__(0,0)]
        for change in mutations:
            job=copy.deepcopy(self.job);change(job)
            with self.subTest(change=change):
                with self.assertRaises(ValueError):verify(job,self.planner)

    def test_retreat_uses_selected_branch_not_lower_stages(self):
        _,branches=verify(self.job,self.planner)
        for stage in range(4):
            e=SimpleNamespace(transport=SimpleNamespace(prepared=None,prepared_index=999),execute_segments=Mock(),settle=Mock(return_value={'verified':True}))
            report={};return_from_stage(e,self.job,stage,branches,report)
            e.execute_segments.assert_called_once_with(branches[stage][0]['segments'])
            self.assertIs(e.transport.prepared,branches[stage][1])
            self.assertTrue(report['safe_to_disable_after_return_or_support'])

    def test_missing_ack_returns_instead_of_descending(self):
        import tempfile
        e=SimpleNamespace(report={'job_sha256':'test'},event=Mock(),pump=Mock(),diagnostics=Mock())
        with tempfile.TemporaryDirectory() as directory,patch('visual_staged_descent.time.monotonic',side_effect=[0,46]):
            result=inspect_stage(e,self.job,Path(directory),0,{'actual_motor_rad':[0]*6})
        self.assertEqual(result['action'],'RETURN')

    def test_gripper_only_command_boundary(self):
        device=GripperOnlyDevice(Mock())
        for mid in range(1,7):
            with self.assertRaises(ValueError):device.enqueue(0x0300fd00|mid,bytes(8))
        for address,value in ((0x5a0,127),(0x5c0,1),(0x800,1)):
            with self.assertRaises(ValueError):device.write(address,value)
        device.device.enqueue.assert_not_called();device.device.write.assert_not_called()

    def test_gripper_target_is_inward_and_bounded(self):
        self.assertLess(validate(1.69658,1.22)['initial_target_rad'],1.677834153175354)
        for start,target in ((float('nan'),1.2),(1.69,.1),(1.69,1.8),(1.9,1.4)):
            with self.assertRaises(ValueError):validate(start,target)


if __name__=='__main__':unittest.main()
