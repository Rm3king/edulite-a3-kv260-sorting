"""No-hardware lifecycle checks for attended wrist observations."""
import copy
import json
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch
import wrist_handeye_motion as motion


class Lifecycle(unittest.TestCase):
    def test_initial_review_requires_exact_image_and_job(self):
        class Executor:
            def pump(self,*args):pass
            def diagnostics(self):pass
        result=dict(job_sha256='job',frames=[{},dict(sha256='image')])
        with tempfile.TemporaryDirectory() as root:
            marker=Path(root)/'initial_capture_review.json'
            review=dict(full_grid_found=True,corner_count=289,job_sha256='job',image_sha256='image')
            marker.write_text(json.dumps(review))
            self.assertTrue(motion.review_initial_capture(Executor(),root,result))
            for field,value in [('job_sha256','old'),('image_sha256','old'),('full_grid_found',False),('corner_count',288)]:
                marker.write_text(json.dumps(dict(review,**{field:value})))
                self.assertFalse(motion.review_initial_capture(Executor(),root,result))
            marker.unlink()
            with patch.object(motion.time,'monotonic',side_effect=[0.,6.]):
                self.assertFalse(motion.review_initial_capture(Executor(),root,result))

    def run_case(self, fail=False):
        events = []
        class Device:
            max_gap = 0.
            def status(self):return dict(fault=0,control=0)
            def read(self, address):return 0
            def close(self):events.append('close')
        class Feedback:
            def require(self, mid, mode):return dict(temperature=25)
            def stop_all(self):
                events.append('disable')
                report['reset_confirmed'] = list(range(1,8))
        class Executor:
            def __init__(self,p,d,r,records):
                nonlocal report
                report=r;self.s=Feedback();self.d=d
            def prepare(self,job):
                events.append('enable');report.update(owned=True,enable_attempted=True)
            def settle_arm(self,*args,**kwargs):return dict(actual_motor_rad=[0.]*6)
            def release_host_hold(self):pass
            def event(self,*args,**kwargs):pass
            def execute_segments(self,plans):
                events.append('move')
                if fail:raise RuntimeError('injected control failure')
        class Writer:
            def __init__(self,out):events.append('writer_started')
            def close(self):pass
        class Worker:
            def __init__(self,out):
                import queue
                events.append('camera_worker_started');self.requests=queue.Queue()
        report={}
        job=dict(start_rad=[0.]*7,start_motor_rad=[0.]*7,tasks=[
            dict(name='pose0',goal_rad=[0.]*7,segments=[{}],capture=True,split='fit'),
            dict(name='return',goal_rad=[0.]*7,segments=[{}],capture=False)])
        with tempfile.TemporaryDirectory() as root, \
             patch.object(motion,'CanDevice',return_value=object()), \
             patch.object(motion,'TransferDevice',return_value=Device()), \
             patch.object(motion,'GraspExecutor',Executor), \
             patch.object(motion,'WaitStateWriter',Writer), \
             patch.object(motion,'CameraWorker',Worker), \
             patch.object(motion,'stationary_capture',return_value=dict(status='CAPTURED')), \
             patch.object(motion,'retain_until_supported',side_effect=lambda *a:events.append('support_required')), \
             patch.object(motion,'await_disable_approval',side_effect=lambda *a:events.append('approval_required')):
            result=motion.execute(job,None,Path(root)/'run',([],{}))
        return events,result

    def test_success_workers_precede_enable_and_disable_follows_approval(self):
        events,result=self.run_case()
        self.assertLess(events.index('camera_worker_started'),events.index('enable'))
        self.assertLess(events.index('writer_started'),events.index('enable'))
        self.assertLess(events.index('approval_required'),events.index('disable'))
        self.assertEqual(result['status'],'COMPLETED_HAND_EYE_CAPTURE_RETURN')

    def test_failure_retains_before_approval_and_disable(self):
        events,result=self.run_case(True)
        self.assertLess(events.index('support_required'),events.index('approval_required'))
        self.assertLess(events.index('approval_required'),events.index('disable'))
        self.assertEqual(result['status'],'FAILED')

    def test_default_policy_cannot_reset_or_select_another_mask(self):
        for job in ({},dict(policy=motion.POLICY,startup_no_reset=False,motion_mask=127),
                    dict(policy=motion.POLICY,startup_no_reset=True,motion_mask=63)):
            with self.assertRaises(ValueError):motion.verify(job,None)


if __name__=='__main__':unittest.main()
