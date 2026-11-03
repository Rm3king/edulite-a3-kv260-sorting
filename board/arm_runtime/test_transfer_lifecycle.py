"""Exercise the real transfer exception/finally paths without opening hardware."""
import contextlib
import io
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import Mock,patch

import continuous_transfer as transfer


class LifecycleTests(unittest.TestCase):
    def run_case(self,kind,write_failure=False):
        events=[];report_ref={}
        q=[0.]*6+[1.12]
        names=['departure','screw_0_approach','screw_0_close','screw_0_lift',
               'screw_0_transfer','screw_0_place','screw_0_open','screw_0_exit','return_supported']
        if kind in ('marker','marker_lift'):names=['marker_start_approach','marker_tilt_approach','return_supported']
        if kind=='marker_lift':names[1]='marker_lift_approach'
        if kind=='hover':names=['departure','hover_approach','hover_lift','return_supported']
        tasks=[]
        for name in names:
            if name.endswith('_close'):q=q[:6]+[1.6]
            if name.endswith('_open'):q=q[:6]+[1.12]
            tasks.append(dict(name=name,goal_rad=q[:],segments=[{'name':name}]))
        recoveries={t['name']:dict(start_rad=t['goal_rad'],goal_rad=t['goal_rad'],segments=[{'name':'recovery'}])
                    for t in tasks if t['name']=='departure' or t['name'].endswith(('_close','_open'))}
        records=SimpleNamespace(recoveries={n:['reviewed'] for n in recoveries})
        job=dict(hardware_start_verified=True,start_rad=[0.]*6+[1.12],open_rad=1.12,close_rad=1.6,
                 tasks=tasks,recoveries=recoveries,automatic_task_failure_return=True)
        if kind in ('marker','marker_lift'):job.update(policy=transfer.MARKER_LIFT_POLICY if kind=='marker_lift' else transfer.MARKER_POLICY,marker_capture_seconds=3.,start_motor_rad=[0.]*6+[1.12])
        if kind=='hover':job.update(policy=transfer.HOVER_POLICY,hover_capture_seconds=2.)
        planner=SimpleNamespace(to_motor=lambda value:list(value))
        device=Mock(max_gap=.005);device.read.side_effect=lambda a:127 if a in (0x5a0,0x5a8) else 0
        device.status.return_value={'fault':0}

        class Executor:
            def __init__(self,p,d,report,prepared):
                self.p=p;self.d=d;self.report=report;report_ref['value']=report;self.state='HOLDING'
                self.transport=SimpleNamespace(status=lambda:{'fault':0},prepared=None,prepared_index=0)
                self.s=SimpleNamespace(read=self.read,require=lambda mid,mode:{'position':0. if mid<7 else 1.5},stop_all=self.stop)
            def event(self,event_code,**kw):events.append(event_code)
            def prepare(self,job):
                events.append('prepare_before_enable')
                self.report['owned']=True
                if kind=='prepare':raise ValueError('pre-enable mismatch')
                self.report['enable_attempted']=True
            def release_host_hold(self):pass
            def pump(self,seconds):pass
            def hold(self,seconds):
                assert seconds==(3. if kind in ('marker','marker_lift') else 2.);events.append('bounded_hover_capture')
            def settle_arm(self,goal,**kw):return {'ok':True,'actual_motor_rad':list(goal[:6])}
            def execute_segments(self,segments):
                name=segments[0]['name'];events.append(name);self.state='MOVING'
                if kind=='following' and name=='screw_0_transfer':raise RuntimeError('Axis 2: sustained following error')
                if kind=='recovery_fails' and name=='recovery':raise RuntimeError('recovery motor fault')
                self.state='HOLDING'
            def read(self,mid,index):
                task=self.report['last_completed_task']
                if task.endswith('_close'):return 1.6 if kind in ('empty','recovery_fails') else 1.5
                return 1.498 if kind=='release' else 1.12
            def stop(self):
                assert self.report.get('supported_disable_requested')
                events.append('disable');self.report['reset_confirmed']=list(range(1,8))

        def retained(e,out,report):
            events.append('operator_support_and_permission')
            report['supported_disable_requested']=True
        def approval(e,out,report):
            if not report.get('supported_disable_requested'):
                events.append('operator_disable_permission');report['supported_disable_requested']=True

        original_write=Path.write_text
        def write(path,*args,**kwargs):
            if write_failure and path.name=='failure_context.json':raise OSError('disk failure')
            return original_write(path,*args,**kwargs)
        with tempfile.TemporaryDirectory() as tmp,contextlib.ExitStack() as stack:
            for name,value in [('CanDevice',lambda:device),('TransferDevice',lambda d:d),('GraspExecutor',Executor),
                               ('motion_gc_scope',contextlib.nullcontext),('WaitStateWriter',lambda out:(events.append('writer_initialized') or Mock())),('retain_until_supported',retained),('await_disable_approval',approval)]:
                stack.enter_context(patch.object(transfer,name,value))
            stack.enter_context(patch.object(transfer,'open',create=True,return_value=io.StringIO()))
            stack.enter_context(patch.object(transfer.fcntl,'flock'))
            stack.enter_context(patch.object(Path,'write_text',write))
            result=transfer.execute(job,planner,Path(tmp)/'execution',records)
            summary=json.loads((Path(tmp)/'execution/runtime_summary.json').read_text())
            self.assertEqual(result['status'],summary['status'])
            self.assertEqual(result.get('automatic_return_error'),summary.get('automatic_return_error'))
            self.assertEqual(result.get('recovery_start_check'),summary.get('recovery_start_check'))
            self.assertTrue((Path(tmp)/'execution/result.json.gz').is_file())
        return events,result

    def test_complete_requires_permission_before_disable(self):
        events,result=self.run_case('ok')
        self.assertLess(events.index('writer_initialized'),events.index('prepare_before_enable'))
        self.assertTrue(result['wait_writer_prepared_before_enable'])
        self.assertEqual(events[-2:],['operator_disable_permission','disable'])
        self.assertEqual(result['status'],'COMPLETED_TRANSFER_MOTION_VERIFY_IMAGES')

    def test_hover_captures_then_returns_without_any_grasp(self):
        events,result=self.run_case('hover')
        self.assertEqual(events.count('bounded_hover_capture'),1)
        self.assertLess(events.index('bounded_hover_capture'),events.index('hover_lift'))
        self.assertNotIn('JAW_OUTCOME',events)
        self.assertEqual(events[-2:],['operator_disable_permission','disable'])

    def test_marker_observation_returns_and_requires_disable_permission(self):
        events,result=self.run_case('marker')
        self.assertEqual(events.count('bounded_hover_capture'),2)
        self.assertNotIn('JAW_OUTCOME',events)
        self.assertEqual(events[-2:],['operator_disable_permission','disable'])
        self.assertEqual(result['status'],'COMPLETED_MARKER_OBSERVATION_RETURN')
        self.assertEqual(len(result['marker_capture_ends']),2)
        self.assertTrue(all(len(x['actual_motor_rad'])==7 for x in result['marker_capture_ends']))

    def test_marker_lift_returns_and_waits_for_disable(self):
        events,result=self.run_case('marker_lift')
        self.assertEqual(events.count('bounded_hover_capture'),2)
        self.assertEqual(result['status'],'COMPLETED_MARKER_OBSERVATION_RETURN')
        self.assertEqual(events[-2:],['operator_disable_permission','disable'])

    def test_empty_grasp_returns_without_attempting_transfer(self):
        events,result=self.run_case('empty')
        self.assertNotIn('screw_0_transfer',events);self.assertIn('recovery',events)
        self.assertEqual(events[-2:],['operator_disable_permission','disable'])
        self.assertEqual(result['status'],'TASK_FAILED_RETURNED_SUPPORTED')

    def test_failed_release_returns_instead_of_starting_next_task(self):
        events,result=self.run_case('release')
        self.assertIn('recovery',events);self.assertNotIn('screw_0_exit',events)
        self.assertEqual(result['status'],'TASK_FAILED_RETURNED_SUPPORTED')

    def test_moving_following_fault_does_not_launch_endpoint_return(self):
        events,result=self.run_case('following')
        self.assertNotIn('recovery',events)
        self.assertEqual(events[-2:],['operator_support_and_permission','disable'])

    def test_failure_to_write_evidence_does_not_skip_return(self):
        events,result=self.run_case('empty',write_failure=True)
        self.assertIn('recovery',events);self.assertIn('failure_context_write_error',result)

    def test_recovery_fault_never_auto_disables(self):
        events,result=self.run_case('recovery_fails')
        self.assertEqual(events[-2:],['operator_support_and_permission','disable'])
        self.assertIn('automatic_return_error',result)
        self.assertIn('AUTOMATIC_RETURN_REJECTED',events)

    def test_pre_enable_failure_sends_no_motor_disable(self):
        events,result=self.run_case('prepare')
        self.assertNotIn('disable',events);self.assertEqual(result['status'],'FAIL')


if __name__=='__main__':unittest.main()
