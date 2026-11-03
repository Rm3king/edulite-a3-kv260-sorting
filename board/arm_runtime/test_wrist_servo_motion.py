"""Pure validation / timeout regressions; no device is opened."""
import copy,math,queue,unittest
from unittest.mock import patch
import wrist_servo_motion as m

class Binding(unittest.TestCase):
    def setUp(self):
        self.req=dict(nonce='new',job_sha256='job',frame_sha256=['a','b','c'],iteration=0)
        self.reply=dict(self.req,action='CONVERGED',pixel_error=2.)
    def test_binds_each_frame_and_iteration(self):
        for key,value in [('nonce','old'),('job_sha256','old'),('frame_sha256',['x','b','c']),('iteration',1)]:
            with self.assertRaises(ValueError):m.validate_reply(dict(self.reply,**{key:value}),self.req,{},None)
    def test_convergence_requires_finite_small_error(self):
        for value in [float('nan'),float('inf'),5.01]:
            with self.assertRaises(ValueError):m.validate_reply(dict(self.reply,pixel_error=value),self.req,{},None)
        self.assertEqual(m.validate_reply(self.reply,self.req,{},None)['action'],'CONVERGED')
    def test_short_interior_segment_refused_at_rest(self):
        # 2026-10-28: durations .25/.015/.25 starved the PL queue at non-zero velocity mid-move
        sliver=dict(name='visual_xy',segments=[dict(duration_s=x) for x in (.25,.015,.25)])
        fine=dict(name='visual_xy',segments=[dict(duration_s=x) for x in (.25,.13,.13,.25)])
        with self.assertRaisesRegex(ValueError,'too short'):m.require_serviceable_segments([fine,sliver])
        m.require_serviceable_segments([fine,dict(name='x',segments=[dict(duration_s=.015),dict(duration_s=.25)])])
    def test_unknown_actions_rejected(self):
        with self.assertRaises(ValueError):m.validate_reply(dict(self.reply,action='GRASP'),self.req,{},None)
    def test_timeout_keeps_pumping(self):
        calls=[]
        class Executor:
            def pump(self,*args):calls.append('pump')
            def diagnostics(self):calls.append('diagnostics')
        with patch.object(m.time,'monotonic',side_effect=[0,1,17,33]):
            result=m.bounded_reply(Executor(),queue.Queue())
        self.assertFalse(result['ok']);self.assertEqual(calls,['pump','diagnostics']*2)
    def test_large_reply_services_control_and_preserves_identity(self):
        import tempfile,pickle
        from pathlib import Path
        calls=[]
        class Executor:
            def pump(self,*args):calls.append('pump')
            def diagnostics(self):calls.append('diagnostics')
        plan={'coefficients':list(range(100000))}
        value={'candidate':plan,'record':{'plan':plan}}
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'reply.pkl';path.write_bytes(pickle.dumps(value,protocol=4))
            result=m.read_prepared_reply(Executor(),path)
        self.assertIs(result['candidate'],result['record']['plan'])
        self.assertGreater(calls.count('pump'),5)
        self.assertEqual(calls.count('pump'),calls.count('diagnostics'))

    def test_ready_response_does_not_wait(self):
        class Executor:
            def pump(self,*args):pass
            def diagnostics(self):pass
        q=queue.Queue();q.put({'ok':True});self.assertTrue(m.bounded_reply(Executor(),q)['ok'])


class Lifecycle(unittest.TestCase):
    def run_case(self, fail=False, vision_ok=True, retry=False, chain=1, tail=None):
        import tempfile
        from pathlib import Path
        events=[];report={}
        class Process:
            def start(self):events.append('worker_start')
            def terminate(self):pass
            def join(self,**kw):pass
        class Context:
            Queue=queue.Queue
            def Process(self,**kw):return Process()
        class Device:
            max_gap=0
            def status(self):return {'fault':0,'control':0}
            def read(self,a):return 0
            def close(self):pass
        class Feedback:
            def require(self,*a):return {'temperature':20}
            def stop_all(self):events.append('disable');report['reset_confirmed']=list(range(1,8))
        class Executor:
            def __init__(self,p,d,r,records):
                nonlocal report
                report=r;self.s=Feedback();self.transport=type('Transport',(),{})()
            def prepare(self,j):events.append('enable');report.update(owned=True,enable_attempted=True)
            def settle_arm(self,*a,**kw):events.append('settle');return {'actual_motor_rad':[0.]*6}
            def release_host_hold(self):pass
            def event(self,name,**kw):events.append(name)
            def execute_segments(self,s):
                events.append('move')
                if fail:raise RuntimeError('mid-motion fault')
        class Writer:
            def __init__(self,*a):pass
            def close(self):pass
        class Camera:
            def __init__(self,*a):self.requests=queue.Queue()
        job=dict(start_rad=[0.]*7,start_motor_rad=[0.]*7,tasks=[dict(name='hover',goal_rad=[0.]*7,segments=[{}])],recoveries={'hover':{'segments':[{}]}})
        jobs=[dict(job,motor_lower_rad=[0.]*7,motor_upper_rad=[1.]*7,chain=dict(index=k,count=chain,support_rad=[0.]*7)) for k in range(chain)] if chain>1 else [job]
        answer=dict(ok=True,reply=dict(action='CONVERGED',pixel_error=2.)) if vision_ok else dict(ok=False,error='vision timeout')
        with tempfile.TemporaryDirectory() as root, patch.object(m.mp,'get_context',return_value=Context()), patch.object(m,'CanDevice',return_value=None), patch.object(m,'TransferDevice',return_value=Device()), patch.object(m,'GraspExecutor',Executor), patch.object(m,'WaitStateWriter',Writer), patch.object(m,'CameraWorker',Camera), patch.object(m,'stationary_capture',return_value=dict(status='CAPTURED',frames=[dict(sha256='a',headers={'X-Camera-Session':'s'})]*3)), patch.object(m,'bounded_reply',return_value=answer,side_effect=[dict(ok=False,error="<HTTPError 422: 'Unprocessable Entity'>"),answer] if retry else None), patch.object(m,'retain_until_supported',side_effect=lambda *a:events.append('support_confirmation')), patch.object(m,'servo_nut',side_effect=tail) if tail else patch.object(m,'servo_nut',m.servo_nut):
            result=m.execute(jobs,None,'unused',Path(root)/'run',[([],{'hover':[]})]*len(jobs))
        return events,result
    def test_normal_return_auto_disable(self):
        e,r=self.run_case();self.assertLess(e.index('worker_start'),e.index('enable'));self.assertLess(e.index('VERIFIED_START_RETURN'),e.index('disable'));self.assertNotIn('support_confirmation',e)
    def test_vision_timeout_returns_before_disable(self):
        e,r=self.run_case(vision_ok=False);self.assertEqual(e.count('move'),2);self.assertEqual(r['status'],'RETURN_VISION_REJECTED');self.assertLess(e.index('VERIFIED_START_RETURN'),e.index('disable'))
    def test_one_vision_rejection_retries_without_extra_motion(self):
        e,r=self.run_case(retry=True);self.assertIn('VISION_RETRY_SAME_POSE',e);self.assertEqual(e.count('move'),2);self.assertEqual(r['status'],'RETURN_CONVERGED');self.assertNotIn('support_confirmation',e)
    def test_motion_fault_needs_support_before_disable(self):
        e,r=self.run_case(fail=True);self.assertNotIn('VERIFIED_START_RETURN',e);self.assertLess(e.index('support_confirmation'),e.index('disable'))

    def test_chain_returns_only_after_last_nut(self):
        calls=[]
        def tail(index,*a):
            calls.append(index);return 'COMPLETED_ONLINE_GRASP_MOTION_VERIFY_IMAGES'
        e,r=self.run_case(chain=3,tail=tail)
        self.assertEqual(calls,[0,1,2]);self.assertEqual(e.count('enable'),1);self.assertEqual(e.count('CHAIN_NEXT_NUT'),2)
        self.assertEqual(e.count('disable'),1);self.assertEqual(r['status'],'COMPLETED_ONLINE_GRASP_MOTION_VERIFY_IMAGES');self.assertEqual(len(r['nut_statuses']),3)
        self.assertLess(e.index('CHAIN_NEXT_NUT'),e.index('VERIFIED_START_RETURN'));self.assertNotIn('support_confirmation',e)
    def test_chain_stops_when_a_nut_returns_to_support(self):
        calls=[]
        def tail(index,*a):
            calls.append(index);return 'COMPLETED_ONLINE_GRASP_MOTION_VERIFY_IMAGES' if index==0 else 'RETURN_VISION_REJECTED'
        e,r=self.run_case(chain=3,tail=tail)
        self.assertEqual(calls,[0,1]);self.assertEqual(r['status'],'RETURN_VISION_REJECTED');self.assertEqual(r['nut_statuses'][-1],'RETURN_VISION_REJECTED')
        self.assertLess(e.index('VERIFIED_START_RETURN'),e.index('disable'))
    def test_chain_fault_in_later_nut_needs_support(self):
        def tail(index,*a):
            if index==1:raise RuntimeError('mid-motion fault')
            return 'COMPLETED_ONLINE_GRASP_MOTION_VERIFY_IMAGES'
        e,r=self.run_case(chain=2,tail=tail)
        self.assertNotIn('VERIFIED_START_RETURN',e);self.assertLess(e.index('support_confirmation'),e.index('disable'))

class NoHeavyWorkWhileEnabled(unittest.TestCase):
    def test_digest_is_computed_before_enable_and_never_in_the_servo_loop(self):
        # job_digest takes ~0.3 s on the board; the controller lease is 0.1 s.
        order=[]
        real=m.job_digest
        def digest(job):
            order.append('digest');return real(job)
        case=Lifecycle('test_normal_return_auto_disable')
        with patch.object(m,'job_digest',digest):
            events,result=case.run_case()
        self.assertEqual(order,['digest'])
        self.assertLess(events.index('worker_start'),events.index('enable'))
        with patch.object(m,'job_digest',digest):
            order.clear();case.run_case(chain=3,tail=lambda *a:'COMPLETED_ONLINE_GRASP_MOTION_VERIFY_IMAGES')
        self.assertEqual(order,['digest']*3)

class ChainBinding(unittest.TestCase):
    def jobs(self):
        exit_pose=[.1]*7
        first=dict(profile_id='p',start_rad=[0.]*7,chain=dict(index=0,count=2,support_rad=[0.]*7),visual_servo=dict(integrated_grasp=True,endpoint='http://h:1/'),targets=[dict(target_id='screw_0')],tasks=[dict(name='screw_0_exit',goal_rad=exit_pose)])
        second=dict(profile_id='p',start_rad=[.1+1e-12]*7,chain=dict(index=1,count=2,support_rad=[0.]*7),visual_servo=dict(integrated_grasp=True,endpoint='http://h:2/'),targets=[dict(target_id='nut_0')],tasks=[dict(name='nut_0_exit',goal_rad=[0.]*7)])
        return first,second
    def test_bound_chain_is_accepted(self):
        m.check_chain(list(self.jobs()))
    def test_a_screw_then_a_nut_chain_is_bound_by_each_target_id(self):
        a,b=self.jobs();m.check_chain([a,b])
        a['tasks'][0]['name']='nut_0_exit'  # exit task must belong to the screw that precedes
        with self.assertRaisesRegex(ValueError,'previous exit'):m.check_chain([a,b])
    def test_start_must_equal_previous_exit(self):
        a,b=self.jobs();b['start_rad']=[.2]*7
        with self.assertRaisesRegex(ValueError,'previous exit'):m.check_chain([a,b])
    def test_each_job_needs_its_own_endpoint(self):
        a,b=self.jobs();b['visual_servo']['endpoint']=a['visual_servo']['endpoint']
        with self.assertRaisesRegex(ValueError,'endpoint'):m.check_chain([a,b])
    def test_support_pose_must_be_shared(self):
        a,b=self.jobs();b['chain']['support_rad']=[.05]*7
        with self.assertRaisesRegex(ValueError,'one bound sequence'):m.check_chain([a,b])

class JawHoldsPart(unittest.TestCase):
    """Measured 2026-10-29: close command 1.6 rad, current limit 0.65 A."""
    def test_recorded_cases(self):
        for actual,iqf,held in [(1.5718,.648,True),(1.5852,.651,True),(1.5997,.325,False),(1.5998,.02,False),(1.6,.0,False)]:
            self.assertEqual(m.jaw_holds_part(actual,1.6,iqf),held,(actual,iqf))
    def test_stall_needs_both_conditions(self):
        self.assertFalse(m.jaw_holds_part(1.585,1.6,.3))     # short of the command but low current
        self.assertFalse(m.jaw_holds_part(1.5965,1.6,.65))   # full current but at the command
        self.assertTrue(m.jaw_holds_part(1.50,1.6,.0))       # M12 behaviour unchanged


class EmptyGraspRecovery(unittest.TestCase):
    """PL offset zero precedes physical settling; no return before the original gate."""
    def run_recovery(self, stuck=False, continuous=True):
        import tempfile
        from types import SimpleNamespace
        from unittest.mock import Mock
        from calibration_motion import CalibrationExecutor
        clock=[0.];events=[]
        goal=[0.]*6+[1.66]
        branch=dict(start_rad=goal,goal_rad=[0.]*6+[1.12],segments=[dict(name='reviewed_return')])
        candidate=dict(tasks=[dict(name='departure'),dict(name='nut_0_close',goal_rad=goal,segments=[dict(name='close')])],
                       recoveries={'nut_0_close':branch})
        records=SimpleNamespace(recoveries={'nut_0_close':['encoded_return']})
        e=SimpleNamespace(state='HOLDING',p=SimpleNamespace(to_motor=lambda q:q),
            d=SimpleNamespace(read=lambda a:{0x418:0,0x5a8:127}[a]),
            transport=SimpleNamespace(status=lambda:dict(fault=0)),event=lambda event,**k:events.append(event))
        def position(mid):return .01 if mid in (2,3) and (stuck or clock[0]<.3) else 0.
        def read(mid,address):
            if mid<7:return position(mid)
            return 1.660193 if address==0x7019 else .31668
        e.s=SimpleNamespace(require=lambda mid,mode:dict(position=position(mid)),read=read)
        def pump(seconds):clock[0]+=seconds
        e.pump=pump
        def settle(q,seconds=.3):
            events.append('physical_settle')
            return CalibrationExecutor.settle(e,q,seconds)
        e.settle_arm=settle
        e.execute_segments=lambda seq:events.append(seq[0]['name'])
        channel=SimpleNamespace(disable=lambda d:events.append('offset_disable'),wait_zero=lambda e:events.append('offset_zero'))
        report={}
        with tempfile.TemporaryDirectory() as out, patch.object(m.time,'monotonic',side_effect=lambda:clock[0]):
            try:
                status=m.execute_grasp_tail(e,dict(candidate=candidate,records=[],full_records=records),out,report,
                                            continuous=channel if continuous else None)
            except RuntimeError as error:
                return events,report,str(error)
        return events,report,status

    def test_empty_grasp_waits_for_physical_joint_settling(self):
        events,report,status=self.run_recovery()
        self.assertEqual(status,'TASK_FAILED_RETURNED_SUPPORTED')
        self.assertLess(events.index('offset_zero'),events.index('physical_settle'))
        self.assertLess(events.index('physical_settle'),events.index('reviewed_return'))
        self.assertTrue(report['automatic_supported_return_completed'])
        self.assertEqual(report['recovery_start_check']['tolerance_rad'],.004)

    def test_persistent_displacement_does_not_launch_return(self):
        events,report,error=self.run_recovery(stuck=True)
        self.assertIn('did not settle',error)
        self.assertNotIn('reviewed_return',events)
        self.assertNotIn('automatic_supported_return_completed',report)

    def test_noncontinuous_return_keeps_the_original_strict_gate(self):
        events,report,error=self.run_recovery(continuous=False)
        self.assertIn('Actual arm pose',error)
        self.assertNotIn('reviewed_return',events)


if __name__=='__main__':unittest.main()


class GraspHeightByClass(unittest.TestCase):
    def test_screw_jobs_are_gripped_higher_nuts_and_mixed_batches_keep_the_reviewed_height(self):
        from continuous_transfer import expected_grasp_height_offset_m,SCREW_GRASP_RAISE_M
        self.assertAlmostEqual(expected_grasp_height_offset_m(['nut']),.017)
        self.assertAlmostEqual(expected_grasp_height_offset_m(['screw']),.017+SCREW_GRASP_RAISE_M)
        self.assertAlmostEqual(expected_grasp_height_offset_m(['screw','nut']),.017)
        self.assertAlmostEqual(expected_grasp_height_offset_m([]),.017)
    def test_raise_is_a_few_millimetres_not_more(self):
        from continuous_transfer import SCREW_GRASP_RAISE_M
        self.assertTrue(.002<=SCREW_GRASP_RAISE_M<=.006)
