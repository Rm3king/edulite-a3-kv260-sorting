"""Speculative grasp planning: reuse only for bit-identical q/prefix/correction; never without a real build."""
import json,tempfile,threading,time,unittest
from pathlib import Path
from workcell.wrist_servo_host import Servo


def servo(speculative=True):
    root=Path(tempfile.mkdtemp());(root/'servo_config.json').write_text(json.dumps(dict(
        job_sha256='x',integrated_grasp=True,speculative_grasp=speculative,python_paths=[],launcher='/bin/false')))
    return Servo(root)


MOVE=dict(goal_rad=[.1]*7,segments=[dict(name='a',v=1.),dict(name='b',v=2.)])
REQUEST=dict(executed_prefix=[dict(name='p',v=0.)])


class Speculation(unittest.TestCase):
    def start(self,s,delay=0.):
        calls=[]
        def build(request,folder,timings,spec=None):
            calls.append(dict(request));time.sleep(delay);timings['planning_s']=delay
            return dict(candidate={'built':'speculative'},raw=b'bundle',sha='s')
        s.build_grasp=build;s.start_speculation(REQUEST,MOVE,[.001,.002],s.root);return calls

    def next_request(self,**changes):
        base=dict(q_rad=[.1]*7,cumulative_tool_xy_m=[.001,.002],executed_prefix=REQUEST['executed_prefix']+MOVE['segments'])
        return dict(base,**changes)

    def test_identical_inputs_reuse_and_evidence_is_labelled_as_gate_bound(self):
        s=servo();calls=self.start(s)
        built,why=s.take_speculation(self.next_request())
        self.assertEqual(why,'used');self.assertEqual(built['sha'],'s');self.assertIsNone(s.spec)
        self.assertEqual(calls[0]['pixel_error'],5.)
        self.assertEqual(calls[0]['pixel_error_basis'],'GATE_BOUND_SPECULATIVE_NOT_A_MEASUREMENT')

    def test_any_changed_input_discards(self):
        for change in (dict(q_rad=[.1]*6+[.2]),dict(cumulative_tool_xy_m=[.001,.0021]),dict(executed_prefix=REQUEST['executed_prefix'])):
            s=servo();self.start(s)
            built,why=s.take_speculation(self.next_request(**change))
            self.assertIsNone(built);self.assertEqual(why,'inputs_differ');self.assertIsNone(s.spec)

    def test_first_hover_grasp_is_planned_before_any_move(self):
        s=servo();segments=[dict(name='d',v=3.)];(s.root/'job.json').write_text(json.dumps(dict(tasks=[dict(goal_rad=[.2]*7,segments=segments)])))
        calls=[]
        def build(request,folder,timings,spec=None):
            calls.append(dict(request));return dict(candidate={},raw=b'b',sha='i')
        s.build_grasp=build;s.start_initial_speculation()
        built,why=s.take_speculation(dict(q_rad=[.2]*7,cumulative_tool_xy_m=[0.,0.],executed_prefix=segments))
        self.assertEqual(why,'used');self.assertEqual(built['sha'],'i');self.assertEqual(calls[0]['cumulative_tool_xy_m'],[0.,0.])
        s.start_initial_speculation()
        built,why=s.take_speculation(dict(q_rad=[.2]*7,cumulative_tool_xy_m=[0.,.001],executed_prefix=segments))
        self.assertIsNone(built);self.assertEqual(why,'inputs_differ')

    def test_disabled_by_default_flag(self):
        s=servo(False);calls=self.start(s);time.sleep(.05)
        self.assertEqual(calls,[]);self.assertIsNone(s.spec)

    def test_failed_speculation_falls_back(self):
        s=servo()
        def build(*a,**k):raise ValueError('Camera housing floor clearance')
        s.build_grasp=build;s.start_speculation(REQUEST,MOVE,[.001,.002],s.root)
        built,why=s.take_speculation(self.next_request())
        self.assertIsNone(built);self.assertIn('Camera housing',why)

    def test_cancel_kills_running_planner(self):
        s=servo();started=threading.Event()
        def build(request,folder,timings,spec=None):
            import subprocess
            proc=subprocess.Popen(['sleep','30']);spec.procs.append(proc);started.set();proc.wait();raise RuntimeError('Speculation cancelled')
        s.build_grasp=build;s.start_speculation(REQUEST,MOVE,[.001,.002],s.root);self.assertTrue(started.wait(2))
        proc=s.spec.procs[0];s.cancel_speculation();proc.wait(timeout=3);self.assertIsNotNone(proc.returncode)


if __name__=='__main__':unittest.main()
