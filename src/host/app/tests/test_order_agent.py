import unittest
from workcell.order_agent import OrderAgent, Events
from workcell.tasks import result

TEMPS = {m: 30. for m in range(1, 8)}


def make(outcomes, placed=True, temps=TEMPS):
    calls = []
    def run_batch(parts, slots, skip_thermal=False):
        calls.append((list(parts), list(slots)))
        o = dict(outcomes.pop(0));o.setdefault('statuses', ['COMPLETED_X'] * len(parts));o.setdefault('fault', False);o.setdefault('reduced_to', None);o.setdefault('returncode', 0);o.setdefault('root', 'r')
        return o
    def verify(done):
        return dict(parts=[dict(part_class=d['part_class'], slot=d['slot'], status='PLACED' if placed else 'MISSING') for d in done])
    agent = OrderAgent('/nonexistent', events=Events(), observe=lambda: dict(objects=[]), temperatures=lambda: dict(temps), run_batch=run_batch, verify=verify, sleep=lambda s: None)
    return agent, calls


def plan_for(parts):
    from workcell.order_planner import batches
    return dict(parts=parts, batches=batches(TEMPS, parts), task=None)


class Agent(unittest.TestCase):
    def test_alignment_without_residual_keeps_reading_completion_and_disable_proof(self):
        import io, json, tempfile
        from pathlib import Path
        from unittest.mock import Mock, patch
        with tempfile.TemporaryDirectory() as tmp:
            summary = dict(nut_statuses=['COMPLETED_X', 'RETURN_VISION_LOST', 'NOT_ATTEMPTED'],
                           reset_confirmed=list(range(1, 8)), stop_errors={}, final_active_mask=0,
                           final_controller=dict(control=0, fault=0, queue_counts=0))
            (Path(tmp) / 'completed.json').write_text(json.dumps(dict(results=[dict(summary=summary)])))
            lines = []
            for part, error, status in [(0, 2.5, 'CONVERGED'), (1, None, 'RETURN_VISION_LOST')]:
                lines.append('ALIGNMENT ' + json.dumps(dict(part=part, mode='continuous', seconds=1.6,
                                                            error_px=error, status=status)))
            lines += ["END_SESSION 0 ['COMPLETED_X', 'RETURN_VISION_LOST', 'NOT_ATTEMPTED']", 'BATCH_FINISHED ' + tmp]
            process = Mock(stdout=io.StringIO('\n'.join(lines)))
            process.wait.return_value = 0
            agent = OrderAgent('/nonexistent')
            with patch('workcell.order_agent.subprocess.Popen', return_value=process):
                outcome = agent.run_batch(['screw', 'nut', 'screw_m6'], [0, 1, 2])
            process.wait.assert_called_once()
            self.assertEqual(outcome['root'], tmp)
            self.assertEqual(outcome['statuses'], summary['nut_statuses'])
            self.assertTrue(outcome['disabled_confirmed'])
            self.assertIn('2.5 px', agent.alignments[0]['text'])
            self.assertIn('残差未取得', agent.alignments[1]['text'].replace(' ', ''))

    def test_console_exception_leaves_running_state_and_preserves_fault(self):
        from unittest.mock import Mock
        from workcell.console import Console
        for fault, expected in [(False, 'ERROR'), (True, 'FAULT_HELD')]:
            with self.subTest(fault=fault):
                console = Console('/nonexistent')
                console.busy, console.fault, console.agent.state = True, fault, 'RUNNING'
                console.agent.plan = Mock(side_effect=RuntimeError('display failed'))
                console._run('test', False)
                self.assertFalse(console.state()['busy'])
                self.assertEqual(console.state()['state'], expected)
                self.assertEqual(console.state()['fault'], fault)

    def test_not_ready_refuses(self):
        agent, calls = make([{}])
        with self.assertRaises(ValueError):agent.execute(plan_for(['screw']), operator_ready=False)
        self.assertEqual(calls, [])

    def test_all_placed(self):
        agent, calls = make([{}])
        report = agent.execute(plan_for(['screw', 'nut']), True)
        self.assertEqual((len(report['done']), report['state']), (2, 'DONE'));self.assertEqual(len(calls), 1)

    def test_transient_failure_is_retried_then_succeeds(self):
        agent, calls = make([dict(statuses=['RETURN_CAPTURE_REJECTED']), {}])
        report = agent.execute(plan_for(['screw']), True)
        self.assertEqual(len(report['done']), 1);self.assertEqual(len(calls), 2)

    def test_gives_up_after_retries(self):
        agent, calls = make([dict(statuses=['RETURN_VISION_REJECTED'])] * 5)
        report = agent.execute(plan_for(['screw']), True)
        self.assertEqual((len(report['done']), len(report['failed'])), (0, 1));self.assertEqual(len(calls), 3)

    def test_fault_stops_everything_and_asks_the_operator(self):
        agent, calls = make([dict(fault=True, statuses=[])] + [{}])
        report = agent.execute(plan_for(['screw', 'screw']), True)
        self.assertEqual(report['state'], 'FAULT_HELD');self.assertEqual(len(calls), 1)
        self.assertTrue(any(e['kind'] == 'stop' for e in agent.events.items))

    def test_later_fault_preserves_verified_placements_and_records_failed_attempt(self):
        agent, calls = make([dict(fault=True, statuses=['COMPLETED_X']*3,
            disabled_confirmed=True, task_error="TaskOutcomeError('Empty closure: grasp not confirmed')",
            error="RuntimeError('Actual arm pose differs from reviewed return start')")])
        agent.worker=Review.Worker()
        report=agent.execute(dict(plan_for(['screw','nut','screw_m6','nut_m6']),skip_cooling=True),True)
        self.assertEqual(len(calls),1)
        self.assertEqual(report['state'],'STOPPED')
        self.assertEqual([p for p,s in report['done']],['screw','nut','screw_m6'])
        self.assertEqual(report['failed'][0][0],'nut_m6')
        self.assertIn('空夹',report['failed'][0][2])
        self.assertEqual(report['pending'],[])
        self.assertEqual(report['review']['source'],'records')
        self.assertEqual(agent.worker.asked,[])
        self.assertTrue(any(e['kind']=='disabled' for e in agent.events.items))

    def test_completed_motion_without_slot_evidence_is_not_counted(self):
        agent,calls=make([dict(fault=True,statuses=['COMPLETED_X'],disabled_confirmed=True)],placed=False)
        report=agent.execute(plan_for(['screw','nut']),True)
        self.assertEqual(report['done'],[])
        self.assertEqual(report['failed'][0][0],'screw')
        self.assertEqual([x['part'] for x in report['pending']],['nut'])
        self.assertEqual(len(calls),1)

    def test_camera_failure_does_not_hide_a_held_arm(self):
        agent,calls=make([dict(fault=True,statuses=['COMPLETED_X'])])
        def unavailable(done):raise RuntimeError('camera offline')
        agent._verify=unavailable
        report=agent.execute(plan_for(['screw','nut']),True)
        self.assertEqual(report['state'],'FAULT_HELD')
        self.assertEqual(report['done'],[])
        self.assertFalse(any(e['kind']=='disabled' for e in agent.events.items))
        self.assertEqual(len(calls),1)

    def test_disabled_event_clears_current_console_alarm(self):
        from workcell.console import Console
        console=Console('/nonexistent')
        console.events('fault','held')
        self.assertTrue(console.state()['fault'])
        console.events('operator','marker written')
        self.assertTrue(console.state()['fault'])
        console.events('disabled','confirmed')
        self.assertFalse(console.state()['fault'])

    def test_disable_proof_requires_all_resets_and_zero_outputs(self):
        from workcell.order_agent import disabled_confirmed
        good=dict(reset_confirmed=list(range(1,8)),stop_errors={},final_active_mask=0,
                  final_controller=dict(control=0,fault=0,queue_counts=0))
        self.assertTrue(disabled_confirmed(good))
        for changed in [dict(reset_confirmed=list(range(1,7))),dict(stop_errors={'7':'timeout'}),
                        dict(final_active_mask=1),dict(final_controller=dict(control=7,fault=0,queue_counts=0)),
                        dict(final_controller=dict(control=0,fault=8,queue_counts=0)),dict(final_controller={})]:
            self.assertFalse(disabled_confirmed(dict(good,**changed)))

    def test_completion_uses_the_bound_batch_not_a_newer_directory(self):
        import json,tempfile
        from pathlib import Path
        from workcell.order_agent import read_batch_completion
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp)/'bound';root.mkdir()
            summary=dict(nut_statuses=['COMPLETED_X'],error='failed',task_outcome_error='empty',
                         reset_confirmed=list(range(1,8)),stop_errors={},final_active_mask=0,
                         final_controller=dict(control=0,fault=0,queue_counts=0))
            (root/'completed.json').write_text(json.dumps(dict(results=[dict(summary=summary)])))
            outcome=read_batch_completion(dict(root=str(root),fault=False,statuses=[]))
            self.assertTrue(outcome['disabled_confirmed']);self.assertTrue(outcome['fault'])
            self.assertEqual(outcome['statuses'],['COMPLETED_X'])
            missing=read_batch_completion(dict(root=str(Path(tmp)/'missing'),fault=True,statuses=[]))
            self.assertNotIn('disabled_confirmed',missing)

    def test_reduced_batch_continues_with_the_rest(self):
        agent, calls = make([dict(reduced_to=1, statuses=['COMPLETED_X']), {}])
        report = agent.execute(plan_for(['screw', 'nut']), True)
        self.assertEqual(len(report['done']), 2);self.assertEqual(len(calls), 2)

    def test_not_seen_in_the_slot_is_never_picked_again(self):
        agent, calls = make([{}], placed=False)
        report = agent.execute(plan_for(['screw']), True)
        self.assertEqual((len(report['done']), len(report['failed'])), (0, 1));self.assertEqual(len(calls), 1)

    def test_abort_before_enable_is_retried(self):
        agent, calls = make([dict(statuses=[], returncode=1, error="ValueError('Scene changed before enable')"), {}])
        report = agent.execute(plan_for(['screw']), True)
        self.assertEqual(len(report['done']), 1);self.assertEqual(len(calls), 2)

    def test_unknown_error_is_not_retried(self):
        agent, calls = make([dict(statuses=[], returncode=1, error='RuntimeError("something else")'), {}])
        report = agent.execute(plan_for(['screw']), True)
        self.assertEqual(report['state'], 'ERROR');self.assertEqual(len(calls), 1)

    def test_geometry_failure_is_explained_without_model_guessing(self):
        error = '/very/long/report/path/' * 20 + '\\nValueError: Unlocalised nut_m6 (geometry rejected) 136 px from a target/slot: move or remove it'
        agent, calls = make([dict(statuses=[], returncode=1, error=error)])
        worker = Review.Worker();agent.worker = worker
        report = agent.execute(plan_for(['screw','nut','screw_m6','nut_m6']), True)
        self.assertEqual(len(calls),1)
        self.assertIn('小螺母', report['stop_reason'])
        self.assertIn('136', report['review']['summary'])
        self.assertEqual(report['review']['source'],'records')
        self.assertEqual(worker.asked,[])
        event=next(e for e in agent.events.items if e['kind']=='stop')
        self.assertEqual(event['raw_error'],error)
        self.assertNotIn('/very/',event['text'])

    def test_skip_cooling_never_waits_and_does_not_retry_a_heat_return(self):
        agent, calls = make([dict(statuses=['COMPLETED_X', 'RETURN_BUDGET_OR_TEMPERATURE'])], temps={m: 50. for m in range(1, 8)})
        plan = dict(parts=['screw', 'screw'], batches=[], skip_cooling=True)
        report = agent.execute(plan, True)
        self.assertEqual((len(report['done']), len(report['failed']), len(calls)), (1, 1, 1))
        self.assertFalse(any('降温中' in e['text'] for e in agent.events.items))

    def test_retry_uses_a_slot_not_already_filled(self):
        agent, calls = make([dict(statuses=['COMPLETED_X', 'RETURN_CAPTURE_REJECTED']), {}])
        agent.execute(plan_for(['screw', 'screw']), True)
        self.assertNotIn(calls[0][1][0], calls[1][1])



class Review(unittest.TestCase):
    """The post-run VLM review (2026-10-31): shown and stored, never raised, never a command."""
    class Worker:
        def __init__(self, fail=False):
            self.fail, self.asked = fail, []
        def review(self, facts, image_files=(), max_new_tokens=180):
            self.asked.append(facts)
            if self.fail:
                raise ValueError('model crashed')
            return dict(event='REVIEW_RESULT', summary='4 件全部放入左盘，图片与记录一致。', generation_s=8.)

    def run_with(self, worker):
        agent, _ = make([{}])
        agent.worker = worker
        seen = []
        agent.events = Events(listener=seen.append)
        report = agent.execute(plan_for(['screw', 'nut', 'screw_m6', 'nut_m6']), True)
        return report, [e for e in seen if e['kind'] == 'review'], agent

    def test_vlm_summary_is_reported(self):
        worker = self.Worker()
        report, reviews, _ = self.run_with(worker)
        self.assertEqual(report['review']['source'], 'vlm')
        self.assertIn('视觉大模型复核', reviews[-1]['text'])
        self.assertIn('小螺母', worker.asked[0])                       # the recorded facts name the M6 parts

    def test_model_failure_falls_back_to_the_records(self):
        report, reviews, _ = self.run_with(self.Worker(fail=True))
        self.assertEqual((report['review']['source'], report['state']), ('records', 'DONE'))
        self.assertIn('error', report['review'])

    def test_without_a_model(self):
        report, reviews, _ = self.run_with(None)
        self.assertEqual(report['review']['summary'], report['text'])


if __name__ == '__main__':unittest.main()
