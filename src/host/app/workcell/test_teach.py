"""Teach mode: nothing starts without the operator's confirmation and an idle board; recordings are stored; finish asks freedrive for its own stop."""
import json
import queue
import subprocess
import tempfile
import time as real_time
import unittest
from pathlib import Path
from unittest import mock

from . import teach as T

REAL_SLEEP = real_time.sleep   # mock.patch('time.sleep') replaces the function on the shared module

PROFILE = dict(motor_directions=[-1, 1, -1, 1, -1, 1, 1], motor_offsets_rad=[0, 0, .265, 0, 0, 0, 0])


class Stream:
    def __init__(self):
        self.q = queue.Queue()
        self.stdout = iter(self.q.get, None)


class Board:
    """A fake board behind the ssh helper."""
    def __init__(self, owners='', control=0, fault=0):
        self.commands, self.owners, self.control, self.fault = [], owners, control, fault
        self.alive, self.elapsed, self.stream, self.scripts = False, 0., None, []
        self.samples = [[i / 10, [i * .01, 0, .265, 0, 0, 0, 1.12]] for i in range(0, 100)]

    def popen(self, args, **kw):
        self.scripts.append(args[2]);self.stream = Stream()
        for i in range(1, 40):
            live = dict(phase='DRAG', elapsed_s=i * .1, actual_motor_rad=[i * .01, 0, .265, 0, 0, 0, 1.12], torque_saturated=[False] * 6, finish_pending=False)
            self.stream.q.put(json.dumps(live))
        return self.stream

    def __call__(self, args, **kw):
        c = args[2];self.commands.append(c);out = ''
        if 'owner.json' in c:out = self.owners
        elif c.startswith('pgrep') and 'echo yes' in c:out = 'yes\n' if self.alive and False else 'no\n'
        elif 'canctl.py status' in c:out = json.dumps(dict(control=self.control, fault=self.fault))
        elif 'initialize_disabled_csp' in c:out = 'BASELINE ' + json.dumps(dict(before={str(i): 0 for i in range(1, 7)}, after={str(i): 5 for i in range(1, 7)})) + '\n'
        elif 'nohup setsid' in c:self.alive = True;out = 'launched\n'
        elif c.startswith('cat') and c.endswith('profile.json'):out = json.dumps(PROFILE)
        elif 'SUPPORTED_DISABLE' in c:self.alive = False;self.stream.q.put('__ENDED__');out = 'signalled\n'
        elif 'out.json' in c:
            out = json.dumps(dict(status='FAIL', error="KeyboardInterrupt('operator interrupt')", reset=[1, 2, 3, 4, 5, 6, 7], stop_errors={},
                                  controller=dict(control=0, fault=0), samples=self.samples))
        return subprocess.CompletedProcess(args, 0, stdout=out)


class Clock:
    def __init__(self):self.t = 0.
    def __call__(self):self.t += .05;return self.t


class TeachTests(unittest.TestCase):
    def make(self, **kw):
        board = Board(**kw)
        teach = T.Teach(tempfile.mkdtemp(), 'helper.sh', run=board, clock=Clock(), popen=board.popen)
        return teach, board

    def test_confirmation_and_busy_are_required(self):
        teach, board = self.make()
        with self.assertRaises(T.TeachError):teach.start('')
        with self.assertRaises(T.TeachError):teach.start(T.CONFIRMATION, busy=True)
        self.assertFalse([c for c in board.commands if 'nohup' in c])

    def test_live_owner_or_fault_blocks_start(self):
        for kw in (dict(owners='four_integrated_X\n'), dict(fault=8), dict(control=7)):
            teach, board = self.make(**kw)
            with self.assertRaises(T.TeachError):teach.start(T.CONFIRMATION)
            self.assertFalse([c for c in board.commands if 'nohup' in c], kw)

    def test_start_record_and_finish(self):
        teach, board = self.make()
        with mock.patch('workcell.teach.time.sleep', lambda s: REAL_SLEEP(.002)):
            snap = teach.start(T.CONFIRMATION)
            self.assertEqual(snap['state'], 'ACTIVE');self.assertEqual(snap['live']['phase'], 'DRAG')
            self.assertLess([i for i, c in enumerate(board.commands) if 'initialize_disabled_csp' in c][0], [i for i, c in enumerate(board.commands) if 'nohup' in c][0])
            launch = [c for c in board.commands if 'nohup' in c][0]
            self.assertIn('freedrive.py', launch);self.assertIn('--hands-supporting-attended', launch);self.assertNotIn('kill', launch)
            pose = teach.record_pose('左盘槽1')
            self.assertEqual(pose['name'], '左盘槽1');self.assertEqual(len(pose['joint_rad']), 7)
            self.assertAlmostEqual(pose['joint_rad'][2], (pose['motor_rad'][2] - .265) / -1)
            teach.mark('start', '示教轨迹')
            teach.last = dict(teach.last, elapsed_s=teach.last['elapsed_s'] + 1.5)   # the fake stream has delivered everything already
            teach.mark('stop')
            final = teach.finish()
        self.assertTrue(final['disabled'])
        self.assertEqual(len(teach.trajectories), 1)
        self.assertGreater(len(teach.trajectories[0]['samples']), 3)
        stop = [c for c in board.commands if 'SUPPORTED_DISABLE' in c]
        self.assertEqual(len(stop), 1)
        self.assertIn('kill -TERM $pid', stop[0]);self.assertNotIn('pkill', stop[0]);self.assertNotIn('finish_supported', ' '.join(board.commands))
        self.assertEqual(final['status'], 'OPERATOR_STOP')
        saved = json.loads((Path(teach.app) / 'reports/teach' / teach.run_id / 'teach.json').read_text())
        self.assertEqual(saved['poses'][0]['name'], '左盘槽1')

    def test_pgrep_pattern_cannot_match_the_remote_shell_itself(self):
        # 2026-11-01: 'pgrep -f "freedrive.py --profile"' inside an ssh command line matched its own shell: the board always looked busy
        teach, board = self.make()
        with mock.patch('workcell.teach.time.sleep', lambda s: REAL_SLEEP(.002)):
            teach.start(T.CONFIRMATION);teach.poll()
        for command in board.commands:
            if 'pgrep' in command:
                self.assertIn('[f]reedrive.py', command)
                self.assertNotIn('"freedrive.py', command)

    def test_finish_without_start_is_refused(self):
        teach, board = self.make()
        with self.assertRaises(T.TeachError):teach.finish()
        self.assertFalse([c for c in board.commands if 'SUPPORTED_DISABLE' in c])


if __name__ == '__main__':
    unittest.main()
