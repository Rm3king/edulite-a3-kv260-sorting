"""The support marker goes into the folder of the live owner and nowhere else (2026-11-01: it went into the previous run's folder)."""
import subprocess
import unittest
from unittest import mock

from .order_agent import OrderAgent


def agent_with(live, commands):
    agent = OrderAgent.__new__(OrderAgent)
    agent.last_root = '/x/four_integrated_OLD'
    agent.events = lambda kind, text, **kw: commands.append(('event', text))
    session = mock.Mock(config={'ssh_helper': 'helper.sh'})
    agent._session = lambda: session

    def run(args, **kw):
        commands.append(args[2])
        return subprocess.CompletedProcess(args, 0, stdout=live if OrderAgent.LIVE_OWNERS in args[2] else '')
    return agent, run


class ConfirmSupported(unittest.TestCase):
    def test_marker_goes_to_live_owner_not_last_root(self):
        log = []
        agent, run = agent_with('four_integrated_NEW\n', log)
        with mock.patch('subprocess.run', run):
            agent.confirm_supported()
        touched = [c for c in log if isinstance(c, str) and 'touch' in c]
        self.assertEqual(len(touched), 1)
        self.assertIn('four_integrated_NEW/execution/SUPPORTED_DISABLE', touched[0])
        self.assertNotIn('OLD', touched[0])

    def test_no_live_owner_writes_nothing(self):
        log = []
        agent, run = agent_with('', log)
        with mock.patch('subprocess.run', run):
            with self.assertRaises(RuntimeError):
                agent.confirm_supported()
        self.assertFalse([c for c in log if isinstance(c, str) and 'touch' in c])

    def test_two_live_owners_is_ambiguous(self):
        log = []
        agent, run = agent_with('four_integrated_A\nfour_integrated_B\n', log)
        with mock.patch('subprocess.run', run):
            with self.assertRaises(RuntimeError):
                agent.confirm_supported()
        self.assertFalse([c for c in log if isinstance(c, str) and 'touch' in c])

    def test_explicit_root_must_be_live(self):
        log = []
        agent, run = agent_with('four_integrated_NEW\n', log)
        with mock.patch('subprocess.run', run):
            with self.assertRaises(RuntimeError):
                agent.confirm_supported('/x/four_integrated_OLD')
        self.assertFalse([c for c in log if isinstance(c, str) and 'touch' in c])


if __name__ == '__main__':
    unittest.main()
