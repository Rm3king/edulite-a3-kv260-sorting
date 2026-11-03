"""Real Qt controls, isolated config, no camera/robot/model processes."""
import json
import os
import tempfile
import unittest
from argparse import Namespace
from pathlib import Path

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
from PyQt6.QtWidgets import QApplication
from workcell.desktop import Window
from workcell.inventory_goal import decode_model_output


class GoalUITests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.qt = QApplication.instance() or QApplication([])

    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        root = Path(self.tmp.name)
        (root/'integration/config').mkdir(parents=True)
        (root/'reports').mkdir()
        (root/'integration/config/live_workflow.json').write_text(json.dumps(
            {'inventory_goal_execution_enabled': True}))
        self.w = Window(Namespace(app_root=str(root), replay=None, scene=None,
                                  live=True, prewarm_model=False))
        self.w.live_timer.stop()
        self.w.timer.stop()
        self.calls = []
        self.w.workflow_request = self.calls.append
        self.goal = decode_model_output({'action': 'set_count', 'tray': 'B', 'quantity': 2})

    def tearDown(self):
        self.w.live_busy = self.w.live_owner = False
        self.w.close()
        self.tmp.cleanup()

    def test_understanding_only_previews_and_requires_execution_click(self):
        self.w.apply_goal_intent(self.goal)
        self.assertEqual([v['op'] for v in self.calls], ['preview_goal'])
        self.assertFalse(self.w.live_auto_execute)

    def test_edit_revokes_pending_launch_but_keeps_task_verification_goal(self):
        self.w.apply_goal_intent(self.goal)
        self.w.live_busy = False
        self.w.prepare_live(True)
        self.assertTrue(self.w.live_auto_execute)
        self.w.input.setText('B盒最终有三颗螺丝')
        self.assertFalse(self.w.live_auto_execute)
        self.assertIsNone(self.w.live_goal)
        self.assertEqual(self.w.live_task_goal, self.goal)
        self.assertNotIn('execute', [v['op'] for v in self.calls])

    def test_old_model_result_cannot_apply_after_instruction_edit(self):
        self.w.input.setText('让B盒最终有两颗螺丝')
        result = dict(event='MODEL_RESULT', instruction='让B盒最终有三颗螺丝', intent=self.goal)
        class Reply:
            def readAllStandardOutput(self):
                return (json.dumps(result)+'\n').encode()
        self.w.model = Reply()
        self.w.model_buffer = b''
        self.w.model_output()
        self.w.model = None
        self.assertIsNone(self.w.live_goal)
        self.assertEqual(self.calls, [])

    def test_summary_before_owner_exit_keeps_polling_before_verification(self):
        self.w.live_owner = True
        self.w.live_directory = '/test/task'
        self.w.live_task_goal = self.goal
        state = {'alive': True, 'summary': {'status': 'COMPLETED_TRANSFER_MOTION_VERIFY_IMAGES'}}
        class Reply:
            def readAllStandardOutput(self):
                return (json.dumps(dict(event='WORKFLOW_RESULT', op='poll', result=state))+'\n').encode()
        self.w.workflow = Reply()
        self.w.workflow_output()
        self.assertTrue(self.w.live_owner)
        self.assertEqual(self.calls, [])
        state['alive'] = False
        self.w.workflow_output()
        self.w.workflow = None
        self.assertFalse(self.w.live_owner)
        self.assertEqual(self.calls, [dict(op='verify_goal', directory='/test/task')])


if __name__ == '__main__':
    unittest.main()
