"""A dead detector process is restarted once by Session.camera(); a failed restart gives a clear message, not a bare FileNotFoundError."""
import json
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path
from unittest import mock

from . import live_session


class CameraSelfHeal(unittest.TestCase):
    def setUp(self):
        self.tmp = Path(tempfile.mkdtemp())
        (self.tmp / 'reports').mkdir()
        self.pointer = self.tmp / 'reports/rgbd_surface_preview_active.json'
        self.pointer.write_text(json.dumps(dict(pid=2 ** 22 + 12345, report_dir=str(self.tmp / 'reports/live'))))
        self.session = live_session.Session.__new__(live_session.Session)
        self.session.app = self.tmp

    def test_dead_process_is_restarted_once(self):
        report = self.tmp / 'reports/live'
        # a live process whose command line names the script and the report directory
        child = subprocess.Popen([sys.executable, '-c', 'import time;time.sleep(30)', 'detect_parts_rgbd_live.py', str(report)])
        try:
            def restart(app):
                self.pointer.write_text(json.dumps(dict(pid=child.pid, report_dir=str(report))))
            with mock.patch('workcell.live_startup.ensure_camera', restart) as patched:
                active, folder = self.session.camera()
            self.assertEqual(active['pid'], child.pid)
            self.assertEqual(folder, report)
        finally:
            child.kill();child.wait()

    def test_failed_restart_is_a_clear_error(self):
        with mock.patch('workcell.live_startup.ensure_camera', lambda app: None):
            with self.assertRaises(RuntimeError) as ctx:
                self.session.camera()
        self.assertIn('自动重启失败', str(ctx.exception))


if __name__ == '__main__':
    unittest.main()
