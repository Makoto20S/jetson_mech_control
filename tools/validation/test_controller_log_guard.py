"""Recorder fault detection: runtime faults may leave fresh-looking ROS states."""
import importlib.util
from pathlib import Path
import tempfile
import unittest

spec = importlib.util.spec_from_file_location(
    'controller_log_guard', Path(__file__).parents[1]/'bench/controller_log_guard.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class ManagerLogGuardTests(unittest.TestCase):
    def test_runtime_rejection_is_detected_and_latched(self):
        with tempfile.TemporaryDirectory() as root:
            path = Path(root)/'manager.log'
            path.write_text('[INFO] active\n')
            guard = module.ControllerLogGuard(path)
            self.assertIsNone(guard.poll())
            with path.open('a') as out:
                out.write('[ERROR] DM4340 first read failure: feedback_rejected\n')
            failure = guard.poll()
            self.assertIn('feedback_rejected', failure)
            path.write_text('[INFO] active again\n')
            self.assertEqual(guard.poll(), failure)

    def test_partial_marker_and_bounded_read(self):
        with tempfile.TemporaryDirectory() as root:
            path = Path(root)/'manager.log'
            path.write_bytes(b'x'*65536+b'[ERR')
            guard = module.ControllerLogGuard(path)
            self.assertIsNone(guard.poll())
            self.assertEqual(guard.offset, 65536)
            self.assertIsNone(guard.poll())
            with path.open('ab') as out:
                out.write(b'OR] update failed\n')
            self.assertIn('update failed', guard.poll())

    def test_truncation_is_not_silent_recovery(self):
        with tempfile.TemporaryDirectory() as root:
            path = Path(root)/'manager.log'
            path.write_text('[INFO] active\n')
            guard = module.ControllerLogGuard(path)
            self.assertIsNone(guard.poll())
            path.write_bytes(b'')
            self.assertIn('truncated', guard.poll())


if __name__ == '__main__':
    unittest.main()
