"""Deployment wrapper checks that never start ROS or open hardware."""
import pathlib
import subprocess
import tempfile
import unittest

ROOT = pathlib.Path(__file__).resolve().parent


class EntryTests(unittest.TestCase):
    def test_help_does_not_need_ros_or_config(self):
        for name in ('servo-range', 'servo-status', 'servo-control'):
            with tempfile.TemporaryDirectory() as directory:
                entry = pathlib.Path(directory) / name
                entry.symlink_to(ROOT / 'entrypoint.sh')
                result = subprocess.run(['bash', str(entry), '--help'],
                                        text=True, capture_output=True)
                self.assertEqual(result.returncode, 0, result.stderr)
                self.assertIn('10', result.stdout)

    def test_missing_release_fails_before_hardware(self):
        with tempfile.TemporaryDirectory() as directory:
            entry = pathlib.Path(directory) / 'servo-status'
            entry.symlink_to(ROOT / 'entrypoint.sh')
            result = subprocess.run(['bash', str(entry), 'check'],
                                    text=True, capture_output=True)
            self.assertNotEqual(result.returncode, 0)
            self.assertIn('servo-current', result.stderr)


if __name__ == '__main__':
    unittest.main()
