"""Reject wrong/native-emulated platform labels before CI tests start."""
import json
from pathlib import Path
import subprocess
import tempfile
import unittest

from check_ci_platform import check


class PlatformTests(unittest.TestCase):
    def exercise(self, expected='arm64', machine='aarch64', system='Linux', image_platform='linux/arm64', image=True, failure=0):
        calls = []
        def run(args, **kwargs):
            calls.append(args)
            return subprocess.CompletedProcess(args, failure, image_platform, 'inspect unavailable' if failure else '')
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / 'runner-platform.json'
            report = check(expected, path, 'fixture:sha' if image else None, system, machine, run)
            self.assertEqual(json.loads(path.read_text()), report)
        return report, calls

    def test_native_arm64_image_passes(self):
        report, calls = self.exercise()
        self.assertEqual(report['errors'], [])
        self.assertEqual(calls[0], ['docker', 'image', 'inspect', '--format', '{{.Os}}/{{.Architecture}}', 'fixture:sha'])

    def test_native_amd64_image_passes(self):
        report, _ = self.exercise(expected='amd64', machine='x86_64', image_platform='linux/amd64')
        self.assertEqual(report['errors'], [])

    def test_arm_image_on_x86_host_rejected_before_inspection(self):
        report, calls = self.exercise(machine='x86_64')
        self.assertTrue(report['errors'])
        self.assertEqual(calls, [])

    def test_wrong_image_on_native_arm_rejected(self):
        report, _ = self.exercise(image_platform='linux/amd64')
        self.assertTrue(report['errors'])

    def test_non_linux_host_rejected(self):
        report, calls = self.exercise(system='Windows')
        self.assertTrue(report['errors'])
        self.assertEqual(calls, [])

    def test_inspection_failure_records_error(self):
        report, _ = self.exercise(failure=1)
        self.assertTrue(report['errors'])

    def test_host_check_does_not_need_an_image(self):
        report, calls = self.exercise(image=False)
        self.assertEqual(report['errors'], [])
        self.assertEqual(calls, [])


if __name__ == '__main__':
    unittest.main()
