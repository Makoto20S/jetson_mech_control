import json
from pathlib import Path
import subprocess
import tempfile
import unittest
from run_stability import run, TESTS


class StabilityTests(unittest.TestCase):
    def test_later_success_does_not_erase_failure(self):
        exits = iter([0, 1, 0])
        def runner(args, **kwargs):
            if '--show-only=json-v1' in args:
                return subprocess.CompletedProcess(args, 0, json.dumps({'tests': [{'name': name} for name in TESTS]}))
            return subprocess.CompletedProcess(args, next(exits))
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            self.assertEqual(run(root / 'build', root / 'results', 3, runner), 1)
            report = json.loads((root / 'results/summary.json').read_text())
            self.assertEqual([r['exit_code'] for r in report['rounds']], [0, 1, 0])
            self.assertEqual(report['passed'], 2)

    def test_missing_target_cannot_pass_as_smaller_suite(self):
        def runner(args, **kwargs):
            return subprocess.CompletedProcess(args, 0, '{"tests": []}')
        with tempfile.TemporaryDirectory() as directory:
            with self.assertRaisesRegex(ValueError, 'incomplete stability suite'):
                run(Path(directory) / 'build', Path(directory) / 'results', 1, runner)


if __name__ == '__main__':
    unittest.main()
