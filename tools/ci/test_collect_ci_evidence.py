"""Failure-path tests for diagnostic retention, without Docker or devices."""
import json
from pathlib import Path
import subprocess
import tempfile
import unittest

from collect_ci_evidence import collect


class EvidenceTests(unittest.TestCase):
    def exercise(self, state=None, missing=(), no_container=False, stop_error=False, sanitizers=False):
        calls = []

        def run(*args):
            calls.append(args)
            code, stdout, stderr = 0, "", ""
            if args[0] == "inspect":
                code = int(no_container)
                stdout = json.dumps(state or {"Running": False, "ExitCode": 1, "OOMKilled": True})
            elif args[0] == "stop":
                code = int(stop_error)
            elif args[0] == "cp":
                code = int(any(part in args[1] for part in missing))
                if not code:
                    folder = Path(args[2])
                    folder.mkdir(parents=True)
                    (folder / "evidence.txt").write_text("preserved", encoding="utf-8")
                else:
                    stderr = "source unavailable"
            return subprocess.CompletedProcess(args, code, stdout, stderr)

        with tempfile.TemporaryDirectory() as directory:
            rc = collect("ci-fixture", directory, run, sanitizers=sanitizers)
            self.assertFalse(json.loads((Path(directory) / 'test-summary.json').read_text())['all_package_reports_available'])
            report = json.loads((Path(directory) / "collection.json").read_text())
            files = [str(p.relative_to(directory)).replace("\\", "/")
                     for p in Path(directory).rglob("evidence.txt")]
        return rc, report, calls, files

    def test_failed_oom_container_retains_logs_and_reports(self):
        rc, report, calls, files = self.exercise()
        self.assertEqual(rc, 0)  # collection success does not change test exit code
        self.assertEqual(report["container_state"]["ExitCode"], 1)
        self.assertTrue(report["container_state"]["OOMKilled"])
        self.assertIn("colcon-log/evidence.txt", files)
        self.assertIn("pty-tests/evidence.txt", files)
        self.assertIn("stability/evidence.txt", files)
        self.assertIn("reports/mech_bringup/test_results/evidence.txt", files)
        self.assertFalse(any(c[0] in ("rm", "exec") for c in calls))

    def test_normal_collection_does_not_request_sanitizer_proof(self):
        _, _, calls, _ = self.exercise()
        self.assertFalse(any('sanitizer-build.json' in ' '.join(call) for call in calls))

    def test_sanitizer_collection_requests_instrumentation_proof(self):
        _, _, calls, _ = self.exercise(sanitizers=True)
        self.assertTrue(any('sanitizer-build.json' in ' '.join(call) for call in calls))

    def test_image_failure_without_container_still_writes_metadata(self):
        rc, report, calls, files = self.exercise(no_container=True)
        self.assertEqual(rc, 0)
        self.assertEqual(report["container_state"], "unavailable")
        self.assertEqual(len(calls), 1)
        self.assertEqual(files, [])

    def test_partial_test_run_preserves_available_evidence(self):
        rc, report, _, files = self.exercise(missing=("test_results", "ci-ros-logs"))
        self.assertEqual(rc, 0)
        self.assertIn("colcon-log/evidence.txt", files)
        self.assertTrue(any(c["status"] == "unavailable" for c in report["copies"]))

    def test_required_copy_error_is_not_success(self):
        rc, report, _, files = self.exercise(missing=("/workspace/log",))
        self.assertEqual(rc, 1)
        self.assertTrue(report["errors"])
        self.assertTrue(files)  # continue collecting other reports

    def test_running_container_stopped_before_copy(self):
        rc, report, calls, _ = self.exercise(state={"Running": True})
        self.assertEqual(rc, 0)
        self.assertEqual(calls[1][0], "stop")
        self.assertEqual(calls[2][0], "cp")
        self.assertTrue(report["stopped_for_collection"])

    def test_stop_failure_does_not_copy_live_files(self):
        rc, report, calls, files = self.exercise(state={"Running": True}, stop_error=True)
        self.assertEqual(rc, 1)
        self.assertTrue(report["errors"])
        self.assertFalse(any(c[0] == "cp" for c in calls))
        self.assertEqual(files, [])


if __name__ == "__main__":
    unittest.main()
