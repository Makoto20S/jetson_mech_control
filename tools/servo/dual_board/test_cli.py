#!/usr/bin/env python3
"""Bounded subprocess tests; uses temporary files/PTYs only, never hardware."""
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest


EXECUTABLE = Path(sys.argv[1]).resolve() if len(sys.argv) > 1 else None
if len(sys.argv) > 1:
    del sys.argv[1]


class DiagnosticCli(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory(prefix="dual-board-cli-")
        self.root = Path(self.directory.name)
        self.output = self.root / "must-not-exist"

    def tearDown(self):
        self.directory.cleanup()

    def call(self, *arguments, expected=0):
        result = subprocess.run([str(EXECUTABLE), *map(str, arguments)], cwd=self.root,
                                stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                                text=True, timeout=5)
        self.assertEqual(result.returncode, expected, (arguments, result.stdout, result.stderr))
        self.assertFalse(self.output.exists(), "dry-run/rejected admission created output")
        return result

    def reject(self, *arguments):
        result = self.call(*arguments, expected=2)
        self.assertIn("error:", result.stderr)
        self.assertEqual(result.stdout, "")
        return result

    def test_default_dry_run_no_files(self):
        before = list(self.root.iterdir())
        plan = json.loads(self.call().stdout)
        self.assertFalse(plan["run_requested"])
        self.assertFalse(plan["firmware_verified"])
        self.assertFalse(plan["bitrate_verified"])
        self.assertIsNone(plan["actual_identities"])
        self.assertEqual(plan["frames_per_id"], 20)
        self.assertEqual(before, list(self.root.iterdir()))

    def test_dry_run_paths_are_not_opened(self):
        # Nonexistent paths and output are inert strings in the plan.
        plan = json.loads(self.call("--tx", self.root / "absent-tx", "--rx", self.root / "absent-rx",
                                    "--output", self.output, "--nonce", "0").stdout)
        self.assertEqual(plan["nonce"], 0)
        self.assertEqual(list(self.root.iterdir()), [])

    def test_help_and_duplicates(self):
        self.assertIn("default prints JSON plan", self.call("--help").stdout)
        for flag in ("--help", "--run", "--motors-disconnected"):
            with self.subTest(flag=flag):
                self.reject(flag, flag)
        for flag, value in (("--hz", "10"), ("--tx", "x"), ("--profile", "sequence"),
                            ("--nonce", "1"), ("--output", self.output)):
            with self.subTest(flag=flag):
                self.reject(flag, value, flag, value)

    def test_unknown_and_missing(self):
        for arguments in (("--unknown",), ("positional",), ("--hz",), ("--tx",),
                          ("--nonce", "--help"), ("--output", ""), ("--",)):
            with self.subTest(arguments=arguments):
                self.reject(*arguments)

    def test_numeric_syntax_overflow_and_ranges(self):
        for key in ("--hz", "--seconds", "--lanes", "--drain-ms", "--log-mib", "--nonce"):
            for value in ("-1", "+1", "1.0", "0x10", "1e2", " 1", "1 ", "", "4294967296",
                          "999999999999999999999999999999999999999999999999999999999"):
                with self.subTest(key=key, value=value):
                    self.reject(key, value)
        for key, values in (("--hz", ("0", "501")), ("--seconds", ("0", "121")),
                            ("--lanes", ("0", "3")), ("--drain-ms", ("199", "5001")),
                            ("--log-mib", ("0", "65"))):
            for value in values:
                with self.subTest(key=key, value=value):
                    self.reject(key, value)

    def test_numeric_boundaries(self):
        for arguments in (("--hz", "1", "--seconds", "1", "--drain-ms", "200", "--log-mib", "1", "--nonce", "0"),
                          ("--hz", "500", "--seconds", "120", "--drain-ms", "5000", "--log-mib", "64",
                           "--lanes", "2", "--nonce", "4294967295")):
            plan = json.loads(self.call(*arguments).stdout)
            self.assertEqual(plan["frames_per_id"], plan["hz_per_id"] * plan["seconds"])

    def test_packing_profiles(self):
        for profile in ("sequence", "constant", "mode6"):
            for packing, sizes in (("separate", [21, 21]), ("joined", [42]), ("batch", [35])):
                with self.subTest(profile=profile, packing=packing):
                    plan = json.loads(self.call("--lanes", "2", "--profile", profile,
                                               "--packing", packing, "--nonce", "1").stdout)
                    self.assertEqual([len(bytes.fromhex(p)) for p in plan["first_writes_hex"]], sizes)
                    self.assertEqual(plan["sequence_observable"], profile == "sequence")
        for arguments in (("--profile", "unknown"), ("--packing", "unknown"),
                          ("--packing", "joined"), ("--packing", "batch")):
            self.reject(*arguments)

    def test_run_requires_motor_isolation_and_paths(self):
        self.assertIn("motors-disconnected", self.reject("--run").stderr)
        self.assertIn("motors-disconnected", self.reject("--run", "--tx", "a", "--rx", "b",
                                                        "--output", self.output).stderr)
        for missing in ("--tx", "--rx", "--output"):
            args = ["--run", "--motors-disconnected"]
            for flag, value in (("--tx", "a"), ("--rx", "b"), ("--output", self.output)):
                if flag != missing:
                    args.extend((flag, value))
            self.reject(*args)

    def test_equal_endpoints_are_rejected_before_admission(self):
        self.assertIn("must differ", self.reject("--tx", "same", "--rx", "same").stderr)
        self.assertIn("must differ", self.reject("--run", "--motors-disconnected", "--tx", "same",
                                                "--rx", "same", "--output", self.output).stderr)

    def test_invalid_live_paths_cannot_create_output(self):
        self.reject("--run", "--motors-disconnected", "--tx", self.root / "absent-tx",
                    "--rx", self.root / "absent-rx", "--output", self.output)
        self.assertEqual(list(self.root.iterdir()), [])

    @unittest.skipUnless(sys.platform.startswith("linux"), "PTY admission is Linux-only")
    def test_arbitrary_pty_is_rejected_and_untouched(self):
        # Two actual PTYs are intentionally forbidden CLI endpoints. The core
        # production-port tests use PTYs directly, never bypass CLI admission.
        first = os.openpty()
        second = os.openpty()
        try:
            self.reject("--run", "--motors-disconnected", "--tx", os.ttyname(first[1]),
                        "--rx", os.ttyname(second[1]), "--output", self.output)
            for master, _ in (first, second):
                os.set_blocking(master, False)
                with self.assertRaises(BlockingIOError):
                    os.read(master, 1024)
        finally:
            for descriptor in (*first, *second):
                os.close(descriptor)


if __name__ == "__main__":
    if EXECUTABLE is None or not EXECUTABLE.is_file():
        raise SystemExit("usage: test_cli.py CLI_EXECUTABLE")
    unittest.main()
