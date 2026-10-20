#!/usr/bin/env python3
"""Bounded observer CLI tests; never opens a real device."""
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

EXECUTABLE = Path(sys.argv.pop(1)).resolve() if len(sys.argv) > 1 else None


class ObserverCli(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix='observer-cli-')
        self.root = Path(self.temp.name)
        self.output = self.root / 'must-not-exist'

    def tearDown(self):
        self.temp.cleanup()

    def call(self, *arguments, expected=0):
        result = subprocess.run([str(EXECUTABLE), *map(str, arguments)], cwd=self.root,
                                text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=5)
        self.assertEqual(result.returncode, expected, (arguments, result.stdout, result.stderr))
        self.assertFalse(self.output.exists())
        return result

    def reject(self, *arguments):
        result = self.call(*arguments, expected=2)
        self.assertIn('error:', result.stderr)
        self.assertEqual(result.stdout, '')
        return result

    def test_default_dry_run_no_files(self):
        plan = json.loads(self.call().stdout)
        self.assertFalse(plan['run_requested'])
        self.assertEqual(plan['role'], 'observe')
        self.assertEqual(list(self.root.iterdir()), [])

    def test_explicit_paths_remain_inert_without_run(self):
        json.loads(self.call('--port', self.root / 'absent', '--output', self.output).stdout)
        self.assertEqual(list(self.root.iterdir()), [])

    def test_help_duplicates_unknown_and_missing(self):
        self.assertIn('observer', self.call('--help').stdout.lower())
        for flag in ('--help', '--run', '--motors-disconnected', '--count-only'):
            self.reject(flag, flag)
        for flag, value in (('--role', 'observe'), ('--port', 'a'), ('--seconds', '1'), ('--nonce', '1')):
            self.reject(flag, value, flag, value)
        for arguments in (('--unknown',), ('positional',), ('--port',), ('--seconds', '--help'),
                          ('--output', ''), ('--tx', 'x'), ('--rx', 'x'), ('--packing', 'batch')):
            self.reject(*arguments)

    def test_numeric_syntax_overflow_and_ranges(self):
        for key in ('--hz', '--seconds', '--log-mib', '--nonce'):
            for value in ('-1', '+1', '1.0', '0x10', '1e2', ' 1', '1 ', '', '4294967296', '999999999999999999999999'):
                with self.subTest(key=key, value=value):
                    self.reject(key, value)
        for key, values in (('--hz', ('0', '501')), ('--seconds', ('0', '181')), ('--log-mib', ('0', '65'))):
            for value in values:
                self.reject(key, value)

    def test_numeric_boundaries(self):
        for arguments in (('--seconds', '1', '--log-mib', '1'),
                          ('--seconds', '180', '--log-mib', '64'),
                          ('--role', 'fixture', '--seconds', '1', '--nonce', '0'),
                          ('--role', 'fixture', '--seconds', '180', '--nonce', '4294967295')):
            json.loads(self.call(*arguments).stdout)

    def test_roles_profiles_and_count_only(self):
        plan = json.loads(self.call('--role', 'observe', '--count-only').stdout)
        self.assertEqual(plan['role'], 'observe')
        for profile in ('sequence', 'four-id'):
            plan = json.loads(self.call('--role', 'fixture', '--profile', profile, '--nonce', '1').stdout)
            self.assertEqual(plan['role'], 'fixture')
            self.assertEqual(plan['profile'], profile)
        self.reject('--role', 'bad')
        self.reject('--role', 'fixture', '--profile', 'mode6')
        self.reject('--role', 'fixture', '--count-only')
        for flag,value in (('--profile','sequence'),('--hz','10'),('--lanes','1'),('--frames','1'),('--nonce','1'),('--drain-ms','200')):
            self.reject('--role','observe',flag,value)
        self.reject('--role','fixture','--profile','four-id','--frames','3')
        self.reject('--role','fixture','--profile','four-id','--frames','11')
        self.reject('--role','fixture','--profile','four-id','--hz','11')
        self.reject('--role','fixture','--profile','four-id','--lanes','2')

    def test_run_admission_requires_paths_and_fixture_isolation(self):
        self.reject('--run')
        self.reject('--run', '--role', 'observe', '--port', self.root / 'absent', '--output', self.output)
        result = self.reject('--run', '--role', 'fixture', '--port', self.root / 'absent', '--output', self.output)
        self.assertIn('motors-disconnected', result.stderr)
        self.reject('--run', '--role', 'fixture', '--motors-disconnected', '--port', self.root / 'absent', '--output', self.output)
        self.assertEqual(list(self.root.iterdir()), [])

    @unittest.skipUnless(sys.platform.startswith('linux'), 'Linux-only PTY admission')
    def test_arbitrary_pty_rejected_untouched(self):
        master, slave = os.openpty()
        try:
            self.reject('--run', '--role', 'observe', '--port', os.ttyname(slave), '--output', self.output)
            os.set_blocking(master, False)
            with self.assertRaises(BlockingIOError):
                os.read(master, 1024)
        finally:
            os.close(master)
            os.close(slave)


if __name__ == '__main__':
    if EXECUTABLE is None or not EXECUTABLE.is_file():
        raise SystemExit('usage: test_cli.py CLI_EXECUTABLE')
    unittest.main()
