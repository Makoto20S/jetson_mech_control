"""Exercise retry/abort behavior with fake apt and DNS; never install packages."""
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest


BASH = os.environ.get("MECH_TEST_BASH") or shutil.which("bash")
SCRIPT = Path(__file__).with_name("apt_update.sh").resolve()


@unittest.skipUnless(BASH, "Bash is unavailable on this host")
class AptUpdateTests(unittest.TestCase):
    def run_fixture(self, failures, dns_failure=False):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            bin_dir = root / "bin"
            bin_dir.mkdir()
            commands = {
                "apt-get": '''#!/usr/bin/env bash
printf '%s\\n' "$*" >> "$APT_TEST_ARGS"
count=0
if [[ -f "$APT_TEST_COUNT" ]]; then read -r count < "$APT_TEST_COUNT"; fi
count=$((count + 1))
printf '%s\\n' "$count" > "$APT_TEST_COUNT"
if [[ "$count" -le "$APT_TEST_FAILURES" ]]; then
  echo "E: index download failed" >&2
  exit 100
fi
''',
                "sleep": '#!/usr/bin/env bash\nprintf "%s\\n" "$1" >> "$APT_TEST_SLEEPS"\n',
                "getent": '#!/usr/bin/env bash\nexit "$APT_TEST_DNS_FAILURE"\n',
                "timeout": '#!/usr/bin/env bash\nshift\n"$@"\n',
            }
            for name, content in commands.items():
                path = bin_dir / name
                path.write_bytes(content.encode())
                path.chmod(0o755)
            env = dict(os.environ, APT_TEST_FAILURES=str(failures),
                       APT_TEST_DNS_FAILURE=str(int(dns_failure)),
                       APT_TEST_COUNT=(root / "count").as_posix(),
                       APT_TEST_ARGS=(root / "args").as_posix(),
                       APT_TEST_SLEEPS=(root / "sleeps").as_posix())
            # MSYS Bash needs a POSIX fixture directory in PATH; Linux uses it as-is.
            command = '''fixture_bin="$1"
if command -v cygpath >/dev/null 2>&1; then fixture_bin="$(cygpath -u "$fixture_bin")"; fi
export PATH="$fixture_bin:$PATH"
bash "$2" && printf installed > "$3"
'''
            result = subprocess.run([BASH, "-c", command, "apt-fixture",
                                     bin_dir.as_posix(), SCRIPT.as_posix(),
                                     (root / "installed").as_posix()],
                                    env=env, capture_output=True, text=True, timeout=20)
            self.assertTrue((root / "args").exists(), result.stdout + result.stderr)
            args = (root / "args").read_text().splitlines()
            sleeps = ((root / "sleeps").read_text().splitlines()
                      if (root / "sleeps").exists() else [])
            return result, args, sleeps, (root / "installed").exists()

    def test_success_advances_without_retry(self):
        result, args, sleeps, installed = self.run_fixture(0)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertTrue(installed)
        self.assertEqual(args, ["-o APT::Update::Error-Mode=any update"])
        self.assertEqual(sleeps, [])

    def test_transient_index_failures_retry_then_advance(self):
        result, args, sleeps, installed = self.run_fixture(2)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertTrue(installed)
        self.assertEqual(args, ["-o APT::Update::Error-Mode=any update"] * 3)
        self.assertEqual(sleeps, ["5", "10"])
        self.assertEqual(result.stdout.count("CI DNS diagnostics: build-container"), 2)

    def test_persistent_failure_blocks_installation(self):
        result, args, sleeps, installed = self.run_fixture(99)
        self.assertEqual(result.returncode, 100, result.stderr)
        self.assertFalse(installed)
        self.assertEqual(len(args), 3)
        self.assertEqual(sleeps, ["5", "10"])
        self.assertIn("complete APT indexes unavailable", result.stderr)

    def test_dns_diagnostics_cannot_hide_the_apt_failure(self):
        result, args, _, installed = self.run_fixture(99, dns_failure=True)
        self.assertEqual(result.returncode, 100, result.stderr)
        self.assertFalse(installed)
        self.assertEqual(len(args), 3)
        self.assertIn("DNS lookup failed or timed out: archive.ubuntu.com", result.stderr)


if __name__ == "__main__":
    unittest.main()
