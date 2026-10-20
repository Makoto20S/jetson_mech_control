"""Build/test split failure-path tests with a fake colcon, no ROS or devices."""
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest

from workspace_stamp import stamp


class StampTests(unittest.TestCase):
    def fixture(self, root):
        package = root / "ros2_ws/src/example"
        package.mkdir(parents=True)
        (package / "package.xml").write_text('<package><name>example</name></package>')
        (package / "code.cpp").write_text('int value = 1;')
        (root / "tools").mkdir()
        output = root / "output"
        (output / "build/example").mkdir(parents=True)
        (output / "build/example/CTestTestfile.cmake").write_text('# fixture')
        (output / "install").mkdir()
        (output / "install/setup.bash").write_text('# fixture')
        return output

    def test_changed_source_refuses_old_build(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder); output = self.fixture(root)
            stamp(root, output)
            stamp(root, output, verify=True)
            (root / "ros2_ws/src/example/code.cpp").write_text('int value = 2;')
            with self.assertRaisesRegex(ValueError, 'inputs changed'):
                stamp(root, output, verify=True)

    def test_missing_output_refuses_test_only(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder); output = self.fixture(root)
            stamp(root, output)
            (output / "build/example/CTestTestfile.cmake").unlink()
            with self.assertRaisesRegex(ValueError, 'missing test-enabled build'):
                stamp(root, output, verify=True)

    def test_changed_tool_is_an_input_but_python_cache_is_not(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder); output = self.fixture(root)
            stamp(root, output)
            cache = root / 'tools/__pycache__'; cache.mkdir()
            (cache / 'ignored.pyc').write_bytes(b'cache')
            stamp(root, output, verify=True)
            (root / 'tools/tool.py').write_text('print(1)')
            with self.assertRaisesRegex(ValueError, 'inputs changed'):
                stamp(root, output, verify=True)


@unittest.skipUnless(os.environ.get('MECH_TEST_BASH') or shutil.which('bash'), 'fixture requires Bash')
class BuildModeTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.output = StampTests().fixture(self.root)
        tools = self.root / 'tools/ci'; tools.mkdir()
        source = Path(__file__).resolve().parent
        self.bash = os.environ.get('MECH_TEST_BASH') or shutil.which('bash')
        shutil.copyfile(source / 'workspace_stamp.py', tools / 'workspace_stamp.py')
        setup = self.root / 'setup.bash'
        setup.write_text('export PATH="$(cd "$(dirname "${BASH_SOURCE[0]}")/bin" && pwd):$PATH"\n')
        binary = self.root / 'bin'; binary.mkdir()
        fake = binary / 'colcon'
        fake.write_text('''#!/usr/bin/env bash
case " $* " in
  *" test-result "*) mode=test-result ;;
  *" build "*) mode=build ;;
  *" test "*) mode=test ;;
  *) exit 99 ;;
esac
printf '%s\\n' "$mode" >> "$CALLS"
if [[ "$mode" == test ]]; then exit "${FAIL_TEST:-0}"; fi
if [[ "$mode" == build ]]; then exit "${FAIL_BUILD:-0}"; fi
exit "${FAIL_TEST_RESULT:-0}"
''')
        fake.chmod(0o755)
        # Replace only host locations in a copy; exercise the actual mode logic.
        text = (source / 'build_workspace.sh').read_text()
        text = text.replace('/opt/ros/${ROS_DISTRO}/setup.bash', setup.as_posix())
        text = text.replace('SYSTEM_PATH="/opt/ros/${ROS_DISTRO}/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin"',
                            'SYSTEM_PATH="$(cd "$SCRIPT_DIR/../../bin" && pwd):$PATH"')
        text = text.replace('/usr/bin/python3', '"' + Path(sys.executable).as_posix() + '"')
        self.script = tools / 'build_workspace.sh'; self.script.write_text(text)
        self.calls = self.root / 'calls.txt'

    def run_mode(self, **flags):
        env = {k:v for k,v in os.environ.items() if not k.startswith('MECH_')}
        env.update(MECH_OUTPUT_ROOT=self.output.as_posix(), MECH_SKIP_ROSDEP='1',
                   CALLS=self.calls.as_posix())
        env.update(flags)
        return subprocess.run([self.bash, self.script.as_posix()], env=env, capture_output=True, text=True)

    def test_default_builds_then_tests(self):
        result = self.run_mode()
        self.assertEqual(result.returncode, 0, result.stderr + result.stdout)
        self.assertEqual(self.calls.read_text().splitlines(), ['build','test','test-result'])

    def test_split_does_not_rebuild(self):
        first = self.run_mode(MECH_BUILD_ONLY='1')
        self.assertEqual(first.returncode, 0, first.stderr + first.stdout)
        second = self.run_mode(MECH_TEST_ONLY='1', MECH_SKIP_ROSDEP='0')
        self.assertEqual(second.returncode, 0, second.stderr + second.stdout)
        self.assertEqual(self.calls.read_text().splitlines(), ['build','test','test-result'])

    def test_test_only_refuses_changed_input_before_colcon(self):
        self.assertEqual(self.run_mode(MECH_BUILD_ONLY='1').returncode, 0)
        (self.root / 'ros2_ws/src/example/code.cpp').write_text('changed')
        result = self.run_mode(MECH_TEST_ONLY='1')
        self.assertEqual(result.returncode, 2)
        self.assertEqual(self.calls.read_text().splitlines(), ['build'])

    def test_invalid_modes_fail_before_build(self):
        result = self.run_mode(MECH_BUILD_ONLY='1', MECH_TEST_ONLY='1')
        self.assertEqual(result.returncode, 2)
        self.assertFalse(self.calls.exists())

    def test_test_failure_exit_code_is_preserved(self):
        self.assertEqual(self.run_mode(MECH_BUILD_ONLY='1').returncode, 0)
        result = self.run_mode(MECH_TEST_ONLY='1', FAIL_TEST='7')
        self.assertEqual(result.returncode, 7)


if __name__ == '__main__':
    unittest.main()
