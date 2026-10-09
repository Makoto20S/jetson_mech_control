"""Sanitizer configuration and failure paths, with fake tools and no ROS/devices."""
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest

from check_sanitizer_build import check


def fixture(root):
    source = root / 'ros2_ws/src/example'
    source.mkdir(parents=True)
    (source / 'package.xml').write_text('<package><name>example</name></package>')
    output = root / 'output'
    build = output / 'build/example'
    build.mkdir(parents=True)
    (build / 'CTestTestfile.cmake').write_text('# fixture')
    (build / 'CMakeCache.txt').write_text(
        'CMAKE_CXX_FLAGS:STRING=-fsanitize=address,undefined -fno-omit-frame-pointer\n'
        'CMAKE_EXE_LINKER_FLAGS:STRING=-fsanitize=address,undefined\n'
        'CMAKE_SHARED_LINKER_FLAGS:STRING=-fsanitize=address,undefined\n')
    (build / 'compile_commands.json').write_text(json.dumps([
        {'file': 'example.cpp', 'command': 'c++ -fsanitize=address,undefined -fno-omit-frame-pointer -c example.cpp'}]))
    return output, build


class InstrumentationTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(); self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.output, self.build = fixture(self.root)

    def test_enabled_build_records_proof(self):
        report = check(self.root, self.output)
        self.assertEqual(report['packages']['example']['compilations'], 1)
        self.assertTrue((self.output / 'sanitizer-build.json').is_file())

    def test_missing_link_flags_rejected(self):
        path = self.build / 'CMakeCache.txt'
        path.write_text(path.read_text().replace('CMAKE_SHARED_LINKER_FLAGS:STRING=-fsanitize=address,undefined', 'CMAKE_SHARED_LINKER_FLAGS:STRING='))
        with self.assertRaisesRegex(ValueError, 'CMAKE_SHARED_LINKER_FLAGS'):
            check(self.root, self.output)

    def test_uninstrumented_compilation_rejected_even_with_correct_cache(self):
        (self.build / 'compile_commands.json').write_text(json.dumps([
            {'file': 'example.cpp', 'command': 'c++ -c example.cpp'}]))
        with self.assertRaisesRegex(ValueError, 'compilation lacks'):
            check(self.root, self.output)

    def test_disabled_sanitizer_rejected(self):
        (self.build / 'compile_commands.json').write_text(json.dumps([
            {'file': 'example.cpp', 'arguments': ['c++', '-fsanitize=address,undefined', '-fno-sanitize=undefined', '-fno-omit-frame-pointer']}]))
        with self.assertRaisesRegex(ValueError, 'compilation lacks'):
            check(self.root, self.output)

    def test_empty_compile_evidence_rejected(self):
        (self.build / 'compile_commands.json').write_text('[]')
        with self.assertRaisesRegex(ValueError, 'no compilation evidence'):
            check(self.root, self.output)

    def test_missing_test_build_rejected(self):
        (self.build / 'CTestTestfile.cmake').unlink()
        with self.assertRaisesRegex(ValueError, 'missing test-enabled build'):
            check(self.root, self.output)


@unittest.skipUnless(os.environ.get('MECH_TEST_BASH') or shutil.which('bash'), 'fixture requires Bash')
class SanitizerScriptTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(); self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.output, self.build = fixture(self.root)
        self.bash = os.environ.get('MECH_TEST_BASH') or shutil.which('bash')
        tools = self.root / 'tools/ci'; tools.mkdir(parents=True)
        source = Path(__file__).resolve().parent
        shutil.copyfile(source / 'check_sanitizer_build.py', tools / 'check_sanitizer_build.py')
        binary = self.root / 'bin'; binary.mkdir()
        runtime = self.root / 'libasan.so'; runtime.write_text('fixture only')
        compiler = binary / 'c++'
        compiler.write_text('#!/usr/bin/env bash\nprintf "%s\\n" "$FIXTURE_ASAN"\n'); compiler.chmod(0o755)
        fake = binary / 'colcon'
        fake.write_text('''#!/usr/bin/env bash
case " $* " in
  *" test-result "*) mode=test-result; status="${FAIL_TEST_RESULT:-0}" ;;
  *" build "*) mode=build; status="${FAIL_BUILD:-0}" ;;
  *" test "*) mode=test; status="${FAIL_TEST:-0}" ;;
  *) exit 99 ;;
esac
printf '%s\\n' "$mode" >> "$CALLS"
if [[ "$mode" == test ]]; then
  printf '%s\\n%s\\n%s\\n' "$ASAN_OPTIONS" "$UBSAN_OPTIONS" "$MECH_ASAN_RUNTIME" > "$OPTIONS"
fi
exit "$status"
'''); fake.chmod(0o755)
        setup = self.root / 'setup.bash'; setup.write_text('# fixture\n')
        text = (source / 'run_sanitizers.sh').read_text().replace('/opt/ros/humble/setup.bash', setup.as_posix())
        text = text.replace('SYSTEM_PATH="/opt/ros/humble/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin"',
                            'SYSTEM_PATH="$(cd "$REPO_ROOT/bin" && pwd):$PATH"')
        text = text.replace('/usr/bin/python3', '"' + Path(sys.executable).as_posix() + '"')
        self.script = tools / 'run_sanitizers.sh'; self.script.write_text(text)
        self.calls = self.root / 'calls.txt'
        self.options = self.root / 'options.txt'
        self.runtime = runtime

    def run_script(self, **flags):
        env = {k:v for k,v in os.environ.items() if not k.startswith('MECH_')}
        env.update(CALLS=self.calls.as_posix(), OPTIONS=self.options.as_posix(), FIXTURE_ASAN=self.runtime.as_posix())
        env.update(flags)
        return subprocess.run([self.bash, self.script.as_posix(), self.output.as_posix()], env=env, text=True, capture_output=True)

    def test_success_and_leak_setting_propagate(self):
        result = self.run_script(MECH_ASAN_DETECT_LEAKS='1')
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertEqual(self.calls.read_text().splitlines(), ['build', 'test', 'test-result'])
        options = self.options.read_text().splitlines()
        self.assertEqual(options[:2], ['detect_leaks=1:halt_on_error=1', 'halt_on_error=1:print_stacktrace=1'])
        self.assertTrue(options[2].endswith('/libasan.so'))

    def test_build_failure_stops_before_tests(self):
        self.assertEqual(self.run_script(FAIL_BUILD='7').returncode, 7)
        self.assertEqual(self.calls.read_text().splitlines(), ['build'])

    def test_bad_instrumentation_stops_before_tests(self):
        (self.build / 'compile_commands.json').write_text('[]')
        self.assertEqual(self.run_script().returncode, 2)
        self.assertEqual(self.calls.read_text().splitlines(), ['build'])

    def test_test_failure_still_collects_result_and_preserves_exit(self):
        self.assertEqual(self.run_script(FAIL_TEST='7', FAIL_TEST_RESULT='9').returncode, 7)
        self.assertEqual(self.calls.read_text().splitlines(), ['build', 'test', 'test-result'])

    def test_result_failure_is_not_passing(self):
        self.assertEqual(self.run_script(FAIL_TEST_RESULT='9').returncode, 9)

    def test_invalid_leak_option_refused_before_build(self):
        self.assertEqual(self.run_script(MECH_ASAN_DETECT_LEAKS='yes').returncode, 2)
        self.assertFalse(self.calls.exists())

    def test_missing_runtime_refused_before_build(self):
        self.runtime.unlink()
        self.assertEqual(self.run_script().returncode, 2)
        self.assertFalse(self.calls.exists())


if __name__ == '__main__':
    unittest.main()
