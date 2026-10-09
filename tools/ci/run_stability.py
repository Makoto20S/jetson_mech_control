#!/usr/bin/env python3
"""Independent repetitions, never retry-to-green. Every exit status is retained."""
import argparse
import json
from pathlib import Path
import shutil
import subprocess

TESTS = {'mech_bringup_' + name for name in (
    'servo_motion_ros', 'servo_two_bus_ros', 'servo_jtc_manager_test',
    'jtc_manager_test', 'e5_process_host_pause')}
PATTERN = '^(' + '|'.join(sorted(TESTS)) + ')$'


def run(build, output, repetitions, runner=subprocess.run):
    output.mkdir(parents=True, exist_ok=True)
    inventory = runner(['ctest', '--test-dir', str(build), '-R', PATTERN,
                        '--show-only=json-v1'], capture_output=True, text=True, check=True)
    available = {test['name'] for test in json.loads(inventory.stdout)['tests']}
    if available != TESTS:
        raise ValueError(f'incomplete stability suite: {sorted(available)}')
    (output / 'inventory.json').write_text(inventory.stdout, encoding='utf-8')
    report = {'repetitions': repetitions, 'pattern': PATTERN, 'rounds': []}
    for number in range(1, repetitions + 1):
        directory = output / f'round-{number:02}'
        directory.mkdir()
        with (directory / 'console.log').open('w', encoding='utf-8') as stream:
            # CTest owns each test's timeout and process cleanup. Killing only
            # the CTest parent here would leave ROS children in the next round.
            result = runner(['ctest', '--test-dir', str(build), '-R', PATTERN,
                             '--no-tests=error', '--output-on-failure', '--timeout', '180'],
                            stdout=stream, stderr=subprocess.STDOUT)
            code = result.returncode
        if (build / 'test_results').exists():
            shutil.copytree(build / 'test_results', directory / 'test_results')
        report['rounds'].append({'round': number, 'exit_code': code})
        report['passed'] = sum(r['exit_code'] == 0 for r in report['rounds'])
        (output / 'summary.json').write_text(json.dumps(report, indent=2), encoding='utf-8')
        print(f'Stability round {number}/{repetitions}: exit {code}', flush=True)
    return int(report['passed'] != repetitions)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--build', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--repetitions', type=int, choices=range(1, 21), required=True)
    args = parser.parse_args()
    return run(args.build, args.output, args.repetitions)


if __name__ == '__main__':
    raise SystemExit(main())
