#!/usr/bin/env python3
"""Fail CI if its native Linux host or Docker image has the wrong architecture."""
import argparse
import json
from pathlib import Path
import platform
import subprocess


MACHINES = {'amd64': 'x86_64', 'arm64': 'aarch64'}


def check(expected, output, image=None, system=None, machine=None, run=subprocess.run):
    report = {'expected_platform': f'linux/{expected}',
              'host_system': system if system is not None else platform.system(),
              'host_machine': machine if machine is not None else platform.machine(),
              'errors': []}
    try:
        if report['host_system'] != 'Linux' or report['host_machine'] != MACHINES.get(expected):
            raise ValueError('native Linux host does not match the requested architecture')
        if image:
            result = run(['docker', 'image', 'inspect', '--format', '{{.Os}}/{{.Architecture}}', image],
                         capture_output=True, text=True, timeout=20, check=False)
            if result.returncode:
                raise ValueError('Docker image platform inspection failed: ' + result.stderr.strip())
            report['image'] = image
            report['image_platform'] = result.stdout.strip()
            if report['image_platform'] != report['expected_platform']:
                raise ValueError('Docker image does not match the native CI platform')
    except (OSError, ValueError, subprocess.TimeoutExpired) as error:
        report['errors'].append(str(error))
    output = Path(output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, indent=2) + '\n', encoding='utf-8')
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--expected', choices=MACHINES, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--image')
    args = parser.parse_args()
    report = check(args.expected, args.output, args.image)
    if report['errors']:
        print('ERROR: ' + '; '.join(report['errors']))
        return 2
    print(f'Native CI platform verified: {report["expected_platform"]}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
