#!/usr/bin/env python3
"""Summarize available XML, including GTest skips omitted by Humble colcon.

Report availability is not proof that the entire test suite passed or ran.
"""
import argparse
import json
from pathlib import Path
from xml.etree import ElementTree


PACKAGES = (
    'mech_control_core', 'mech_simulation', 'mech_hardware_ros2_control',
    'mech_controllers', 'mech_protocol_cubemars', 'mech_protocol_ctrboard',
    'mech_ctrboard_bridge', 'mech_bringup',
)


def summarize(root, packages=PACKAGES):
    root = Path(root)
    report = {'schema': 1, 'packages': {}, 'testcases': 0, 'skipped': [],
              'failures': [], 'not_run': [], 'parse_errors': [], 'missing_packages': []}
    for package in packages:
        paths = sorted((root / package / 'test_results').rglob('*.xml'))
        report['packages'][package] = {'xml_files': len(paths)}
        if not paths:
            report['missing_packages'].append(package)
        for path in paths:
            relative = path.relative_to(root).as_posix()
            try:
                document = ElementTree.parse(path)
            except (OSError, ElementTree.ParseError) as error:
                report['parse_errors'].append({'report': relative, 'error': str(error)})
                continue
            for case in document.iter('testcase'):
                report['testcases'] += 1
                label = {'report': relative, 'classname': case.get('classname'), 'name': case.get('name')}
                if case.find('skipped') is not None or case.get('result') == 'skipped':
                    report['skipped'].append(label)
                elif case.get('status') == 'notrun':
                    report['not_run'].append(label)
                if case.find('failure') is not None or case.find('error') is not None:
                    report['failures'].append(label)
    report['all_package_reports_available'] = not report['missing_packages'] and not report['parse_errors']
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    report = summarize(args.root)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + '\n', encoding='utf-8')
    print(f'Available XML: {report["testcases"]} testcases, {len(report["skipped"])} skips, '
          f'{len(report["failures"])} failures/errors; missing packages: {len(report["missing_packages"])}')
    return 2 if report['parse_errors'] else 0


if __name__ == '__main__':
    raise SystemExit(main())
