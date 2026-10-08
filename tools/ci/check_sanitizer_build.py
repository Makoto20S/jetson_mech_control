#!/usr/bin/env python3
"""Verify sanitizer flags on all package compilations before running tests."""
import argparse
import json
from pathlib import Path
import shlex
from xml.etree import ElementTree


def sanitized(tokens):
    enabled = set()
    for token in tokens:
        if token.startswith('-fno-sanitize='):
            if set(token.split('=', 1)[1].split(',')) & {'all', 'address', 'undefined'}:
                return False
        if token.startswith('-fsanitize='):
            enabled.update(token.split('=', 1)[1].split(','))
    return {'address', 'undefined'} <= enabled


def check(root, output):
    packages = {}
    manifests = sorted((Path(root) / 'ros2_ws/src').glob('*/package.xml'))
    if not manifests:
        raise ValueError('no workspace packages found')
    for manifest in manifests:
        name = ElementTree.parse(manifest).getroot().findtext('name')
        if not name or name in packages:
            raise ValueError('missing or duplicate package name')
        build = Path(output) / 'build' / name
        if not (build / 'CTestTestfile.cmake').is_file():
            raise ValueError(f'{name}: missing test-enabled build')
        cache = {}
        for line in (build / 'CMakeCache.txt').read_text().splitlines():
            if line and not line.startswith(('#', '//')) and '=' in line:
                key, value = line.split('=', 1)
                cache[key.split(':', 1)[0]] = value
        for key in ('CMAKE_CXX_FLAGS', 'CMAKE_EXE_LINKER_FLAGS', 'CMAKE_SHARED_LINKER_FLAGS'):
            if not sanitized(shlex.split(cache.get(key, ''))):
                raise ValueError(f'{name}: {key} lacks mandatory ASan/UBSan flags')
        compilations = json.loads((build / 'compile_commands.json').read_text())
        if not compilations:
            raise ValueError(f'{name}: no compilation evidence')
        for entry in compilations:
            tokens = entry.get('arguments') or shlex.split(entry['command'])
            if not sanitized(tokens) or '-fno-omit-frame-pointer' not in tokens:
                raise ValueError(f'{name}: compilation lacks ASan/UBSan/frame pointers: {entry["file"]}')
        packages[name] = {'compilations': len(compilations), 'flags': {
            key: cache[key] for key in ('CMAKE_CXX_FLAGS', 'CMAKE_EXE_LINKER_FLAGS', 'CMAKE_SHARED_LINKER_FLAGS')}}
    report = {'schema': 1, 'sanitizers': ['address', 'undefined'], 'packages': packages}
    (Path(output) / 'sanitizer-build.json').write_text(json.dumps(report, indent=2) + '\n')
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    try:
        report = check(args.root, args.output)
    except (OSError, ValueError, KeyError, ElementTree.ParseError) as error:
        print(f'ERROR: {error}')
        return 2
    print(f'ASan/UBSan instrumentation verified for {len(report["packages"])} packages')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
