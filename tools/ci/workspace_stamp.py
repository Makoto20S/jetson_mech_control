#!/usr/bin/env python3
"""Reject stale/incomplete build inputs before a test-only run.

This is a local consistency check, not a signed build or test attestation.
It hashes workspace inputs and tool scripts without reading ignored evidence.
"""
import argparse
import hashlib
import json
from pathlib import Path
from xml.etree import ElementTree


def inputs(root):
    files = {}
    packages = {}
    for path in sorted((root / "ros2_ws/src").glob("*/package.xml")):
        name = ElementTree.parse(path).getroot().findtext("name")
        if not name or name in packages:
            raise ValueError("missing or duplicate ROS package name")
        packages[name] = path.parent.relative_to(root).as_posix()
    if not packages:
        raise ValueError("no workspace packages found")
    for directory in (root / "ros2_ws/src", root / "tools"):
        for path in sorted(directory.rglob("*")):
            if path.is_file() and "__pycache__" not in path.parts and path.suffix != ".pyc":
                files[path.relative_to(root).as_posix()] = hashlib.sha256(path.read_bytes()).hexdigest()
    return {"schema": 1, "packages": packages, "files": files}


def stamp(root, output, verify=False):
    root, output = Path(root), Path(output)
    current = inputs(root)
    for package in current["packages"]:
        if not (output / "build" / package / "CTestTestfile.cmake").is_file():
            raise ValueError(f"missing test-enabled build for {package}; build first")
    if not (output / "install/setup.bash").is_file():
        raise ValueError("missing install/setup.bash; build first")
    path = output / "build/mech-build-inputs.json"
    if verify:
        if json.loads(path.read_text(encoding="utf-8")) != current:
            raise ValueError("build inputs changed; rebuild before test-only execution")
    else:
        path.write_text(json.dumps(current, sort_keys=True, indent=2) + "\n", encoding="utf-8")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("mode", choices=("write", "verify"))
    parser.add_argument("--root", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    try:
        stamp(args.root, args.output, args.mode == "verify")
    except (OSError, ValueError, ElementTree.ParseError) as error:
        print(f"ERROR: {error}")
        return 2
    print(f"Build inputs {args.mode}: OK")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
