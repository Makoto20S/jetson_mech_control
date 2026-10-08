#!/usr/bin/env python3
"""Parse Git-visible Python and Bash files without importing or executing them."""
import argparse
import ast
from pathlib import Path
import subprocess
import tokenize

from context_check import tracked_paths


def check(root, paths, bash="bash"):
    counts = {"Python": 0, "Bash": 0}
    errors = []
    for relative in sorted(set(paths)):
        path = root / relative
        if path.suffix == ".py":
            counts["Python"] += 1
            try:
                with tokenize.open(path) as stream:
                    ast.parse(stream.read(), filename=relative)
            except (SyntaxError, ValueError, UnicodeError, OSError) as error:
                errors.append(f"{relative}: {error}")
        elif path.suffix == ".sh":
            counts["Bash"] += 1
            try:
                result = subprocess.run([bash, "-n", str(path.resolve())],
                                        capture_output=True, text=True, timeout=10)
                if result.returncode:
                    errors.append(f"{relative}: {result.stderr.strip()}")
            except (OSError, subprocess.TimeoutExpired) as error:
                errors.append(f"{relative}: {error}")
    return counts, errors


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, default=Path(__file__).resolve().parents[2])
    parser.add_argument("--bash", default="bash")
    args = parser.parse_args()
    counts, errors = check(args.root, tracked_paths(args.root), args.bash)
    for error in errors:
        print("ERROR: " + error)
    print(f"Parsed {counts['Python']} Python and {counts['Bash']} Bash files; {len(errors)} errors")
    return int(bool(errors))


if __name__ == "__main__":
    raise SystemExit(main())
