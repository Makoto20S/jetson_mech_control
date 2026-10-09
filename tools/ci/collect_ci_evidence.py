#!/usr/bin/env python3
"""Copy allowlisted diagnostics from the CI container, including after failure.

Never copies the source tree, environment, install tree or full Docker inspect.
The workflow owns removal after upload. Missing optional reports are recorded,
not confused with passing tests. A failed required copy fails collection.
"""
import argparse
import json
from pathlib import Path
import subprocess
from summarize_test_reports import PACKAGES, summarize


def docker(*args):
    return subprocess.run(
        ["docker", *args], capture_output=True, text=True, timeout=20,
        encoding="utf-8", errors="replace", check=False,
    )


def collect(container, output, run=docker, sanitizers=False):
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    report = {"container": container, "copies": [], "errors": []}
    failed = False
    try:
        result = run("inspect", "--format", "{{json .State}}", container)
        if result.returncode:
            # Image build or docker run may have failed before container creation.
            report["container_state"] = "unavailable"
            report["inspection_error"] = result.stderr.strip()
            print("::warning::No inspectable test container; see image-build/test-console logs")
            return 0
        state = json.loads(result.stdout)
        report["container_state"] = {
            key: state.get(key) for key in
            ("Status", "Running", "ExitCode", "OOMKilled", "Error", "StartedAt", "FinishedAt")
        }
        if state.get("Running"):
            # Cancellation can leave docker's child running. Stop before copying.
            result = run("stop", "--time", "5", container)
            if result.returncode:
                raise RuntimeError("could not stop test container: " + result.stderr.strip())
            report["stopped_for_collection"] = True
        sources = [("/workspace/log", "colcon-log", True),
                   ("/workspace/ci-ros-logs", "ros-log", False),
                   ("/workspace/ci-test-artifacts", "pty-tests", False),
                   ("/workspace/ci-stability", "stability", False)]
        if sanitizers:
            sources.append(("/workspace/sanitizer-build.json", "sanitizer-build.json", False))
        for package in PACKAGES:
            for directory in ("test_results", "Testing"):
                sources.append((f"/workspace/build/{package}/{directory}",
                                f"reports/{package}/{directory}", False))
        for source, destination, required in sources:
            target = output / destination
            target.parent.mkdir(parents=True, exist_ok=True)
            result = run("cp", f"{container}:{source}", str(target))
            item = {"source": source, "destination": destination,
                    "status": "copied" if result.returncode == 0 else "unavailable"}
            if result.returncode:
                item["error"] = result.stderr.strip()
                if required:
                    failed = True
                    report["errors"].append("required colcon logs could not be copied")
            report["copies"].append(item)
        missing = sum(item["status"] != "copied" for item in report["copies"])
        if missing:
            print(f"::warning::{missing} diagnostic paths unavailable; see collection.json")
    except (OSError, subprocess.TimeoutExpired, ValueError, RuntimeError) as error:
        report["errors"].append(str(error))
        failed = True
    finally:
        (output / "test-summary.json").write_text(
            json.dumps(summarize(output / 'reports'), indent=2) + '\n', encoding='utf-8')
        (output / "collection.json").write_text(
            json.dumps(report, indent=2) + "\n", encoding="utf-8")
    return 1 if failed else 0


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--container", required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--sanitizers", action="store_true")
    args = parser.parse_args()
    return collect(args.container, args.output, sanitizers=args.sanitizers)


if __name__ == "__main__":
    raise SystemExit(main())
