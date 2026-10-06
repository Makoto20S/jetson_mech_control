#!/usr/bin/env python3
"""Build the standalone offline/live diagnostic without ROS or installed libraries."""
import argparse
import os
from pathlib import Path
import shutil
import subprocess
import sys


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--cxx", default=os.environ.get("CXX", "g++"))
    parser.add_argument("--test", action="store_true", help="run only offline tests; never opens real devices")
    args = parser.parse_args()
    here = Path(__file__).resolve().parent
    root = here.parents[2]
    output = (args.output or root / "tmp" / "servo-duplex-build").resolve()
    if output.exists():
        parser.error("output already exists; select a fresh directory to preserve prior evidence")
    compiler = shutil.which(args.cxx)
    if not compiler:
        parser.error("C++ compiler unavailable: " + args.cxx)
    packages = root / "ros2_ws" / "src"
    core = packages / "mech_control_core"
    cube = packages / "mech_protocol_cubemars"
    bringup = packages / "mech_bringup"
    linux = sys.platform.startswith("linux")
    if sys.platform != "win32" and not linux:
        parser.error("supported build hosts: Windows offline, Linux offline/live")
    includes = [here, core / "include", cube / "include", bringup / "include"]
    shared = [here / "duplex.cpp", here.parent / "observer" / "observer.cpp", here.parent / "dual_board" / "diagnostic.cpp",
              core / "src" / "usb_cdc_transport.cpp",
              cube / "src" / "ak30_servo_wire.cpp"]
    live_sources = [bringup / "src" / name for name in
                    ("posix_cdc_serial_port.cpp", "pass_through_init.cpp",
                     "command_trace.cpp", "trace_snapshot.cpp")]
    flags = ["-std=c++17", "-O2", "-Wall", "-Wextra", "-Wpedantic", "-Werror", "-pthread"]
    flags += ["-I" + str(path) for path in includes]
    if sys.platform == "win32":
        flags += ["-static-libgcc", "-static-libstdc++"]
    extension = ".exe" if sys.platform == "win32" else ""
    targets = [("servo_duplex", [here / "main.cpp", *shared, *(live_sources if linux else [])])]
    tests = [("test_duplex", here / "test_duplex.cpp")]
    if linux:
        tests += [("test_posix", here / "test_posix.cpp")]
        if (here / "test_supervisor.cpp").exists():
            tests += [("test_supervisor", here / "test_supervisor.cpp")]
    for name, source in tests:
        if source.exists():
            targets.append((name, [source, *shared, *(live_sources if name in ("test_posix", "test_supervisor") else [])]))
        elif args.test:
            parser.error("required offline test source missing: " + str(source))
    for _, sources in targets:
        for source in sources:
            if not source.is_file():
                parser.error("required source missing: " + str(source))
    if args.test and not (here / "test_cli.py").is_file():
        parser.error("required CLI test source missing")
    if args.test and not all((here / name).is_file() for name in ("audit.py", "test_audit.py")):
        parser.error("required independent audit test source missing")
    # A build directory is an explicit new artifact; no existing output is reused.
    output.parent.mkdir(parents=True, exist_ok=True)
    output.mkdir()
    for name, sources in targets:
        command = [compiler, *flags, *(str(path) for path in sources), "-o", str(output / (name + extension))]
        if name == "test_posix":
            command += ["-Wl,--wrap=write"]
        print("Building " + name, flush=True)
        completed = subprocess.run(command, text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
        (output / (name + "-build.log")).write_text(completed.stdout, encoding="utf-8")
        if completed.returncode:
            print(completed.stdout, file=sys.stderr)
            return completed.returncode
    if args.test:
        commands = [(name, [str(output / (name + extension))]) for name, _ in tests]
        commands += [("test_cli", [sys.executable, str(here / "test_cli.py"),
                                   str(output / ("servo_duplex" + extension))])]
        commands += [("test_audit", [sys.executable, "-X", "utf8", str(here / "test_audit.py")])]
        for name, command in commands:
            completed = subprocess.run(command, text=True, stdout=subprocess.PIPE,
                                       stderr=subprocess.STDOUT, timeout=120)
            (output / (name + "-test.log")).write_text(completed.stdout, encoding="utf-8")
            print(completed.stdout, end="", flush=True)
            if completed.returncode:
                return completed.returncode
    print("Build complete: " + str(output))
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (OSError, subprocess.TimeoutExpired) as error:
        print("Build/test failed: " + str(error), file=sys.stderr)
        raise SystemExit(1)
