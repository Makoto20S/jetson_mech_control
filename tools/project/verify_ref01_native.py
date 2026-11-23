"""Compile the portable REF01 native core/model and compare with Python on Windows.
No ROS or device access. Requires the existing MuJoCo Python environment and a C++17 compiler.
"""
import argparse
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[2]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--compiler", type=Path)
    args = parser.parse_args()
    import mujoco
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    sdk = Path(mujoco.__file__).parent
    windows = sys.platform == "win32"
    compiler = args.compiler or (ROOT/".local/toolchains/zig-0.13.0/ziglang/zig.exe" if windows else Path("/usr/bin/g++"))
    if not compiler.is_file():
        raise RuntimeError("Provide a C++17 compiler; see impedance_simulation.md")
    env = dict(os.environ, PYTHONDONTWRITEBYTECODE="1")
    cache = ROOT/".local/cache/zig"
    cache.mkdir(parents=True, exist_ok=True)
    env.update(ZIG_GLOBAL_CACHE_DIR=str(cache), ZIG_LOCAL_CACHE_DIR=str(cache/"local"))
    common = [str(compiler)] + (["c++"] if compiler.stem == "zig" else [])
    common += ["-std=c++17", "-O2", "-DNDEBUG", "-Wall", "-Wextra", "-Werror",
               "-I"+str(ROOT/"ros2_ws/src/mech_controllers/include/mech_controllers"),
               "-I"+str(ROOT/"mujoco/ref01/cpp"), "-I"+str(sdk/"include")]
    calls = []
    def call(command, name):
        done = subprocess.run(command, cwd=ROOT, env=env, capture_output=True, text=True,
                              encoding="utf-8", errors="replace", timeout=300)
        (output/(name+".log")).write_text(done.stdout+done.stderr, encoding="utf-8")
        calls.append(dict(name=name, command=command, returncode=done.returncode))
        (output/"commands.json").write_text(json.dumps(calls,indent=2),encoding="utf-8")
        if done.returncode:
            raise RuntimeError(f"{name} failed: {done.stdout[-1000:]} {done.stderr[-3000:]}")
    suffix = ".exe" if windows else ""
    core = output/("core_test"+suffix)
    call(common+[str(ROOT/"tools/project/test_ref01_core_portable.cpp"), "-o", str(core)],"compile_core")
    call([str(core)],"core_test")
    probe = output/("native_probe"+suffix)
    library = sdk/"mujoco.dll" if windows else next(sdk.glob("libmujoco.so.*"))
    link = [str(library)]
    if not windows: link += ["-Wl,-rpath,"+str(sdk)]
    call(common+[str(ROOT/"tools/project/ref01_native_probe.cpp"),*link,"-o",str(probe)],"compile_probe")
    if windows:
        shutil.copy2(library,output/library.name)
    call([sys.executable,"-B",str(ROOT/"tools/project/test_ref01_native.py"),
          "--probe",str(probe),"--output",str(output/"numeric.json"),"--repo",str(ROOT)],"numeric")
    call([sys.executable,"-B","-m","unittest","discover","-s","tools/project",
          "-p","test_timing_contract.py","-v"],"timing_contract")
    receiver = output/("receiver_test"+suffix)
    call(common+["-I"+str(ROOT/"ros2_ws/src/mech_simulation/include"),
          str(ROOT/"ros2_ws/src/mech_simulation/test/test_ref01_sim_receiver.cpp"),
          "-o",str(receiver)],"compile_receiver")
    call([str(receiver)],"receiver_test")
    call([sys.executable,"-B","-m","unittest","discover","-s","tools/project",
          "-p","test_ref01_manager*.py","-v"],"manager_validation")
    report = json.loads((output/"numeric.json").read_text(encoding="utf-8"))
    print(json.dumps({k: report[k] for k in ("passed","samples","maximum_errors","failures")}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
