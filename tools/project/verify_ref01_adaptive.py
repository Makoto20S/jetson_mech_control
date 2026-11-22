
"""Windows-only validation of the opt-in adaptive predictor; same plant and thresholds."""
import argparse
import hashlib
import json
import os
import shutil
import subprocess
from pathlib import Path

import mujoco
import numpy as np
import verify_ref01_robustness as baseline

ROOT = Path(__file__).resolve().parents[2]
HEADER = "ros2_ws/src/mech_controllers/include/mech_controllers/ref01_adaptive_predictor.hpp"

def compile_probe(out):
    sdk = Path(mujoco.__file__).parent
    cmd = [str(ROOT/".local/toolchains/zig-0.13.0/ziglang/zig.exe"),
           "c++", "-std=c++17", "-O2", "-DNDEBUG", "-Wall", "-Wextra", "-Werror"]
    for folder in ["ros2_ws/src/mech_controllers/include/mech_controllers",
                   "ros2_ws/src/mech_protocol_cubemars/include", "mujoco/ref01/cpp"]:
        cmd += ["-I"+str(ROOT/folder)]
    cmd += ["-I"+str(sdk/"include"), str(ROOT/"tools/project/ref01_adaptive_probe.cpp"),
            str(ROOT/"ros2_ws/src/mech_protocol_cubemars/src/ak30_force_wire.cpp"),
            str(sdk/"mujoco.dll"), "-o", str(out/"probe.exe")]
    env = dict(os.environ, ZIG_GLOBAL_CACHE_DIR=str(ROOT/".local/cache/zig"),
               ZIG_LOCAL_CACHE_DIR=str(ROOT/".local/cache/zig/local"))
    result = subprocess.run(cmd, capture_output=True, text=True, env=env, timeout=300)
    (out/"compile.log").write_text(result.stdout+result.stderr, encoding="utf-8")
    if result.returncode:
        raise RuntimeError(result.stderr)
    shutil.copy2(sdk/"mujoco.dll", out/"mujoco.dll")

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--long", action="store_true")
    parser.add_argument("--cross", action="store_true")
    parser.add_argument("--damping", type=float, default=1.)
    args = parser.parse_args()
    out = args.output.resolve()
    out.mkdir(parents=True, exist_ok=False)
    compile_probe(out)
    sources = [HEADER, "tools/project/ref01_pair_quantization.hpp", "tools/project/ref01_adaptive_probe.cpp",
               "tools/project/verify_ref01_adaptive.py", "tools/project/verify_ref01_robustness.py",
               "mujoco/ref01/ref01_dynamics_api.py", "mujoco/ref01/cpp/ref01_model.hpp",
               "ros2_ws/src/mech_controllers/include/mech_controllers/ref01_impedance_core.hpp",
               "ros2_ws/src/mech_protocol_cubemars/src/ak30_force_wire.cpp",
               "mujoco/ref01/config/mujoco/ref01_dynamics_cad_v1.xml"]
    before = {s:hashlib.sha256((ROOT/s).read_bytes()).hexdigest() for s in sources}
    (out/"source_hashes.json").write_text(json.dumps(before, indent=2))
    shutil.copy2(ROOT/HEADER, out/"candidate_predictor.hpp")
    mode = (f"predict_B{args.damping:g}", "predict", args.damping)
    specs=baseline.SCENARIOS
    if args.cross:
        specs=[(f"cross_J{j:g}_D{d}",1.,j,0.,0.,d)
               for j in (.003,.012,.020) for d in (10,30,40)]
        specs += [("cross_friction_D40",1.,.005,.1,.02,40)]
        jobs=[(s,mode,ax,sign,np.deg2rad([0,20]),6.,"cross")
              for s in specs for ax in (0,1) for sign in (-1,1)]
    elif args.long:
        jobs = [(s,mode,ax,1,np.deg2rad([0,20]),30.,"long")
                for s in baseline.SCENARIOS for ax in (0,1)]
    else:
        jobs = [(s,mode,ax,sign,np.deg2rad([0,20]),6.,"grid")
                for s in baseline.SCENARIOS for ax in (0,1) for sign in (-1,1)]
        jobs += [(s,mode,ax,sign,np.deg2rad([.55,15.288231]),6.,"recorded")
                 for s in baseline.SCENARIOS if s[0] in ("nominal","delay_10","combined_rotor")
                 for ax in (0,1) for sign in (-1,1)]
    design = dict(scenarios=specs,modes=[mode],expected_cases=len(jobs),
                  controller_h=.005, feedback_h=.020, core="AdaptiveBenchPredictor", damping_scale=[.8,1.], quantization="joint acceleration-error selection, unchanged MIT codec",
                  plant_parameters_hidden=True, stiffness=10, damping=args.damping,
                  boundary="Windows numerical simulation; no ROS/devices")
    (out/"design.json").write_text(json.dumps(design, indent=2))
    results = []
    for i,job in enumerate(jobs):
        r = baseline.run_case(out,*job)
        results.append(r)
        (out/"summary.json").write_text(json.dumps(dict(complete=len(results)==len(jobs),cases=results),indent=2))
        print(i+1,len(jobs),r["id"],r["passed"],r["failure"],r["tail_speed_deg_s"],flush=True)
    assert before == {s:hashlib.sha256((ROOT/s).read_bytes()).hexdigest() for s in sources}
    print("Experiment complete; recovery pass",sum(r["passed"] for r in results),"/",len(results))
if __name__ == "__main__":
    main()
