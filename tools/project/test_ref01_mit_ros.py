"""Official controller_manager + two existing MIT/CDC stacks + private MuJoCo PTYs."""
import argparse,hashlib,json,os,sys,traceback
from pathlib import Path
from ref01_mit_fixture import ROOT,CASES,MitFixture
from test_ref01_manager import run_case,save

def parse_args(argv=None):
    p=argparse.ArgumentParser();p.add_argument("--output",type=Path,required=True)
    p.add_argument("--duration",type=float,default=3.)
    p.add_argument("--cases",nargs="+",choices=CASES,default=list(CASES))
    p.add_argument("--predictive",action="store_true")
    p.add_argument("--typed",action="store_true",help="Use fixed_pose_v1; ordinary controller only")
    a=p.parse_args(argv)
    if a.typed and a.predictive:p.error("--typed only supports ordinary fixed-pose control")
    if not 3 <= a.duration <= 30:p.error("duration must be finite and 3..30s")
    return a

def main():
    a=parse_args()
    if sys.platform!="linux" or os.environ.get("EXO_SYNC_LOCK_HELD")!="1":
        raise RuntimeError("Orin with held sync.lock required")
    if (ROOT/".local/sync/incomplete.json").exists(): raise RuntimeError("interrupted sync")
    if os.environ.get("ROS_DOMAIN_ID")!="179" or os.environ.get("ROS_LOCALHOST_ONLY")!="1":
        raise RuntimeError("isolated ROS domain 179 required")
    out=a.output.resolve()
    if not out.is_relative_to(ROOT/".local/validation"):raise ValueError("project validation output required")
    if any(p.is_symlink() for p in [a.output,*a.output.parents]):raise ValueError("output symlink")
    if a.duration<3 or a.duration>30:raise ValueError("duration must be 3..30s")
    out.mkdir(parents=True,exist_ok=False)
    sources=["tools/project/"+n for n in ("test_ref01_mit_ros.py","ref01_mit_fixture.py","test_ref01_manager.py","mit_communication.py")]
    sources += ["ros2_ws/src/mech_bringup/"+n for n in ("src/ak30_kinematic_system.cpp","include/mech_bringup/ak30_kinematic_lease.hpp","src/ak30_system.cpp")]
    sources += ["ros2_ws/src/mech_controllers/src/ref01_impedance_controller.cpp","ros2_ws/src/mech_protocol_cubemars/src/ak30_force_wire.cpp"]
    sources += ["mujoco/ref01/config/mujoco/ref01_dynamics_cad_v1.xml","mujoco/ref01/cpp/ref01_model.hpp","ros2_ws/src/mech_controllers/include/mech_controllers/ref01_impedance_core.hpp"]
    sources += [str(p.relative_to(ROOT)).replace("\\","/") for p in
                (ROOT/"ros2_ws/src/mech_controllers/include/mech_controllers/ref01_predictive").glob("*.hpp")]
    sources += ["ros2_ws/src/mech_protocol_cubemars/src/ak30_force_session.cpp",
                "ros2_ws/src/mech_bringup/src/ak30_force_runtime.cpp",
                "ros2_ws/src/mech_controllers/include/mech_controllers/ref01_impedance_controller.hpp"]
    sources += ["ros2_ws/src/ph_exo_interfaces/msg/FixedPosePlan.msg",
                "ros2_ws/src/ph_exo_interfaces/msg/ImpedanceStatus.msg"]
    sources += ["tools/project/ref01_stale_validation.py"]
    save(out/"invocation.json",dict(typed=a.typed,predictive=a.predictive,duration=a.duration,cases=a.cases))
    save(out/"source_hashes.json",{s:hashlib.sha256((ROOT/s).read_bytes()).hexdigest() for s in sources})
    results=[]
    for case in a.cases:
        fixture=MitFixture(predictive=a.predictive)
        try:
            result=run_case(out/case,case,a.duration if case in ("psi","beta") else 3.,fixture,typed=a.typed)
        except Exception:
            fixture.close()
            result=dict(case=case,passed=False,error=traceback.format_exc())
            (out/case).mkdir(parents=True,exist_ok=True)
            save(out/case/"summary.json",result)
        results.append(result)
        print(json.dumps(result,allow_nan=False),flush=True)
        save(out/"summary.json",dict(passed=all(r["passed"] for r in results),cases=results))
    return 0 if all(r["passed"] for r in results) else 1
if __name__=="__main__":raise SystemExit(main())
