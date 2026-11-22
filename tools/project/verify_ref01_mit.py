"""Windows portable C++ MIT codec + native controller + MuJoCo numerical validation."""
import argparse,csv,json,os,shutil,subprocess,sys
from pathlib import Path
import mujoco
from ref01_mit_fixture import ROOT,Plant,feedback_words,recovery

def main():
    p=argparse.ArgumentParser();p.add_argument("--output",required=True,type=Path);p.add_argument("--damping",type=float,nargs=2,default=[.18,.15])
    a=p.parse_args();out=a.output.resolve();out.mkdir(parents=True,exist_ok=False)
    sdk=Path(mujoco.__file__).parent
    compiler=ROOT/".local/toolchains/zig-0.13.0/ziglang/zig.exe"
    env=dict(os.environ,PYTHONDONTWRITEBYTECODE="1",
             ZIG_GLOBAL_CACHE_DIR=str(ROOT/".local/cache/zig"),ZIG_LOCAL_CACHE_DIR=str(ROOT/".local/cache/zig/local"))
    common=[str(compiler),"c++","-std=c++17","-O2","-DNDEBUG","-Wall","-Wextra","-Werror",
            "-I"+str(ROOT/"ros2_ws/src/mech_controllers/include/mech_controllers"),
            "-I"+str(ROOT/"ros2_ws/src/mech_bringup/include"),
            "-I"+str(ROOT/"ros2_ws/src/mech_protocol_cubemars/include"),
            "-I"+str(ROOT/"mujoco/ref01/cpp"),"-I"+str(sdk/"include")]
    commands=[]
    def call(args,name):
        r=subprocess.run(args,cwd=ROOT,env=env,capture_output=True,text=True,timeout=300)
        (out/(name+".log")).write_text(r.stdout+r.stderr,encoding="utf-8")
        commands.append(dict(name=name,command=args,returncode=r.returncode))
        (out/"commands.json").write_text(json.dumps(commands,indent=2),encoding="utf-8")
        if r.returncode:raise RuntimeError(name+": "+r.stderr[-2500:])
    probe=out/"mit_probe.exe";lease=out/"lease_test.exe"
    call(common+[str(ROOT/"tools/project/ref01_mit_probe.cpp"),
         str(ROOT/"ros2_ws/src/mech_protocol_cubemars/src/ak30_force_wire.cpp"),str(sdk/"mujoco.dll"),"-o",str(probe)],"compile_probe")
    shutil.copy2(sdk/"mujoco.dll",out/"mujoco.dll")
    call(common+[str(ROOT/"tools/project/test_kinematic_lease.cpp"),"-o",str(lease)],"compile_lease")
    call([str(lease)],"lease")
    call([sys.executable,"-B","-m","unittest","discover","-s","tools/project","-p","test_ref01_mit.py","-v"],"contract_tests")
    results={}
    for case in ("psi","beta"):
        plant=Plant();rows=[];torque=[0.,0.]
        error=(out/(case+"_probe.log")).open("w")
        proc=subprocess.Popen([str(probe),"mujoco/ref01/config/mujoco/ref01_dynamics_cad_v1.xml",*map(str,a.damping)],
             cwd=ROOT,stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=error,text=True,bufsize=1)
        try:
            for tick in range(601):
                if tick%4==0:
                    pos,vel=feedback_words(plant.state["alpha"],plant.state["alpha_velocity"])
                proc.stdin.write(" ".join(map(str,pos+vel))+"\n");proc.stdin.flush()
                raw=proc.stdout.readline()
                values=list(map(float,raw.split()))
                if len(values)!=10:raise RuntimeError("native MIT probe failed: "+raw)
                torque=[values[5],values[8]]
                row=plant.advance(torque,tick*.005,case)
                row.update(estimated_psi_rad=values[0],estimated_beta_rad=values[1],
                           requested104_nm=values[4],requested105_nm=values[7],
                           code104=int(values[6]),code105=int(values[9]))
                rows.append(row)
            proc.stdin.close()
            if proc.wait(timeout=10):raise RuntimeError("native probe error")
        finally:
            if proc.poll() is None:proc.kill();proc.wait()
            error.close()
        with (out/(case+".csv")).open("w",newline="") as stream:
            writer=csv.DictWriter(stream,fieldnames=list(rows[0]));writer.writeheader();writer.writerows(rows)
        results[case]=recovery(rows,case)
    result=dict(passed=all(r["passed"] for r in results.values()),cases=results,
       feedback_period_s=.02,controller_period_s=.005,damping=a.damping,production_truncating_codec=True,contract_passed=True,
       boundary="Portable C++ codec/model/control + numerical MuJoCo; no ROS, PTY, physical device or realtime claim.")
    (out/"summary.json").write_text(json.dumps(result,indent=2),encoding="utf-8")
    print(json.dumps(result,indent=2))
    return 0 if result["passed"] else 1
if __name__=="__main__":raise SystemExit(main())
