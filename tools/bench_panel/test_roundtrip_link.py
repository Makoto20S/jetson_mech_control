"""Independent panel -> original worker private PTYs, no upper ROS service."""
import json,os,shlex,subprocess,sys,threading,time,urllib.request
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/"tools/project"));sys.path.insert(0,str(ROOT/"tools/bench_panel"))
from ph_exo_bench_panel import BenchPanel,make_server,ssh_command
from ph_exo_bench_worker import DEFAULT
DEFAULT=dict(DEFAULT,amp_deg=[3.,2.],period_s=[6.,4.])

def main():
    stop_kind=sys.argv[1] if len(sys.argv)>1 else "reset"
    assert stop_kind in ("reset","estop")
    out=ROOT/".local/validation/roundtrip_20261117"/("bench_"+time.strftime("%H%M%S"))
    out.mkdir(parents=True,exist_ok=False)
    sync=json.loads((ROOT/"config/parallel_exo/orin_sync.json").read_text(encoding="utf-8"))
    def spawn(cmd,**kwargs):
        assert "--software-test" in cmd[-1] and "--execute-panel" not in cmd[-1]
        return subprocess.Popen(cmd,**kwargs)
    p=BenchPanel(software=True,popen=spawn)
    server=make_server(p);closed=threading.Event()
    threading.Thread(target=server.serve_forever,daemon=True).start()
    def watch():
        while not closed.wait(.2):p.watchdog()
    threading.Thread(target=watch,daemon=True).start()
    def http(path,body=None):
        req=urllib.request.Request("http://127.0.0.1:"+str(server.server_port)+path,
            data=None if body is None else json.dumps(body).encode(),
            headers={"X-Panel-Token":server.panel_token,"Content-Type":"application/json"})
        with urllib.request.urlopen(req,timeout=3) as r:return json.load(r)
    def wait(test,timeout=30):
        end=time.monotonic()+timeout
        while time.monotonic()<end:
            state=http("/state")
            if test(state):return state
            time.sleep(.1)
        raise AssertionError({k:state.get(k) for k in ("phase","actual","logs","run_id")})
    try:
        assert p.proc is None
        http("/start",DEFAULT)
        running=wait(lambda s:s["phase"]=="running")
        wait(lambda s:len([e for e in s["events"] if e.get("type")=="sample" and
            len(e["sample"].get("motor_target_nm",{}))==2 and len(e["sample"].get("feedback",{}))==2])>=3)
        # Cover two psi and three beta periods using actual ROS reference echoes.
        def cycled(s):
            samples=[e["sample"] for e in s["events"] if e.get("type")=="sample"]
            return len(samples)>220 and samples[-1]["observed_s"]-samples[0]["observed_s"]>=13.
        cycling=wait(cycled,25)
        samples=[e["sample"] for e in cycling["events"] if e.get("type")=="sample"]
        import math
        span=[math.degrees(max(r["reference"][i] for r in samples)-min(r["reference"][i] for r in samples)) for i in (0,1)]
        assert 5.95<span[0]<=6.001 and 1.95<span[1]<=2.001,span
        assert all(r["state_valid"] and r["reason"]=="none" for r in samples[5:]),samples[-1]
        # Isolated domain 181 contains this worker's manager; no upper interface.
        code="""import rclpy,time,json
rclpy.init(args=[])
n=rclpy.create_node('ph_exo_dual_entry_probe')
end=time.monotonic()+1.5
while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.05)
names=n.get_node_names_and_namespaces()
services=[x for x,_ in n.get_service_names_and_types()]
assert any(ns.startswith('/ph_exo_bench_') for name,ns in names),names
assert not any('/exoskeleton/impedance/' in x for x in services),services
print(json.dumps({'nodes':names,'upper_services':[],'domain':181}))
n.destroy_node();rclpy.shutdown()
"""
        cmd=ssh_command(sync,True)
        shell=shlex.split(cmd[-1])[-1]
        shell=shell.replace("exec python3 -B tools/project/ph_exo_bench_worker.py --software-test",
            'export ROS_DOMAIN_ID=181 ROS_LOCALHOST_ONLY=1 ROS_LOG_DIR="$PWD/.local/ros/ref01_native/ros_logs"; exec python3 -B -c '+shlex.quote(code))
        cmd[-1]="bash -lc "+shlex.quote(shell)
        # A short read-only probe runs while panel HTTP polls keep its heartbeat alive.
        probe=subprocess.Popen(cmd,stdout=subprocess.PIPE,stderr=subprocess.PIPE)
        until=time.monotonic()+10
        while probe.poll() is None and time.monotonic()<until:http("/state");time.sleep(.15)
        stdout,stderr=probe.communicate(timeout=3)
        assert probe.returncode==0,stderr.decode()
        graph=json.loads(stdout)
        if stop_kind=="estop":
            http("/estop",{})
            wait(lambda s:s.get("actual",s["phase"])=="finished",35)
        http("/reset",{})
        assert http("/state")["phase"]=="idle"
        http("/reset",{})
        final=wait(lambda s:s.get("recover")=="ready",35)
        assert final["actual"]=="finished",final
        r=http("/result")
        assert r["rehearsal"] and r["disable_ack"]==[104,105] and not r.get("shutdown_error"),r
        done=json.loads((p.directory/"completion.json").read_text())
        assert done["archive"]["status"]=="local_verified_remote_removed",done["archive"]
        import xml.etree.ElementTree as ET
        files=list((p.directory/"remote_raw").rglob("disable.urdf"));assert files
        ports=[x.text for f in files for x in ET.parse(f).findall(".//hardware/param[@name='device_path']")]
        assert ports and all(x.startswith("/dev/pts/") for x in ports),ports
        evidence=dict(passed=True,software_only=True,graph=graph,ports=ports,run=p.run_id,
            local=str(p.directory),remote=done["output"],archive=done["archive"],final=final["actual"],
            disable_ack=r["disable_ack"],parameters=DEFAULT,reference_span_deg=span,stop_kind=stop_kind,ui_phase=final["phase"])
        (out/"result.json").write_text(json.dumps(evidence,indent=2),encoding="utf-8")
        print(json.dumps(evidence),flush=True)
    except BaseException:
        (out/"failure.json").write_text(json.dumps(p.snapshot(heartbeat=False),indent=2),encoding="utf-8")
        raise
    finally:
        p.stop("software test ending")
        if p.proc:
            try:p.proc.wait(timeout=35)
            except subprocess.TimeoutExpired:raise RuntimeError("Software worker exit not confirmed")
        closed.set();server.shutdown();server.server_close()

if __name__=="__main__":main()
