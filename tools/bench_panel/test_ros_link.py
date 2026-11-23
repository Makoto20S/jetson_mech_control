"""Windows HTTP -> SSH -> ROS software integration. Continuous UI uses native MuJoCo; other cases use private MIT PTY."""
import argparse,json,os,queue,subprocess,sys,threading,time,urllib.request,urllib.error
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/"tools/project"));sys.path.insert(0,str(ROOT/"tools/bench_panel"))
from ros_panel import RosPanel,ros_command
from ph_exo_bench_panel import make_server

def main():
    parser=argparse.ArgumentParser()
    parser.add_argument("--cases",nargs="+",choices=["continuous","lifecycle","no_ack","state_loss","eof","cancel","heartbeat","reset_running","reset_start","reset_no_ack","reset_disconnect","reset_debug"],default=["lifecycle","no_ack","state_loss","eof","cancel","heartbeat"])
    args=parser.parse_args()
    out=ROOT/".local/validation/ros_panel_reset_all_20261114"/("link_"+time.strftime("%H%M%S"))
    out.mkdir(parents=True,exist_ok=False)
    sync=json.loads((ROOT/"config/parallel_exo/orin_sync.json").read_text(encoding="utf-8"))
    results=[]
    for case in args.cases:
        folder=out/case;folder.mkdir()
        log=(folder/"graph.log").open("wb")
        command=ros_command(sync,True)
        command[-1]=command[-1].replace("ph_exo_ros_client.py --software","test_ros_panel_graph.py --case "+("mit_no_ack" if case in ("no_ack","reset_no_ack") else "mit_reset"))
        if case=="continuous":command[-1]=command[-1].replace("--case mit_reset","--case mit_reset --cycle")
        command[-1]=command[-1].replace("exec python3","exec 9>.local/sync.lock; flock -n 9; exec python3")
        graph=subprocess.Popen(command,stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=subprocess.STDOUT)
        messages=queue.Queue()
        def reader():
            for line in iter(graph.stdout.readline,b""):
                log.write(line);log.flush()
                if line.startswith(b"PH_EXO_GRAPH "):messages.put(json.loads(line[len(b"PH_EXO_GRAPH "):]))
        thread=threading.Thread(target=reader,daemon=True);thread.start()
        panel=None;server=None;closed=threading.Event()
        def message(kind,timeout=25):
            end=time.monotonic()+timeout
            while time.monotonic()<end:
                try:value=messages.get(timeout=.1)
                except queue.Empty:
                    if graph.poll() is not None:raise RuntimeError("software graph exited")
                    continue
                if value["type"]==kind:return value
            raise TimeoutError(kind)
        def graph_call(cmd):
            graph.stdin.write((json.dumps(dict(cmd=cmd))+"\n").encode());graph.stdin.flush()
            return message("check")
        try:
            ready=message("ready")
            def spawn(cmd,**kw):
                cmd=list(cmd);cmd[-1]="env ROS_DOMAIN_ID=183 ROS_LOCALHOST_ONLY=1 "+cmd[-1]
                return subprocess.Popen(cmd,**kw)
            panel=RosPanel(software=True,popen=spawn,api=ready["api"],node=ready["node"])
            server=make_server(panel,0)
            threading.Thread(target=server.serve_forever,daemon=True).start()
            def watch():
                while not closed.wait(.2):panel.watchdog()
            threading.Thread(target=watch,daemon=True).start()
            url="http://127.0.0.1:"+str(server.server_port)
            def http(path,body=None):
                req=urllib.request.Request(url+path,data=None if body is None else json.dumps(body).encode(),
                    headers={"X-Panel-Token":server.panel_token,"Content-Type":"application/json"})
                with urllib.request.urlopen(req,timeout=3) as response:return json.load(response)
            def wait(predicate,timeout=10):
                end=time.monotonic()+timeout;current={}
                while time.monotonic()<end:
                    current=http("/state")
                    if predicate(current):return current
                    time.sleep(.05)
                raise AssertionError({k:current.get(k) for k in ("phase","ros","logs","notice")})
            http("/connect",{})
            wait(lambda s:s["connected"] and s["phase"]=="debug")
            first=graph_call("check");assert first["tx"]==0,first
            config=dict(k=[2.4,2.],b=[.18,.15],tau=[.2,.2])
            if case=="continuous":config=dict(k=[20.,20.],b=[.3,.3],tau=[10.,10.],amp_deg=[3.,-2.],period_s=[6.,4.])
            if case=="reset_debug":
                http("/reset",{})
                final=wait(lambda s:s["phase"]=="idle" and s["recover"]=="ready")
                assert not final["connected"] and graph_call("check")["tx"]==0
                http("/connect",{})
                wait(lambda s:s["phase"]=="debug" and s["connected"])
                assert graph_call("check")["tx"]==0
            else:
                http("/start",config)
                if case in ("cancel","reset_start"):
                    http("/reset" if case=="reset_start" else "/estop",{})
                else:
                    running=wait(lambda s:s["phase"]=="running")
                    first_id=running["ros"]["id"]
                    wait(lambda s:len([e for e in s["events"] if e.get("type")=="sample" and len(e["sample"]["feedback"])==2 and len(e["sample"]["motor_target_nm"])==2])>=3)
                if case=="continuous":
                    began=time.monotonic();observed=[]
                    while time.monotonic()-began<13.1:
                        current=http("/state");assert current['phase']=='running',current['ros']
                        if current['ros'].get('q_ref'):observed.append(current['ros']['q_ref'])
                        time.sleep(.025)
                    import math
                    span=[math.degrees(max(r[i] for r in observed)-min(r[i] for r in observed)) for i in (0,1)]
                    assert all(abs(span[i]-[3,2][i])<.01 for i in (0,1)),span
                    (folder/'reference_span.json').write_text(json.dumps(dict(span_deg=span,config=config)))
                if case=="reset_disconnect":
                    old=panel.proc;old.stdin.close()
                    wait(lambda s:old.poll() is not None,16)
                    http("/reset",{})
                elif case in ("reset_running","reset_no_ack"):
                    http("/reset",{})
                elif case=="heartbeat":
                    closed.set()  # Stop desktop heartbeats, leave stdin open.
                elif case=="state_loss":
                    graph_call("fault")
                elif case=="eof":
                    panel.proc.stdin.close()
                elif case not in ("cancel","reset_start"):
                    http("/estop",{})
                if case.startswith("reset_"):
                    assert http("/state")["phase"]=="idle","return did not restore initial UI"
                    # Repeat click is accepted during stop/reset, without a second start.
                    http("/reset",{})
                    final=wait(lambda s:s["phase"]=="idle" and s["recover"] in ("ready","failed"),20)
                    if case=="reset_no_ack":
                        assert final["recover"]=="failed" and not final["can_start"],final
                        assert final["ros"]["disable_ok"]==[True,False],final
                        a=graph_call("check")["tx"]
                        http("/reset",{})
                        final=wait(lambda s:s["recover"]=="failed",15)
                        assert final["phase"]=="idle" and not final["can_start"]
                    else:
                        assert final["recover"]=="ready" and final["can_start"],final
                        if final["connected"]:
                            assert final["ros"]["state"]=="debug" and final["ros"]["stop_ok"],final
                        a=graph_call("check")["tx"]
                    for _ in range(5):http("/state");time.sleep(.05)
                    assert graph_call("check")["tx"]==a,"return sent new torque"
                else:
                    final=wait(lambda s:s["phase"] in ("stopped","unconfirmed"),12)
                    if case=="no_ack":
                        assert final["phase"]=="unconfirmed" and final["ros"]["disable_ok"]==[True,False],final
                        http("/reset",{})
                        refreshed=wait(lambda s:s["recover"]=="failed",15)
                        assert refreshed["phase"]=="idle" and not refreshed["can_start"]
                    else:
                        assert final["phase"]=="stopped" and final["ros"]["stop_ok"],final
                        assert final["ros"]["disable_ok"]==([False,False] if case=="continuous" else [True,True])
                        if case not in ("eof","heartbeat"):
                            http("/reset",{})
                            wait(lambda s:s["phase"]=="idle" and s["recover"]=="ready")
                            a=graph_call("check")["tx"]
                            for _ in range(5):http("/state");time.sleep(.05)
                            assert graph_call("check")["tx"]==a,"reset sent new torque"
                            if case=="lifecycle":
                                http("/start",config);second=wait(lambda s:s["phase"]=="running")
                                assert second["ros"]["id"]!=first_id
                                http("/stop",{});wait(lambda s:s["phase"]=="stopped")
                                http("/reset",{});wait(lambda s:s["phase"]=="idle" and s["recover"]=="ready")
            assert graph_call("check")["manager_alive"]
            results.append(dict(case=case,passed=True,remote=ready["output"],run=panel.run_id,
                final=final["ros"],manager_alive=True,software_only=True,backend="native_mujoco" if case=="continuous" else "mit_pty"))
            (folder/"result.json").write_text(json.dumps(results[-1],indent=2),encoding="utf-8")
            print(json.dumps(dict(case=case,passed=True,remote=ready["output"],run=panel.run_id)),flush=True)
        except Exception:
            if panel:(folder/"failure.json").write_text(json.dumps(panel.snapshot(heartbeat=False),indent=2),encoding="utf-8")
            raise
        finally:
            if panel:
                panel.shutdown()
                if panel.proc:
                    try:panel.proc.wait(timeout=16)
                    except subprocess.TimeoutExpired:pass
            closed.set()
            if server:server.shutdown();server.server_close()
            if graph.poll() is None:
                graph.stdin.write(b'{"cmd":"finish"}\n');graph.stdin.flush()
                graph.wait(timeout=20)
            thread.join(timeout=2);log.close()
    (out/"summary.json").write_text(json.dumps(results,indent=2),encoding="utf-8")
    print(json.dumps(dict(output=str(out),cases=len(results),passed=True)),flush=True)
if __name__=="__main__":main()
