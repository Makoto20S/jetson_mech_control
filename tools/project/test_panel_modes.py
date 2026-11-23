"""First-class GUI routing; no ROS/serial access on Windows."""
import io,json,time
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import Mock,patch
import pytest
import ph_exo_bench_panel as ui
from ph_exo_bench_worker import DEFAULT

@pytest.mark.parametrize("mode,legacy,want",[(None,False,"ros"),("ros",False,"ros"),
 ("bench",False,"bench"),(None,True,"bench"),("bench",True,"bench")])
def test_mode_routing(mode,legacy,want):
    assert ui.resolve_mode(mode,legacy)==want

def test_conflicting_alias_rejected():
    with pytest.raises(ValueError):ui.resolve_mode("ros",True)

def test_bench_factory_does_not_create_upper_client():
    p=ui.create_panel("bench",preview=True,software=True)
    assert isinstance(p,ui.BenchPanel) and not hasattr(p,"ros") and p.proc is None
    assert p.snapshot()["mode"]=="bench" and p.software
    with pytest.raises(ValueError):p.start(DEFAULT)

def test_ros_factory_retains_existing_upper_adapter():
    p=ui.create_panel("ros",preview=True)
    assert hasattr(p,"ros") and p.snapshot()["mode"]=="ros" and p.proc is None

@pytest.mark.parametrize("software,flag",[(False,"--execute-panel"),(True,"--software-test")])
def test_independent_worker_routing(software,flag):
    cmd=ui.ssh_command(dict(ssh_host="orin-nx",remote_root="/safe"),software)
    assert flag in cmd[-1] and "ph_exo_bench_worker.py" in cmd[-1]
    assert "ph_exo_ros_client.py" not in cmd[-1] and "ph_exo_upper" not in cmd[-1]

@pytest.mark.parametrize("phase",["idle","preparing","running","stopping","finished","cancelled","error","unconfirmed"])
def test_independent_return_is_unconditional(phase):
    p=ui.BenchPanel();p.phase=phase
    p.proc=NS(stdin=io.BytesIO(),poll=lambda:None)
    p.reset()
    s=p.snapshot()
    assert s["phase"]=="idle" and s["run_id"] is None and s["refresh"]
    assert not s["can_start"] # Live worker must complete first.
    if phase in ("preparing","running"):
        assert p.proc.stdin.getvalue()==b"STOP\n"
    elif phase=="unconfirmed":assert p.proc.stdin.getvalue()==b"ESTOP\n"

def test_independent_reset_stays_unknown_without_disable_evidence():
    p=ui.BenchPanel();p.phase="unconfirmed";p.proc=NS(poll=lambda:255)
    p.reset();s=p.snapshot()
    assert s["phase"]=="idle" and s["recover"]=="failed" and not s["can_start"]
    with pytest.raises(ValueError):p.start(DEFAULT)

def test_independent_finished_becomes_ready_after_worker_exit():
    alive=[True]
    p=ui.BenchPanel();p.phase="finished";p.proc=NS(poll=lambda:None if alive[0] else 0)
    p.reset()
    assert p.snapshot()["recover"]=="wait"
    alive[0]=False
    assert p.snapshot()["recover"]=="ready" and p.snapshot()["can_start"]

def test_return_does_not_delete_previous_records(tmp_path):
    p=ui.BenchPanel();p.phase="finished";p.proc=NS(poll=lambda:0)
    p.directory=tmp_path;p.result={"result":{"test":True}}
    file=tmp_path/"result.json";file.write_text('{"test":true}')
    p.reset();p.reset()
    assert file.read_text()=='{"test":true}' and p.result["result"]["test"]


def test_failed_start_pose_returns_to_clean_idle(tmp_path, monkeypatch):
    monkeypatch.syspath_prepend(str(Path(__file__).resolve().parents[2] / "mujoco/ref01"))
    import ref01_bench_hold as bench
    from ref01_mujoco_kinematics import KinematicsError, MuJoCoForwardKinematics
    import math
    with pytest.raises(bench.PreflightRejected, match="outside_ref01_branch") as err:
        with bench.read_only_preflight():
            MuJoCoForwardKinematics().solve([math.radians(34.5-45),math.radians(193.7-45)])
    assert isinstance(err.value.__cause__, KinematicsError)
    p=ui.BenchPanel()
    p.phase="preparing";p.directory=tmp_path
    event=dict(type="finished",result=None,error=str(err.value),cancelled_before_start=True)
    proc=NS(stdout=io.BytesIO((ui.PREFIX+json.dumps(event)+"\n").encode()),poll=lambda:1,wait=lambda:1)
    p.proc=proc
    p.read_worker(proc)
    assert p.phase=="cancelled"
    p.reset()
    state=p.snapshot()
    assert state["actual"]=="idle" and state["can_start"] and not state["logs"]
