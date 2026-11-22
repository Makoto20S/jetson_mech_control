"""Portable regression for the ROS panel, with no ROS or devices."""
import io
import json
import math
from pathlib import Path
import sys
import tempfile
import time
from types import SimpleNamespace as NS
from unittest.mock import Mock,patch
import pytest
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/"tools/bench_panel"))
import ros_panel
import ph_exo_ros_client as client

@pytest.mark.parametrize("key,bad",[("k",[0,1]),("k",[31,1]),("b",[-1,0]),("b",[0,2]),
                                    ("tau",[11,1]),("tau",[1]),("tau",[True,1]),
                                    ("k",[math.nan,1]),("b",["1",0])])
def test_invalid_config(key,bad):
    c={k:list(v) for k,v in client.DEFAULT.items()};c[key]=bad
    with pytest.raises(ValueError):client.validate_config(c)

def test_short_fields_only():
    assert client.validate_config(client.DEFAULT)==client.DEFAULT
    with pytest.raises(ValueError):client.validate_config(dict(client.DEFAULT,current_a=6))
    with pytest.raises(ValueError):client.route("/path; command")

def panel():
    p=ros_panel.RosPanel(software=True,popen=Mock(side_effect=OSError("test connection unavailable")))
    p.proc=NS(stdin=io.BytesIO(),poll=lambda:None)
    p.ready=True;p.ros=dict(state="debug",stop_ok=False,disable_ok=[False,False],sim=True,backend="mit_pty")
    p.phase="debug";p.last_rx=time.monotonic();return p

def test_preview_no_connection():
    spawn=Mock();p=ros_panel.RosPanel(preview=True,popen=spawn)
    with pytest.raises(ValueError):p.connect()
    with pytest.raises(ValueError):p.start(client.DEFAULT)
    spawn.assert_not_called()

def test_ssh_only_calls_ros_client():
    cmd=ros_panel.ros_command(dict(ssh_host="orin-nx",remote_root="/project ' quoted"),True)
    assert "--software" in cmd[-1] and "ph_exo_ros_client.py" in cmd[-1]
    assert "ref01_bench_hold.py" not in cmd[-1] and "ros2_control_node" not in cmd[-1]
    assert "StrictHostKeyChecking=yes" in cmd

def test_begin_waits_for_ros_debug():
    p=panel();p.ready=False;p.pending=client.DEFAULT;p.maybe_start()
    assert p.proc.stdin.getvalue()==b""
    p.ready=True;p.maybe_start()
    m=json.loads(p.proc.stdin.getvalue());assert m["cmd"]=="start" and m["config"]==client.DEFAULT
    assert p.phase=="preparing"

@pytest.mark.parametrize("command,expected",[("stop","stop"),("emergency_stop","estop")])
def test_actions_are_ros_requests(command,expected):
    p=panel();getattr(p,command)()
    assert json.loads(p.proc.stdin.getvalue())["cmd"]==expected
    assert not p.ros["stop_ok"]

def test_failed_reset_does_not_unlock():
    p=panel();p.phase="unconfirmed";p.control=True
    p.reset()
    assert p.snapshot(heartbeat=False)["phase"]=="idle"
    assert p.control and p.recover=="stop"
    with pytest.raises(ValueError):p.start(client.DEFAULT)

def test_reset_needs_positive_ros_reply():
    p=panel();p.control=True;p.phase="running"
    p.ros=dict(p.ros,state="running")
    p.reset()
    assert p.snapshot(heartbeat=False)["phase"]=="idle" and p.recover=="stop"
    p.handle(dict(type="reply",cmd="stop",seq=p.seq,ok=True,message="accepted"))
    p.handle(dict(type="state",status=dict(p.ros,state="stopped",stop_ok=True)))
    assert p.phase=="resetting" and p.recover=="reset"
    assert not p.snapshot(heartbeat=False)["can_start"]
    p.handle(dict(type="reply",cmd="reset",seq=p.seq,ok=True,message="debug"))
    assert p.phase=="debug" and not p.owns and not p.control
    assert p.snapshot(heartbeat=False)["phase"]=="idle" and p.recover=="ready"
    assert b'"cmd": "start"' not in p.proc.stdin.getvalue()


def test_unknown_reply_stays_latched_until_stop():
    p=panel();p.handle(dict(type="reply",cmd="start",seq=1,ok=False,message="ROS服务结果未确认"))
    p.handle(dict(type="state",status=dict(p.ros,state="debug")))
    assert p.phase=="unconfirmed"
    p.handle(dict(type="state",status=dict(p.ros,state="stopped",stop_ok=True)))
    assert p.phase=="stopped"

def test_readonly_close_does_not_request_stop():
    p=panel()
    p.proc.stdin=Mock()
    p.shutdown()
    assert json.loads(p.proc.stdin.write.call_args.args[0])["cmd"]=="close"

def test_plot_order_units_and_missing():
    now=100.
    c=NS(observed_at_monotonic_s=now,position_rad=[1,2],velocity_rad_s=[3,4],
         reference_rad=[5,6],mode=2,reason="",state_valid=True,motor_target_nm=[.1,-.2])
    m=NS(valid=True,age_ms=1,drive_id=104,position_deg=22,output_velocity_rad_s=math.pi,
         output_velocity_valid=True,erpm=5000,iq_a=1)
    r=NS(sources=[NS(valid=True,source_age_ms=2)],mit_motors=[m])
    s=client.sample(c,r,now)
    assert s["motor_target_nm"]=={"104":.1,"105":-.2}
    assert s["feedback"]["104"]["rpm"]==pytest.approx(30)
    assert "105" not in s["feedback"]
    assert client.sample(c,None,now+1)["motor_target_nm"]=={}

def test_nonfinite_stays_missing():
    assert client.clean(dict(x=float("nan"),y=[float("inf"),1]))==dict(x=None,y=[None,1])

def test_simulation_stop_is_not_real_disable():
    p=panel()
    p.handle(dict(type="state",status=dict(p.ros,state="stopped",stop_ok=True)))
    assert p.phase=="stopped" and p.ros["disable_ok"]==[False,False]

def test_historical_script_files_are_not_used_by_client():
    source=Path(client.__file__).read_text(encoding="utf-8")
    assert "servo_disable" not in source and "subprocess" not in source
    assert "SetParametersAtomically" in source

def test_start_receipt_cannot_revert_to_old_debug():
    p=panel();p.pending=client.DEFAULT;p.maybe_start()
    p.handle(dict(type="reply",cmd="start",seq=p.seq,ok=True,message="accepted"))
    p.handle(dict(type="state",status=dict(p.ros,state="debug")))
    assert p.phase=="preparing"
    p.handle(dict(type="state",status=dict(p.ros,state="running")))
    assert p.phase=="running"

def test_explicit_retry_updates_stop_evidence(tmp_path):
    p=panel();p.directory=tmp_path;p.owns=True
    p.ros=dict(p.ros,state="unconfirmed",id=1,reason="missing",stop_ok=False,disable_ok=[True,False])
    p.finish()
    assert not json.loads((tmp_path/"result.json").read_text())["stop_ok"]
    p.ros=dict(p.ros,state="stopped",reason="retry",stop_ok=True,disable_ok=[True,True])
    p.finish()
    assert json.loads((tmp_path/"result.json").read_text())["stop_ok"]
    before=(tmp_path/"result.json").read_bytes()
    p.owns=False;p.ros=dict(p.ros,reason="other operation");p.finish()
    assert (tmp_path/"result.json").read_bytes()==before


@pytest.mark.parametrize("end",["closed","eof","spawn_error","stale"])
def test_readonly_connection_failure_can_return_to_local_debug(end):
    p=panel();p.ready=False;p.phase="connecting";p.ros={}
    proc=p.proc
    if end=="closed":
        p.handle(dict(type="closed",status={}))
    elif end=="eof":
        proc.stdout=io.BytesIO();proc.wait=lambda:1
        p.read_worker(proc)
    elif end=="spawn_error":
        p.proc=None;p.popen=Mock(side_effect=OSError("SSH unavailable"))
        with pytest.raises(OSError):p.connect()
    else:
        p.ready=True;p.ros=dict(state="debug");p.last_rx=0
        p.watchdog()
    assert p.phase=="connect_error"
    assert p.snapshot(heartbeat=False)["link_reset"]
    p.reset()
    assert p.phase=="idle" and not p.ready and p.ros=={} and not p.owns
    assert p.events[-1]["type"]=="ros_reset" and p.events[-1]["local"]
    assert not p.snapshot(heartbeat=False)["connected"]

def test_return_during_connect_cancels_pending_start_and_only_closes_client():
    p=panel();p.phase="connecting";p.ready=False;p.ros={}
    p.pending=client.DEFAULT;p.owns=True
    p.proc.stdin=Mock();stream=p.proc.stdin
    p.reset()
    assert p.pending is None and p.proc is None and not p.owns
    assert [json.loads(c.args[0])["cmd"] for c in stream.write.call_args_list]==["close"]
    stream.close.assert_called_once()

def test_old_client_cannot_overwrite_reset_or_new_connection():
    p=panel();old=p.proc
    old.stdout=io.BytesIO((client.PREFIX+json.dumps(dict(type="closed",status={}))+ "\n").encode())
    old.wait=lambda:0
    p.phase="connecting";p.ros={};p.reset()
    p.read_worker(old)
    assert p.phase=="idle" and not p.ros
    # A stale client's late exception also cannot break a replacement connection.
    old.stdout=NS(readline=Mock(side_effect=OSError("late EOF")))
    p.proc=NS(stdin=io.BytesIO(),poll=lambda:None);p.phase="debug";p.ready=True
    p.read_worker(old)
    assert p.phase=="debug" and p.ready

def test_sent_start_remains_unconfirmed_after_connection_failure():
    p=panel();p.send("start",config=client.DEFAULT)
    p.link_failed("lost")
    assert p.phase=="unconfirmed" and not p.can_reset_link()
    p.reset()
    assert p.snapshot(heartbeat=False)["phase"]=="idle"
    with pytest.raises(ValueError):p.start(client.DEFAULT)
    assert p.control

@pytest.mark.parametrize("state",["running","stopping","unconfirmed"])
def test_observed_remote_activity_cannot_be_cleared_locally(state):
    p=panel();p.handle(dict(type="state",status=dict(p.ros,state=state)))
    p.link_failed("lost")
    assert p.phase=="unconfirmed" and not p.can_reset_link()
    p.reset()
    assert p.snapshot(heartbeat=False)["phase"]=="idle" and p.control
    with pytest.raises(ValueError):p.start(client.DEFAULT)

def test_no_empty_sample_is_displayed_as_live_motor_data():
    p=panel();p.ros={};p.handle(dict(type="sample",sample={"q":[],"feedback":{}}))
    assert not p.samples and not p.events

def test_final_remote_unconfirmed_state_cannot_be_treated_as_connect_failure():
    p=panel();p.ros={}
    p.handle(dict(type="closed",status={"state":"unconfirmed","stop_ok":False}))
    assert p.phase=="unconfirmed" and not p.can_reset_link()
    p.reset()
    assert p.snapshot(heartbeat=False)["phase"]=="idle"
    assert p.control and not p.snapshot(heartbeat=False)["can_start"]


@pytest.mark.parametrize("phase",["idle","connecting","connect_error","debug","preparing",
                                  "starting","running","stopping","resetting","stopped","unconfirmed","error"])
def test_reset_is_always_accepted_and_returns_initial_view(phase):
    p=panel();p.phase=phase
    if phase in {"preparing","starting","running","stopping","resetting","stopped","unconfirmed"}:
        p.control=True;p.ros=dict(p.ros,state="unconfirmed" if phase=="resetting" else phase)
    else:p.ready=False;p.ros={}
    p.reset();state=p.snapshot(heartbeat=False)
    assert state["phase"]=="idle" and state["run_id"] is None and state["refresh"]
    assert p.pending is None and not p.await_start

def test_unavailable_stop_before_any_start_does_not_latch_control():
    p=panel();p.ready=False;p.ros={};p.phase="connecting"
    p.stop()
    p.handle(dict(type="reply",cmd="stop",seq=p.seq,ok=False,message="ROS服务不可用"))
    p.handle(dict(type="closed",status={}))
    assert not p.control and p.can_reset_link()
    p.reset()
    assert p.recover=="ready" and p.snapshot(heartbeat=False)["phase"]=="idle"

def test_repeated_reset_is_idempotent_and_never_sends_start():
    p=panel();p.control=True;p.ros=dict(p.ros,state="running")
    p.reset();p.reset();p.reset()
    cmds=[json.loads(line)["cmd"] for line in p.proc.stdin.getvalue().splitlines()]
    assert cmds==["stop"] and p.recover=="stop"

def test_failed_reset_still_allows_page_refresh_but_not_motor_restart():
    p=panel();p.control=True;p.ros=dict(p.ros,state="running")
    p.reset()
    p.handle(dict(type="reply",cmd="stop",seq=p.seq,ok=True,message="accepted"))
    p.handle(dict(type="state",status=dict(p.ros,state="unconfirmed",stop_ok=False)))
    assert p.recover=="failed" and p.snapshot(heartbeat=False)["phase"]=="idle"
    with pytest.raises(ValueError):p.start(client.DEFAULT)
    p.reset()
    assert p.recover=="stop" and p.snapshot(heartbeat=False)["phase"]=="idle"

def test_cached_stopped_flag_cannot_skip_new_stop_confirmation():
    p=panel();p.control=True;p.ros=dict(p.ros,state="stopped",stop_ok=True)
    p.reset()
    p.handle(dict(type="reply",cmd="stop",seq=p.seq,ok=True,message="accepted"))
    p.advance_reset()
    assert p.recover=="wait" and b'"cmd": "reset"' not in p.proc.stdin.getvalue()
    p.handle(dict(type="state",status=dict(p.ros,state="stopped",stop_ok=True)))
    assert p.recover=="reset"

def test_reconnect_failure_returns_initial_view_without_faking_stop():
    p=panel();p.control=True;p.proc=None;p.ready=False;p.phase="unconfirmed";p.ros={}
    p.reset()
    s=p.snapshot(heartbeat=False)
    assert s["phase"]=="idle" and p.recover=="failed" and not s["can_start"]
    assert not p.ros.get("stop_ok") and p.control

def test_recovery_timeout_does_not_disable_return_button_or_clear_motor_state():
    p=panel();p.control=True;p.ros=dict(p.ros,state="running");p.reset()
    p.recover_at=time.monotonic()-17;p.watchdog()
    assert p.recover=="failed" and p.snapshot(heartbeat=False)["phase"]=="idle" and p.control


def test_trajectory_to_si_and_legacy_start_clears_previous_cycle():
    c=dict(client.DEFAULT,pose_deg=[1.,20.],move_s=3,amp_deg=[3.,-2.],period_s=[6.,4.])
    p=client.parameters(c)
    assert p['ref']=='cycle' and p['target']
    assert p['q']==pytest.approx([math.radians(1),math.radians(20)])
    assert p['amp']==pytest.approx([math.radians(3),math.radians(-2)])
    assert type(p['move']) is float
    p=client.parameters(client.DEFAULT)
    assert p['ref']=='fixed' and not p['target'] and p['amp']==[0.,0.]

@pytest.mark.parametrize('extra',[
    dict(amp_deg=[1.,2.]),dict(pose_deg=[0.,20.]),dict(move_s=3),
    dict(amp_deg=[True,2],period_s=[6,6]),dict(amp_deg=[0,0],period_s=[6,6]),
    dict(pose_deg=[0,0],move_s=3,amp_deg=[0,-9],period_s=[6,6]),
    dict(pose_deg=[0,20],move_s=True)])
def test_invalid_trajectory_request(extra):
    with pytest.raises(ValueError):client.validate_config(dict(client.DEFAULT,**extra))
