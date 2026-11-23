"""Windows-portable boundary tests for the ROS adapter (ROS endpoints replaced)."""
import importlib.util
import json
import sys
import threading
import time
from pathlib import Path
from types import ModuleType, SimpleNamespace as NS
from unittest.mock import Mock, patch

import pytest

from mech_assist_control.robot.ph_exo_session import FixedPoseSession


def load_adapter():
    modules = {}
    definitions = {
        "std_msgs.msg": ("String",),
        "std_srvs.srv": ("Trigger",),
        "ph_exo_interfaces.msg": ("FixedPosePlan", "ImpedanceRef", "ImpedanceStatus"),
        "mech_exo_state.msg": ("RobotState",),
        "controller_manager_msgs.srv": ("SwitchController", "SetHardwareComponentState",
                                        "ListHardwareComponents", "ListControllers"),
        "rcl_interfaces.srv": ("GetParameters",),
        "rcl_interfaces.msg": ("SetParametersResult",),
    }
    class Message(NS):
        pass
    class Request(NS):
        STRICT = 2
    for name, classes in definitions.items():
        module = ModuleType(name)
        for cls in classes:
            setattr(module, cls, type(cls, (Message,), {"Request": Request}))
        modules[name] = module
    modules["rclpy.qos"] = NS(qos_profile_sensor_data=object())
    modules["ament_index_python.packages"] = NS(get_package_prefix=lambda _: "/fake")
    path = Path(__file__).parents[1] / "mech_assist_control/ph_exo_adapter.py"
    spec = importlib.util.spec_from_file_location("mech_assist_control._adapter_test", path)
    adapter = importlib.util.module_from_spec(spec)
    with patch.dict(sys.modules, modules):
        spec.loader.exec_module(adapter)
    return adapter


module = load_adapter()


@pytest.fixture
def adapter():
    a = module.PhExoAdapter.__new__(module.PhExoAdapter)
    a.lock = threading.RLock()
    a.operation_lock = threading.Lock()
    a.session = FixedPoseSession()
    a.cancel = threading.Event()
    a.worker = None
    a.state = None
    a.controller_state = None
    a.renew = False
    a.started_at = 0.
    a.last_status_at = 0.
    a.last_state_stamp = 0
    a.manager = "/test/controller_manager"
    a.controller = "/test/ref01_impedance"
    a.controller_name = "ref01_impedance"
    a.hardware = ["pair"]
    a.backend = "simulation"
    a.source="fixed_pose_v1";a.profile="";a.plan_topic=a.controller+"/fixed_pose_plan"
    a.trajectory=None;a.ref_cfg=dict(module.reference.DEFAULT);a.q_ref=None;a.dq_ref=None
    a.node = NS(count_publishers=lambda _: 1,
                get_clock=lambda: NS(now=lambda: NS(nanoseconds=1_000_000_000)))
    a.plan_pub = Mock()
    a.plan_pub.get_subscription_count.return_value = 1
    a.raw_pub = Mock()
    a.status_pub = Mock()
    a.ctrl_stop = Mock()
    a.ctrl_stop.service_is_ready.return_value = True
    return a


def running(a, age=0.):
    now = time.monotonic()
    a.session.start([0., .2], [2.4, 2., .18, .15], [.2, .2], now)
    a.session.phase = "running"
    a.started_at = now - age
    a.state = ([0., .2], now, True)
    a.last_status_at = now
    a.controller_state = NS(mode=2, session_id=a.session.session_id)
    a.renew = True


def state(stamp=990_000_000, valid=True):
    return NS(sources=[NS(source_stamp=NS(sec=stamp//10**9, nanosec=stamp%10**9),
                         valid=valid, source_age_ms=10., receive_age_ms=1.)],
              q_valid=valid, q_velocity_valid=valid, mapping_confirmed=valid,
              q_position=[0., .2], q_velocity=[0., 0.])


def test_replay_does_not_refresh_source(adapter):
    adapter.on_state(state())
    before = adapter.state
    adapter.on_state(state())
    assert adapter.state is before
    adapter.on_state(state(980_000_000))
    assert adapter.state is before


@pytest.mark.parametrize("stamp", [1_100_000_000, 800_000_000])
def test_invalid_source_time(adapter, stamp):
    adapter.on_state(state(stamp))
    assert not adapter.state[2]


def test_invalid_state_clears_previous(adapter):
    adapter.on_state(state())
    adapter.on_state(state(valid=False))
    assert adapter.state is None


def test_following_only_own_session(adapter):
    running(adapter)
    adapter.session.phase = "starting"
    t = time.monotonic()
    message = NS(observed_at_monotonic_s=t, mode=2, state_valid=True, session_id=1)
    adapter.on_status(message)
    assert adapter.session.phase == "starting"
    message = NS(observed_at_monotonic_s=t+.01, mode=2, state_valid=True,
                 session_id=adapter.session.session_id)
    adapter.on_status(message)
    assert adapter.session.phase == "running"
    adapter.on_status(NS(observed_at_monotonic_s=t, mode=3))
    assert adapter.controller_state is message


@pytest.mark.parametrize("fault", ["stale", "wrong_session", "waiting", "second_publisher"])
def test_tick_revokes_on_bad_state_or_route(adapter, fault):
    running(adapter, age=2.)
    if fault == "stale":
        adapter.state = ([0., .2], time.monotonic()-.2, True)
    if fault == "wrong_session":
        adapter.controller_state.session_id += 1
    if fault == "waiting":
        adapter.controller_state.mode = 1
    if fault == "second_publisher":
        adapter.node.count_publishers = lambda _: 2
    adapter.request_stop = Mock()
    adapter.tick()
    adapter.request_stop.assert_called_once()
    adapter.plan_pub.publish.assert_not_called()


def test_debug_never_publishes(adapter):
    adapter.tick()
    adapter.plan_pub.publish.assert_not_called()


def test_route_rejects_competing_publisher(adapter):
    adapter.node.count_publishers = lambda _: 2
    with pytest.raises(RuntimeError):
        adapter.check_plan_route()


def test_route_can_be_cancelled(adapter):
    adapter.plan_pub.get_subscription_count.return_value = 0
    adapter.cancel.set()
    with pytest.raises(RuntimeError):
        adapter.check_plan_route()


def stop_fixture(a):
    a.validate_targets = lambda: {"pair": NS(state=NS(id=3))}
    a.ctrl_list = object()
    a.call = lambda *args: NS(controller=[NS(name=a.controller_name, state="active", type="mech_controllers/Ref01ImpedanceController")])
    a.switch_controller = Mock()
    a.hardware_state = Mock()
    a.session.stop("test")


def test_simulation_stop_does_not_claim_disable_ack(adapter):
    stop_fixture(adapter)
    adapter._stop_work()
    adapter.switch_controller.assert_called_once_with(False)
    adapter.hardware_state.assert_called_once_with("pair", 2)
    assert adapter.session.phase == "stopped"
    assert adapter.session.stop_confirmed
    assert adapter.session.disable_confirmed == [False, False]


@pytest.mark.parametrize("acks", [[104,105], [104], []])
def test_mit_cleanup_precedes_independent_ack(adapter, tmp_path, acks):
    stop_fixture(adapter)
    adapter.backend = "mit_pty"
    adapter.output = str(tmp_path)
    adapter.disable_urdf = str(tmp_path/"disable.urdf")
    def disable(*args, **kwargs):
        assert adapter.hardware_state.call_args_list == [
            (( "pair", 2),), (( "pair", 1),)]
        lines = [dict(event="disable_ack", id=i) for i in acks]
        lines += [dict(event="disable_complete", confirmed=len(acks)==2,
                       missing=[] if len(acks)==2 else [105])]
        return NS(returncode=0, stdout="\n".join(map(json.dumps, lines)), stderr="")
    with patch.object(module.subprocess, "run", side_effect=disable):
        adapter._stop_work()
    assert adapter.session.stop_confirmed == (len(acks)==2)
    assert (adapter.session.phase=="stopped") == (len(acks)==2)
    if len(acks)!=2:
        with pytest.raises(ValueError):
            adapter.session.reset()


def test_hardware_failure_prevents_reset(adapter):
    stop_fixture(adapter)
    adapter.hardware_state.side_effect = RuntimeError("unconfirmed")
    adapter._stop_work()
    assert adapter.session.phase == "unconfirmed"
    result = adapter.reset(None, NS())
    assert not result.success


def test_stop_revokes_before_slow_worker_and_serializes(adapter):
    running(adapter)
    entered = threading.Event()
    release = threading.Event()
    def slow():
        entered.set()
        release.wait(2.)
    adapter._stop_work = slow
    try:
        adapter.request_stop("急停")
        assert entered.wait(1.)
        first_worker = adapter.worker
        assert not adapter.renew and adapter.cancel.is_set()
        assert adapter.session.phase == "stopping"
        adapter.request_stop("重复急停")
        assert adapter.worker is first_worker
        adapter.plan_pub.publish.assert_not_called()
        assert not adapter.reset(None, NS()).success
    finally:
        release.set()
        adapter.worker.join(2.)


def test_reset_never_enables_or_renews(adapter):
    running(adapter)
    stop_fixture(adapter)
    adapter._stop_work()
    adapter.renew = False
    assert adapter.reset(None, NS()).success
    assert adapter.session.phase == "debug"
    adapter.tick()
    assert not adapter.renew
    adapter.plan_pub.publish.assert_not_called()


def test_source_plugin_mismatch_rejected(adapter):
    adapter.hw_list = "hardware"
    adapter.call = lambda *args: NS(component=[NS(name="pair", class_type="real_unknown")])
    with pytest.raises(RuntimeError, match="类型"):
        adapter.validate_targets()


def test_unknown_shutdown_does_not_retry_disable(adapter):
    adapter.session.phase = "unconfirmed"
    adapter.request_stop = Mock()
    adapter._stop_work = Mock()
    adapter.close()
    adapter.stop_work()
    adapter.request_stop.assert_not_called()
    adapter._stop_work.assert_not_called()


@pytest.mark.parametrize("error", ["none", "bus", "id", "plugin", "physical_pty", "check"])
def test_disable_file_matches_manager_before_activation(adapter, tmp_path, error):
    import xml.etree.ElementTree as ET
    manager = ET.Element("robot")
    disable = ET.Element("robot")
    adapter.hardware = ["motor104", "motor105"]
    for mid in (104, 105):
        control = ET.SubElement(manager, "ros2_control", name="motor"+str(mid))
        hw = ET.SubElement(control, "hardware")
        for k, v in dict(drive_id=mid, device_path="/dev/ttyACM"+str(mid),
                         logical_bus=mid-103).items():
            ET.SubElement(hw, "param", name=k).text = str(v)
        control = ET.SubElement(disable, "ros2_control", name="disable"+str(mid))
        hw = ET.SubElement(control, "hardware")
        ET.SubElement(hw, "plugin").text = ("wrong" if error=="plugin" else "mech_bringup/Ak30ServoSystem")
        ET.SubElement(hw, "param", name="device_path").text = "/dev/ttyACM"+str(mid)
        ET.SubElement(hw, "param", name="logical_bus").text = str(mid-102 if error=="bus" else mid-103)
        joint = ET.SubElement(control, "joint")
        ET.SubElement(joint, "param", name="drive_id").text = str(106 if error=="id" else mid)
    path = tmp_path/"disable.urdf"
    path.write_text(ET.tostring(disable, encoding="unicode"))
    adapter.disable_urdf = str(path)
    adapter.output = str(tmp_path)
    adapter.parameters = object()
    adapter.call = lambda *args: NS(values=[NS(string_value=ET.tostring(manager, encoding="unicode"))])
    adapter.backend = "mit_pty" if error=="physical_pty" else "mit"
    with patch.object(module.subprocess, "run", return_value=NS(returncode=1 if error=="check" else 0)):
        if error=="none":
            adapter.validate_disable_scope()
        else:
            with pytest.raises((RuntimeError,ValueError)):
                adapter.validate_disable_scope()

@pytest.mark.parametrize("mismatch", ["type", "missing", "namespace"])
def test_wrong_controller_cannot_be_stopped(adapter, mismatch):
    stop_fixture(adapter)
    if mismatch=="type":
        adapter.call=lambda *args:NS(controller=[NS(name=adapter.controller_name,state="active",type="other")])
    elif mismatch=="missing":
        adapter.call=lambda *args:NS(controller=[])
    else:
        adapter.controller="/another/ref01_impedance"
    adapter._stop_work()
    assert adapter.session.phase=="unconfirmed"
    adapter.switch_controller.assert_not_called()
    adapter.hardware_state.assert_not_called()


def test_dynamic_tick_keeps_session_and_gains(adapter):
    running(adapter)
    adapter.source="dynamic_ref_v1"
    adapter.plan_topic=adapter.controller+"/ref"
    adapter.ref_cfg=dict(module.reference.DEFAULT,ref="cycle",amp=[.05,-.03],period=[6.,4.])
    adapter.trajectory=module.reference.create(adapter.ref_cfg)
    now=adapter.started_at+2.5
    adapter.state=([0.,.2],now,True);adapter.last_status_at=now
    with patch.object(module.time,"monotonic",return_value=now):adapter.tick()
    packet=adapter.plan_pub.publish.call_args.args[0]
    assert packet.id==adapter.session.session_id and packet.seq==1 and packet.on
    assert packet.q[0]==pytest.approx(.05)
    assert packet.k==[2.4,2.] and packet.b==[.18,.15] and packet.tau==[.2,.2]
    assert adapter.snapshot()['q_ref']==packet.q
    assert adapter.session.reference==(0.,.2)

def test_dynamic_invalid_endpoint_stops_without_publish(adapter):
    running(adapter)
    adapter.source="dynamic_ref_v1"
    adapter.trajectory=module.reference.RoundTrip([0.,-.4],[6.,6.])
    adapter.request_stop=Mock()
    adapter.tick()
    adapter.request_stop.assert_called_once()
    adapter.plan_pub.publish.assert_not_called()

def test_fixed_route_rejects_dynamic_config(adapter):
    c={"hold."+k:v for k,v in dict(module.reference.DEFAULT,ref="cycle",amp=[.05,0.]).items()}
    with pytest.raises(ValueError,match="dynamic_ref_v1"):adapter.validate_reference(c)
    adapter.source="dynamic_ref_v1"
    assert adapter.validate_reference(c)['ref']=='cycle'

def test_running_cannot_change_reference(adapter):
    running(adapter)
    result=adapter.change_parameters([NS(name="hold.amp",value=[.1,.1])])
    assert not result.successful


def test_expiry_recovery_never_activates_hardware(adapter):
    adapter.backend="mit_pty"
    adapter.validate_targets=lambda:{"pair":NS(state=NS(id=1))}
    adapter.call=lambda *a:NS(success=True)
    adapter.hardware_state=Mock();adapter.switch_controller=Mock()
    adapter.validate_controller=lambda:NS(state="inactive")
    adapter.deactivate_after_fault()
    adapter.hardware_state.assert_called_once_with("pair",2)
    adapter.switch_controller.assert_called_once_with(False)

@pytest.mark.parametrize("state",[2,3,4])
def test_recovery_rejects_other_hardware_states(adapter,state):
    adapter.backend="mit_pty"
    adapter.validate_targets=lambda:{"pair":NS(state=NS(id=state))}
    adapter.hardware_state=Mock()
    with pytest.raises(RuntimeError):adapter.deactivate_after_fault()
    adapter.hardware_state.assert_not_called()

def test_recovery_requires_stop_latch_ack(adapter):
    adapter.backend="mit_pty"
    adapter.validate_targets=lambda:{"pair":NS(state=NS(id=1))}
    adapter.call=lambda *a:NS(success=False)
    adapter.hardware_state=Mock()
    with pytest.raises(RuntimeError):adapter.deactivate_after_fault()
    adapter.hardware_state.assert_not_called()
