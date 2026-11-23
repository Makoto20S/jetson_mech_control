"""Run official ros2_control_node with the isolated REF01 MuJoCo hardware plugin.

Software only: default plant is in-process MuJoCo; an optional fixture supplies
private PTY MIT drivers. Caller holds sync.lock.
Windows can import build_configuration without ROS to validate fixture scope.
"""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import traceback
import uuid
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[2]
MODEL = "mujoco/ref01/config/mujoco/ref01_dynamics_cad_v1.xml"
CONTROLLER = "ref01_impedance"
HARDWARE = "ref01_mujoco"
TYPE = "mech_controllers/Ref01ImpedanceController"
PLUGIN = "mech_simulation/Ref01MujocoSystem"
JOINTS = ("motor104_joint", "motor105_joint")
STATES = ("position", "velocity", "sample_time_s", "sequence")
COMMANDS = ("effort", "command_generation")
EXECUTABLE = Path("/opt/ros/humble/lib/controller_manager/ros2_control_node")


def save(path, value):
    path.write_text(json.dumps(value, indent=2, allow_nan=False), encoding="utf-8")


def build_configuration(model, output, case, duration, namespace):
    if case not in ("psi", "beta", "loss", "duplicate", "reference"):
        raise ValueError("unknown scenario")
    if not math.isfinite(duration) or not 2.5 <= duration <= 120:
        raise ValueError("duration must be 2.5..120s")
    if abs(duration/.005-round(duration/.005)) > 1e-7:
        raise ValueError("duration must be a multiple of 5 ms")
    if not namespace.startswith("/ph_exo_manager_") or "/" in namespace[1:]:
        raise ValueError("isolated namespace required")
    if not Path(model).is_absolute() or not Path(output).is_absolute():
        raise ValueError("model/output must be absolute")
    robot = ET.Element("robot", name="ph_exo_ref01_simulation")
    ET.SubElement(robot, "link", name="base")
    for index, name in enumerate(JOINTS):
        ET.SubElement(robot, "link", name="motor_link_"+str(index))
        joint = ET.SubElement(robot, "joint", name=name, type="continuous")
        ET.SubElement(joint, "parent", link="base")
        ET.SubElement(joint, "child", link="motor_link_"+str(index))
        ET.SubElement(joint, "axis", xyz="0 0 1")
    control = ET.SubElement(robot, "ros2_control", name=HARDWARE, type="system")
    hardware = ET.SubElement(control, "hardware")
    ET.SubElement(hardware, "plugin").text = PLUGIN
    for name, value in dict(simulation_only="true", model_path=str(model), output_dir=str(output),
                            scenario=case, duration_s=str(duration),
                            max_samples=str(round(duration/.005)+1)).items():
        ET.SubElement(hardware, "param", name=name).text = value
    for name in JOINTS:
        joint = ET.SubElement(control, "joint", name=name)
        for interface in STATES:
            ET.SubElement(joint, "state_interface", name=interface)
        for interface in COMMANDS:
            ET.SubElement(joint, "command_interface", name=interface)
    urdf = ET.tostring(robot, encoding="unicode")
    params = {
        namespace+"/controller_manager": {"ros__parameters": {
            "robot_description": urdf, "update_rate": 200, "use_sim_time": False,
            "lock_memory": False, CONTROLLER: {"type": TYPE}}},
        namespace+"/"+CONTROLLER: {"ros__parameters": {
            "simulation_only": True, "model_path": str(model), "joints": list(JOINTS)}}}
    return urdf, params


def run_case(folder, case, duration, fixture=None, typed=False, dynamic_case=None, dynamic_gains=None):
    import rclpy
    from rclpy.node import Node
    from rosidl_runtime_py.convert import message_to_ordereddict
    from std_msgs.msg import Float64MultiArray
    from ph_exo_interfaces.msg import FixedPosePlan, ImpedanceStatus
    from std_srvs.srv import Trigger
    from rcl_interfaces.srv import GetParameters
    from controller_manager_msgs.srv import (LoadController, ConfigureController, SwitchController,
        ListControllers, ListHardwareComponents, ListHardwareInterfaces, SetHardwareComponentState)
    from lifecycle_msgs.msg import State
    from ament_index_python.packages import get_package_prefix
    from ref01_manager_validation import summarize_case

    dynamic=None
    if dynamic_case is not None:
        if typed or (fixture is None and case!="reference") or (fixture and fixture.predictive):
            raise ValueError("dynamic runner requires ordinary isolated software plant")
        from ref01_dynamic_validation import DynamicRun
        from ph_exo_interfaces.msg import ImpedanceRef
        if fixture:
            from ref01_dynamic_mit import MitReferenceRun
            dynamic=MitReferenceRun(dynamic_case,dynamic_gains)
        else:dynamic=DynamicRun(dynamic_case)
    folder.mkdir(parents=True, exist_ok=False)
    namespace = "/ph_exo_manager_"+uuid.uuid4().hex[:10]
    urdf, params = (fixture.start(folder,case,duration,namespace) if fixture else
        build_configuration((ROOT/MODEL).resolve(), folder.resolve(), case, duration, namespace))
    if typed: params[namespace+"/"+CONTROLLER]["ros__parameters"]["command_source"]="fixed_pose_v1"
    if dynamic:params[namespace+"/"+CONTROLLER]["ros__parameters"]["command_source"]="dynamic_ref_v1"
    (folder/"robot.urdf").write_text(urdf, encoding="utf-8")
    save(folder/"params.json", params)  # JSON is valid YAML; avoid an extra YAML dependency.
    plugins = ("mech_controllers", "mech_bringup" if fixture else "mech_simulation")
    prefixes = {name: get_package_prefix(name) for name in ("controller_manager", *plugins)}
    expected = ROOT/".local/ros/ref01_native/install"
    for name in plugins:
        if Path(prefixes[name]).resolve() != expected/name:
            raise RuntimeError("wrong overlay: "+name+": "+prefixes[name])
    if Path(prefixes["controller_manager"]) != Path("/opt/ros/humble"):
        raise RuntimeError("expected official Humble controller_manager")
    env = dict(os.environ, ROS_LOG_DIR=str(folder/"ros_logs"), ROS_LOCALHOST_ONLY="1",
               PYTHONDONTWRITEBYTECODE="1")
    command = [str(EXECUTABLE),"--ros-args","-r","__ns:="+namespace,
               "--params-file",str(folder/"params.json")]
    save(folder/"command.json", dict(command=command, prefixes=prefixes,
                                   domain=os.environ.get("ROS_DOMAIN_ID")))
    os.environ["ROS_LOG_DIR"] = str(folder/"supervisor_ros_logs")
    log = (folder/"manager.log").open("w", encoding="utf-8")
    try:
        rclpy.init()
        try:
            node = Node("test_supervisor", namespace=namespace)
        except Exception:
            rclpy.shutdown()
            raise
        try:
            process = subprocess.Popen(command, env=env, stdout=log, stderr=subprocess.STDOUT)
        except Exception:
            node.destroy_node()
            rclpy.shutdown()
            raise
    except Exception:
        log.close()
        raise
    statuses, evidence, lifecycle = [], {}, {}
    typed_statuses=[]
    active = False
    error = None
    clients = {}
    def client(kind, name):
        key = (kind, name)
        if key not in clients: clients[key] = node.create_client(kind, namespace+"/"+name)
        return clients[key]
    def request(kind, name, req, timeout=12):
        handle = client(kind,name)
        if not handle.wait_for_service(timeout_sec=timeout):
            raise RuntimeError("service unavailable: "+name)
        future = handle.call_async(req)
        end = time.monotonic()+timeout
        while rclpy.ok() and not future.done() and time.monotonic()<end:
            rclpy.spin_once(node, timeout_sec=.01)
            if process.poll() is not None: raise RuntimeError("manager exited during "+name)
        if not future.done(): raise RuntimeError("service timed out: "+name)
        return future.result()
    def switch(enable):
        req = SwitchController.Request()
        req.activate_controllers = [CONTROLLER] if enable else []
        req.deactivate_controllers = [] if enable else [CONTROLLER]
        req.strictness = SwitchController.Request.STRICT
        req.activate_asap = True
        req.timeout.sec = 5
        response = request(SwitchController,"controller_manager/switch_controller",req)
        if not response.ok: raise RuntimeError("switch failed: "+str(enable))
    def controller_list():
        return request(ListControllers,"controller_manager/list_controllers",ListControllers.Request())
    try:
        req = LoadController.Request(); req.name = CONTROLLER
        if not request(LoadController,"controller_manager/load_controller",req).ok:
            raise RuntimeError("load failed")
        req = ConfigureController.Request(); req.name = CONTROLLER
        if not request(ConfigureController,"controller_manager/configure_controller",req).ok:
            raise RuntimeError("configure failed")
        publisher = node.create_publisher(ImpedanceRef if dynamic else FixedPosePlan if typed else Float64MultiArray, namespace+"/"+CONTROLLER+("/ref" if dynamic else "/fixed_pose_plan" if typed else "/plan"),10)
        typed_statuses=[]
        status_subscription=node.create_subscription(ImpedanceStatus, namespace+"/"+CONTROLLER+"/status",lambda msg:typed_statuses.append(message_to_ordereddict(msg)),10)
        # Discovery before activation; no plan issued while inactive.
        discovery_end = time.monotonic()+5
        while publisher.get_subscription_count() != 1 and time.monotonic()<discovery_end:
            rclpy.spin_once(node,timeout_sec=.01)
        if publisher.get_subscription_count() != 1: raise RuntimeError("plan subscriber missing/ambiguous")
        switch(True); active = True
        controllers = controller_list()
        hardware = request(ListHardwareComponents,"controller_manager/list_hardware_components",ListHardwareComponents.Request())
        interfaces = request(ListHardwareInterfaces,"controller_manager/list_hardware_interfaces",ListHardwareInterfaces.Request())
        req = GetParameters.Request(); req.names = ["update_rate"]
        rate = request(GetParameters,"controller_manager/get_parameters",req).values[0].integer_value
        controller = [item for item in controllers.controller if item.name == CONTROLLER]
        component = [item for item in hardware.component if item.name == HARDWARE]
        command_names = {j+"/"+i for j in JOINTS for i in COMMANDS}
        state_names = {j+"/"+i for j in JOINTS for i in STATES}
        checks = dict(
            official_binary=Path("/proc/"+str(process.pid)+"/exe").resolve()==EXECUTABLE,
            controller_active=len(controller)==1 and controller[0].state=="active" and controller[0].type==TYPE,
            command_interfaces_claimed=len(controller)==1 and set(controller[0].claimed_interfaces)==command_names,
            hardware_active=len(component)==1 and component[0].state.label=="active" and component[0].class_type==PLUGIN,
            command_interfaces_available=command_names=={item.name for item in interfaces.command_interfaces if item.is_available and item.is_claimed},
            state_interfaces_available=state_names=={item.name for item in interfaces.state_interfaces if item.is_available},
            update_rate_200hz=rate==200,
            plan_subscription_unique=publisher.get_subscription_count()==1)
        if fixture: checks.update(fixture.hardware_checks(hardware,interfaces))
        evidence = dict(checks=checks, pid=process.pid, prefixes=prefixes,
                        controllers=message_to_ordereddict(controllers),
                        hardware=message_to_ordereddict(hardware),
                        interfaces=message_to_ordereddict(interfaces), update_rate=rate)
        threads = []
        for task in Path("/proc/"+str(process.pid)+"/task").iterdir():
            try:
                tid = int(task.name)
                threads.append(dict(tid=tid, policy=os.sched_getscheduler(tid),
                                    realtime_priority=os.sched_getparam(tid).sched_priority))
            except ProcessLookupError:
                # DDS/executor helper threads can exit during a /proc snapshot.
                threads.append(dict(tid=int(task.name), exited_during_snapshot=True))
        evidence["threads"] = threads
        save(folder/"manager_interfaces.json", evidence)
        if not all(checks.values()): raise RuntimeError("manager interface evidence failed")
        status_client = client(Trigger,CONTROLLER+"/software_status")
        if not status_client.wait_for_service(timeout_sec=5): raise RuntimeError("status service unavailable")
        future = None
        sequence = 0
        next_plan = 0.
        next_status = 0.
        plan = None
        stop_requested = False
        deadline = time.monotonic()+duration+25
        while time.monotonic()<deadline:
            now = time.monotonic()
            if process.poll() is not None: raise RuntimeError("manager exited before completion")
            policy = fixture.policy() if fixture else "publish"
            if policy == "stop" and not stop_requested:
                lifecycle["stop_requested_ns"] = time.monotonic_ns()
                switch(False)
                stop_requested = True
            if now >= next_plan:
                if policy == "publish":
                    sequence += 1
                    values = fixture.plan_data(sequence,now) if fixture and hasattr(fixture,"plan_data") else [1.,float(sequence),now,1.,0.,math.pi/9.,2.4,2.,.18,.15,.2,.2]
                    if dynamic:
                        plan=dynamic.message(now,sequence,typed_statuses)
                    elif typed:
                        plan=FixedPosePlan(schema_version=1,session_id=1,sequence=sequence,
                            issued_at_monotonic_s=now,enabled=True)
                        plan.reference_rad=values[4:6]
                        plan.stiffness_nm_per_rad=values[6:8]
                        plan.damping_nm_s_per_rad=values[8:10]
                        plan.motor_torque_cap_nm=[values[11],values[10]]
                    else:
                        plan=Float64MultiArray(data=values)
                    if plan is not None:publisher.publish(plan)
                elif policy == "replay" and plan is not None:
                    publisher.publish(plan)  # preserve BOTH sequence and source time
                next_plan = now+.02
            if future is not None and future.done():
                response = future.result()
                if not response.success: raise RuntimeError("invalid status response")
                status = json.loads(response.message)
                status["received_ns"] = time.monotonic_ns()
                statuses.append(status)
                future = None
            if future is None and now >= next_status:
                future = status_client.call_async(Trigger.Request())
                next_status = now+.02
            rclpy.spin_once(node,timeout_sec=.001)
            if (folder/"complete.json").is_file(): break
            if (folder/"writer_error.txt").is_file(): raise RuntimeError("hardware writer failed")
        else: raise RuntimeError("hardware completion timeout")
        lifecycle["completed_ns"] = time.monotonic_ns()
    except Exception as exc:
        error = repr(exc)
        (folder/"supervisor_error.txt").write_text(traceback.format_exc(), encoding="utf-8")
    finally:
        if active and process.poll() is None:
            try:
                current = [item for item in controller_list().controller if item.name==CONTROLLER]
                if any(item.state=="active" for item in current):
                    try: switch(False)
                    except Exception as exc:
                        if fixture is None: raise
                        lifecycle["switch_error_after_fault"] = repr(exc)
                current = [item for item in controller_list().controller if item.name==CONTROLLER]
                lifecycle["controller_deactivated"] = len(current)==1 and current[0].state=="inactive"
                components = request(ListHardwareComponents,"controller_manager/list_hardware_components",ListHardwareComponents.Request()).component
                hardware_results = {}
                for component in components:
                    if component.state.label=="active":
                        req = SetHardwareComponentState.Request()
                        req.name = component.name
                        req.target_state = State(id=State.PRIMARY_STATE_INACTIVE, label="inactive")
                        result = request(SetHardwareComponentState,"controller_manager/set_hardware_component_state",req)
                        hardware_results[component.name] = result.ok and result.state.label=="inactive"
                    else:
                        hardware_results[component.name] = component.state.label in ("inactive","unconfigured","finalized")
                lifecycle["hardware_states"] = hardware_results
                lifecycle["hardware_deactivated"] = bool(hardware_results) and all(hardware_results.values())
            except Exception as exc:
                lifecycle["deactivation_error"] = repr(exc)
        if process.poll() is None: process.send_signal(signal.SIGINT)
        try: process.wait(timeout=10)
        except subprocess.TimeoutExpired:
            lifecycle["forced_termination"] = True
            process.terminate()
            try: process.wait(timeout=5)
            except subprocess.TimeoutExpired: process.kill(); process.wait(timeout=5)
        node.destroy_node()
        rclpy.shutdown()
        log.close()
        if fixture: fixture.close()
        lifecycle.update(returncode=process.returncode, error=error)
        save(folder/"controller_status.json",statuses)
        save(folder/"typed_status.json",typed_statuses)
        save(folder/"lifecycle.json",lifecycle)
    evidence.setdefault("checks",{}).update(
        explicit_controller_deactivation=lifecycle.get("controller_deactivated") is True,
        explicit_hardware_deactivation=lifecycle.get("hardware_deactivated") is True,
        no_forced_termination=not lifecycle.get("forced_termination",False),
        no_supervisor_error=error is None)
    if fixture:
        manager_text=(folder/"manager.log").read_text(encoding="utf-8")
        evidence["fault_shutdown_verified"]=(
            process.returncode==0 and not lifecycle.get("forced_termination",False) and
            "Deactivating controller '"+CONTROLLER+"'" in manager_text and
            all("Successful 'shutdown' of hardware 'mit_comm_"+str(mid)+"'" in manager_text for mid in (104,105)) and
            "Shutting down the controller manager" in manager_text)
    save(folder/"manager_interfaces.json",evidence)
    if error: raise RuntimeError(case+": "+error)
    if typed and not any(s["mode"]==2 and s["session_id"]==1 and s["state_valid"] for s in typed_statuses):
        raise RuntimeError("typed following status missing")
    if dynamic:
        save(folder/"reference_events.json",dynamic.events)
        result=(dynamic.summarize_mit(folder,process.returncode,evidence,statuses,typed_statuses,fixture)
                if fixture else dynamic.summarize(folder,process.returncode,evidence,statuses,typed_statuses))
        save(folder/"summary.json",result)
        return result
    result = (fixture.summarize(process.returncode,evidence,statuses) if fixture else
              summarize_case(folder,case,duration,process.returncode,evidence,statuses))
    save(folder/"summary.json",result)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--typed",action="store_true")
    parser.add_argument("--output",required=True,type=Path)
    parser.add_argument("--duration",type=float,default=30.)
    parser.add_argument("--cases",nargs="+",choices=["psi","beta","loss","duplicate"],default=["psi","beta","loss","duplicate"])
    args = parser.parse_args()
    if sys.platform!="linux" or os.environ.get("EXO_SYNC_LOCK_HELD")!="1":
        raise RuntimeError("run on Orin under the acquired project sync.lock")
    if (ROOT/".local/sync/incomplete.json").exists(): raise RuntimeError("interrupted sync")
    if os.environ.get("ROS_LOCALHOST_ONLY")!="1" or os.environ.get("ROS_DOMAIN_ID")!="179":
        raise RuntimeError("use isolated ROS_DOMAIN_ID=179 ROS_LOCALHOST_ONLY=1")
    output=args.output.resolve()
    if not output.is_relative_to(ROOT/".local/validation"): raise ValueError("use project validation output")
    for ancestor in [args.output,*args.output.parents]:
        if ancestor.is_symlink(): raise ValueError("output path traverses symlink")
    output.mkdir(parents=True,exist_ok=False)
    sources = [MODEL,"tools/project/test_ref01_manager.py","tools/project/ref01_manager_validation.py",
               "ros2_ws/src/mech_simulation/src/ref01_mujoco_system.cpp",
               "ros2_ws/src/mech_controllers/src/ref01_impedance_controller.cpp"]
    save(output/"source_hashes.json",{name:hashlib.sha256((ROOT/name).read_bytes()).hexdigest() for name in sources})
    results=[]
    for case in args.cases:
        result=run_case(output/case,case,args.duration if case in ("psi","beta") else 2.5,typed=args.typed)
        results.append(result)
        print(json.dumps(dict(case=case, functional_passed=result["functional_passed"],
            timing_passed=result["timing_passed"], metrics=result.get("metrics"),
            failed_checks=[k for k,v in result["functional"]["checks"].items() if not v],
            errors=result.get("errors")),allow_nan=False),flush=True)
    from timing_contract import evaluate_suite
    suite=evaluate_suite(results)
    save(output/"summary.json",dict(cases=results,suite=suite))
    return 0 if suite.get("passed") else 1


if __name__=="__main__":
    raise SystemExit(main())
