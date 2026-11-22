"""No-device integration: official manager + MuJoCo + real state node + existing upper node."""
import json,os,sys,time,subprocess,signal,uuid,csv,math,argparse
from pathlib import Path
ROOT=Path.cwd()
sys.path.insert(0,str(ROOT/"tools/project"))
from test_ref01_manager import build_configuration
from ref01_manager_validation import load_rows, _recovery
import rclpy
from rclpy.parameter import Parameter
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_srvs.srv import Trigger
from mech_exo_state.msg import RobotState
from rclpy.qos import qos_profile_sensor_data
from rosidl_runtime_py.convert import message_to_ordereddict
from mech_assist_control.robot_node import ExoskeletonNode

def run_case(base,case,driver=None,reference_config=None,fixture_factory=None):
    folder=base/case;folder.mkdir()
    ns="/ph_exo_manager_upper_"+uuid.uuid4().hex[:7]
    model=ROOT/"mujoco/ref01/config/mujoco/ref01_dynamics_cad_v1.xml"
    fixture=None
    is_mit=case.startswith("mit_")
    if is_mit:
        from ref01_mit_fixture import MitFixture
        fixture=(fixture_factory() if fixture_factory else MitFixture(feedback_period_s=.005,acknowledge_disable=True,
                           disable_ack_ids=(104,) if case=="mit_no_ack" else (104,105)))
        _,params=fixture.start(folder,"psi",20.,ns)
        params[ns+"/controller_manager"]["ros__parameters"]["hardware_components_initial_state"]={
            "inactive":["mit_comm_104","mit_comm_105"]}
        cfg=json.loads((ROOT/"config/parallel_exo/servo_pair.ref01.json").read_text())
        for bus in cfg["buses"]:
            mid=next(m["id"] for m in cfg["motors"] if m["logical_bus"]==bus["logical_bus"])
            bus["device_path"]=fixture.ports[mid-104]["path"]
        sys.path.insert(0,str(ROOT/"tools/servo"))
        from servo_operator import generate_urdf
        (folder/"disable.urdf").write_text(generate_urdf(cfg,observe=True))
    else:
        _,params=build_configuration(model,folder,"reference" if reference_config else "beta" if case=="beta" else "psi",20. if reference_config else 3.,ns)
    params[ns+"/ref01_impedance"]["ros__parameters"]["command_source"]="fixed_pose_v1"
    if reference_config:
        params[ns+"/ref01_impedance"]["ros__parameters"].update(command_source="dynamic_ref_v1",
            simulation_only=False,bench_profile="ref01_supported_v1")
    params[ns+"/controller_manager"]["ros__parameters"]["joint_state_broadcaster"]={"type":"joint_state_broadcaster/JointStateBroadcaster"}
    params[ns+"/joint_state_broadcaster"]={"ros__parameters":{"use_local_topics":True}}
    config=folder/"manager.json";config.write_text(json.dumps(params))
    log=(folder/"process.log").open("w")
    children=[];executor=None;upper=None;node=None
    def launch(cmd):
        p=subprocess.Popen(cmd,stdout=log,stderr=log,start_new_session=True);children.append(p);return p
    manager=launch(["/opt/ros/humble/lib/controller_manager/ros2_control_node","--ros-args","-r","__ns:="+ns,"--params-file",str(config)])
    try:
        for name in ("joint_state_broadcaster","ref01_impedance"):
            cmd=["ros2","run","controller_manager","spawner",name,"-c",ns+"/controller_manager","--param-file",str(config),"--controller-manager-timeout","12"]
            if name=="ref01_impedance":cmd.append("--inactive")
            subprocess.run(cmd,stdout=log,stderr=log,check=True,timeout=20)
        state_command=["ros2","run","mech_exo_state","state_node.py","--ros-args","-r","__ns:="+ns,
            "-p","joint_topic:="+ns+"/joint_state_broadcaster/joint_states","-p","mapping_confirmed:=true","-p","publish_rate_hz:=100.0"]
        overrides=[]
        if is_mit:
            state_command+=["-p","motor_source:=mit",
                "-p","mit_topic:="+ns+"/joint_state_broadcaster/dynamic_joint_states",
                "-p","mit_joint_names:=[motor104_joint, motor105_joint]",
                "-p","mit_pole_pairs:=[14, 14]" if fixture_factory else "mit_pole_pairs:=[21, 21]","-p","mit_gear_ratios:=[8.0, 8.0]",
                "-p","mit_speed_confirmed:=[true, true]"]
            overrides=[Parameter("hold.backend",value="mit_pty"),
                Parameter("hold.hw",value=["mit_comm_104","mit_comm_105"]),
                Parameter("hold.disable",value=str(folder/"disable.urdf")),
                Parameter("hold.log",value=str(folder))]
        if reference_config:
            overrides += [Parameter("hold."+k,value=v) for k,v in dict(reference_config,
                source="dynamic_ref_v1",profile="ref01_supported_v1").items()]
        state=launch(state_command)
        upper=ExoskeletonNode(namespace=ns,parameter_overrides=overrides+[
            Parameter("policy",value="ph_exo_fixed_pose"),
            Parameter("hold.cm",value=ns+"/controller_manager"),
            Parameter("hold.ctrl",value=ns+"/ref01_impedance"),
            Parameter("hold.state",value=ns+"/ph_exo/state")])
        node=Node("upper_test_client",namespace=ns)
        # Only the final RobotState is used for diagnostics. Deep conversion
        # of every 100Hz state competes with the synthetic Python device peer.
        observed=[]
        node.create_subscription(RobotState,ns+"/ph_exo/state",
            lambda m:observed.__setitem__(slice(None),[m]),qos_profile_sensor_data)
        executor=MultiThreadedExecutor(num_threads=2);executor.add_node(upper);executor.add_node(node)
        def spin_for(condition,timeout):
            end=time.monotonic()+timeout
            while not condition() and time.monotonic()<end:
                executor.spin_once(timeout_sec=.01)
            if not condition():
                (folder/"diagnostic.json").write_text(json.dumps(dict(
                    upper=upper.ph_exo.snapshot(),state=upper.ph_exo.state,
                    robot_state=message_to_ordereddict(observed[-1]) if observed else None,
                    topics=node.get_topic_names_and_types()),indent=2))
            assert condition(),upper.ph_exo.snapshot()
        def call(name):
            client=node.create_client(Trigger,ns+"/exoskeleton/impedance/"+name)
            assert client.wait_for_service(timeout_sec=3)
            f=client.call_async(Trigger.Request());spin_for(f.done,5)
            result=f.result();node.destroy_client(client)
            return result
        if not is_mit:
            spin_for(lambda:upper.ph_exo.state is not None and upper.ph_exo.state[2],10)
        if driver is not None:
            return driver(dict(ns=ns,folder=folder,upper=upper,node=node,executor=executor,
                               manager=manager,state=state,fixture=fixture))
        assert call("start").success
        spin_for(lambda:upper.ph_exo.session.phase in ("running","unconfirmed","stopped"),8)
        assert upper.ph_exo.session.phase=="running",upper.ph_exo.snapshot()
        running_at=time.monotonic()
        if case in ("state_loss","mit_state_loss"):
            os.killpg(state.pid,signal.SIGTERM);state.wait(timeout=5)
            spin_for(lambda:upper.ph_exo.session.phase in ("stopped","unconfirmed"),5)
        else:
            if case in ("psi","beta"):
                spin_for(lambda:(folder/"complete.json").exists() or upper.ph_exo.session.phase!="running",5)
            else:
                spin_for(lambda:time.monotonic()-running_at>.3 or upper.ph_exo.session.phase!="running",2)
            assert upper.ph_exo.session.phase=="running",upper.ph_exo.snapshot()
            assert call("emergency_stop").success
            spin_for(lambda:upper.ph_exo.session.phase in ("stopped","unconfirmed"),5)
        snapshot=upper.ph_exo.snapshot()
        assert manager.poll() is None,"shared manager was killed"
        spin_for(lambda:not upper.ph_exo.worker.is_alive(),2)
        if case=="mit_no_ack":
            assert snapshot["state"]=="unconfirmed" and not snapshot["stop_ok"],snapshot
            assert not call("reset").success
        else:
            assert snapshot["state"]=="stopped",snapshot
            assert snapshot["stop_ok"]
            assert snapshot["disable_ok"]==([True,True] if is_mit else [False,False])
            assert call("reset").success
            assert upper.ph_exo.session.phase=="debug" and not upper.ph_exo.renew
            sequence=upper.ph_exo.session.sequence
            tx_count=len(fixture.tx) if fixture else 0
            until=time.monotonic()+.15
            spin_for(lambda:time.monotonic()>until,1)
            assert upper.ph_exo.session.sequence==sequence
            if fixture:assert len(fixture.tx)==tx_count,"reset automatically resumed MIT commands"
        assert call("status").success
        if case=="mit_reset":
            old_session=upper.ph_exo.session.session_id
            assert call("start").success
            spin_for(lambda:upper.ph_exo.session.phase in ("running","stopped","unconfirmed"),8)
            assert upper.ph_exo.session.phase=="running",upper.ph_exo.snapshot()
            assert upper.ph_exo.session.session_id>old_session
            assert call("stop").success
            spin_for(lambda:upper.ph_exo.session.phase in ("stopped","unconfirmed"),5)
            assert upper.ph_exo.session.phase=="stopped",upper.ph_exo.snapshot()
        rows=load_rows(folder/"trace.csv") if case in ("psi","beta") else []
        recovery={}
        if case in ("psi","beta"):
            metadata=json.loads((folder/"metadata.json").read_text())
            recovery=_recovery(rows,metadata,1 if case=="beta" else 0)
            assert recovery["passed"],recovery
        if fixture:
            assert not fixture.errors,fixture.errors
            assert {r["drive"] for r in fixture.disable_requests}=={104,105}
        result=dict(case=case,manager_survived=True,reset_debug=case!="mit_no_ack",stop=snapshot,
                    restart_passed=case=="mit_reset",missing_ack_blocks_reset=case=="mit_no_ack",
                    recovery=recovery,rows=len(rows),software_only=True)
        (folder/"upper_result.json").write_text(json.dumps(result,indent=2))
        return result
    finally:
        if upper:
            (folder/"end_state.json").write_text(json.dumps(dict(
                upper=upper.ph_exo.snapshot(),
                robot=message_to_ordereddict(observed[-1]) if observed else None,
                ctrl=message_to_ordereddict(upper.ph_exo.controller_state) if upper.ph_exo.controller_state else None),indent=2))
            upper.stop()
            end=time.monotonic()+5
            while executor and upper.ph_exo.worker and upper.ph_exo.worker.is_alive() and time.monotonic()<end:
                executor.spin_once(timeout_sec=.02)
        if executor:
            executor.shutdown()
            # Humble shutdown does not join Python pool workers.
            executor._executor.shutdown(wait=True)
        if upper:upper.destroy_node()
        if node:node.destroy_node()
        for child in reversed(children):
            if child.poll() is None:
                os.killpg(child.pid,signal.SIGINT)
                try:child.wait(timeout=5)
                except subprocess.TimeoutExpired:os.killpg(child.pid,signal.SIGTERM);child.wait(timeout=3)
        if fixture:fixture.close()
        log.close()

if __name__=="__main__":
    parser=argparse.ArgumentParser()
    parser.add_argument("--cases",nargs="+",choices=["psi","beta","state_loss","estop_active","mit_reset","mit_state_loss","mit_no_ack"],
                        default=["psi","beta","state_loss","estop_active"])
    args=parser.parse_args()
    base=ROOT/".local/validation"/("ph_exo_upper_"+time.strftime("%Y%m%d_%H%M%S"))
    base.mkdir()
    os.environ.update(ROS_DOMAIN_ID="183",ROS_LOCALHOST_ONLY="1",ROS_LOG_DIR=str(base/"ros_logs"))
    rclpy.init()
    try:
        results=[run_case(base,c) for c in args.cases]
        (base/"summary.json").write_text(json.dumps(results,indent=2))
        print(json.dumps({"output":str(base),"results":results},indent=2))
    finally:rclpy.shutdown()
