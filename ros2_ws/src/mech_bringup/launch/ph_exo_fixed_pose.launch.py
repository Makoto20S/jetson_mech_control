"""Software-only MuJoCo startup; controller loaded inactive, no plan publisher."""
import importlib.util
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def configure(context):
    share=Path(get_package_share_directory("mech_bringup"))
    spec=importlib.util.spec_from_file_location("ph_exo_configuration",share/"config/ph_exo_configuration.py")
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    output=Path(LaunchConfiguration("output").perform(context))
    if not output.is_absolute() or output.exists():
        raise ValueError("output必须是新的绝对路径；本入口仅MuJoCo软件运行")
    output.mkdir(parents=True)
    ns="/ph_exo_manager_launch"
    model=share/"ph_exo/ref01/config/mujoco/ref01_dynamics_cad_v1.xml"
    source=LaunchConfiguration("command_source").perform(context)
    if source not in ("fixed_pose_v1","dynamic_ref_v1"):raise ValueError("unsupported command source")
    _,params=module.build_configuration(model,output,"reference" if source=="dynamic_ref_v1" else "psi",30.,ns)
    params[ns+"/ref01_impedance"]["ros__parameters"]["command_source"]=source
    profile=LaunchConfiguration("profile").perform(context)
    with_state=LaunchConfiguration("state").perform(context)=="true"
    if profile=="prototype":
        params[ns+"/ref01_impedance"]["ros__parameters"].update(simulation_only=False,bench_profile="ref01_supported_v1")
    if with_state:
        params[ns+"/controller_manager"]["ros__parameters"]["joint_state_broadcaster"]={"type":"joint_state_broadcaster/JointStateBroadcaster"}
        params[ns+"/joint_state_broadcaster"]={"ros__parameters":{"use_local_topics":True}}
    actions=[
        Node(package="controller_manager",executable="ros2_control_node",namespace=ns,
             parameters=[params[ns+"/controller_manager"]["ros__parameters"]],
             output="screen"),
        Node(package="controller_manager",executable="spawner",namespace=ns,
             arguments=["ref01_impedance","--inactive","-c",ns+"/controller_manager",
                        "--param-file",str(write_parameters(output,params))],
             output="screen")]
    if with_state:
        actions.extend([
            Node(package="controller_manager",executable="spawner",namespace=ns,
                 arguments=["joint_state_broadcaster","-c",ns+"/controller_manager",
                     "--param-file",str(output/"controller_parameters.json")],output="screen"),
            Node(package="mech_exo_state",executable="state_node.py",namespace=ns,
                 parameters=[{"joint_topic":ns+"/joint_state_broadcaster/joint_states",
                     "mapping_confirmed":True,"publish_rate_hz":100.0}],output="screen")])
    return actions

def write_parameters(output,params):
    import json
    path=output/"controller_parameters.json"
    path.write_text(json.dumps(params),encoding="utf-8")
    return path

def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument("output",
        description="新建软件运行结果目录的绝对路径"),
        DeclareLaunchArgument("command_source",default_value="fixed_pose_v1",
            choices=["fixed_pose_v1","dynamic_ref_v1"]),
        DeclareLaunchArgument("profile",default_value="low",choices=["low","prototype"]),
        DeclareLaunchArgument("state",default_value="false",choices=["false","true"]),
        OpaqueFunction(function=configure)])
