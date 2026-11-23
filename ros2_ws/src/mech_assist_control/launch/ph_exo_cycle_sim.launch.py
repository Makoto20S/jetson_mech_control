"""No-device continuous-reference entry: MuJoCo + state + upper, awaiting start."""
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    bringup=Path(get_package_share_directory("mech_bringup"))
    upper=Path(get_package_share_directory("mech_assist_control"))
    return LaunchDescription([
        DeclareLaunchArgument("output",description="New absolute directory for this software fixture"),
        DeclareLaunchArgument("config",default_value=str(upper/"config/ph_exo_cycle.yaml")),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(bringup/"launch/ph_exo_fixed_pose.launch.py")),
            launch_arguments={"output":LaunchConfiguration("output"),"command_source":"dynamic_ref_v1",
                              "profile":"prototype","state":"true"}.items()),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(upper/"launch/ph_exo_upper.launch.py")),
            launch_arguments={"config":LaunchConfiguration("config")}.items())])
