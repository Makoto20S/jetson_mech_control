"""Existing upper node, opt-in fixed-pose adapter; no automatic start."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from pathlib import Path
def generate_launch_description():
    default=str(Path(get_package_share_directory("mech_assist_control"))/"config/ph_exo_upper.yaml")
    return LaunchDescription([DeclareLaunchArgument("config",default_value=default),
        Node(package="mech_assist_control",executable="exoskeleton_control_node",
             parameters=[LaunchConfiguration("config")],output="screen")])
