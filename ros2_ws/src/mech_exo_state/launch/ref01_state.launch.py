from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default = str(Path(get_package_share_directory('mech_exo_state')) / 'config/ref01_state.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=default),
        Node(package='mech_exo_state', executable='state_node.py', name='ref01_state',
             remappings=[("ph_exo/state", "ref01/state")],
             parameters=[LaunchConfiguration('config')], output='screen'),
    ])
