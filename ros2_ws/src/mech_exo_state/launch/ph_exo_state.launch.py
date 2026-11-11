"""Launch the PH-EXO state receiver, accepting previously saved parameter files."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml


def launch_receiver(context):
    path = LaunchConfiguration('config').perform(context)
    params = [path]
    # A ROS parameter file scoped to the previous node name would otherwise
    # silently lose its settings. Carry that block into the renamed receiver.
    with open(path, encoding='utf-8') as stream:
        config = yaml.safe_load(stream)
    if isinstance(config, dict) and not any(k in config for k in ('ph_exo_state', '/ph_exo_state')):
        old = config.get('ref01_state', config.get('/ref01_state'))
        if isinstance(old, dict) and 'ros__parameters' in old:
            params.append(old['ros__parameters'])
    return [Node(package='mech_exo_state', executable='state_node.py', name='ph_exo_state',
                 parameters=params, output='screen')]


def generate_launch_description():
    default = Path(get_package_share_directory('mech_exo_state')) / 'config/ph_exo_state.yaml'
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=str(default)),
        OpaqueFunction(function=launch_receiver),
    ])
