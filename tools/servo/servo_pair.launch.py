"""Load the calibrated servo pair; leave the trajectory controller inactive."""
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def read_description(path):
    return Path(path).read_text(encoding='utf-8')


def setup(context):
    description = read_description(LaunchConfiguration('urdf').perform(context))
    return [
        Node(package='controller_manager', executable='ros2_control_node',
             parameters=[str(Path(__file__).with_name('servo_controllers.yaml')),
                         {'robot_description': ParameterValue(description, value_type=str)}],
             output='screen'),
        Node(package='controller_manager', executable='spawner',
             arguments=['joint_state_broadcaster'], output='screen'),
        Node(package='controller_manager', executable='spawner',
             arguments=['servo_trajectory_controller', '--inactive'], output='screen'),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('urdf', description='Generated calibrated servo URDF'),
        OpaqueFunction(function=setup),
    ])
