"""Encoder-only resistance preview; launches neither IMU nor hardware manager."""
from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        Node(package='mech_assist_control', executable='exoskeleton_control_node',
             output='screen', parameters=[PathJoinSubstitution([
                 FindPackageShare('mech_assist_control'), 'config',
                 'motor_resistance_preview.yaml']),
                 {'output_mode': 'preview', 'allow_motor_output': False}]),
    ])
