"""Whole-robot application preview. No serial bridge or motor manager launch."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('start_estimator', default_value='false'),
        DeclareLaunchArgument('policy', default_value='resistance',
                              choices=['transparent', 'resistance', 'hold_velocity']),
        Node(package='mech_knee_kinematics', executable='knee_kinematics_node',
             condition=IfCondition(LaunchConfiguration('start_estimator')),
             output='screen'),
        Node(package='mech_assist_control', executable='exoskeleton_control_node',
             output='screen', parameters=[PathJoinSubstitution([
                 FindPackageShare('mech_assist_control'), 'config', 'exoskeleton_preview.yaml']),
                 {'policy': LaunchConfiguration('policy'), 'output_mode': 'preview',
                  'allow_motor_output': False}]),
    ])
