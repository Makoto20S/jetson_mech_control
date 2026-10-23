"""Receive-only attitude pipeline; deliberately does not start a serial bridge."""
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    return LaunchDescription([Node(
        package='mech_knee_kinematics', executable='knee_kinematics_node',
        name='knee_kinematics', output='screen',
        parameters=[PathJoinSubstitution([
            FindPackageShare('mech_knee_kinematics'), 'config', 'dual_imu_attitude.yaml'])],
    )])
