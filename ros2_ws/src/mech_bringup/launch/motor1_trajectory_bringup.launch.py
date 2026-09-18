"""Standard joint_trajectory_controller deployment; controller is loaded inactive.

Starting this launch opens the configured physical transport. Use only under
the approved bench procedure; offline tests construct it without starting nodes.
"""

from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    share = FindPackageShare('mech_bringup')
    robot_description = ParameterValue(Command([
        FindExecutable(name='xacro'), ' ',
        PathJoinSubstitution([share, 'config', 'motor1.urdf.xacro']),
    ]), value_type=str)
    broadcaster = Node(
        package='controller_manager', executable='spawner',
        arguments=['joint_state_broadcaster'],
    )
    trajectory = Node(
        package='controller_manager', executable='spawner',
        arguments=['motor1_trajectory_controller', '--inactive'],
    )
    # Serialize spawners: both call the same manager's load/configure services.
    # Never activate trajectory control as a side effect of launching.
    after_broadcaster = RegisterEventHandler(OnProcessExit(
        target_action=broadcaster, on_exit=[trajectory],
    ))
    return LaunchDescription([
        Node(
            package='controller_manager', executable='ros2_control_node',
            parameters=[{'robot_description': robot_description},
                        PathJoinSubstitution([
                            share, 'config',
                            'motor1_trajectory_controllers.yaml',
                        ])],
            output='both',
        ),
        after_broadcaster,
        broadcaster,
    ])
