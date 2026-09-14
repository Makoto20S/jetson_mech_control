# Single-motor AK3.0 force-control bring-up entry.
#
# WARNING: launching against real hardware sends position commands to the
# motor once the position controller is activated. Real-device activation
# is gated by ADR-006 (Proposed) and G0-G3 evidence, and every real-motor
# run needs the owner's explicit per-test authorization. For offline
# exercise, keep the controller deactivated (comment out its spawner).

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        ' ',
        PathJoinSubstitution([
            FindPackageShare('mech_bringup'), 'config', 'motor1.urdf.xacro'
        ]),
    ])

    controllers_file = PathJoinSubstitution([
        FindPackageShare('mech_bringup'), 'config', 'motor1_controllers.yaml'
    ])

    # Declared for symmetry with future multi-motor deployments; the example
    # URDF reads the serial device path from its own <param> block.
    DeclareLaunchArgument(
        'device_path',
        default_value='/dev/ttyACM0',
        description='Serial device of the USB-CDC CAN channel (bench: ttyACM0)',
    )

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description_content}],
        ),
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[robot_description_content, controllers_file],
            output='both',
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster'],
        ),
        # Keep the position controller spawner commented out for offline
        # bring-up: uncommenting it arms position commands on the motor.
        # Node(
        #     package='controller_manager',
        #     executable='spawner',
        #     arguments=['motor1_position_controller'],
        # ),
    ])
