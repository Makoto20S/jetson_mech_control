from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_file = PathJoinSubstitution([
        FindPackageShare('mech_ctrboard_bridge'),
        'config',
        'ctrboard_usb_cdc.yaml',
    ])

    device_path = LaunchConfiguration('device_path')
    imu_1_frame_id = LaunchConfiguration('imu_1_frame_id')
    imu_2_frame_id = LaunchConfiguration('imu_2_frame_id')

    return LaunchDescription([
        DeclareLaunchArgument(
            'device_path',
            default_value='/dev/serial/by-id/usb-Livelybot_Livelybot_Communication_Board_2024051701-if08',
            description='Livelybot USB-CDC channel carrying STM32 telemetry',
        ),
        DeclareLaunchArgument(
            'imu_1_frame_id',
            default_value='imu_1_link',
            description='ROS frame for the first IMU',
        ),
        DeclareLaunchArgument(
            'imu_2_frame_id',
            default_value='imu_2_link',
            description='ROS frame for the second IMU',
        ),
        Node(
            package='mech_ctrboard_bridge',
            executable='ctrboard_bridge_node',
            name='stm32_ctrboard',
            output='both',
            parameters=[
                config_file,
                {
                    'device_path': device_path,
                    'imu_1_frame_id': imu_1_frame_id,
                    'imu_2_frame_id': imu_2_frame_id,
                },
            ],
        ),
    ])
