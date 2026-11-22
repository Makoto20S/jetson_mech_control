from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('input_topic',default_value='/imu/imu1/data'),
        DeclareLaunchArgument('output_topic',default_value='/ph_exo/gait_phase'),
        DeclareLaunchArgument('axis',default_value='x'),
        DeclareLaunchArgument('input_scale_rad_s',default_value='1.0'),
        DeclareLaunchArgument('input_offset_rad_s',default_value='0.0'),
        Node(package='mech_exo_state',executable='ao_node.py',name='ph_exo_ao',parameters=[{
            'input_topic':LaunchConfiguration('input_topic'),
            'output_topic':LaunchConfiguration('output_topic'),
            'axis':LaunchConfiguration('axis'),
            'input_scale_rad_s':ParameterValue(LaunchConfiguration('input_scale_rad_s'),value_type=float),
            'input_offset_rad_s':ParameterValue(LaunchConfiguration('input_offset_rad_s'),value_type=float)}])])
