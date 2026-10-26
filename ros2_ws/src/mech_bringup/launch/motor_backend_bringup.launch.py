"""Select exactly one motor backend for a physical USB-CAN channel.

Loading a hardware plugin may open the configured port. The Damiao hardware
and selected controller remain inactive until explicitly activated.
"""

import re

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, OpaqueFunction,
                            RegisterEventHandler)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


BACKENDS = {
    'dm4340_dual': ('dm4340_dual.urdf.xacro',
                    'dm4340_dual_controllers.yaml', 'dm4340_motor1_velocity_controller'),
    'ak30': ('motor1_velocity.urdf.xacro',
             'motor1_velocity_controllers.yaml', 'motor1_velocity_controller'),
    'dm4340': ('dm4340_velocity.urdf.xacro',
               'dm4340_velocity_controllers.yaml', 'dm4340_velocity_controller'),
}


def after_broadcaster_exit(event, context, controller, manager_exited=False):
    if context.is_shutdown or manager_exited:
        return []
    if event.returncode != 0:
        return [EmitEvent(event=Shutdown(reason='joint state broadcaster failed'))]
    return [controller]


def start_selected_backend(context):
    backend = LaunchConfiguration('backend').perform(context)
    device_path = LaunchConfiguration('device_path').perform(context)
    if backend not in BACKENDS:
        raise ValueError('backend must be ak30, dm4340 or dm4340_dual')
    if not re.fullmatch(r'/dev/ttyACM[0-9]+', device_path):
        raise ValueError('device_path must name a /dev/ttyACM device')

    urdf, controllers, controller = BACKENDS[backend]
    control_mode = LaunchConfiguration('dm4340_command_mode').perform(context)
    if control_mode not in ('velocity', 'effort', 'position'):
        raise ValueError('dm4340_command_mode must be velocity, effort or position')
    if backend not in ('dm4340', 'dm4340_dual') and control_mode != 'velocity':
        raise ValueError('non-velocity mode is supported only by dm4340')
    if backend in ('dm4340', 'dm4340_dual') and control_mode != 'velocity':
        controller = f'dm4340_{control_mode}_controller'
    bench = LaunchConfiguration('supervised_effort_bench').perform(context)
    if bench not in ('true', 'false') or (bench == 'true' and
                                          (backend != 'dm4340' or
                                           control_mode != 'effort')):
        raise ValueError('supervised_effort_bench requires dm4340 effort mode')
    effort_profile = LaunchConfiguration('dm4340_effort_profile', default='standard').perform(context)
    if effort_profile not in ('standard', 'torque8', 'training'):
        raise ValueError('dm4340_effort_profile must be standard, torque8 or training')
    if effort_profile in ('torque8', 'training'):
        if backend != 'dm4340_dual' or control_mode != 'effort' or bench != 'false':
            raise ValueError(f'{effort_profile} requires dm4340_dual standard effort mode')
        controllers = ('dm4340_dual_effort8_controllers.yaml' if effort_profile == 'torque8'
                       else 'dm4340_dual_training_controllers.yaml')
    share = FindPackageShare('mech_bringup')
    xacro_command = [
        FindExecutable(name='xacro'), ' ',
        PathJoinSubstitution([share, 'config', urdf]),
        ' device_path:=', device_path,
    ]
    if effort_profile in ('torque8', 'training'):
        cap = '8' if effort_profile == 'torque8' else '0.2'
        xacro_command.extend([f' effort_limit_nm:={cap} feedback_torque_limit_nm:=8'])
    if backend in ('dm4340', 'dm4340_dual'):
        xacro_command.extend([' ', 'command_mode:=', control_mode,
                              ' supervised_effort_bench:=', bench])
        for name in ('motor_side_timeout_verified', 'board_version_verified',
                     'can_bitrate_verified', 'joint_mapping_verified'):
            value = LaunchConfiguration(name).perform(context)
            if value not in ('true', 'false'):
                raise ValueError(f'{name} must be true or false')
            if value != 'true' and not (bench == 'true' and name in (
                    'motor_side_timeout_verified', 'joint_mapping_verified')):
                raise ValueError(f'{name} must be verified before DM4340 launch')
            if (bench == 'true' and name in (
                    'motor_side_timeout_verified', 'joint_mapping_verified') and
                    value != 'false'):
                raise ValueError(f'{name} must remain false in supervised bench mode')
            xacro_command.extend([' ', name, ':=', value])
        enable = LaunchConfiguration('enable_on_activate').perform(context)
        if enable not in ('true', 'false'):
            raise ValueError('enable_on_activate must be true or false')
        xacro_command.extend([' ', 'enable_on_activate:=', enable])
        feedforward = LaunchConfiguration('velocity_feedforward_nm').perform(context)
        if (not re.fullmatch(r'[0-9]+(?:\.[0-9]+)?', feedforward) or
                float(feedforward) > 0.5):
            raise ValueError('velocity_feedforward_nm must be in [0, 0.5]')
        if control_mode != 'velocity' and float(feedforward) != 0:
            raise ValueError('velocity_feedforward_nm requires velocity mode')
        xacro_command.extend([
            ' ', 'velocity_feedforward_nm:=', feedforward,
        ])
        kd = LaunchConfiguration('velocity_kd').perform(context)
        if not re.fullmatch(r'[0-9]+(?:\.[0-9]+)?', kd) or not 0 < float(kd) <= 3:
            raise ValueError('velocity_kd must be in (0, 3]')
        xacro_command.extend([' ', 'velocity_kd:=', kd])
    if backend == 'dm4340' and control_mode == 'position':
        for name in ('position_kp', 'position_kd', 'max_target_error_rad'):
            value = LaunchConfiguration(name).perform(context)
            if not re.fullmatch(r'[0-9]+(?:\.[0-9]+)?', value) or float(value) <= 0:
                raise ValueError(f'{name} must be positive numeric')
            xacro_command.extend([' ', name, ':=', value])
    if backend == 'dm4340_dual':
        controller = f'dm4340_motor1_{control_mode}_controller'
        for axis in (1, 2):
            for suffix in ('command_can_id', 'feedback_can_id', 'motor_id',
                           'firmware_id', 'joint_scale', 'joint_offset_rad',
                           'position_min_rad', 'position_max_rad',
                           'position_guard_margin_rad', 'position_kp', 'position_kd',
                           'max_target_error_rad'):
                name = f'motor{axis}_{suffix}'
                value = LaunchConfiguration(name).perform(context)
                # Passed through a Command substitution: reject shell metacharacters.
                if not re.fullmatch(r'-?[0-9]+(?:\.[0-9]+)?', value):
                    raise ValueError(f'{name} must be numeric')
                xacro_command.extend([' ', name, ':=', value])
    robot_description = ParameterValue(Command(xacro_command), value_type=str)
    broadcaster = Node(
        package='controller_manager', executable='spawner',
        arguments=['joint_state_broadcaster'],
    )
    selected = Node(
        package='controller_manager', executable='spawner',
        arguments=([controller, f'dm4340_motor2_{control_mode}_controller', '--inactive']
                   if backend == 'dm4340_dual' else [controller, '--inactive']),
    )
    manager_parameters = [
        {'robot_description': robot_description},
        PathJoinSubstitution([share, 'config', controllers]),
    ]
    if backend in ('dm4340', 'dm4340_dual'):
        manager_parameters.append(
            {'hardware_components_initial_state.inactive': ['dm4340_system']})
    manager = Node(
        package='controller_manager', executable='ros2_control_node',
        parameters=manager_parameters,
        output='both',
    )
    manager_state = {'exited': False}

    def manager_exit(event, ctx):
        manager_state['exited'] = True
        return [EmitEvent(event=Shutdown(reason='controller manager exited'))]

    return [
        manager,
        RegisterEventHandler(OnProcessExit(
            target_action=manager,
            on_exit=manager_exit,
        )),
        RegisterEventHandler(OnProcessExit(
            target_action=broadcaster,
            on_exit=lambda event, ctx: after_broadcaster_exit(
                event, ctx, selected, manager_state['exited']),
        )),
        broadcaster,
    ]


def generate_launch_description():
    dual_arguments = []
    for axis in (1, 2):
        for suffix, value in (
                ('command_can_id', str(axis)), ('motor_id', str(axis)),
                ('feedback_can_id', '0'), ('firmware_id', '5010107'),
                ('joint_scale', '1'), ('joint_offset_rad', '0'),
                ('position_min_rad', '-12.5'), ('position_max_rad', '12.5'),
                ('position_guard_margin_rad', '0.02'), ('position_kp', '1'),
                ('position_kd', '0.2'), ('max_target_error_rad', '0.3')):
            dual_arguments.append(DeclareLaunchArgument(f'motor{axis}_{suffix}',
                                                       default_value=value))
    return LaunchDescription(dual_arguments + [
        DeclareLaunchArgument('dm4340_effort_profile', default_value='standard'),
        DeclareLaunchArgument('backend', default_value='none'),
        DeclareLaunchArgument('device_path', default_value=''),
        DeclareLaunchArgument('motor_side_timeout_verified', default_value='false'),
        DeclareLaunchArgument('board_version_verified', default_value='false'),
        DeclareLaunchArgument('can_bitrate_verified', default_value='false'),
        DeclareLaunchArgument('joint_mapping_verified', default_value='false'),
        DeclareLaunchArgument('enable_on_activate', default_value='false'),
        DeclareLaunchArgument('velocity_feedforward_nm', default_value='0'),
        DeclareLaunchArgument('position_kp', default_value='1'),
        DeclareLaunchArgument('position_kd', default_value='0.2'),
        DeclareLaunchArgument('max_target_error_rad', default_value='0.3'),
        DeclareLaunchArgument('velocity_kd', default_value='3'),
        DeclareLaunchArgument('dm4340_command_mode', default_value='velocity'),
        DeclareLaunchArgument('supervised_effort_bench', default_value='false'),
        OpaqueFunction(function=start_selected_backend),
    ])
