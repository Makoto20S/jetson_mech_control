"""Ordinary dual DM4310, measured shaft bounds, inactive hardware/controllers.

Reading/rendering configuration is offline. Launching a manager opens the device
only after all declarations below are explicit; this is not motion authorization.
"""
import json
import math
import os
from pathlib import Path
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnShutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro
import yaml


def default_range_file():
    config_home = Path(os.environ.get('XDG_CONFIG_HOME', Path.home()/'.config'))
    return str(config_home/'jetson_mech_control/dm4310_limits.json')


def range_mappings(path):
    data = json.loads(Path(path).read_text())
    if data.get('calibration_status') != 'valid':
        raise ValueError('position reference requires endpoint recheck after power cycle')
    if (data['schema'] != 'dm4310-shaft-range-candidate-v1' or
            data['model'] != 'DM4310' or data['coordinate'] != 'motor_feedback_rad' or
            data['encoding_ranges_assumed'] not in ([12.5, 30.0, 10.0], [12.5, 50.0, 10.0]) or
            len(data['axes']) != 2):
        raise ValueError('requires ordinary dual DM4310 measured shaft range file')
    # Device VMAX can differ from the model SDK default. Never infer voltage
    # variant from VMAX or silently change drive registers to fit a default.
    mappings = {'limit_velocity_rad_s': str(data['encoding_ranges_assumed'][1])}
    inset = math.radians(data['mechanical_inset_deg'])
    if not math.isfinite(inset) or inset <= .02:
        raise ValueError('invalid measured endpoint inset')
    for motor, axis in enumerate(data['axes'], 1):
        if (axis['motor_id'] != motor or axis['command_can_id'] != motor or
                axis['feedback_can_id'] != motor+3 or
                axis['joint_scale'] != 1 or axis['joint_offset_rad'] != 0):
            raise ValueError('range file identity/mapping mismatch; recalibrate')
        lower, upper, guard = (axis[k] for k in (
            'position_min_rad', 'position_max_rad', 'position_guard_margin_rad'))
        if (not all(math.isfinite(v) for v in (lower, upper, guard)) or
                not -12.5 <= lower < upper <= 12.5 or
                guard <= 0 or guard >= inset or upper-lower <= 2*guard):
            raise ValueError('invalid measured position bounds')
        low, high = sorted((axis['endpoint_a'], axis['endpoint_b']),
                           key=lambda endpoint: endpoint['median_rad'])
        for endpoint in (low, high):
            values = [endpoint[k] for k in (
                'observed_min_rad', 'median_rad', 'observed_max_rad')]
            if (not all(math.isfinite(v) for v in values) or
                    not -12.5 <= values[0] <= values[1] <= values[2] <= 12.5 or
                    values[2]-values[0] > .005):
                raise ValueError('invalid endpoint observations')
        if (lower < low['observed_max_rad']+inset-1e-9 or
                upper > high['observed_min_rad']-inset+1e-9):
            raise ValueError('limits extend beyond inset measured endpoints')
        for suffix, value in (('position_min_rad', lower),
                              ('position_max_rad', upper),
                              ('position_guard_margin_rad', guard)):
            mappings[f'motor{motor}_{suffix}'] = str(value)
    return mappings


def controller_parameters(mappings):
    manager = {'update_rate': 500,
               'joint_state_broadcaster': {'type': 'joint_state_broadcaster/JointStateBroadcaster'}}
    output = {'controller_manager': {'ros__parameters': manager},
              'joint_state_broadcaster': {'ros__parameters': {'use_local_topics': False}}}
    for motor in (1, 2):
        for mode in ('position', 'velocity', 'effort'):
            name = f'dm4310_motor{motor}_{mode}_controller'
            manager[name] = {'type': f'mech_controllers/{mode.title()}CommandController'}
            cap = float(mappings.get('effort_limit_nm', '.1')) if mode == 'effort' else .3
            output[name] = {'ros__parameters': {
                'joint': f'dm4310_motor{motor}_joint',
                'minimum': float(mappings[f'motor{motor}_position_min_rad']) if mode == 'position' else -cap,
                'maximum': float(mappings[f'motor{motor}_position_max_rad']) if mode == 'position' else cap,
                'max_slew_per_second': (500.0*cap if mode == 'effort' and cap in (2.0, 8.0)
                                        else (1.0 if mode != 'position' else .03)),
                'target_ttl_nanoseconds': 100000000,
                'target_hard_ttl_nanoseconds': 106000000}}
            if mode == 'effort' and mappings.get('resistance_fast') == 'true':
                manager[name]['type'] = 'mech_controllers/ResistanceTrainingController'
                output[name]['ros__parameters'].update(
                    minimum=0.0, maximum=1.0, max_slew_per_second=1000000.0,
                    damping=float(mappings.get('resistance_damping', '8.0')),
                    torque_limit_nm=cap,
                    buildup_nm_per_second=float(mappings.get('resistance_buildup_nm_per_second', '8.0')),
                    velocity_filter_tau_s=.025, stop_velocity_rad_s=.02,
                    start_velocity_rad_s=.06, direction_confirmation_s=.02,
                    max_velocity_rad_s=float(mappings.get('feedback_velocity_limit_rad_s', '1.0')),
                    position_min_rad=float(mappings[f'motor{motor}_position_min_rad']),
                    position_max_rad=float(mappings[f'motor{motor}_position_max_rad']),
                    position_margin_rad=float(mappings[f'motor{motor}_position_guard_margin_rad']),
                    max_update_period_s=.006,
                    run_limit_s=float(mappings.get('resistance_run_limit_s', '5.0')))
            if mode == 'effort' and mappings.get('fixed_position_impedance') == 'true':
                manager[name]['type'] = 'mech_controllers/FixedPositionImpedanceController'
                output[name]['ros__parameters'].update(
                    minimum=0.0, maximum=1.0, max_slew_per_second=1000000.0,
                    command_source=mappings.get('impedance_command_source', 'enable'),
                    stiffness_nm_per_rad=15.0, damping=.5, torque_limit_nm=cap,
                    activation_velocity_rad_s=.05,
                    max_velocity_rad_s=float(mappings['feedback_velocity_limit_rad_s']),
                    position_min_rad=float(mappings[f'motor{motor}_position_min_rad']),
                    position_max_rad=float(mappings[f'motor{motor}_position_max_rad']),
                    position_margin_rad=float(mappings[f'motor{motor}_position_guard_margin_rad']),
                    max_update_period_s=.006,
                    run_limit_s=float(mappings.get('impedance_run_limit_s', '5.0')))
    return output


def validate_effort_caps(mappings):
    cap = float(mappings.get('effort_limit_nm', '.1'))
    trip = float(mappings.get('feedback_torque_limit_nm', '5'))
    if not (math.isfinite(cap) and math.isfinite(trip) and 0 < cap <= trip <= 10):
        raise ValueError('requires command effort cap <= feedback torque trip <= encoding TMAX10')


def start(context):
    mappings = range_mappings(LaunchConfiguration('range_file').perform(context))
    selected_caps = []
    for cap in (2, 8):
        key = f'resistance_training_{cap}nm'
        selected = LaunchConfiguration(key, default='false').perform(context)
        if selected not in ('true', 'false'):
            raise ValueError(f'{key} must be true or false')
        if selected == 'true':
            selected_caps.append(cap)
    if len(selected_caps) > 1:
        raise ValueError('select only one resistance training torque cap')
    fast = LaunchConfiguration('resistance_training_fast', default='false').perform(context)
    if fast not in ('true', 'false'):
        raise ValueError('resistance_training_fast must be true or false')
    if fast == 'true':
        if selected_caps:
            raise ValueError('fast resistance is a separate profile; do not combine with legacy caps')
        selected_caps = [8]
        mappings['resistance_fast'] = 'true'
        run_limit = float(LaunchConfiguration('resistance_run_limit_s', default='5.0').perform(context))
        if not math.isfinite(run_limit) or not 0 < run_limit <= 120:
            raise ValueError('resistance run limit requires 0 < seconds <= 120')
        mappings['resistance_run_limit_s'] = str(run_limit)
        guard = LaunchConfiguration('resistance_velocity_guard', default='true').perform(context)
        if guard not in ('true', 'false'):
            raise ValueError('resistance_velocity_guard must be true or false')
        # When the training trip is omitted, retain only the finite protocol
        # decoding range. Never use infinity or alter the drive's VMAX register.
        mappings['feedback_velocity_limit_rad_s'] = (
            '1.0' if guard == 'true' else mappings['limit_velocity_rad_s'])
        for key, default in (('resistance_damping', '8.0'),
                             ('resistance_buildup_nm_per_second', '8.0')):
            value = float(LaunchConfiguration(key, default=default).perform(context))
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f'{key} must be finite and positive')
            mappings[key] = str(value)
    elif LaunchConfiguration('resistance_velocity_guard', default='true').perform(context) != 'true':
        raise ValueError('omitting the training speed trip requires fast resistance effort profile')
    fixed = LaunchConfiguration('fixed_position_impedance', default='false').perform(context)
    if fixed not in ('true', 'false'):
        raise ValueError('fixed_position_impedance must be true or false')
    source = LaunchConfiguration('impedance_command_source', default='enable').perform(context)
    if source not in ('enable', 'training_command') or (source != 'enable' and fixed != 'true'):
        raise ValueError('training_command requires fixed position impedance profile')
    mappings['impedance_command_source'] = source
    if fixed == 'true':
        if selected_caps or fast == 'true':
            raise ValueError('fixed position impedance cannot combine with resistance profiles')
        if LaunchConfiguration('command_mode', default='effort').perform(context) != 'effort':
            raise ValueError('fixed position impedance requires pure effort mode')
        seconds = float(LaunchConfiguration('impedance_run_limit_s', default='5.0').perform(context))
        if not math.isfinite(seconds) or not 0 < seconds <= 120:
            raise ValueError('impedance run limit requires 0 < seconds <= 120')
        mappings.update(fixed_position_impedance='true', impedance_run_limit_s=str(seconds),
                        effort_limit_nm='7.0', feedback_torque_limit_nm='8.0',
                        feedback_velocity_limit_rad_s=mappings['limit_velocity_rad_s'])
    if selected_caps:
        if LaunchConfiguration('command_mode', default='effort').perform(context) != 'effort':
            raise ValueError('resistance training requires pure effort mode')
        mappings['effort_limit_nm'] = str(float(selected_caps[0]))
        if selected_caps[0] == 8:
            mappings['feedback_torque_limit_nm'] = '8.0'
    validate_effort_caps(mappings)
    check_only = LaunchConfiguration('check_only', default='true').perform(context)
    if check_only not in ('true', 'false'):
        raise ValueError('check_only must be true or false')
    if check_only == 'true':
        # Render and inspect the exact lower/controller configuration without
        # setting evidence declarations or constructing any device node.
        share = Path(get_package_share_directory('mech_bringup'))
        robot = xacro.process_file(str(share/'config/dm4310_dual.urdf.xacro'),
                                   mappings=dict(mappings)).toxml()
        import xml.etree.ElementTree as ET
        control = ET.fromstring(robot).find('ros2_control')
        controllers = controller_parameters(mappings)
        for axis, joint in enumerate(control.findall('joint'), 1):
            p = {item.attrib['name']: item.text for item in joint.findall('param')}
            controller = controllers[f'dm4310_motor{axis}_position_controller']['ros__parameters']
            for field, key in (('minimum','position_min_rad'),('maximum','position_max_rad')):
                wanted = float(mappings[f'motor{axis}_{key}'])
                actual = float(p[key])
                urdf_joint = ET.fromstring(robot).find(f'joint[@name="dm4310_motor{axis}_joint"]/limit')
                urdf_value = float(urdf_joint.attrib['lower' if field == 'minimum' else 'upper'])
                if actual != wanted or controller[field] != wanted or urdf_value != wanted:
                    raise ValueError('position bounds differ across deployment layers')
        for axis, joint in enumerate(control.findall('joint'), 1):
            if joint.find('command_interface[@name="effort"]') is not None:
                cap = float(joint.find('param[@name="max_command_effort_nm"]').text)
                controller = controllers[f'dm4310_motor{axis}_effort_controller']['ros__parameters']
                actual_cap = controller.get('torque_limit_nm', controller['maximum'])
                if (actual_cap != cap or (fast == 'false' and fixed == 'false' and controller['minimum'] != -cap)):
                    raise ValueError('effort limits differ across deployment layers')
                if (fast == 'true' or fixed == 'true') and controller['max_velocity_rad_s'] != float(
                        joint.find('param[@name="max_feedback_velocity_rad_s"]').text):
                    raise ValueError('resistance velocity validity bounds differ across layers')
        return [LogInfo(msg='DM4310 offline check passed: URDF/controller/lower position '
                            f'limits agree; no device opened. {json.dumps(mappings)}')]
    for key in ('ranges_accepted', 'board_version_verified', 'can_bitrate_verified',
                'motor_side_timeout_verified', 'joint_mapping_verified',
                'firmware_evidence_declared', 'limits_evidence_declared'):
        if LaunchConfiguration(key).perform(context) != 'true':
            raise ValueError(f'{key} must be confirmed before opening the device')
        if key != 'ranges_accepted':
            mappings[key] = 'true'
    for axis in (1, 2):
        key = f'motor{axis}_firmware_id'
        value = LaunchConfiguration(key).perform(context)
        if not value.isdecimal() or not 0 < int(value) <= 0xffffffff:
            raise ValueError(f'{key} requires current firmware evidence')
        mappings[key] = value
    mode = LaunchConfiguration('command_mode').perform(context)
    if mode not in ('effort', 'velocity', 'position'):
        raise ValueError('invalid command_mode')
    path = LaunchConfiguration('device_path').perform(context)
    if not path.startswith('/dev/ttyACM') or not path[11:].isdigit():
        raise ValueError('requires explicit /dev/ttyACMx')
    mappings.update(device_path=path, command_mode=mode, enable_on_activate='false')
    share = Path(get_package_share_directory('mech_bringup'))
    robot = xacro.process_file(str(share/'config/dm4310_dual.urdf.xacro'),
                               mappings=mappings).toxml()
    params = controller_parameters(mappings)
    with tempfile.NamedTemporaryFile(mode='w', prefix='dm4310-controllers-',
                                     suffix='.yaml', delete=False) as stream:
        yaml.safe_dump(params, stream)
        parameter_path = stream.name

    def cleanup(event, ctx):
        Path(parameter_path).unlink(missing_ok=True)
        return []

    return [
        RegisterEventHandler(OnShutdown(on_shutdown=cleanup)),
        Node(package='controller_manager', executable='ros2_control_node',
             parameters=[parameter_path, {'robot_description': robot,
                 'hardware_components_initial_state.inactive': ['dm4310_system']}], output='both'),
        Node(package='controller_manager', executable='spawner',
             arguments=['joint_state_broadcaster', '--inactive']),
        # Motion controller activation is always a separate reviewed action.
        *[Node(package='controller_manager', executable='spawner',
               arguments=[f'dm4310_motor{axis}_{mode}_controller', '--inactive'])
          for axis in (1, 2)],
    ]


def generate_launch_description():
    return LaunchDescription([
        *[DeclareLaunchArgument(key, default_value='false') for key in (
            'ranges_accepted', 'board_version_verified', 'can_bitrate_verified',
            'motor_side_timeout_verified', 'joint_mapping_verified',
            'firmware_evidence_declared', 'limits_evidence_declared')],
        DeclareLaunchArgument('range_file', default_value=default_range_file()),
        DeclareLaunchArgument('check_only', default_value='true'),
        DeclareLaunchArgument('device_path', default_value='/dev/ttyACM1'),
        DeclareLaunchArgument('command_mode', default_value='effort'),
        DeclareLaunchArgument('resistance_training_2nm', default_value='false'),
        DeclareLaunchArgument('resistance_training_8nm', default_value='false'),
        DeclareLaunchArgument('resistance_training_fast', default_value='false'),
        DeclareLaunchArgument('fixed_position_impedance', default_value='false'),
        DeclareLaunchArgument('impedance_run_limit_s', default_value='5.0'),
        DeclareLaunchArgument('impedance_command_source', default_value='enable'),
        DeclareLaunchArgument('resistance_run_limit_s', default_value='5.0'),
        DeclareLaunchArgument('resistance_velocity_guard', default_value='true'),
        DeclareLaunchArgument('resistance_damping', default_value='8.0'),
        DeclareLaunchArgument('resistance_buildup_nm_per_second', default_value='8.0'),
        DeclareLaunchArgument('motor1_firmware_id', default_value='0'),
        DeclareLaunchArgument('motor2_firmware_id', default_value='0'),
        OpaqueFunction(function=start),
    ])
