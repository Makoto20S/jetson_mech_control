"""Offline DM4310 calibrated launch checks: never executes a ROS process."""
import copy
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
import xml.etree.ElementTree as ET

from launch import LaunchContext
import yaml


class Dm4310LaunchTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.root = Path(__file__).resolve().parents[1]
        spec = importlib.util.spec_from_file_location(
            'dm4310_launch', cls.root/'launch/dm4310_dual_bringup.launch.py')
        cls.module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(cls.module)

    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.path = Path(self.tmp.name)/'range.json'
        self.data = {'schema': 'dm4310-shaft-range-candidate-v1',
                     'model': 'DM4310', 'coordinate': 'motor_feedback_rad',
                     'calibration_status': 'valid',
                     'encoding_ranges_assumed': [12.5,30.,10.],
                     'mechanical_inset_deg': 5., 'axes': []}
        for axis, endpoints in ((1,(-1.,1.)), (2,(-2.5,-.5))):
            lo, hi = endpoints
            self.data['axes'].append({
                'motor_id': axis, 'command_can_id': axis, 'feedback_can_id': axis+3,
                'joint_scale': 1., 'joint_offset_rad': 0.,
                'position_min_rad': lo+.1, 'position_max_rad': hi-.1,
                'position_guard_margin_rad': .02,
                'endpoint_a': dict(median_rad=lo,observed_min_rad=lo,observed_max_rad=lo),
                'endpoint_b': dict(median_rad=hi,observed_min_rad=hi,observed_max_rad=hi)})
        self.write(self.data)

    def write(self, data):
        self.path.write_text(json.dumps(data))

    def context(self):
        context = LaunchContext()
        context.launch_configurations.update(
            range_file=str(self.path), device_path='/dev/ttyACM1',
            command_mode='effort', check_only='false', motor1_firmware_id='1', motor2_firmware_id='1')
        for key in ('ranges_accepted','board_version_verified','can_bitrate_verified',
                    'motor_side_timeout_verified','joint_mapping_verified',
                    'firmware_evidence_declared','limits_evidence_declared'):
            context.launch_configurations[key] = 'true'  # Synthetic test declaration only.
        return context

    def test_each_missing_declaration_blocks_before_nodes(self):
        for key in ('ranges_accepted','board_version_verified','can_bitrate_verified',
                    'motor_side_timeout_verified','joint_mapping_verified',
                    'firmware_evidence_declared','limits_evidence_declared'):
            ctx = self.context(); ctx.launch_configurations[key] = 'false'
            with self.subTest(key=key), patch.object(self.module,'Node') as node:
                with self.assertRaises(ValueError):
                    self.module.start(ctx)
                node.assert_not_called()

    def test_default_check_renders_limits_without_evidence_or_device_nodes(self):
        ctx = LaunchContext()
        ctx.launch_configurations['range_file'] = str(self.path)
        with patch.object(self.module,'get_package_share_directory',return_value=str(self.root)), \
                patch.object(self.module,'Node') as node, \
                patch.object(self.module.tempfile,'NamedTemporaryFile') as temp:
            actions = self.module.start(ctx)
            self.assertEqual(len(actions),1)
            node.assert_not_called()
            temp.assert_not_called()

    def test_fixed_impedance_offline_synchronizes_pd_cap_and_absolute_bounds(self):
        ctx = self.context()
        ctx.launch_configurations.update(check_only='true', fixed_position_impedance='true')
        observed = []
        original = self.module.controller_parameters
        def capture(mapping):
            observed.append(dict(mapping))
            return original(mapping)
        with patch.object(self.module, 'get_package_share_directory', return_value=str(self.root)), \
                patch.object(self.module, 'controller_parameters', side_effect=capture), \
                patch.object(self.module, 'Node') as node:
            self.module.start(ctx)
            node.assert_not_called()
        mapping = observed[0]
        self.assertEqual(mapping['effort_limit_nm'], '7.0')
        self.assertEqual(mapping['feedback_torque_limit_nm'], '8.0')
        params = original(mapping)
        for axis in (1, 2):
            name = f'dm4310_motor{axis}_effort_controller'
            self.assertEqual(params['controller_manager']['ros__parameters'][name]['type'],
                             'mech_controllers/FixedPositionImpedanceController')
            p = params[name]['ros__parameters']
            self.assertEqual((p['stiffness_nm_per_rad'], p['damping'], p['torque_limit_nm']),
                             (15., .5, 7.))
            self.assertEqual((p['minimum'], p['maximum']), (0., 1.))
            self.assertEqual(p['max_velocity_rad_s'], 30.)
            self.assertEqual(p['position_min_rad'], float(mapping[f'motor{axis}_position_min_rad']))

    def test_structured_impedance_source_is_explicit_and_rejects_other_profiles(self):
        ctx = self.context()
        ctx.launch_configurations.update(check_only='true', fixed_position_impedance='true',
                                         impedance_command_source='training_command')
        mappings = []
        original = self.module.controller_parameters
        def capture(mapping):
            mappings.append(dict(mapping))
            return original(mapping)
        with patch.object(self.module, 'get_package_share_directory', return_value=str(self.root)), \
                patch.object(self.module, 'controller_parameters', side_effect=capture), \
                patch.object(self.module, 'Node') as node:
            self.module.start(ctx)
            node.assert_not_called()
        params = original(mappings[0])
        for axis in (1, 2):
            self.assertEqual(params[f'dm4310_motor{axis}_effort_controller']['ros__parameters']['command_source'],
                             'training_command')
        ctx.launch_configurations['fixed_position_impedance'] = 'false'
        with patch.object(self.module, 'Node') as node, self.assertRaises(ValueError):
            self.module.start(ctx)
        node.assert_not_called()

    def test_fixed_impedance_rejects_conflicts_or_invalid_run_before_device_access(self):
        extras = [{'resistance_training_fast': 'true'}, {'resistance_training_2nm': 'true'},
                  {'resistance_training_8nm': 'true'}, {'command_mode': 'position'},
                  {'fixed_position_impedance': 'invalid'}]
        extras += [{'impedance_run_limit_s': value} for value in ('0', '-1', '121', 'nan', 'inf')]
        for extra in extras:
            ctx = self.context()
            ctx.launch_configurations['fixed_position_impedance'] = 'true'
            ctx.launch_configurations.update(extra)
            with self.subTest(extra=extra), patch.object(self.module, 'Node') as node:
                with self.assertRaises(ValueError): self.module.start(ctx)
                node.assert_not_called()

    def test_fixed_impedance_device_description_keeps_native_gains_zero_and_inactive(self):
        ctx = self.context()
        ctx.launch_configurations['fixed_position_impedance'] = 'true'
        with patch.object(self.module, 'get_package_share_directory', return_value=str(self.root)), \
                patch.object(self.module, 'Node') as node:
            self.module.start(ctx)
        calls = [call.kwargs for call in node.call_args_list]
        manager = next(call for call in calls if call['executable'] == 'ros2_control_node')
        parameter_path = Path(manager['parameters'][0])
        self.addCleanup(parameter_path.unlink, missing_ok=True)
        robot = ET.fromstring(manager['parameters'][1]['robot_description'])
        self.assertEqual(robot.find('ros2_control/hardware/param[@name="enable_on_activate"]').text, 'false')
        for joint in robot.findall('ros2_control/joint'):
            p = {item.attrib['name']: item.text for item in joint.findall('param')}
            self.assertEqual((float(p['kp']), float(p['kd'])), (0., 0.))
            self.assertEqual(float(p['max_command_effort_nm']), 7.)
            self.assertEqual(float(p['max_feedback_torque_nm']), 8.)
        for call in calls:
            if call['executable'] == 'spawner':
                self.assertIn('--inactive', call['arguments'])

    def test_default_range_path_respects_user_config_home(self):
        with patch.dict(self.module.os.environ,{'XDG_CONFIG_HOME':self.tmp.name}):
            self.assertEqual(self.module.default_range_file(),
                             str(Path(self.tmp.name)/'jetson_mech_control/dm4310_limits.json'))

    def test_fast_resistance_is_offline_and_uses_local_feedback_controller(self):
        ctx = self.context()
        ctx.launch_configurations.update(check_only='true', resistance_training_fast='true')
        with patch.object(self.module, 'get_package_share_directory', return_value=str(self.root)), \
                patch.object(self.module, 'Node') as node:
            self.module.start(ctx)
            node.assert_not_called()
        mapping = self.module.range_mappings(self.path)
        mapping.update(resistance_fast='true', effort_limit_nm='8', feedback_torque_limit_nm='8')
        params = self.module.controller_parameters(mapping)
        manager = params['controller_manager']['ros__parameters']
        self.assertEqual(manager['update_rate'], 500)
        for axis in (1, 2):
            name = f'dm4310_motor{axis}_effort_controller'
            self.assertEqual(manager[name]['type'], 'mech_controllers/ResistanceTrainingController')
            p = params[name]['ros__parameters']
            self.assertEqual((p['minimum'], p['maximum']), (0., 1.))
            self.assertEqual(p['torque_limit_nm'], 8.)
            self.assertEqual(p['damping'], 8.)
            self.assertEqual(p['direction_confirmation_s'], .02)
            self.assertEqual(p['position_min_rad'], float(mapping[f'motor{axis}_position_min_rad']))
            self.assertEqual(p['position_max_rad'], float(mapping[f'motor{axis}_position_max_rad']))

    def test_fast_profile_rejects_legacy_mix_or_non_effort_mode(self):
        for extra in ({'resistance_training_8nm': 'true'}, {'command_mode': 'velocity'}):
            ctx = self.context()
            ctx.launch_configurations.update(resistance_training_fast='true', **extra)
            with patch.object(self.module, 'Node') as node:
                with self.assertRaises(ValueError): self.module.start(ctx)
                node.assert_not_called()

    def test_fast_run_budget_is_bounded_before_device_access(self):
        for value in ('0', '-1', '121', 'nan', 'inf'):
            ctx = self.context()
            ctx.launch_configurations.update(resistance_training_fast='true', resistance_run_limit_s=value)
            with patch.object(self.module, 'Node') as node:
                with self.assertRaises(ValueError): self.module.start(ctx)
                node.assert_not_called()
        mapping = self.module.range_mappings(self.path)
        mapping.update(resistance_fast='true', effort_limit_nm='8', resistance_run_limit_s='20')
        params = self.module.controller_parameters(mapping)
        self.assertEqual(params['dm4310_motor1_effort_controller']['ros__parameters']['run_limit_s'], 20.)

    def test_fast_upper_layer_is_preview_only_and_permission_topics_only(self):
        path = self.root.parent/'mech_assist_control/config/dm4310_resistance_fast_preview.yaml'
        p = yaml.safe_load(path.read_text())['exoskeleton_control']['ros__parameters']
        self.assertEqual(p['resistance_execution'], 'controller')
        self.assertEqual(p['output_mode'], 'preview')
        self.assertFalse(p['allow_motor_output'])
        self.assertTrue(all(topic.endswith('/enable') for topic in p['controller_topics']))

    def test_training_trip_can_be_omitted_without_changing_encoding_or_other_limits(self):
        self.data['encoding_ranges_assumed'] = [12.5, 50., 10.]
        self.write(self.data)
        ctx = self.context()
        ctx.launch_configurations.update(check_only='true', resistance_training_fast='true',
            resistance_velocity_guard='false', resistance_damping='12',
            resistance_buildup_nm_per_second='20')
        original = self.module.controller_parameters
        observed = []
        def capture(mapping):
            observed.append(dict(mapping))
            return original(mapping)
        with patch.object(self.module, 'get_package_share_directory', return_value=str(self.root)), \
                patch.object(self.module, 'controller_parameters', side_effect=capture), \
                patch.object(self.module, 'Node') as node:
            self.module.start(ctx)
            node.assert_not_called()
        params = original(observed[0])
        for axis in (1, 2):
            p = params[f'dm4310_motor{axis}_effort_controller']['ros__parameters']
            self.assertEqual(p['max_velocity_rad_s'], 50.)
            self.assertEqual((p['damping'], p['buildup_nm_per_second']), (12., 20.))
            self.assertEqual(p['torque_limit_nm'], 8.)
            self.assertEqual(p['position_min_rad'], self.data['axes'][axis-1]['position_min_rad'])

    def test_speed_trip_option_and_training_gains_reject_invalid_values_before_nodes(self):
        for extra in ({'resistance_velocity_guard': 'false'},
                {'resistance_training_fast': 'true', 'resistance_velocity_guard': 'invalid'},
                *({'resistance_training_fast': 'true', key: value}
                  for key in ('resistance_damping', 'resistance_buildup_nm_per_second')
                  for value in ('0', '-1', 'nan', 'inf'))):
            ctx = self.context()
            ctx.launch_configurations.update(extra)
            with patch.object(self.module, 'Node') as node:
                with self.assertRaises(ValueError): self.module.start(ctx)
                node.assert_not_called()

    def test_unverified_power_cycle_reference_blocks_before_nodes(self):
        for state in (None, 'requires_endpoint_recheck_after_power_cycle'):
            data = copy.deepcopy(self.data)
            if state is None:
                data.pop('calibration_status')
            else:
                data['calibration_status'] = state
            self.write(data)
            with self.subTest(state=state), patch.object(self.module,'Node') as node:
                with self.assertRaises(ValueError):
                    self.module.start(self.context())
                node.assert_not_called()

    def test_device_vmax50_is_used_without_changing_position_bounds(self):
        self.data['encoding_ranges_assumed'] = [12.5,50.,10.]
        self.write(self.data)
        mapping = self.module.range_mappings(self.path)
        self.assertEqual(float(mapping['limit_velocity_rad_s']),50)
        import xacro
        robot = ET.fromstring(xacro.process_file(
            str(self.root/'config/dm4310_dual.urdf.xacro'), mappings=mapping).toxml())
        for joint in robot.findall('ros2_control/joint'):
            self.assertEqual(float(joint.find('param[@name="limit_velocity_rad_s"]').text),50)
        for joint in robot.findall('joint'):
            self.assertEqual(float(joint.find('limit').attrib['velocity']),50)

    def assert_resistance_cap(self, cap):
        ctx = self.context()
        ctx.launch_configurations[f'resistance_training_{cap}nm'] = 'true'
        with patch.object(self.module,'get_package_share_directory', return_value=str(self.root)), \
                patch.object(self.module,'Node') as node:
            self.module.start(ctx)
        manager = next(call.kwargs for call in node.call_args_list
                       if call.kwargs['executable'] == 'ros2_control_node')
        path = Path(manager['parameters'][0])
        self.addCleanup(path.unlink, missing_ok=True)
        params = yaml.safe_load(path.read_text())
        app = yaml.safe_load((self.root.parent/'mech_assist_control/config/'
            f'dm4310_resistance_training_{cap}nm_preview.yaml').read_text())['exoskeleton_control']['ros__parameters']
        self.assertEqual(app['max_command'], [float(cap), float(cap)])
        self.assertEqual(app['max_actuator_command'], [float(cap), float(cap)])
        self.assertFalse(app['allow_motor_output'])
        self.assertEqual(app['output_mode'], 'preview')
        robot = ET.fromstring(manager['parameters'][1]['robot_description'])
        for axis, joint in enumerate(robot.findall('ros2_control/joint'), 1):
            p = {item.attrib['name']: item.text for item in joint.findall('param')}
            self.assertEqual(float(p['max_command_effort_nm']), float(cap))
            self.assertEqual(float(p['kp']), 0.)
            self.assertEqual(float(p['kd']), 0.)
            controller = params[f'dm4310_motor{axis}_effort_controller']['ros__parameters']
            self.assertEqual((controller['minimum'], controller['maximum']), (-float(cap), float(cap)))
            self.assertEqual(controller['max_slew_per_second'], 500.*cap)
            self.assertEqual(float(p['max_feedback_torque_nm']), 8. if cap == 8 else 5.)

    def test_resistance_cap_matches_application_controller_and_lower(self):
        self.assert_resistance_cap(2)

    def test_resistance_8nm_cap_matches_all_layers(self):
        self.assert_resistance_cap(8)

    def test_output_cap_above_feedback_trip_rejected_offline(self):
        with self.assertRaises(ValueError):
            self.module.validate_effort_caps({'effort_limit_nm': '8', 'feedback_torque_limit_nm': '5'})
        self.module.validate_effort_caps({'effort_limit_nm': '8', 'feedback_torque_limit_nm': '8'})

    def test_conflicting_training_caps_rejected_before_nodes(self):
        ctx = self.context()
        ctx.launch_configurations.update(resistance_training_2nm='true', resistance_training_8nm='true')
        with patch.object(self.module, 'Node') as node:
            with self.assertRaises(ValueError):
                self.module.start(ctx)
            node.assert_not_called()

    def test_resistance_flag_rejects_native_pd_before_nodes(self):
        ctx = self.context()
        ctx.launch_configurations.update(resistance_training_2nm='true', command_mode='position')
        with patch.object(self.module, 'Node') as node:
            with self.assertRaises(ValueError):
                self.module.start(ctx)
            node.assert_not_called()

    def test_invalid_or_widened_ranges_are_rejected(self):
        for key, value in (('position_min_rad', -2.), ('position_max_rad', float('nan')),
                           ('feedback_can_id', 0), ('joint_scale', -1),
                           ('position_guard_margin_rad', 1.)):
            data = copy.deepcopy(self.data); data['axes'][0][key] = value
            self.write(data)
            with self.subTest(key=key), self.assertRaises(ValueError):
                self.module.range_mappings(self.path)

    def test_model_ranges_and_controller_bounds_agree_and_stay_inactive(self):
        with patch.object(self.module,'get_package_share_directory', return_value=str(self.root)), \
                patch.object(self.module,'Node') as node:
            self.module.start(self.context())
        calls = [call.kwargs for call in node.call_args_list]
        manager = next(call for call in calls if call['executable']=='ros2_control_node')
        parameter_path = Path(manager['parameters'][0])
        self.addCleanup(parameter_path.unlink, missing_ok=True)
        params = yaml.safe_load(parameter_path.read_text())
        robot = ET.fromstring(manager['parameters'][1]['robot_description'])
        control = robot.find('ros2_control')
        self.assertEqual(control.find('hardware/param[@name="profile"]').text,'dm4310_mit_standard')
        self.assertEqual(control.find('hardware/param[@name="enable_on_activate"]').text,'false')
        for index, joint in enumerate(control.findall('joint'), 1):
            p = {item.attrib['name']:item.text for item in joint.findall('param')}
            self.assertEqual(float(p['limit_velocity_rad_s']),30)
            self.assertEqual(float(p['limit_torque_nm']),10)
            self.assertEqual(int(p['feedback_can_id']),index+3)
            controller = params[f'dm4310_motor{index}_position_controller']['ros__parameters']
            self.assertEqual(controller['minimum'],float(p['position_min_rad']))
            self.assertEqual(controller['maximum'],float(p['position_max_rad']))
        for call in calls:
            if call['executable']=='spawner':
                self.assertIn('--inactive',call['arguments'])


if __name__ == '__main__':
    unittest.main()
