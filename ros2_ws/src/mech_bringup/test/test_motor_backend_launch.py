"""Check backend selection without launching a device or controller manager."""

import importlib.util
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import patch
import xml.etree.ElementTree as ET

from launch import LaunchContext
from launch.actions import EmitEvent, RegisterEventHandler
from launch_ros.actions import Node
from launch_ros.utilities import evaluate_parameters, normalize_parameters


class MotorBackendLaunchTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        source = Path(__file__).resolve().parents[1]
        spec = importlib.util.spec_from_file_location(
            'motor_backend_launch', source / 'launch/motor_backend_bringup.launch.py')
        cls.module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(cls.module)

    def context(self, backend, device_path='/dev/ttyACM1'):
        context = LaunchContext()
        context.launch_configurations['backend'] = backend
        context.launch_configurations['device_path'] = device_path
        context.launch_configurations['enable_on_activate'] = 'false'
        context.launch_configurations['velocity_feedforward_nm'] = '0'
        context.launch_configurations['velocity_kd'] = '3'
        for name, value in (('position_kp', '1'), ('position_kd', '0.2'),
                            ('max_target_error_rad', '0.3')):
            context.launch_configurations[name] = value
        context.launch_configurations['dm4340_command_mode'] = 'velocity'
        context.launch_configurations['supervised_effort_bench'] = 'false'
        for name in ('motor_side_timeout_verified', 'board_version_verified',
                     'can_bitrate_verified', 'joint_mapping_verified'):
            context.launch_configurations[name] = 'false'
        return context

    def dual_context(self, mode='velocity'):
        context = self.context('dm4340_dual')
        for action in self.module.generate_launch_description().entities:
            if hasattr(action, 'name') and action.name.startswith('motor'):
                context.launch_configurations[action.name] = ''.join(
                    part.perform(context) for part in action.default_value)
        context.launch_configurations['dm4340_command_mode'] = mode
        for name in ('motor_side_timeout_verified', 'board_version_verified',
                     'can_bitrate_verified', 'joint_mapping_verified'):
            context.launch_configurations[name] = 'true'
        return context

    def test_dual_torque8_profile_routes_matching_limits(self):
        import yaml
        context = self.dual_context('effort')
        context.launch_configurations['dm4340_effort_profile'] = 'torque8'
        with patch.object(self.module, 'Node', side_effect=Node) as node:
            self.module.start_selected_backend(context)
        manager = next(c.kwargs for c in node.call_args_list
                       if c.kwargs['executable'] == 'ros2_control_node')
        parameters = evaluate_parameters(LaunchContext(), normalize_parameters(manager['parameters']))
        control = ET.fromstring(parameters[0]['robot_description']).find('ros2_control')
        self.assertEqual(control.find('hardware/param[@name="enable_on_activate"]').text, 'false')
        for joint in control.findall('joint'):
            for name, value in (('max_command_effort_nm', 8), ('max_feedback_torque_nm', 8), ('kp', 0), ('kd', 0)):
                self.assertEqual(float(joint.find(f'param[@name="{name}"]').text), value)
        config = yaml.safe_load(Path(parameters[1]).read_text())
        for axis in (1, 2):
            limits = config[f'dm4340_motor{axis}_effort_controller']['ros__parameters']
            self.assertEqual((limits['minimum'], limits['maximum']), (-8, 8))
        context.launch_configurations['dm4340_command_mode'] = 'position'
        with patch.object(self.module, 'Node') as node:
            with self.assertRaisesRegex(ValueError, 'torque8'):
                self.module.start_selected_backend(context)
            node.assert_not_called()

    def test_dual_training_profile_routes_matching_limits(self):
        import yaml
        context = self.dual_context('effort')
        context.launch_configurations['dm4340_effort_profile'] = 'training'
        with patch.object(self.module, 'Node', side_effect=Node) as node:
            self.module.start_selected_backend(context)
        manager = next(c.kwargs for c in node.call_args_list
                       if c.kwargs['executable'] == 'ros2_control_node')
        parameters = evaluate_parameters(LaunchContext(), normalize_parameters(manager['parameters']))
        control = ET.fromstring(parameters[0]['robot_description']).find('ros2_control')
        self.assertEqual(control.find('hardware/param[@name="enable_on_activate"]').text, 'false')
        for joint in control.findall('joint'):
            for name, value in (('max_command_effort_nm', .2), ('max_feedback_torque_nm', 8), ('kp', 0), ('kd', 0)):
                self.assertEqual(float(joint.find(f'param[@name="{name}"]').text), value)
        config = yaml.safe_load(Path(parameters[1]).read_text())
        for axis in (1, 2):
            limits = config[f'dm4340_motor{axis}_effort_controller']['ros__parameters']
            self.assertEqual((limits['minimum'], limits['maximum']), (-.2, .2))
            self.assertEqual(limits['max_slew_per_second'], 100.0)
        context.launch_configurations['dm4340_command_mode'] = 'position'
        with patch.object(self.module, 'Node') as node:
            with self.assertRaisesRegex(ValueError, 'training'):
                self.module.start_selected_backend(context)
            node.assert_not_called()

    def test_dual_modes_one_manager_two_inactive_controllers(self):
        import yaml
        for mode in ('velocity', 'effort', 'position'):
            with self.subTest(mode=mode):
                context = self.dual_context(mode)
                # Exercise distinct mapping and limit overrides through actual xacro.
                context.launch_configurations['motor2_joint_scale'] = '-1'
                context.launch_configurations['motor2_position_min_rad'] = '-0.4'
                context.launch_configurations['motor2_position_max_rad'] = '0.8'
                with patch.object(self.module, 'Node', side_effect=Node) as node:
                    self.module.start_selected_backend(context)
                calls = [call.kwargs for call in node.call_args_list]
                managers = [c for c in calls if c['executable'] == 'ros2_control_node']
                self.assertEqual(len(managers), 1)
                parameters = evaluate_parameters(
                    LaunchContext(), normalize_parameters(managers[0]['parameters']))
                controls = ET.fromstring(parameters[0]['robot_description']).findall(
                    'ros2_control')
                self.assertEqual(len(controls), 1)
                self.assertEqual(controls[0].find('hardware/plugin').text,
                                 'mech_bringup/Dm4340System')
                self.assertEqual(controls[0].find(
                    'hardware/param[@name="enable_on_activate"]').text, 'false')
                self.assertEqual(parameters[2][
                    'hardware_components_initial_state.inactive'], ('dm4340_system',))
                joints = controls[0].findall('joint')
                self.assertEqual(len(joints), 2)
                with open(parameters[1]) as stream:
                    config = yaml.safe_load(stream)
                for i, joint in enumerate(joints, 1):
                    self.assertEqual(joint.attrib['name'], f'dm4340_motor{i}_joint')
                    for key, expected in (('command_can_id', i), ('motor_id', i),
                                          ('feedback_can_id', 0)):
                        self.assertEqual(int(joint.find(f'param[@name="{key}"]').text),
                                         expected)
                    self.assertEqual([x.attrib['name'] for x in
                                      joint.findall('command_interface')],
                                     [mode, 'command_generation'])
                    name = f'dm4340_motor{i}_{mode}_controller'
                    self.assertEqual(config[name]['ros__parameters']['joint'],
                                     joint.attrib['name'])
                if mode == 'position':
                    self.assertEqual(float(joints[1].find('param[@name="kp"]').text), 1)
                    self.assertEqual(float(joints[1].find('param[@name="kd"]').text), .2)
                    self.assertEqual(float(joints[1].find('param[@name="max_command_velocity_rad_s"]').text), 0)
                self.assertEqual(joints[1].find('param[@name="joint_scale"]').text, '-1')
                self.assertEqual(joints[1].find('param[@name="position_min_rad"]').text,
                                 '-0.4')
                self.assertTrue(any(c.get('arguments') == [
                    f'dm4340_motor1_{mode}_controller',
                    f'dm4340_motor2_{mode}_controller', '--inactive'] for c in calls))

    def test_removed_host_impedance_rejected_before_device_nodes(self):
        for backend in ('dm4340', 'dm4340_dual'):
            with self.subTest(backend=backend):
                context = self.dual_context('impedance')
                context.launch_configurations['backend'] = backend
                with patch.object(self.module, 'Node') as node:
                    with self.assertRaisesRegex(ValueError, 'dm4340_command_mode'):
                        self.module.start_selected_backend(context)
                    node.assert_not_called()

    def test_dual_rejects_single_motor_supervised_profile(self):
        context = self.dual_context('effort')
        context.launch_configurations['supervised_effort_bench'] = 'true'
        with self.assertRaises(ValueError):
            self.module.start_selected_backend(context)

    def test_dual_rejects_shell_tokens_before_xacro(self):
        context = self.dual_context()
        context.launch_configurations['motor2_feedback_can_id'] = '5;false'
        with self.assertRaises(ValueError):
            self.module.start_selected_backend(context)

    def test_only_selected_plugin_and_inactive_controller_are_loaded(self):
        for backend, plugin, controller in (
                ('ak30', 'mech_bringup/Ak30System', 'motor1_velocity_controller'),
                ('dm4340', 'mech_bringup/Dm4340System',
                 'dm4340_velocity_controller')):
            with self.subTest(backend=backend):
                calls = []

                def record_node(**kwargs):
                    node = Node(**kwargs)
                    calls.append(kwargs)
                    return node

                with patch.object(self.module, 'Node', side_effect=record_node):
                    context = self.context(backend)
                    if backend == 'dm4340':
                        for name in ('motor_side_timeout_verified', 'board_version_verified',
                                     'can_bitrate_verified', 'joint_mapping_verified'):
                            context.launch_configurations[name] = 'true'
                    actions = self.module.start_selected_backend(context)
                manager = next(args for args in calls
                               if args['executable'] == 'ros2_control_node')
                parameters = evaluate_parameters(
                    LaunchContext(), normalize_parameters(manager['parameters']))
                control = ET.fromstring(parameters[0]['robot_description']).find(
                    'ros2_control')
                self.assertEqual(control.find('hardware/plugin').text, plugin)
                self.assertEqual(control.find('hardware/param[@name="device_path"]').text,
                                 '/dev/ttyACM1')
                self.assertEqual(sum(args['executable'] == 'ros2_control_node'
                                     for args in calls), 1)
                self.assertEqual(sum(args.get('arguments') == [controller, '--inactive']
                                     for args in calls), 1)
                self.assertEqual(sum(isinstance(action, RegisterEventHandler)
                                     for action in actions), 2)
                if backend == 'dm4340':
                    self.assertEqual(control.find(
                        'hardware/param[@name="motor_side_timeout_verified"]').text,
                        'true')
                    self.assertEqual(control.find(
                        'hardware/param[@name="enable_on_activate"]').text,
                        'false')
                    self.assertEqual(control.find(
                        'joint/param[@name="velocity_feedforward_nm"]').text,
                        '0')
                    self.assertEqual(control.find('joint/param[@name="kd"]').text,
                                     '3')
                    self.assertEqual(
                        parameters[2]['hardware_components_initial_state.inactive'],
                        ('dm4340_system',))

    def test_single_position_mode_uses_native_gains_and_inactive_controller(self):
        context = self.context('dm4340')
        context.launch_configurations['dm4340_command_mode'] = 'position'
        context.launch_configurations['position_kp'] = '2'
        context.launch_configurations['position_kd'] = '0.4'
        for name in ('motor_side_timeout_verified', 'board_version_verified',
                     'can_bitrate_verified', 'joint_mapping_verified'):
            context.launch_configurations[name] = 'true'
        with patch.object(self.module, 'Node', side_effect=Node) as node:
            self.module.start_selected_backend(context)
        calls = [call.kwargs for call in node.call_args_list]
        self.assertTrue(any(c.get('arguments') ==
                            ['dm4340_position_controller', '--inactive'] for c in calls))
        manager = next(c for c in calls if c['executable'] == 'ros2_control_node')
        params = evaluate_parameters(LaunchContext(), normalize_parameters(manager['parameters']))
        joint = ET.fromstring(params[0]['robot_description']).find('ros2_control/joint')
        self.assertEqual([x.attrib['name'] for x in joint.findall('command_interface')],
                         ['position', 'command_generation'])
        self.assertEqual(float(joint.find('param[@name="kp"]').text), 2)
        self.assertEqual(float(joint.find('param[@name="kd"]').text), .4)
        self.assertEqual(float(joint.find('param[@name="max_command_velocity_rad_s"]').text), 0)
        context.launch_configurations['velocity_feedforward_nm'] = '0.1'
        with patch.object(self.module, 'Node') as node:
            with self.assertRaisesRegex(ValueError, 'requires velocity mode'):
                self.module.start_selected_backend(context)
            node.assert_not_called()
        context.launch_configurations['velocity_feedforward_nm'] = '0'
        context.launch_configurations['position_kp'] = 'nan'
        with patch.object(self.module, 'Node') as node:
            with self.assertRaisesRegex(ValueError, 'position_kp'):
                self.module.start_selected_backend(context)
            node.assert_not_called()

    def test_dm_effort_mode_claims_only_effort_controller(self):
        context = self.context('dm4340')
        context.launch_configurations['dm4340_command_mode'] = 'effort'
        for name in ('motor_side_timeout_verified', 'board_version_verified',
                     'can_bitrate_verified', 'joint_mapping_verified'):
            context.launch_configurations[name] = 'true'
        with patch.object(self.module, 'Node', side_effect=Node) as node:
            self.module.start_selected_backend(context)
        calls = [call.kwargs for call in node.call_args_list]
        self.assertEqual(sum(call.get('arguments') ==
                             ['dm4340_effort_controller', '--inactive']
                             for call in calls), 1)
        self.assertFalse(any(call.get('arguments') ==
                             ['dm4340_velocity_controller', '--inactive']
                             for call in calls))
        manager = next(call for call in calls
                       if call['executable'] == 'ros2_control_node')
        parameters = evaluate_parameters(
            LaunchContext(), normalize_parameters(manager['parameters']))
        control = ET.fromstring(parameters[0]['robot_description']).find(
            'ros2_control')
        self.assertEqual([item.attrib['name'] for item in
                          control.findall('joint/command_interface')],
                         ['effort', 'command_generation'])
        self.assertEqual(control.find('joint/param[@name="kd"]').text, '0')
        self.assertEqual(control.find(
            'joint/param[@name="max_command_effort_nm"]').text, '0.3')

    def test_supervised_effort_bench_keeps_unverified_flags_false(self):
        context = self.context('dm4340')
        context.launch_configurations['dm4340_command_mode'] = 'effort'
        context.launch_configurations['supervised_effort_bench'] = 'true'
        context.launch_configurations['board_version_verified'] = 'true'
        context.launch_configurations['can_bitrate_verified'] = 'true'
        with patch.object(self.module, 'Node', side_effect=Node) as node:
            self.module.start_selected_backend(context)
        manager = next(call.kwargs for call in node.call_args_list
                       if call.kwargs['executable'] == 'ros2_control_node')
        parameters = evaluate_parameters(
            LaunchContext(), normalize_parameters(manager['parameters']))
        control = ET.fromstring(parameters[0]['robot_description']).find(
            'ros2_control')
        self.assertEqual(control.find('hardware/param[@name="profile"]').text,
                         'dm4340_mit_supervised_effort_bench')
        self.assertEqual(control.find(
            'hardware/param[@name="motor_side_timeout_verified"]').text,
            'false')
        self.assertEqual(control.find(
            'joint/param[@name="joint_mapping_verified"]').text, 'false')
        self.assertEqual(control.find(
            'joint/param[@name="max_command_effort_nm"]').text, '0.1')
        context.launch_configurations['motor_side_timeout_verified'] = 'true'
        with patch.object(self.module, 'Node') as node:
            with self.assertRaisesRegex(ValueError, 'remain false'):
                self.module.start_selected_backend(context)
            node.assert_not_called()

    def test_dm_evidence_flags_are_explicit(self):
        context = self.context('dm4340')
        for name in ('motor_side_timeout_verified', 'board_version_verified',
                     'can_bitrate_verified', 'joint_mapping_verified'):
            context.launch_configurations[name] = 'true'
        with patch.object(self.module, 'Node', side_effect=Node) as node:
            self.module.start_selected_backend(context)
        manager = next(call.kwargs for call in node.call_args_list
                       if call.kwargs['executable'] == 'ros2_control_node')
        parameters = evaluate_parameters(
            LaunchContext(), normalize_parameters(manager['parameters']))
        control = ET.fromstring(parameters[0]['robot_description']).find(
            'ros2_control')
        for name in ('motor_side_timeout_verified', 'board_version_verified',
                     'can_bitrate_verified'):
            self.assertEqual(control.find(f'hardware/param[@name="{name}"]').text,
                             'true')
        self.assertEqual(control.find(
            'joint/param[@name="joint_mapping_verified"]').text, 'true')
        context.launch_configurations['enable_on_activate'] = 'true'
        context.launch_configurations['velocity_feedforward_nm'] = '0.4'
        context.launch_configurations['velocity_kd'] = '2'
        with patch.object(self.module, 'Node', side_effect=Node) as node:
            self.module.start_selected_backend(context)
        manager = next(call.kwargs for call in node.call_args_list
                       if call.kwargs['executable'] == 'ros2_control_node')
        parameters = evaluate_parameters(
            LaunchContext(), normalize_parameters(manager['parameters']))
        control = ET.fromstring(parameters[0]['robot_description']).find(
            'ros2_control')
        self.assertEqual(control.find(
            'hardware/param[@name="enable_on_activate"]').text, 'true')
        self.assertEqual(control.find(
            'joint/param[@name="velocity_feedforward_nm"]').text, '0.4')
        self.assertEqual(control.find('joint/param[@name="kd"]').text, '2')

    def test_unverified_dm_fails_before_node_creation(self):
        for missing in ('motor_side_timeout_verified', 'board_version_verified',
                        'can_bitrate_verified', 'joint_mapping_verified'):
            with self.subTest(missing=missing):
                context = self.context('dm4340')
                for name in ('motor_side_timeout_verified', 'board_version_verified',
                             'can_bitrate_verified', 'joint_mapping_verified'):
                    context.launch_configurations[name] = 'true'
                context.launch_configurations[missing] = 'false'
                with patch.object(self.module, 'Node') as node:
                    with self.assertRaisesRegex(ValueError, missing):
                        self.module.start_selected_backend(context)
                    node.assert_not_called()

    def test_invalid_velocity_feedforward_fails_before_node_creation(self):
        for value in ('-0.1', '0.6', 'nan', '0.4;bad'):
            with self.subTest(value=value):
                context = self.context('dm4340')
                for name in ('motor_side_timeout_verified', 'board_version_verified',
                             'can_bitrate_verified', 'joint_mapping_verified'):
                    context.launch_configurations[name] = 'true'
                context.launch_configurations['velocity_feedforward_nm'] = value
                with patch.object(self.module, 'Node') as node:
                    with self.assertRaisesRegex(ValueError, 'velocity_feedforward_nm'):
                        self.module.start_selected_backend(context)
                    node.assert_not_called()

    def test_invalid_velocity_kd_fails_before_node_creation(self):
        for value in ('0', '-1', '3.1', 'nan', '2;bad'):
            with self.subTest(value=value):
                context = self.context('dm4340')
                for name in ('motor_side_timeout_verified', 'board_version_verified',
                             'can_bitrate_verified', 'joint_mapping_verified'):
                    context.launch_configurations[name] = 'true'
                context.launch_configurations['velocity_kd'] = value
                with patch.object(self.module, 'Node') as node:
                    with self.assertRaisesRegex(ValueError, 'velocity_kd'):
                        self.module.start_selected_backend(context)
                    node.assert_not_called()

    def test_invalid_selection_fails_before_node_creation(self):
        for backend, path in (('none', '/dev/ttyACM1'),
                              ('both', '/dev/ttyACM1'),
                              ('ak30', ''),
                              ('dm4340', '/dev/ttyACM1;bad')):
            with self.subTest(backend=backend, path=path):
                with patch.object(self.module, 'Node') as node:
                    with self.assertRaises(ValueError):
                        self.module.start_selected_backend(self.context(backend, path))
                    node.assert_not_called()
        context = self.context('dm4340')
        for name in ('motor_side_timeout_verified', 'board_version_verified',
                     'can_bitrate_verified', 'joint_mapping_verified'):
            context.launch_configurations[name] = 'true'
        context.launch_configurations['can_bitrate_verified'] = 'yes'
        with patch.object(self.module, 'Node') as node:
            with self.assertRaises(ValueError):
                self.module.start_selected_backend(context)
            node.assert_not_called()

    def test_failed_broadcaster_does_not_queue_motor_controller(self):
        controller = object()
        for code in (0, 1, -1):
            with self.subTest(returncode=code):
                event = type('ExitEvent', (), {'returncode': code})()
                actions = self.module.after_broadcaster_exit(
                    event, LaunchContext(), controller)
                if code == 0:
                    self.assertEqual(actions, [controller])
                else:
                    self.assertEqual(len(actions), 1)
                    self.assertIsInstance(actions[0], EmitEvent)
        event = type('ExitEvent', (), {'returncode': 0})()
        self.assertEqual(self.module.after_broadcaster_exit(
            event, SimpleNamespace(is_shutdown=True), controller), [])
        self.assertEqual(self.module.after_broadcaster_exit(
            event, LaunchContext(), controller, manager_exited=True), [])


if __name__ == '__main__':
    unittest.main()
