"""Evaluate the JTC launch parameters/xacro without executing any node."""

import importlib.util
from pathlib import Path
import unittest
from unittest.mock import patch
import xml.etree.ElementTree as ET

from launch import LaunchContext
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from launch_ros.utilities import evaluate_parameters, normalize_parameters
import yaml


class TrajectoryLaunchTest(unittest.TestCase):
    def test_description_uses_position_hardware_and_jtc_starts_inactive(self):
        source = Path(__file__).resolve().parents[1]
        spec = importlib.util.spec_from_file_location(
            'trajectory_launch',
            source / 'launch/motor1_trajectory_bringup.launch.py')
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        calls = []

        def record_node(**kwargs):
            node = Node(**kwargs)
            calls.append((node, kwargs))
            return node

        with patch.object(module, 'Node', side_effect=record_node):
            description = module.generate_launch_description()
        manager = next(args for _, args in calls
                       if args['executable'] == 'ros2_control_node')
        parameters = evaluate_parameters(
            LaunchContext(), normalize_parameters(manager['parameters']))
        urdf = parameters[0]['robot_description']
        self.assertIsInstance(urdf, str)
        control = ET.fromstring(urdf).find('ros2_control')
        sub_mode = control.find('hardware/param[@name="sub_mode"]')
        self.assertTrue(sub_mode is None or sub_mode.text == 'position')
        for name, value in {
                'kp': '1.0',
                'kd': '1.0',
                'position_min_rad': '-12.0',
                'position_max_rad': '6.0',
                'position_max_error_rad': '0.5',
        }.items():
            self.assertEqual(
                control.find(f'hardware/param[@name="{name}"]').text, value)
        self.assertEqual(
            [item.attrib['name']
             for item in control.findall('joint/command_interface')],
            ['position', 'command_generation'])

        config_path = Path(parameters[1])
        self.assertEqual(config_path.name, 'motor1_trajectory_controllers.yaml')
        with config_path.open(encoding='utf-8') as stream:
            config = yaml.safe_load(stream)
        manager_params = config['controller_manager']['ros__parameters']
        self.assertEqual(manager_params['update_rate'], 500)
        self.assertEqual(
            manager_params['motor1_trajectory_controller']['type'],
            'joint_trajectory_controller/JointTrajectoryController')
        controller = config['motor1_trajectory_controller']['ros__parameters']
        self.assertEqual(controller['joints'], ['motor1_joint'])
        self.assertEqual(controller['command_interfaces'], ['position'])
        self.assertEqual(controller['state_interfaces'], ['position'])
        self.assertFalse(controller['allow_partial_joints_goal'])
        self.assertFalse(
            controller['allow_nonzero_velocity_at_trajectory_end'])

        trajectory, args = next(
            (node, args) for node, args in calls
            if args.get('arguments', [None])[0] ==
            'motor1_trajectory_controller')
        self.assertEqual(
            args['arguments'], ['motor1_trajectory_controller', '--inactive'])
        handler = next(entity for entity in description.entities
                       if isinstance(entity, RegisterEventHandler))
        self.assertIsInstance(handler.event_handler, OnProcessExit)
        conditional = handler.describe_conditional_sub_entities()
        self.assertEqual(len(conditional), 1)
        self.assertIn(trajectory, conditional[0][1])
        # JTC is scheduled only by the broadcaster's exit handler.
        self.assertNotIn(trajectory, description.entities)


if __name__ == '__main__':
    unittest.main()
