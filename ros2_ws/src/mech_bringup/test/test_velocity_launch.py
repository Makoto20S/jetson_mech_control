"""Evaluate the actual launch parameters/xacro without executing any node."""

import importlib.util
from pathlib import Path
import unittest
from unittest.mock import patch
import xml.etree.ElementTree as ET

from launch import LaunchContext
from launch_ros.actions import Node
from launch_ros.utilities import evaluate_parameters, normalize_parameters


class VelocityLaunchTest(unittest.TestCase):
    def test_description_is_a_string_and_velocity_starts_inactive(self):
        source = Path(__file__).resolve().parents[1]
        spec = importlib.util.spec_from_file_location(
            'velocity_launch', source / 'launch/motor1_velocity_bringup.launch.py')
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
        self.assertEqual(control.find('hardware/param[@name="sub_mode"]').text, 'velocity')
        self.assertEqual([item.attrib['name'] for item in control.findall(
            'joint/command_interface')], ['velocity', 'command_generation'])
        self.assertEqual(Path(parameters[1]).name, 'motor1_velocity_controllers.yaml')
        self.assertTrue(Path(parameters[1]).is_file())
        velocity, args = next((node, args) for node, args in calls
                              if args.get('arguments', [None])[0] == 'motor1_velocity_controller')
        self.assertEqual(args['arguments'], ['motor1_velocity_controller', '--inactive'])
        # Velocity is only scheduled by the broadcaster's exit handler.
        self.assertNotIn(velocity, description.entities)


if __name__ == '__main__':
    unittest.main()
