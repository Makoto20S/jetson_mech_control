import importlib.util
import os
import pathlib
import tempfile
import unittest

ROOT = pathlib.Path(__file__).resolve().parent


class ControlLaunchTests(unittest.TestCase):
    def test_controller_joint_list_follows_selected_urdf(self):
        spec=importlib.util.spec_from_file_location('servo_launch',ROOT/'servo_pair.launch.py')
        module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
        for names in (['motor105_joint'],['motor104_joint','motor105_joint']):
            description='<robot><ros2_control>'+''.join('<joint name="'+n+'"/>' for n in names)+'</ros2_control></robot>'
            config=module.controller_config(description)
            self.assertEqual(config['servo_trajectory_controller']['ros__parameters']['joints'],names)
            self.assertFalse(config['servo_trajectory_controller']['ros__parameters']['allow_partial_joints_goal'])
        with self.assertRaises(ValueError):module.controller_config('<robot/>')

    def test_readiness_requires_both_successful_spawners(self):
        spec = importlib.util.spec_from_file_location('servo_launch', ROOT / 'servo_pair.launch.py')
        module = importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
        with tempfile.TemporaryDirectory() as directory:
            path=pathlib.Path(directory)/'ready'
            completed=set()
            module.mark_ready(completed,'state',0,str(path))
            self.assertFalse(path.exists())
            with self.assertRaises(RuntimeError):
                module.mark_ready(completed,'trajectory',1,str(path))
            self.assertFalse(path.exists())
            module.mark_ready(completed,'trajectory',0,str(path))
            self.assertTrue(path.exists())

    def test_saved_description_and_inactive_controller(self):
        os.environ.setdefault("ROS_LOG_DIR", "/tmp/servo-tools-launch-tests")
        from launch import LaunchContext
        from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions
        from launch_ros.actions import Node
        spec = importlib.util.spec_from_file_location('servo_launch', ROOT / 'servo_pair.launch.py')
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        with tempfile.TemporaryDirectory() as directory:
            urdf = pathlib.Path(directory) / 'saved.urdf'
            urdf.write_text('<robot name="saved_calibrated_pair"><ros2_control><joint name="motor104_joint"/><joint name="motor105_joint"/></ros2_control></robot>')
            context = LaunchContext()
            context.launch_configurations['urdf'] = str(urdf)
            actions = module.setup(context)
            nodes = [action for action in actions if isinstance(action, Node)]
            self.assertEqual(len(nodes), 3)
            self.assertIn('--inactive', [perform_substitutions(
                context, normalize_to_list_of_substitutions(x)) for x in nodes[2].cmd])
            # Controller manager uses the exact saved description, no alternate URDF.
            self.assertEqual(module.read_description(str(urdf)), urdf.read_text())


if __name__ == '__main__':
    unittest.main()
