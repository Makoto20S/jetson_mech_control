import importlib.util
import os
import pathlib
import tempfile
import unittest

ROOT = pathlib.Path(__file__).resolve().parent


class ControlLaunchTests(unittest.TestCase):
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
            urdf.write_text('<robot name="saved_calibrated_pair"/>')
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
