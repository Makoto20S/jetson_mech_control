"""Pure-stdlib path, scheduler and owned-child cleanup checks; no MuJoCo load."""
from pathlib import Path
import subprocess
import tempfile
import unittest
from unittest.mock import Mock

from benchmark_mujoco_orin import (
    model_inputs, next_periodic_slot, stop_owned_process, validate_paths,
)


class BenchmarkTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.repo = Path(self.temporary.name) / 'repository'
        self.release = self.repo / 'mujoco/ref01'
        self.release.mkdir(parents=True)
        self.output = self.repo / '.local/validation/new_run'

    def tearDown(self):
        self.temporary.cleanup()

    def test_repository_model_and_fresh_local_output_are_accepted(self):
        actual = validate_paths(self.repo, self.release, self.output)
        self.assertEqual(actual, (self.release.resolve(), self.output.resolve()))
        self.assertFalse(self.output.exists())

    def test_other_model_and_outside_or_root_output_are_rejected(self):
        another = self.repo / 'another_model'
        another.mkdir()
        for release, output in (
            (another, self.output),
            (self.release, self.repo / 'output'),
            (self.release, self.repo / '.local/validation'),
            (self.release, self.repo / '.local/validation/../outside'),
        ):
            with self.subTest(release=release, output=output), self.assertRaises(ValueError):
                validate_paths(self.repo, release, output)

    def test_existing_evidence_directory_is_preserved(self):
        self.output.mkdir(parents=True)
        evidence = self.output / 'result.json'
        evidence.write_text('preserved', encoding='utf-8')
        with self.assertRaises(FileExistsError):
            validate_paths(self.repo, self.release, self.output)
        self.assertEqual(evidence.read_text(encoding='utf-8'), 'preserved')

    def test_hash_inputs_cover_runtime_sources_config_and_mesh_without_manifest(self):
        names = ['ref01_dynamics_api.py', 'ref01_kinematics.py', 'tools/ref01_v1.py',
                 'config/mujoco/ref01_dynamics_cad_v1.xml',
                 'config/mujoco/parameters.json', 'config/mujoco/meshes/link.stl']
        for name in names:
            path = self.release / name
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text('fixture', encoding='utf-8')
        selected = {p.relative_to(self.release).as_posix() for p in model_inputs(self.release)}
        self.assertEqual(selected, set(names))
        self.assertFalse((self.release / 'manifest.json').exists())

    def test_incomplete_runtime_source_set_is_rejected(self):
        with self.assertRaises(FileNotFoundError):
            model_inputs(self.release)

    def test_normal_release_and_exact_boundary_do_not_skip(self):
        self.assertEqual(next_periodic_slot(0, 101, 100, 10), 1)
        self.assertEqual(next_periodic_slot(0, 110, 100, 10), 1)
        self.assertEqual(next_periodic_slot(1, 111, 100, 10), 2)

    def test_overdue_releases_are_skipped_without_catchup_burst(self):
        self.assertEqual(next_periodic_slot(0, 111, 100, 10), 2)
        self.assertEqual(next_periodic_slot(0, 135, 100, 10), 4)
        self.assertEqual(next_periodic_slot(4, 141, 100, 10), 5)
        with self.assertRaises(ValueError):
            next_periodic_slot(0, 100, 100, 0)

    def test_completed_child_is_waited_without_signal(self):
        child = Mock()
        child.poll.return_value = 0
        stop_owned_process(child)
        child.terminate.assert_not_called()
        child.kill.assert_not_called()
        child.wait.assert_called_once_with(timeout=5)
        stop_owned_process(None)

    def test_child_exit_between_poll_and_signal_is_tolerated(self):
        child = Mock()
        child.poll.return_value = None
        child.terminate.side_effect = ProcessLookupError
        stop_owned_process(child)
        child.wait.assert_called_once_with(timeout=5)
        child.kill.assert_not_called()

    def test_unresponsive_owned_child_is_killed_and_waited(self):
        child = Mock()
        child.poll.return_value = None
        child.wait.side_effect = [subprocess.TimeoutExpired('owned worker', 5), -9]
        stop_owned_process(child)
        child.terminate.assert_called_once()
        child.kill.assert_called_once()
        self.assertEqual(child.wait.call_count, 2)


if __name__ == '__main__':
    unittest.main()
