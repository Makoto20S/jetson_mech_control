"""Exercise the complete sensor-estimator-supervisor-plant simulation."""

import contextlib
import csv
import importlib.util
import io
import math
from pathlib import Path
import tempfile
import unittest

from mech_knee_kinematics.offline_loop import (
    LoopConfig, Measurement, PressureMeasurement, VirtualAssistSupervisor,
    VirtualFootGatedAssist, calibrated_estimator, main, run_trial, tracking_rmse,
)


class OfflineLoopTests(unittest.TestCase):
    def test_calibration_and_virtual_assistance_reduce_fixture_error(self):
        estimator = calibrated_estimator()
        self.assertGreater(abs(estimator.hinge_axis[1]), 0.99)
        baseline = run_trial(False)
        assisted = run_trial(True)
        self.assertLess(tracking_rmse(assisted), tracking_rmse(baseline))
        self.assertTrue(any(abs(row['virtual_effort_nm']) > 0.01 for row in assisted))
        self.assertTrue(all(abs(row['virtual_effort_nm']) <= 0.35 + 1e-9
                            for row in assisted))
        self.assertTrue(all(row['supervisor_state'] == 'armed' for row in assisted))
        estimate_rmse = math.sqrt(sum(
            (row['estimated_angle_deg'] - row['true_angle_deg']) ** 2
            for row in assisted) / len(assisted))
        self.assertLess(estimate_rmse, 2.0)

    def test_dropout_latches_zero_output_until_manual_reset(self):
        rows = run_trial(True, dropout=(4.0, 4.25))
        self.assertTrue(all(row['virtual_effort_nm'] == 0.0
                            for row in rows if row['time_s'] >= 4.0))
        self.assertTrue(all(row['supervisor_state'] == 'fault'
                            for row in rows if row['time_s'] >= 4.06))
        self.assertTrue(any(row['imu_valid'] == 1
                            for row in rows if row['time_s'] > 4.25))

    def test_slew_invalid_sample_and_explicit_rearm(self):
        supervisor = VirtualAssistSupervisor()
        first = Measurement(0.0, 0.0, 0.0, True)
        self.assertTrue(supervisor.arm(first, 0.0))
        effort = supervisor.command(0.01, Measurement(0.01, 0.0, 0.0, True), 60.0)
        self.assertAlmostEqual(effort, 0.02)
        self.assertEqual(supervisor.command(0.02, None, 60.0), 0.0)
        self.assertEqual(supervisor.command(0.07, None, 60.0), 0.0)
        self.assertEqual(supervisor.state, 'fault')
        self.assertFalse(supervisor.arm(Measurement(0.08, 0.0, 0.0, True), 0.08))
        supervisor.reset()
        self.assertTrue(supervisor.arm(Measurement(0.09, 0.0, 0.0, True), 0.09))
        self.assertEqual(supervisor.command(
            0.10, Measurement(0.10, 130.0, 0.0, True), 60.0), 0.0)
        self.assertEqual(supervisor.state, 'fault')

    def test_virtual_pressure_gate_contact_loss_and_dropout(self):
        baseline = run_trial(False)
        assisted = run_trial(True, pressure_gated=True)
        self.assertLess(tracking_rmse(assisted), tracking_rmse(baseline))
        self.assertTrue(any(row['virtual_effort_nm'] > 0.01 for row in assisted))
        self.assertTrue(all(abs(row['virtual_effort_nm']) <= 0.35 + 1e-9
                            for row in assisted))

        swing = run_trial(True, pressure_gated=True, contact_off=(2.0, 2.5))
        self.assertTrue(all(row['virtual_effort_nm'] == 0.0
                            for row in swing if 2.0 <= row['time_s'] < 2.5))
        self.assertTrue(any(abs(row['virtual_effort_nm']) > 0.01
                            for row in swing if 2.6 <= row['time_s'] < 3.0))
        self.assertTrue(all(row['supervisor_state'] == 'armed' for row in swing))

        missing = run_trial(True, pressure_gated=True,
                            pressure_dropout=(4.0, 4.25))
        self.assertTrue(all(row['virtual_effort_nm'] == 0.0
                            for row in missing if row['time_s'] >= 4.0))
        self.assertTrue(all(row['supervisor_state'] == 'fault'
                            for row in missing if row['time_s'] >= 4.0))

    def test_virtual_pressure_gate_rejects_invalid_and_requires_reset(self):
        gate = VirtualFootGatedAssist()
        knee = Measurement(0.0, 0.0, 0.0, True)
        contact = PressureMeasurement(0.0, 100.0, 0.0, True)
        self.assertFalse(gate.arm(knee, PressureMeasurement(0.0, 0.0, 0.0, True), 0.0))
        self.assertTrue(gate.arm(knee, contact, 0.0))
        invalid = PressureMeasurement(0.01, math.nan, 0.0, True)
        self.assertEqual(gate.command(0.01, Measurement(0.01, 0.0, 0.0, True),
                                      invalid, 60.0), 0.0)
        self.assertEqual(gate.state, 'fault')
        self.assertFalse(gate.arm(Measurement(0.02, 0.0, 0.0, True),
                                  PressureMeasurement(0.02, 100.0, 0.0, True), 0.02))
        gate.reset()
        self.assertTrue(gate.arm(Measurement(0.03, 0.0, 0.0, True),
                                 PressureMeasurement(0.03, 100.0, 0.0, True), 0.03))

    def test_bad_config_and_non_finite_measurement_fail_closed(self):
        with self.assertRaises(ValueError):
            LoopConfig(max_effort_nm=float('nan'))
        with self.assertRaises(ValueError):
            LoopConfig(sample_timeout_s=0.0)
        with self.assertRaises(ValueError):
            run_trial(False, duration_s=0.0)
        supervisor = VirtualAssistSupervisor()
        self.assertFalse(supervisor.arm(Measurement(0.0, math.nan, 0.0, True), 0.0))
        self.assertEqual(supervisor.state, 'disarmed')

    def test_virtual_phase_envelope_zeroes_effort_and_rejects_bad_scale(self):
        supervisor = VirtualAssistSupervisor(LoopConfig(sample_timeout_s=0.3))
        self.assertTrue(supervisor.arm(Measurement(0.0, 5.0, 0.0, True), 0.0))
        self.assertGreater(supervisor.command(
            0.1, Measurement(0.1, 5.0, 0.0, True), 30.0), 0.0)
        self.assertEqual(supervisor.command(
            0.2, Measurement(0.2, 5.0, 0.0, True), 30.0, scale=0.0), 0.0)
        self.assertEqual(supervisor.last_effort_nm, 0.0)
        self.assertEqual(supervisor.command(
            0.3, Measurement(0.3, 5.0, 0.0, True), 30.0,
            scale=math.nan), 0.0)
        self.assertEqual(supervisor.state, 'fault')

    def test_cli_writes_all_three_trials(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'virtual_loop.csv'
            with contextlib.redirect_stdout(io.StringIO()):
                main(['--csv', str(path)])
            with path.open(newline='', encoding='utf-8') as source:
                rows = list(csv.DictReader(source))
        self.assertEqual(len(rows), 2400)
        self.assertEqual({row['trial'] for row in rows},
                         {'baseline', 'assisted', 'imu_dropout'})

    @unittest.skipUnless(importlib.util.find_spec('PIL'), 'Pillow not installed')
    def test_png_export_is_nonblank(self):
        from PIL import Image

        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'virtual_loop.png'
            with contextlib.redirect_stdout(io.StringIO()):
                main(['--png', str(path)])
            with Image.open(path) as image:
                self.assertEqual(image.size, (1100, 680))
                self.assertGreater(len(image.getcolors(maxcolors=1000000)), 3)


if __name__ == '__main__':
    unittest.main()
