"""Regression checks for the simulated IMU-to-motor velocity loop."""

import math
import unittest

from mech_knee_kinematics.motor_sim import (
    KneeVelocityFollower, VelocityConfig, run_trial,
)
from mech_knee_kinematics.offline_loop import Measurement


class MotorSimulationTests(unittest.TestCase):
    def test_periodic_tracking_uses_bounded_slewed_velocity(self):
        rows = run_trial()
        rmse = math.sqrt(sum((row['target_deg'] - row['true_angle_deg']) ** 2
                             for row in rows[200:]) / len(rows[200:]))
        self.assertLess(rmse, 12.0)
        self.assertTrue(any(abs(row['motor_velocity_target_rad_s']) > 0.1
                            for row in rows))
        self.assertTrue(all(abs(row['motor_velocity_target_rad_s']) <= 1.5
                            for row in rows))
        self.assertTrue(all(abs(b['motor_velocity_target_rad_s'] -
                                a['motor_velocity_target_rad_s']) <= 0.010001
                            for a, b in zip(rows, rows[1:])))
        self.assertTrue(all(row['controller_state'] == 'armed' for row in rows))

    def test_imu_dropout_latches_zero_target_after_recovery(self):
        rows = run_trial(dropout=(4.0, 4.25))
        self.assertTrue(all(row['motor_velocity_target_rad_s'] == 0.0
                            for row in rows if row['time_s'] >= 4.0))
        self.assertTrue(all(row['controller_state'] == 'fault'
                            for row in rows if row['time_s'] >= 4.0))
        self.assertTrue(any(row['imu_valid'] == 1
                            for row in rows if row['time_s'] > 4.25))

    def test_invalid_stale_and_nonmonotonic_samples_fail_closed(self):
        follower = KneeVelocityFollower()
        self.assertFalse(follower.arm(Measurement(0, math.nan, 0, True), 0))
        self.assertTrue(follower.arm(Measurement(0, 0, 0, True), 0))
        self.assertGreater(follower.command(
            0.01, Measurement(0.01, 0, 0, True), 30, 0), 0)
        self.assertEqual(follower.command(
            0.02, Measurement(0.01, 0, 0, True), 30, 0), 0)
        self.assertEqual(follower.state, 'fault')
        self.assertFalse(follower.arm(Measurement(0.03, 0, 0, True), 0.03))
        follower.reset()
        self.assertTrue(follower.arm(Measurement(0.04, 0, 0, True), 0.04))
        self.assertEqual(follower.command(
            0.05, Measurement(0.05, 121, 0, True), 30, 0), 0)
        self.assertEqual(follower.state, 'fault')

    def test_timeout_and_rearm_require_fresh_sample(self):
        follower = KneeVelocityFollower()
        self.assertTrue(follower.arm(Measurement(0, 0, 0, True), 0))
        self.assertGreater(follower.command(
            0.1, Measurement(0.1, 0, 0, True), 30, 0), 0)
        self.assertEqual(follower.tick(0.41), 0)
        self.assertEqual(follower.state, 'fault')
        follower.reset()
        self.assertFalse(follower.arm(Measurement(0.1, 0, 0, True), 0.41))
        self.assertTrue(follower.arm(Measurement(0.42, 0, 0, True), 0.42))

    def test_config_and_reference_rejection(self):
        with self.assertRaises(ValueError):
            VelocityConfig(max_velocity_rad_s=math.nan)
        with self.assertRaises(ValueError):
            run_trial(duration_s=0)
        follower = KneeVelocityFollower()
        self.assertTrue(follower.arm(Measurement(0, 0, 0, True), 0))
        self.assertEqual(follower.command(
            0.01, Measurement(0.01, 0, 0, True), math.inf, 0), 0)
        self.assertEqual(follower.state, 'fault')


if __name__ == '__main__':
    unittest.main()
