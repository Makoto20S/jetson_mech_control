import math
import unittest

from mech_assist_control.motor_follow import MotorFollowLogic


class MotorFollowTests(unittest.TestCase):
    def test_preview_follows_only_after_diagnostic_and_arm(self):
        control = MotorFollowLogic()
        control.update_knee(0.0, 0.0, 1.0)
        self.assertFalse(control.arm(1.01))
        control.update_diagnostic(True, 1.0)
        self.assertTrue(control.arm(1.01))
        control.update_knee(0.0, 0.0, 1.11)
        self.assertGreater(control.tick(1.12), 0)
        control.update_diagnostic(False, 1.13)
        self.assertEqual(control.tick(1.14), 0)
        self.assertEqual(control.state, 'fault')
        control.reset()
        self.assertFalse(control.arm(1.15))

    def test_motor_mapping_and_feedback_gate(self):
        control = MotorFollowLogic(motor_mode=True, motor_to_knee_scale=0.5)
        control.update_diagnostic(True, 1.0)
        control.update_knee(math.radians(10), 0.0, 1.0)
        self.assertFalse(control.arm(1.01))
        control.update_motor(math.radians(20), 0.0, 1.0)
        self.assertTrue(control.arm(1.01))
        for index in range(1, 5):
            now = 1.01 + index * 0.02
            control.update_motor(math.radians(20), 0.0, now)
            control.update_diagnostic(True, now)
            control.update_knee(math.radians(10), 0.0, now)
            self.assertLessEqual(abs(control.tick(now + 0.001)), 1.5)
        self.assertGreater(control.target_motor_rad_s, 0)
        control.update_motor(math.radians(50), 0.0, 1.10)
        self.assertEqual(control.tick(1.11), 0.0)
        self.assertEqual(control.state, 'fault')

    def test_missing_motor_and_imu_timeout_revoke_output(self):
        control = MotorFollowLogic(motor_mode=True, motor_to_knee_scale=1.0)
        control.update_diagnostic(True, 1.0)
        control.update_knee(0.0, 0.0, 1.0)
        control.update_motor(0.0, 0.0, 1.0)
        self.assertTrue(control.arm(1.0))
        control.update_knee(0.0, 0.0, 1.05)
        self.assertGreater(control.tick(1.06), 0)
        self.assertEqual(control.tick(1.11), 0)
        self.assertEqual(control.state, 'fault')
        control.reset()
        control.update_diagnostic(True, 2.0)
        control.update_knee(0.0, 0.0, 2.0)
        control.update_motor(0.0, 0.0, 2.0)
        self.assertTrue(control.arm(2.0))
        self.assertEqual(control.tick(2.16), 0)
        self.assertEqual(control.state, 'fault')

    def test_invalid_mapping_and_nonfinite_feedback(self):
        with self.assertRaises(ValueError):
            MotorFollowLogic(motor_mode=True)
        control = MotorFollowLogic(motor_mode=True, motor_to_knee_scale=1.0)
        control.update_motor(0.0, 0.0, 1.0)
        control.update_diagnostic(True, 1.0)
        control.update_knee(0.0, 0.0, 1.0)
        self.assertTrue(control.arm(1.0))
        control.update_motor(math.nan, 0.0, 1.01)
        self.assertEqual(control.state, 'fault')

    def test_motor_mode_stops_at_run_limit(self):
        control = MotorFollowLogic(motor_mode=True, motor_to_knee_scale=1.0,
                                   motor_run_limit_s=0.2)
        control.update_diagnostic(True, 1.0)
        control.update_knee(0.0, 0.0, 1.0)
        control.update_motor(0.0, 0.0, 1.0)
        self.assertTrue(control.arm(1.0))
        self.assertEqual(control.tick(1.201), 0.0)
        self.assertEqual(control.state, 'fault')

    def test_larger_trial_limits_are_simulation_only(self):
        with self.assertRaises(ValueError):
            MotorFollowLogic(motor_mode=True, motor_to_knee_scale=1.0,
                             amplitude_deg=40.0, motor_run_limit_s=13.0)
        control = MotorFollowLogic(motor_mode=True, motor_to_knee_scale=1.0,
                                   amplitude_deg=40.0, motor_run_limit_s=13.0,
                                   bench_simulation=True)
        self.assertEqual(control.amplitude_deg, 40.0)
        self.assertEqual(control.motor_run_limit_s, 13.0)


if __name__ == '__main__':
    unittest.main()
