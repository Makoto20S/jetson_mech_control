"""Pure, non-actuating gait shadow policy checks."""

import math
import unittest

from mech_knee_kinematics.gait_policy import GaitConfig, GaitShadow


def two_heel_strikes(control):
    control.update_diagnostic(True, 0.0)
    control.update_knee(5.0, 0.0, 0.0)
    control.update_pressure(0, 0.0, 0.0)
    control.update_pressure(1, 100.0, 0.1)
    control.update_pressure(2, 100.0, 0.2)
    control.update_pressure(0, 0.0, 0.8)
    control.update_pressure(1, 100.0, 1.1)
    control.update_knee(5.0, 0.0, 1.1)
    control.update_diagnostic(True, 1.1)


class GaitPolicyTests(unittest.TestCase):
    def test_requires_validated_foot_and_two_strikes(self):
        control = GaitShadow()
        two_heel_strikes(control)
        self.assertEqual(control.period_s, 1.0)
        self.assertFalse(control.arm(1.11))
        validated = GaitShadow(pressure_validated=True)
        validated.update_diagnostic(True, 0.0)
        validated.update_knee(5.0, 0.0, 0.0)
        validated.update_pressure(0, 0.0, 0.0)
        validated.update_pressure(1, 100.0, 0.1)
        self.assertFalse(validated.arm(0.11))
        validated = GaitShadow(pressure_validated=True)
        two_heel_strikes(validated)
        self.assertTrue(validated.arm(1.11))

    def test_periodic_virtual_effort_and_swing_zero(self):
        control = GaitShadow(pressure_validated=True)
        two_heel_strikes(control)
        self.assertTrue(control.arm(1.11))
        control.update_pressure(2, 100.0, 1.25)
        control.update_diagnostic(True, 1.25)
        control.update_knee(5.0, 0.0, 1.25)
        self.assertGreater(control.effort_nm, 0.0)
        self.assertLessEqual(abs(control.effort_nm), 0.35)
        self.assertAlmostEqual(control.phase, 0.15)
        control.update_pressure(0, 0.0, 1.3)
        self.assertEqual(control.effort_nm, 0.0)
        self.assertEqual(control.tick(1.31), 0.0)
        self.assertEqual(control.state, 'armed')

    def test_invalid_pressure_and_diagnostic_latch_zero(self):
        control = GaitShadow(pressure_validated=True)
        two_heel_strikes(control)
        self.assertTrue(control.arm(1.11))
        control.update_pressure(2, 0.0, 1.2)
        self.assertEqual(control.state, 'fault')
        self.assertEqual(control.tick(1.21), 0.0)
        self.assertFalse(control.arm(1.21))
        control.reset()
        self.assertFalse(control.arm(1.21))
        two_heel_strikes(control)
        self.assertTrue(control.arm(1.11))
        control.update_diagnostic(False, 1.2)
        self.assertEqual(control.state, 'fault')
        self.assertEqual(control.tick(1.21), 0.0)

    def test_timeout_bad_period_and_nonfinite_knee(self):
        control = GaitShadow(pressure_validated=True)
        two_heel_strikes(control)
        self.assertTrue(control.arm(1.11))
        self.assertEqual(control.tick(1.5), 0.0)
        self.assertEqual(control.state, 'fault')
        control.reset()
        two_heel_strikes(control)
        self.assertTrue(control.arm(1.11))
        control.update_knee(math.nan, 0.0, 1.2)
        self.assertEqual(control.state, 'fault')
        control.reset()
        two_heel_strikes(control)
        self.assertTrue(control.arm(1.11))
        control.update_pressure(0, 0.0, 1.2)
        control.update_pressure(1, 100.0, 1.3)
        self.assertEqual(control.state, 'fault')

    def test_rejects_bad_configuration_and_duplicate_sample(self):
        with self.assertRaises(ValueError):
            GaitConfig(stance_fraction=1.0)
        with self.assertRaises(ValueError):
            GaitConfig(reference_offset_deg=110.0, reference_amplitude_deg=20.0)
        control = GaitShadow(pressure_validated=True)
        self.assertTrue(control.update_pressure(0, 0.0, 1.0))
        self.assertFalse(control.update_pressure(1, 100.0, 1.0))
        self.assertIsNone(control.pressure)


if __name__ == '__main__':
    unittest.main()
