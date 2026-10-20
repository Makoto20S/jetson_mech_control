"""The input/strategy/output contract is explicit and fail-closed."""

import unittest

from mech_assist_control.modes import resolve_mode


class ModeTests(unittest.TestCase):
    def test_supported_modes(self):
        self.assertEqual(resolve_mode('velocity_follow', 'imu', 'preview'),
                         'velocity_preview')
        self.assertEqual(resolve_mode('velocity_follow', 'imu', 'motor'),
                         'velocity_motor')
        self.assertEqual(resolve_mode('velocity_follow',
                                      'motor_feedback_simulation', 'motor'),
                         'periodic_bench')
        self.assertEqual(resolve_mode('resistance', 'imu', 'preview'),
                         'resistance_preview')
        self.assertEqual(resolve_mode('resistance', 'imu', 'motor'),
                         'resistance_motor')

    def test_invalid_combinations_do_not_start(self):
        for values in (
                ('resistance', 'motor_feedback_simulation', 'preview'),
                ('velocity_follow', 'motor_feedback_simulation', 'preview'),
                ('unknown', 'imu', 'preview'),
                ('velocity_follow', 'unknown', 'preview'),
                ('velocity_follow', 'imu', 'unknown')):
            with self.subTest(values=values), self.assertRaises(ValueError):
                resolve_mode(*values)


if __name__ == '__main__':
    unittest.main()
