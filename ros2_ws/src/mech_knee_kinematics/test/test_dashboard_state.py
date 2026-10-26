"""The dashboard must not show stale or invalid sensor observations."""

import unittest

from mech_knee_kinematics.dashboard_state import DashboardState


class DashboardStateTest(unittest.TestCase):
    def test_valid_and_zero_angles(self):
        state = DashboardState()
        state.set_packet(100, 10.0)
        state.set_imu(0, 1, True, 10.0)
        state.set_euler(0, 1, (0.0, 0.0, 0.0))
        self.assertEqual(state.imus[0].angles(10.1), (0.0, 0.0, 0.0))

    def test_euler_before_imu_and_old_stamp(self):
        state = DashboardState()
        state.set_euler(0, 2, (1.0, 2.0, 3.0))
        state.set_imu(0, 2, True, 10.0)
        self.assertEqual(state.imus[0].angles(10.1), (1.0, 2.0, 3.0))
        state.set_euler(0, 1, (8.0, 8.0, 8.0))
        self.assertEqual(state.imus[0].angles(10.1), (1.0, 2.0, 3.0))

    def test_invalid_imu_clears_only_its_own_angles(self):
        state = DashboardState()
        for index in range(2):
            state.set_imu(index, 1, True, 10.0)
            state.set_euler(index, 1, (1.0, 2.0, 3.0))
        state.set_imu(0, 2, False, 10.1)
        self.assertIsNone(state.imus[0].angles(10.2))
        self.assertEqual(state.imus[1].angles(10.2), (1.0, 2.0, 3.0))

    def test_timeout_hides_pressure_and_knee(self):
        state = DashboardState()
        state.set_packet(100, 10.0)
        state.set_pressure(0, [0] * 20, 10.0)
        for index in range(2):
            state.set_imu(index, 1, True, 10.0)
            state.set_euler(index, 1, (0.0, 0.0, 0.0))
        state.knee_status = "calibrated"
        state.knee_angle = 0.0
        state.knee_at = 10.0
        self.assertEqual(state.live_knee_angle(10.1), 0.0)
        self.assertEqual(state.pressure_values(0, 10.1), [0] * 20)
        self.assertIsNone(state.live_knee_angle(10.6))
        self.assertIsNone(state.pressure_values(0, 10.6))

    def test_pressure_length_rejected(self):
        state = DashboardState()
        state.set_pressure(0, [12] * 19, 10.0)
        self.assertEqual(state.pressure_at[0], 0.0)

    def test_missing_diagnostic_does_not_look_healthy(self):
        state = DashboardState()
        state.set_packet(100, 10.0)
        self.assertIsNone(state.bridge_status(10.1))
        state.bridge_state = "healthy"
        state.bridge_diagnostic_at = 10.0
        self.assertEqual(state.bridge_status(10.1), "healthy")
        self.assertIsNone(state.bridge_status(11.1))


if __name__ == "__main__":
    unittest.main()
