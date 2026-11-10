"""State assembly contract tests; no ROS graph, device, or actuator is started."""
import copy
import math
import unittest

from sensor_msgs.msg import Imu, JointState
from state_core import StateCore, ns, set_stamp
from ref01_kinematics import inverse_kinematics


NOW = 10_000_000_000
MONO = 50_000_000_000
MS = 1_000_000


def joints(stamp=NOW, names=None, positions=None, velocities=None):
    msg = JointState()
    set_stamp(msg.header.stamp, stamp)
    msg.name = ['motor104_joint', 'motor105_joint'] if names is None else names
    msg.position = [math.pi / 3, math.pi / 2] if positions is None else positions
    msg.velocity = [] if velocities is None else velocities
    return msg


def imu(stamp=NOW, frame='imu1_link'):
    msg = Imu()
    set_stamp(msg.header.stamp, stamp)
    msg.header.frame_id = frame
    msg.orientation.w = 1.0
    msg.angular_velocity.x = 0.3
    msg.linear_acceleration.z = 9.81
    # Zero covariance means unknown covariance, not unavailable measurement.
    return msg


def populate(core, stamp=NOW, now=NOW, mono=MONO):
    core.joints(joints(stamp), now, mono)
    core.imu(1, imu(stamp), now, mono)
    core.imu(2, imu(stamp, 'imu2_link'), now, mono)


class StateCoreTests(unittest.TestCase):
    def test_mit_cache_never_refreshes_an_axis_and_ignores_placeholders(self):
        from types import SimpleNamespace
        from mit_feedback import FIELDS
        core = StateCore(motor_source='mit')
        def feed(index, mid, seq, rx, now, code=0):
            row = SimpleNamespace(interface_names=list(FIELDS),
                    values=[float(x) for x in (mid, seq, rx//1000, 1, 72, -20, -.04, 42, code)])
            msg = SimpleNamespace(joint_names=[core.mit.names[index]], interface_values=[row])
            core.mit_feedback(msg, NOW+now-MONO, now)
        core.joints(joints(), NOW, MONO)
        self.assertFalse(core.snapshot(NOW,MONO).sources[0].valid)
        feed(0,104,1,MONO,MONO)
        feed(1,105,1,MONO+MS,MONO+2*MS)
        state = core.snapshot(NOW+2*MS,MONO+2*MS)
        self.assertTrue(state.sources[0].valid)
        feed(0,104,1,MONO,MONO+61*MS)
        feed(1,105,2,MONO+60*MS,MONO+61*MS)
        axes = core.snapshot(NOW+61*MS,MONO+61*MS).mit_motors
        self.assertEqual((axes[0].valid, axes[0].reason, axes[0].rx_sequence), (False,'feedback_timeout',1))
        self.assertTrue(axes[1].valid)
        feed(0,105,3,MONO+62*MS,MONO+62*MS)
        self.assertFalse(core.mit.axes[0].valid)
        feed(1,105,3,MONO+62*MS,MONO+62*MS,0x77)
        self.assertFalse(core.mit.axes[1].valid)
        self.assertEqual(core.mit.axes[1].fault_code, 0x77)
        feed(1,105,2,MONO+60*MS,MONO+63*MS)
        self.assertEqual(core.mit.axes[1].reason, 'out_of_order')
        self.assertEqual(core.mit.axes[1].fault_code, 0x77)

    def test_initial_state_has_no_valid_or_fabricated_measurement(self):
        out = StateCore().snapshot(NOW, MONO)
        self.assertFalse(any(s.valid for s in out.sources))
        self.assertFalse(out.messages_aligned)
        self.assertFalse(out.alpha_valid)
        self.assertTrue(all(math.isnan(x) for x in out.alpha_position))
        self.assertTrue(all(math.isnan(x) for x in out.alpha_velocity))

    def test_history_mapping_is_candidate_until_confirmed(self):
        core = StateCore()
        populate(core)
        out = core.snapshot(NOW, MONO)
        self.assertAlmostEqual(out.alpha_position[0], math.pi / 4)
        self.assertAlmostEqual(out.alpha_position[1], math.pi / 12)
        self.assertFalse(out.mapping_confirmed)
        self.assertFalse(out.alpha_valid)
        self.assertTrue(out.messages_aligned)
        self.assertFalse(out.hardware_freshness_verified)
        self.assertFalse(any(s.sample_time_verified for s in out.sources))

    def test_confirmed_mapping_and_velocity_are_radians_without_gear_division(self):
        core = StateCore(mapping_confirmed=True)
        core.joints(joints(velocities=[2.0, -3.0]), NOW, MONO)
        out = core.snapshot(NOW, MONO)
        self.assertTrue(out.alpha_valid)
        self.assertTrue(out.alpha_velocity_valid)
        self.assertAlmostEqual(out.alpha_position[0], math.pi / 4)
        self.assertEqual(list(out.alpha_velocity), [-3.0, 2.0])

    def test_joint_order_is_resolved_by_name_and_unrelated_joint_is_ignored(self):
        core = StateCore(mapping_confirmed=True)
        msg = joints(names=['extra', 'motor105_joint', 'motor104_joint'],
                     positions=[99.0, math.pi / 2, math.pi / 3],
                     velocities=[99.0, -3.0, 2.0])
        core.joints(msg, NOW, MONO)
        out = core.snapshot(NOW, MONO)
        self.assertEqual(list(out.joints.name), ['motor104_joint', 'motor105_joint'])
        self.assertEqual(list(out.joints.velocity), [2.0, -3.0])
        self.assertAlmostEqual(out.alpha_position[0], math.pi / 4)

    def test_missing_velocity_is_not_zero_or_estimated(self):
        core = StateCore(mapping_confirmed=True)
        core.joints(joints(), NOW, MONO)
        out = core.snapshot(NOW, MONO)
        self.assertTrue(out.alpha_valid)
        self.assertEqual(list(out.joints.velocity), [])
        self.assertFalse(out.alpha_velocity_valid)
        self.assertTrue(all(math.isnan(x) for x in out.alpha_velocity))

    def test_configured_sign_and_offset_apply_in_alpha_order(self):
        core = StateCore(scales=(-2.0, 3.0), offsets=(0.1, -0.2), mapping_confirmed=True)
        core.joints(joints(positions=[0.5, 0.8], velocities=[2.0, 4.0]), NOW, MONO)
        out = core.snapshot(NOW, MONO)
        self.assertAlmostEqual(out.alpha_position[0], -1.5)
        self.assertAlmostEqual(out.alpha_position[1], 1.3)
        self.assertEqual(list(out.alpha_velocity), [-8.0, 6.0])

    def test_bad_joint_frame_invalidates_previous_valid_state_immediately(self):
        bad_frames = [
            joints(names=['motor104_joint'], positions=[1.0]),
            joints(names=['motor104_joint', 'motor104_joint']),
            joints(positions=[1.0]),
            joints(velocities=[1.0]),
            joints(positions=[math.nan, 1.0]),
            joints(velocities=[math.inf, 1.0]),
        ]
        malformed_effort = joints()
        malformed_effort.effort = [1.0]
        bad_frames.append(malformed_effort)
        for bad in bad_frames:
            with self.subTest(message=str(bad)):
                core = StateCore(mapping_confirmed=True)
                populate(core)
                set_stamp(bad.header.stamp, NOW + MS)
                core.joints(bad, NOW + MS, MONO + MS)
                out = core.snapshot(NOW + MS, MONO + MS)
                self.assertFalse(out.sources[0].valid)
                self.assertFalse(out.alpha_valid)
                self.assertFalse(out.messages_aligned)

    def test_duplicate_and_out_of_order_stamps_do_not_refresh_age(self):
        for stamp in (NOW, NOW - MS):
            with self.subTest(stamp=stamp):
                core = StateCore()
                populate(core)
                core.joints(joints(stamp), NOW + 10 * MS, MONO + 10 * MS)
                out = core.snapshot(NOW + 10 * MS, MONO + 10 * MS)
                self.assertFalse(out.sources[0].valid)
                self.assertEqual(out.sources[0].accepted_samples, 1)
                self.assertAlmostEqual(out.sources[0].receive_age_ms, 10.0)
                self.assertEqual(ns(out.sources[0].source_stamp), NOW)

    def test_zero_old_and_future_stamps_are_rejected_without_poisoning_next_sample(self):
        for stamp in (0, NOW - 101 * MS, NOW + 6 * MS):
            with self.subTest(stamp=stamp):
                core = StateCore()
                core.joints(joints(stamp), NOW, MONO)
                self.assertFalse(core.snapshot(NOW, MONO).sources[0].valid)
                core.joints(joints(NOW + MS), NOW + MS, MONO + MS)
                self.assertTrue(core.snapshot(NOW + MS, MONO + MS).sources[0].valid)

    def test_invalid_nanosecond_component_is_rejected(self):
        core = StateCore()
        msg = joints()
        msg.header.stamp.nanosec = 1_000_000_000
        core.joints(msg, NOW + 2_000_000_000, MONO)
        self.assertFalse(core.snapshot(NOW + 2_000_000_000, MONO).sources[0].valid)

    def test_source_and_receive_timestamps_are_distinct(self):
        core = StateCore()
        core.joints(joints(NOW - 20 * MS), NOW, MONO)
        out = core.snapshot(NOW + 10 * MS, MONO + 10 * MS)
        self.assertEqual(ns(out.sources[0].source_stamp), NOW - 20 * MS)
        self.assertEqual(ns(out.sources[0].received_stamp), NOW)
        self.assertAlmostEqual(out.sources[0].source_age_ms, 30.0)
        self.assertAlmostEqual(out.sources[0].receive_age_ms, 10.0)

    def test_paused_ros_clock_cannot_extend_monotonic_ttl(self):
        core = StateCore()
        populate(core)
        out = core.snapshot(NOW, MONO + 101 * MS)
        self.assertFalse(any(s.valid for s in out.sources))
        self.assertFalse(out.messages_aligned)

    def test_forward_ros_clock_jump_expires_source_even_with_short_receive_age(self):
        core = StateCore()
        populate(core)
        out = core.snapshot(NOW + 101 * MS, MONO + MS)
        self.assertFalse(any(s.valid for s in out.sources))

    def test_backward_ros_clock_clears_all_sources_and_accepts_new_epoch(self):
        core = StateCore()
        populate(core)
        new_time = NOW - 1_000_000_000
        out = core.snapshot(new_time, MONO + MS)
        self.assertFalse(any(s.valid for s in out.sources))
        self.assertFalse(out.messages_aligned)
        core.joints(joints(new_time), new_time, MONO + 2 * MS)
        out = core.snapshot(new_time, MONO + 2 * MS)
        self.assertTrue(out.sources[0].valid)
        self.assertFalse(out.messages_aligned)
        core.imu(1, imu(new_time), new_time, MONO + 2 * MS)
        core.imu(2, imu(new_time, 'imu2_link'), new_time, MONO + 2 * MS)
        self.assertTrue(core.snapshot(new_time, MONO + 2 * MS).messages_aligned)

    def test_one_fresh_source_cannot_hide_other_sources_expiring(self):
        core = StateCore()
        populate(core)
        core.joints(joints(NOW + 110 * MS), NOW + 110 * MS, MONO + 110 * MS)
        out = core.snapshot(NOW + 110 * MS, MONO + 110 * MS)
        self.assertEqual([s.valid for s in out.sources], [True, False, False])
        self.assertFalse(out.messages_aligned)

    def test_timestamp_skew_is_independent_of_individual_freshness(self):
        core = StateCore(skew_ms=30.0)
        core.joints(joints(NOW - 40 * MS), NOW, MONO)
        core.imu(1, imu(), NOW, MONO)
        core.imu(2, imu(frame='imu2_link'), NOW, MONO)
        out = core.snapshot(NOW, MONO)
        self.assertTrue(all(s.valid for s in out.sources))
        self.assertAlmostEqual(out.source_skew_ms, 40.0)
        self.assertFalse(out.messages_aligned)

    def test_imu_bad_orientation_preserves_independent_valid_velocity_and_acceleration(self):
        core = StateCore()
        msg = imu()
        msg.orientation.w = 0.0
        core.imu(1, msg, NOW, MONO)
        out = core.snapshot(NOW, MONO)
        self.assertTrue(out.sources[1].valid)
        self.assertFalse(out.imu_orientation_valid[0])
        self.assertTrue(out.imu_angular_velocity_valid[0])
        self.assertTrue(out.imu_linear_acceleration_valid[0])
        self.assertEqual(out.imus[0].header.frame_id, 'imu1_link')
        self.assertEqual(out.imus[0].orientation.w, 0.0)
        self.assertEqual(out.imus[0].linear_acceleration.z, 9.81)

    def test_imu_unavailable_and_nonfinite_fields_are_independent(self):
        core = StateCore()
        msg = imu()
        msg.orientation_covariance[0] = -1.0
        msg.angular_velocity.x = math.nan
        core.imu(1, msg, NOW, MONO)
        out = core.snapshot(NOW, MONO)
        self.assertTrue(out.sources[1].valid)
        self.assertFalse(out.imu_orientation_valid[0])
        self.assertFalse(out.imu_angular_velocity_valid[0])
        self.assertTrue(out.imu_linear_acceleration_valid[0])

    def test_imu_missing_frame_or_all_unavailable_invalidates_source(self):
        bad_frames = [imu(frame='')]
        unavailable = imu()
        for field in ('orientation_covariance', 'angular_velocity_covariance', 'linear_acceleration_covariance'):
            getattr(unavailable, field)[0] = -1.0
        bad_frames.append(unavailable)
        for bad in bad_frames:
            core = StateCore()
            populate(core)
            set_stamp(bad.header.stamp, NOW + MS)
            core.imu(1, bad, NOW + MS, MONO + MS)
            out = core.snapshot(NOW + MS, MONO + MS)
            self.assertFalse(out.sources[1].valid)
            self.assertFalse(out.imu_orientation_valid[0])
            self.assertFalse(out.messages_aligned)

    def test_inputs_and_snapshots_are_copied(self):
        core = StateCore()
        msg = imu()
        core.imu(1, msg, NOW, MONO)
        msg.angular_velocity.x = 99.0
        first = core.snapshot(NOW, MONO)
        self.assertAlmostEqual(first.imus[0].angular_velocity.x, 0.3)
        first.imus[0].angular_velocity.x = 88.0
        self.assertAlmostEqual(core.snapshot(NOW, MONO).imus[0].angular_velocity.x, 0.3)

    def test_invalid_configuration_is_rejected(self):
        cases = [
            {'names': ['one']}, {'names': ['same', 'same']}, {'names': ['', 'two']},
            {'names': [104, 105]}, {'mapping_confirmed': 'false'},
            {'scales': [1.0]}, {'scales': [0.0, 1.0]}, {'scales': [math.nan, 1.0]},
            {'offsets': [0.0]}, {'offsets': [0.0, math.inf]},
            {'timeout_ms': 0.0}, {'timeout_ms': math.nan}, {'timeout_ms': -1.0},
            {'skew_ms': -1.0}, {'skew_ms': math.inf}, {'future_ms': -1.0},
        ]
        for config in cases:
            with self.subTest(config=config), self.assertRaises((ValueError, TypeError)):
                StateCore(**config)

    def test_invalid_imu_index_is_rejected(self):
        with self.assertRaises(ValueError):
            StateCore().imu(0, imu(), NOW, MONO)



class ForwardStateTests(unittest.TestCase):
    @staticmethod
    def motor_message(psi=10., beta=20., stamp=NOW):
        a1,a2 = inverse_kinematics(psi,beta,unit='deg')
        # Input deliberately reversed: names, not array position, define order.
        return joints(stamp,names=['motor105_joint','motor104_joint'],
                      positions=[math.radians(a1+45.),math.radians(a2+45.)])

    def test_motor_only_pose_and_imu_independence(self):
        core = StateCore(mapping_confirmed=True)
        core.joints(self.motor_message(),NOW,MONO)
        out = core.snapshot(NOW,MONO)
        self.assertTrue(out.q_valid)
        self.assertEqual(out.q_reason,'valid_from_motor_angles')
        self.assertLessEqual(out.q_closure_residual_m,1e-11)
        self.assertAlmostEqual(out.q_position[0],math.radians(10.))
        self.assertAlmostEqual(out.q_position[1],math.radians(20.))
        self.assertFalse(out.messages_aligned)
        self.assertFalse(any(s.valid for s in out.sources[1:]))
        self.assertFalse(out.alpha_velocity_valid)
        core.imu(1,imu(frame=''),NOW,MONO)
        self.assertEqual(list(core.snapshot(NOW,MONO).q_position),list(out.q_position))
        self.assertFalse(out.hardware_freshness_verified)

    def test_unconfirmed_mapping_does_not_publish_pose(self):
        core = StateCore()
        core.joints(self.motor_message(),NOW,MONO)
        out = core.snapshot(NOW,MONO)
        self.assertFalse(out.q_valid)
        self.assertEqual(out.q_reason,'mapping_unconfirmed')
        self.assertTrue(all(math.isnan(x) for x in out.q_position))

    def test_invalid_fk_clears_pose_and_recovers_on_new_valid_input(self):
        core = StateCore(mapping_confirmed=True)
        core.joints(self.motor_message(),NOW,MONO)
        self.assertTrue(core.snapshot(NOW,MONO).q_valid)
        core.joints(joints(NOW+MS,positions=[0.,0.]),NOW+MS,MONO+MS)
        out = core.snapshot(NOW+MS,MONO+MS)
        self.assertTrue(out.alpha_valid)
        self.assertFalse(out.q_valid)
        self.assertTrue(out.q_reason.startswith('fk_invalid:'))
        self.assertTrue(all(math.isnan(x) for x in out.q_position))
        core.joints(self.motor_message(stamp=NOW+2*MS),NOW+2*MS,MONO+2*MS)
        self.assertTrue(core.snapshot(NOW+2*MS,MONO+2*MS).q_valid)

    def test_joint_bad_duplicate_dropout_and_clock_reset_invalidate_pose(self):
        for mode in ('bad','duplicate','dropout','clock_reset'):
            with self.subTest(mode=mode):
                core = StateCore(mapping_confirmed=True)
                core.joints(self.motor_message(),NOW,MONO)
                self.assertTrue(core.snapshot(NOW,MONO).q_valid)
                now,mono = NOW+MS,MONO+MS
                if mode == 'bad':
                    core.joints(joints(NOW+MS,positions=[math.nan,0.]),now,mono)
                elif mode == 'duplicate':
                    core.joints(self.motor_message(),now,mono)
                elif mode == 'dropout':
                    # IMU can stay fresh while motor feedback expires.
                    now,mono = NOW+101*MS,MONO+101*MS
                    core.imu(1,imu(now),now,mono)
                else:
                    now = NOW-1
                out = core.snapshot(now,mono)
                self.assertFalse(out.q_valid)
                self.assertEqual(out.q_reason,'joint_source_invalid')
                self.assertTrue(all(math.isnan(x) for x in out.q_position))


class MitSpeedTest(unittest.TestCase):
    def test_calibration_and_freshness_are_per_axis(self):
        from types import SimpleNamespace
        from mit_feedback import FIELDS
        for confirmed in ((False, False), (True, False), (True, True)):
            core = StateCore(motor_source='mit', mit_speed_confirmed=confirmed,
                             mit_speed_directions=(1, -1), mapping_confirmed=True)
            rows = [SimpleNamespace(interface_names=list(FIELDS),
                                    values=[104+i, 1, MONO//1000, 1, 70., erpm, .1, 40., 0])
                    for i, erpm in enumerate((1120., 2240.))]
            core.mit_feedback(SimpleNamespace(joint_names=list(core.mit.names), interface_values=rows),
                              NOW, MONO)
            out = core.snapshot(NOW, MONO)
            expected = [math.tau/6, -math.tau/3]
            for i, axis in enumerate(out.mit_motors):
                self.assertAlmostEqual(axis.output_velocity_rad_s, expected[i])
                self.assertEqual(axis.output_velocity_valid, confirmed[i])
            self.assertEqual(out.alpha_velocity_valid, all(confirmed))
            if all(confirmed):
                for v, want in zip(out.joints.velocity, expected):
                    self.assertAlmostEqual(v, want)
                self.assertAlmostEqual(out.alpha_velocity[0], expected[1])
            else:
                self.assertFalse(out.joints.velocity)
            stale = core.snapshot(NOW+61*MS, MONO+61*MS)
            self.assertFalse(any(a.output_velocity_valid for a in stale.mit_motors))
            self.assertFalse(stale.joints.velocity)


if __name__ == '__main__':
    unittest.main()
