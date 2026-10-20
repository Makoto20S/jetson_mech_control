"""Exercise the ROS wrapper's publish path without requiring a ROS installation."""

import math
import sys
import types
import unittest
from unittest.mock import patch


class Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class FakeNode:
    def __init__(self, name):
        self.name = name
        self.parameters = {}

    def declare_parameter(self, name, default):
        self.parameters[name] = default

    def get_parameter(self, name):
        return types.SimpleNamespace(value=self.parameters[name])

    def create_publisher(self, *args):
        return Publisher()

    def create_subscription(self, *args):
        return None

    def create_service(self, *args):
        return None

    def create_timer(self, *args):
        return None

    def get_logger(self):
        return types.SimpleNamespace(info=lambda *args: None)

    def get_clock(self):
        return types.SimpleNamespace(now=lambda: types.SimpleNamespace(
            to_msg=lambda: types.SimpleNamespace()))


class DiagnosticStatus:
    OK = 0
    WARN = 1

    def __init__(self):
        self.level = self.WARN


class JointState:
    def __init__(self):
        self.header = types.SimpleNamespace()
        self.name = []
        self.position = []
        self.velocity = []


class Float64:
    def __init__(self):
        self.data = 0.0


def fake_module(name, **attributes):
    module = types.ModuleType(name)
    vars(module).update(attributes)
    return module


def load_node():
    fake = {
        'rclpy': fake_module('rclpy'),
        'rclpy.node': fake_module('rclpy.node', Node=FakeNode),
        'rclpy.qos': fake_module('rclpy.qos', qos_profile_sensor_data=object()),
        'diagnostic_msgs': fake_module('diagnostic_msgs'),
        'diagnostic_msgs.msg': fake_module(
            'diagnostic_msgs.msg', DiagnosticArray=lambda: types.SimpleNamespace(
                header=types.SimpleNamespace(), status=[]),
            DiagnosticStatus=DiagnosticStatus,
            KeyValue=lambda **kwargs: types.SimpleNamespace(**kwargs)),
        'sensor_msgs': fake_module('sensor_msgs'),
        'sensor_msgs.msg': fake_module('sensor_msgs.msg', Imu=object,
                                       JointState=JointState),
        'std_msgs': fake_module('std_msgs'),
        'std_msgs.msg': fake_module('std_msgs.msg', Float64=Float64),
        'std_srvs': fake_module('std_srvs'),
        'std_srvs.srv': fake_module('std_srvs.srv', Trigger=object),
    }
    with patch.dict(sys.modules, fake):
        sys.modules.pop('mech_knee_kinematics.node', None)
        from mech_knee_kinematics.node import KneeKinematicsNode
        return KneeKinematicsNode


def quaternion_y(degrees):
    half = math.radians(degrees) / 2.0
    return math.cos(half), 0.0, math.sin(half), 0.0


def imu_message(stamp, angle=0.0, rate_dps=0.0, orientation_valid=True):
    # Invert the node's default HiPNUC mapping: standard Y is native X.
    w, x, y, z = quaternion_y(angle)
    header = types.SimpleNamespace(stamp=types.SimpleNamespace(sec=0, nanosec=stamp))
    return types.SimpleNamespace(
        header=header,
        orientation_covariance=[0.0 if orientation_valid else -1.0],
        angular_velocity_covariance=[0.0],
        linear_acceleration_covariance=[0.0],
        orientation=types.SimpleNamespace(w=w, x=y, y=x, z=-z),
        angular_velocity=types.SimpleNamespace(x=math.radians(rate_dps), y=0.0, z=0.0),
        linear_acceleration=types.SimpleNamespace(x=0.0, y=0.0, z=9.80665),
    )


def service_response():
    return types.SimpleNamespace(success=None, message=None)


def clock(value):
    return patch('time.monotonic', return_value=value)


class NodeBehaviorTests(unittest.TestCase):
    @staticmethod
    def ready_node():
        node = load_node()()
        node.estimator.calibrate_zero([(1.0, 0.0, 0.0, 0.0)])
        node.axis_ready = True
        return node

    def test_calibration_publish_and_invalid_pair(self):
        node = load_node()()
        self.assertTrue(node._start_static(None, service_response()).success)
        for i in range(20):
            node._receive(0, imu_message(1000 + i))
            node._receive(1, imu_message(1000 + i))
        node.capture_end = 0.0
        node._tick()
        self.assertIsNotNone(node.estimator.neutral_relative)
        self.assertTrue(node._start_axis(None, service_response()).success)
        for i in range(80):
            rate = 45.0 * math.sin(i * 0.31)
            node._receive(0, imu_message(2000 + i))
            node._receive(1, imu_message(2000 + i, angle=40 * math.sin(i * 0.1),
                                         rate_dps=rate))
        node.capture_end = 0.0
        node._tick()
        self.assertTrue(node.axis_ready, node.last_error)
        # Re-zero after action calibration without discarding the hinge axes.
        self.assertTrue(node._start_static(None, service_response()).success)
        for i in range(20):
            node._receive(0, imu_message(3000 + i))
            node._receive(1, imu_message(3000 + i))
        node.capture_end = 0.0
        node._tick()
        self.assertTrue(node.axis_ready)
        node._receive(0, imu_message(4000))
        node._receive(1, imu_message(4000, angle=30))
        self.assertEqual(len(node.angle_pub.messages), 1)
        self.assertAlmostEqual(abs(node.angle_pub.messages[-1].data), 30, delta=1.0)
        self.assertAlmostEqual(abs(node.joint_pub.messages[-1].position[0]),
                               math.radians(30), delta=0.02)
        node._receive(0, imu_message(4001))
        node._receive(1, imu_message(4001, angle=30, orientation_valid=False))
        self.assertEqual(len(node.angle_pub.messages), 1)
        node._tick()
        self.assertEqual(node.diagnostic_pub.messages[-1].status[0].level,
                         DiagnosticStatus.WARN)

    def test_pairs_only_matching_packet_stamps_and_rejects_reordered_data(self):
        node = self.ready_node()
        with clock(1.0):
            node._receive(0, imu_message(1000))
            node._receive(1, imu_message(1001, angle=30))
            self.assertEqual(len(node.angle_pub.messages), 0)
            node._receive(1, imu_message(1000, angle=30))
            self.assertEqual(len(node.angle_pub.messages), 1)
            node._receive(0, imu_message(999))
            node._receive(1, imu_message(999, angle=30))
            self.assertEqual(len(node.angle_pub.messages), 1)
            self.assertEqual(node.out_of_order_messages, 2)
            node._receive(0, imu_message(1001))
            self.assertEqual(len(node.angle_pub.messages), 2)

    def test_incomplete_pair_expires_and_calibration_aborts(self):
        node = self.ready_node()
        with clock(1.0):
            node._receive(0, imu_message(1000))
        with clock(1.2):
            node._tick()
            node._receive(1, imu_message(1000, angle=30))
            node._receive(0, imu_message(1000))
        self.assertEqual(len(node.angle_pub.messages), 0)
        self.assertEqual(node.dropped_pairs, 1)
        self.assertEqual(node.out_of_order_messages, 2)
        with clock(1.4):
            node._tick()
        self.assertEqual(node.diagnostic_pub.messages[-1].status[0].level,
                         DiagnosticStatus.WARN)
        self.assertTrue(node._start_static(None, service_response()).success)
        with clock(2.0):
            node._receive(0, imu_message(2000))
        with clock(2.2):
            node._tick()
        self.assertIsNone(node.capture_mode)
        self.assertEqual(len(node.static_pairs), 0)

    def test_reset_clears_pending_and_allows_new_timestamp_epoch(self):
        node = self.ready_node()
        with clock(1.0):
            node._receive(0, imu_message(3000))
            node._receive(1, imu_message(3000, angle=-10))
            node._receive(0, imu_message(3001))
        self.assertEqual(len(node.angle_pub.messages), 1)
        node._reset(None, service_response())
        self.assertFalse(node.pending)
        self.assertEqual(node.last_closed_stamp, 0)
        self.assertIsNone(node.last_pair_time)
        node.estimator.calibrate_zero([(1.0, 0.0, 0.0, 0.0)])
        node.axis_ready = True
        with clock(1.1):
            node._receive(0, imu_message(1000))
            node._receive(1, imu_message(1000, angle=-20))
        self.assertEqual(len(node.angle_pub.messages), 2)

    def test_timeout_resets_filter_and_recovers_on_new_pair(self):
        node = self.ready_node()
        with clock(1.0):
            node._receive(0, imu_message(1000))
            node._receive(1, imu_message(1000, angle=-30))
        self.assertEqual(len(node.angle_pub.messages), 1)
        with clock(1.6):
            node._tick()
        self.assertIsNone(node.estimator._last_time)
        self.assertEqual(node.diagnostic_pub.messages[-1].status[0].level,
                         DiagnosticStatus.WARN)
        with clock(1.7):
            node._receive(0, imu_message(1001))
            node._receive(1, imu_message(1001, angle=-20))
            node._tick()
        self.assertEqual(len(node.angle_pub.messages), 2)
        self.assertAlmostEqual(abs(node.angle_pub.messages[-1].data), 20, delta=1.0)
        self.assertEqual(node.diagnostic_pub.messages[-1].status[0].level,
                         DiagnosticStatus.OK)

    def test_invalid_imu_pair_resets_filter_before_recovery(self):
        node = self.ready_node()
        with clock(1.0):
            node._receive(0, imu_message(1000))
            node._receive(1, imu_message(1000, angle=-30))
        self.assertEqual(len(node.angle_pub.messages), 1)
        with clock(1.1):
            node._receive(0, imu_message(1001))
            node._receive(1, imu_message(1001, orientation_valid=False))
        self.assertEqual(len(node.angle_pub.messages), 1)
        self.assertIsNone(node.estimator._last_time)
        with clock(1.2):
            node._receive(0, imu_message(1002))
            node._receive(1, imu_message(1002, angle=-20))
            node._tick()
        self.assertEqual(len(node.angle_pub.messages), 2)
        self.assertAlmostEqual(node.angle_pub.messages[-1].data, 20, delta=1.0)
        self.assertEqual(node.diagnostic_pub.messages[-1].status[0].level,
                         DiagnosticStatus.OK)

    def test_second_timeout_after_recovery_aborts_new_capture(self):
        node = self.ready_node()
        with clock(1.0):
            node._receive(0, imu_message(1000))
            node._receive(1, imu_message(1000, angle=-10))
        with clock(1.6):
            node._tick()
        self.assertTrue(node.pair_timed_out)
        self.assertTrue(node._start_static(None, service_response()).success)
        with clock(1.7):
            node._receive(0, imu_message(1001))
            node._receive(1, imu_message(1001))
        self.assertFalse(node.pair_timed_out)
        self.assertEqual(node.capture_mode, 'static')
        with clock(2.3):
            node._tick()
        self.assertTrue(node.pair_timed_out)
        self.assertIsNone(node.capture_mode)
        self.assertEqual(len(node.static_pairs), 0)


if __name__ == '__main__':
    unittest.main()
