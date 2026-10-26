"""Legacy ROS ingress must preserve source age and latch transient health faults."""
import time
import unittest

try:
    import rclpy
    from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
    from sensor_msgs.msg import JointState
    from mech_assist_control.node import KneeControlNode
except ImportError:
    rclpy = None


@unittest.skipIf(rclpy is None, 'ROS 2 unavailable')
class SourceTimeTests(unittest.TestCase):
    def setUp(self):
        rclpy.init()
        self.node = KneeControlNode(use_global_arguments=False)

    def tearDown(self):
        self.node.destroy_node()
        rclpy.shutdown()

    def joint(self, age, name='knee_joint'):
        msg = JointState()
        stamp = self.node.get_clock().now().nanoseconds - int(age * 1e9)
        msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(stamp, 1_000_000_000)
        msg.name, msg.position, msg.velocity = [name], [0.], [0.]
        return msg

    def diagnostic(self, healthy=True):
        msg = DiagnosticArray()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.status = [DiagnosticStatus(name='knee_kinematics',
            level=DiagnosticStatus.OK if healthy else DiagnosticStatus.WARN)]
        return msg

    def test_delayed_but_valid_knee_preserves_source_age(self):
        self.node._on_knee(self.joint(.09))
        self.assertLessEqual(self.node.control.knee.time_s, time.monotonic() - .089)
        self.assertNotEqual(self.node.control.state, 'fault')

    def test_stale_future_and_delayed_motor_inputs_fault_without_refresh(self):
        for handler, name, ages in ((self.node._on_knee, 'knee_joint', (.31, -.05)),
                                    (self.node._on_motor, 'dm4340_joint', (.11, -.05))):
            for age in ages:
                self.node.control.reset()
                handler(self.joint(age, name))
                self.assertEqual(self.node.control.state, 'fault')
                self.assertIn('source timestamp', self.node.control.reason)

    def test_transient_warn_followed_by_fresh_ok_cannot_rearm(self):
        self.node._on_knee(self.joint(0.))
        self.node._on_diagnostic(self.diagnostic())
        self.assertTrue(self.node.control.arm(time.monotonic()))
        self.node._on_diagnostic(self.diagnostic(False))
        self.node._on_diagnostic(self.diagnostic(True))
        self.node._on_knee(self.joint(0.))
        self.assertEqual(self.node.control.state, 'fault')
        self.assertEqual(self.node.control.tick(time.monotonic()), 0.)
        self.assertFalse(self.node.control.arm(time.monotonic()))

    def test_replayed_and_stale_diagnostics_cannot_refresh_health(self):
        msg = self.diagnostic()
        self.node._on_diagnostic(msg)
        self.node._on_diagnostic(msg)
        self.assertEqual(self.node.control.state, 'fault')
        self.node.control.reset()
        msg = self.diagnostic()
        msg.header.stamp.sec -= 1
        self.node._on_diagnostic(msg)
        self.assertEqual(self.node.control.state, 'fault')
