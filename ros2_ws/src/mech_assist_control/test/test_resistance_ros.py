"""Real ROS graph test for the preview-only resistance node."""

import time
import unittest

try:
    import rclpy
    from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Float64
    from std_srvs.srv import Trigger
    from mech_assist_control.node import MotorFollowNode
except ImportError:
    rclpy = None


@unittest.skipIf(rclpy is None, 'ROS 2 is not installed')
class ResistanceRosTests(unittest.TestCase):
    def test_reverse_mount_maps_knee_resistance_to_motor_effort(self):
        rclpy.init()
        controller = None
        probe = None
        executor = None
        try:
            controller = MotorFollowNode(
                use_global_arguments=False,
                cli_args=['--ros-args', '-p', 'control_strategy:=resistance',
                          '-p', 'output_mode:=motor',
                          '-p', 'joint_mapping_verified:=true',
                          '-p', 'motor_to_knee_scale:=-2.0',
                          '-p', 'motor_to_knee_offset_rad:=0.5'])
            probe = Node('reverse_resistance_probe')
            knee_pub = probe.create_publisher(JointState, 'knee/joint_state', 10)
            motor_pub = probe.create_publisher(JointState, '/joint_states', 10)
            diag_pub = probe.create_publisher(DiagnosticArray, 'knee/diagnostics', 10)
            efforts = []
            probe.create_subscription(
                Float64, '/dm4340_effort_controller/target_effort',
                lambda msg: efforts.append(msg.data), 10)
            client = probe.create_client(Trigger, 'knee_control/arm')
            executor = SingleThreadedExecutor()
            executor.add_node(controller)
            executor.add_node(probe)

            def feed(seconds):
                end = time.monotonic() + seconds
                while time.monotonic() < end:
                    stamp = probe.get_clock().now().to_msg()
                    knee = JointState()
                    knee.header.stamp = stamp
                    knee.name = ['knee_joint']
                    knee.position = [0.5]
                    knee.velocity = [0.8]
                    knee_pub.publish(knee)
                    motor = JointState()
                    motor.header.stamp = stamp
                    motor.name = ['dm4340_joint']
                    motor.position = [0.0]
                    motor.velocity = [-0.4]
                    motor_pub.publish(motor)
                    diagnostic = DiagnosticArray()
                    diagnostic.header.stamp = probe.get_clock().now().to_msg()
                    status = DiagnosticStatus()
                    status.name = 'knee_kinematics'
                    status.level = DiagnosticStatus.OK
                    diagnostic.status = [status]
                    diag_pub.publish(diagnostic)
                    executor.spin_once(timeout_sec=0.01)

            feed(0.3)
            self.assertTrue(client.wait_for_service(timeout_sec=1.0))
            future = client.call_async(Trigger.Request())
            end = time.monotonic() + 1.0
            while not future.done() and time.monotonic() < end:
                executor.spin_once(timeout_sec=0.02)
            self.assertTrue(future.done())
            self.assertTrue(future.result().success, future.result().message)
            feed(0.4)
            self.assertTrue(any(value > 0 for value in efforts))
            self.assertTrue(all(0 <= value <= 0.3 for value in efforts))
            self.assertTrue(all(value * -0.4 <= 0 for value in efforts))
            probe.destroy_client(client)
        finally:
            if executor is not None:
                executor.shutdown()
            if probe is not None:
                probe.destroy_node()
            if controller is not None:
                controller.destroy_node()
            rclpy.shutdown()

    def test_motor_effort_requires_matching_feedback_and_stops_after_fault(self):
        rclpy.init()
        controller = None
        probe = None
        executor = None
        try:
            controller = MotorFollowNode(
                use_global_arguments=False,
                cli_args=['--ros-args', '-p', 'control_strategy:=resistance',
                          '-p', 'output_mode:=motor',
                          '-p', 'joint_mapping_verified:=true',
                          '-p', 'motor_to_knee_scale:=1.0'])
            probe = Node('resistance_motor_test_probe')
            knee_pub = probe.create_publisher(JointState, 'knee/joint_state', 10)
            motor_pub = probe.create_publisher(JointState, '/joint_states', 10)
            diag_pub = probe.create_publisher(DiagnosticArray, 'knee/diagnostics', 10)
            efforts = []
            probe.create_subscription(
                Float64, '/dm4340_effort_controller/target_effort',
                lambda msg: efforts.append(msg.data), 10)
            client = probe.create_client(Trigger, 'knee_control/arm')
            executor = SingleThreadedExecutor()
            executor.add_node(controller)
            executor.add_node(probe)

            def feed(velocity, motor_velocity, seconds):
                end = time.monotonic() + seconds
                while time.monotonic() < end:
                    stamp = probe.get_clock().now().to_msg()
                    knee = JointState()
                    knee.header.stamp = stamp
                    knee.name = ['knee_joint']
                    knee.position = [0.5]
                    knee.velocity = [velocity]
                    knee_pub.publish(knee)
                    motor = JointState()
                    motor.header.stamp = stamp
                    motor.name = ['dm4340_joint']
                    motor.position = [0.5]
                    motor.velocity = [motor_velocity]
                    motor_pub.publish(motor)
                    status = DiagnosticStatus()
                    status.name = 'knee_kinematics'
                    status.level = DiagnosticStatus.OK
                    diagnostic = DiagnosticArray()
                    diagnostic.header.stamp = probe.get_clock().now().to_msg()
                    diagnostic.status = [status]
                    diag_pub.publish(diagnostic)
                    executor.spin_once(timeout_sec=0.01)

            feed(0.5, 0.5, 0.3)
            self.assertTrue(client.wait_for_service(timeout_sec=1.0))
            future = client.call_async(Trigger.Request())
            end = time.monotonic() + 1.0
            while not future.done() and time.monotonic() < end:
                executor.spin_once(timeout_sec=0.02)
            self.assertTrue(future.done())
            self.assertTrue(future.result().success, future.result().message)
            feed(0.5, 0.5, 0.15)
            self.assertTrue(any(value < 0 for value in efforts))
            feed(-0.5, -0.5, 0.15)
            self.assertTrue(any(value > 0 for value in efforts))
            feed(-0.5, 0.5, 0.06)
            self.assertEqual(controller.control.state, 'fault')
            self.assertEqual(efforts[-1], 0.0)
            count = len(efforts)
            end = time.monotonic() + 0.15
            while time.monotonic() < end:
                executor.spin_once(timeout_sec=0.01)
            self.assertEqual(len(efforts), count)
            probe.destroy_client(client)
        finally:
            if executor is not None:
                executor.shutdown()
            if probe is not None:
                probe.destroy_node()
            if controller is not None:
                controller.destroy_node()
            rclpy.shutdown()

    def test_opposing_torque_and_fault_zero_without_motor_publisher(self):
        rclpy.init()
        controller = None
        probe = None
        executor = None
        try:
            controller = MotorFollowNode(
                use_global_arguments=False,
                cli_args=['--ros-args', '-p', 'control_strategy:=resistance'])
            probe = Node('resistance_test_probe')
            knee_pub = probe.create_publisher(JointState, 'knee/joint_state', 10)
            diag_pub = probe.create_publisher(DiagnosticArray, 'knee/diagnostics', 10)
            efforts = []
            probe.create_subscription(Float64,
                                      'knee_resistance/preview_effort_nm',
                                      lambda msg: efforts.append(msg.data), 10)
            client = probe.create_client(Trigger, 'knee_control/arm')
            executor = SingleThreadedExecutor()
            executor.add_node(controller)
            executor.add_node(probe)

            def feed(velocity, healthy, seconds):
                end = time.monotonic() + seconds
                while time.monotonic() < end:
                    knee = JointState()
                    knee.header.stamp = probe.get_clock().now().to_msg()
                    knee.name = ['knee_joint']
                    knee.position = [0.5]
                    knee.velocity = [velocity]
                    knee_pub.publish(knee)
                    status = DiagnosticStatus()
                    status.name = 'knee_kinematics'
                    status.level = (DiagnosticStatus.OK if healthy else
                                    DiagnosticStatus.WARN)
                    diagnostic = DiagnosticArray()
                    diagnostic.header.stamp = probe.get_clock().now().to_msg()
                    diagnostic.status = [status]
                    diag_pub.publish(diagnostic)
                    executor.spin_once(timeout_sec=0.01)

            feed(0.5, True, 0.3)
            self.assertTrue(client.wait_for_service(timeout_sec=1.0))
            future = client.call_async(Trigger.Request())
            end = time.monotonic() + 1.0
            while not future.done() and time.monotonic() < end:
                executor.spin_once(timeout_sec=0.02)
            self.assertTrue(future.done())
            self.assertTrue(future.result().success, future.result().message)
            feed(0.5, True, 0.15)
            self.assertTrue(any(value < 0 for value in efforts))
            feed(-0.5, True, 0.15)
            self.assertTrue(any(value > 0 for value in efforts))
            feed(-0.5, False, 0.1)
            self.assertEqual(controller.control.state, 'fault')
            self.assertEqual(efforts[-1], 0.0)
            self.assertNotIn(
                '/dm4340_velocity_controller/target_velocity',
                [topic for topic, _ in controller.get_publisher_names_and_types_by_node(
                    'knee_motor_follow', '/')])
            self.assertNotIn(
                '/dm4340_effort_controller/target_effort',
                [topic for topic, _ in controller.get_publisher_names_and_types_by_node(
                    'knee_motor_follow', '/')])
            probe.destroy_client(client)
        finally:
            if executor is not None:
                executor.shutdown()
            if probe is not None:
                probe.destroy_node()
            if controller is not None:
                controller.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    unittest.main()
