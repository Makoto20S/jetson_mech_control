"""Real ROS publications, remapped away from the physical motor controller."""

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
class MotorFollowRosTests(unittest.TestCase):
    def test_motor_feedback_simulation_needs_no_imu_and_stops_on_travel(self):
        rclpy.init()
        follower = None
        probe = None
        executor = None
        try:
            follower = MotorFollowNode(
                use_global_arguments=False,
                cli_args=['--ros-args', '-p', 'output_mode:=motor',
                          '-p', 'input_mode:=motor_feedback_simulation',
                          '-p', 'amplitude_deg:=40.0',
                          '-p', 'motor_run_limit_s:=13.0',
                          '-p', 'simulated_travel_limit_rad:=0.75',
                          '-r', '/dm4340_velocity_controller/target_velocity:=/test/sim_target'])
            self.assertEqual(follower.control.amplitude_deg, 40.0)
            self.assertEqual(follower.control.motor_run_limit_s, 13.0)
            self.assertEqual(follower.simulated_travel_limit_rad, 0.75)
            probe = Node('motor_simulation_test_probe')
            motor_pub = probe.create_publisher(JointState, '/joint_states', 10)
            targets = []
            probe.create_subscription(Float64, '/test/sim_target',
                                      lambda msg: targets.append(msg.data), 10)
            client = probe.create_client(Trigger, 'knee_motor_follow/arm')
            executor = SingleThreadedExecutor()
            executor.add_node(follower)
            executor.add_node(probe)

            def feed(position, seconds):
                end = time.monotonic() + seconds
                while time.monotonic() < end:
                    motor = JointState()
                    motor.header.stamp = probe.get_clock().now().to_msg()
                    motor.name = ['dm4340_joint']
                    motor.position = [position]
                    motor.velocity = [0.0]
                    motor_pub.publish(motor)
                    executor.spin_once(timeout_sec=0.015)

            feed(2.0, 0.2)
            self.assertTrue(client.wait_for_service(timeout_sec=1.0))
            future = client.call_async(Trigger.Request())
            end = time.monotonic() + 1.0
            while not future.done() and time.monotonic() < end:
                executor.spin_once(timeout_sec=0.02)
            self.assertTrue(future.result().success)
            feed(2.0, 0.4)
            self.assertTrue(any(value > 0.01 for value in targets), targets)
            feed(2.8, 0.1)
            self.assertEqual(follower.control.state, 'fault')
            end = time.monotonic() + 1.0
            while targets[-1] != 0.0 and time.monotonic() < end:
                executor.spin_once(timeout_sec=0.02)
            self.assertEqual(targets[-1], 0.0)
            self.assertEqual(follower.count_subscribers('knee/joint_state'), 0)
            probe.destroy_client(client)
        finally:
            if follower is not None:
                follower.stop()
            if executor is not None:
                executor.shutdown()
            if probe is not None:
                probe.destroy_node()
            if follower is not None:
                follower.destroy_node()
            rclpy.shutdown()

    def test_motor_mode_remapped_target_and_mismatch_shutdown(self):
        rclpy.init()
        follower = None
        probe = None
        executor = None
        try:
            follower = MotorFollowNode(
                use_global_arguments=False,
                cli_args=['--ros-args', '-p', 'output_mode:=motor',
                          '-p', 'joint_mapping_verified:=true',
                          '-p', 'motor_to_knee_scale:=1.0',
                          '-r', '/dm4340_velocity_controller/target_velocity:=/test/motor_target'])
            probe = Node('motor_follow_test_probe')
            knee_pub = probe.create_publisher(JointState, 'knee/joint_state', 10)
            motor_pub = probe.create_publisher(JointState, '/joint_states', 10)
            diag_pub = probe.create_publisher(DiagnosticArray, 'knee/diagnostics', 10)
            targets = []
            probe.create_subscription(Float64, '/test/motor_target',
                                      lambda msg: targets.append(msg.data), 10)
            client = probe.create_client(Trigger, 'knee_motor_follow/arm')
            executor = SingleThreadedExecutor()
            executor.add_node(follower)
            executor.add_node(probe)

            def spin_for(seconds, motor_position=0.0):
                end = time.monotonic() + seconds
                while time.monotonic() < end:
                    knee = JointState()
                    knee.header.stamp = probe.get_clock().now().to_msg()
                    knee.name = ['knee_joint']
                    knee.position = [0.0]
                    knee.velocity = [0.0]
                    knee_pub.publish(knee)
                    motor = JointState()
                    motor.header.stamp = knee.header.stamp
                    motor.name = ['dm4340_joint']
                    motor.position = [motor_position]
                    motor.velocity = [0.0]
                    motor_pub.publish(motor)
                    diagnostic = DiagnosticArray()
                    diagnostic.header.stamp = probe.get_clock().now().to_msg()
                    status = DiagnosticStatus()
                    status.name = 'knee_kinematics'
                    status.level = DiagnosticStatus.OK
                    diagnostic.status = [status]
                    diag_pub.publish(diagnostic)
                    executor.spin_once(timeout_sec=0.015)

            spin_for(0.3)
            self.assertTrue(client.wait_for_service(timeout_sec=1.0))
            future = client.call_async(Trigger.Request())
            end = time.monotonic() + 1.0
            while not future.done() and time.monotonic() < end:
                executor.spin_once(timeout_sec=0.02)
            self.assertTrue(future.done())
            self.assertTrue(future.result().success, future.result().message)
            spin_for(0.4)
            self.assertTrue(any(value > 0.01 for value in targets), targets)
            spin_for(0.1, motor_position=1.0)
            end = time.monotonic() + 1.0
            while follower.control.state != 'fault' and time.monotonic() < end:
                executor.spin_once(timeout_sec=0.02)
            self.assertEqual(follower.control.state, 'fault')
            end = time.monotonic() + 1.0
            while (not targets or targets[-1] != 0.0) and time.monotonic() < end:
                executor.spin_once(timeout_sec=0.02)
            self.assertEqual(targets[-1], 0.0)
            self.assertNotIn('/dm4340_velocity_controller/target_velocity',
                             [topic for topic, _ in follower.get_publisher_names_and_types_by_node(
                                 'knee_motor_follow', '/')])
            probe.destroy_client(client)
        finally:
            if follower is not None:
                follower.stop()
            if executor is not None:
                executor.shutdown()
            if probe is not None:
                probe.destroy_node()
            if follower is not None:
                follower.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    unittest.main()
