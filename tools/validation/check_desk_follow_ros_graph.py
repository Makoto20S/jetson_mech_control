#!/usr/bin/env python3
"""Synthetic ROS preview and motor-topic checks; no hardware/serial or manager."""
import math
import time
from types import SimpleNamespace
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.executors import SingleThreadedExecutor
from geometry_msgs.msg import QuaternionStamped
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from mech_assist_control.desk_node import DeskFollowNode


def run(mode):
    app = DeskFollowNode(parameter_overrides=[Parameter('output_mode', value=mode)])
    probe = Node('desk_follow_synthetic_probe')
    executor = SingleThreadedExecutor()
    executor.add_node(app); executor.add_node(probe)
    pose = probe.create_publisher(QuaternionStamped, 'attitude/relative/orientation', 10)
    diag = probe.create_publisher(DiagnosticArray, 'ctrboard/diagnostics', 10)
    joint = probe.create_publisher(JointState, '/joint_states', 10)
    outputs = []
    probe.create_subscription(Float64, app.target_topic, lambda msg: outputs.append(msg.data), 10)
    angle = [0.0]
    def publish():
        stamp = probe.get_clock().now().to_msg()
        msg = QuaternionStamped(); msg.header.stamp = stamp
        msg.quaternion.w = math.cos(math.radians(angle[0])/2)
        msg.quaternion.y = math.sin(math.radians(angle[0])/2)
        pose.publish(msg)
        status = DiagnosticStatus(); status.name = 'ctrboard_sensor_safety'; status.level = DiagnosticStatus.OK
        message = DiagnosticArray(); message.header.stamp = stamp
        message.status = [status]; diag.publish(message)
        feedback = JointState(); feedback.header.stamp = stamp
        feedback.name, feedback.position, feedback.velocity = ['dm4340_joint'], [0.0], [0.0]
        joint.publish(feedback)
    timer = probe.create_timer(0.05, publish)
    def spin(seconds):
        end = time.monotonic()+seconds
        while time.monotonic()<end:
            executor.spin_once(timeout_sec=0.005)
    try:
        spin(0.5)
        if mode == 'preview':
            assert probe.count_publishers('/dm4340_velocity_controller/target_velocity') == 0
            assert not outputs
        response = app.arm(None, SimpleNamespace())
        assert response.success, response.message
        angle[0] = 20
        spin(0.8)
        assert app.logic.state == 'armed' and max(outputs)>0.1
        assert all(abs(value)<=0.3 for value in outputs)
        probe.destroy_timer(timer)
        spin(0.5)
        assert app.logic.state == 'fault' and outputs[-1] == 0
        count = len(outputs); spin(0.2)
        assert len(outputs) == count, 'fault must stop refreshing targets'
        print(f'PASS desk {mode}: arm, bounded target, stale fault-zero and no refresh')
    finally:
        executor.shutdown(); app.destroy_node(); probe.destroy_node()


def main():
    rclpy.init()
    try:
        run('preview'); run('motor')
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
