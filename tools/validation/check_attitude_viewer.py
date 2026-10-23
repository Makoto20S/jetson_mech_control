#!/usr/bin/env python3
"""Bounded Tk + synthetic ROS subscriber test. Needs DISPLAY; no device I/O.

Run in an isolated ROS_DOMAIN_ID with ROS_LOCALHOST_ONLY=1.
"""
import time
import tkinter as tk
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import QuaternionStamped, Vector3Stamped
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from sensor_msgs.msg import JointState
from mech_knee_kinematics.attitude_dashboard import AttitudeDashboard, make_subscriber
from mech_knee_kinematics.attitude_view_state import AttitudeViewState
from mech_knee_kinematics.dual_imu_knee import quaternion_from_euler_degrees


def main():
    rclpy.init()
    state = AttitudeViewState()
    subscriber = make_subscriber(state)
    probe = Node('synthetic_attitude_viewer_probe')
    pubs = {}
    for name in ('thigh', 'shank'):
        pubs[name] = probe.create_publisher(
            QuaternionStamped, f'attitude/{name}_from_zero/orientation', 10)
        pubs[name+'_rpy'] = probe.create_publisher(
            Vector3Stamped, f'attitude/{name}/rpy_rad', 10)
    joint = probe.create_publisher(JointState, 'knee/joint_state', 10)
    diagnostic = probe.create_publisher(DiagnosticArray, 'knee/diagnostics', 10)
    start = time.monotonic()

    def publish():
        elapsed = time.monotonic()-start
        if 1.5 < elapsed < 2.5:
            return
        stamp = probe.get_clock().now().to_msg()
        for name, angle in (('thigh', 20), ('shank', 60)):
            msg = QuaternionStamped()
            msg.header.stamp = stamp
            q = quaternion_from_euler_degrees(0, angle, 0)
            msg.quaternion.w, msg.quaternion.x, msg.quaternion.y, msg.quaternion.z = q
            pubs[name].publish(msg)
            rpy = Vector3Stamped()
            rpy.vector.y = angle*3.141592653589793/180
            pubs[name+'_rpy'].publish(rpy)
        msg = JointState()
        msg.header.stamp = stamp
        msg.name, msg.position, msg.velocity = ['knee_joint'], [0.6981317008], [0.1]
        joint.publish(msg)
        diag = DiagnosticArray()
        status = DiagnosticStatus()
        status.name, status.level, status.message = 'knee_kinematics', DiagnosticStatus.OK, 'calibrated'
        diag.status = [status]
        diagnostic.publish(diag)

    probe.create_timer(0.02, publish)
    root = tk.Tk()
    errors = []
    checks = []
    root.report_callback_exception = lambda *exc: errors.append(str(exc))

    def pump():
        rclpy.spin_once(probe, timeout_sec=0)
        for _ in range(32):
            rclpy.spin_once(subscriber, timeout_sec=0)
    AttitudeDashboard(root, state, pump=pump)

    def check(expected):
        checks.append(state.ready(time.monotonic())[0] == expected)
        if expected:
            checks.append(state.joint is not None and abs(state.joint[1]-40) < 0.01)
    root.after(1200, lambda: check(True))
    root.after(2200, lambda: check(False))
    root.after(3600, lambda: check(True))
    root.after(3900, root.destroy)
    try:
        root.mainloop()
        assert not errors, errors
        assert len(checks) == 5 and all(checks), checks
        assert len(state.history) > 5
        assert 'thigh' in state.rpy and 'shank' in state.rpy
        print('PASS attitude viewer: ROS pose/joint/diagnostics, stale-grey state, '
              'recovery, curve history, rendered Tk callbacks and clean exit')
    finally:
        subscriber.destroy_node()
        probe.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
