"""Non-actuating dual-IMU -> knee -> shadow-assist ROS integration check."""

import math
import threading
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Float64

from mech_knee_kinematics.node import KneeKinematicsNode
from mech_knee_kinematics.shadow_assist import ShadowAssistNode
from mech_assist_control.node import MotorFollowNode
from check_knee_ros_graph import Probe


def require(condition, message):
    if not condition:
        raise RuntimeError(message)


class ArchitectureProbe(Probe):
    def __init__(self):
        super().__init__()
        self.virtual_efforts = []
        self.preview_velocities = []
        self.shadow_status = None
        self.create_subscription(Float64, 'assist_shadow/virtual_effort_nm',
                                 lambda msg: self.virtual_efforts.append(msg.data), 10)
        self.create_subscription(Float64, 'knee_motor_follow/preview_velocity',
                                 lambda msg: self.preview_velocities.append(msg.data), 10)
        self.create_subscription(DiagnosticArray, 'assist_shadow/diagnostics',
                                 self._on_shadow, 10)

    def _on_shadow(self, msg):
        if msg.status:
            self.shadow_status = msg.status[0]

    def wait_for(self, predicate, seconds=2.0):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            if predicate():
                return True
            rclpy.spin_once(self, timeout_sec=0.02)
        return False


def main():
    rclpy.init(args=['--ros-args', '-p', 'static_seconds:=0.5',
                     '-p', 'motion_seconds:=1.5', '-p', 'flexion_sign:=1'])
    knee = KneeKinematicsNode()
    shadow = ShadowAssistNode()
    follow = MotorFollowNode()
    probe = ArchitectureProbe()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(knee)
    executor.add_node(shadow)
    executor.add_node(follow)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    try:
        probe.service('knee/reset_calibration')
        probe.feed(0.3)
        probe.service('knee/start_static_calibration')
        probe.feed(0.85)
        probe.service('knee/start_axis_calibration')
        probe.feed(1.85, motion=True)
        probe.service('knee/start_static_calibration')
        probe.feed(0.85)
        probe.ramp(20.0)
        probe.feed(0.35, angle=20.0)
        require(probe.angles and abs(probe.angles[-1] - 20.0) < 3.0,
                f'knee angle missing: {probe.angles[-3:]}')
        require(probe.diagnostic is not None and
                probe.diagnostic.level == DiagnosticStatus.OK,
                f'knee calibration not healthy: '
                f'{probe.diagnostic.message if probe.diagnostic else "missing"}')
        probe.service('assist_shadow/arm')
        probe.service('knee_motor_follow/arm')
        probe.feed(0.35, angle=20.0)
        require(any(abs(value) > 0.01 for value in probe.virtual_efforts),
                'no virtual output after arming')
        require(any(abs(value) > 0.01 for value in probe.preview_velocities),
                'no preview motor velocity after arming')
        marker = len(probe.virtual_efforts)
        probe.feed(0.4, angle=20.0, valid=False)
        require(probe.wait_for(lambda: probe.shadow_status is not None and
                               probe.shadow_status.level == DiagnosticStatus.ERROR),
                'invalid IMU did not fault the shadow controller')
        require(probe.wait_for(lambda: len(probe.virtual_efforts) > marker and
                               probe.virtual_efforts[-1] == 0.0),
                'invalid IMU did not clear virtual output')
        require(probe.wait_for(lambda: follow.control.state == 'fault'),
                'invalid IMU did not fault motor follower')
        require(probe.wait_for(lambda: probe.preview_velocities and
                               probe.preview_velocities[-1] == 0.0),
                'invalid IMU did not clear preview motor velocity')
        published = {name for name, _ in probe.get_publisher_names_and_types_by_node(
            'shadow_assist', '/')}
        require('/dm4340_velocity_controller/target_velocity' not in published,
                'shadow controller exposed a motor command publisher')
        published = {name for name, _ in probe.get_publisher_names_and_types_by_node(
            'knee_motor_follow', '/')}
        require('/dm4340_velocity_controller/target_velocity' not in published,
                'preview follower exposed a motor command publisher')
        print(f'PASS architecture ROS graph: {probe.angles[-1]:.2f} deg; '
              'virtual effort active then fault-zero; motor publisher absent')
    finally:
        executor.shutdown()
        thread.join(timeout=2.0)
        probe.destroy_node()
        shadow.destroy_node()
        follow.destroy_node()
        knee.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
