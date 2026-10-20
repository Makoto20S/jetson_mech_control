#!/usr/bin/env python3
"""Exercise the running knee ROS node with synthetic paired IMUs; never opens hardware."""

import math
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64
from std_srvs.srv import Trigger


class Probe(Node):
    def __init__(self):
        super().__init__('knee_graph_probe')
        self.thigh = self.create_publisher(Imu, 'imu/imu1/data', qos_profile_sensor_data)
        self.shank = self.create_publisher(Imu, 'imu/imu2/data', qos_profile_sensor_data)
        self.angles = []
        self.diagnostic = None
        self.create_subscription(Float64, 'knee/flexion_deg',
                                 lambda msg: self.angles.append(msg.data), 10)
        self.create_subscription(DiagnosticArray, 'knee/diagnostics',
                                 self.on_diagnostic, 10)

    def on_diagnostic(self, msg):
        if msg.status:
            self.diagnostic = msg.status[0]

    def service(self, name):
        client = self.create_client(Trigger, name)
        deadline = time.monotonic() + 4.0
        while not client.wait_for_service(timeout_sec=0.1):
            if time.monotonic() > deadline:
                raise RuntimeError(f'service unavailable: {name}')
        future = client.call_async(Trigger.Request())
        while not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.02)
        if not future.done() or not future.result().success:
            raise RuntimeError(f'service failed: {name}')

    def publish_pair(self, angle=0.0, rate=0.0, valid=True):
        stamp = self.get_clock().now().to_msg()
        for publisher, degrees, dps, orientation_valid in (
                (self.thigh, 0.0, 0.0, True),
                (self.shank, angle, rate, valid)):
            msg = Imu()
            msg.header.stamp = stamp
            msg.orientation.w = math.cos(math.radians(degrees) / 2.0)
            # Native HiPNUC X maps to standard Y in the node.
            msg.orientation.x = math.sin(math.radians(degrees) / 2.0)
            msg.angular_velocity.x = math.radians(dps)
            msg.linear_acceleration.z = 9.80665
            if not orientation_valid:
                msg.orientation_covariance[0] = -1.0
            publisher.publish(msg)

    def feed(self, seconds, motion=False, angle=0.0, valid=True):
        start = time.monotonic()
        while time.monotonic() - start < seconds:
            elapsed = time.monotonic() - start
            position = 30.0 * math.sin(4.0 * math.pi * elapsed) if motion else angle
            rate = 120.0 * math.pi * math.cos(4.0 * math.pi * elapsed) if motion else 0.0
            self.publish_pair(position, rate, valid)
            rclpy.spin_once(self, timeout_sec=0.01)
            time.sleep(0.01)

    def ramp(self, target, seconds=0.6):
        start = time.monotonic()
        while time.monotonic() - start < seconds:
            fraction = min(1.0, (time.monotonic() - start) / seconds)
            self.publish_pair(target * fraction, target / seconds)
            rclpy.spin_once(self, timeout_sec=0.01)
            time.sleep(0.01)


def main():
    rclpy.init()
    probe = Probe()
    try:
        probe.service('knee/reset_calibration')
        probe.feed(0.4)
        probe.service('knee/start_static_calibration')
        probe.feed(0.9)
        probe.service('knee/start_axis_calibration')
        probe.feed(1.9, motion=True)
        probe.service('knee/start_static_calibration')
        probe.feed(0.9)
        probe.ramp(30.0)
        probe.feed(0.3, angle=30.0)
        if not probe.angles or abs(abs(probe.angles[-1]) - 30.0) > 3.0:
            raise RuntimeError(f'30-degree output missing or inaccurate: {probe.angles[-3:]}')
        probe.feed(0.2, angle=30.0, valid=False)
        count = len(probe.angles)
        probe.feed(0.2, angle=30.0, valid=False)
        if len(probe.angles) != count:
            raise RuntimeError('invalid orientation still produced an angle')
        probe.feed(0.2, angle=20.0)
        if len(probe.angles) <= count:
            raise RuntimeError('valid pair did not recover angle publication')
        time.sleep(0.7)
        for _ in range(10):
            rclpy.spin_once(probe, timeout_sec=0.02)
        if not probe.diagnostic or probe.diagnostic.level != DiagnosticStatus.WARN:
            raise RuntimeError('pair timeout did not raise a warning')
        print(f'ROS knee graph passed: angle={probe.angles[-1]:.2f} deg; '
              'invalid pair suppressed; valid pair recovered; timeout warned')
    finally:
        probe.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
