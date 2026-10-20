#!/usr/bin/env python3
"""Display the live CtrBoard ROS 2 sensor state on one terminal line."""

from __future__ import annotations

import math
import sys
import time

import rclpy
from geometry_msgs.msg import Vector3Stamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import UInt8MultiArray, UInt32


class CtrBoardMonitor(Node):
    def __init__(self) -> None:
        super().__init__("ctrboard_terminal_monitor")
        self.imu1 = [math.nan, math.nan, math.nan]
        self.imu2 = [math.nan, math.nan, math.nan]
        self.left_total = 0
        self.right_total = 0
        self.status = [0, 0, 0, 0]
        self.timestamp_ms = 0
        self.sample_count = 0
        self.started_at = time.monotonic()

        self.create_subscription(
            Vector3Stamped,
            "/imu/imu1/euler_deg",
            lambda message: self._set_euler(self.imu1, message),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Vector3Stamped,
            "/imu/imu2/euler_deg",
            lambda message: self._set_euler(self.imu2, message),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            UInt32,
            "/fsr/left/total",
            self._set_left_total,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            UInt32,
            "/fsr/right/total",
            self._set_right_total,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            UInt8MultiArray,
            "/ctrboard/sensor_status",
            self._set_status,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            UInt32,
            "/ctrboard/timestamp_ms",
            self._set_timestamp,
            qos_profile_sensor_data,
        )
        self.create_timer(0.1, self._render)

    @staticmethod
    def _set_euler(target: list[float], message: Vector3Stamped) -> None:
        target[:] = [message.vector.x, message.vector.y, message.vector.z]

    def _set_left_total(self, message: UInt32) -> None:
        self.left_total = message.data

    def _set_right_total(self, message: UInt32) -> None:
        self.right_total = message.data

    def _set_status(self, message: UInt8MultiArray) -> None:
        values = list(message.data[:4])
        self.status = values + [0] * (4 - len(values))

    def _set_timestamp(self, message: UInt32) -> None:
        self.timestamp_ms = message.data
        self.sample_count += 1

    @staticmethod
    def _angles(values: list[float]) -> str:
        if not all(math.isfinite(value) for value in values):
            return "waiting"
        return "R={:7.2f} P={:7.2f} Y={:7.2f}".format(*values)

    def _render(self) -> None:
        elapsed = max(time.monotonic() - self.started_at, 1.0e-6)
        rate = self.sample_count / elapsed
        line = (
            f"t={self.timestamp_ms:10d} ms | "
            f"IMU1 {self._angles(self.imu1)} | "
            f"IMU2 {self._angles(self.imu2)} | "
            f"FSR L={self.left_total:6d} R={self.right_total:6d} | "
            f"status={self.status} | {rate:5.1f} Hz"
        )
        sys.stdout.write(f"\r\033[2K{line}")
        sys.stdout.flush()


def main() -> int:
    rclpy.init()
    node = CtrBoardMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        sys.stdout.write("\n")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
