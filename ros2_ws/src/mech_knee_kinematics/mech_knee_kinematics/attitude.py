"""Receive-only segment attitudes; quaternions use wxyz, angles use radians.

Uses the sensor's onboard orientation solution. This is not a raw accel/gyro
AHRS and does not establish a geographic or anatomical reference frame.
"""
from dataclasses import dataclass
import math

from .dual_imu_knee import (
    quaternion_average, quaternion_conjugate, quaternion_multiply,
    quaternion_normalize, relative_quaternion,
)


def euler_zyx(quaternion):
    """Return roll, pitch, yaw; Euler singularities remain display-only."""
    w, x, y, z = quaternion_normalize(quaternion)
    return (
        math.atan2(2 * (w*x + y*z), 1 - 2 * (x*x + y*y)),
        math.asin(max(-1.0, min(1.0, 2 * (w*y - z*x)))),
        math.atan2(2 * (w*z + x*y), 1 - 2 * (y*y + z*z)),
    )


@dataclass(frozen=True)
class AttitudeSnapshot:
    thigh: tuple
    shank: tuple
    relative: tuple
    thigh_from_zero: tuple | None
    shank_from_zero: tuple | None


class SegmentAttitudes:
    def __init__(self):
        self.reset()

    def reset(self):
        self.neutral_thigh = None
        self.neutral_shank = None

    def calibrate_zero(self, pairs):
        if not pairs:
            raise ValueError('standing pairs required')
        # Compute both before assignment: invalid data cannot install half a zero.
        thigh = quaternion_average([pair[0] for pair in pairs])
        shank = quaternion_average([pair[1] for pair in pairs])
        self.neutral_thigh, self.neutral_shank = thigh, shank

    def update(self, thigh, shank):
        thigh, shank = quaternion_normalize(thigh), quaternion_normalize(shank)
        return AttitudeSnapshot(
            thigh, shank, relative_quaternion(thigh, shank),
            None if self.neutral_thigh is None else quaternion_multiply(
                quaternion_conjugate(self.neutral_thigh), thigh),
            None if self.neutral_shank is None else quaternion_multiply(
                quaternion_conjugate(self.neutral_shank), shank),
        )
