"""Quaternion-based knee flexion estimation for two body-mounted IMUs."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Iterable, Optional, Sequence, Tuple

Quaternion = Tuple[float, float, float, float]  # w, x, y, z
Vector3 = Tuple[float, float, float]


def vector_dot(lhs: Sequence[float], rhs: Sequence[float]) -> float:
    if len(lhs) != 3 or len(rhs) != 3 or not _finite((*lhs, *rhs)):
        raise ValueError("invalid 3D vector")
    return sum(float(a) * float(b) for a, b in zip(lhs, rhs))


def _finite(values: Iterable[float]) -> bool:
    return all(math.isfinite(float(value)) for value in values)


def vector_norm(vector: Sequence[float]) -> float:
    return math.sqrt(sum(float(value) * float(value) for value in vector))


def normalize_vector(vector: Sequence[float]) -> Vector3:
    if len(vector) != 3 or not _finite(vector):
        raise ValueError("invalid 3D vector")
    norm = vector_norm(vector)
    if norm < 1.0e-9:
        raise ValueError("zero-length 3D vector")
    return tuple(float(value) / norm for value in vector)  # type: ignore[return-value]


def quaternion_normalize(quaternion: Sequence[float]) -> Quaternion:
    if len(quaternion) != 4 or not _finite(quaternion):
        raise ValueError("invalid quaternion")
    norm = vector_norm(quaternion)
    if norm < 1.0e-9:
        raise ValueError("zero-length quaternion")
    return tuple(float(value) / norm for value in quaternion)  # type: ignore[return-value]


def hipnuc_quaternion_to_standard(quaternion: Sequence[float]) -> Quaternion:
    """Map HiPNUC q0/q1/q2/q3 into the ZYX frame used by its Euler output."""
    q0, q1, q2, q3 = quaternion_normalize(quaternion)
    return quaternion_normalize((q0, q2, q1, -q3))


def hipnuc_vector_to_standard(vector: Sequence[float]) -> Vector3:
    """Apply the matching right-handed basis mapping to IMU body vectors."""
    if len(vector) != 3 or not _finite(vector):
        raise ValueError("invalid HiPNUC vector")
    x, y, z = (float(value) for value in vector)
    return y, x, -z


def quaternion_conjugate(quaternion: Sequence[float]) -> Quaternion:
    w, x, y, z = quaternion_normalize(quaternion)
    return w, -x, -y, -z


def quaternion_multiply(lhs: Sequence[float], rhs: Sequence[float]) -> Quaternion:
    aw, ax, ay, az = quaternion_normalize(lhs)
    bw, bx, by, bz = quaternion_normalize(rhs)
    return quaternion_normalize(
        (
            aw * bw - ax * bx - ay * by - az * bz,
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
        )
    )


def quaternion_rotate_vector(
    quaternion: Sequence[float], vector: Sequence[float]
) -> Vector3:
    """Rotate a 3D vector by a wxyz quaternion without normalizing the vector."""
    if len(vector) != 3 or not _finite(vector):
        raise ValueError("invalid 3D vector")
    w, x, y, z = quaternion_normalize(quaternion)
    vx, vy, vz = (float(value) for value in vector)
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    return (
        vx + w * tx + (y * tz - z * ty),
        vy + w * ty + (z * tx - x * tz),
        vz + w * tz + (x * ty - y * tx),
    )


def quaternion_from_euler_degrees(roll: float, pitch: float, yaw: float) -> Quaternion:
    """Convert intrinsic roll-pitch-yaw (ZYX convention) to wxyz."""
    if not _finite((roll, pitch, yaw)):
        raise ValueError("non-finite Euler angle")
    half_roll = math.radians(roll) * 0.5
    half_pitch = math.radians(pitch) * 0.5
    half_yaw = math.radians(yaw) * 0.5
    cr, sr = math.cos(half_roll), math.sin(half_roll)
    cp, sp = math.cos(half_pitch), math.sin(half_pitch)
    cy, sy = math.cos(half_yaw), math.sin(half_yaw)
    return quaternion_normalize(
        (
            cr * cp * cy + sr * sp * sy,
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
        )
    )


def relative_quaternion(proximal: Sequence[float], distal: Sequence[float]) -> Quaternion:
    """Return distal orientation relative to the proximal segment."""
    return quaternion_multiply(quaternion_conjugate(proximal), distal)


def quaternion_average(samples: Sequence[Sequence[float]]) -> Quaternion:
    if not samples:
        raise ValueError("no quaternion samples")
    reference = quaternion_normalize(samples[0])
    total = [0.0, 0.0, 0.0, 0.0]
    for sample in samples:
        current = quaternion_normalize(sample)
        if sum(a * b for a, b in zip(reference, current)) < 0.0:
            current = tuple(-value for value in current)  # type: ignore[assignment]
        for index, value in enumerate(current):
            total[index] += value
    return quaternion_normalize(total)


def quaternion_rotation_vector(quaternion: Sequence[float]) -> Vector3:
    w, x, y, z = quaternion_normalize(quaternion)
    if w < 0.0:
        w, x, y, z = -w, -x, -y, -z
    vector_length = math.sqrt(x * x + y * y + z * z)
    if vector_length < 1.0e-9:
        return 0.0, 0.0, 0.0
    angle = 2.0 * math.atan2(vector_length, max(0.0, w))
    scale = angle / vector_length
    return x * scale, y * scale, z * scale


def principal_axis(rotation_vectors: Sequence[Sequence[float]]) -> Vector3:
    """Estimate the dominant hinge axis using a 3x3 covariance matrix."""
    if len(rotation_vectors) < 10:
        raise ValueError("at least 10 motion samples are required")
    vectors = [tuple(float(value) for value in vector) for vector in rotation_vectors]
    if any(len(vector) != 3 or not _finite(vector) for vector in vectors):
        raise ValueError("invalid motion samples")
    means = [sum(vector[index] for vector in vectors) / len(vectors) for index in range(3)]
    covariance = [[0.0] * 3 for _ in range(3)]
    for vector in vectors:
        centered = [vector[index] - means[index] for index in range(3)]
        for row in range(3):
            for column in range(3):
                covariance[row][column] += centered[row] * centered[column]
    axis = [1.0, 1.0, 1.0]
    for _ in range(24):
        candidate = [
            sum(covariance[row][column] * axis[column] for column in range(3))
            for row in range(3)
        ]
        norm = vector_norm(candidate)
        if norm < 1.0e-8:
            raise ValueError("motion amplitude is too small")
        axis = [value / norm for value in candidate]
    projected_energy = sum(
        sum((vector[index] - means[index]) * axis[index] for index in range(3)) ** 2
        for vector in vectors
    )
    if projected_energy / len(vectors) < math.radians(3.0) ** 2:
        raise ValueError("motion amplitude is too small")
    return normalize_vector(axis)


@dataclass(frozen=True)
class HingeAxisCalibration:
    proximal_axis: Vector3
    distal_axis: Vector3
    explained_ratio: float
    rms_rate_dps: float
    sample_count: int


def calibrate_hinge_axis_from_gyroscopes(
    samples: Sequence[
        Tuple[Sequence[float], Sequence[float], Sequence[float], Sequence[float]]
    ],
    reference_axis: Sequence[float] = (0.0, 1.0, 0.0),
    minimum_rate_dps: float = 8.0,
    minimum_explained_ratio: float = 0.70,
) -> HingeAxisCalibration:
    """Estimate thigh/shank hinge axes from synchronized quaternion and gyro data.

    Each sample is ``(thigh_q, shank_q, thigh_gyro, shank_gyro)``. Gyroscope
    units may be deg/s or rad/s as long as both sensors use the same unit.
    """
    if len(samples) < 20:
        raise ValueError("at least 20 synchronized gyro samples are required")

    relative_rates = []
    relative_orientations = []
    for thigh_q, shank_q, thigh_gyro, shank_gyro in samples:
        if (
            len(thigh_gyro) != 3
            or len(shank_gyro) != 3
            or not _finite((*thigh_gyro, *shank_gyro))
        ):
            continue
        relative = relative_quaternion(thigh_q, shank_q)
        distal_rate_in_proximal = quaternion_rotate_vector(relative, shank_gyro)
        relative_rate = tuple(
            distal_rate_in_proximal[index] - float(thigh_gyro[index])
            for index in range(3)
        )
        if vector_norm(relative_rate) >= minimum_rate_dps:
            relative_rates.append(relative_rate)
            relative_orientations.append(relative)

    if len(relative_rates) < 20:
        raise ValueError("joint motion is too small for axis calibration")

    proximal_axis = principal_axis(relative_rates)
    reference = normalize_vector(reference_axis)
    if vector_dot(proximal_axis, reference) < 0.0:
        proximal_axis = tuple(-value for value in proximal_axis)  # type: ignore[assignment]

    means = [
        sum(rate[index] for rate in relative_rates) / len(relative_rates)
        for index in range(3)
    ]
    centered = [
        tuple(rate[index] - means[index] for index in range(3))
        for rate in relative_rates
    ]
    total_energy = sum(vector_dot(rate, rate) for rate in centered)
    axial_energy = sum(vector_dot(rate, proximal_axis) ** 2 for rate in centered)
    explained_ratio = axial_energy / total_energy if total_energy > 1.0e-9 else 0.0
    if explained_ratio < minimum_explained_ratio:
        raise ValueError(
            f"motion is not hinge-like enough ({explained_ratio * 100.0:.0f}%)"
        )

    distal_axes = []
    for relative in relative_orientations:
        candidate = quaternion_rotate_vector(
            quaternion_conjugate(relative), proximal_axis
        )
        if distal_axes and vector_dot(candidate, distal_axes[0]) < 0.0:
            candidate = tuple(-value for value in candidate)
        distal_axes.append(candidate)
    distal_axis = normalize_vector(
        tuple(
            sum(axis[index] for axis in distal_axes) / len(distal_axes)
            for index in range(3)
        )
    )
    rms_rate = math.sqrt(
        sum(vector_dot(rate, rate) for rate in relative_rates) / len(relative_rates)
    )
    return HingeAxisCalibration(
        proximal_axis=proximal_axis,
        distal_axis=distal_axis,
        explained_ratio=explained_ratio,
        rms_rate_dps=rms_rate,
        sample_count=len(relative_rates),
    )


def wrap_degrees(angle: float) -> float:
    return (float(angle) + 180.0) % 360.0 - 180.0


def signed_twist_degrees(quaternion: Sequence[float], axis: Sequence[float]) -> float:
    """Extract signed rotation around a normalized hinge axis."""
    w, x, y, z = quaternion_normalize(quaternion)
    ax, ay, az = normalize_vector(axis)
    projection = x * ax + y * ay + z * az
    twist = quaternion_normalize((w, ax * projection, ay * projection, az * projection))
    tw, tx, ty, tz = twist
    signed_vector = tx * ax + ty * ay + tz * az
    return wrap_degrees(math.degrees(2.0 * math.atan2(signed_vector, tw)))


def segment_twist_degrees(
    neutral_orientation: Sequence[float],
    current_orientation: Sequence[float],
    hinge_axis: Sequence[float],
) -> float:
    """Return segment rotation about its neutral-frame hinge axis."""
    motion = quaternion_multiply(
        quaternion_conjugate(neutral_orientation), current_orientation
    )
    return signed_twist_degrees(motion, hinge_axis)


@dataclass(frozen=True)
class KneeEstimate:
    raw_angle_deg: float
    angle_deg: float
    angular_velocity_dps: float
    within_limits: bool
    reason: str
    orientation_measurement_valid: bool = True


class KneeEstimator:
    def __init__(
        self,
        cutoff_hz: float = 6.0,
        angle_limits: Tuple[float, float] = (-10.0, 150.0),
        velocity_limit_dps: float = 500.0,
        orientation_correction_hz: float = 1.0,
        orientation_jump_limit_deg: float = 20.0,
    ) -> None:
        self.cutoff_hz = max(0.1, float(cutoff_hz))
        self.angle_limits = angle_limits
        self.velocity_limit_dps = abs(float(velocity_limit_dps))
        self.orientation_correction_hz = max(0.05, float(orientation_correction_hz))
        self.orientation_jump_limit_deg = max(5.0, float(orientation_jump_limit_deg))
        self.neutral_relative: Optional[Quaternion] = None
        self.hinge_axis: Vector3 = (0.0, 1.0, 0.0)
        self.distal_hinge_axis: Vector3 = (0.0, 1.0, 0.0)
        self.direction = 1.0
        self.reset_filter()

    def reset_filter(self) -> None:
        self._last_time: Optional[float] = None
        self._last_raw: Optional[float] = None
        self._unwrapped: Optional[float] = None
        self._filtered: Optional[float] = None
        self._fused: Optional[float] = None
        self._filtered_rate = 0.0

    def set_axis(self, axis: Sequence[float]) -> None:
        self.hinge_axis = normalize_vector(axis)
        self.distal_hinge_axis = self.hinge_axis
        self.reset_filter()

    def set_hinge_axes(
        self, proximal_axis: Sequence[float], distal_axis: Sequence[float]
    ) -> None:
        self.hinge_axis = normalize_vector(proximal_axis)
        self.distal_hinge_axis = normalize_vector(distal_axis)
        self.reset_filter()

    def set_direction(self, direction: float) -> None:
        self.direction = 1.0 if direction >= 0.0 else -1.0
        self.reset_filter()

    def calibrate_zero(self, relative_samples: Sequence[Sequence[float]]) -> None:
        self.neutral_relative = quaternion_average(relative_samples)
        self.reset_filter()

    def motion_rotation_vector(self, relative: Sequence[float]) -> Vector3:
        if self.neutral_relative is None:
            raise RuntimeError("zero calibration is required")
        motion = quaternion_multiply(quaternion_conjugate(self.neutral_relative), relative)
        return quaternion_rotation_vector(motion)

    def calibrate_axis(self, relative_samples: Sequence[Sequence[float]]) -> Vector3:
        vectors = [self.motion_rotation_vector(sample) for sample in relative_samples]
        axis = principal_axis(vectors)
        self.set_axis(axis)
        return axis

    def update(
        self,
        thigh_quaternion: Sequence[float],
        shank_quaternion: Sequence[float],
        timestamp: float,
        thigh_gyro: Optional[Sequence[float]] = None,
        shank_gyro: Optional[Sequence[float]] = None,
    ) -> KneeEstimate:
        if self.neutral_relative is None:
            raise RuntimeError("zero calibration is required")
        relative = relative_quaternion(thigh_quaternion, shank_quaternion)
        motion = quaternion_multiply(quaternion_conjugate(self.neutral_relative), relative)
        wrapped = self.direction * signed_twist_degrees(motion, self.hinge_axis)

        if self._last_raw is None:
            self._unwrapped = wrapped
        else:
            self._unwrapped = float(self._unwrapped) + wrap_degrees(wrapped - self._last_raw)
        self._last_raw = wrapped

        if self._last_time is None or self._filtered is None:
            orientation_measurement_valid = True
            self._fused = float(self._unwrapped)
            self._filtered = float(self._fused)
            self._filtered_rate = 0.0
            self._last_time = float(timestamp)
        else:
            dt = float(timestamp) - self._last_time
            if dt <= 0.0 or dt > 0.5:
                orientation_measurement_valid = True
                self.reset_filter()
                self._last_raw = wrapped
                self._unwrapped = wrapped
                self._fused = wrapped
                self._filtered = wrapped
                self._last_time = float(timestamp)
            else:
                orientation_measurement_valid = True
                previous = self._filtered
                projected_rate = None
                if thigh_gyro is not None and shank_gyro is not None:
                    if (
                        len(thigh_gyro) == 3
                        and len(shank_gyro) == 3
                        and _finite((*thigh_gyro, *shank_gyro))
                    ):
                        projected_rate = self.direction * (
                            vector_dot(shank_gyro, self.distal_hinge_axis)
                            - vector_dot(thigh_gyro, self.hinge_axis)
                        )
                        if abs(projected_rate) > self.velocity_limit_dps * 2.0:
                            projected_rate = None

                if self._fused is None:
                    self._fused = float(self._unwrapped)
                if projected_rate is None:
                    self._fused = float(self._unwrapped)
                else:
                    predicted = self._fused + projected_rate * dt
                    innovation = float(self._unwrapped) - predicted
                    dynamic_jump_limit = max(
                        self.orientation_jump_limit_deg,
                        abs(projected_rate) * dt * 3.0 + 5.0,
                    )
                    if abs(innovation) > dynamic_jump_limit:
                        self._fused = predicted
                        orientation_measurement_valid = False
                    else:
                        correction_rc = 1.0 / (
                            2.0 * math.pi * self.orientation_correction_hz
                        )
                        correction_alpha = dt / (correction_rc + dt)
                        self._fused = predicted + correction_alpha * innovation
                rc = 1.0 / (2.0 * math.pi * self.cutoff_hz)
                alpha = dt / (rc + dt)
                self._filtered += alpha * (self._fused - self._filtered)
                raw_rate = (self._filtered - previous) / dt
                self._filtered_rate += alpha * (raw_rate - self._filtered_rate)
                self._last_time = float(timestamp)

        angle = float(self._filtered)
        rate = float(self._filtered_rate)
        reasons = []
        if not self.angle_limits[0] <= angle <= self.angle_limits[1]:
            reasons.append("角度超限")
        if abs(rate) > self.velocity_limit_dps:
            reasons.append("角速度超限")
        if not orientation_measurement_valid:
            reasons.append("姿态跳变已拒绝")
        return KneeEstimate(
            raw_angle_deg=float(self._unwrapped),
            angle_deg=angle,
            angular_velocity_dps=rate,
            within_limits=not reasons,
            reason="，".join(reasons) if reasons else "正常",
            orientation_measurement_valid=orientation_measurement_valid,
        )
