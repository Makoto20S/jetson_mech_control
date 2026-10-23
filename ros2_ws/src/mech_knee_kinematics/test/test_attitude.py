"""Analytic poses, rigid co-motion, sign invariance and zero lifecycle."""
import math
import pytest
from mech_knee_kinematics.attitude import SegmentAttitudes, euler_zyx
from mech_knee_kinematics.dual_imu_knee import quaternion_from_euler_degrees as q


def same_rotation(a, b):
    assert abs(sum(x*y for x, y in zip(a, b))) == pytest.approx(1.0)


def test_independent_segment_zero_and_relative_motion():
    solver = SegmentAttitudes()
    solver.calibrate_zero([(q(0, 20, 0), q(0, 30, 0))] * 20)
    result = solver.update(q(0, 40, 0), q(0, 70, 0))
    same_rotation(result.thigh_from_zero, q(0, 20, 0))
    same_rotation(result.shank_from_zero, q(0, 40, 0))
    same_rotation(result.relative, q(0, 30, 0))
    solver.reset()
    assert solver.update(q(0, 0, 0), q(0, 0, 0)).thigh_from_zero is None


def test_common_rotation_and_quaternion_sign():
    solver = SegmentAttitudes()
    pose = q(30, 70, -140)
    result = solver.update(pose, tuple(-v for v in pose))
    same_rotation(result.relative, q(0, 0, 0))
    assert euler_zyx(pose) == pytest.approx(tuple(map(math.radians, (30, 70, -140))))


@pytest.mark.parametrize('pose', [(0, 90, 0), (0, -90, 0), (0, 0, 179)])
def test_euler_display_remains_finite_at_singularities(pose):
    assert all(math.isfinite(v) for v in euler_zyx(q(*pose)))


def test_invalid_zero_does_not_install_partial_calibration():
    solver = SegmentAttitudes()
    with pytest.raises(ValueError):
        solver.calibrate_zero([(q(0, 0, 0), (0, 0, 0, 0))])
    assert solver.neutral_thigh is None
    with pytest.raises(ValueError):
        solver.update((float('nan'), 0, 0, 0), q(0, 0, 0))
