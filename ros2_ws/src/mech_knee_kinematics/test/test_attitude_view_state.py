import math
import pytest
from mech_knee_kinematics.attitude_view_state import AttitudeViewState
from mech_knee_kinematics.dual_imu_knee import quaternion_from_euler_degrees as q


def ready_state():
    s = AttitudeViewState()
    s.segment('thigh', 100, q(0, 20, 0), 1)
    s.segment('shank', 100, q(0, 60, 0), 1)
    s.update_joint(100, 40, 5, 1)
    s.diagnostic = (0, 'calibrated', 1)
    return s


def test_independent_segment_geometry_and_joint_numbers():
    s = ready_state()
    assert s.ready(1.1)[0]
    a, b = s.directions()
    assert a[0] == pytest.approx(-math.sin(math.radians(20)))
    assert b[0] == pytest.approx(-math.sin(math.radians(60)))
    assert s.joint[1:3] == (40, 5)


def test_diagnostics_stale_pose_and_partial_pair_fail_closed():
    s = ready_state()
    assert not s.ready(1.31)[0]
    s = ready_state()
    s.diagnostic = (1, 'calibrating axis', 1.1)
    assert not s.ready(1.1)[0]
    s = ready_state()
    s.segment('shank', 101, q(0, 70, 0), 1.1)
    assert s.ready(1.1)[0]  # Retain the previous complete pair, never mix stamps.
    assert s.segments['shank'][0] == s.segments['thigh'][0] == 100
    assert not s.ready(1.31)[0]
    s = ready_state()
    s.diagnostic = None
    assert not s.ready(1.1)[0]


def test_invalid_pose_rejected_and_history_not_filled_by_repaint_or_fault():
    s = ready_state()
    s.record_history(1.1)
    s.record_history(1.2)
    assert len(s.history) == 1
    s.segment('shank', 101, (0, 0, 0, 0), 1.2)
    s.record_history(1.2)
    assert not s.ready(1.2)[0]
    assert len(s.history) == 1
    s.update_joint(101, float('nan'), 0, 1.2)
    assert s.joint is None


def test_out_of_order_does_not_replace_newer_pose():
    s = ready_state()
    s.segment('thigh', 99, q(0, 90, 0), 1.1)
    assert s.segments['thigh'][0] == 100


def test_raw_sensor_mode_displays_without_anatomical_calibration_or_joint():
    s = AttitudeViewState(raw=True)
    s.segment('thigh', 100, q(0, 10, 0), 1)
    s.segment('shank', 100, q(0, 30, 0), 1)
    s.rpy = {'thigh': ((0, 10, 0), 1), 'shank': ((0, 30, 0), 1)}
    s.diagnostic = (0, 'healthy', 1)
    assert s.ready(1.1)[0]
    assert s.joint is None
    s.record_history(1.1)
    assert s.history[-1][1:] == (10, 30)
    assert not s.ready(1.4)[0]


def test_pair_commit_is_atomic_and_pending_cache_bounded():
    s = ready_state()
    s.segment('shank', 102, q(0, 70, 0), 1.1)
    assert s.segments['shank'][0] == 100
    s.segment('thigh', 102, q(0, 30, 0), 1.2)
    assert s.segments['thigh'][0] == s.segments['shank'][0] == 102
    for stamp in range(103, 130):
        s.segment('shank', stamp, q(0, 70, 0), 1.2)
    assert len(s.pending) <= 8
