import math
import pytest
from mech_assist_control.desk_follow import DeskFollow
from mech_knee_kinematics.dual_imu_knee import quaternion_from_euler_degrees as q


def feed(c, now, degrees=0, position=0, velocity=0):
    c.orientation(q(0, degrees, 0), now)
    c.healthy(True, now)
    c.motor_feedback(position, velocity, now)


def test_requires_fresh_inputs_and_stationary_feedback():
    c = DeskFollow(motor=True)
    assert not c.arm(0)
    feed(c, 1, velocity=0.2)
    assert not c.arm(1)
    feed(c, 1)
    assert c.arm(1)
    assert c.tick(1) == 0


def test_deadband_speed_slew_and_nonanatomical_relative_zero():
    c = DeskFollow()
    feed(c, 1, 20)
    assert c.arm(1)
    feed(c, 1.01, 24)
    assert c.tick(1.01) == 0
    for i in range(2, 202):
        now = 1+i/100
        feed(c, now, 49)
        previous = c.target
        target = c.tick(now)
        assert abs(target) <= 0.3
        assert abs(target-previous) <= 0.003001
    assert c.target > 0.29


@pytest.mark.parametrize('fault', ['stale', 'unhealthy', 'angle', 'speed', 'displacement', 'timer'])
def test_faults_latch_zero_and_cannot_rearm(fault):
    c = DeskFollow(motor=True)
    feed(c, 1)
    assert c.arm(1)
    now = 1.01
    if fault == 'stale':
        now = 1.4
    else:
        feed(c, now, 31 if fault == 'angle' else 0,
             0.7 if fault == 'displacement' else 0, 1.1 if fault == 'speed' else 0)
    if fault == 'unhealthy':
        c.healthy(False, now)
    if fault == 'timer':
        now = 1.2
        feed(c, now)
    assert c.tick(now) == 0
    assert c.state == 'fault'
    feed(c, now)
    assert not c.arm(now)


def test_expiry_and_invalid_quaternion():
    c = DeskFollow()
    feed(c, 0)
    assert c.arm(0)
    for i in range(1, 501):
        now = i/100
        feed(c, now, 10)
        c.tick(now)
    assert c.state == 'complete' and c.target == 0
    c = DeskFollow()
    feed(c, 0)
    c.arm(0)
    c.orientation((float('nan'), 0, 0, 0), 0.01)
    assert c.state == 'fault'
