"""Whole-robot behavior: multi-joint gating, policy failure and command validity."""
from dataclasses import replace
import math
import pytest
from mech_assist_control.robot.contracts import (
    JointLimit, JointObservation, RobotObservation, CommandMode, PolicyTarget)
from mech_assist_control.robot.policies import DampingPolicy, HoldVelocityPolicy, make_policy
from mech_assist_control.robot.supervisor import ExoskeletonSupervisor
from mech_assist_control.robot.routing import CommandRouter, ControllerRoute


def observation(t=1.0, seq=1, velocities=(0.4, -0.2)):
    return RobotObservation(seq, t, tuple(JointObservation(n, 0.5, v)
                            for n, v in zip(('left_knee', 'right_knee'), velocities)),
                            True, t, True)


def supervisor(policy=None):
    return ExoskeletonSupervisor([JointLimit(n, -0.2, 2.4, 2, .3, .5)
                                  for n in ('left_knee', 'right_knee')],
                                 policy or DampingPolicy())


def active(policy=None):
    s = supervisor(policy)
    s.observe(observation())
    assert s.arm(1)
    return s


def test_explicit_arm_multijoint_bounded_damping():
    s = supervisor()
    assert not s.tick(1).valid
    assert not s.arm(1)
    s.observe(observation())
    assert s.arm(1)
    for i in range(1, 10):
        t = 1+i*.02
        s.observe(observation(t, i+1))
        e = s.tick(t)
        assert e.valid and e.names == ('left_knee', 'right_knee')
        assert all(v*j.velocity <= 0 for v, j in zip(e.values, s.observation.joints))
        assert abs(e.values[0]) <= .5*(t-1)+1e-12
    s.disarm()
    assert not s.tick(1.2).valid


@pytest.mark.parametrize('change', [
    {'healthy': False}, {'calibrated': False}, {'health_received_at': 0},
    {'joints': ()}, {'joints': (JointObservation('left_knee', math.nan, 0),)},
    {'joints': (JointObservation('right_knee', .5, 0), JointObservation('left_knee', .5, 0))},
    {'joints': (JointObservation('left_knee', 3, 0), JointObservation('right_knee', .5, 0))},
    {'joints': (JointObservation('left_knee', .5, 3), JointObservation('right_knee', .5, 0))},
])
def test_bad_state_cannot_arm(change):
    s = supervisor(); s.observe(replace(observation(), **change))
    assert not s.arm(1)


def test_stale_and_replayed_state_latch_until_explicit_reset_and_new_data():
    s = active()
    assert not s.tick(1.31).valid
    assert s.state == 'fault'
    s.observe(observation(1.32, 2))
    assert not s.arm(1.32)
    assert s.reset() and not s.arm(1.32)
    s.observe(observation(1.34, 3)); assert s.arm(1.34)
    s.observe(observation(1.34, 3))
    assert s.state == 'fault'


def test_health_stale_with_fresh_joints_and_tick_stall():
    s = active()
    for i in range(1, 18):
        t = 1+i*.02
        s.observe(replace(observation(t, i+1), health_received_at=1.0))
        s.tick(t)
    assert s.state == 'fault'
    s = active(); assert not s.tick(1.101).valid
    assert s.reason == 'supervisor tick gap'


def test_newer_health_is_compared_to_execution_time_without_renewing_joint_age():
    s = active()
    sample = replace(observation(1.04, 2), health_received_at=1.06)
    s.observe(sample, now=1.07)
    assert s.tick(1.07).valid
    assert s.observation.received_at == 1.04
    s.observe(replace(sample, sequence=3, received_at=1.08, health_received_at=1.20), now=1.09)
    assert s.state == 'fault'  # Truly future diagnostics still fail.


def test_fault_on_direction_reversal_cannot_make_damping_assist():
    s = active()
    for i in range(1, 5):
        s.tick(1+i*.02)
    s.observe(observation(1.10, 2, (-.4, .2)))
    e = s.tick(1.10)
    assert not e.valid and e.values == (0, 0)


def test_hold_starts_at_actual_pose_and_does_not_change_mode():
    s = active(HoldVelocityPolicy())
    assert s.tick(1.02).values == (0, 0)
    s.observe(replace(observation(1.04, 2), joints=(
        JointObservation('left_knee', .4, 0), JointObservation('right_knee', .6, 0))))
    e = s.tick(1.04)
    assert e.mode == CommandMode.VELOCITY and e.values[0] > 0 and e.values[1] < 0


@pytest.mark.parametrize('kind', ['nan', 'wrong_mode', 'wrong_size', 'exception'])
def test_untrusted_policy_output_fails_closed(kind):
    class Broken(DampingPolicy):
        def evaluate(self, o, dt):
            if kind == 'exception': raise RuntimeError('model inference failed')
            return PolicyTarget(CommandMode.VELOCITY if kind == 'wrong_mode' else self.mode,
                                (1,) if kind == 'wrong_size' else (math.nan, 0) if kind == 'nan' else (0, 0))
    s = active(Broken()); assert not s.tick(1.02).valid and s.state == 'fault'


def test_contact_dependent_algorithm_cannot_treat_missing_insole_as_stance():
    class ContactPolicy(DampingPolicy):
        requires_contact = True
    s = supervisor(ContactPolicy()); s.observe(observation())
    assert not s.arm(1) and s.reason == 'verified foot contact required'


def test_deadline_disarms_without_auto_rearm():
    s = active(); s.run_limit = .08
    for i in range(1, 6):
        t = 1+i*.02; s.observe(observation(t, i+1)); e = s.tick(t)
    assert not e.valid and s.state == 'standby'
    assert s.reset()
    assert not s.arm(1.1)


def test_mapping_virtual_work_replay_expiry_and_output_caps():
    s = active(); e = s.tick(1.02)
    router = CommandRouter(s.names, e.mode, [
        ControllerRoute('left_knee', e.mode, '/left/target_effort', -2, .3),
        ControllerRoute('right_knee', e.mode, '/right/target_effort', .5, .3)])
    mapped = router.route(e, 1.02)
    assert mapped[0][1] == -2*e.values[0] and mapped[1][1] == .5*e.values[1]
    with pytest.raises(ValueError): router.route(e, 1.02)
    with pytest.raises(ValueError): router.route(replace(e, sequence=20), e.expires_at)
    with pytest.raises(ValueError): router.route(replace(e, sequence=21, values=(10, 10)), 1.02)
    with pytest.raises(ValueError): router.route(replace(e, sequence=22, valid=False), 1.02)


def test_velocity_mapping_and_conflicting_routes():
    s = active(HoldVelocityPolicy()); e = replace(s.tick(1.02), values=(.1, -.1))
    routes = [ControllerRoute(n, e.mode, '/'+n+'/target_velocity', -2, .3) for n in s.names]
    assert CommandRouter(s.names, e.mode, routes).route(e, 1.02)[0][1] == -.05
    with pytest.raises(ValueError): CommandRouter(s.names, e.mode, [routes[0], routes[0]])
    with pytest.raises(ValueError): make_policy('adaptive_gait_not_implemented')


def test_whole_robot_faults_when_one_of_six_joint_states_is_invalid():
    names = tuple('joint_'+str(i) for i in range(6))
    s = ExoskeletonSupervisor([JointLimit(n, -1, 1, 2, .3, .5) for n in names], DampingPolicy())
    o = RobotObservation(1, 1, tuple(JointObservation(n, 0, .1) for n in names), True, 1, True)
    s.observe(o); assert s.arm(1)
    assert len(s.tick(1.02).values) == 6
    broken = list(o.joints); broken[4] = replace(broken[4], position=2)
    s.observe(replace(o, sequence=2, received_at=1.04, joints=tuple(broken)))
    e = s.tick(1.04)
    assert not e.valid and e.values == (0,)*6


def test_contact_data_requires_quality_and_freshness_but_not_fabricated_stance():
    from mech_assist_control.robot.contracts import ContactObservation
    class ContactPolicy(DampingPolicy):
        requires_contact = True
    contacts = tuple(ContactObservation(n, False, 1, True) for n in ('left_foot', 'right_foot'))
    s = supervisor(ContactPolicy()); s.observe(replace(observation(), contacts=contacts))
    assert s.arm(1)  # Verified no-contact is legitimate data, not missing data.
    for bad in (contacts[:1], (replace(contacts[0], received_at=0), contacts[1]),
                (replace(contacts[0], verified=False), contacts[1])):
        s = supervisor(ContactPolicy()); s.observe(replace(observation(), contacts=bad))
        assert not s.arm(1)


def test_resistance_deadband_releases_residual_torque_immediately():
    s = active(DampingPolicy(.5, .02))
    for i in range(1, 5):
        s.observe(observation(1+i*.02, i+1))
        assert s.tick(1+i*.02).valid
    assert s.previous[0] < 0
    s.observe(observation(1.1, 6, (.01, -.01)))
    e = s.tick(1.1)
    assert e.valid and e.values == (0.0, 0.0)


@pytest.mark.parametrize('deadband', [-.01, math.nan, math.inf])
def test_invalid_damping_deadband_rejected(deadband):
    with pytest.raises(ValueError):
        DampingPolicy(.5, deadband)


def test_shaft_resistance_requires_no_fabricated_anatomical_calibration():
    o = replace(observation(), coordinates='motor_shaft', calibrated=False)
    s = supervisor(); s.observe(o)
    assert s.arm(1)
    s = supervisor(HoldVelocityPolicy()); s.observe(o)
    assert not s.arm(1)
    s = supervisor(); s.observe(replace(o, calibrated=True))
    assert not s.arm(1)


def test_assist_zero_direction_taper_and_cutoff():
    from mech_assist_control.robot.policies import MotionAssistPolicy
    p = MotionAssistPolicy()
    for velocity in (0, .02, -.02):
        assert p.evaluate(observation(velocities=(velocity, velocity)), .02).values == (0, 0)
    forward = p.evaluate(observation(velocities=(.2, -.2)), .02).values
    assert forward[0] > 0 and forward[1] < 0
    assert p.evaluate(observation(velocities=(.49, .49)), .02).values[0] < forward[0]
    with pytest.raises(ValueError):
        p.evaluate(observation(velocities=(.5, 0)), .02)


def test_assist_reversal_zero_deadband_budget_and_reset():
    from mech_assist_control.robot.policies import MotionAssistPolicy
    s = active(MotionAssistPolicy(work_budget=.0005))
    for i in range(1, 20):
        t=1+i*.02
        s.observe(observation(t, i+1, (.2, -.2)))
        e=s.tick(t)
        if not e.valid: break
        assert all(tau*j.velocity >= 0 for tau,j in zip(e.values,s.observation.joints))
    assert s.state == 'fault' and not e.valid
    assert s.reset()
    s.observe(observation(2, 100, (.2, -.2)))
    assert s.arm(2) and s.command_work == 0
    s.tick(2.02)
    s.observe(observation(2.04,101,(-.2,.2)))
    e=s.tick(2.04)
    assert e.valid and e.values == (0,0)
    s.observe(observation(2.06,102,(0,0)))
    assert s.tick(2.06).values == (0,0)


@pytest.mark.parametrize('kwargs', [
    {'gain': -1}, {'deadband': -.1}, {'taper_speed': .02},
    {'cutoff_speed': .3}, {'work_budget': 0}, {'work_budget': math.nan},
])
def test_bad_assist_parameters_rejected(kwargs):
    from mech_assist_control.robot.policies import MotionAssistPolicy
    with pytest.raises(ValueError):
        MotionAssistPolicy(**kwargs)


def test_impedance_training_permission_still_checks_shaft_health_bounds_and_run_budget():
    policy = make_policy('impedance_training')
    s = supervisor(policy)
    o = replace(observation(velocities=(0., 0.)), calibrated=False, coordinates='motor_shaft')
    s.observe(o)
    assert s.arm(1.)
    e = s.tick(1.02)
    assert e.valid and e.policy == 'impedance_training'
    assert e.values == (0., 0.)  # Placeholder only; PD belongs to C++.
    s.observe(replace(o, sequence=2, received_at=1.04, health_received_at=1.04,
                      joints=(JointObservation('left_knee', 3., 0.), o.joints[1])))
    assert not s.tick(1.04).valid
    assert s.state == 'fault'
