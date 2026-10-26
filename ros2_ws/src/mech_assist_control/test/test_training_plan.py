"""Pure upper plan -> supervision -> route; no ROS or device dependencies."""
from dataclasses import FrozenInstanceError, replace

import pytest

from mech_assist_control.robot.contracts import (
    CommandMode, ImpedancePlan, JointLimit, JointObservation, RobotObservation)
from mech_assist_control.robot.policies import ImpedanceTrainingPolicy
from mech_assist_control.robot.routing import CommandRouter, ControllerRoute
from mech_assist_control.robot.supervisor import ExoskeletonSupervisor


def observation(t=1., seq=1, positions=(.2, -.3), velocities=(0., 0.)):
    return RobotObservation(seq, t, tuple(JointObservation(n, q, v) for n, q, v in
        zip(('left', 'right'), positions, velocities)), True, t, False,
        coordinates='motor_shaft')


def active():
    policy = ImpedanceTrainingPolicy(10., .25, (2., 1.))
    supervisor = ExoskeletonSupervisor([
        JointLimit(n, -1., 1., 2., 2., 10.) for n in ('left', 'right')], policy)
    supervisor.observe(observation())
    assert supervisor.arm(1.)
    return policy, supervisor


def router(scale=1., caps=(2., 1.)):
    return CommandRouter(('left', 'right'), CommandMode.EFFORT, [
        ControllerRoute(n, CommandMode.EFFORT, '/'+n+'/training_command', scale, cap)
        for n, cap in zip(('left', 'right'), caps)])


def test_policy_owns_immutable_reference_and_parameters_without_generating_pd():
    policy, supervisor = active()
    plan = policy.plan
    assert plan.reference == (.2, -.3)
    with pytest.raises(FrozenInstanceError):
        plan.stiffness = 999.
    supervisor.observe(observation(1.02, 2, (.3, -.4), (.2, -.2)))
    envelope = supervisor.tick(1.02)
    assert envelope.valid and envelope.training_plan is plan
    assert envelope.values == (0., 0.)  # No Python PD even with nonzero errors.
    assert router().route_training(envelope, 1.03) == (
        ('/left/training_command', (.2, 10., .25, 2.)),
        ('/right/training_command', (-.3, 10., .25, 1.)))
    supervisor.disarm()
    assert supervisor.tick(1.04).training_plan is None


@pytest.mark.parametrize('change', [
    {'reference': [0., 0.]}, {'reference': (float('nan'), 0.)},
    {'reference': (0.,)}, {'names': ('left', 'left')},
    {'torque_caps': (2., -1.)}, {'stiffness': 0.}, {'damping': -1.},
])
def test_invalid_or_mutable_plans_rejected(change):
    plan = ImpedancePlan(('left', 'right'), (0., 0.), 10., .25, (2., 1.))
    with pytest.raises(ValueError):
        replace(plan, **change)


def test_policy_cannot_capture_moving_or_uncalibrated_anatomical_reference():
    policy = ImpedanceTrainingPolicy()
    with pytest.raises(ValueError):
        policy.evaluate(observation(), .02)
    for sample in (observation(velocities=(.051, 0.)),
                   replace(observation(), coordinates='calibrated_joint')):
        with pytest.raises(ValueError):
            policy.start(sample)
        assert policy.plan is None


@pytest.mark.parametrize('change', [
    {'reference': (.25, -.3)}, {'stiffness': 11.}, {'torque_caps': (2.01, 1.)},
    {'names': ('right', 'left')},
])
def test_supervision_rejects_live_plan_changes_before_any_route(change):
    policy, supervisor = active()
    assert supervisor.tick(1.01).valid
    policy.plan = replace(policy.plan, **change)
    envelope = supervisor.tick(1.02)
    assert not envelope.valid and supervisor.state == 'fault'
    assert envelope.training_plan is None


def test_router_rejects_replay_expiry_future_scalar_bypass_and_partial_dual_admission():
    _, supervisor = active()
    envelope = supervisor.tick(1.02)
    route = router()
    route.route_training(envelope, 1.03)
    with pytest.raises(ValueError):
        route.route_training(envelope, 1.04)
    for now in (.99, envelope.expires_at):
        with pytest.raises(ValueError):
            router().route_training(envelope, now)
    with pytest.raises(ValueError):
        router().route(envelope, 1.03)
    with pytest.raises(ValueError):
        router(scale=-1.).route_training(envelope, 1.03)
    route = router(caps=(2., .9))
    with pytest.raises(ValueError):
        route.route_training(envelope, 1.03)
    assert route.last_sequence == -1  # No first-axis admission before second-axis failure.


def test_fault_clears_plan_and_recovery_does_not_reauthorize_without_explicit_arm():
    _, supervisor = active()
    assert supervisor.tick(1.02).training_plan is not None
    supervisor.observe(replace(observation(1.04, 2), healthy=False))
    assert not supervisor.tick(1.04).valid
    supervisor.observe(observation(1.06, 3))
    assert not supervisor.tick(1.06).valid
    assert supervisor.reset()
    assert not supervisor.arm(1.06)
    supervisor.observe(observation(1.08, 4, (.4, -.2)))
    assert supervisor.arm(1.08)
    assert supervisor.tick(1.09).training_plan.reference == (.4, -.2)
