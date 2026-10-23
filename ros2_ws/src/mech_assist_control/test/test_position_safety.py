"""Behavioral boundary tests for every policy and direct supervisor caller."""
from dataclasses import replace
import math
import pytest
from mech_assist_control.robot.contracts import JointLimit, JointObservation, RobotObservation
from mech_assist_control.robot.safety import PositionGuard
from mech_assist_control.robot.supervisor import ExoskeletonSupervisor
from mech_assist_control.robot.policies import HoldVelocityPolicy, DampingPolicy


def limits():
    return [JointLimit('axis', -1, 1, 2, .2, 1)]


def obs(p, v=0, t=1, sequence=1):
    return RobotObservation(sequence,t,(JointObservation('axis',p,v),),True,t,True)


@pytest.mark.parametrize('p,outward,inward', [(.95,.2,-.2),(-.95,-.2,.2)])
def test_directional_soft_margin_and_inward_recovery(p,outward,inward):
    g=PositionGuard(limits(),soft_margin=.1)
    joints=obs(p).joints
    assert abs(g.constrain((outward,),joints)[0]) == pytest.approx(.1)
    assert g.constrain((inward,),joints) == (inward,)


@pytest.mark.parametrize('p,command', [(1,.2),(-1,-.2)])
def test_at_boundary_outward_command_removed_even_after_slew(p,command):
    g=PositionGuard(limits())
    assert g.constrain((command,),obs(p).joints) == (0,)
    assert g.error(obs(p).joints) == ''
    assert g.error(obs(p+math.copysign(.001,p)).joints)


def test_relative_limit_checked_without_ros_and_reset_recaptures():
    s=ExoskeletonSupervisor(limits(),DampingPolicy(),max_excursion=.3)
    s.observe(obs(.2));assert s.arm(1)
    s.observe(obs(.51,t=1.02,sequence=2))
    assert s.state == 'fault' and 'excursion' in s.reason
    assert not s.tick(1.02).valid
    assert s.reset()
    s.observe(obs(-.5,t=2,sequence=3));assert s.arm(2)
    assert s.position_guard.bounds(0) == pytest.approx((-.8,-.2))
    s.disarm()
    s.observe(obs(.5,t=3,sequence=4));assert s.arm(3)
    assert s.position_guard.bounds(0) == pytest.approx((.2,.8))


def test_relative_limit_softens_before_absolute_limit():
    g=PositionGuard(limits(),soft_margin=.1,max_excursion=.3)
    g.arm(obs(0).joints)
    assert g.constrain((.2,),obs(.25).joints)[0] == pytest.approx(.1)
    assert g.constrain((-.2,),obs(.25).joints)[0] == -.2


def test_guard_applies_after_slew_for_velocity_policy():
    s=ExoskeletonSupervisor(limits(),HoldVelocityPolicy())
    s.observe(obs(.9));assert s.arm(1)
    # First develop an outward target, then move to the upper hard boundary.
    s.observe(obs(.8,t=1.02,sequence=2));assert s.tick(1.02).values[0] > 0
    s.observe(obs(1,t=1.04,sequence=3))
    assert s.tick(1.04).values == (0,)


@pytest.mark.parametrize('kwargs', [
    {'soft_margin':0}, {'soft_margin':math.nan}, {'soft_margin':1},
    {'max_excursion':0}, {'max_excursion':math.inf},
])
def test_invalid_guard_configuration_rejected(kwargs):
    with pytest.raises(ValueError):PositionGuard(limits(),**kwargs)
