"""Dual shaft resistance: filtering, caps, reversal, release and supervision."""
import math
from dataclasses import replace
import pytest
from pathlib import Path
import yaml
from mech_assist_control.robot.contracts import JointLimit, JointObservation, RobotObservation
from mech_assist_control.robot.policies import ResistanceTrainingPolicy
from mech_assist_control.robot.supervisor import ExoskeletonSupervisor


def observation(t=1.0, sequence=1, velocities=(.8, -.8)):
    return RobotObservation(sequence, t, tuple(JointObservation(n, 0.0, v)
        for n, v in zip(('axis1', 'axis2'), velocities)), True, t, False,
        coordinates='motor_shaft')


def test_time_based_filter_matches_equal_elapsed_time():
    outputs = []
    for dt, steps in ((.01, 10), (.02, 5)):
        policy = ResistanceTrainingPolicy((1.0, 2.0), 0, .05)
        policy.start(observation())
        for _ in range(steps):
            result = policy.evaluate(observation(), dt)
        outputs.append(result.values)
    assert outputs[0] == pytest.approx(outputs[1])
    assert outputs[0][0] < 0 < outputs[0][1]
    assert outputs[0][1] == pytest.approx(-2 * outputs[0][0])


def test_rest_reversal_and_restart_clear_filter_memory():
    p = ResistanceTrainingPolicy()
    p.start(observation())
    assert p.evaluate(observation(), .1).values[0] < 0
    assert p.evaluate(observation(velocities=(-.8, .8)), .02).values == (0, 0)
    reversed_values = p.evaluate(observation(velocities=(-.8, .8)), .02).values
    assert reversed_values[0] > 0 > reversed_values[1]
    assert p.evaluate(observation(velocities=(.01, -.01)), .02).values == (0, 0)
    p.start(observation())
    assert p.evaluate(observation(), 0).values == (0, 0)


@pytest.mark.parametrize('caps', [(.2, .1), (2., 2.)])
def test_dual_saturated_training_reverses_without_assisting_or_latching(caps):
    p = ResistanceTrainingPolicy((10.0, 20.0), .02, .05)
    s = ExoskeletonSupervisor([JointLimit('axis1', -1, 1, 2, caps[0], 5),
                               JointLimit('axis2', -1, 1, 2, caps[1], 5)], p, run_limit=5)
    s.observe(observation()); assert s.arm(1)
    for i in range(1, 101):
        velocity = (.8, -.8) if i <= 30 else (-.8, .8) if i <= 60 else (0, 0)
        t = 1 + i * .02
        s.observe(observation(t, i+1, velocity))
        e = s.tick(t)
        assert e.valid
        assert abs(e.values[0]) <= caps[0] and abs(e.values[1]) <= caps[1]
        assert all(torque*v <= 1e-12 for torque, v in zip(e.values, velocity))
        if i == 30: assert e.values == pytest.approx((-caps[0], caps[1]))
        if i in (31, 61): assert e.values == (0, 0)
    assert s.state == 'active'
    assert s.previous == (0, 0)
    s.disarm(); assert not s.tick(3.02).valid


def test_training_stale_data_and_duration_stop():
    for stale in (True, False):
        s = ExoskeletonSupervisor([JointLimit(n, -1, 1, 2, .2, 5)
                                    for n in ('axis1', 'axis2')],
                                   ResistanceTrainingPolicy(), run_limit=.1)
        s.observe(observation()); assert s.arm(1)
        if stale:
            assert not s.tick(1.31).valid and s.state == 'fault'
        else:
            s.observe(observation(1.05, 2))
            assert s.tick(1.05).valid
            s.observe(observation(1.1, 3))
            assert not s.tick(1.1).valid and s.reason == 'run limit reached'


@pytest.mark.parametrize('gains,tau', [((), .05), ((math.nan,), .05), ((-1,), .05),
                                      ((1,), 0), ((1,), math.inf)])
def test_training_invalid_parameters(gains, tau):
    with pytest.raises(ValueError): ResistanceTrainingPolicy(gains, filter_tau=tau)


def test_eight_nm_saturation_with_slow_buildup_still_releases_on_reversal_and_rest():
    s = ExoskeletonSupervisor([JointLimit(n, -1, 1, 1, 8., .5)
                               for n in ('axis1', 'axis2')],
                              ResistanceTrainingPolicy((40., 40.), .02, .05), run_limit=30)
    s.observe(observation()); assert s.arm(1.)
    for i in range(1, 901):
        t = 1. + i*.02
        s.observe(observation(t, i+1))
        e = s.tick(t)
        assert e.valid and all(abs(v) <= 8. for v in e.values)
    assert e.values == pytest.approx((-8., 8.))
    s.observe(observation(19.02, 902, (-.8, .8)))
    assert s.tick(19.02).values == (0., 0.)
    s.observe(observation(19.04, 903, (0., 0.)))
    assert s.tick(19.04).values == (0., 0.)


def test_stm32_reference_recurrence_and_deadband_is_only_a_rest_gate():
    # Historical STM32 alpha=.15 at 1 kHz, T=-B*vf, independently computed.
    p = ResistanceTrainingPolicy((4., 2.), .02, -.001/math.log(.85))
    p.start(observation())
    filtered = [0., 0.]
    for velocity in ((.5, -.3),)*100:
        filtered = [.85*old+.15*v for old, v in zip(filtered, velocity)]
        actual = p.evaluate(observation(velocities=velocity), .001).values
        assert actual == pytest.approx(tuple(-b*v for b, v in zip((4., 2.), filtered)))
    assert p.evaluate(observation(velocities=(0., .01)), .001).values == (0., 0.)


def test_training_deceleration_unloads_without_buildup_slew_or_assistance():
    s = ExoskeletonSupervisor([JointLimit(n, -1, 1, 1, 8., 20.)
                               for n in ('axis1', 'axis2')],
                              ResistanceTrainingPolicy((4., 4.), .02, .0061531294))
    s.observe(observation()); assert s.arm(1.)
    for i in range(1, 101):
        t = 1.+i*.002
        s.observe(observation(t, i+1, (.5, -.5)))
        assert s.tick(t).valid
    assert s.previous == pytest.approx((-2., 2.), abs=1e-6)
    # Very slow buildup must not also make unloading slow.
    s.limits = tuple(replace(limit, max_slew=.01) for limit in s.limits)
    s.observe(observation(1.21, 102, (.03, -.03)))
    result = s.tick(1.21)
    expected = 4*(.03+(.5-.03)*math.exp(-.01/.0061531294))
    assert result.values == pytest.approx((-expected, expected), abs=1e-6)
    s.observe(observation(1.212, 103, (-.03, .03)))
    assert s.tick(1.212).values == (0., 0.)


def test_eight_nm_candidate_parameters_resist_a_short_stroke_without_changing_caps():
    path = Path(__file__).parents[1]/'config/dm4310_resistance_training_8nm_preview.yaml'
    config = yaml.safe_load(path.read_text())['exoskeleton_control']['ros__parameters']
    assert config['output_mode'] == 'preview' and not config['allow_motor_output']
    results = []
    for slew in ((.5, .5), config['max_slew']):
        policy = ResistanceTrainingPolicy(config['training_damping'],
                    config['velocity_deadband'], config['training_velocity_filter_tau_s'])
        s = ExoskeletonSupervisor([JointLimit(n, -1, 1, config['max_velocity'][i],
                         config['max_command'][i], slew[i])
                         for i, n in enumerate(('axis1', 'axis2'))], policy)
        s.observe(observation()); assert s.arm(1.)
        for i in range(1, 101):
            t = 1.+i*.002
            s.observe(observation(t, i+1, (.5, -.5)))
            e = s.tick(t)
            assert e.valid and all(v*w <= 0 for v, w in zip(e.values, (.5, -.5)))
            assert all(abs(v) <= 8. for v in e.values)
        results.append(abs(e.values[0]))
        s.observe(observation(1.202, 102, (0., 0.)))
        assert s.tick(1.202).values == (0., 0.)
    assert results[0] <= .100001  # Old slew masks a 200 ms hand-driven stroke.
    assert results[1] == pytest.approx(2., abs=1e-6)


def test_hysteresis_rejects_quantized_rest_noise_and_preserves_release():
    p = ResistanceTrainingPolicy((8., 8.), .02, .025, start_velocity=.06)
    p.start(observation())
    for velocity in ((.01221, -.03663), (.03663, -.01221)) * 100:
        assert p.evaluate(observation(velocities=velocity), .002).values == (0., 0.)
    assert p.evaluate(observation(velocities=(.08, -.08)), .002).values[0] < 0
    # Once moving, crossing back below the start threshold does not toggle off.
    hold = p.evaluate(observation(velocities=(.04, -.04)), .002).values
    assert hold[0] < 0 < hold[1]
    assert p.evaluate(observation(velocities=(-.08, .08)), .002).values == (0., 0.)
    reverse = p.evaluate(observation(velocities=(-.08, .08)), .002).values
    assert reverse[0] > 0 > reverse[1]
    assert p.evaluate(observation(velocities=(.01221, -.01221)), .002).values == (0., 0.)
    assert p.evaluate(observation(velocities=(.04, -.04)), .002).values == (0., 0.)
    p.start(observation())
    assert not any(p.moving)


@pytest.mark.parametrize('start', [.01, -.1, math.inf, math.nan])
def test_hysteresis_invalid_start_threshold_rejected(start):
    with pytest.raises(ValueError):
        ResistanceTrainingPolicy(start_velocity=start)


def test_smooth_preview_increases_sustained_low_speed_resistance_without_rest_torque():
    path = Path(__file__).parents[1]/'config/dm4310_resistance_training_smooth_preview.yaml'
    f = yaml.safe_load(path.read_text())['exoskeleton_control']['ros__parameters']
    assert f['output_mode'] == 'preview' and not f['allow_motor_output']
    p = ResistanceTrainingPolicy(f['training_damping'], f['velocity_deadband'],
                                f['training_velocity_filter_tau_s'], f['training_start_velocity_rad_s'])
    s = ExoskeletonSupervisor([JointLimit(n, -1, 1, f['max_velocity'][i],
                 f['max_command'][i], f['max_slew'][i]) for i,n in enumerate(('axis1', 'axis2'))], p)
    s.observe(observation()); assert s.arm(1.)
    for i in range(1, 501):
        t = 1.+i*.002
        s.observe(observation(t, i+1, (.1, -.1)))
        e = s.tick(t)
        assert e.valid and e.values[0] <= 0 <= e.values[1]
        assert all(abs(v) <= 8 for v in e.values)
    assert e.values == pytest.approx((-.8, .8))
    s.observe(observation(2.002, 502, (0., 0.)))
    assert s.tick(2.002).values == (0., 0.)
