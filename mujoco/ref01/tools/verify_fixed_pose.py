"""Run the two fixed-pose perturbation experiments with native MuJoCo (no devices)."""
import argparse
import csv
import json
from pathlib import Path
import sys

import mujoco
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from fixed_pose_impedance import FixedPoseImpedance
from ref01_dynamics_api import REF01Dynamics

REFERENCE = np.deg2rad([0., 20.])
STIFFNESS = [2.4, 2.]
DAMPING = [.18, .15]
LIMIT = [.2, .2]  # Simulation envelope in N m, not a hardware setting.
CONTROL_DT = .005
PULSE_START, PULSE_END, DURATION = .4, 1., 2.5
PULSE_NM = .09


def run_case(model, controller, axis):
    state = model.reset(REFERENCE)
    rows = []
    steps = round(CONTROL_DT / model.model.opt.timestep)
    if steps < 1 or not np.isclose(steps * model.model.opt.timestep, CONTROL_DT, atol=1e-12, rtol=0):
        raise ValueError('Controller period must contain an integer number of physics ticks')
    for tick in range(round(DURATION / CONTROL_DT)):
        t = tick * CONTROL_DT
        q, dq = state['q'], state['dq']
        command = controller.command(q, dq)
        external = np.zeros(2)
        if PULSE_START <= t < PULSE_END:
            external[axis] = PULSE_NM
        motor = command.motor104_105_nm
        state = model.step(command.alpha_nm, steps=steps, external_torque=external)
        # State sampled at t; commands and external torque held on [t,t+dt).
        rows.append(dict(case=('psi','beta')[axis], time_s=t,
                         psi_rad=q[0], beta_rad=q[1], psi_ref_rad=REFERENCE[0], beta_ref_rad=REFERENCE[1],
                         dpsi_rad_s=dq[0], dbeta_rad_s=dq[1], tau104_nm=motor[0], tau105_nm=motor[1],
                         external_psi_nm=external[0], external_beta_nm=external[1],
                         feedback_psi_nm=command.feedback_q_nm[0], feedback_beta_nm=command.feedback_q_nm[1],
                         gravity_psi_nm=command.gravity_q_nm[0], gravity_beta_nm=command.gravity_q_nm[1],
                         scale=command.scale, closure_end_m=state['max_closure_m'],
                         applied104_end_nm=state['tau_applied'][1], applied105_end_nm=state['tau_applied'][0]))
    return rows, state


def assess(rows, state, axis):
    def cols(*names):
        return np.array([[r[n] for n in names] for r in rows])

    t = cols('time_s')[:,0]
    q = cols('psi_rad','beta_rad')
    err = np.rad2deg(q - REFERENCE)
    speed = np.rad2deg(cols('dpsi_rad_s','dbeta_rad_s'))
    motor = cols('tau104_nm','tau105_nm')
    applied = cols('applied104_end_nm','applied105_end_nm')
    during = (t >= PULSE_START) & (t < PULSE_END)
    tail = t >= DURATION - .4
    # Settling requires BOTH coordinates to stay inside both bands until the end.
    good = (np.max(np.abs(err), axis=1) <= .05) & (np.max(np.abs(speed), axis=1) <= .25)
    settled = np.flatnonzero((t >= PULSE_END) & np.logical_and.accumulate(good[::-1])[::-1])
    finite = all(np.isfinite([v for k,v in r.items() if k != 'case']).all() for r in rows)
    peak = float(np.max(err[:,axis]))
    baseline = float(np.max(np.abs(err[t<PULSE_START])))
    tail_error = float(np.max(np.abs(err[tail])))
    closure = float(np.max(cols('closure_end_m')))
    active_steady_deg = float(np.mean(err[(t>=PULSE_END-.05) & during,axis]))
    expected_static_deg = float(np.rad2deg(PULSE_NM / STIFFNESS[axis]))
    checks = dict(finite=finite, gravity_hold=baseline < .02,
                  positive_deflection=peak > .5,
                  elastic_deflection=abs(active_steady_deg-expected_static_deg) < .1,
                  restores_both_axes=bool(len(settled) and t[settled[0]] <= DURATION-.4 and good[tail].all()),
                  last_step=bool(np.max(np.abs(np.rad2deg(state['q']-REFERENCE))) < .05 and
                                 np.max(np.abs(np.rad2deg(state['dq']))) < .25),
                  closure=closure < 1e-4, solver_warnings=int(state['warnings']) == 0,
                  unsaturated=bool(np.min(cols('scale')) == 1.),
                  actuator_match=bool(np.max(np.abs(motor-applied)) < 1e-10))
    return dict(passed=all(checks.values()), checks=checks, peak_deg=peak,
                cross_peak_deg=float(np.max(np.abs(err[:,1-axis]))),
                baseline_max_error_deg=baseline, steady_deflection_deg=active_steady_deg,
                theoretical_static_deg=expected_static_deg, tail_max_error_deg=tail_error,
                settling_after_release_s=float(t[settled[0]]-PULSE_END) if len(settled) else None,
                motor_min_nm=motor.min(axis=0).tolist(), motor_max_nm=motor.max(axis=0).tolist(),
                max_closure_m=closure, warnings=int(state['warnings']))


def experiment():
    model = REF01Dynamics()
    controller = FixedPoseImpedance(model, REFERENCE, STIFFNESS, DAMPING, LIMIT)
    results, traces = {}, []
    for i, name in enumerate(('psi','beta')):
        rows, state = run_case(model, controller, i)
        results[name] = assess(rows, state, i)
        traces.extend(rows)

    # Exercise paired saturation separately from the two nominal response plots.
    q = np.deg2rad([20.,32.])
    command = controller.command(q,[0.,0.])
    achieved = model.dynamics_terms(q)['A'].T @ command.alpha_nm
    expected = command.scale * (command.gravity_q_nm + command.feedback_q_nm)
    allocation_ok = bool(0 < command.scale < 1 and np.max(np.abs(command.alpha_nm)) <= .2+1e-12 and
                         np.allclose(achieved, expected, atol=1e-12, rtol=0))
    return dict(passed=allocation_ok and all(v['passed'] for v in results.values()),
                model_sha256=model.metadata['model_sha256'], mujoco=mujoco.__version__, numpy=np.__version__,
                reference_deg=np.rad2deg(REFERENCE).tolist(), stiffness_nm_rad=STIFFNESS,
                damping_nm_s_rad=DAMPING, limit_alpha_nm=LIMIT, control_period_s=CONTROL_DT,
                physics_period_s=model.model.opt.timestep, pulse_nm=PULSE_NM,
                pulse_interval_s=[PULSE_START,PULSE_END], duration_s=DURATION,
                paired_saturation_check=allocation_ok, cases=results), traces


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, help='Directory for new traces.csv and summary.json')
    args = parser.parse_args()
    summary, rows = experiment()
    serialized = json.dumps(summary, indent=2, allow_nan=False)
    if args.output:
        args.output.mkdir(exist_ok=True, parents=True)
        csv_path = args.output / 'traces.csv'
        summary_path = args.output / 'summary.json'
        if csv_path.exists() or summary_path.exists():
            parser.error('Use a fresh output directory')
        with csv_path.open('x', newline='', encoding='utf-8') as stream:
            writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
            writer.writeheader()
            writer.writerows(rows)
        summary_path.write_text(serialized+'\n', encoding='utf-8')
    print(serialized)
    return 0 if summary['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
