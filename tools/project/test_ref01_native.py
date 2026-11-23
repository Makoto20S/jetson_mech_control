"""Compare native REF01 model/control against existing Python numerical oracles.

Run from the repository: python -B test_ref01_native.py --probe <exe> --output <json>
This is numerical software verification; subprocess round trips and Python
MuJoCo physics stepping are deliberately NOT performance measurements.
"""
import argparse
from collections import deque
import hashlib
import json
import math
from pathlib import Path
import platform
import queue
import subprocess
import sys
import threading


class Probe:
    def __init__(self, executable, model_path, cwd):
        self.child = subprocess.Popen([str(executable), model_path], cwd=cwd,
                                      stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                      stderr=subprocess.PIPE, text=True, bufsize=1)
        self.lines = queue.Queue()
        self.errors = deque(maxlen=30)
        threading.Thread(target=self._read, daemon=True).start()
        threading.Thread(target=self._read_errors, daemon=True).start()

    def _read(self):
        for line in self.child.stdout:
            self.lines.put(line)
        self.lines.put(None)

    def _read_errors(self):
        for line in self.child.stderr:
            self.errors.append(line.rstrip())

    def evaluate(self, alpha, velocity):
        self.child.stdin.write(' '.join(format(float(x), '.17g') for x in [*alpha, *velocity])+'\n')
        self.child.stdin.flush()
        try:
            line = self.lines.get(timeout=10.)
        except queue.Empty as error:
            raise RuntimeError('Native numerical probe timed out') from error
        if line is None:
            raise RuntimeError('Native probe exited: '+'; '.join(self.errors))
        fields = line.split()
        if fields == ['0']:
            return None
        if len(fields) != 14 or fields[0] != '1':
            raise ValueError('Malformed probe output: '+line[:500])
        values = [float(x) for x in fields[1:]]
        if not all(math.isfinite(value) for value in values):
            raise ValueError('Native probe marked nonfinite values successful')
        return dict(q=values[0:2], dq=values[2:4], gravity=values[4:6],
                    A=[values[6:8], values[8:10]], torque=values[10:12], scale=values[12])

    def close(self):
        self.child.stdin.close()
        try:
            code = self.child.wait(timeout=5.)
        except subprocess.TimeoutExpired:
            self.child.kill()
            self.child.wait()
            raise RuntimeError('Native probe did not exit after EOF')
        if code:
            raise RuntimeError('Native probe failed: '+'; '.join(self.errors))


def verify(args):
    import mujoco
    import numpy as np
    from fixed_pose_impedance import FixedPoseImpedance
    from ref01_dynamics_api import REF01Dynamics
    from ref01_kinematics import inverse_kinematics
    from ref01_mujoco_kinematics import MuJoCoForwardKinematics
    from tools.verify_fixed_pose import (REFERENCE, STIFFNESS, DAMPING, LIMIT,
        CONTROL_DT, PULSE_START, PULSE_END, DURATION, PULSE_NM, assess, run_case)

    model = REF01Dynamics()
    controller = FixedPoseImpedance(model, REFERENCE, STIFFNESS, DAMPING, LIMIT)
    fk = MuJoCoForwardKinematics()
    limits = dict(q=1e-7, dq=1e-7, gravity=1e-8, A=2e-7, torque=1e-8, scale=1e-7)
    maximum = {key: 0. for key in limits}
    comparisons, failures = [], []
    probe = Probe(args.probe, args.model, args.repo)
    try:
        # 27 distinct poses include all workspace edges. Nonzero input velocities
        # are independent of native A and exercise opposite motor directions.
        poses = [(float(psi), beta) for psi in np.linspace(-45., 45., 9)
                 for beta in (0., 22.5, 45.)]
        velocities = [(0., 0.), (.30, -.15), (-.25, .40)]
        for pose_deg in poses:
            q = np.deg2rad(pose_deg)
            alpha = inverse_kinematics(*q, unit='rad')
            python_q = np.asarray(fk.solve(alpha).q)
            if np.max(np.abs(python_q-q)) > limits['q']:
                raise ValueError('Existing FK disagrees with frozen analytic pose input')
            terms = model.dynamics_terms(q)
            for velocity in velocities:
                dq, _ = fk.velocity(alpha, velocity)
                expected_command = controller.command(q, dq)
                expected = dict(q=q, dq=dq, gravity=terms['bias'], A=terms['A'],
                                torque=expected_command.alpha_nm, scale=expected_command.scale)
                actual = probe.evaluate(alpha, velocity)
                record = dict(pose_deg=list(pose_deg), alpha_rad=list(alpha),
                              alpha_velocity_rad_s=list(velocity), native=actual)
                if actual is None:
                    record['passed'] = False
                    failures.append('Native rejected valid pose '+str(pose_deg))
                else:
                    errors = {key: float(np.max(np.abs(np.asarray(actual[key])-value)))
                              for key, value in expected.items()}
                    record['errors'] = errors
                    record['expected'] = {key: np.asarray(value).tolist() for key, value in expected.items()}
                    record['passed'] = all(np.isfinite(value) and value <= limits[key]
                                           for key, value in errors.items())
                    for key, value in errors.items():
                        maximum[key] = max(maximum[key], value)
                    if not record['passed']:
                        failures.append('Numerical mismatch '+str(pose_deg)+' '+str(velocity))
                comparisons.append(record)

        q = np.deg2rad([20., 32.])
        alpha = inverse_kinematics(*q, unit='rad')
        actual = probe.evaluate(alpha, (0., 0.))
        expected = controller.command(q, (0., 0.))
        allocation_ok = actual is not None
        if actual:
            achieved = np.asarray(actual['A']).T @ actual['torque']
            demand = np.asarray(actual['gravity']) + np.asarray(STIFFNESS)*(REFERENCE-actual['q'])
            allocation_ok = bool(0 < actual['scale'] < 1 and
                np.max(np.abs(actual['torque'])) <= .2+1e-12 and
                np.max(np.abs(achieved-actual['scale']*demand)) < 1e-10 and
                np.max(np.abs(actual['torque']-expected.alpha_nm)) < limits['torque'])

        # Invalid frames must fail without contaminating the retained model seed.
        reference_alpha = inverse_kinematics(*REFERENCE, unit='rad')
        baseline = probe.evaluate(reference_alpha, (0., 0.))
        invalid_inputs = [([float('nan'), 0.], [0., 0.]),
                          ([0., float('inf')], [0., 0.]),
                          (reference_alpha, [float('nan'), 0.]),
                          ([4., 0.], [0., 0.]),
                          ([-3., 3.], [0., 0.])]
        invalid_checks = []
        for invalid_alpha, invalid_velocity in invalid_inputs:
            rejected = probe.evaluate(invalid_alpha, invalid_velocity) is None
            recovered = probe.evaluate(reference_alpha, (0., 0.))
            recovered_ok = baseline is not None and recovered is not None
            if recovered_ok:
                recovered_ok = all(np.max(np.abs(np.asarray(recovered[k])-baseline[k])) < limits[k]
                                   for k in limits)
            invalid_checks.append(dict(input=[str(x) for x in [*invalid_alpha, *invalid_velocity]],
                                       rejected=rejected, next_valid_matches=bool(recovered_ok)))

        recovery, traces = {}, {}
        steps = round(CONTROL_DT/model.model.opt.timestep)
        if steps < 1 or abs(steps*model.model.opt.timestep-CONTROL_DT) > 1e-12:
            raise ValueError('Control period does not divide the physics period')
        for axis, name in enumerate(('psi', 'beta')):
            state = model.reset(REFERENCE)
            rows = []
            for tick in range(round(DURATION/CONTROL_DT)):
                t = tick*CONTROL_DT
                q, dq = state['q'], state['dq']
                command = probe.evaluate(state['alpha'], state['alpha_velocity'])
                if command is None:
                    raise ValueError('Native rejected '+name+' recovery state at '+str(t))
                external = np.zeros(2)
                if PULSE_START <= t < PULSE_END:
                    external[axis] = PULSE_NM
                torque = np.asarray(command['torque'])
                state = model.step(torque, steps=steps, external_torque=external)
                rows.append(dict(case=name, time_s=t, psi_rad=q[0], beta_rad=q[1],
                    psi_ref_rad=REFERENCE[0], beta_ref_rad=REFERENCE[1],
                    dpsi_rad_s=dq[0], dbeta_rad_s=dq[1],
                    tau104_nm=torque[1], tau105_nm=torque[0], scale=command['scale'],
                    closure_end_m=state['max_closure_m'],
                    applied104_end_nm=state['tau_applied'][1], applied105_end_nm=state['tau_applied'][0]))
            native_result = assess(rows, state, axis)
            old_rows, old_state = run_case(model, controller, axis)
            old_result = assess(old_rows, old_state, axis)
            curve_errors = {key: max(abs(a[key]-b[key]) for a, b in zip(rows, old_rows))
                            for key in ('psi_rad', 'beta_rad', 'dpsi_rad_s', 'dbeta_rad_s',
                                        'tau104_nm', 'tau105_nm')}
            # The native loop reconstructs q from motor feedback, unlike the old
            # physics-state reference loop. MuJoCo soft closure causes small,
            # expected differences; recovery criteria stay exactly the old ones.
            curve_limits = dict(psi_rad=2e-4, beta_rad=2e-4,
                                dpsi_rad_s=2e-3, dbeta_rad_s=2e-3,
                                tau104_nm=2e-3, tau105_nm=2e-3)
            curve_ok = all(curve_errors[key] <= value for key, value in curve_limits.items())
            recovery[name] = dict(passed=native_result['passed'] and old_result['passed'] and curve_ok,
                                  native=native_result, python_reference=old_result,
                                  max_curve_difference=curve_errors, curve_difference_limits=curve_limits)
            traces[name] = rows

        passed = (not failures and allocation_ok and all(
            check['rejected'] and check['next_valid_matches'] for check in invalid_checks)
            and all(case['passed'] for case in recovery.values()))
        return dict(schema=1, passed=bool(passed), evidence_kind='native_numerical_comparison',
            performance_verified=False, hardware_verified=False,
            host=platform.platform(), python=sys.version, mujoco=mujoco.__version__, numpy=np.__version__,
            model_sha256=model.metadata['model_sha256'], probe_sha256=hashlib.sha256(args.probe.read_bytes()).hexdigest(),
            distinct_poses=len(poses), velocity_cases_per_pose=len(velocities), samples=len(comparisons),
            tolerances=limits, maximum_errors=maximum, failures=failures,
            paired_saturation_check=allocation_ok, invalid_inputs=invalid_checks,
            recovery=recovery, comparisons=comparisons, recovery_traces=traces)
    finally:
        probe.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--probe', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--repo', type=Path, default=Path.cwd())
    parser.add_argument('--model', default='mujoco/ref01/config/mujoco/ref01_dynamics_cad_v1.xml')
    args = parser.parse_args()
    args.repo, args.probe = args.repo.resolve(), args.probe.resolve()
    if Path(args.model).is_absolute() or not args.model.isascii():
        parser.error('--model must be a relative ASCII path (Windows native loading)')
    if args.output.exists():
        parser.error('--output must name a new file')
    sys.path.insert(0, str(args.repo/'mujoco/ref01'))
    try:
        result = verify(args)
    except Exception as error:
        result = dict(schema=1, passed=False, evidence_kind='native_numerical_comparison',
                      performance_verified=False, hardware_verified=False,
                      error=type(error).__name__+': '+str(error))
    args.output.parent.mkdir(exist_ok=True, parents=True)
    args.output.write_text(json.dumps(result, indent=2, allow_nan=False)+'\n', encoding='utf-8')
    print(json.dumps({key: value for key, value in result.items()
                      if key not in ('comparisons', 'recovery_traces')}, indent=2, allow_nan=False))
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
