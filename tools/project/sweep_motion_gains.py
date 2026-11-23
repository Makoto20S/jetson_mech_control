"""Standalone Windows MuJoCo gain experiment. From the checkout, for example:
.local/venv/Scripts/python tools/project/sweep_motion_gains.py
.local/venv/Scripts/python tools/project/sweep_motion_gains.py --motion psi --k 20 45 --b .3 1.5
Omit gains to run the comparison matrix. No ROS, panel, serial or CAN transport.
"""
import argparse
import csv
from datetime import datetime
import json
import math
from pathlib import Path
import subprocess

import numpy as np

from ph_exo_bench_worker import DEFAULT, make_trajectory
from ref01_dynamic_mit import current_scale
from ref01_mit_fixture import ROOT, LSB, Plant, feedback_words
from sync_orin import safe_path
from test_ref01_dynamic_mit import build

DT = .005
PERIOD = 2.
CAP = 10.
MODEL = ROOT / 'mujoco/ref01/config/mujoco/ref01_dynamics_cad_v1.xml'
COLUMNS = ('time_s,ref_psi_deg,ref_beta_deg,psi_deg,beta_deg,'
           'target104_nm,target105_nm,applied104_nm,applied105_nm,torque_scale,closure_m,warnings')


def cases(motion):
    for axis in ('psi', 'beta') if motion is None else (motion,):
        # Vary the moving axis, and separately the held beta axis under psi motion.
        groups = ('psi', 'beta') if axis == 'psi' else ('beta',)
        seen = set()
        for group in groups:
            ks, bs = ((20, 40, 60), (.3, 1, 2)) if group == 'psi' else ((30, 45, 60), (1, 1.5, 2))
            for k in ks:
                for b in bs:
                    gains = (k, 30, b, 1) if group == 'psi' else (20, k, .3, b)
                    if gains not in seen:
                        yield axis, gains
                        seen.add(gains)
        yield axis, (60, 60, 2, 2)


def simulate(out, exe, scale, axis, gains):
    kpsi, kbeta, bpsi, bbeta = gains
    name = axis + '_' + '_'.join(format(v, '.12g') for v in gains)
    cfg = dict(DEFAULT, amp_deg=[30., 0.] if axis == 'psi' else [0., 10.], period_s=[PERIOD]*2)
    if axis == 'psi':
        cfg['wave'] = 'sine'
    trajectory = make_trajectory(cfg)
    home = [0., 0.]
    plant = Plant(pulse_nm=0.)
    plant.state = plant.model.reset(np.array(home))
    rows = []
    fault = ''
    args = [str(exe), str(MODEL), *map(str, (scale, kpsi, kbeta, CAP, bpsi, bbeta))]
    with subprocess.Popen(args, stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True, bufsize=1) as proc:
        try:
            # 1 s initial hold, 1 cycle of entry, then four complete evaluation cycles.
            for tick in range(2200):
                t = tick * DT
                new_ref = tick % 4 == 0
                if new_ref:
                    qref, vref = trajectory.sample(t, home)
                st = plant.state
                pos, vel = feedback_words(st['alpha'], st['alpha_velocity'], scale)
                msg = [t, *pos, *vel, int(new_ref), tick//4 + 1, 1.005 + t, *qref, *vref]
                proc.stdin.write(' '.join(map(str, msg)) + '\n')
                proc.stdin.flush()
                z = np.fromstring(proc.stdout.readline(), sep=' ')
                if len(z) != 15 or not np.isfinite(z).all():
                    raise RuntimeError(f'{name}: invalid probe output at {t}s')
                if z[0] != 2 or z[1] != 0 or z[2] != 1:
                    fault = f'controller mode {int(z[0])}, reason {int(z[1])} at {t:.3f}s'
                    break
                applied = z[[9, 12]]
                sample = plant.advance(applied, t, axis)
                row = [t, *np.degrees(qref), *np.degrees([sample['psi_rad'], sample['beta_rad']]),
                       z[8], z[11], *applied, z[14], sample['closure_m'], sample['warnings']]
                rows.append(row)
                if (not np.isfinite(row).all() or sample['warnings'] or sample['closure_m'] >= 1e-4
                        or max(abs(z[[8, 11]])) > CAP + 1e-12 or max(abs(applied)) > CAP + LSB/2):
                    fault = f'model or torque check at {t:.3f}s'
                    break
        finally:
            proc.stdin.close()
        if proc.wait(timeout=10):
            raise RuntimeError(f'{name}: numerical probe exited {proc.returncode}')
    a = np.asarray(rows).reshape(-1, 12)
    np.savetxt(out / (name+'.csv.gz'), a, delimiter=',', header=COLUMNS, comments='', fmt='%.9g')
    result = dict(motion=axis, kpsi=kpsi, kbeta=kbeta, bpsi=bpsi, bbeta=bbeta,
                  completed=not fault, fault=fault, samples=len(a))
    if not fault:
        window = a[a[:, 0] >= 3.]
        errors = window[:, 3:5] - window[:, 1:3]
        rms = np.sqrt(np.mean(errors**2, axis=0))
        steps = np.abs(np.diff(window[:, 7:9], axis=0))
        result.update(rms_psi_deg=rms[0], rms_beta_deg=rms[1],
                      peak_beta_error_deg=max(abs(errors[:, 1])),
                      peak104_nm=max(abs(a[:, 7])), peak105_nm=max(abs(a[:, 8])),
                      step95_104_nm=np.quantile(steps[:, 0], .95), step95_105_nm=np.quantile(steps[:, 1], .95),
                      max_step_nm=steps.max(), saturated_ticks=int((a[:, 9] < 1.).sum()))
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--motion', choices=['psi','beta'])
    parser.add_argument('--k', nargs=2, type=float, metavar=('KPSI','KBETA'))
    parser.add_argument('--b', nargs=2, type=float, metavar=('BPSI','BBETA'))
    args = parser.parse_args()
    if (args.k is None) != (args.b is None):
        parser.error('supply both --k and --b')
    if args.k is not None:
        if (not all(math.isfinite(v) and 0 < v <= 60 for v in args.k)
                or not all(math.isfinite(v) and 0 <= v <= 2 for v in args.b)):
            parser.error('software envelope: 0<K<=60, 0<=B<=2')
        requests = [(axis, (*args.k,*args.b)) for axis in ((args.motion,) if args.motion else ('psi','beta'))]
    else:
        requests = list(cases(args.motion))
    stamp = datetime.now().strftime('%Y%m%d_%H%M%S_%f')
    out = safe_path(ROOT, '.local/validation/motion_gains/'+stamp)
    out.mkdir(parents=True, exist_ok=False)
    print(out, flush=True)
    exe = build(out, offline_gain_sweep=True)
    scale, _ = current_scale()
    metadata = dict(period_s=PERIOD, feedback_s=DT, reference_s=4*DT, eval_s=[3.,11.],
                    psi_sine_amplitude_deg=30., beta_quintic_excursion_deg=10., cap_nm=CAP,
                    erpm_to_rad_s=scale, home_deg=[0,0], requested_cases=len(requests))
    (out/'settings.json').write_text(json.dumps(metadata, indent=2))
    results = []
    for axis, gains in requests:
        result = simulate(out, exe, scale, axis, gains)
        results.append(result)
        print(axis, gains, result.get('fault') or 'complete', flush=True)
    fields = dict.fromkeys(key for row in results for key in row)
    with (out/'summary.csv').open('w',newline='',encoding='utf-8') as f:
        writer = csv.DictWriter(f,fieldnames=fields)
        writer.writeheader()
        writer.writerows(results)
    return int(any(not row['completed'] for row in results))


if __name__ == '__main__':
    raise SystemExit(main())
