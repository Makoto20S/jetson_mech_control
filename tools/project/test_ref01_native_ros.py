"""Orin software-only native controller/MuJoCo loop and explicit timing acceptance.

Run under the outer sync/build lock (EXO_SYNC_LOCK_HELD=1). No serial, CAN,
hardware plugin or motor launch is used. A short smoke run is permitted, but
cannot satisfy the unchanged >=30 s/6000-cycle timing contract.
"""
import argparse
import csv
import hashlib
import json
import math
import os
from pathlib import Path
import platform
import subprocess
import sys
import time

from timing_contract import TimingSpec, evaluate_case, evaluate_suite

ROOT = Path(__file__).resolve().parents[2]
MODEL = 'mujoco/ref01/config/mujoco/ref01_dynamics_cad_v1.xml'
INTEGER_FIELDS = {'cycle_start_ns', 'feedback_ns', 'application_ns', 'finish_ns',
                  'mode', 'reason', 'generation'}
REQUIRED_FIELDS = INTEGER_FIELDS | {
    'time_s', 'psi_rad', 'beta_rad', 'dpsi_rad_s', 'dbeta_rad_s',
    'estimated_psi_rad', 'estimated_beta_rad', 'estimated_dpsi', 'estimated_dbeta',
    'tau104_nm', 'tau105_nm', 'applied104_end_nm', 'applied105_end_nm',
    'scale', 'closure_end_m', 'compute_ms'}


def load_rows(path):
    with path.open(newline='', encoding='utf-8') as stream:
        reader = csv.DictReader(stream)
        if not REQUIRED_FIELDS <= set(reader.fieldnames or ()):
            raise ValueError('Missing CSV fields: '+str(sorted(REQUIRED_FIELDS-set(reader.fieldnames or ()))))
        rows = []
        for index, source in enumerate(reader):
            row = {}
            for key in REQUIRED_FIELDS:
                if key in INTEGER_FIELDS:
                    # int(string) preserves nanosecond precision and rejects NaN.
                    row[key] = int(source[key])
                else:
                    row[key] = float(source[key])
                    if not math.isfinite(row[key]):
                        raise ValueError('Nonfinite '+key+' at row '+str(index))
            if (row['cycle_start_ns'] <= 0 or row['feedback_ns'] <= 0 or
                    row['finish_ns'] < row['cycle_start_ns'] or row['application_ns'] < 0):
                raise ValueError('Invalid event timestamp at row '+str(index))
            if row['application_ns'] and not (row['cycle_start_ns'] <= row['application_ns'] <= row['finish_ns']
                                              and row['feedback_ns'] <= row['application_ns']):
                raise ValueError('Invalid feedback/application order at row '+str(index))
            rows.append(row)
    if not rows:
        raise ValueError('No recorded cycles')
    return rows


def tracking_error(rows):
    pairs = {'psi_rad': 'estimated_psi_rad', 'beta_rad': 'estimated_beta_rad',
             'dpsi_rad_s': 'estimated_dpsi', 'dbeta_rad_s': 'estimated_dbeta'}
    valid = [row for row in rows if row['mode'] == 2]
    return dict(samples=len(valid), acceptance_gate=False,
                note='Native constrained estimate versus soft-constraint plant; errors recorded, no unvalidated threshold.',
                maximum_absolute={actual: max((abs(row[actual]-row[estimate]) for row in valid), default=None)
                                  for actual, estimate in pairs.items()})


def shared_checks(rows, meta, returncode, duration):
    count = round(duration/.005)+1
    return dict(process_completed=returncode == 0,
                expected_cycles=len(rows) == count and meta.get('samples') == len(rows),
                ros_observation_received=meta.get('ros_observed', 0) > 0,
                ros_observation_published=meta.get('ros_published', 0) > 0,
                no_mujoco_warnings=meta.get('warnings') == 0,
                increasing_cycles=all(b['cycle_start_ns'] > a['cycle_start_ns']
                                      for a, b in zip(rows, rows[1:])),
                generation_never_regresses=all(b['generation'] >= a['generation']
                                              for a, b in zip(rows, rows[1:])))


def normal_function(rows, meta, case, checks):
    import numpy as np
    sys.path.insert(0, str(ROOT/'mujoco/ref01'))
    from tools.verify_fixed_pose import assess
    state = dict(q=np.asarray(meta['final_q'], dtype=float),
                 dq=np.asarray(meta['final_dq'], dtype=float),
                 max_closure_m=float(meta['max_closure_m']), warnings=meta['warnings'])
    if (state['q'].shape != (2,) or state['dq'].shape != (2,) or
            not np.isfinite(state['q']).all() or not np.isfinite(state['dq']).all() or
            not math.isfinite(state['max_closure_m'])):
        raise ValueError('Invalid final native plant state')
    assessment = assess(rows, state, 1 if case == 'beta' else 0)
    checks.update(assessment['checks'])
    checks.update(controller_remains_following=all(row['mode'] == 2 and row['reason'] == 0 for row in rows),
                  fresh_command_each_cycle=all(b['generation'] == a['generation']+1
                                               for a, b in zip(rows, rows[1:])),
                  actual_application_each_cycle=all(row['application_ns'] > 0 for row in rows),
                  final_controller_reason_none=meta.get('final_reason') == 'none',
                  receiver_not_latched=meta.get('receiver_latched') is False)
    return dict(passed=all(checks.values()), checks=checks, recovery=assessment,
                estimated_state_error=tracking_error(rows))


def fault_function(rows, meta, checks):
    cut, zero = meta.get('interruption_ns'), meta.get('receiver_zero_ns')
    valid_events = type(cut) is int and type(zero) is int and cut > 0 and zero >= cut
    # application_ns is nonzero ONLY for an accepted new generation and an
    # actual ctrl write. Holding an old effort is not counted as a new command.
    applications = [row for row in rows if row['application_ns'] > 0]
    post_cut = [row for row in rows if type(cut) is int and row['cycle_start_ns'] >= cut]
    returned = [row for row in rows if row['time_s'] >= 1.65]
    tail = rows[-20:]
    last_application = max((row['application_ns'] for row in applications), default=0)
    fixed_generation = bool(post_cut) and all(row['generation'] == post_cut[0]['generation'] for row in post_cut)
    zero_tail = len(tail) == 20 and all(
        row[key] == 0. for row in tail
        for key in ('tau104_nm', 'tau105_nm', 'applied104_end_nm', 'applied105_end_nm'))
    checks.update(valid_interruption_and_zero_events=valid_events,
                  receiver_zero_within_80ms=valid_events and zero-cut <= 80_000_000,
                  generation_stopped_after_cut=fixed_generation,
                  no_new_application_after_cut=type(cut) is int and 0 < last_application <= cut,
                  no_restart_after_feedback_returns=bool(returned) and all(
                      row['application_ns'] == 0 and row['mode'] == 3 for row in returned),
                  final_effort_zero=zero_tail,
                  receiver_latched=meta.get('receiver_latched') is True,
                  feedback_timeout_fault=meta.get('final_reason') == 'feedback_expired')
    return dict(passed=all(checks.values()), checks=checks,
                final_reason=meta.get('final_reason'),
                zero_applied_after_cut_ms=(zero-cut)/1e6 if valid_events else None,
                last_application_after_cut_ms=(last_application-cut)/1e6 if type(cut) is int else None,
                estimated_state_error=tracking_error(rows))


def summarize_case(folder, case, duration, returncode):
    csv_path = folder/'trace.csv'
    rows = load_rows(csv_path)
    meta = json.loads(Path(str(csv_path)+'.meta.json').read_text(encoding='utf-8'))
    checks = shared_checks(rows, meta, returncode, duration)
    functional = (normal_function(rows, meta, case, checks) if case in ('psi', 'beta')
                  else fault_function(rows, meta, checks))
    trace = dict(schema=1, clock='monotonic_ns_same_host',
                 measurement='Native loop synthetic motor feedback snapshot -> accepted new-generation MuJoCo ctrl write; excludes actuator physics, USB/CAN and real devices.',
                 window_start_ns=rows[0]['cycle_start_ns'], window_end_ns=rows[-1]['finish_ns'],
                 samples=[{key: row[key] for key in ('cycle_start_ns', 'feedback_ns', 'application_ns')}
                          for row in rows])
    result = evaluate_case(case, functional['passed'], trace, TimingSpec())
    result.update(functional=functional, native_metadata=meta,
                  diagnostics=dict(compute_deadline_misses=sum(row['compute_ms'] >= 5. for row in rows),
                                   cycle_work_over_5ms=sum(row['finish_ns']-row['cycle_start_ns'] >= 5_000_000 for row in rows),
                                   max_compute_ms=max(row['compute_ms'] for row in rows),
                                   max_callback_ms=max((row['finish_ns']-row['cycle_start_ns'])/1e6 for row in rows)))
    return result


def hashes():
    paths = [Path(MODEL), Path('mujoco/ref01/cpp/ref01_model.hpp'),
             Path('ros2_ws/src/mech_controllers/test/ref01_impedance_software_loop.cpp'),
             Path('ros2_ws/src/mech_controllers/include/mech_controllers/ref01_impedance_core.hpp'),
             Path('ros2_ws/src/mech_controllers/include/mech_controllers/ref01_impedance_controller.hpp'),
             Path('ros2_ws/src/mech_controllers/src/ref01_impedance_controller.cpp'),
             Path('ros2_ws/src/mech_controllers/src/ref01_impedance_controller_plugin.cpp'),
             Path('tools/project/test_ref01_native_ros.py'), Path('tools/project/timing_contract.py')]
    paths.extend(path.relative_to(ROOT) for path in sorted((ROOT/'mujoco/ref01/config/mujoco/assets/ref01_cad_v1').glob('*.stl')))
    return {path.as_posix(): hashlib.sha256((ROOT/path).read_bytes()).hexdigest() for path in paths}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--executable', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--duration', type=float, default=30.)
    parser.add_argument('--cases', nargs='+', choices=['psi', 'beta', 'loss', 'duplicate'],
                        default=['psi', 'beta', 'loss', 'duplicate'])
    args = parser.parse_args()
    if platform.system() != 'Linux' or platform.machine().lower() not in ('aarch64', 'arm64'):
        parser.error('This entry requires the Orin Linux/ARM64 software environment')
    if os.environ.get('EXO_SYNC_LOCK_HELD') != '1':
        parser.error('Outer project sync/build lock must be held (EXO_SYNC_LOCK_HELD=1)')
    if not math.isfinite(args.duration) or not 2.5 <= args.duration <= 120.:
        parser.error('--duration must be in [2.5, 120] seconds; timing acceptance still requires 30 seconds')
    if len(args.cases) != len(set(args.cases)):
        parser.error('--cases must not repeat a case')
    executable = args.executable.resolve()
    if not executable.is_file() or executable.name != 'ref01_impedance_software_loop':
        parser.error('--executable must name the compiled ref01_impedance_software_loop fixture')
    output = args.output.resolve()
    if not output.is_relative_to(ROOT.resolve()):
        parser.error('--output must remain in this project')
    output.mkdir(parents=True, exist_ok=False)
    import mujoco
    env = dict(os.environ, ROS_DOMAIN_ID='179', ROS_LOCALHOST_ONLY='1', PYTHONDONTWRITEBYTECODE='1')
    board_file = Path('/proc/device-tree/model')
    environment = dict(host=platform.node(), platform=platform.platform(), machine=platform.machine(),
                       kernel=platform.release(), python=sys.version, python_mujoco_sdk=mujoco.__version__,
                       board=board_file.read_text().rstrip('\x00\n') if board_file.exists() else None,
                       ros_distro=os.environ.get('ROS_DISTRO'), ros_domain_id=179, ros_localhost_only=True)
    source_hashes = hashes()
    baseline = dict(schema=1, evidence_kind='orin_native_ros_mujoco_software_loop',
                    environment=environment, source_sha256=source_hashes,
                    executable_sha256=hashlib.sha256(executable.read_bytes()).hexdigest(),
                    model_sha256=source_hashes[MODEL], requested_duration_s=args.duration,
                    performance_scope='Candidate software timing; no hard-real-time or real motor/transport acceptance.',
                    commands=[], started_utc=time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime()))
    (output/'run_metadata.json').write_text(json.dumps(baseline, indent=2)+'\n', encoding='utf-8')
    results = []
    for case in args.cases:
        folder = output/case
        folder.mkdir()
        # Fault fixtures need the 1.2 s cut and 1.65 s return plus a verified
        # zero-effort tail. A long uncontrolled plant run adds no evidence.
        case_duration = 2.5 if case in ('loss', 'duplicate') else args.duration
        command = [str(executable), MODEL, str(folder/'trace.csv'), case,
                   str(case_duration), '--software-only']
        run_error = None
        with (folder/'process.log').open('w', encoding='utf-8') as log:
            try:
                completed = subprocess.run(command, cwd=ROOT, env=env, stdout=log, stderr=subprocess.STDOUT,
                                           timeout=max(30., case_duration*2+15.))
                returncode = completed.returncode
            except (subprocess.TimeoutExpired, OSError) as error:
                returncode, run_error = -1, type(error).__name__+': '+str(error)
                log.write('\n'+run_error+'\n')
        baseline['commands'].append(dict(case=case, duration_s=case_duration, argv=command,
                                         returncode=returncode, error=run_error))
        try:
            result = summarize_case(folder, case, case_duration, returncode)
        except Exception as error:
            result = evaluate_case(case, False, None, TimingSpec())
            result['errors'].append(type(error).__name__+': '+str(error))
        if run_error:
            result['errors'].append(run_error)
        (folder/'summary.json').write_text(json.dumps(result, indent=2, allow_nan=False)+'\n', encoding='utf-8')
        results.append(result)
        summary = dict(baseline, **evaluate_suite(results))
        summary['completed_cases'] = [row['case'] for row in results]
        summary['requested_cases'] = args.cases
        (output/'summary.json').write_text(json.dumps(summary, indent=2, allow_nan=False)+'\n', encoding='utf-8')
        print(json.dumps(dict(case=case, functional_passed=result['functional_passed'],
                              timing_passed=result['timing_passed'], errors=result['errors'])), flush=True)
    print(json.dumps(dict(functional_passed=summary['functional_passed'], timing_passed=summary['timing_passed'],
                          passed=summary['passed'], output=str(output))), flush=True)
    return 0 if summary['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
