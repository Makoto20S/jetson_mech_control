"""Portable acceptance of actual controller_manager / REF01 MuJoCo evidence.

This module never starts ROS or opens devices. Timestamps are from one host's
monotonic clock. Feedback-to-ctrl-write latency excludes actuator response.
The recovery criteria follow tools/verify_fixed_pose.py::assess, except that
hardware cannot observe the controller's pre-limit torque/scale: no saturation
claim is made here. Actual commanded effort is checked against the 0.2 Nm
simulation envelope independently.
"""
import csv
import json
import math
from pathlib import Path

import numpy as np

from timing_contract import TimingSpec, evaluate_case

INTEGER_FIELDS = {
    'index', 'cycle_start_ns', 'feedback_ns', 'application_ns', 'finish_ns',
    'read_count', 'write_count', 'sequence', 'generation', 'source_fresh',
    'command_accepted', 'receiver_latched',
}
REAL_FIELDS = {
    'sim_time_s', 'period_arg_ms', 'psi', 'beta', 'dpsi', 'dbeta',
    'effort104', 'effort105', 'actual_force104', 'actual_force105', 'closure_m',
    'motor104_position', 'motor105_position', 'motor104_velocity',
    'motor105_velocity', 'command_generation104', 'command_generation105',
}
REQUIRED_FIELDS = INTEGER_FIELDS | REAL_FIELDS
FORCE_FIELDS = ('effort104', 'effort105', 'actual_force104', 'actual_force105')
REQUIRED_MANAGER_CHECKS = {
    'official_binary', 'controller_active', 'command_interfaces_claimed',
    'hardware_active', 'command_interfaces_available', 'state_interfaces_available',
    'update_rate_200hz', 'plan_subscription_unique',
    'explicit_controller_deactivation', 'explicit_hardware_deactivation',
    'no_forced_termination', 'no_supervisor_error',
}
META_FIELDS = {
    'schema', 'simulation_only', 'scheduler', 'scenario', 'duration_s',
    'target_samples', 'max_samples', 'samples', 'read_count', 'write_count',
    'startup_writes', 'nominal_period_s', 'physics_step_s',
    'physics_steps_per_cycle', 'receiver_ttl_ns', 'first_read_ns', 'finish_ns',
    'runtime_s', 'sim_time_s', 'receiver_latched', 'receiver_fault',
    'receiver_zero_ns', 'interruption_ns', 'warnings', 'max_closure_m',
    'final_q', 'final_dq', 'runtime_error',
}


def _finite(value):
    return type(value) in (int, float) and math.isfinite(value)


def _integer(value, minimum=0):
    return type(value) is int and value >= minimum


def load_rows(path):
    """Preserve integer nanosecond precision; reject incomplete/NaN traces."""
    with Path(path).open(newline='', encoding='utf-8') as stream:
        reader = csv.DictReader(stream)
        missing = REQUIRED_FIELDS - set(reader.fieldnames or ())
        if missing:
            raise ValueError('Missing CSV fields: ' + str(sorted(missing)))
        rows = []
        for index, source in enumerate(reader):
            row = {key: int(source[key]) for key in INTEGER_FIELDS}
            row.update({key: float(source[key]) for key in REAL_FIELDS})
            if not all(math.isfinite(row[key]) for key in REAL_FIELDS):
                raise ValueError('Nonfinite CSV value at row ' + str(index))
            if any(row[key] < 0 for key in INTEGER_FIELDS):
                raise ValueError('Negative counter/timestamp at row ' + str(index))
            if any(row[key] not in (0, 1) for key in
                   ('source_fresh', 'command_accepted', 'receiver_latched')):
                raise ValueError('Invalid CSV boolean at row ' + str(index))
            if (row['cycle_start_ns'] <= 0 or row['feedback_ns'] <= 0 or
                    row['finish_ns'] < row['cycle_start_ns']):
                raise ValueError('Invalid event timestamp at row ' + str(index))
            if row['application_ns'] and not (
                    row['cycle_start_ns'] <= row['application_ns'] <= row['finish_ns']
                    and row['feedback_ns'] <= row['application_ns']):
                raise ValueError('Invalid feedback/application order at row ' + str(index))
            rows.append(row)
    if len(rows) < 2:
        raise ValueError('At least two measured cycles required')
    return rows


def _load_metadata(folder):
    meta = json.loads((folder / 'metadata.json').read_text(encoding='utf-8'))
    complete = json.loads((folder / 'complete.json').read_text(encoding='utf-8'))
    if not isinstance(meta, dict) or not META_FIELDS <= meta.keys():
        raise ValueError('Missing hardware metadata schema/fields')
    if meta['schema'] != 'ref01_mujoco_system_v1':
        raise ValueError('Unsupported hardware metadata schema')
    if not isinstance(complete, dict) or not {'complete', 'samples', 'runtime_error'} <= complete.keys():
        raise ValueError('Missing completion marker fields')
    for key in ('target_samples', 'max_samples', 'samples', 'read_count',
                'write_count', 'startup_writes', 'physics_steps_per_cycle',
                'receiver_ttl_ns', 'first_read_ns', 'finish_ns', 'receiver_zero_ns',
                'interruption_ns', 'warnings'):
        if not _integer(meta[key]):
            raise ValueError('Invalid metadata integer: ' + key)
    for key in ('duration_s', 'nominal_period_s', 'physics_step_s', 'runtime_s',
                'sim_time_s', 'max_closure_m'):
        if not _finite(meta[key]) or meta[key] < 0:
            raise ValueError('Invalid metadata number: ' + key)
    for key in ('final_q', 'final_dq'):
        if not isinstance(meta[key], list) or len(meta[key]) != 2 or not all(map(_finite, meta[key])):
            raise ValueError('Invalid metadata vector: ' + key)
    if type(meta['simulation_only']) is not bool or type(meta['receiver_latched']) is not bool:
        raise ValueError('Invalid metadata boolean')
    return meta, complete


def _status_window(samples, rows):
    """Use observation time, not delivery time, to exclude post-completion faults."""
    if not isinstance(samples, list):
        raise ValueError('Controller status samples must be a list')
    required = {'mode', 'reason_name', 'generation', 'source_time_s',
                'observed_at_monotonic_s', 'compute_ms', 'q', 'dq',
                'closure_residual', 'received_ns'}
    window = []
    start, end = rows[0]['cycle_start_ns'] / 1e9, rows[-1]['finish_ns'] / 1e9
    for sample in samples:
        if not isinstance(sample, dict) or not required <= sample.keys():
            raise ValueError('Missing controller status fields')
        if (not _integer(sample['mode']) or not _integer(sample['generation']) or
                not _integer(sample['received_ns'], 1) or not isinstance(sample['reason_name'], str)):
            raise ValueError('Invalid controller status scalar')
        observed = sample['observed_at_monotonic_s']
        if not _finite(observed) or observed <= 0:
            raise ValueError('Invalid controller status observation time')
        # A null inactive snapshot is not plant evidence. It is valid outside
        # the measurement window and cannot substitute for a Following sample.
        if not start <= observed <= end:
            continue
        for key in ('source_time_s', 'compute_ms', 'closure_residual'):
            if not _finite(sample[key]) or sample[key] < 0:
                raise ValueError('Invalid measured controller status: ' + key)
        for key in ('q', 'dq'):
            if (not isinstance(sample[key], list) or len(sample[key]) != 2 or
                    not all(map(_finite, sample[key]))):
                raise ValueError('Invalid measured controller status vector: ' + key)
        if sample['received_ns'] / 1e9 < observed - 1e-6:
            raise ValueError('Controller status arrives before its observation')
        window.append(sample)
    return window


def _recovery(rows, meta, axis):
    """Same recovery bands/physics checks as verify_fixed_pose.assess.

    Deliberately omits its `unsaturated` criterion because SystemInterface
    does not observe pre-limit controller output. No dummy scale is supplied.
    """
    def cols(*keys):
        return np.asarray([[row[key] for key in keys] for row in rows])

    reference = np.deg2rad([0., 20.])
    t = cols('sim_time_s')[:, 0]
    err = np.rad2deg(cols('psi', 'beta') - reference)
    speed = np.rad2deg(cols('dpsi', 'dbeta'))
    motor, actual = cols('effort104', 'effort105'), cols('actual_force104', 'actual_force105')
    during = (t >= .4) & (t < 1.)
    tail = t >= 2.1
    if not any(t < .4) or not any((t >= .95) & during) or not any(tail):
        raise ValueError('Recovery trace lacks baseline, pulse or settling window')
    good = (np.max(np.abs(err), axis=1) <= .05) & (np.max(np.abs(speed), axis=1) <= .25)
    settled = np.flatnonzero((t >= 1.) & np.logical_and.accumulate(good[::-1])[::-1])
    peak = float(np.max(err[:, axis]))
    baseline = float(np.max(np.abs(err[t < .4])))
    steady = float(np.mean(err[(t >= .95) & during, axis]))
    expected = float(np.rad2deg(.09 / (2.4, 2.)[axis]))
    closure = float(np.max(cols('closure_m')))
    checks = dict(finite=all(math.isfinite(v) for row in rows for v in row.values()),
                  gravity_hold=baseline < .02, positive_deflection=peak > .5,
                  elastic_deflection=abs(steady - expected) < .1,
                  restores_both_axes=bool(len(settled) and t[settled[0]] <= 2.1 and good[tail].all()),
                  last_step=bool(np.max(np.abs(np.rad2deg(np.asarray(meta['final_q']) - reference))) < .05
                                 and np.max(np.abs(np.rad2deg(meta['final_dq']))) < .25),
                  closure=closure < 1e-4, solver_warnings=meta['warnings'] == 0,
                  actuator_match=bool(np.max(np.abs(motor - actual)) < 1e-10))
    return dict(passed=all(checks.values()), checks=checks,
                criteria_source='mujoco/ref01/tools/verify_fixed_pose.py::assess',
                saturation_assessed=False,
                saturation_note='Pre-limit torque/scale is not observed by hardware; only actual effort envelope is checked.',
                peak_deg=peak, cross_peak_deg=float(np.max(np.abs(err[:, 1-axis]))),
                baseline_max_error_deg=baseline, steady_deflection_deg=steady,
                theoretical_static_deg=expected, tail_max_error_deg=float(np.max(np.abs(err[tail]))),
                settling_after_release_s=float(t[settled[0]] - 1.) if len(settled) else None,
                motor_min_nm=motor.min(axis=0).tolist(), motor_max_nm=motor.max(axis=0).tolist(),
                max_closure_m=closure, warnings=meta['warnings'])


def summarize_case(folder, case, duration, returncode, manager_evidence, status_samples):
    """Return separate functional and unchanged candidate timing assessments.

    `manager_evidence['checks']` contains all REQUIRED_MANAGER_CHECKS as bools,
    produced by the orchestrator from actual ListControllers,
    ListHardwareComponents, update_rate and official executable observations.
    Missing/malformed evidence raises ValueError; false evidence cannot pass.
    """
    if case not in ('psi', 'beta', 'loss', 'duplicate'):
        raise ValueError('Unknown scenario')
    if not _finite(duration) or not 2.5 <= duration <= 120.:
        raise ValueError('Duration must be finite and in [2.5, 120] seconds')
    folder = Path(folder)
    rows = load_rows(folder / 'trace.csv')
    meta, complete = _load_metadata(folder)
    status = _status_window(status_samples, rows)
    claims = manager_evidence.get('checks') if isinstance(manager_evidence, dict) else None
    if not isinstance(claims, dict) or not claims or not all(
            isinstance(key, str) and key and type(value) is bool for key, value in claims.items()):
        raise ValueError('Explicit nonempty manager evidence checks required')
    missing_claims = REQUIRED_MANAGER_CHECKS - claims.keys()
    if missing_claims:
        raise ValueError('Missing required manager evidence checks: ' + str(sorted(missing_claims)))
    count = round(duration / .005) + 1
    checks = {'manager_' + key: value for key, value in claims.items()}
    checks.update(
        process_completed=returncode == 0,
        completion_recorded=complete['complete'] is True and complete['samples'] == len(rows)
                            and complete['runtime_error'] == 'none',
        hardware_runtime_completed=meta['runtime_error'] == 'none',
        actual_manager_scheduler=meta['scheduler'] == 'controller_manager',
        simulation_only=meta['simulation_only'] is True,
        matching_scenario=meta['scenario'] == case,
        expected_cycles=len(rows) == count == meta['samples'] == meta['target_samples']
                        and meta['max_samples'] >= count,
        matching_duration=abs(meta['duration_s'] - duration) < 1e-9,
        expected_physics_steps=meta['nominal_period_s'] == .005 and meta['physics_step_s'] == .0005
                               and meta['physics_steps_per_cycle'] == 10,
        receiver_ttl_60ms=meta['receiver_ttl_ns'] == 60_000_000,
        no_mujoco_warnings=meta['warnings'] == 0,
        recorded_window_matches=0 < meta['first_read_ns'] <= rows[0]['cycle_start_ns']
                                and meta['finish_ns'] == rows[-1]['finish_ns'],
        hardware_counts_cover_trace=meta['read_count'] >= len(rows) and meta['write_count'] >= len(rows)
                                   and meta['read_count'] == rows[-1]['read_count']
                                   and meta['write_count'] == rows[-1]['write_count'],
        indexed_cycles=all(row['index'] == i for i, row in enumerate(rows)),
        increasing_cycles=all(b['cycle_start_ns'] > a['cycle_start_ns'] for a, b in zip(rows, rows[1:])),
        read_write_each_cycle=all(b['read_count'] == a['read_count'] + 1 and
                                  b['write_count'] == a['write_count'] + 1 for a, b in zip(rows, rows[1:])),
        actual_period_positive=all(row['period_arg_ms'] > 0 for row in rows),
        generation_never_regresses=all(b['generation'] >= a['generation'] for a, b in zip(rows, rows[1:])),
        effort_within_simulation_envelope=all(abs(row[key]) <= .2 + 1e-12 for row in rows for key in FORCE_FIELDS),
    )
    functional = dict(checks=checks)
    if case in ('psi', 'beta'):
        recovery = _recovery(rows, meta, 1 if case == 'beta' else 0)
        checks.update(recovery['checks'])
        following = [sample for sample in status if sample['mode'] == 2 and sample['reason_name'] == 'none']
        checks.update(
            following_status_in_window=bool(following),
            no_controller_fault_in_window=bool(status) and all(
                sample['mode'] == 2 and sample['reason_name'] == 'none' for sample in status),
            fresh_following_status=bool(following) and all(
                0 <= sample['observed_at_monotonic_s'] - sample['source_time_s'] <= .060001
                and sample['generation'] > 0 for sample in following),
            fresh_command_each_cycle=rows[0]['generation'] > 0 and all(
                b['generation'] == a['generation'] + 1 for a, b in zip(rows, rows[1:])),
            fresh_source_each_cycle=all(row['source_fresh'] == 1 for row in rows) and all(
                b['sequence'] == a['sequence'] + 1 and b['feedback_ns'] > a['feedback_ns']
                for a, b in zip(rows, rows[1:])),
            actual_application_each_cycle=all(row['application_ns'] > 0 and row['command_accepted'] == 1 for row in rows),
            paired_generations=all(row['command_generation104'] == row['command_generation105'] == row['generation'] for row in rows),
            receiver_not_latched=meta['receiver_latched'] is False and all(row['receiver_latched'] == 0 for row in rows),
        )
        functional['recovery'] = recovery
    else:
        cut, guard_zero = meta['interruption_ns'], meta['receiver_zero_ns']
        zero_rows = [row for row in rows if row['cycle_start_ns'] >= cut
                     and row['receiver_latched'] == 1
                     and all(row[key] == 0. for key in FORCE_FIELDS)]
        # receiver_zero_ns is the guard-latch instant, not actuator application.
        # First completed physics cycle with all requested and actual forces
        # zero gives a conservative, independently observed zero boundary.
        zero = zero_rows[0]['finish_ns'] if zero_rows else 0
        valid_events = rows[0]['cycle_start_ns'] < cut <= guard_zero <= zero <= rows[-1]['finish_ns']
        pre = [row for row in rows if row['cycle_start_ns'] < cut]
        post = [row for row in rows if row['cycle_start_ns'] >= cut]
        interrupted = [row for row in rows if 1.2 <= row['sim_time_s'] < 1.65]
        returned = [row for row in rows if row['sim_time_s'] >= 1.65]
        frozen_fields = ('sequence', 'feedback_ns', 'motor104_position',
                         'motor105_position', 'motor104_velocity', 'motor105_velocity')
        applications = [row['application_ns'] for row in rows if row['application_ns'] > 0]
        last_application = max(applications, default=0)
        fault_status = [sample for sample in status if sample['observed_at_monotonic_s'] >= cut / 1e9]
        checks.update(
            valid_interruption_and_zero_events=valid_events,
            receiver_zero_within_80ms=valid_events and zero - cut <= 80_000_000,
            generation_stopped_after_cut=bool(pre and post) and all(
                row['generation'] == pre[-1]['generation'] for row in post),
            no_new_application_after_cut=0 < last_application <= cut and bool(post) and all(
                row['application_ns'] == 0 and row['command_accepted'] == 0 for row in post),
            feedback_frozen_during_interruption=bool(pre and interrupted) and all(
                row['source_fresh'] == 0 and all(row[key] == pre[-1][key] for key in frozen_fields)
                for row in interrupted),
            feedback_really_returns=bool(interrupted and returned)
                                   and all(row['source_fresh'] == 1 for row in returned)
                                   and all(b['sequence'] == a['sequence'] + 1
                                           and b['feedback_ns'] > a['feedback_ns']
                                           for a, b in zip([interrupted[-1]] + returned, returned)),
            no_restart_after_feedback_returns=bool(returned) and all(
                row['application_ns'] == 0 and row['command_accepted'] == 0 and row['receiver_latched'] == 1
                for row in returned),
            final_effort_zero=len(rows) >= 20 and all(row[key] == 0. for row in rows[-20:] for key in FORCE_FIELDS),
            receiver_latched=meta['receiver_latched'] is True,
            receiver_expiration_fault=meta['receiver_fault'] == 'expired',
            feedback_timeout_fault=any(sample['mode'] == 3 and sample['reason_name'] == 'feedback_expired'
                                       for sample in fault_status),
        )
        functional.update(zero_applied_after_cut_ms=(zero - cut) / 1e6 if valid_events else None,
                          guard_latched_after_cut_ms=(guard_zero - cut) / 1e6 if guard_zero >= cut > 0 else None,
                          last_application_after_cut_ms=(last_application - cut) / 1e6,
                          receiver_fault=meta['receiver_fault'])
    functional['passed'] = all(checks.values())
    trace = dict(schema=1, clock='monotonic_ns_same_host',
                 measurement='Official controller_manager read() synthetic encoder source -> accepted new-generation MuJoCo ctrl write() on the same host; excludes actuator physics, USB/CAN and real devices.',
                 window_start_ns=rows[0]['cycle_start_ns'], window_end_ns=rows[-1]['finish_ns'],
                 samples=[{key: row[key] for key in ('cycle_start_ns', 'feedback_ns', 'application_ns')} for row in rows])
    result = evaluate_case(case, functional['passed'], trace, TimingSpec())
    result.update(functional=functional, hardware_metadata=meta,
                  manager_evidence=manager_evidence,
                  diagnostics=dict(status_received=len(status_samples), status_in_measurement_window=len(status),
                                   compute_deadline_misses=sum(sample['compute_ms'] >= 5. for sample in status),
                                   max_observed_compute_ms=max((sample['compute_ms'] for sample in status), default=None),
                                   compute_sampling_note='Status service is sampled; these are not all-cycle compute deadline counts.',
                                   cycle_work_over_5ms=sum(row['finish_ns'] - row['cycle_start_ns'] >= 5_000_000 for row in rows),
                                   max_callback_ms=max((row['finish_ns'] - row['cycle_start_ns']) / 1e6 for row in rows),
                                   period_argument_min_ms=min(row['period_arg_ms'] for row in rows),
                                   period_argument_max_ms=max(row['period_arg_ms'] for row in rows)))
    return result
