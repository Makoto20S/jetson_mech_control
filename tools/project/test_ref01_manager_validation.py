"""Windows/offline tests of manager evidence acceptance; never runs ROS/devices.

Synthetic timestamps exercise the verifier, not host timing. Recovery curves
come from the separately verified Windows native-vs-Python numeric fixture.
Set PH_EXO_REPO when running the staged copy outside tools/project.
"""
import csv
import json
import os
from pathlib import Path
import sys
import tempfile
import unittest

ROOT = Path(os.environ.get('PH_EXO_REPO', Path(__file__).resolve().parents[2]))
sys.path.insert(0, str(ROOT / 'tools/project'))
sys.path.insert(0, str(Path(__file__).resolve().parent))
from ref01_manager_validation import REQUIRED_FIELDS, summarize_case


class ManagerAcceptance(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        fixture = ROOT / '.local/validation/ref01_native_windows_20261111_v1/numeric.json'
        if not fixture.is_file():
            raise RuntimeError('Required prior Windows numeric evidence is missing: ' + str(fixture))
        cls.numeric = json.loads(fixture.read_text(encoding='utf-8'))
        if cls.numeric.get('passed') is not True or cls.numeric.get('evidence_kind') != 'native_numerical_comparison':
            raise RuntimeError('Invalid prior numeric fixture')

    def setUp(self):
        # Explicit task-local temporary storage; do not write canonical .local.
        self.tmp = tempfile.TemporaryDirectory(prefix='manager_validation_', dir=Path(__file__).resolve().parent)
        self.folder = Path(self.tmp.name)
        self.manager = {'checks': {'official_binary': True, 'controller_active': True,
                                  'command_interfaces_claimed': True, 'hardware_active': True,
                                  'command_interfaces_available': True, 'state_interfaces_available': True,
                                  'update_rate_200hz': True, 'plan_subscription_unique': True,
                                  'explicit_controller_deactivation': True,
                                  'explicit_hardware_deactivation': True,
                                  'no_forced_termination': True, 'no_supervisor_error': True}}

    def tearDown(self):
        self.tmp.cleanup()

    def make_case(self, case='psi'):
        duration = 2.5 if case in ('loss', 'duplicate') else 30.
        start, dt = 100_000_000_000, 5_000_000
        curve = self.numeric['recovery_traces']['beta' if case == 'beta' else 'psi']
        rows = []
        fault = case in ('loss', 'duplicate')
        for i in range(round(duration / .005) + 1):
            raw = curve[min(i, len(curve) - 1)]
            cycle = start + i * dt
            cut = fault and i >= 240
            latched = fault and i >= 252
            gen = min(i + 1, 240) if fault else i + 1
            row = dict(index=i, sim_time_s=i * .005,
                       cycle_start_ns=cycle, feedback_ns=cycle if not cut or i >= 330 else start + 239 * dt,
                       application_ns=0 if cut else cycle + 200_000, finish_ns=cycle + 400_000,
                       period_arg_ms=5., read_count=i + 4, write_count=i + 4,
                       sequence=(min(i + 1, 240) if i < 330 else i - 89) if fault else i + 1,
                       generation=gen, source_fresh=int(not (fault and 240 <= i < 330)),
                       command_accepted=int(not cut), receiver_latched=int(latched),
                       psi=raw['psi_rad'], beta=raw['beta_rad'], dpsi=raw['dpsi_rad_s'], dbeta=raw['dbeta_rad_s'],
                       effort104=0. if latched else raw['tau104_nm'],
                       effort105=0. if latched else raw['tau105_nm'],
                       actual_force104=0. if latched else raw['applied104_end_nm'],
                       actual_force105=0. if latched else raw['applied105_end_nm'],
                       closure_m=raw['closure_end_m'],
                       motor104_position=0., motor105_position=0., motor104_velocity=0., motor105_velocity=0.,
                       command_generation104=float(gen), command_generation105=float(gen))
            rows.append(row)
        last = rows[-1]
        meta = dict(schema='ref01_mujoco_system_v1', simulation_only=True,
                    scheduler='controller_manager', scenario=case, mujoco_version='3.17.0',
                    duration_s=duration, target_samples=len(rows), max_samples=len(rows), samples=len(rows),
                    read_count=last['read_count'], write_count=last['write_count'], startup_writes=3,
                    nominal_period_s=.005, physics_step_s=.0005, physics_steps_per_cycle=10,
                    receiver_ttl_ns=60_000_000, first_read_ns=start, finish_ns=last['finish_ns'],
                    runtime_s=(last['finish_ns'] - start) / 1e9, sim_time_s=duration,
                    receiver_latched=fault, receiver_fault='expired' if fault else 'none',
                    receiver_zero_ns=start + 252 * dt if fault else 0,
                    interruption_ns=start + 240 * dt if fault else 0, warnings=0,
                    max_closure_m=max(row['closure_m'] for row in rows),
                    final_q=[last['psi'], last['beta']], final_dq=[last['dpsi'], last['dbeta']],
                    runtime_error='none')
        status = dict(mode=2, reason_name='none', generation=101, source_time_s=100.5,
                      observed_at_monotonic_s=100.5002, compute_ms=.1,
                      q=[0., .3490658503988659], dq=[0., 0.], closure_residual=1e-8,
                      received_ns=100_501_000_000)
        statuses = [status]
        if fault:
            statuses.append(dict(status, mode=3, reason_name='feedback_expired', generation=240,
                                 source_time_s=101.195, observed_at_monotonic_s=101.2652,
                                 received_ns=101_266_000_000))
        self.case, self.duration = case, duration
        self.rows, self.meta, self.status = rows, meta, statuses
        self.complete = dict(complete=True, samples=len(rows), runtime_error='none')
        return self

    def evaluate(self):
        with (self.folder / 'trace.csv').open('w', newline='', encoding='utf-8') as stream:
            writer = csv.DictWriter(stream, fieldnames=sorted(self.rows[0]))
            writer.writeheader()
            writer.writerows(self.rows)
        (self.folder / 'metadata.json').write_text(json.dumps(self.meta), encoding='utf-8')
        (self.folder / 'complete.json').write_text(json.dumps(self.complete), encoding='utf-8')
        return summarize_case(self.folder, self.case, self.duration, 0, self.manager, self.status)

    def test_real_windows_curves_pass_both_axes_with_synthetic_timing(self):
        for case in ('psi', 'beta'):
            with self.subTest(case=case):
                result = self.make_case(case).evaluate()
                self.assertTrue(result['functional_passed'], result['functional'])
                self.assertTrue(result['timing_passed'], result)
                recovery = result['functional']['recovery']
                old = self.numeric['recovery'][case]['native']
                self.assertFalse(recovery['saturation_assessed'])
                self.assertNotIn('unsaturated', recovery['checks'])
                for key in ('peak_deg', 'baseline_max_error_deg', 'settling_after_release_s'):
                    self.assertAlmostEqual(recovery[key], old[key], places=10)

    def test_fault_cases_pass_function_only(self):
        for case in ('loss', 'duplicate'):
            with self.subTest(case=case):
                result = self.make_case(case).evaluate()
                self.assertTrue(result['functional_passed'], result['functional'])
                self.assertIsNone(result['timing_passed'])

    def test_missing_csv_schema_rejected(self):
        self.make_case()
        for row in self.rows:
            row.pop('application_ns')
        with self.assertRaisesRegex(ValueError, 'Missing CSV fields'):
            self.evaluate()

    def test_missing_metadata_schema_rejected(self):
        self.make_case().meta.pop('schema')
        with self.assertRaisesRegex(ValueError, 'metadata schema'):
            self.evaluate()

    def test_false_manager_claim_fails(self):
        self.make_case().manager['checks']['command_interfaces_claimed'] = False
        result = self.evaluate()
        self.assertFalse(result['functional_passed'])
        self.assertFalse(result['functional']['checks']['manager_command_interfaces_claimed'])

    def test_each_missing_required_manager_claim_is_rejected(self):
        self.make_case()
        required = dict(self.manager['checks'])
        for missing in required:
            with self.subTest(missing=missing):
                self.manager['checks'] = {key: value for key, value in required.items() if key != missing}
                with self.assertRaisesRegex(ValueError, 'Missing required manager evidence checks'):
                    self.evaluate()

    def test_empty_or_non_boolean_manager_claims_rejected(self):
        self.make_case()
        for claims in ({}, {'controller_active': 1}):
            self.manager['checks'] = claims
            with self.assertRaisesRegex(ValueError, 'manager evidence'):
                self.evaluate()

    def test_normal_stalled_generation_cannot_pass(self):
        self.make_case()
        for row in self.rows[500:]:
            row['generation'] = 500
            row['command_generation104'] = row['command_generation105'] = 500.
        result = self.evaluate()
        self.assertTrue(result['timing_passed'])
        self.assertFalse(result['functional_passed'])
        self.assertFalse(result['functional']['checks']['fresh_command_each_cycle'])

    def test_period_spike_fails_timing(self):
        self.make_case()
        for row in self.rows[500:]:
            for key in ('cycle_start_ns', 'feedback_ns', 'application_ns', 'finish_ns'):
                row[key] += 8_000_000
        self.meta['finish_ns'] = self.rows[-1]['finish_ns']
        result = self.evaluate()
        self.assertTrue(result['functional_passed'], result['functional'])
        self.assertFalse(result['timing_passed'])
        self.assertFalse(result['checks']['maximum_period'])

    def test_wrong_fault_reason_fails(self):
        self.make_case('loss').status[-1]['reason_name'] = 'compute_deadline'
        self.assertFalse(self.evaluate()['functional']['checks']['feedback_timeout_fault'])

    def test_after_completion_fault_does_not_contaminate_normal(self):
        self.make_case()
        sample = dict(self.status[0], mode=3, reason_name='feedback_expired',
                      observed_at_monotonic_s=130.1, received_ns=130_101_000_000)
        self.status.append(sample)
        result = self.evaluate()
        self.assertTrue(result['functional_passed'], result['functional'])
        self.assertEqual(result['diagnostics']['status_in_measurement_window'], 1)

    def test_in_window_fault_fails_normal_even_if_received_after_completion(self):
        self.make_case()
        self.status.append(dict(self.status[0], mode=3, reason_name='feedback_expired',
                                observed_at_monotonic_s=120.1, received_ns=130_101_000_000))
        result = self.evaluate()
        self.assertFalse(result['functional']['checks']['no_controller_fault_in_window'])

    def test_fault_seen_only_after_completion_cannot_pass(self):
        self.make_case('duplicate').status[-1].update(observed_at_monotonic_s=102.6,
                                                     received_ns=102_601_000_000)
        self.assertFalse(self.evaluate()['functional']['checks']['feedback_timeout_fault'])

    def test_receiver_late_zero_fails(self):
        self.make_case('loss')
        for row in self.rows[252:259]:
            row['actual_force104'] = .01
        self.assertFalse(self.evaluate()['functional']['checks']['receiver_zero_within_80ms'])

    def test_startup_reads_do_not_enter_measurement_window(self):
        self.make_case().meta['first_read_ns'] -= 1_000_000_000
        result = self.evaluate()
        self.assertTrue(result['functional_passed'], result['functional'])
        self.assertAlmostEqual(result['metrics']['duration_s'], 30.0004)

    def test_zero_boundary_comes_from_actual_force_not_guard(self):
        self.make_case('loss')
        self.rows[252]['actual_force104'] = .01
        result = self.evaluate()
        self.assertTrue(result['functional_passed'], result['functional'])
        self.assertAlmostEqual(result['functional']['guard_latched_after_cut_ms'], 60.)
        self.assertAlmostEqual(result['functional']['zero_applied_after_cut_ms'], 65.4)

    def test_returned_feedback_cannot_restart(self):
        self.make_case('duplicate')
        self.rows[-1]['application_ns'] = self.rows[-1]['cycle_start_ns'] + 200_000
        self.rows[-1]['command_accepted'] = 1
        self.assertFalse(self.evaluate()['functional']['checks']['no_restart_after_feedback_returns'])

    def test_feedback_never_returning_cannot_pass(self):
        self.make_case('loss')
        for row in self.rows[330:]:
            row['source_fresh'] = 0
            row['sequence'] = 240
            row['feedback_ns'] = self.rows[239]['feedback_ns']
        result = self.evaluate()
        self.assertFalse(result['functional_passed'])
        self.assertFalse(result['functional']['checks']['feedback_really_returns'])

    def test_changed_payload_during_interruption_cannot_pass(self):
        self.make_case('duplicate').rows[250]['motor104_position'] = .01
        self.assertFalse(self.evaluate()['functional']['checks']['feedback_frozen_during_interruption'])

    def test_nonexpiration_receiver_fault_cannot_pass(self):
        self.make_case('loss').meta['receiver_fault'] = 'generation_regression'
        self.assertFalse(self.evaluate()['functional']['checks']['receiver_expiration_fault'])

    def test_actual_force_tail_must_be_zero(self):
        self.make_case('loss').rows[-1]['actual_force104'] = .01
        self.assertFalse(self.evaluate()['functional']['checks']['final_effort_zero'])

    def test_nonfinite_trace_rejected(self):
        self.make_case().rows[100]['psi'] = float('nan')
        with self.assertRaisesRegex(ValueError, 'Nonfinite CSV'):
            self.evaluate()

    def test_hardware_counts_must_cover_measurement(self):
        self.make_case().meta['read_count'] = 3
        self.assertFalse(self.evaluate()['functional']['checks']['hardware_counts_cover_trace'])

    def test_empty_status_cannot_pass(self):
        self.make_case().status.clear()
        self.assertFalse(self.evaluate()['functional_passed'])

    def test_command_publication_is_not_actual_application(self):
        self.make_case().rows[100]['application_ns'] = 0
        result = self.evaluate()
        self.assertFalse(result['functional']['checks']['actual_application_each_cycle'])
        self.assertFalse(result['timing_passed'])


if __name__ == '__main__':
    unittest.main(verbosity=2)
