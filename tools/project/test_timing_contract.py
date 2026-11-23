"""Synthetic fault injection tests, not performance evidence for a host."""
import copy
from dataclasses import replace
import unittest

from timing_contract import TimingSpec, evaluate_case, evaluate_suite


def trace(cycles=6000, period_ns=5_000_000, latency_ns=1_000_000):
    start = 100_000_000_000
    rows = [dict(cycle_start_ns=start+i*period_ns, feedback_ns=start+i*period_ns,
                 application_ns=start+i*period_ns+latency_ns) for i in range(cycles)]
    return dict(schema=1, clock='monotonic_ns_same_host',
                measurement='synthetic feedback generation -> synthetic effort application',
                window_start_ns=start,
                window_end_ns=max(start+cycles*period_ns, rows[-1]['application_ns']),
                samples=rows)


class TimingContractTests(unittest.TestCase):
    def test_complete_two_axis_suite_and_fault_cases(self):
        results = [evaluate_case(axis, True, trace()) for axis in ('psi', 'beta')]
        results += [evaluate_case(case, True, None) for case in ('loss', 'duplicate')]
        report = evaluate_suite(results)
        self.assertTrue(report['passed'])
        self.assertFalse(report['hard_realtime_verified'])
        self.assertIsNone(results[-1]['timing_passed'])

    def test_latency_failure_cannot_hide_behind_good_period(self):
        result = evaluate_case('psi', True, trace(latency_ns=6_000_000))
        self.assertTrue(result['functional_passed'])
        self.assertTrue(result['checks']['mean_period'])
        self.assertFalse(result['timing_passed'])
        self.assertFalse(result['checks']['p99_feedback_to_application'])

    def test_single_stall_rejected_even_when_p99_and_mean_pass(self):
        data = trace()
        for row in data['samples'][3000:]:
            for key in row:
                row[key] += 5_000_000
        data['window_end_ns'] += 5_000_000
        result = evaluate_case('psi', True, data)
        self.assertTrue(result['checks']['p99_period'])
        self.assertTrue(result['checks']['mean_period'])
        self.assertFalse(result['checks']['maximum_period'])
        self.assertEqual(result['metrics']['periods_above_5ms'], 1)

    def test_single_ten_ms_application_rejects_only_bad_axis(self):
        data = trace()
        data['samples'][100]['application_ns'] = data['samples'][100]['feedback_ns']+10_000_000
        beta = evaluate_case('beta', True, data)
        self.assertTrue(beta['checks']['p99_feedback_to_application'])
        self.assertFalse(beta['checks']['maximum_feedback_to_application'])
        self.assertFalse(evaluate_suite([evaluate_case('psi', True, trace()), beta])['timing_passed'])

    def test_deadline_misses_reported_without_claiming_hard_realtime(self):
        result = evaluate_case('psi', True, trace(period_ns=5_010_000))
        self.assertTrue(result['timing_passed'])
        self.assertEqual(result['metrics']['periods_above_5ms'], 5999)
        self.assertFalse(result['hard_realtime_verified'])

    def test_function_and_timing_fail_independently(self):
        result = evaluate_case('psi', False, trace())
        self.assertFalse(result['functional_passed'])
        self.assertTrue(result['timing_passed'])
        self.assertFalse(evaluate_suite([result, evaluate_case('beta', True, trace())])['passed'])

    def test_missing_axis_or_fault_case_cannot_supply_timing(self):
        psi = evaluate_case('psi', True, trace())
        fault = evaluate_case('loss', True, trace())
        self.assertFalse(evaluate_suite([psi, fault])['timing_passed'])
        self.assertFalse(evaluate_suite([psi, psi, evaluate_case('beta', True, trace())])['timing_passed'])

    def test_model_clock_and_malformed_source_are_rejected(self):
        for key, value in [('clock', 'simulation'), ('schema', 2), ('measurement', '')]:
            data = trace()
            data[key] = value
            result = evaluate_case('psi', True, data)
            self.assertFalse(result['timing_passed'])
            self.assertTrue(result['errors'])
        for value in (float('nan'), float('inf'), None, True):
            data = trace()
            data['samples'][7]['feedback_ns'] = value
            self.assertFalse(evaluate_case('psi', True, data)['timing_passed'])
        data = trace()
        del data['samples'][100]['application_ns']
        self.assertFalse(evaluate_case('psi', True, data)['timing_passed'])

    def test_summary_without_schema_cannot_pass_a_suite(self):
        rows = [dict(case=axis, functional_passed=True, timing_passed=True)
                for axis in ('psi', 'beta')]
        self.assertFalse(evaluate_suite(rows)['timing_passed'])

    def test_insufficient_duration_and_samples_are_independent(self):
        short = trace(cycles=5999)
        short['window_end_ns'] += 10_000_000
        result = evaluate_case('psi', True, short)
        self.assertTrue(result['checks']['enough_wall_time'])
        self.assertFalse(result['checks']['enough_cycles'])
        fast = evaluate_case('psi', True, trace(period_ns=4_900_000))
        self.assertTrue(fast['checks']['enough_cycles'])
        self.assertFalse(fast['checks']['enough_wall_time'])

    def test_duplicate_cycles_and_future_feedback_fail(self):
        data = trace()
        data['samples'][100] = copy.deepcopy(data['samples'][99])
        result = evaluate_case('psi', True, data)
        self.assertFalse(result['checks']['positive_intervals'])
        self.assertFalse(result['timing_passed'])
        data = trace()
        data['samples'][0]['feedback_ns'] = data['samples'][0]['application_ns']+1
        self.assertFalse(evaluate_case('psi', True, data)['timing_passed'])

    def test_thresholds_are_explicit_and_shared_between_axes(self):
        strict = replace(TimingSpec(), latency_p99_max_ms=0.5)
        self.assertFalse(evaluate_case('psi', True, trace(), strict)['timing_passed'])
        psi = evaluate_case('psi', True, trace())
        beta = evaluate_case('beta', True, trace(latency_ns=100_000), strict)
        self.assertTrue(beta['timing_passed'])
        self.assertFalse(evaluate_suite([psi, beta])['timing_passed'])
        with self.assertRaises(ValueError):
            TimingSpec(target_period_ns=10_000_000)
        with self.assertRaises(ValueError):
            TimingSpec(latency_p99_max_ms=float('nan'))


if __name__ == '__main__':
    unittest.main()
