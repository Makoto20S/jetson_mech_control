"""PH-EXO candidate software timing acceptance; no ROS or hardware access.

Input schema 1 uses nanoseconds from ONE host's monotonic clock. Each sample
associates the feedback consumed by a control cycle with the instant its effort
is actually applied. Publishing a target is not application. ``measurement``
must describe these instrumented boundaries; hardware latency is not inferred.
Window times are observed wall-clock boundaries, never model/simulation time.

This module separates functional, soft timing and strict 5 ms deadline results.
Its configurable limits are experiment criteria, not a hard-real-time guarantee.
"""
from dataclasses import asdict, dataclass
import math
from statistics import fmean


@dataclass(frozen=True)
class TimingSpec:
    target_period_ns: int = 5_000_000
    min_cycles: int = 6000
    min_duration_ns: int = 30_000_000_000
    mean_min_ms: float = 4.95
    mean_max_ms: float = 5.05
    period_p99_min_ms: float = 4.5
    period_p99_max_ms: float = 5.5
    jitter_p99_max_ms: float = 0.5
    period_max_exclusive_ms: float = 10.0
    latency_p99_max_ms: float = 5.0
    latency_max_exclusive_ms: float = 10.0

    def __post_init__(self):
        if type(self.target_period_ns) is not int or self.target_period_ns != 5_000_000:
            raise ValueError('PH-EXO timing target must remain 5 ms')
        if type(self.min_cycles) is not int or self.min_cycles < 2:
            raise ValueError('min_cycles must be an integer >= 2')
        if type(self.min_duration_ns) is not int or self.min_duration_ns <= 0:
            raise ValueError('min_duration_ns must be a positive integer')
        for name, value in asdict(self).items():
            if name.endswith('_ms'):
                if isinstance(value, bool) or not isinstance(value, (float, int)):
                    raise ValueError(name + ' must be numeric')
                if not math.isfinite(value) or value <= 0:
                    raise ValueError(name + ' must be finite and positive')
        if self.mean_min_ms > self.mean_max_ms:
            raise ValueError('mean limits are reversed')
        if self.period_p99_min_ms > self.period_p99_max_ms:
            raise ValueError('period P99 limits are reversed')


def _p99(values):
    """Linear interpolation, matching NumPy's default percentile convention."""
    ordered = sorted(values)
    index = (len(ordered) - 1) * 0.99
    lo = int(index)
    hi = min(lo + 1, len(ordered) - 1)
    return ordered[lo] + (ordered[hi] - ordered[lo]) * (index - lo)


def _timestamp(value):
    return type(value) is int and value > 0


def evaluate_case(case, functional_passed, trace, spec=None):
    """Return independent functional/timing results for psi, beta, loss or duplicate.

    trace = {schema: 1, clock: 'monotonic_ns_same_host', measurement: <text>,
             window_start_ns: <int>, window_end_ns: <int>, samples: [
                 {cycle_start_ns: <int>, feedback_ns: <int>, application_ns: <int>}]}

    ``timing_passed=None`` means not applicable for a fault-injection case.
    Missing/malformed data always prevent normal timing acceptance.
    """
    spec = spec or TimingSpec()
    result = dict(schema=1, case=case, functional_passed=functional_passed is True,
                  timing_passed=None, timing_applicable=case in ('psi', 'beta'),
                  acceptance_kind='candidate_software_timing',
                  hard_realtime_verified=False, spec=asdict(spec),
                  measurement=None, checks={}, metrics={}, errors=[])
    if type(functional_passed) is not bool:
        result['errors'].append('functional_passed must be an explicit bool')
    if case in ('loss', 'duplicate'):
        result['timing_note'] = 'Fault injection checks function only; excluded from timing acceptance.'
        return result
    result['timing_passed'] = False
    if case not in ('psi', 'beta'):
        result['errors'].append('unknown case')
        return result
    if not isinstance(trace, dict):
        result['errors'].append('missing trace object')
        return result
    result['measurement'] = trace.get('measurement')
    if type(trace.get('schema')) is not int or trace.get('schema') != 1:
        result['errors'].append('trace schema must be 1')
    if trace.get('clock') != 'monotonic_ns_same_host':
        result['errors'].append('one-host monotonic clock required')
    if not isinstance(result['measurement'], str) or not result['measurement'].strip():
        result['errors'].append('actual measurement boundaries must be documented')
    start, end = trace.get('window_start_ns'), trace.get('window_end_ns')
    if not _timestamp(start) or not _timestamp(end) or end <= start:
        result['errors'].append('invalid observed window')
    rows = trace.get('samples')
    if not isinstance(rows, (list, tuple)) or len(rows) < 2:
        result['errors'].append('at least two associated samples required')
    if result['errors']:
        return result
    for i, row in enumerate(rows):
        if not isinstance(row, dict) or not all(
                _timestamp(row.get(key)) for key in
                ('cycle_start_ns', 'feedback_ns', 'application_ns')):
            result['errors'].append('sample %d has missing/non-integer timestamps' % i)
            return result
        cycle, feedback, applied = (row[key] for key in
                                   ('cycle_start_ns', 'feedback_ns', 'application_ns'))
        if not start <= cycle <= applied <= end or feedback > applied:
            result['errors'].append('sample %d has inconsistent timestamp order/window' % i)
            return result
    intervals_ns = [b['cycle_start_ns'] - a['cycle_start_ns']
                    for a, b in zip(rows, rows[1:])]
    intervals = [value / 1e6 for value in intervals_ns]
    latencies = [(row['application_ns'] - row['feedback_ns']) / 1e6 for row in rows]
    jitter = [abs(value - spec.target_period_ns) / 1e6 for value in intervals_ns]
    metrics = dict(cycles=len(rows), intervals=len(intervals), duration_s=(end-start)/1e9,
                   period_mean_ms=fmean(intervals), period_p99_ms=_p99(intervals),
                   period_min_ms=min(intervals), period_max_ms=max(intervals),
                   absolute_jitter_p99_ms=_p99(jitter),
                   feedback_to_application_p99_ms=_p99(latencies),
                   feedback_to_application_max_ms=max(latencies),
                   periods_above_5ms=sum(value > spec.target_period_ns
                                                for value in intervals_ns))
    metrics['fraction_periods_above_5ms'] = (
        metrics['periods_above_5ms'] / len(intervals))
    checks = dict(enough_cycles=len(rows) >= spec.min_cycles,
                  enough_wall_time=end-start >= spec.min_duration_ns,
                  positive_intervals=min(intervals_ns) > 0,
                  mean_period=spec.mean_min_ms <= metrics['period_mean_ms'] <= spec.mean_max_ms,
                  p99_period=spec.period_p99_min_ms <= metrics['period_p99_ms'] <= spec.period_p99_max_ms,
                  p99_absolute_jitter=metrics['absolute_jitter_p99_ms'] <= spec.jitter_p99_max_ms,
                  maximum_period=metrics['period_max_ms'] < spec.period_max_exclusive_ms,
                  p99_feedback_to_application=metrics['feedback_to_application_p99_ms'] <= spec.latency_p99_max_ms,
                  maximum_feedback_to_application=metrics['feedback_to_application_max_ms'] < spec.latency_max_exclusive_ms)
    result.update(metrics=metrics, checks=checks, timing_passed=all(checks.values()))
    return result


def evaluate_suite(case_results):
    """Require one psi and one beta normal run; fault cases cannot substitute.

    Both normal cases must use the same explicit spec. Additional fault cases
    contribute only to functional_passed. Duplicate case names are rejected.
    """
    rows = list(case_results)
    names = [row.get('case') for row in rows]
    normal = [row for row in rows if row.get('case') in ('psi', 'beta')]
    complete = len(names) == len(set(names)) and set(names) >= {'psi', 'beta'}
    complete = complete and all(name in ('psi', 'beta', 'loss', 'duplicate') for name in names)
    valid_schema = all(type(row.get('schema')) is int and row['schema'] == 1
                       and isinstance(row.get('spec'), dict)
                       and row.get('acceptance_kind') == 'candidate_software_timing'
                       and row.get('hard_realtime_verified') is False
                       and row.get('errors') == [] for row in rows)
    matching_spec = (len(normal) == 2 and isinstance(normal[0].get('spec'), dict)
                     and normal[0]['spec'] == normal[1].get('spec'))
    functional = bool(rows) and all(row.get('functional_passed') is True for row in rows)
    timing = (valid_schema and complete and matching_spec
              and all(row.get('timing_passed') is True for row in normal))
    return dict(schema=1, functional_passed=functional, timing_passed=timing,
                passed=functional and timing, normal_axes_complete=complete,
                normal_axes_use_same_spec=matching_spec, valid_result_schema=valid_schema,
                acceptance_kind='candidate_software_timing', hard_realtime_verified=False,
                cases=rows)
