"""Pure-stdlib timing tests; no ROS installation or hardware needed."""
import csv
import gc
import json
import math
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from timing_trace import COLUMNS, MISSING, MAX_CAPACITY, PhaseGrid, TimingTrace, milliseconds_to_ns


class PhaseGridTests(unittest.TestCase):
    def test_preanchored_phase_preserves_first_callback_lateness(self):
        grid = PhaseGrid(10)
        grid.anchor_phase(100)
        self.assertEqual(grid.observe(104), (100, 0, 0))
        self.assertEqual(grid.observe(111), (110, 0, 1))
        self.assertEqual(grid.phase_ns, 100)

    def test_preanchored_first_callback_can_skip_multiple_periods(self):
        grid = PhaseGrid(10)
        grid.anchor_phase(100)
        self.assertEqual(grid.observe(135), (130, 3, 3))
        self.assertEqual(grid.skipped_total, 3)
        self.assertEqual(grid.observe(141), (140, 0, 4))

    def test_preanchored_small_early_error_does_not_reanchor_or_skip(self):
        grid = PhaseGrid(10_000_000)
        grid.anchor_phase(100_000_000)
        self.assertEqual(grid.observe(99_999_900), (100_000_000, 0, 0))
        self.assertEqual(grid.observe(110_000_050), (110_000_000, 0, 1))

    def test_phase_cannot_be_overwritten_after_anchor_or_first_callback(self):
        for anchor_first in (True, False):
            grid = PhaseGrid(10)
            if anchor_first:
                grid.anchor_phase(100)
            else:
                grid.observe(100)
            with self.assertRaises(ValueError):
                grid.anchor_phase(200)

    def test_first_observation_establishes_fixed_phase(self):
        grid = PhaseGrid(10)
        self.assertEqual(grid.observe(100), (100, 0, 0))
        self.assertEqual(grid.observe(111), (110, 0, 1))
        self.assertEqual(grid.observe(122), (120, 0, 2))
        self.assertEqual(grid.phase_ns, 100)

    def test_late_callback_counts_skipped_grid_slots_without_reanchoring(self):
        grid = PhaseGrid(10)
        grid.observe(100)
        self.assertEqual(grid.observe(135), (130, 2, 3))
        self.assertEqual(grid.observe(141), (140, 0, 4))
        self.assertEqual(grid.skipped_total, 2)

    def test_early_callback_does_not_reuse_a_slot(self):
        grid = PhaseGrid(10)
        grid.observe(100)
        self.assertEqual(grid.observe(109), (110, 0, 1))
        self.assertEqual(grid.observe(119), (120, 0, 2))

    def test_clock_regression_is_counted_without_resetting_phase(self):
        grid = PhaseGrid(10)
        grid.observe(100)
        grid.observe(110)
        self.assertEqual(grid.observe(105), (120, 0, 2))
        self.assertEqual(grid.clock_regressions, 1)
        self.assertEqual(grid.phase_ns, 100)

    def test_invalid_period_is_rejected(self):
        for value in (0, -1, 1.5, True):
            with self.subTest(value=value), self.assertRaises(ValueError):
                PhaseGrid(value)


class TraceTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.path = Path(self.directory.name) / 'trace.csv'
        self.recorders = []

    def tearDown(self):
        for recorder in self.recorders:
            recorder.flush()
        self.directory.cleanup()

    def recorder(self, capacity=10, capture_gc=False):
        trace = TimingTrace(str(self.path), period_ns=10_000_000,
                            capacity_rows=capacity, capture_gc=capture_gc)
        self.recorders.append(trace)
        return trace

    @staticmethod
    def state(trace, start):
        trace.record_state(start, start + 5, start + 20, start + 30,
                           1_700_000_000_000_000_123, 1001, 1002,
                           False, True, True, 10, 20, 30, 40)

    def read(self):
        with self.path.open(encoding='utf-8', newline='') as stream:
            rows = list(csv.DictReader(stream))
        meta = json.loads(Path(str(self.path) + '.meta.json').read_text(encoding='utf-8'))
        return rows, meta

    def test_default_disabled_has_no_buffer_hook_or_disk_output(self):
        hooks = len(gc.callbacks)
        trace = TimingTrace()
        trace.record_imu(1, 100, 10, 20)
        self.state(trace, 100)
        trace.flush()
        self.assertFalse(trace.enabled)
        self.assertIsNone(trace.buffer)
        self.assertIsNone(trace.path)
        self.assertEqual(len(gc.callbacks), hooks)
        self.assertEqual(trace.count, 0)

    def test_csv_integer_nanoseconds_preserve_values_above_float_precision(self):
        trace = self.recorder()
        exact = 1_700_000_000_000_000_123
        trace.record_imu(2, exact, 123456789123456789, 123456789123456799)
        self.assertFalse(self.path.exists())
        trace.flush()
        rows, meta = self.read()
        self.assertEqual(rows[0]['source_stamp_ns'], str(exact))
        self.assertEqual(rows[0]['entry_ns'], '123456789123456789')
        self.assertEqual(rows[0]['expected_ns'], '')
        self.assertEqual(rows[0]['index'], '2')
        self.assertEqual(meta['rows_written'], 1)
        self.assertTrue(meta['completed'])
        self.assertFalse(meta['overflow'])

    def test_state_columns_preserve_snapshot_publish_boundary_and_flags(self):
        trace = self.recorder()
        self.state(trace, 100)
        trace.flush()
        rows, meta = self.read()
        self.assertEqual(tuple(rows[0]), COLUMNS)
        self.assertEqual(rows[0]['event'], 'state')
        self.assertEqual(rows[0]['start_ns'], '100')
        self.assertEqual(rows[0]['entry_ns'], '100')
        self.assertEqual(rows[0]['snapshot_ns'], '105')
        self.assertEqual(rows[0]['publish_begin_ns'], '120')
        self.assertEqual(rows[0]['end_ns'], '130')
        self.assertEqual(rows[0]['joints_valid'], '0')
        self.assertEqual(rows[0]['imu1_valid'], '1')
        self.assertEqual(rows[0]['imu1_stamp_ns'], '1001')
        self.assertEqual(rows[0]['imu1_receive_age_ns'], '10')
        self.assertEqual(meta['age_reference'], 'snapshot_ns; not end_ns')

    def test_capacity_stops_storage_counts_overflow_and_keeps_first_rows(self):
        trace = self.recorder(capacity=2)
        self.state(trace, 100)
        trace.record_imu(1, 101, 102, 103)
        self.state(trace, 35_000_100)
        trace.record_imu(2, 105, 106, 107)
        trace.flush()
        rows, meta = self.read()
        self.assertEqual(len(rows), 2)
        self.assertEqual([row['event'] for row in rows], ['state', 'imu'])
        self.assertEqual(meta['rows_dropped'], 2)
        self.assertTrue(meta['overflow'])
        self.assertEqual(meta['observed_event_counts'], {'imu': 2, 'state': 2, 'gc': 0})
        self.assertEqual(meta['skipped_periods_total'], 2)
        self.assertEqual(meta['buffer_bytes'], 2 * len(COLUMNS) * 8)

    def test_phase_and_skips_are_written_as_integers(self):
        trace = self.recorder()
        self.state(trace, 100)
        self.state(trace, 35_000_100)
        trace.flush()
        rows, meta = self.read()
        self.assertEqual(rows[1]['expected_ns'], '30000100')
        self.assertEqual(rows[1]['skipped_periods'], '2')
        self.assertEqual(rows[1]['slot_index'], '3')
        self.assertEqual(meta['first_phase_ns'], 100)
        self.assertIn('not OS timer release', meta['phase_origin'])
        self.assertFalse(meta['deadline_reference_available'])
        self.assertIsNone(meta['anchor_uncertainty_ns'])

    def test_timer_anchor_records_raw_brackets_and_uncertainty(self):
        trace = self.recorder()
        trace.anchor_phase(10_000_100, 20, 90, 110, 10_000_000)
        self.state(trace, 10_001_100)
        trace.flush()
        rows, meta = self.read()
        self.assertEqual(rows[0]['expected_ns'], '10000100')
        self.assertEqual(meta['first_phase_ns'], 10_000_100)
        self.assertEqual(meta['anchor_uncertainty_ns'], 20)
        self.assertEqual(meta['anchor_mono_before_ns'], 90)
        self.assertEqual(meta['anchor_mono_after_ns'], 110)
        self.assertEqual(meta['anchor_until_ns'], 10_000_000)
        self.assertTrue(meta['deadline_reference_available'])
        self.assertIn('time_until_next_call', meta['phase_origin'])

    def test_invalid_anchor_does_not_partially_establish_phase(self):
        trace = self.recorder()
        for phase, uncertainty in ((-1, 1), (1.5, 1), (1, -1), (1, True)):
            with self.subTest(phase=phase, uncertainty=uncertainty), self.assertRaises(ValueError):
                trace.anchor_phase(phase, uncertainty)
        self.assertIsNone(trace.grid.phase_ns)
        self.assertFalse(trace.deadline_reference_available)

    def test_missing_ages_are_empty_and_negative_finite_age_is_retained(self):
        self.assertEqual(milliseconds_to_ns(math.nan), MISSING)
        self.assertEqual(milliseconds_to_ns(math.inf), MISSING)
        self.assertEqual(milliseconds_to_ns(-0.123456), -123456)
        trace = self.recorder()
        trace.record_state(100, 105, 120, 130, 1000, 0, 0, False, False, False,
                           MISSING, MISSING, MISSING, -100)
        trace.flush()
        rows, _ = self.read()
        self.assertEqual(rows[0]['imu1_receive_age_ns'], '')
        self.assertEqual(rows[0]['imu2_source_age_ns'], '-100')

    def test_flush_is_idempotent_and_no_rows_added_after_flush(self):
        trace = self.recorder()
        trace.record_imu(1, 1, 2, 3)
        trace.flush()
        first = self.path.read_bytes()
        trace.record_imu(2, 4, 5, 6)
        self.state(trace, 20)
        trace.flush()
        self.assertEqual(self.path.read_bytes(), first)
        self.assertEqual(trace.count, 1)

    def test_existing_output_is_never_overwritten(self):
        self.path.write_text('preserved', encoding='utf-8')
        with self.assertRaises(FileExistsError):
            self.recorder()
        self.assertEqual(self.path.read_text(encoding='utf-8'), 'preserved')

    def test_bad_capacity_and_relative_path_are_rejected_before_allocation(self):
        for capacity in (0, -1, MAX_CAPACITY + 1, True, 2.5):
            with self.subTest(capacity=capacity), self.assertRaises(ValueError):
                TimingTrace(str(self.path), capacity_rows=capacity)
        with self.assertRaises(ValueError):
            TimingTrace('relative.csv', capacity_rows=1)
        self.assertLess(MAX_CAPACITY * len(COLUMNS) * 8, 1_000_000_000)

    def test_gc_hook_is_removed_at_flush(self):
        trace = self.recorder(capture_gc=True)
        self.assertIn(trace.gc_callback, gc.callbacks)
        trace.flush()
        self.assertNotIn(trace.gc_callback, gc.callbacks)
        _, meta = self.read()
        self.assertTrue(meta['gc_hook_enabled'])

    def test_gc_interval_uses_same_monotonic_clock_and_generation_index(self):
        trace = self.recorder()
        with patch('timing_trace.time.monotonic_ns', side_effect=[100, 180]):
            trace._gc_event('start', {'generation': 2})
            trace._gc_event('stop', {'generation': 2})
        trace.flush()
        rows, _ = self.read()
        self.assertEqual(rows[0]['event'], 'gc')
        self.assertEqual(rows[0]['index'], '2')
        self.assertEqual(rows[0]['entry_ns'], '100')
        self.assertEqual(rows[0]['end_ns'], '180')


if __name__ == '__main__':
    unittest.main()
