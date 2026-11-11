"""Opt-in bounded timing capture; no ROS dependency and no callback disk I/O.

All timestamps ending in _ns are integers. Monotonic timestamps share Linux
CLOCK_MONOTONIC with the C++ bridge; ROS stamps are only message join keys.
"""
from array import array
import csv
import gc
import json
import math
from pathlib import Path
import time
import weakref


MISSING = -(1 << 63)
DEFAULT_CAPACITY = 2_000_000
MAX_CAPACITY = 3_000_000
COLUMNS = (
    'event', 'index', 'source_stamp_ns', 'entry_ns', 'expected_ns',
    'start_ns', 'snapshot_ns', 'publish_begin_ns', 'end_ns', 'state_stamp_ns',
    'imu1_stamp_ns', 'imu2_stamp_ns', 'joints_valid', 'imu1_valid', 'imu2_valid',
    'imu1_receive_age_ns', 'imu2_receive_age_ns', 'imu1_source_age_ns',
    'imu2_source_age_ns', 'skipped_periods', 'slot_index',
)


def milliseconds_to_ns(value):
    """Preserve unavailable age as an empty CSV field, not a false zero."""
    return round(value * 1_000_000) if math.isfinite(value) else MISSING


class PhaseGrid:
    """Fixed timer-API grid, with an explicitly unverified callback fallback."""
    def __init__(self, period_ns):
        if isinstance(period_ns, bool) or not isinstance(period_ns, int) or period_ns <= 0:
            raise ValueError('period_ns must be a positive integer')
        self.period_ns = period_ns
        self.phase_ns = None
        self.next_slot = 0
        self.last_start_ns = None
        self.skipped_total = 0
        self.clock_regressions = 0

    def anchor_phase(self, phase_ns):
        """Set the expected first deadline before callbacks; never reanchor."""
        if self.phase_ns is not None:
            raise ValueError('Timing phase is already established')
        if isinstance(phase_ns, bool) or not isinstance(phase_ns, int) or phase_ns < 0:
            raise ValueError('phase_ns must be a nonnegative integer')
        self.phase_ns = phase_ns

    def observe(self, start_ns):
        if self.phase_ns is None:
            self.phase_ns = start_ns
        if self.last_start_ns is not None and start_ns < self.last_start_ns:
            self.clock_regressions += 1
        slot = max(self.next_slot, (start_ns - self.phase_ns) // self.period_ns)
        skipped = slot - self.next_slot
        expected = self.phase_ns + slot * self.period_ns
        self.next_slot = slot + 1
        self.last_start_ns = start_ns
        self.skipped_total += skipped
        return expected, skipped, slot


class TimingTrace:
    """Single-executor recorder with a hard 504 MB raw-buffer upper bound.

    Default: 2 million rows / 336 MB, covering one hour at <= 555 total events/s
(two 200 Hz IMU callbacks plus 100 Hz state = 500 events/s). Higher rates or
longer runs stop storing at capacity and explicitly count dropped rows.
"""
    def __init__(self, path='', period_ns=10_000_000, capacity_rows=DEFAULT_CAPACITY,
                 capture_gc=True):
        self.enabled = bool(path)
        self.path = Path(path) if self.enabled else None
        self.grid = PhaseGrid(period_ns)
        self.count = 0
        self.dropped = 0
        self.event_counts = {'imu': 0, 'state': 0, 'gc': 0}
        self._flush_attempted = False
        self.capacity = 0
        self.buffer = None
        self.gc_callback = None
        self.gc_starts = [None, None, None]
        self.phase_origin = 'first_observed_state_callback; estimated running phase, not OS timer release'
        self.anchor_uncertainty_ns = None
        self.anchor_mono_before_ns = None
        self.anchor_mono_after_ns = None
        self.anchor_until_ns = None
        self.anchor_error = None
        self.deadline_reference_available = False
        if not self.enabled:
            return
        if not self.path.is_absolute():
            raise ValueError('timing_trace_path must be absolute')
        self.meta_path = Path(str(self.path) + '.meta.json')
        if self.path.exists() or self.meta_path.exists():
            raise FileExistsError('Timing output already exists; use a fresh run path')
        if (isinstance(capacity_rows, bool) or not isinstance(capacity_rows, int)
                or not 1 <= capacity_rows <= MAX_CAPACITY):
            raise ValueError('timing_capacity_rows must be in [1, 3000000]')
        self.capacity = capacity_rows
        self.buffer = array('q', [MISSING]) * (capacity_rows * len(COLUMNS))
        if self.buffer.itemsize != 8:
            raise RuntimeError('Timing trace requires signed 64-bit storage')
        if capture_gc:
            reference = weakref.ref(self)
            def callback(phase, info):
                recorder = reference()
                if recorder is not None:
                    recorder._gc_event(phase, info)
            self.gc_callback = callback
            gc.callbacks.append(callback)

    def anchor_phase(self, phase_ns, uncertainty_ns, mono_before_ns=None,
                     mono_after_ns=None, until_ns=None):
        """Anchor to midpoint + timer.time_until_next_call() on one host.

        The full monotonic bracket width is retained as a conservative timing
        uncertainty; this is a ROS timer API estimate, not a kernel trace.
        """
        if not self.enabled:
            return
        if (isinstance(uncertainty_ns, bool) or not isinstance(uncertainty_ns, int)
                or uncertainty_ns < 0):
            raise ValueError('uncertainty_ns must be a nonnegative integer')
        self.grid.anchor_phase(phase_ns)
        self.phase_origin = 'ros_timer_time_until_next_call_bracket; midpoint_plus_until'
        self.anchor_uncertainty_ns = uncertainty_ns
        self.anchor_mono_before_ns = mono_before_ns
        self.anchor_mono_after_ns = mono_after_ns
        self.anchor_until_ns = until_ns
        self.deadline_reference_available = True

    def _gc_event(self, phase, info):
        generation = info.get('generation', -1)
        if generation not in (0, 1, 2):
            return
        timestamp = time.monotonic_ns()
        if phase == 'start':
            self.gc_starts[generation] = timestamp
        elif phase == 'stop' and self.gc_starts[generation] is not None:
            start = self.gc_starts[generation]
            self.gc_starts[generation] = None
            self._append('gc', (3, generation, MISSING, start, MISSING,
                               MISSING, MISSING, MISSING, timestamp, MISSING,
                               MISSING, MISSING, MISSING, MISSING, MISSING,
                               MISSING, MISSING, MISSING, MISSING, MISSING, MISSING))

    def _append(self, event, row):
        if not self.enabled or self._flush_attempted:
            return
        self.event_counts[event] += 1
        if self.count == self.capacity:
            self.dropped += 1
            return
        offset = self.count * len(COLUMNS)
        # Avoid creating GC-tracked iterators after reserving the row offset:
        # a gc callback is allowed to append its own row between Python calls.
        i = 0
        while i < len(COLUMNS):
            self.buffer[offset + i] = row[i]
            i += 1
        self.count += 1

    def record_imu(self, index, source_stamp_ns, entry_ns, end_ns):
        if not self.enabled:
            return
        self._append('imu', (1, index, source_stamp_ns, entry_ns, MISSING,
                            MISSING, MISSING, MISSING, end_ns, MISSING, MISSING, MISSING,
                            MISSING, MISSING, MISSING, MISSING, MISSING, MISSING,
                            MISSING, MISSING, MISSING))

    def record_state(self, start_ns, snapshot_ns, publish_begin_ns, end_ns, state_stamp_ns,
                     imu1_stamp_ns, imu2_stamp_ns, joints_valid, imu1_valid, imu2_valid,
                     imu1_receive_age_ns, imu2_receive_age_ns,
                     imu1_source_age_ns, imu2_source_age_ns):
        if not self.enabled or self._flush_attempted:
            return
        expected, skipped, slot = self.grid.observe(start_ns)
        self._append('state', (2, MISSING, MISSING, start_ns, expected,
                              start_ns, snapshot_ns, publish_begin_ns, end_ns, state_stamp_ns,
                              imu1_stamp_ns, imu2_stamp_ns, int(joints_valid),
                              int(imu1_valid), int(imu2_valid),
                              imu1_receive_age_ns, imu2_receive_age_ns,
                              imu1_source_age_ns, imu2_source_age_ns, skipped, slot))

    def flush(self):
        """Write once after callbacks stop. Existing output is never overwritten."""
        if not self.enabled or self._flush_attempted:
            return
        if self.gc_callback is not None and self.gc_callback in gc.callbacks:
            gc.callbacks.remove(self.gc_callback)
        self._flush_attempted = True
        self.path.parent.mkdir(parents=True, exist_ok=True)
        with self.path.open('x', encoding='utf-8', newline='') as stream:
            writer = csv.writer(stream)
            writer.writerow(COLUMNS)
            width = len(COLUMNS)
            for row_index in range(self.count):
                row = self.buffer[row_index * width:(row_index + 1) * width]
                values = ['' if value == MISSING else value for value in row]
                values[0] = {1: 'imu', 2: 'state', 3: 'gc'}[row[0]]
                writer.writerow(values)
        metadata = {
            'schema_version': 1, 'completed': True, 'columns': list(COLUMNS),
            'clock': 'time.monotonic_ns / Linux CLOCK_MONOTONIC',
            'ros_stamps': 'message join keys; not hardware acquisition timestamps',
            'phase_origin': self.phase_origin,
            'anchor_uncertainty_ns': self.anchor_uncertainty_ns,
            'anchor_mono_before_ns': self.anchor_mono_before_ns,
            'anchor_mono_after_ns': self.anchor_mono_after_ns,
            'anchor_until_ns': self.anchor_until_ns,
            'anchor_error': self.anchor_error,
            'deadline_reference_available': self.deadline_reference_available,
            'first_phase_ns': self.grid.phase_ns, 'period_ns': self.grid.period_ns,
            'slot_rule': 'max(previous_slot+1, floor((start_ns-first_phase_ns)/period_ns))',
            'skipped_periods_total': self.grid.skipped_total,
            'clock_regressions': self.grid.clock_regressions,
            'rows_written': self.count, 'rows_dropped': self.dropped,
            'overflow': self.dropped > 0, 'observed_event_counts': self.event_counts,
            'capacity_rows': self.capacity,
            'buffer_bytes': self.capacity * len(COLUMNS) * self.buffer.itemsize,
            'age_reference': 'snapshot_ns; not end_ns',
            'end_reference': 'core.imu return / publisher.publish return, before trace insertion',
            'gc_hook_enabled': self.gc_callback is not None,
            'output': str(self.path),
        }
        with self.meta_path.open('x', encoding='utf-8') as stream:
            json.dump(metadata, stream, indent=2, allow_nan=False)
            stream.write('\n')
