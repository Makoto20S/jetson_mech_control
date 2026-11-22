# SPDX-License-Identifier: LGPL-2.1-only
"""PH-EXO ContraKin-derived streaming adaptive oscillator, software evaluation.

Numerical equations adapted from Louis Devillez's ContraKin-RT AO:
https://github.com/ldevillez/ContraKin-RT, commit
6ff1be354055674c926d60b3e2ce4d6810a8b3c3, src/ao_estimator.py (LGPL 2.1).
Changes: standard-library implementation, pre-division guard, bounded frequency,
explicit rad/s normalization, timestamp validation, and conservative state gate.
The original source remains unchanged in the evaluation vendor directory.

phase_rad is oscillator phase, NOT calibrated heel-strike gait phase. valid means
the software tracking gates passed; it is not permission to actuate a robot.
Call update/check_timeout in one serialized context and one monotonic time domain.
"""
from collections import deque
from dataclasses import dataclass, fields
import math


@dataclass(frozen=True)
class AOConfig:
    n_harmonics: int = 2
    input_scale_rad_s: float = 1.0
    input_offset_rad_s: float = 0.0
    max_input_rad_s: float = 20.0
    initial_frequency_hz: float = 1.0
    min_frequency_hz: float = 0.2
    max_frequency_hz: float = 3.0
    nu_omega: float = 20.0
    nu_phi: float = 22.0
    eta: float = 2.0
    denominator_floor: float = 0.03
    max_dt_s: float = 0.05
    timeout_s: float = 0.15
    integration_step_s: float = 0.005
    window_s: float = 0.6
    motion_std_rad_s: float = 0.06
    acquire_s: float = 3.0
    stable_hold_s: float = 0.6
    max_frequency_std_hz: float = 0.12
    max_relative_residual: float = 0.65

    def __post_init__(self):
        if isinstance(self.n_harmonics, bool) or not isinstance(self.n_harmonics, int) or not 1 <= self.n_harmonics <= 8:
            raise ValueError('n_harmonics must be an integer in [1, 8]')
        for f in fields(self):
            value = getattr(self, f.name)
            if not math.isfinite(value):
                raise ValueError(f'{f.name} must be finite')
            if f.name not in ('input_offset_rad_s',) and value <= 0:
                raise ValueError(f'{f.name} must be positive')
        if not self.min_frequency_hz < self.initial_frequency_hz < self.max_frequency_hz:
            raise ValueError('initial frequency must lie strictly within limits')
        if not self.integration_step_s <= self.max_dt_s <= self.timeout_s:
            raise ValueError('require integration_step_s <= max_dt_s <= timeout_s')
        if self.window_s < self.max_dt_s or self.acquire_s < self.window_s:
            raise ValueError('require max_dt_s <= window_s <= acquire_s')


def _std(values):
    mean = sum(values) / len(values)
    return math.sqrt(sum((v - mean) ** 2 for v in values) / len(values))


class AdaptiveOscillator:
    def __init__(self, config=None):
        self.config = config if config is not None else AOConfig()
        self._last_stamp = None
        self._state = 'ACQUIRING'
        self._reason = 'awaiting_input'
        self._reset_model()

    def _reset_model(self):
        c = self.config
        self._omega = 2 * math.pi * c.initial_frequency_hz
        self._alphas = [1.0] + [0.0] * c.n_harmonics
        self._phis = [0.0] * c.n_harmonics
        self._window = deque(maxlen=4096)
        self._motion_start = None
        self._good_since = None
        self._residual = 0.0
        self._estimate = c.input_offset_rad_s + c.input_scale_rad_s
        self._signal_std = 0.0
        self._frequency_std = 0.0
        self._relative_residual = 0.0

    def _snapshot(self, age_s=0.0):
        c = self.config
        return dict(phase_rad=self._phis[0] % (2 * math.pi),
                    frequency_hz=self._omega / (2 * math.pi),
                    amplitude=c.input_scale_rad_s * math.sqrt(sum(a * a for a in self._alphas[1:])),
                    residual=self._residual, estimate_rad_s=self._estimate,
                    valid=self._state == 'TRACKING', state=self._state,
                    reason=self._reason, stamp_s=self._last_stamp,
                    input_age_s=max(0.0, age_s), signal_std_rad_s=self._signal_std,
                    frequency_std_hz=self._frequency_std,
                    relative_residual=self._relative_residual)

    def _invalid(self, reason):
        self._reset_model()
        self._state, self._reason = 'INVALID', reason
        return self._snapshot()

    def update(self, value, timestamp_s):
        """Accept one angular-velocity sample in rad/s with monotonic seconds.

        Invalid samples never advance the last accepted timestamp. Following a
        timestamp gap, a finite timestamp is accepted as a new anchor, but valid
        remains false until fresh motion and convergence gates pass again.
        """
        c = self.config
        if not isinstance(timestamp_s, (int, float)) or not math.isfinite(timestamp_s):
            return self._invalid('nonfinite_timestamp')
        if not isinstance(value, (int, float)) or not math.isfinite(value):
            return self._invalid('nonfinite_input')
        if abs(value) > c.max_input_rad_s:
            return self._invalid('input_out_of_range')
        if self._last_stamp is None:
            self._last_stamp = float(timestamp_s)
            self._window.append((timestamp_s, value, 0.0, self._omega / (2 * math.pi)))
            self._state, self._reason = 'ACQUIRING', 'initializing'
            return self._snapshot()
        dt = float(timestamp_s) - self._last_stamp
        if dt <= 0:
            return self._invalid('nonmonotonic_timestamp')
        self._last_stamp = float(timestamp_s)
        if dt > c.max_dt_s:
            return self._invalid('sample_gap')
        if self._state == 'INVALID':
            self._state, self._reason = 'ACQUIRING', 'recovering'
        x = (value - c.input_offset_rad_s) / c.input_scale_rad_s
        if not math.isfinite(x) or abs(x) > 100.0:
            return self._invalid('normalization_out_of_range')
        n_steps = max(1, math.ceil(dt / c.integration_step_s - 1e-10))
        h = dt / n_steps
        for _ in range(n_steps):
            estimate = self._alphas[0] + sum(a * math.sin(p) for a, p in zip(self._alphas[1:], self._phis))
            error = x - estimate
            denom = sum(self._alphas)
            # Check BEFORE dividing; original implementation checked afterward.
            correction = error / denom if abs(denom) >= c.denominator_floor else 0.0
            old_phis = self._phis[:]
            self._phis = [(p + h * ((i + 1) * self._omega + c.nu_phi * correction * math.cos(p))) % (2 * math.pi)
                          for i, p in enumerate(old_phis)]
            self._omega = min(2 * math.pi * c.max_frequency_hz,
                              max(2 * math.pi * c.min_frequency_hz,
                                  self._omega + h * c.nu_omega * correction * math.cos(old_phis[0])))
            for i, p in enumerate(old_phis):
                self._alphas[i + 1] += h * c.eta * error * math.sin(p)
            self._alphas[0] += h * c.eta * error
        estimate = self._alphas[0] + sum(a * math.sin(p) for a, p in zip(self._alphas[1:], self._phis))
        self._estimate = estimate * c.input_scale_rad_s + c.input_offset_rad_s
        self._residual = value - self._estimate
        if not all(math.isfinite(v) for v in self._alphas + self._phis + [self._omega, self._residual]):
            return self._invalid('numerical_failure')
        self._window.append((timestamp_s, value, self._residual, self._omega / (2 * math.pi)))
        while self._window and timestamp_s - self._window[0][0] > c.window_s + c.max_dt_s:
            self._window.popleft()
        covered = self._window[-1][0] - self._window[0][0]
        if covered < c.window_s:
            self._state, self._reason = 'ACQUIRING', 'collecting_window'
            return self._snapshot()
        self._signal_std = _std([r[1] for r in self._window])
        self._frequency_std = _std([r[3] for r in self._window])
        self._relative_residual = math.sqrt(sum(r[2] ** 2 for r in self._window) / len(self._window)) / max(self._signal_std, c.motion_std_rad_s)
        if self._signal_std < c.motion_std_rad_s:
            self._state, self._reason = 'STOPPED', 'insufficient_motion'
            self._motion_start = None
            self._good_since = None
            # Keep the learned state reset while the window shows no motion;
            # subsequent motion must establish a fresh acquisition interval.
            self._omega = 2 * math.pi * c.initial_frequency_hz
            self._alphas = [x] + [0.0] * c.n_harmonics
            self._phis = [0.0] * c.n_harmonics
            return self._snapshot()
        if self._motion_start is None:
            self._motion_start = timestamp_s
        freq = self._omega / (2 * math.pi)
        good = (self._frequency_std <= c.max_frequency_std_hz and
                self._relative_residual <= c.max_relative_residual and
                c.min_frequency_hz + 1e-6 < freq < c.max_frequency_hz - 1e-6)
        if good:
            if self._good_since is None:
                self._good_since = timestamp_s
        else:
            self._good_since = None
        acquired = (timestamp_s - self._motion_start >= c.acquire_s and
                    self._good_since is not None and timestamp_s - self._good_since >= c.stable_hold_s)
        self._state = 'TRACKING' if acquired else 'ACQUIRING'
        self._reason = 'tracking_gate_passed' if acquired else 'waiting_for_convergence'
        return self._snapshot()

    def check_timeout(self, now_s):
        """Call periodically even when no samples arrive; same clock as update."""
        if not isinstance(now_s, (int, float)) or not math.isfinite(now_s):
            return self._invalid('nonfinite_clock')
        if self._last_stamp is None:
            return self._snapshot()
        age = now_s - self._last_stamp
        if age < 0:
            return self._invalid('clock_moved_backward')
        if age > self.config.timeout_s:
            self._invalid('input_timeout')
        return self._snapshot(age)
