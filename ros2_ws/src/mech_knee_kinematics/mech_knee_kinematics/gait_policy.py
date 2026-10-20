"""Event-locked virtual knee assistance; never produces motor commands."""

from dataclasses import dataclass
import math

from .offline_loop import LoopConfig, Measurement, VirtualAssistSupervisor


@dataclass(frozen=True)
class GaitConfig:
    sample_timeout_s: float = 0.3
    min_period_s: float = 0.6
    max_period_s: float = 2.5
    stance_fraction: float = 0.65
    contact_threshold: float = 20.0
    reference_offset_deg: float = 10.0
    reference_amplitude_deg: float = 20.0

    def __post_init__(self):
        if (not all(math.isfinite(value) for value in vars(self).values()) or
                not 0.1 <= self.sample_timeout_s <= 2.0 or
                not 0.1 < self.min_period_s < self.max_period_s or
                not 0.0 < self.stance_fraction < 1.0 or
                self.contact_threshold <= 0.0 or
                not -10.0 <= self.reference_offset_deg <= 120.0 or
                self.reference_amplitude_deg < 0.0 or
                self.reference_offset_deg + self.reference_amplitude_deg > 120.0):
            raise ValueError('invalid gait simulation configuration')


class GaitShadow:
    """Uses knee feedback and validated foot events for bounded virtual effort."""

    def __init__(self, config=GaitConfig(), pressure_validated=False):
        self.config = config
        self.pressure_validated = pressure_validated
        self.supervisor = VirtualAssistSupervisor(
            LoopConfig(sample_timeout_s=config.sample_timeout_s))
        self.knee = None
        self.pressure = None
        self.diagnostic_time = None
        self.diagnostic_ok = False
        self.last_heel_time = None
        self.period_s = None
        self.effort_nm = 0.0
        self.phase = None

    @property
    def state(self):
        return self.supervisor.state

    @property
    def reason(self):
        return self.supervisor.reason

    def _fault(self, reason):
        self.phase = None
        self.effort_nm = self.supervisor._fault(reason)
        return self.effort_nm

    def update_diagnostic(self, healthy, now):
        self.diagnostic_ok = healthy
        self.diagnostic_time = now
        if not healthy and self.state == 'armed':
            self._fault('knee diagnostic not healthy')

    def invalidate_pressure(self, reason):
        self.pressure = None
        self.effort_nm = 0.0
        self.supervisor.last_effort_nm = 0.0
        if self.state == 'armed':
            self._fault(reason)

    def update_pressure(self, phase, total, now):
        if (not isinstance(phase, int) or phase not in range(4) or
                not math.isfinite(total) or total < 0.0 or
                not math.isfinite(now) or
                (self.pressure is not None and now <= self.pressure[2])):
            self.invalidate_pressure('invalid or non-monotonic foot sample')
            return False
        previous = self.pressure[0] if self.pressure is not None else None
        self.pressure = (phase, total, now)
        if phase == 1 and previous == 0:
            if self.last_heel_time is not None:
                period = now - self.last_heel_time
                if not self.config.min_period_s <= period <= self.config.max_period_s:
                    self.period_s = None
                    if self.state == 'armed':
                        self._fault('heel-strike period outside simulation range')
                else:
                    self.period_s = (period if self.period_s is None else
                                     0.5 * (self.period_s + period))
            self.last_heel_time = now
        if phase == 0 or total < self.config.contact_threshold:
            self.effort_nm = 0.0
            self.supervisor.last_effort_nm = 0.0
        if phase != 0 and total < self.config.contact_threshold and self.state == 'armed':
            self._fault('foot phase and pressure disagree')
        return True

    def _healthy(self, now):
        timeout = self.config.sample_timeout_s
        return (self.diagnostic_ok and self.diagnostic_time is not None and
                0.0 <= now - self.diagnostic_time <= timeout and
                self.pressure_validated and self.pressure is not None and
                0.0 <= now - self.pressure[2] <= timeout and
                self.supervisor._valid_sample(self.knee, now) and
                self.period_s is not None and self.last_heel_time is not None and
                0.0 <= now - self.last_heel_time <= self.config.max_period_s and
                (self.pressure[0] == 0 or
                 self.pressure[1] >= self.config.contact_threshold))

    def arm(self, now):
        if not self._healthy(now) or self.pressure[0] == 0:
            return False
        if self.supervisor.arm(self.knee, now):
            self.effort_nm = 0.0
            return True
        return False

    def reset(self):
        self.supervisor.reset()
        self.knee = None
        self.pressure = None
        self.diagnostic_time = None
        self.diagnostic_ok = False
        self.last_heel_time = None
        self.period_s = None
        self.effort_nm = 0.0
        self.phase = None

    def update_knee(self, angle_deg, rate_dps, now):
        self.knee = Measurement(now, angle_deg, rate_dps, True)
        if self.state != 'armed':
            return
        if not self._healthy(now):
            self._fault('knee, diagnostic, gait period or foot sample unavailable')
            return
        self.phase = min(1.0, (now - self.last_heel_time) / self.period_s)
        target = (self.config.reference_offset_deg +
                  0.5 * self.config.reference_amplitude_deg *
                  (1.0 - math.cos(2.0 * math.pi * self.phase)))
        scale = (math.sin(math.pi * self.phase / self.config.stance_fraction)
                 if self.pressure[0] != 0 and
                 self.phase < self.config.stance_fraction else 0.0)
        self.effort_nm = self.supervisor.command(now, self.knee, target, scale)

    def tick(self, now):
        if self.state == 'armed' and not self._healthy(now):
            self._fault('knee, diagnostic, gait period or foot sample timeout')
        return self.effort_nm
