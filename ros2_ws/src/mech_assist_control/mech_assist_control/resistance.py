"""Bounded dissipative knee resistance policy, independent of motor transport."""

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class ResistanceConfig:
    damping_nm_s_rad: float = 0.5
    max_torque_nm: float = 0.3
    velocity_deadband_rad_s: float = 0.02
    max_slew_nm_s: float = 1.0
    sample_timeout_s: float = 0.2
    diagnostic_timeout_s: float = 0.3
    run_limit_s: float = 10.0
    angle_min_rad: float = -0.175
    angle_max_rad: float = 2.094
    max_velocity_rad_s: float = 2.0

    def __post_init__(self):
        values = vars(self).values()
        if (not all(math.isfinite(value) for value in values) or
                self.damping_nm_s_rad <= 0 or
                not 0 < self.max_torque_nm <= 2.0 or
                not 0 <= self.velocity_deadband_rad_s < 0.2 or
                self.max_slew_nm_s <= 0 or
                self.sample_timeout_s <= 0 or
                self.diagnostic_timeout_s <= 0 or
                not 0 < self.run_limit_s <= 60 or
                self.angle_min_rad >= self.angle_max_rad or
                self.max_velocity_rad_s <= 0):
            raise ValueError('invalid resistance configuration')


class ResistanceLogic:
    def __init__(self, config=None):
        self.config = config or ResistanceConfig()
        self.state = 'disarmed'
        self.reason = ''
        self.effort_nm = 0.0
        self.sample = None
        self.diagnostic_time = None
        self.diagnostic_ok = False
        self.arm_time = None
        self.last_tick = None

    def _fault(self, reason):
        self.state = 'fault'
        self.reason = reason
        self.effort_nm = 0.0

    def update_diagnostic(self, healthy, now):
        if not math.isfinite(now):
            self._fault('invalid diagnostic time')
            return
        self.diagnostic_time = now
        self.diagnostic_ok = bool(healthy)
        if self.state == 'armed' and not healthy:
            self._fault('knee diagnostic unhealthy')

    def update_knee(self, angle_rad, velocity_rad_s, now, sampled_at=None):
        sample_time = now if sampled_at is None else sampled_at
        c = self.config
        if (not all(math.isfinite(value) for value in
                    (angle_rad, velocity_rad_s, now, sample_time)) or sample_time > now or
                not c.angle_min_rad <= angle_rad <= c.angle_max_rad or
                abs(velocity_rad_s) > c.max_velocity_rad_s or
                (self.sample is not None and sample_time <= self.sample[2])):
            self.sample = None
            if self.state == 'armed':
                self._fault('invalid or nonmonotonic knee sample')
            return
        self.sample = (angle_rad, velocity_rad_s, sample_time)

    def _fresh(self, now):
        return (self.sample is not None and self.diagnostic_ok and
                self.diagnostic_time is not None and
                0 <= now - self.sample[2] <= self.config.sample_timeout_s and
                0 <= now - self.diagnostic_time <= self.config.diagnostic_timeout_s)

    def arm(self, now):
        if self.state != 'disarmed' or not math.isfinite(now) or not self._fresh(now):
            return False
        self.state = 'armed'
        self.reason = ''
        self.arm_time = now
        self.last_tick = now
        self.effort_nm = 0.0
        return True

    def tick(self, now):
        if self.state != 'armed':
            return 0.0
        if (not math.isfinite(now) or now < self.last_tick or
                now - self.arm_time >= self.config.run_limit_s or
                not self._fresh(now)):
            self._fault('trial limit or knee input timeout')
            return 0.0
        velocity = self.sample[1]
        magnitude = min(self.config.max_torque_nm,
                        self.config.damping_nm_s_rad * max(
                            0.0, abs(velocity) - self.config.velocity_deadband_rad_s))
        target = -math.copysign(magnitude, velocity) if magnitude else 0.0
        # A sign change or stop must not leave a torque that drives the motion.
        if target == 0.0 or self.effort_nm * velocity > 0:
            self.effort_nm = 0.0
        step = self.config.max_slew_nm_s * (now - self.last_tick)
        self.effort_nm += max(-step, min(step, target - self.effort_nm))
        self.last_tick = now
        return self.effort_nm

    def reset(self):
        self.__init__(self.config)


def map_knee_effort_to_motor(knee_effort_nm, motor_to_knee_scale,
                             motor_velocity_rad_s, max_motor_effort_nm):
    """Map joint torque by virtual work; never drive the measured motor motion."""
    values = (knee_effort_nm, motor_to_knee_scale, motor_velocity_rad_s,
              max_motor_effort_nm)
    if (not all(math.isfinite(value) for value in values) or
            motor_to_knee_scale == 0 or max_motor_effort_nm <= 0):
        raise ValueError('invalid motor effort mapping')
    effort = knee_effort_nm * motor_to_knee_scale
    if effort * motor_velocity_rad_s > 0:
        return 0.0
    return max(-max_motor_effort_nm, min(max_motor_effort_nm, effort))
