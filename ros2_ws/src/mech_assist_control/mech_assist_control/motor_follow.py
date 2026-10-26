"""Knee-angle follower with explicit motor-feedback and diagnostic gates."""

import math

from mech_knee_kinematics.motor_sim import KneeVelocityFollower, VelocityConfig, reference
from mech_knee_kinematics.offline_loop import Measurement


class MotorFollowLogic:
    def __init__(self, motor_mode=False, motor_to_knee_scale=0.0,
                 motor_to_knee_offset_rad=0.0, max_mismatch_rad=0.15,
                 feedback_timeout_s=0.1, amplitude_deg=20.0,
                 motor_run_limit_s=5.0, bench_simulation=False):
        if (motor_mode and (not math.isfinite(motor_to_knee_scale) or
                            abs(motor_to_knee_scale) < 1e-6)):
            raise ValueError('motor mode requires a measured motor-to-knee scale')
        if (not all(math.isfinite(value) for value in
                    (motor_to_knee_offset_rad, max_mismatch_rad,
                     feedback_timeout_s, amplitude_deg, motor_run_limit_s)) or
                max_mismatch_rad <= 0 or feedback_timeout_s <= 0 or
                not 0 < amplitude_deg <= (45 if bench_simulation else 30) or
                not 0 < motor_run_limit_s <= (15 if bench_simulation else 10)):
            raise ValueError('invalid motor follower configuration')
        self.motor_mode = motor_mode
        self.scale = motor_to_knee_scale
        self.offset = motor_to_knee_offset_rad
        self.max_mismatch_rad = max_mismatch_rad
        self.feedback_timeout_s = feedback_timeout_s
        self.amplitude_deg = amplitude_deg
        self.motor_run_limit_s = motor_run_limit_s
        self.follower = KneeVelocityFollower(VelocityConfig(
            sample_timeout_s=0.15 if motor_mode else 0.3))
        self.knee = None
        self.diagnostic_time = None
        self.diagnostic_ok = False
        self.motor = None
        self.arm_time = None
        self.start_angle_deg = None
        self.target_motor_rad_s = 0.0
        self._last_motor_command_time = None

    @property
    def state(self):
        return self.follower.state

    @property
    def reason(self):
        return self.follower.reason

    def _fault(self, reason):
        self.target_motor_rad_s = 0.0
        self.follower._fault(reason)

    def update_diagnostic(self, healthy, now):
        self.diagnostic_ok = healthy
        self.diagnostic_time = now
        if self.state == 'armed' and not healthy:
            self._fault('knee diagnostic unhealthy')

    def update_motor(self, position_rad, velocity_rad_s, now):
        if (not all(math.isfinite(value) for value in
                    (position_rad, velocity_rad_s, now)) or
                abs(velocity_rad_s) > 2.0):
            self.motor = None
            if self.state == 'armed':
                self._fault('invalid motor feedback')
            return
        self.motor = (position_rad, velocity_rad_s, now)

    def _inputs_ok(self, now):
        timeout = self.follower.config.sample_timeout_s
        if (not self.diagnostic_ok or self.diagnostic_time is None or
                not 0 <= now - self.diagnostic_time <= timeout or
                not self.follower._valid(self.knee, now)):
            return False
        if not self.motor_mode:
            return True
        if (self.motor is None or
                not 0 <= now - self.motor[2] <= self.feedback_timeout_s):
            return False
        mapped_angle = self.scale * self.motor[0] + self.offset
        return abs(mapped_angle - math.radians(self.knee.angle_deg)) <= self.max_mismatch_rad

    def update_knee(self, angle_rad, velocity_rad_s, now, sampled_at=None):
        sample_time = now if sampled_at is None else sampled_at
        if not all(math.isfinite(value) for value in
                   (angle_rad, velocity_rad_s, now, sample_time)) or sample_time > now:
            if self.state == 'armed':
                self._fault('invalid knee measurement')
            return
        self.knee = Measurement(sample_time, math.degrees(angle_rad),
                                math.degrees(velocity_rad_s), True)
        if self.state != 'armed':
            return
        if not self._inputs_ok(now):
            self._fault('knee diagnostic, sample or motor feedback unavailable')
            return
        phase_time = now - self.arm_time
        angle_offset, rate = reference(phase_time)
        ratio = self.amplitude_deg / 30.0
        knee_velocity = self.follower.command(
            now, self.knee, self.start_angle_deg + ratio * angle_offset,
            ratio * rate)
        if self.state != 'armed':
            self.target_motor_rad_s = 0.0
        else:
            raw_motor_velocity = (knee_velocity / self.scale if self.motor_mode
                                  else knee_velocity)
            limit = self.follower.config.max_velocity_rad_s
            desired = max(-limit, min(limit, raw_motor_velocity))
            max_step = self.follower.config.max_slew_rad_s2 * (
                now - self._last_motor_command_time)
            self.target_motor_rad_s = max(self.target_motor_rad_s - max_step,
                                          min(self.target_motor_rad_s + max_step,
                                              desired))
            self._last_motor_command_time = now

    def arm(self, now):
        if self.state != 'disarmed' or not self._inputs_ok(now):
            return False
        if self.knee.angle_deg + self.amplitude_deg > self.follower.config.angle_max_deg:
            return False
        if not self.follower.arm(self.knee, now):
            return False
        self.arm_time = now
        self.start_angle_deg = self.knee.angle_deg
        self.target_motor_rad_s = 0.0
        self._last_motor_command_time = now
        return True

    def tick(self, now):
        if self.state == 'armed':
            if self.motor_mode and now - self.arm_time >= self.motor_run_limit_s:
                self._fault('motor trial time limit reached')
            elif not self._inputs_ok(now):
                self._fault('knee or motor feedback timeout/mismatch')
            else:
                self.follower.tick(now)
                if self.state != 'armed':
                    self.target_motor_rad_s = 0.0
        return self.target_motor_rad_s

    def reset(self):
        self.follower.reset()
        self.knee = None
        self.diagnostic_time = None
        self.diagnostic_ok = False
        self.motor = None
        self.arm_time = None
        self.start_angle_deg = None
        self.target_motor_rad_s = 0.0
        self._last_motor_command_time = None
