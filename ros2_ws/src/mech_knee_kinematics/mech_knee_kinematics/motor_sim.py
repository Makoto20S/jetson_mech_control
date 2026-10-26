"""Hardware-free dual-IMU knee angle to motor-velocity control trial."""

import argparse
import csv
from dataclasses import dataclass
import json
import math

from .offline_loop import Measurement, _shank_quaternion, calibrated_estimator


@dataclass(frozen=True)
class VelocityConfig:
    sample_timeout_s: float = 0.3
    angle_min_deg: float = -10.0
    angle_max_deg: float = 120.0
    rate_limit_dps: float = 350.0
    tracking_gain_per_s: float = 3.0
    damping: float = 0.1
    max_velocity_rad_s: float = 1.5
    max_slew_rad_s2: float = 1.0

    def __post_init__(self):
        if (not all(math.isfinite(value) for value in vars(self).values()) or
                self.sample_timeout_s <= 0 or
                self.angle_min_deg >= self.angle_max_deg or
                self.rate_limit_dps <= 0 or
                self.tracking_gain_per_s < 0 or self.damping < 0 or
                self.max_velocity_rad_s <= 0 or self.max_slew_rad_s2 <= 0):
            raise ValueError('invalid simulated velocity controller configuration')


class KneeVelocityFollower:
    """Angle-error velocity law; any lost/invalid IMU latches zero until reset."""

    def __init__(self, config=VelocityConfig()):
        self.config = config
        self.state = 'disarmed'
        self.reason = 'not armed'
        self.last_sample_time = None
        self.last_command_time = None
        self.last_velocity_rad_s = 0.0

    def _valid(self, sample, now):
        return (sample is not None and sample.valid and
                all(math.isfinite(value) for value in
                    (now, sample.time_s, sample.angle_deg, sample.rate_dps)) and
                0 <= now - sample.time_s <= self.config.sample_timeout_s and
                self.config.angle_min_deg <= sample.angle_deg <=
                self.config.angle_max_deg and
                abs(sample.rate_dps) <= self.config.rate_limit_dps)

    def arm(self, sample, now):
        if self.state != 'disarmed' or not self._valid(sample, now):
            return False
        self.state = 'armed'
        self.reason = ''
        self.last_sample_time = sample.time_s
        self.last_command_time = now
        self.last_velocity_rad_s = 0.0
        return True

    def reset(self):
        self.__init__(self.config)

    def _fault(self, reason):
        self.state = 'fault'
        self.reason = reason
        self.last_velocity_rad_s = 0.0
        return 0.0

    def command(self, now, sample, target_deg, target_rate_dps):
        if self.state != 'armed':
            return 0.0
        if not self._valid(sample, now):
            return self._fault('IMU sample missing, stale or invalid')
        if (sample.time_s <= self.last_sample_time or
                now <= self.last_command_time):
            return self._fault('non-monotonic IMU or control time')
        if (not math.isfinite(target_deg) or
                not math.isfinite(target_rate_dps) or
                not self.config.angle_min_deg <= target_deg <=
                self.config.angle_max_deg or
                abs(target_rate_dps) > self.config.rate_limit_dps):
            return self._fault('invalid knee reference')
        desired = (math.radians(target_rate_dps) +
                   self.config.tracking_gain_per_s *
                   math.radians(target_deg - sample.angle_deg) -
                   self.config.damping * math.radians(sample.rate_dps))
        desired = max(-self.config.max_velocity_rad_s,
                      min(self.config.max_velocity_rad_s, desired))
        step = self.config.max_slew_rad_s2 * (now - self.last_command_time)
        velocity = max(self.last_velocity_rad_s - step,
                       min(self.last_velocity_rad_s + step, desired))
        self.last_sample_time = sample.time_s
        self.last_command_time = now
        self.last_velocity_rad_s = velocity
        return velocity

    def tick(self, now):
        if self.state == 'armed' and (not math.isfinite(now) or
                                      now < self.last_command_time or
                                      now - self.last_sample_time >
                                      self.config.sample_timeout_s):
            self._fault('IMU sample timeout')
        return self.last_velocity_rad_s


class VirtualMotor:
    """First-order velocity actuator fixture, not a DM4340 dynamics model."""

    def __init__(self, time_constant_s=0.12):
        self.position_rad = 0.0
        self.velocity_rad_s = 0.0
        self.time_constant_s = time_constant_s

    def step(self, command_rad_s, dt):
        alpha = 1.0 - math.exp(-dt / self.time_constant_s)
        self.velocity_rad_s += alpha * (command_rad_s - self.velocity_rad_s)
        self.position_rad += self.velocity_rad_s * dt


def reference(time_s):
    frequency_hz = 0.25
    angle_deg = 15.0 * (1.0 - math.cos(2.0 * math.pi * frequency_hz * time_s))
    rate_dps = 15.0 * 2.0 * math.pi * frequency_hz * math.sin(
        2.0 * math.pi * frequency_hz * time_s)
    return angle_deg, rate_dps


def run_trial(duration_s=8.0, dropout=None):
    if not math.isfinite(duration_s) or duration_s <= 0:
        raise ValueError('duration_s must be positive and finite')
    estimator = calibrated_estimator()
    follower = KneeVelocityFollower()
    motor = VirtualMotor()
    dt = 0.01
    rows = []
    for index in range(int(duration_s / dt)):
        now = index * dt
        target_deg, target_rate_dps = reference(now)
        valid = dropout is None or not dropout[0] <= now < dropout[1]
        measurement = None
        if valid:
            estimate = estimator.update(
                (1.0, 0.0, 0.0, 0.0), _shank_quaternion(motor.position_rad),
                now, (0.0, 0.0, 0.0),
                (0.0, math.degrees(motor.velocity_rad_s), 0.0))
            measurement = Measurement(now, estimate.angle_deg,
                                      estimate.angular_velocity_dps,
                                      estimate.within_limits and
                                      estimate.orientation_measurement_valid)
        if follower.state == 'disarmed' and measurement is not None:
            follower.arm(measurement, now)
            velocity_command = 0.0
        else:
            velocity_command = follower.command(
                now, measurement, target_deg, target_rate_dps)
        rows.append({
            'time_s': round(now, 4), 'target_deg': target_deg,
            'true_angle_deg': math.degrees(motor.position_rad),
            'estimated_angle_deg': measurement.angle_deg if measurement else '',
            'motor_velocity_target_rad_s': velocity_command,
            'motor_velocity_feedback_rad_s': motor.velocity_rad_s,
            'imu_valid': int(valid), 'controller_state': follower.state,
        })
        motor.step(velocity_command, dt)
    return rows


def main(args=None):
    parser = argparse.ArgumentParser(description='Offline simulated-IMU motor velocity loop')
    parser.add_argument('--csv', help='optional simulation trace')
    options = parser.parse_args(args)
    normal = run_trial()
    lost = run_trial(dropout=(4.0, 4.25))
    if options.csv:
        with open(options.csv, 'w', newline='', encoding='utf-8') as output:
            writer = csv.DictWriter(output, fieldnames=['trial', *normal[0]])
            writer.writeheader()
            for name, rows in (('normal', normal), ('imu_dropout', lost)):
                writer.writerows({'trial': name, **row} for row in rows)
    rmse = math.sqrt(sum((row['target_deg'] - row['true_angle_deg']) ** 2
                         for row in normal) / len(normal))
    print(json.dumps({
        'simulated_tracking_rmse_deg': rmse,
        'max_velocity_target_rad_s': max(abs(row['motor_velocity_target_rad_s'])
                                       for row in normal),
        'imu_dropout_fault_latched': lost[-1]['controller_state'] == 'fault',
        'dropout_post_fault_max_target_rad_s': max(
            abs(row['motor_velocity_target_rad_s']) for row in lost
            if row['time_s'] >= 4.0),
    }, indent=2))


if __name__ == '__main__':
    main()
