"""Deterministic, hardware-free knee estimation and virtual effort loop."""

import argparse
import csv
from dataclasses import dataclass
import json
import math

from .dual_imu_knee import (
    KneeEstimator, calibrate_hinge_axis_from_gyroscopes,
    relative_quaternion,
)


@dataclass(frozen=True)
class Measurement:
    time_s: float
    angle_deg: float
    rate_dps: float
    valid: bool


@dataclass(frozen=True)
class PressureMeasurement:
    time_s: float
    left_total: float
    right_total: float
    valid: bool


@dataclass(frozen=True)
class LoopConfig:
    sample_timeout_s: float = 0.05
    angle_min_deg: float = -10.0
    angle_max_deg: float = 120.0
    rate_limit_dps: float = 350.0
    max_effort_nm: float = 0.35
    max_slew_nm_per_s: float = 2.0
    assist_kp_nm_per_rad: float = 0.8
    assist_kd_nm_s_per_rad: float = 0.45

    def __post_init__(self):
        values = tuple(vars(self).values())
        if (not all(math.isfinite(value) for value in values) or
                self.sample_timeout_s <= 0.0 or
                self.angle_min_deg >= self.angle_max_deg or
                self.rate_limit_dps <= 0.0 or self.max_effort_nm <= 0.0 or
                self.max_slew_nm_per_s <= 0.0 or
                self.assist_kp_nm_per_rad < 0.0 or
                self.assist_kd_nm_s_per_rad < 0.0):
            raise ValueError('invalid virtual-loop configuration')


class VirtualAssistSupervisor:
    """Produces virtual effort only; fault recovery requires reset and re-arm."""

    def __init__(self, config=LoopConfig()):
        self.config = config
        self.state = 'disarmed'
        self.reason = 'not armed'
        self.last_sample_time = None
        self.last_command_time = None
        self.last_effort_nm = 0.0

    def _valid_sample(self, sample, now):
        return (sample is not None and sample.valid and
                all(math.isfinite(value) for value in
                    (sample.time_s, sample.angle_deg, sample.rate_dps)) and
                0.0 <= now - sample.time_s <= self.config.sample_timeout_s and
                self.config.angle_min_deg <= sample.angle_deg <=
                self.config.angle_max_deg and
                abs(sample.rate_dps) <= self.config.rate_limit_dps)

    def arm(self, sample, now):
        if self.state != 'disarmed' or not self._valid_sample(sample, now):
            return False
        self.state = 'armed'
        self.reason = ''
        self.last_sample_time = sample.time_s
        self.last_command_time = now
        self.last_effort_nm = 0.0
        return True

    def reset(self):
        self.state = 'disarmed'
        self.reason = 'not armed'
        self.last_sample_time = None
        self.last_command_time = None
        self.last_effort_nm = 0.0

    def _fault(self, reason):
        self.state = 'fault'
        self.reason = reason
        self.last_effort_nm = 0.0
        return 0.0

    def command(self, now, sample, target_deg, scale=1.0):
        if self.state != 'armed':
            return 0.0
        if (not math.isfinite(now) or not math.isfinite(target_deg) or
                not math.isfinite(scale) or not 0.0 <= scale <= 1.0):
            return self._fault('non-finite time or target')
        if now < self.last_command_time:
            return self._fault('non-monotonic control time')
        if not self.config.angle_min_deg <= target_deg <= self.config.angle_max_deg:
            return self._fault('target outside virtual knee range')
        if sample is None:
            self.last_effort_nm = 0.0
            if now - self.last_sample_time > self.config.sample_timeout_s:
                return self._fault('IMU sample timeout')
            return 0.0
        if not self._valid_sample(sample, now):
            return self._fault('invalid or out-of-range knee measurement')
        if sample.time_s <= self.last_sample_time or now <= self.last_command_time:
            return self._fault('non-monotonic sample or control time')
        dt = now - self.last_command_time
        error_rad = math.radians(target_deg - sample.angle_deg)
        rate_rad_s = math.radians(sample.rate_dps)
        desired = scale * (self.config.assist_kp_nm_per_rad * error_rad -
                           self.config.assist_kd_nm_s_per_rad * rate_rad_s)
        desired = max(-self.config.max_effort_nm,
                      min(self.config.max_effort_nm, desired))
        change = self.config.max_slew_nm_per_s * dt
        effort = (0.0 if scale == 0.0 else
                  max(self.last_effort_nm - change,
                      min(self.last_effort_nm + change, desired)))
        self.last_effort_nm = effort
        self.last_sample_time = sample.time_s
        self.last_command_time = now
        return effort


class VirtualFootGatedAssist:
    """Simulation-only pressure gate around the existing virtual effort law."""

    def __init__(self, config=LoopConfig(), contact_threshold=20.0):
        if not math.isfinite(contact_threshold) or contact_threshold <= 0.0:
            raise ValueError('invalid virtual contact threshold')
        self.supervisor = VirtualAssistSupervisor(config)
        self.contact_threshold = contact_threshold

    @property
    def state(self):
        return self.supervisor.state

    def reset(self):
        self.supervisor.reset()

    def _valid_pressure(self, pressure, now):
        return (pressure is not None and pressure.valid and
                all(math.isfinite(value) for value in
                    (pressure.time_s, pressure.left_total, pressure.right_total)) and
                pressure.left_total >= 0.0 and pressure.right_total >= 0.0 and
                0.0 <= now - pressure.time_s <= self.supervisor.config.sample_timeout_s)

    def _contact(self, pressure):
        return (pressure.left_total + pressure.right_total >=
                self.contact_threshold)

    def arm(self, sample, pressure, now):
        if not self._valid_pressure(pressure, now) or not self._contact(pressure):
            return False
        return self.supervisor.arm(sample, now)

    def command(self, now, sample, pressure, target_deg):
        if self.state != 'armed':
            return 0.0
        if not self._valid_pressure(pressure, now):
            return self.supervisor._fault('FSR measurement missing or invalid')
        effort = self.supervisor.command(now, sample, target_deg)
        if not self._contact(pressure):
            self.supervisor.last_effort_nm = 0.0
            return 0.0
        return effort


class VirtualKnee:
    """A one-DOF test fixture, not a human knee or a motor model."""

    def __init__(self):
        self.angle_rad = 0.0
        self.rate_rad_s = 0.0

    def step(self, target_deg, effort_nm, dt):
        error = math.radians(target_deg) - self.angle_rad
        human_effort = max(-0.45, min(0.45,
                           1.0 * error - 0.15 * self.rate_rad_s))
        acceleration = (human_effort + effort_nm -
                        0.25 * self.rate_rad_s) / 0.18
        self.rate_rad_s += acceleration * dt
        self.angle_rad += self.rate_rad_s * dt


def _shank_quaternion(angle_rad):
    return (math.cos(angle_rad / 2.0), 0.0,
            math.sin(angle_rad / 2.0), 0.0)


def calibrated_estimator():
    estimator = KneeEstimator()
    samples = []
    for index in range(100):
        t = index * 0.01
        angle = math.radians(30.0) * math.sin(2.0 * math.pi * t)
        rate = 30.0 * 2.0 * math.pi * math.cos(2.0 * math.pi * t)
        samples.append(((1.0, 0.0, 0.0, 0.0), _shank_quaternion(angle),
                        (0.0, 0.0, 0.0), (0.0, rate, 0.0)))
    axes = calibrate_hinge_axis_from_gyroscopes(samples)
    estimator.set_hinge_axes(axes.proximal_axis, axes.distal_axis)
    estimator.calibrate_zero([relative_quaternion(
        (1.0, 0.0, 0.0, 0.0), _shank_quaternion(0.0))] * 20)
    return estimator


def _target(t):
    return 30.0 + 25.0 * math.sin(2.0 * math.pi * 0.45 * t)


def run_trial(assisted, duration_s=8.0, dropout=None, pressure_gated=False,
              pressure_dropout=None, contact_off=None):
    if not math.isfinite(duration_s) or duration_s < 0.01:
        raise ValueError('duration_s must be at least 0.01 and finite')
    if not pressure_gated and (pressure_dropout is not None or contact_off is not None):
        raise ValueError('pressure scenarios require pressure_gated=True')
    estimator = calibrated_estimator()
    supervisor = VirtualFootGatedAssist() if pressure_gated else VirtualAssistSupervisor()
    plant = VirtualKnee()
    dt = 0.01
    rows = []
    for index in range(int(duration_s / dt)):
        now = index * dt
        target = _target(now)
        sensor_valid = dropout is None or not dropout[0] <= now < dropout[1]
        pressure = None
        if pressure_gated and (pressure_dropout is None or
                               not pressure_dropout[0] <= now < pressure_dropout[1]):
            contact = contact_off is None or not contact_off[0] <= now < contact_off[1]
            pressure = PressureMeasurement(now, 100.0 if contact else 0.0,
                                           0.0, True)
        measurement = None
        if sensor_valid:
            estimate = estimator.update(
                (1.0, 0.0, 0.0, 0.0), _shank_quaternion(plant.angle_rad), now,
                (0.0, 0.0, 0.0), (0.0, math.degrees(plant.rate_rad_s), 0.0),
            )
            measurement = Measurement(now, estimate.angle_deg,
                                      estimate.angular_velocity_dps,
                                      estimate.within_limits and
                                      estimate.orientation_measurement_valid)
        if pressure_gated:
            just_armed = (assisted and supervisor.state == 'disarmed' and
                          measurement is not None and
                          supervisor.arm(measurement, pressure, now))
            effort = (supervisor.command(now, measurement, pressure, target)
                      if assisted and not just_armed else 0.0)
        else:
            just_armed = (assisted and supervisor.state == 'disarmed' and
                          measurement is not None and supervisor.arm(measurement, now))
            effort = (supervisor.command(now, measurement, target)
                      if assisted and not just_armed else 0.0)
        rows.append({
            'time_s': round(now, 4), 'target_deg': target,
            'true_angle_deg': math.degrees(plant.angle_rad),
            'estimated_angle_deg': (measurement.angle_deg if measurement else ''),
            'virtual_effort_nm': effort, 'imu_valid': int(sensor_valid),
            'supervisor_state': supervisor.state,
        })
        if pressure_gated:
            rows[-1]['fsr_valid'] = int(pressure is not None)
            rows[-1]['foot_contact'] = int(
                pressure is not None and supervisor._contact(pressure))
        plant.step(target, effort, dt)
    return rows


def tracking_rmse(rows, before_s=None):
    selected = [row for row in rows if before_s is None or row['time_s'] < before_s]
    return math.sqrt(sum((row['target_deg'] - row['true_angle_deg']) ** 2
                         for row in selected) / len(selected))


def save_plot(trials, path):
    from PIL import Image, ImageDraw

    image = Image.new('RGB', (1100, 680), '#ffffff')
    draw = ImageDraw.Draw(image)
    left, right = 78, 1050
    panels = ((75, 310, -10.0, 70.0), (385, 610, -0.4, 0.4))

    def point(index, value, panel):
        top, bottom, minimum, maximum = panel
        x = left + (right - left) * index / (len(trials['baseline']) - 1)
        y = bottom - (bottom - top) * (value - minimum) / (maximum - minimum)
        return x, y

    for panel, ticks in zip(panels, ((0, 20, 40, 60), (-0.4, -0.2, 0, 0.2, 0.4))):
        top, bottom, minimum, maximum = panel
        for tick in ticks:
            y = point(0, tick, panel)[1]
            draw.line((left, y, right, y), fill='#e4e8eb', width=1)
            draw.text((15, y - 7), str(tick), fill='#34404a')
        draw.rectangle((left, top, right, bottom), outline='#34404a', width=1)
        for second in range(0, 9, 2):
            x = left + (right - left) * second / 7.99
            draw.text((x - 8, bottom + 8), str(second), fill='#34404a')
    draw.text((left, 20), 'Virtual knee closed loop: synthetic model only',
              fill='#15242d')
    draw.text((left, 54), 'Angle (deg): target / baseline / assisted', fill='#34404a')
    draw.text((left, 364), 'Virtual effort (Nm): normal / IMU dropout', fill='#34404a')
    draw.text((right - 55, 640), 'Time (s)', fill='#34404a')

    lines = (
        (trials['baseline'], 'target_deg', panels[0], '#303f4b'),
        (trials['baseline'], 'true_angle_deg', panels[0], '#c45635'),
        (trials['assisted'], 'true_angle_deg', panels[0], '#078a83'),
        (trials['assisted'], 'virtual_effort_nm', panels[1], '#078a83'),
        (trials['imu_dropout'], 'virtual_effort_nm', panels[1], '#c45635'),
    )
    for rows, key, panel, color in lines:
        draw.line([point(index, row[key], panel) for index, row in enumerate(rows)],
                  fill=color, width=3)
    for x, label, color in ((left, 'Target', '#303f4b'),
                            (left + 110, 'Baseline', '#c45635'),
                            (left + 240, 'Assisted', '#078a83')):
        draw.line((x, 335, x + 24, 335), fill=color, width=4)
        draw.text((x + 30, 327), label, fill='#34404a')
    for x, label, color in ((left + 420, 'Normal', '#078a83'),
                            (left + 540, 'IMU dropout', '#c45635')):
        draw.line((x, 335, x + 24, 335), fill=color, width=4)
        draw.text((x + 30, 327), label, fill='#34404a')
    dropout_x = point(400, 0.0, panels[1])[0]
    draw.line((dropout_x, panels[1][0], dropout_x, panels[1][1]),
              fill='#a83a36', width=2)
    draw.text((dropout_x + 6, 396), 'IMU loss at 4 s', fill='#a83a36')
    image.save(path)


def main(args=None):
    parser = argparse.ArgumentParser(description='Offline virtual knee control loop')
    parser.add_argument('--csv', help='optional output CSV path')
    parser.add_argument('--png', help='optional output PNG path (requires Pillow)')
    options = parser.parse_args(args)
    trials = {
        'baseline': run_trial(False),
        'assisted': run_trial(True),
        'imu_dropout': run_trial(True, dropout=(4.0, 4.25)),
    }
    if options.csv:
        with open(options.csv, 'w', newline='', encoding='utf-8') as output:
            writer = csv.DictWriter(output, fieldnames=['trial', *trials['baseline'][0]])
            writer.writeheader()
            for name, rows in trials.items():
                writer.writerows({'trial': name, **row} for row in rows)
    if options.png:
        save_plot(trials, options.png)
    print(json.dumps({
        'baseline_rmse_deg': tracking_rmse(trials['baseline']),
        'assisted_rmse_deg': tracking_rmse(trials['assisted']),
        'dropout_fault_latched': trials['imu_dropout'][-1]['supervisor_state'] == 'fault',
        'dropout_post_fault_max_effort_nm': max(
            abs(row['virtual_effort_nm']) for row in trials['imu_dropout']
            if row['time_s'] >= 4.06),
    }, indent=2))


if __name__ == '__main__':
    main()
