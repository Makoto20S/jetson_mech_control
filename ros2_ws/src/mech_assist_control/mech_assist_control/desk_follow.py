"""Bounded desktop IMU velocity mapping, never an anatomical knee controller."""
import math
from mech_knee_kinematics.dual_imu_knee import (
    quaternion_normalize, quaternion_conjugate, quaternion_multiply, signed_twist_degrees,
)


class DeskFollow:
    max_speed = 0.3
    max_acceleration = 0.3
    run_seconds = 5.0
    max_displacement = 0.6

    def __init__(self, motor=False):
        self.motor_mode = motor
        self.pose = self.feedback = self.health = None
        self.state, self.reason, self.target = 'idle', 'not armed', 0.0
        self.zero = None
        self.last_tick = None

    def fault(self, reason):
        self.state, self.reason, self.target = 'fault', reason, 0.0

    def orientation(self, quaternion, now):
        try:
            norm = math.sqrt(sum(v*v for v in quaternion))
            if not 0.5 <= norm <= 1.5:
                raise ValueError('quaternion norm')
            q = quaternion_normalize(quaternion)
        except (ValueError, TypeError):
            self.pose = None
            if self.state == 'armed':
                self.fault('invalid IMU orientation')
            return
        self.pose = (q, now)

    def motor_feedback(self, position, velocity, now):
        if not all(math.isfinite(v) for v in (position, velocity)):
            self.feedback = None
            if self.state == 'armed':
                self.fault('invalid motor feedback')
            return
        self.feedback = (position, velocity, now)

    def healthy(self, ok, now):
        self.health = (ok, now)
        if not ok and self.state == 'armed':
            self.fault('sensor diagnostic unhealthy')

    def available(self, now):
        if (self.pose is None or not 0 <= now-self.pose[1] <= 0.3 or
                self.health is None or not self.health[0] or
                not 0 <= now-self.health[1] <= 0.3):
            return False
        return not self.motor_mode or (self.feedback is not None and
                                       0 <= now-self.feedback[2] <= 0.15)

    def arm(self, now):
        if self.state != 'idle' or not self.available(now):
            return False
        if self.motor_mode and abs(self.feedback[1]) > 0.1:
            return False
        self.zero = self.pose[0]
        self.start_position = self.feedback[0] if self.motor_mode else 0.0
        self.started = self.last_tick = now
        self.state, self.reason, self.target = 'armed', '', 0.0
        return True

    def tick(self, now):
        if self.state != 'armed':
            return 0.0
        dt = now-self.last_tick
        self.last_tick = now
        if not self.available(now):
            self.fault('stale sensor or motor feedback')
        elif not 0 <= dt <= 0.1:
            self.fault('control timer gap')
        elif now-self.started >= self.run_seconds:
            self.state, self.reason, self.target = 'complete', 'trial expired', 0.0
        elif self.motor_mode and (abs(self.feedback[1]) > 1.0 or
                                 abs(self.feedback[0]-self.start_position) > self.max_displacement):
            self.fault('motor speed or displacement envelope')
        if self.state != 'armed':
            return 0.0
        angle = math.radians(signed_twist_degrees(
            quaternion_multiply(quaternion_conjugate(self.zero), self.pose[0]), (0, 1, 0)))
        if abs(angle) > math.radians(30):
            self.fault('desktop IMU rotation exceeds 30 degrees')
            return 0.0
        error = math.copysign(max(0, abs(angle)-math.radians(5)), angle)
        desired = max(-self.max_speed, min(self.max_speed, 0.8*error))
        step = self.max_acceleration*dt
        self.target += max(-step, min(step, desired-self.target))
        return self.target
