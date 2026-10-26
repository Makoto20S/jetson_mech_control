"""Bounded, ROS-independent state for the read-only attitude viewer."""
from collections import deque
import math
from .dual_imu_knee import quaternion_normalize, quaternion_rotate_vector


class AttitudeViewState:
    def __init__(self, raw=False):
        self.raw = raw
        self.segments = {}
        self.pending = {}
        self.rpy = {}
        self.joint = None
        self.diagnostic = None
        self.history = deque(maxlen=1200)
        self.last_history_stamp = None

    def segment(self, name, stamp, quaternion, now):
        old = self.segments.get(name)
        if old is not None and stamp <= old[0]:
            return
        try:
            if stamp <= 0:
                raise ValueError('missing stamp')
            q = quaternion_normalize(quaternion)
        except ValueError:
            self.segments.pop(name, None)
            self.pending.clear()
            return
        pair = self.pending.setdefault(stamp, {})
        pair[name] = (stamp, q, now)
        if 'thigh' in pair and 'shank' in pair:
            self.segments = {name: pair[name] for name in ('thigh', 'shank')}
            self.pending = {key: value for key, value in self.pending.items() if key > stamp}
        while len(self.pending) > 8:
            self.pending.pop(min(self.pending))

    def update_joint(self, stamp, angle, rate, now):
        if not all(math.isfinite(v) for v in (angle, rate)) or stamp <= 0:
            self.joint = None
            return
        if self.joint and stamp <= self.joint[0]:
            return
        self.joint = (stamp, angle, rate, now)

    def record_history(self, now):
        if not self.ready(now)[0]:
            return
        stamp = self.segments['thigh'][0] if self.raw else self.joint[0]
        if stamp == self.last_history_stamp:
            return
        if self.raw:
            if any(name not in self.rpy or now-self.rpy[name][1] > 0.3
                   for name in ('thigh', 'shank')):
                return
            values = (self.rpy['thigh'][0][1], self.rpy['shank'][0][1])
        else:
            values = self.joint[1:3]
        self.history.append((now, *values))
        self.last_history_stamp = stamp

    def ready(self, now):
        if self.diagnostic is None or now - self.diagnostic[2] > 0.5:
            return False, '诊断未更新'
        level, message, _ = self.diagnostic
        if level != 0:
            return False, message
        if any(name not in self.segments for name in ('thigh', 'shank')):
            return False, '等待双 IMU 姿态' if self.raw else '等待站立零位姿态'
        a, b = self.segments['thigh'], self.segments['shank']
        if max(now-a[2], now-b[2]) > 0.3:
            return False, '姿态数据失效'
        if a[0] != b[0]:
            return False, '等待同包双 IMU 姿态'
        if not self.raw and (self.joint is None or now-self.joint[3] > 0.3):
            return False, '膝角数据失效'
        return True, '双 IMU 姿态正常 · 膝角未标定' if self.raw else '姿态与膝角正常'

    def directions(self):
        return tuple(quaternion_rotate_vector(self.segments[name][1], (0, 0, -1))
                     for name in ('thigh', 'shank'))
