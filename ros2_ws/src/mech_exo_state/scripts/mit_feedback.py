"""Per-drive 0x29 samples. Producer and consumer share the Orin monotonic clock."""
import copy
import math
from mech_exo_state.msg import MitMotorFeedback

IDS = (104, 105)
FIELDS = ('mit_drive_id', 'mit_rx_sequence', 'mit_rx_us', 'mit_valid', 'mit_position_deg',
          'mit_erpm', 'mit_iq_a', 'mit_temperature_c', 'mit_fault_code')


def empty(mid):
    msg = MitMotorFeedback()
    msg.drive_id = mid
    msg.age_ms = msg.position_deg = msg.erpm = msg.iq_a = msg.temperature_c = math.nan
    msg.output_velocity_rad_s = math.nan
    msg.reason = 'waiting_for_feedback'
    return msg


class MitFeedback:
    def __init__(self, names, ttl_ms, pole_pairs=(14, 14), gear_ratios=(8., 8.),
                 directions=(1, 1), speed_confirmed=(False, False)):
        if len(names) != 2 or len(set(names)) != 2 or not all(names):
            raise ValueError('Two distinct MIT joint names in 104,105 order required')
        if not math.isfinite(ttl_ms) or ttl_ms <= 0:
            raise ValueError('Invalid MIT feedback TTL')
        if (any(len(pair) != 2 for pair in (pole_pairs, gear_ratios, directions, speed_confirmed))
                or not all(type(n) is int and n > 0 for n in pole_pairs)
                or not all(math.isfinite(r) and r > 0 for r in gear_ratios)
                or not all(type(d) is int and d in (-1, 1) for d in directions)
                or not all(type(v) is bool for v in speed_confirmed)):
            raise ValueError('Invalid per-drive speed mapping in 104,105 order')
        self.speed_scales = [d * math.tau / 60 / n / r
                             for d, n, r in zip(directions, pole_pairs, gear_ratios)]
        self.speed_confirmed = tuple(speed_confirmed)
        self.names = tuple(names)
        self.ttl_ms = ttl_ms
        self.reset()

    def reset(self):
        self.axes = [empty(mid) for mid in IDS]
        self.count = 0
        self.arrivals = [None, None]  # (ROS callback time, monotonic callback time)

    def update(self, msg, now_ns, mono_ns):
        for i, name in enumerate(self.names):
            if name not in msg.joint_names:
                continue  # separate/partial publications age independently
            old = self.axes[i]
            try:
                if msg.joint_names.count(name) != 1 or len(msg.joint_names) != len(msg.interface_values):
                    raise ValueError
                row = msg.interface_values[msg.joint_names.index(name)]
                if len(set(row.interface_names)) != len(row.interface_names) or len(row.values) != len(row.interface_names):
                    raise ValueError
                values = dict(zip(row.interface_names, row.values))
                mid, seq, rx, ok, pos, erpm, iq, temp, code = (values[k] for k in FIELDS)
                if (not all(math.isfinite(x) for x in (mid, seq, rx, ok, pos, erpm, iq, temp, code))
                        or mid != IDS[i] or not 0 < seq < 2**53 or int(seq) != seq
                        or not 0 < rx <= mono_ns // 1000 or int(rx) != rx
                        or ok not in (0., 1.) or int(code) != code or not 0 <= code <= 255):
                    raise ValueError
            except (KeyError, ValueError, OverflowError):
                old.valid, old.reason = False, 'invalid_interfaces'
                continue
            # Sequence may restart on reactivation. Only a later transport time
            # can start a new sequence; a repeat never refreshes its receive time.
            if rx < old.host_rx_us or (rx == old.host_rx_us and seq < old.rx_sequence):
                old.valid, old.reason = False, 'out_of_order'
                continue
            if rx == old.host_rx_us and seq == old.rx_sequence:
                if not ok:
                    old.valid = False
                    if old.fault_code == 0:
                        old.reason = 'driver_invalid'
                continue
            sample = MitMotorFeedback()
            sample.drive_id, sample.rx_sequence, sample.host_rx_us = int(mid), int(seq), int(rx)
            sample.position_deg, sample.erpm, sample.iq_a, sample.temperature_c = pos, erpm, iq, temp
            sample.fault_code, sample.has_sample = int(code), True
            sample.valid = bool(ok and code == 0)
            sample.reason = ('feedback_ok' if sample.valid else
                             'device_status_nonzero' if code else 'driver_invalid')
            self.axes[i] = sample
            self.arrivals[i] = (now_ns, mono_ns)
            self.count += 1

    def snapshot(self, mono_ns):
        axes = copy.deepcopy(self.axes)
        for i, sample in enumerate(axes):
            if sample.has_sample:
                sample.age_ms = (mono_ns - sample.host_rx_us * 1000) / 1e6
                if not 0 <= sample.age_ms <= self.ttl_ms:
                    sample.valid, sample.reason = False, 'feedback_timeout'
                sample.output_velocity_rad_s = sample.erpm * self.speed_scales[i]
                sample.output_velocity_valid = sample.valid and self.speed_confirmed[i]
        return axes
