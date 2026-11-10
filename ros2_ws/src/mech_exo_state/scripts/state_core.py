"""Receive-only state assembly. All elapsed-time TTLs use a monotonic clock."""
import copy
import math
from sensor_msgs.msg import JointState, Imu
from mech_exo_state.msg import RobotState, SourceStatus
from mit_feedback import MitFeedback
from ref01_mujoco_kinematics import MuJoCoForwardKinematics, KinematicsError


def ns(stamp):
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def set_stamp(stamp, value):
    stamp.sec, stamp.nanosec = divmod(value, 1_000_000_000)


def finite(values):
    return all(math.isfinite(x) for x in values)


def covariance_available(values):
    return len(values) == 9 and finite(values) and all(values[i] >= 0 for i in (0, 4, 8))


class StateCore:
    def __init__(self, names=('motor104_joint', 'motor105_joint'),
                 scales=(1., 1.), offsets=(-math.pi / 4, -math.pi / 4),
                 mapping_confirmed=False, timeout_ms=100., skew_ms=30., future_ms=5., model_path=None,
                 motor_source='joint_states', mit_names=('mit_comm_motor104','mit_comm_motor105'),
                 mit_ttl_ms=60., mit_pole_pairs=(14, 14), mit_gear_ratios=(8., 8.),
                 mit_speed_directions=(1, 1), mit_speed_confirmed=(False, False)):
        if len(names) != 2 or not all(isinstance(n, str) and n for n in names) or len(set(names)) != 2:
            raise ValueError('Exactly two distinct joint names required')
        if not isinstance(mapping_confirmed, bool):
            raise ValueError('mapping_confirmed must be boolean')
        if len(scales) != 2 or len(offsets) != 2 or not finite([*scales, *offsets]):
            raise ValueError('Two finite mapping scales/offsets required')
        if any(s == 0 for s in scales) or not finite([timeout_ms, skew_ms, future_ms]):
            raise ValueError('Invalid mapping or timing limits')
        if timeout_ms <= 0 or skew_ms < 0 or future_ms < 0:
            raise ValueError('Invalid timing limits')
        self.names, self.scales, self.offsets = tuple(names), tuple(scales), tuple(offsets)
        self.confirmed = mapping_confirmed
        self.ttl, self.skew, self.future = timeout_ms * 1e6, skew_ms, future_ms * 1e6
        self.fk = MuJoCoForwardKinematics(model_path)
        if motor_source not in ('joint_states', 'mit'):
            raise ValueError('Unknown motor source')
        self.motor_source = motor_source
        self.mit = MitFeedback(mit_names, mit_ttl_ms, mit_pole_pairs, mit_gear_ratios,
                               mit_speed_directions, mit_speed_confirmed)
        self.last_ros = None
        self.reset()

    def reset(self):
        self.fk.reset()
        self.mit.reset()
        self.samples = [None, None, None]
        self.status = [SourceStatus(reason='waiting_for_data') for _ in range(3)]
        self.mono = [None] * 3
        self.highwater = [0] * 3

    def clock(self, now):
        if self.last_ros is not None and now < self.last_ros:
            self.reset()
            for status in self.status:
                status.reason = 'ros_clock_reset'
        self.last_ros = now

    def accept(self, index, message, now, mono):
        self.clock(now)
        status = self.status[index]
        source = ns(message.header.stamp)
        # An invalid or duplicate message invalidates the source immediately.
        status.valid = False
        if source <= 0 or message.header.stamp.nanosec >= 1_000_000_000:
            status.reason = 'invalid_stamp'
        elif source > now + self.future:
            status.reason = 'future_stamp'
        elif now - source > self.ttl:
            status.reason = 'old_source_stamp'
        elif source <= self.highwater[index]:
            status.reason = 'non_increasing_stamp'
        else:
            self.highwater[index] = source
            self.mono[index] = mono
            status.source_stamp = copy.deepcopy(message.header.stamp)
            set_stamp(status.received_stamp, now)
            status.accepted_samples += 1
            self.samples[index] = copy.deepcopy(message)
            status.valid = True
            status.reason = 'message_valid_sample_time_unverified'
            return True
        return False

    def joints(self, message, now, mono):
        if self.motor_source == 'mit' or not self.accept(0, message, now, mono):
            return
        size = len(message.name)
        if (len(set(message.name)) != size or not all(n in message.name for n in self.names)
                or len(message.position) != size
                or len(message.velocity) not in (0, size)
                or len(message.effort) not in (0, size)
                or not finite([*message.position, *message.velocity, *message.effort])):
            self.status[0].valid = False
            self.status[0].reason = 'invalid_joint_fields'
            return
        indices = [message.name.index(n) for n in self.names]
        out = JointState(header=copy.deepcopy(message.header), name=list(self.names))
        for field in ('position', 'velocity', 'effort'):
            values = getattr(message, field)
            setattr(out, field, [values[i] for i in indices] if len(values) else [])
        self.samples[0] = out

    @staticmethod
    def imu_fields(message):
        q = message.orientation
        qv = [q.x, q.y, q.z, q.w]
        norm2 = sum(x*x for x in qv)
        orientation = (covariance_available(message.orientation_covariance)
                       and finite(qv) and abs(norm2 - 1.) <= .05)
        w, a = message.angular_velocity, message.linear_acceleration
        velocity = covariance_available(message.angular_velocity_covariance) and finite([w.x, w.y, w.z])
        acceleration = covariance_available(message.linear_acceleration_covariance) and finite([a.x, a.y, a.z])
        return bool(orientation), bool(velocity), bool(acceleration)

    def imu(self, index, message, now, mono):
        if index not in (1, 2):
            raise ValueError('IMU index must be 1 or 2')
        if not self.accept(index, message, now, mono):
            return
        fields = self.imu_fields(message)
        if not message.header.frame_id or not any(fields):
            self.status[index].valid = False
            self.status[index].reason = 'invalid_imu_fields_or_frame'

    def mit_feedback(self, message, now, mono):
        self.clock(now)
        self.mit.update(message, now, mono)

    def snapshot(self, now, mono):
        self.clock(now)
        out = RobotState()
        set_stamp(out.header.stamp, now)
        out.mapping_confirmed = self.confirmed
        out.alpha_position = [math.nan, math.nan]
        out.alpha_velocity = [math.nan, math.nan]
        out.q_position = [math.nan, math.nan]
        out.q_reason = 'joint_source_invalid'
        out.q_closure_residual_m = math.nan
        out.source_skew_ms = math.nan
        for i, source_status in enumerate(self.status):
            status = copy.deepcopy(source_status)
            status.source_age_ms = (now - ns(status.source_stamp)) / 1e6 if self.mono[i] is not None else math.nan
            status.receive_age_ms = (mono - self.mono[i]) / 1e6 if self.mono[i] is not None else math.nan
            if self.mono[i] is not None and (mono < self.mono[i] or mono - self.mono[i] > self.ttl):
                status.valid = False
                status.reason = 'receive_timeout'
            # Additional source-time sanity gate; cannot extend monotonic TTL.
            if status.valid and (status.source_age_ms > self.ttl / 1e6 or status.source_age_ms < -self.future / 1e6):
                status.valid = False
                status.reason = 'source_time_outside_window'
            out.sources[i] = status
        out.mit_motors = self.mit.snapshot(mono)
        if self.motor_source == 'mit':
            axes = out.mit_motors
            status = SourceStatus(reason='mit_axis_invalid', accepted_samples=self.mit.count)
            if all(a.has_sample for a in axes):
                oldest = min(range(2), key=lambda i: axes[i].host_rx_us)
                rx_ns = axes[oldest].host_rx_us * 1000
                arrival_ros, arrival_mono = self.mit.arrivals[oldest]
                stamp = max(0, arrival_ros - (arrival_mono - rx_ns))
                status.source_age_ms = (mono - rx_ns) / 1e6
                status.receive_age_ms = (mono - arrival_mono) / 1e6
                set_stamp(status.source_stamp, stamp)
                set_stamp(status.received_stamp, arrival_ros)
                status.valid = all(a.valid for a in axes)
                if status.valid and abs(axes[0].host_rx_us - axes[1].host_rx_us) > self.skew * 1000:
                    status.valid, status.reason = False, 'mit_pair_skew'
                if status.valid:
                    status.reason = 'mit_host_rx_valid'
                    out.joints = JointState(name=list(self.names),
                                            position=[math.radians(a.position_deg) for a in axes])
                    if all(a.output_velocity_valid for a in axes):
                        out.joints.velocity = [a.output_velocity_rad_s for a in axes]
                    set_stamp(out.joints.header.stamp, stamp)
            else:
                status.source_age_ms = status.receive_age_ms = math.nan
            out.sources[0] = status
        elif out.sources[0].valid:
            out.joints = copy.deepcopy(self.samples[0])
        if out.sources[0].valid:
            # Input JointState is already radians; no motor gear-ratio conversion.
            out.alpha_position = [self.scales[i] * out.joints.position[1-i] + self.offsets[i] for i in range(2)]
            out.alpha_valid = self.confirmed and finite(out.alpha_position)
            if len(out.joints.velocity) == 2:
                out.alpha_velocity = [self.scales[i] * out.joints.velocity[1-i] for i in range(2)]
                out.alpha_velocity_valid = out.alpha_valid and finite(out.alpha_velocity)
        if out.sources[0].valid:
            out.q_reason = 'mapping_unconfirmed' if not self.confirmed else 'invalid_alpha'
            if out.alpha_valid:
                try:
                    result = self.fk.solve(out.alpha_position)
                    out.q_position = list(result.q)
                    out.q_closure_residual_m = result.residual_m
                    out.q_valid = True
                    out.q_reason = 'valid_from_motor_angles'
                except KinematicsError as error:
                    out.q_reason = 'fk_invalid:' + str(error)
        for i in range(2):
            if self.samples[i+1] is not None:
                out.imus[i] = copy.deepcopy(self.samples[i+1])
            if out.sources[i+1].valid:
                flags = self.imu_fields(out.imus[i])
                out.imu_orientation_valid[i], out.imu_angular_velocity_valid[i], out.imu_linear_acceleration_valid[i] = flags
        if all(ns(s.source_stamp) > 0 for s in out.sources):
            stamps = [ns(s.source_stamp) for s in out.sources]
            out.source_skew_ms = (max(stamps) - min(stamps)) / 1e6
            out.messages_aligned = all(s.valid for s in out.sources) and out.source_skew_ms <= self.skew
        # This is message alignment, NOT verified physical sampling synchrony.
        out.hardware_freshness_verified = False
        return out
