"""Receive-only ROS 2 wrapper for the PC viewer's knee estimator."""

from dataclasses import dataclass
import math
import time

import rclpy
from geometry_msgs.msg import QuaternionStamped, Vector3Stamped
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Float64
from std_srvs.srv import Trigger

from .attitude import SegmentAttitudes, euler_zyx
from .dual_imu_knee import (
    KneeEstimator, calibrate_hinge_axis_from_gyroscopes,
    hipnuc_quaternion_to_standard, hipnuc_vector_to_standard,
    relative_quaternion,
)


@dataclass(frozen=True)
class Reading:
    orientation: tuple[float, float, float, float]
    gyro_dps: tuple[float, float, float]
    acceleration: tuple[float, float, float]


class KneeKinematicsNode(Node):
    def __init__(self):
        super().__init__('knee_kinematics')
        self.declare_parameter('imu_1_topic', 'imu/imu1/data')
        self.declare_parameter('imu_2_topic', 'imu/imu2/data')
        self.declare_parameter('basis_mapping', 'hipnuc')
        self.declare_parameter('flexion_sign', -1)
        self.declare_parameter('static_seconds', 1.5)
        self.declare_parameter('motion_seconds', 6.0)
        self.declare_parameter('pair_timeout_ms', 500.0)
        self.declare_parameter('max_pair_delay_ms', 100.0)
        self.declare_parameter('source_max_age_ms', 300.0)
        mapping = self.get_parameter('basis_mapping').value
        if mapping not in ('identity', 'hipnuc'):
            raise ValueError('basis_mapping must be identity or hipnuc')
        self.mapping = mapping
        self.attitudes = SegmentAttitudes()
        self.attitude_pairs = 0
        self.estimator = KneeEstimator()
        self.estimator.set_direction(self.get_parameter('flexion_sign').value)
        self.axis_ready = False
        self.static_pairs = []
        self.motion_pairs = []
        self.pending = {}
        self.last_closed_stamp = 0
        self.dropped_pairs = 0
        self.out_of_order_messages = 0
        self.capture_mode = None
        self.capture_end = 0.0
        self.last_pair_time = None
        self.pair_timed_out = False
        self.last_error = 'not calibrated'
        self.pair_timeout = self.get_parameter('pair_timeout_ms').value / 1000.0
        self.max_pair_delay = self.get_parameter('max_pair_delay_ms').value / 1000.0
        self.source_max_age = self.get_parameter('source_max_age_ms').value / 1000.0
        if not math.isfinite(self.source_max_age) or not 0 < self.source_max_age <= self.pair_timeout:
            raise ValueError('source_max_age_ms must be finite, positive and <= pair_timeout_ms')
        if not 0 < self.max_pair_delay < self.pair_timeout:
            raise ValueError('max_pair_delay_ms must be positive and below pair_timeout_ms')
        self.orientation_pubs = {name: self.create_publisher(
            QuaternionStamped, f'attitude/{name}/orientation', 10)
            for name in ('thigh', 'shank', 'relative', 'thigh_from_zero', 'shank_from_zero')}
        self.euler_pubs = {name: self.create_publisher(
            Vector3Stamped, f'attitude/{name}/rpy_rad', 10)
            for name in ('thigh', 'shank')}
        self.joint_pub = self.create_publisher(JointState, 'knee/joint_state', 10)
        self.angle_pub = self.create_publisher(Float64, 'knee/flexion_deg', 10)
        self.diagnostic_pub = self.create_publisher(DiagnosticArray, 'knee/diagnostics', 10)
        self.create_subscription(Imu, self.get_parameter('imu_1_topic').value,
                                 lambda msg: self._receive(0, msg), qos_profile_sensor_data)
        self.create_subscription(Imu, self.get_parameter('imu_2_topic').value,
                                 lambda msg: self._receive(1, msg), qos_profile_sensor_data)
        self.create_service(Trigger, 'knee/start_static_calibration', self._start_static)
        self.create_service(Trigger, 'knee/start_axis_calibration', self._start_axis)
        self.create_service(Trigger, 'knee/reset_calibration', self._reset)
        self.create_timer(0.1, self._tick)

    def _reading(self, msg):
        if (msg.orientation_covariance[0] < 0 or
                msg.angular_velocity_covariance[0] < 0 or
                msg.linear_acceleration_covariance[0] < 0):
            return None
        q = (msg.orientation.w, msg.orientation.x,
             msg.orientation.y, msg.orientation.z)
        gyro = (msg.angular_velocity.x, msg.angular_velocity.y,
                msg.angular_velocity.z)
        accel = (msg.linear_acceleration.x, msg.linear_acceleration.y,
                 msg.linear_acceleration.z)
        if (not all(math.isfinite(value) for value in (*q, *gyro, *accel)) or
                not 0.5 <= math.sqrt(sum(value * value for value in q)) <= 1.5 or
                not 0.2 * 9.80665 <= math.sqrt(sum(value * value for value in accel)) <=
                4.0 * 9.80665):
            return None
        gyro_dps = tuple(math.degrees(value) for value in gyro)
        if self.mapping == 'hipnuc':
            q = hipnuc_quaternion_to_standard(q)
            gyro_dps = hipnuc_vector_to_standard(gyro_dps)
            accel = hipnuc_vector_to_standard(accel)
        return Reading(tuple(value / math.sqrt(sum(v*v for v in q)) for value in q),
                       gyro_dps, accel)

    def _receive(self, index, msg):
        stamp = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        if stamp <= 0:
            self._interrupt_pair('missing ROS timestamp')
            return
        # Source headers and node clock must share the ROS time domain. Queue
        # arrival cannot renew an old measurement's lifetime.
        age = (self.get_clock().now().nanoseconds - stamp) / 1e9
        if not 0 <= age <= self.source_max_age:
            self._interrupt_pair('IMU source timestamp stale or future')
            return
        now = time.monotonic()
        self._expire_pending(now)
        if stamp <= self.last_closed_stamp:
            self.out_of_order_messages += 1
            return
        if stamp not in self.pending:
            self.pending[stamp] = (now, {})
        first_arrival, pair = self.pending[stamp]
        pair[index] = self._reading(msg)
        while len(self.pending) > 32:
            oldest = min(self.pending)
            self.pending.pop(oldest)
            self.last_closed_stamp = max(self.last_closed_stamp, oldest)
            self._interrupt_pair('IMU pair buffer overflow')
        if len(pair) != 2:
            return
        self.pending.pop(stamp, None)
        self.last_closed_stamp = stamp
        if now - first_arrival > self.max_pair_delay:
            self._interrupt_pair('IMU pair arrived too late')
            return
        for old_stamp in tuple(self.pending):
            if old_stamp < stamp:
                self.pending.pop(old_stamp)
                self._interrupt_pair('IMU pair incomplete')
        if pair[0] is None or pair[1] is None:
            self._interrupt_pair('IMU orientation, gyro or acceleration unavailable')
            return
        readings = (pair[0], pair[1])
        self.last_pair_time = now - age
        self.pair_timed_out = False
        if self.capture_mode is None:
            self._publish_attitudes(readings, msg.header.stamp)
        if self.capture_mode == 'static':
            self.static_pairs.append(readings)
        elif self.capture_mode == 'axis':
            self.motion_pairs.append(readings)
        elif self.axis_ready and self.estimator.neutral_relative is not None:
            try:
                estimate = self.estimator.update(
                    readings[0].orientation, readings[1].orientation,
                    stamp / 1_000_000_000.0,
                    readings[0].gyro_dps, readings[1].gyro_dps,
                )
            except (RuntimeError, ValueError) as exc:
                self.last_error = str(exc)
                self._publish_diagnostic(now)
                return
            if not estimate.within_limits:
                self.last_error = estimate.reason
                # Event publication cannot be deferred to the 100 ms timer:
                # a following healthy sample must not hide this invalid one.
                self._publish_diagnostic(now)
                return
            joint = JointState()
            joint.header.stamp = msg.header.stamp
            joint.name = ['knee_joint']
            joint.position = [math.radians(estimate.angle_deg)]
            joint.velocity = [math.radians(estimate.angular_velocity_dps)]
            self.joint_pub.publish(joint)
            value = Float64()
            value.data = estimate.angle_deg
            self.angle_pub.publish(value)
            self.last_error = '' if estimate.within_limits else estimate.reason

    def _publish_attitudes(self, readings, stamp):
        snapshot = self.attitudes.update(readings[0].orientation, readings[1].orientation)
        for name, publisher in self.orientation_pubs.items():
            q = getattr(snapshot, name)
            if q is None:
                continue
            message = QuaternionStamped()
            message.header.stamp = stamp
            # Reference frames are sensor-derived, not geographic ENU or anatomy.
            message.header.frame_id = ('thigh_imu' if name == 'relative' else
                                       name.removesuffix('_from_zero') + '_imu_zero'
                                       if name.endswith('_from_zero') else
                                       'imu_reference')
            message.quaternion.w, message.quaternion.x = q[0], q[1]
            message.quaternion.y, message.quaternion.z = q[2], q[3]
            publisher.publish(message)
        for name, publisher in self.euler_pubs.items():
            message = Vector3Stamped()
            message.header.stamp = stamp
            message.header.frame_id = 'imu_reference'
            message.vector.x, message.vector.y, message.vector.z = euler_zyx(
                getattr(snapshot, name))
            publisher.publish(message)
        self.attitude_pairs += 1

    def _interrupt_pair(self, reason):
        self.dropped_pairs += 1
        self.estimator.reset_filter()
        if self.capture_mode is not None:
            self.capture_mode = None
            self.static_pairs.clear()
            self.motion_pairs.clear()
        self.last_error = reason
        self._publish_diagnostic(time.monotonic())

    def _expire_pending(self, now):
        for stamp, (first_arrival, _) in tuple(self.pending.items()):
            if now - first_arrival > self.max_pair_delay:
                self.pending.pop(stamp)
                self.last_closed_stamp = max(self.last_closed_stamp, stamp)
                self._interrupt_pair('IMU pair incomplete or delayed')

    def _start_static(self, request, response):
        del request
        if self.capture_mode is not None:
            response.success, response.message = False, 'calibration already running'
            return response
        self.attitudes.reset()
        self.estimator.neutral_relative = None
        self.estimator.reset_filter()
        self.static_pairs.clear()
        self.motion_pairs.clear()
        self.capture_mode = 'static'
        self.capture_end = time.monotonic() + self.get_parameter('static_seconds').value
        response.success, response.message = True, 'hold the leg straight and still'
        return response

    def _start_axis(self, request, response):
        del request
        if self.capture_mode is not None or self.estimator.neutral_relative is None:
            response.success, response.message = False, 'complete static calibration first'
            return response
        self.motion_pairs.clear()
        self.axis_ready = False
        self.capture_mode = 'axis'
        self.capture_end = time.monotonic() + self.get_parameter('motion_seconds').value
        response.success, response.message = True, 'flex and extend the knee 3-5 times'
        return response

    def _reset(self, request, response):
        del request
        self.capture_mode = None
        self.pending.clear()
        self.last_closed_stamp = 0
        self.last_pair_time = None
        self.pair_timed_out = False
        self.attitudes.reset()
        self.estimator.neutral_relative = None
        self.estimator.set_axis((0.0, 1.0, 0.0))
        self.axis_ready = False
        self.static_pairs.clear()
        self.motion_pairs.clear()
        self.last_error = 'not calibrated'
        response.success, response.message = True, 'calibration cleared'
        return response

    def _tick(self):
        now = time.monotonic()
        self._expire_pending(now)
        if self.last_pair_time is not None and now - self.last_pair_time >= self.pair_timeout:
            if not self.pair_timed_out:
                self.pair_timed_out = True
                self.estimator.reset_filter()
                if self.capture_mode is not None:
                    self.capture_mode = None
                    self.static_pairs.clear()
                    self.motion_pairs.clear()
                self.last_error = 'IMU pair timed out'
        if self.capture_mode is not None and now >= self.capture_end:
            stage = self.capture_mode
            self.capture_mode = None
            if stage == 'static':
                try:
                    if len(self.static_pairs) < 10:
                        raise ValueError('insufficient standing pairs')
                    for thigh, shank in self.static_pairs:
                        if (math.hypot(*thigh.gyro_dps) > 12.0 or
                                math.hypot(*shank.gyro_dps) > 12.0):
                            raise ValueError('standing calibration includes motion')
                    self.estimator.calibrate_zero([
                        relative_quaternion(thigh.orientation, shank.orientation)
                        for thigh, shank in self.static_pairs
                    ])
                    self.attitudes.calibrate_zero([
                        (thigh.orientation, shank.orientation)
                        for thigh, shank in self.static_pairs
                    ])
                    self.last_error = ('axis calibration required' if not self.axis_ready else '')
                except ValueError as exc:
                    self.static_pairs.clear()
                    self.last_error = str(exc)
            else:
                try:
                    result = calibrate_hinge_axis_from_gyroscopes(
                        [(thigh.orientation, shank.orientation,
                          thigh.gyro_dps, shank.gyro_dps)
                         for thigh, shank in self.motion_pairs],
                        reference_axis=self.estimator.hinge_axis,
                        minimum_rate_dps=5.0,
                        minimum_explained_ratio=0.70,
                    )
                    self.estimator.set_hinge_axes(result.proximal_axis, result.distal_axis)
                    self.axis_ready = True
                    self.estimator.neutral_relative = None
                    self.attitudes.reset()
                    self.last_error = 'repeat standing zero required'
                    self.get_logger().info(
                        f'axis calibration ready: quality={result.explained_ratio:.3f}, '
                        f'samples={result.sample_count}; repeat standing zero')
                except ValueError as exc:
                    self.last_error = str(exc)
        self._publish_diagnostic(now)

    def _publish_diagnostic(self, now):
        age = None if self.last_pair_time is None else now - self.last_pair_time
        stale = age is None or age >= self.pair_timeout
        status = DiagnosticStatus()
        status.name = 'knee_kinematics'
        status.hardware_id = 'ctrboard_dual_imu'
        status.level = DiagnosticStatus.OK if (self.axis_ready and
                                               self.estimator.neutral_relative is not None and
                                               not stale and not self.last_error and
                                               self.capture_mode is None) else DiagnosticStatus.WARN
        status.message = ('calibrating ' + self.capture_mode if self.capture_mode else
                          'IMU pair timed out' if stale else
                          self.last_error or 'calibrated')
        status.values = [KeyValue(key='attitude_pairs', value=str(self.attitude_pairs)),
                         KeyValue(key='basis_mapping', value=self.mapping),
                         KeyValue(key='dropped_pairs', value=str(self.dropped_pairs)),
                         KeyValue(key='out_of_order_messages',
                                  value=str(self.out_of_order_messages)),
                         KeyValue(key='static_pairs', value=str(len(self.static_pairs))),
                         KeyValue(key='motion_pairs', value=str(len(self.motion_pairs)))]
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        array.status = [status]
        self.diagnostic_pub.publish(array)


def main(args=None):
    from rclpy.executors import ExternalShutdownException

    rclpy.init(args=args)
    node = None
    try:
        node = KneeKinematicsNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:
        if rclpy.ok():
            raise
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
