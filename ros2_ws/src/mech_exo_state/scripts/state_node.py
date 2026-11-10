#!/usr/bin/env python3
"""ROS receive-only PH-EXO state node; no hardware or command publishers."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
import math
import time
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from rclpy.qos import qos_profile_sensor_data
from rcl_interfaces.msg import ParameterDescriptor
from control_msgs.msg import DynamicJointState
from sensor_msgs.msg import JointState, Imu
from mech_exo_state.msg import RobotState
from state_core import StateCore, ns
from timing_trace import TimingTrace, milliseconds_to_ns


class StateNode(Node):
    def __init__(self):
        super().__init__('ph_exo_state')
        def param(name, default):
            return self.declare_parameter(name, default, ParameterDescriptor(read_only=True)).value
        model_path = str(Path(get_package_share_directory('mech_exo_state')) / 'mujoco/ref01_dynamics_cad_v1.xml')
        self.core = StateCore(model_path=model_path,
            motor_source=param('motor_source', 'joint_states'),
            mit_names=param('mit_joint_names', ['mit_comm_motor104', 'mit_comm_motor105']),
            mit_pole_pairs=param('mit_pole_pairs', [14, 14]),
            mit_gear_ratios=param('mit_gear_ratios', [8., 8.]),
            mit_speed_directions=param('mit_speed_directions', [1, 1]),
            mit_speed_confirmed=param('mit_speed_confirmed', [False, False]),
            mit_ttl_ms=param('mit_feedback_ttl_ms', 60.),
            names=param('joint_names', ['motor104_joint', 'motor105_joint']),
            scales=param('alpha_scales', [1., 1.]),
            offsets=param('alpha_offsets_rad', [-math.pi/4, -math.pi/4]),
            mapping_confirmed=param('mapping_confirmed', False),
            timeout_ms=param('timeout_ms', 100.),
            skew_ms=param('max_skew_ms', 30.), future_ms=param('future_tolerance_ms', 5.))
        rate = param('publish_rate_hz', 100.)
        if not math.isfinite(rate) or not 1 <= rate <= 1000:
            raise ValueError('publish_rate_hz must be in [1, 1000]')
        self.trace = TimingTrace(param('timing_trace_path', ''), round(1e9 / rate),
                                 param('timing_capacity_rows', 2_000_000))
        self.publisher = self.create_publisher(RobotState, 'ph_exo/state', qos_profile_sensor_data)
        self.create_subscription(DynamicJointState, param('mit_topic', '/mit_comm/dynamic_joint_states'),
                                 self.on_mit_feedback, qos_profile_sensor_data)
        self.create_subscription(JointState, param('joint_topic', '/joint_states'), self.on_joints, qos_profile_sensor_data)
        self.create_subscription(Imu, param('imu1_topic', '/imu/imu1/data'), lambda msg: self.on_imu(1, msg), qos_profile_sensor_data)
        self.create_subscription(Imu, param('imu2_topic', '/imu/imu2/data'), lambda msg: self.on_imu(2, msg), qos_profile_sensor_data)
        self.timer = self.create_timer(1./rate, self.publish_state, clock=Clock(clock_type=ClockType.STEADY_TIME))
        if self.trace.enabled:
            # Bracket the timer API in the same CLOCK_MONOTONIC domain as CSV.
            # A missing API result retains the explicitly unverified fallback.
            before = time.monotonic_ns()
            try:
                until = self.timer.time_until_next_call()
                after = time.monotonic_ns()
                if isinstance(until, bool) or not isinstance(until, int):
                    raise ValueError('Timer did not provide an integer next-call interval')
                self.trace.anchor_phase((before + after) // 2 + until,
                                        after - before, before, after, until)
            except Exception as error:
                self.trace.anchor_mono_before_ns = before
                self.trace.anchor_mono_after_ns = time.monotonic_ns()
                self.trace.anchor_error = f'{type(error).__name__}: {error}'
        self.get_logger().info('Read-only state assembly; message timestamps do not prove hardware freshness. Parameters are startup-only.')

    def on_mit_feedback(self, msg):
        self.core.mit_feedback(msg, self.get_clock().now().nanoseconds, time.monotonic_ns())

    def on_joints(self, msg):
        self.core.joints(msg, self.get_clock().now().nanoseconds, time.monotonic_ns())

    def on_imu(self, index, msg):
        if not self.trace.enabled:
            self.core.imu(index, msg, self.get_clock().now().nanoseconds, time.monotonic_ns())
            return
        entry = time.monotonic_ns()
        self.core.imu(index, msg, self.get_clock().now().nanoseconds, time.monotonic_ns())
        end = time.monotonic_ns()
        self.trace.record_imu(index, ns(msg.header.stamp), entry, end)

    def publish_state(self):
        if not self.trace.enabled:
            self.publisher.publish(self.core.snapshot(self.get_clock().now().nanoseconds, time.monotonic_ns()))
            return
        start = time.monotonic_ns()
        now = self.get_clock().now().nanoseconds
        snapshot = time.monotonic_ns()
        state = self.core.snapshot(now, snapshot)
        publish_begin = time.monotonic_ns()
        self.publisher.publish(state)
        end = time.monotonic_ns()
        one, two = state.sources[1:3]
        self.trace.record_state(
            start, snapshot, publish_begin, end, ns(state.header.stamp),
            ns(state.imus[0].header.stamp), ns(state.imus[1].header.stamp),
            state.sources[0].valid, one.valid, two.valid,
            milliseconds_to_ns(one.receive_age_ms), milliseconds_to_ns(two.receive_age_ms),
            milliseconds_to_ns(one.source_age_ms), milliseconds_to_ns(two.source_age_ms))

    def destroy_node(self):
        try:
            if hasattr(self, 'trace'):
                self.trace.flush()
        finally:
            result = super().destroy_node()
        return result


def main():
    rclpy.init()
    node = None
    try:
        node = StateNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
