"""One opt-in knee-control strategy per ROS node instance."""

import math
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from std_srvs.srv import Trigger

from .motor_follow import MotorFollowLogic
from .modes import resolve_mode
from .resistance import (ResistanceConfig, ResistanceLogic,
                         map_knee_effort_to_motor)


class KneeControlNode(Node):
    def __init__(self, **node_options):
        super().__init__('knee_motor_follow', **node_options)
        self.declare_parameter('control_strategy', 'velocity_follow')
        self.declare_parameter('output_mode', 'preview')
        self.declare_parameter('input_mode', 'imu')
        self.declare_parameter('joint_mapping_verified', False)
        self.declare_parameter('motor_to_knee_scale', 0.0)
        self.declare_parameter('motor_to_knee_offset_rad', 0.0)
        self.declare_parameter('motor_joint_name', 'dm4340_joint')
        self.declare_parameter('amplitude_deg', 20.0)
        self.declare_parameter('motor_run_limit_s', 5.0)
        self.declare_parameter('simulated_travel_limit_rad', 0.5)
        self.declare_parameter('resistance_damping_nm_s_rad', 0.5)
        self.declare_parameter('resistance_max_torque_nm', 0.3)
        self.declare_parameter('resistance_run_limit_s', 10.0)
        self.declare_parameter('resistance_motor_max_effort_nm', 0.3)
        strategy = self.get_parameter('control_strategy').value
        self.strategy = strategy
        mode = self.get_parameter('output_mode').value
        input_mode = self.get_parameter('input_mode').value
        self.selected_mode = resolve_mode(strategy, input_mode, mode)
        self.resistance_mode = self.selected_mode.startswith('resistance_')
        self.motor_mode = mode == 'motor'
        verified = self.get_parameter('joint_mapping_verified').value
        if (type(verified) is not bool or
                (self.motor_mode and input_mode == 'imu' and not verified)):
            raise ValueError('motor mode requires an independently verified joint mapping')
        self.motor_joint_name = self.get_parameter('motor_joint_name').value
        if not self.motor_joint_name:
            raise ValueError('motor_joint_name is required')
        self.motor_scale = self.get_parameter('motor_to_knee_scale').value
        self.motor_offset = self.get_parameter('motor_to_knee_offset_rad').value
        if self.selected_mode == 'resistance_motor':
            if (not isinstance(self.motor_scale, (float, int)) or
                    not math.isfinite(self.motor_scale) or self.motor_scale == 0 or
                    not isinstance(self.motor_offset, (float, int)) or
                    not math.isfinite(self.motor_offset)):
                raise ValueError('resistance motor mode requires measured joint mapping')
            self.motor_effort_limit = self.get_parameter(
                'resistance_motor_max_effort_nm').value
            if (not isinstance(self.motor_effort_limit, (float, int)) or
                    not math.isfinite(self.motor_effort_limit) or
                    not 0 < self.motor_effort_limit <= 0.3):
                raise ValueError('motor effort limit exceeds controller configuration')
        self.motor_sample = None
        self.simulated_input = self.selected_mode == 'periodic_bench'
        self.simulated_travel_limit_rad = self.get_parameter(
            'simulated_travel_limit_rad').value
        if (self.simulated_input and
                (not isinstance(self.simulated_travel_limit_rad, (int, float)) or
                 not 0 < self.simulated_travel_limit_rad <= 1.0)):
            raise ValueError('invalid simulated travel limit')
        self.last_knee_stamp = 0
        self.last_motor_stamp = 0
        self.sim_motor_zero = None
        if self.resistance_mode:
            self.control = ResistanceLogic(ResistanceConfig(
                damping_nm_s_rad=self.get_parameter(
                    'resistance_damping_nm_s_rad').value,
                max_torque_nm=self.get_parameter(
                    'resistance_max_torque_nm').value,
                run_limit_s=self.get_parameter(
                    'resistance_run_limit_s').value))
        else:
            self.control = MotorFollowLogic(
                motor_mode=self.motor_mode,
                motor_to_knee_scale=(1.0 if self.simulated_input else
                                     self.get_parameter('motor_to_knee_scale').value),
                motor_to_knee_offset_rad=(0.0 if self.simulated_input else
                                          self.get_parameter('motor_to_knee_offset_rad').value),
                amplitude_deg=self.get_parameter('amplitude_deg').value,
                motor_run_limit_s=self.get_parameter('motor_run_limit_s').value,
                bench_simulation=self.simulated_input)
        topic = ('/dm4340_effort_controller/target_effort'
                 if self.selected_mode == 'resistance_motor' else
                 'knee_resistance/preview_effort_nm' if self.resistance_mode else
                 '/dm4340_velocity_controller/target_velocity' if self.motor_mode
                 else 'knee_motor_follow/preview_velocity')
        self.command_pub = self.create_publisher(Float64, topic, 10)
        self.diagnostic_pub = self.create_publisher(
            DiagnosticArray, ('knee_resistance/diagnostics' if self.resistance_mode
                              else 'knee_motor_follow/diagnostics'), 10)
        if not self.simulated_input:
            self.create_subscription(JointState, 'knee/joint_state', self._on_knee, 10)
            self.create_subscription(DiagnosticArray, 'knee/diagnostics',
                                     self._on_diagnostic, 10)
        if self.motor_mode:
            self.create_subscription(JointState, '/joint_states',
                                     self._on_motor, 10)
        self.create_service(Trigger, 'knee_control/arm', self._arm)
        self.create_service(Trigger, 'knee_control/reset', self._reset)
        if not self.resistance_mode:
            self.create_service(Trigger, 'knee_motor_follow/arm', self._arm)
            self.create_service(Trigger, 'knee_motor_follow/reset', self._reset)
        self._was_armed = False
        self.create_timer(0.02, self._tick)
        self.add_on_set_parameters_callback(self._reject_runtime_configuration)

    def _reject_runtime_configuration(self, parameters):
        del parameters
        return SetParametersResult(
            successful=False, reason='restart knee control to change configuration')

    def _on_knee(self, msg):
        if 'knee_joint' not in msg.name:
            return
        index = msg.name.index('knee_joint')
        if index >= len(msg.position) or index >= len(msg.velocity):
            self.control._fault('incomplete knee joint state')
            return
        stamp = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        if stamp <= self.last_knee_stamp:
            self.control._fault('missing or repeated knee sample timestamp')
            return
        self.last_knee_stamp = stamp
        self.control.update_knee(msg.position[index], msg.velocity[index],
                                 time.monotonic())

    def _on_motor(self, msg):
        if self.motor_joint_name not in msg.name:
            return
        index = msg.name.index(self.motor_joint_name)
        if index >= len(msg.position) or index >= len(msg.velocity):
            self.control._fault('incomplete motor joint state')
            return
        stamp = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        if stamp <= self.last_motor_stamp:
            self.control._fault('missing or repeated motor feedback timestamp')
            return
        self.last_motor_stamp = stamp
        now = time.monotonic()
        position = msg.position[index]
        velocity = msg.velocity[index]
        if self.selected_mode == 'resistance_motor':
            if not math.isfinite(position) or not math.isfinite(velocity):
                self.control._fault('invalid motor feedback')
                return
            self.motor_sample = (position, velocity, now)
            return
        if self.simulated_input and self.sim_motor_zero is None:
            self.sim_motor_zero = position
            self.control.offset = -position
        self.control.update_motor(position, velocity, now)
        if self.simulated_input and self.control.state != 'fault':
            if abs(position - self.sim_motor_zero) > self.simulated_travel_limit_rad:
                self.control._fault('simulated trial displacement limit exceeded')
                return
            self.control.update_diagnostic(True, now)
            self.control.update_knee(position - self.sim_motor_zero, velocity, now)

    def _on_diagnostic(self, msg):
        status = next((item for item in msg.status
                       if item.name == 'knee_kinematics'), None)
        self.control.update_diagnostic(
            status is not None and status.level == DiagnosticStatus.OK,
            time.monotonic())

    def _arm(self, request, response):
        del request
        now = time.monotonic()
        response.success = (self.selected_mode != 'resistance_motor' or
                            self._resistance_motor_healthy(now)) and self.control.arm(now)
        response.message = ('knee control armed' if response.success else
                            'requires fresh knee data, healthy diagnostics and '
                            'mapped motor feedback when applicable')
        if response.success:
            self._was_armed = True
        return response

    def _reset(self, request, response):
        del request
        if self._was_armed:
            self.command_pub.publish(Float64(data=0.0))
        self.control.reset()
        self.sim_motor_zero = None
        self.motor_sample = None
        self._was_armed = False
        response.success = True
        response.message = 'control disarmed; fresh inputs and arm required'
        return response

    def _tick(self):
        if (self.selected_mode == 'resistance_motor' and
                self.control.state == 'armed' and
                not self._resistance_motor_healthy(time.monotonic())):
            self.control._fault('motor feedback stale or disagrees with knee')
        command = self.control.tick(time.monotonic())
        if self.selected_mode == 'resistance_preview':
            self.command_pub.publish(Float64(data=command))
            self._was_armed = self.control.state == 'armed'
        elif self.control.state == 'armed':
            if self.selected_mode == 'resistance_motor':
                command = map_knee_effort_to_motor(
                    command, self.motor_scale, self.motor_sample[1],
                    self.motor_effort_limit)
            self.command_pub.publish(Float64(data=command))
            self._was_armed = True
        elif self._was_armed:
            self.command_pub.publish(Float64(data=0.0))
            self._was_armed = False
        status = DiagnosticStatus()
        status.name = ('knee_resistance' if self.resistance_mode
                       else 'knee_motor_follow')
        status.hardware_id = ('dm4340' if self.motor_mode else 'preview_only')
        status.level = (DiagnosticStatus.OK if self.control.state == 'armed' else
                        DiagnosticStatus.ERROR if self.control.state == 'fault' else
                        DiagnosticStatus.WARN)
        status.message = self.control.reason or self.selected_mode
        status.values = [KeyValue(key='state', value=self.control.state),
                         KeyValue(key='control_strategy', value=self.strategy),
                         KeyValue(key='selected_mode', value=self.selected_mode),
                         KeyValue(key='output_mode', value=(
                             'motor' if self.motor_mode else 'preview')),
                         KeyValue(key='input_mode', value=(
                             'motor_feedback_simulation' if self.simulated_input
                             else 'imu'))]
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        array.status = [status]
        self.diagnostic_pub.publish(array)

    def _resistance_motor_healthy(self, now):
        if self.motor_sample is None or self.control.sample is None:
            return False
        position, velocity, received = self.motor_sample
        knee_angle, knee_velocity, knee_received = self.control.sample
        mapped_angle = position * self.motor_scale + self.motor_offset
        mapped_velocity = velocity * self.motor_scale
        return (0 <= now - received <= 0.1 and
                0 <= now - knee_received <= self.control.config.sample_timeout_s and
                abs(mapped_angle - knee_angle) <= 0.15 and
                abs(mapped_velocity) <= self.control.config.max_velocity_rad_s and
                (abs(knee_velocity) <= 0.02 or abs(mapped_velocity) <= 0.02 or
                 knee_velocity * mapped_velocity > 0))

    def stop(self):
        if self._was_armed:
            self.command_pub.publish(Float64(data=0.0))
        self._was_armed = False


MotorFollowNode = KneeControlNode


def main(args=None):
    from rclpy.executors import ExternalShutdownException

    rclpy.init(args=args)
    node = None
    try:
        node = KneeControlNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.stop()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
