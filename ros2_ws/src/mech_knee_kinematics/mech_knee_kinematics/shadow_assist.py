"""ROS shadow assistance monitor. It never publishes actuator commands."""

import math
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from std_srvs.srv import Trigger

from .offline_loop import LoopConfig, Measurement, VirtualAssistSupervisor


class ShadowAssist:
    def __init__(self, timeout_s=0.3, target_deg=30.0):
        if not math.isfinite(target_deg) or not -10.0 <= target_deg <= 120.0:
            raise ValueError('invalid virtual target')
        self.supervisor = VirtualAssistSupervisor(
            LoopConfig(sample_timeout_s=timeout_s))
        self.target_deg = target_deg
        self.last_sample = None
        self.last_diagnostic_time = None
        self.diagnostic_ok = False
        self.effort_nm = 0.0

    @property
    def state(self):
        return self.supervisor.state

    @property
    def reason(self):
        return self.supervisor.reason

    def update_diagnostic(self, healthy, now):
        self.last_diagnostic_time = now
        self.diagnostic_ok = healthy
        if not healthy and self.state == 'armed':
            self.effort_nm = self.supervisor._fault('knee diagnostic not healthy')

    def update_knee(self, angle_deg, rate_dps, now):
        sample = Measurement(now, angle_deg, rate_dps, True)
        self.last_sample = sample
        if self.state == 'armed':
            if not self._healthy(now):
                self.effort_nm = self.supervisor._fault('knee diagnostic unavailable')
            else:
                self.effort_nm = self.supervisor.command(now, sample, self.target_deg)

    def _healthy(self, now):
        return (self.diagnostic_ok and self.last_diagnostic_time is not None and
                0.0 <= now - self.last_diagnostic_time <=
                self.supervisor.config.sample_timeout_s)

    def arm(self, now):
        if not self._healthy(now):
            return False
        if self.supervisor.arm(self.last_sample, now):
            self.effort_nm = 0.0
            return True
        return False

    def reset(self):
        self.supervisor.reset()
        self.last_sample = None
        self.last_diagnostic_time = None
        self.diagnostic_ok = False
        self.effort_nm = 0.0

    def tick(self, now):
        if self.state == 'armed':
            if not self._healthy(now):
                self.effort_nm = self.supervisor._fault('knee diagnostic timeout')
            elif self.last_sample is None or not self.supervisor._valid_sample(
                    self.last_sample, now):
                self.effort_nm = self.supervisor._fault('knee measurement timeout')
        return self.effort_nm


class ShadowAssistNode(Node):
    def __init__(self):
        super().__init__('shadow_assist')
        self.declare_parameter('sample_timeout_ms', 300)
        self.declare_parameter('target_deg', 30.0)
        timeout_ms = self.get_parameter('sample_timeout_ms').value
        if not 100 <= timeout_ms <= 2000:
            raise ValueError('sample_timeout_ms must be in [100, 2000]')
        self.control = ShadowAssist(
            timeout_ms / 1000.0, self.get_parameter('target_deg').value)
        self.effort_pub = self.create_publisher(Float64,
                                                'assist_shadow/virtual_effort_nm', 10)
        self.diagnostic_pub = self.create_publisher(
            DiagnosticArray, 'assist_shadow/diagnostics', 10)
        self.create_subscription(JointState, 'knee/joint_state', self._on_knee, 10)
        self.create_subscription(DiagnosticArray, 'knee/diagnostics',
                                 self._on_diagnostic, 10)
        self.create_service(Trigger, 'assist_shadow/arm', self._arm)
        self.create_service(Trigger, 'assist_shadow/reset', self._reset)
        self.create_timer(0.05, self._tick)

    def _on_knee(self, msg):
        if 'knee_joint' not in msg.name:
            return
        index = msg.name.index('knee_joint')
        if index >= len(msg.position) or index >= len(msg.velocity):
            return
        self.control.update_knee(math.degrees(msg.position[index]),
                                 math.degrees(msg.velocity[index]), time.monotonic())

    def _on_diagnostic(self, msg):
        status = next((item for item in msg.status
                       if item.name == 'knee_kinematics'), None)
        healthy = status is not None and status.level == DiagnosticStatus.OK
        self.control.update_diagnostic(healthy, time.monotonic())

    def _arm(self, request, response):
        del request
        response.success = self.control.arm(time.monotonic())
        response.message = 'virtual output armed' if response.success else (
            'requires fresh knee sample and healthy diagnostic')
        return response

    def _reset(self, request, response):
        del request
        self.control.reset()
        response.success = True
        response.message = 'virtual output disarmed; arm again after fresh data'
        return response

    def _tick(self):
        self.control.tick(time.monotonic())
        effort = Float64()
        effort.data = self.control.effort_nm
        self.effort_pub.publish(effort)
        status = DiagnosticStatus()
        status.name = 'shadow_assist'
        status.hardware_id = 'virtual_only'
        status.level = DiagnosticStatus.OK if self.control.state == 'armed' else (
            DiagnosticStatus.ERROR if self.control.state == 'fault'
            else DiagnosticStatus.WARN)
        status.message = self.control.reason or 'virtual output active'
        status.values = [KeyValue(key='state', value=self.control.state),
                         KeyValue(key='actuator_output', value='none')]
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        array.status = [status]
        self.diagnostic_pub.publish(array)


def main(args=None):
    from rclpy.executors import ExternalShutdownException

    rclpy.init(args=args)
    node = None
    try:
        node = ShadowAssistNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
