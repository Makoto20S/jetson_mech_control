"""ROS-only gait-synchronized virtual assistance; no actuator topic or CAN."""

import math
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, UInt8MultiArray, UInt32
from std_srvs.srv import Trigger

from .gait_policy import GaitConfig, GaitShadow


class GaitShadowNode(Node):
    def __init__(self, **node_options):
        super().__init__('gait_shadow', **node_options)
        self.declare_parameter('foot_side', 'left')
        self.declare_parameter('pressure_validated', False)
        self.declare_parameter('sample_timeout_ms', 300)
        foot_side = self.get_parameter('foot_side').value
        timeout_ms = self.get_parameter('sample_timeout_ms').value
        pressure_validated = self.get_parameter('pressure_validated').value
        if foot_side not in ('left', 'right'):
            raise ValueError('foot_side must be left or right')
        if type(pressure_validated) is not bool:
            raise ValueError('pressure_validated must be boolean')
        self.foot_index = 0 if foot_side == 'left' else 1
        self.control = GaitShadow(GaitConfig(sample_timeout_s=timeout_ms / 1000.0),
                                  pressure_validated)
        self.latest_total = None
        self.latest_total_time = None
        self.effort_pub = self.create_publisher(
            Float64, 'gait_shadow/virtual_effort_nm', 10)
        self.phase_pub = self.create_publisher(Float64, 'gait_shadow/phase', 10)
        self.diagnostic_pub = self.create_publisher(
            DiagnosticArray, 'gait_shadow/diagnostics', 10)
        self.create_subscription(JointState, 'knee/joint_state', self._on_knee, 10)
        self.create_subscription(DiagnosticArray, 'knee/diagnostics',
                                 self._on_diagnostic, 10)
        self.create_subscription(UInt32, f'fsr/{foot_side}/total',
                                 self._on_total, 10)
        self.create_subscription(UInt8MultiArray, 'ctrboard/sensor_status',
                                 self._on_status, 10)
        self.create_service(Trigger, 'gait_shadow/arm', self._arm)
        self.create_service(Trigger, 'gait_shadow/reset', self._reset)
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
        self.control.update_diagnostic(
            status is not None and status.level == DiagnosticStatus.OK,
            time.monotonic())

    def _on_total(self, msg):
        self.latest_total = msg.data
        self.latest_total_time = time.monotonic()

    def _on_status(self, msg):
        now = time.monotonic()
        if (len(msg.data) < 4 or self.latest_total_time is None or
                not 0.0 <= now - self.latest_total_time <= 0.15):
            self.control.invalidate_pressure('missing paired foot phase or total')
            return
        self.control.update_pressure(int(msg.data[self.foot_index]),
                                     self.latest_total, now)

    def _arm(self, request, response):
        del request
        response.success = self.control.arm(time.monotonic())
        response.message = ('virtual gait assistance armed' if response.success
                            else 'requires validated foot data, two heel strikes and fresh knee data')
        return response

    def _reset(self, request, response):
        del request
        self.control.reset()
        self.latest_total = None
        self.latest_total_time = None
        response.success = True
        response.message = 'virtual gait assistance disarmed'
        return response

    def _tick(self):
        self.control.tick(time.monotonic())
        self.effort_pub.publish(Float64(data=self.control.effort_nm))
        self.phase_pub.publish(Float64(data=(self.control.phase
                                             if self.control.phase is not None
                                             else math.nan)))
        status = DiagnosticStatus()
        status.name = 'gait_shadow'
        status.hardware_id = 'virtual_only'
        status.level = (DiagnosticStatus.OK if self.control.state == 'armed' else
                        DiagnosticStatus.ERROR if self.control.state == 'fault' else
                        DiagnosticStatus.WARN)
        status.message = self.control.reason or 'virtual gait assistance active'
        status.values = [KeyValue(key='state', value=self.control.state),
                         KeyValue(key='actuator_output', value='none'),
                         KeyValue(key='pressure_validated',
                                  value=str(self.control.pressure_validated).lower())]
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        array.status = [status]
        self.diagnostic_pub.publish(array)


def main(args=None):
    from rclpy.executors import ExternalShutdownException

    rclpy.init(args=args)
    node = None
    try:
        node = GaitShadowNode()
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
