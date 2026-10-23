"""Explicitly armed desktop relative-orientation -> velocity preview/bench node."""
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import QuaternionStamped
from diagnostic_msgs.msg import DiagnosticArray
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from std_srvs.srv import Trigger
from .desk_follow import DeskFollow


class DeskFollowNode(Node):
    def __init__(self, **kwargs):
        super().__init__('desk_imu_follow', **kwargs)
        self.declare_parameter('output_mode', 'preview')
        mode = self.get_parameter('output_mode').value
        if mode not in ('preview', 'motor'):
            raise ValueError('output_mode must be preview or motor')
        self.logic = DeskFollow(motor=mode == 'motor')
        self.was_armed = False
        self.last_stamp = 0
        self.last_motor_stamp = 0
        self.last_health_stamp = 0
        topic = ('/dm4340_velocity_controller/target_velocity' if self.logic.motor_mode
                 else 'desk_imu_follow/preview_velocity')
        self.target_topic = topic
        self.output = self.create_publisher(Float64, topic, 10)
        self.create_subscription(QuaternionStamped, 'attitude/relative/orientation', self.pose, 10)
        self.create_subscription(DiagnosticArray, 'ctrboard/diagnostics', self.health, 10)
        if self.logic.motor_mode:
            self.create_subscription(JointState, '/joint_states', self.feedback, 10)
        self.create_service(Trigger, 'desk_imu_follow/arm', self.arm)
        self.create_timer(0.01, self.tick)

    @staticmethod
    def stamp(msg):
        return msg.header.stamp.sec*10**9+msg.header.stamp.nanosec

    def sampled_at(self, stamp):
        return time.monotonic() - (self.get_clock().now().nanoseconds - stamp) / 1e9

    def pose(self, msg):
        stamp = self.stamp(msg)
        if (stamp <= self.last_stamp or
                not 0 <= self.get_clock().now().nanoseconds-stamp <= 300_000_000):
            self.logic.fault('missing or repeated IMU timestamp')
            return
        self.last_stamp = stamp
        q = msg.quaternion
        self.logic.orientation((q.w, q.x, q.y, q.z), self.sampled_at(stamp))

    def health(self, msg):
        stamp = self.stamp(msg)
        if (stamp <= self.last_health_stamp or
                not 0 <= self.get_clock().now().nanoseconds-stamp <= 300_000_000):
            self.logic.fault('invalid sensor health timestamp')
            return
        self.last_health_stamp = stamp
        for status in msg.status:
            if status.name == 'ctrboard_sensor_safety':
                level = (int.from_bytes(status.level, 'little')
                         if isinstance(status.level, bytes) else int(status.level))
                self.logic.healthy(level == 0, self.sampled_at(stamp))

    def feedback(self, msg):
        if 'dm4340_joint' not in msg.name:
            return
        i = msg.name.index('dm4340_joint')
        stamp = self.stamp(msg)
        if (stamp <= self.last_motor_stamp or
                not 0 <= self.get_clock().now().nanoseconds-stamp <= 150_000_000 or
                i >= len(msg.position) or i >= len(msg.velocity)):
            self.logic.fault('invalid motor feedback stamp or fields')
            return
        self.last_motor_stamp = stamp
        self.logic.motor_feedback(msg.position[i], msg.velocity[i], self.sampled_at(stamp))

    def arm(self, request, response):
        if self.logic.motor_mode and self.count_publishers(self.target_topic) != 1:
            response.success, response.message = False, 'motor target publisher conflict'
            return response
        response.success = self.logic.arm(time.monotonic())
        self.was_armed = response.success
        response.message = 'armed for at most 5 seconds' if response.success else self.logic.reason or 'inputs unavailable'
        return response

    def tick(self):
        was_armed = self.was_armed or self.logic.state == 'armed'
        if self.logic.motor_mode and was_armed and self.count_publishers(self.target_topic) != 1:
            self.logic.fault('motor target publisher conflict')
        target = self.logic.tick(time.monotonic())
        if (was_armed or (self.logic.state == 'idle' and self.logic.motor_mode
                          and self.logic.available(time.monotonic()))):
            self.output.publish(Float64(data=target))
            if was_armed and self.logic.state != 'armed':
                self.get_logger().warn(f'{self.logic.state}: {self.logic.reason}; target refresh stopped')
        self.was_armed = self.logic.state == 'armed'


def main(args=None):
    from rclpy.executors import ExternalShutdownException
    rclpy.init(args=args)
    node = None
    try:
        node = DeskFollowNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            if node.logic.state == 'armed':
                node.output.publish(Float64(data=0.0))
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
