"""Exercise the real shadow ROS node with synthetic, non-actuating inputs."""

import math
import threading
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from std_srvs.srv import Trigger

from mech_knee_kinematics.shadow_assist import ShadowAssistNode


def require(condition, message):
    if not condition:
        raise AssertionError(message)


def wait_until(predicate, seconds=2.0):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        if predicate():
            return True
        time.sleep(0.01)
    return False


def main():
    rclpy.init()
    shadow = ShadowAssistNode()
    probe = Node('shadow_assist_test_probe')
    values = []
    statuses = []
    effort_pub = probe.create_publisher(JointState, 'knee/joint_state', 10)
    diag_pub = probe.create_publisher(DiagnosticArray, 'knee/diagnostics', 10)
    probe.create_subscription(Float64, 'assist_shadow/virtual_effort_nm',
                              lambda msg: values.append(msg.data), 10)
    probe.create_subscription(DiagnosticArray, 'assist_shadow/diagnostics',
                              lambda msg: statuses.append(msg.status[0]), 10)
    arm_client = probe.create_client(Trigger, 'assist_shadow/arm')
    reset_client = probe.create_client(Trigger, 'assist_shadow/reset')
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(shadow)
    executor.add_node(probe)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()

    def send(healthy=True):
        diagnostic = DiagnosticArray()
        status = DiagnosticStatus()
        status.name = 'knee_kinematics'
        status.level = DiagnosticStatus.OK if healthy else DiagnosticStatus.WARN
        diagnostic.status = [status]
        diag_pub.publish(diagnostic)
        joint = JointState()
        joint.name = ['knee_joint']
        joint.position = [math.radians(20.0)]
        joint.velocity = [0.0]
        effort_pub.publish(joint)

    def call(client):
        future = client.call_async(Trigger.Request())
        require(wait_until(future.done), 'service call timed out')
        return future.result()

    try:
        require(arm_client.wait_for_service(timeout_sec=2.0), 'arm unavailable')
        require(reset_client.wait_for_service(timeout_sec=2.0), 'reset unavailable')
        names = {name for name, _ in
                 probe.get_publisher_names_and_types_by_node('shadow_assist', '/')}
        application_names = names - {'/rosout', '/parameter_events'}
        require(application_names == {'/assist_shadow/virtual_effort_nm',
                                      '/assist_shadow/diagnostics'},
                f'unexpected shadow publishers: {names}')
        require(not call(arm_client).success, 'armed before a healthy knee sample')
        for _ in range(5):
            send()
            time.sleep(0.05)
        require(call(arm_client).success, 'fresh healthy knee sample did not arm')
        for _ in range(5):
            send()
            time.sleep(0.05)
        require(wait_until(lambda: any(value > 0.0 for value in values)),
                'no nonzero virtual output')
        marker = len(values)
        send(False)
        require(wait_until(lambda: any(status.level == DiagnosticStatus.ERROR
                                       for status in statuses)),
                'degraded knee did not latch fault')
        require(wait_until(lambda: len(values) > marker and values[-1] == 0.0),
                'fault did not publish zero virtual output')
        require(not call(arm_client).success, 'fault rearmed without reset')
        require(call(reset_client).success, 'reset failed')
        require(not call(arm_client).success, 'reset reused stale sample')
        print('PASS shadow ROS graph: virtual output, fault zero, reset, no motor publisher')
    finally:
        executor.shutdown()
        thread.join(timeout=2.0)
        probe.destroy_node()
        shadow.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
