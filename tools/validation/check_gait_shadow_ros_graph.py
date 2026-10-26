"""Exercise the real virtual gait node with synthetic ROS inputs, no hardware."""

import math
import threading
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, UInt8MultiArray, UInt32
from std_srvs.srv import Trigger

from mech_knee_kinematics.gait_shadow import GaitShadowNode


def require(condition, message):
    if not condition:
        raise AssertionError(message)


def wait_until(predicate, seconds=2.0):
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.01)
    return False


def main():
    rclpy.init()
    shadow = GaitShadowNode(parameter_overrides=[
        Parameter('pressure_validated', value=True)])
    probe = Node('gait_shadow_test_probe')
    efforts = []
    phases = []
    statuses = []
    knee_pub = probe.create_publisher(JointState, 'knee/joint_state', 10)
    diag_pub = probe.create_publisher(DiagnosticArray, 'knee/diagnostics', 10)
    total_pub = probe.create_publisher(UInt32, 'fsr/left/total', 10)
    status_pub = probe.create_publisher(UInt8MultiArray, 'ctrboard/sensor_status', 10)
    probe.create_subscription(Float64, 'gait_shadow/virtual_effort_nm',
                              lambda msg: efforts.append(msg.data), 10)
    probe.create_subscription(Float64, 'gait_shadow/phase',
                              lambda msg: phases.append(msg.data), 10)
    probe.create_subscription(DiagnosticArray, 'gait_shadow/diagnostics',
                              lambda msg: statuses.append(msg.status[0]), 10)
    arm_client = probe.create_client(Trigger, 'gait_shadow/arm')
    reset_client = probe.create_client(Trigger, 'gait_shadow/reset')
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(shadow)
    executor.add_node(probe)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()

    def send(phase, total, healthy=True):
        diagnostic = DiagnosticArray()
        item = DiagnosticStatus()
        item.name = 'knee_kinematics'
        item.level = DiagnosticStatus.OK if healthy else DiagnosticStatus.WARN
        diagnostic.status = [item]
        diag_pub.publish(diagnostic)
        total_pub.publish(UInt32(data=total))
        status_pub.publish(UInt8MultiArray(data=[phase, 0, 1, 0]))
        knee = JointState()
        knee.name = ['knee_joint']
        knee.position = [math.radians(5.0)]
        knee.velocity = [0.0]
        knee_pub.publish(knee)

    def call(client):
        future = client.call_async(Trigger.Request())
        require(wait_until(future.done), 'service call timed out')
        return future.result()

    try:
        require(arm_client.wait_for_service(timeout_sec=2.0), 'arm unavailable')
        require(reset_client.wait_for_service(timeout_sec=2.0), 'reset unavailable')
        names = {name for name, _ in
                 probe.get_publisher_names_and_types_by_node('gait_shadow', '/')}
        application = names - {'/rosout', '/parameter_events'}
        require(application == {'/gait_shadow/virtual_effort_nm',
                                '/gait_shadow/phase', '/gait_shadow/diagnostics'},
                f'unexpected publishers: {application}')
        require(not call(arm_client).success, 'armed without a gait period')
        start = time.monotonic()
        while time.monotonic() - start < 1.45:
            elapsed = time.monotonic() - start
            phase = (0 if elapsed < 0.2 or 0.8 <= elapsed < 1.2 else
                     1 if elapsed < 0.3 or 1.2 <= elapsed < 1.3 else 2)
            send(phase, 0 if phase == 0 else 100)
            time.sleep(0.02)
        require(call(arm_client).success, 'two heel strikes did not arm')
        for _ in range(8):
            send(2, 100)
            time.sleep(0.02)
        require(wait_until(lambda: any(value > 0.0 for value in efforts)),
                'no periodic virtual output')
        require(any(0.0 < value < 1.0 for value in phases if math.isfinite(value)),
                'no gait phase publication')
        marker = len(efforts)
        for _ in range(4):
            send(0, 0)
            time.sleep(0.02)
        require(wait_until(lambda: len(efforts) > marker and efforts[-1] == 0.0),
                'swing did not publish zero')
        send(0, 0, healthy=False)
        require(wait_until(lambda: any(status.level == DiagnosticStatus.ERROR
                                       for status in statuses)),
                'bad knee diagnostic did not latch fault')
        require(not call(arm_client).success, 'fault rearmed without reset')
        require(call(reset_client).success, 'reset failed')
        require(not call(arm_client).success, 'reset reused old gait data')
        print('PASS gait shadow ROS graph: phase, virtual effort, swing/fault zero, no motor publisher')
    finally:
        executor.shutdown()
        thread.join(timeout=2.0)
        probe.destroy_node()
        shadow.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
