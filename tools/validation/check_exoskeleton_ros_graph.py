#!/usr/bin/env python3
"""Synthetic paired IMUs -> actual estimator -> whole-robot application preview.

No serial bridge, motor manager, device port or hardware lifecycle is started.
"""
import json
import threading
import time
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.parameter import Parameter
from std_msgs.msg import String
from mech_knee_kinematics.node import KneeKinematicsNode
from mech_assist_control.robot_node import ExoskeletonNode
from check_knee_ros_graph import Probe


def main():
    rclpy.init(args=['--ros-args', '-p', 'static_seconds:=0.5',
                     '-p', 'motion_seconds:=1.5', '-p', 'flexion_sign:=1'])
    knee = KneeKinematicsNode()
    app = ExoskeletonNode(parameter_overrides=[Parameter('policy', value='hold_velocity')])
    probe = Probe()
    envelopes = []
    probe.create_subscription(String, 'exoskeleton/command_envelope',
                              lambda msg: envelopes.append(json.loads(msg.data)), 10)
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(knee); executor.add_node(app)
    worker = threading.Thread(target=executor.spin, daemon=True); worker.start()
    try:
        probe.service('knee/reset_calibration')
        probe.feed(.3)
        assert not app.supervisor.arm(time.monotonic()), 'uncalibrated inputs armed'
        probe.service('knee/start_static_calibration'); probe.feed(.85)
        probe.service('knee/start_axis_calibration'); probe.feed(1.85, motion=True)
        probe.service('knee/start_static_calibration'); probe.feed(.85)
        probe.feed(.25)
        probe.service('exoskeleton/arm')
        probe.ramp(10.0)
        probe.feed(.25, angle=10)
        assert any(e['valid'] and abs(e['values'][0]) > .01 for e in envelopes), envelopes[-3:]
        probe.feed(.35, angle=10, valid=False)
        deadline = time.monotonic()+1
        while app.supervisor.state != 'fault' and time.monotonic()<deadline:
            rclpy.spin_once(probe, timeout_sec=.01)
        assert app.supervisor.state == 'fault'
        assert app.supervisor.state == 'fault'
        publishers = {name for name, _ in probe.get_publisher_names_and_types_by_node(
            'exoskeleton_control', '/')}
        assert not any('target_effort' in name or 'target_velocity' in name for name in publishers)
        print('PASS whole-robot ROS graph: raw paired IMUs, static-axis-static calibration, '
              'explicit arm, nonzero bounded preview, invalid IMU fault and no motor publisher')
    finally:
        executor.shutdown(); worker.join(timeout=2)
        probe.destroy_node(); app.stop(); app.destroy_node(); knee.destroy_node(); rclpy.shutdown()


if __name__ == '__main__':
    main()
