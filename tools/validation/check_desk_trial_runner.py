#!/usr/bin/env python3
"""Synthetic lifecycle services and feedback for bounded desk runner; no hardware."""
import math
import subprocess
import sys
import time
from pathlib import Path
import rclpy
from rclpy.node import Node
from controller_manager_msgs.srv import (
    ListControllers, ListHardwareComponents, SetHardwareComponentState, SwitchController)
from controller_manager_msgs.msg import ControllerState, HardwareComponentState
from lifecycle_msgs.msg import State
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from geometry_msgs.msg import QuaternionStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64


class FakeManager(Node):
    def __init__(self):
        super().__init__('desk_trial_fake_manager')
        self.hw = State.PRIMARY_STATE_INACTIVE
        self.active = False
        self.position = self.target = 0.0
        self.activation_time = None
        self.events = []
        self.drop_input = False
        for name, kind, handler in (
                ('list_controllers', ListControllers, self.controllers),
                ('list_hardware_components', ListHardwareComponents, self.hardware),
                ('set_hardware_component_state', SetHardwareComponentState, self.set_hw),
                ('switch_controller', SwitchController, self.switch)):
            self.create_service(kind, '/controller_manager/'+name, handler)
        self.joint = self.create_publisher(JointState, '/joint_states', 10)
        self.pose = self.create_publisher(QuaternionStamped, 'attitude/relative/orientation', 10)
        self.diag = self.create_publisher(DiagnosticArray, 'ctrboard/diagnostics', 10)
        self.create_subscription(Float64, '/dm4340_velocity_controller/target_velocity',
                                 lambda msg: setattr(self, 'target', msg.data), 10)
        self.create_timer(0.01, self.tick)

    def controllers(self, request, response):
        response.controller = [ControllerState(name='dm4340_velocity_controller',
                                               state='active' if self.active else 'inactive')]
        return response

    def hardware(self, request, response):
        response.component = [HardwareComponentState(name='dm4340_system', state=State(id=self.hw))]
        return response

    def set_hw(self, request, response):
        self.hw = request.target_state.id
        self.events.append(('hardware', self.hw))
        response.ok, response.state = True, State(id=self.hw)
        return response

    def switch(self, request, response):
        self.active = bool(request.activate_controllers)
        self.activation_time = time.monotonic() if self.active else None
        self.events.append(('controller', self.active))
        response.ok = True
        return response

    def tick(self):
        stamp = self.get_clock().now().to_msg()
        if self.hw == State.PRIMARY_STATE_ACTIVE:
            velocity = self.target if self.active else 0.0
            self.position += velocity*0.01
            msg = JointState(); msg.header.stamp = stamp
            msg.name, msg.position, msg.velocity = ['dm4340_joint'], [self.position], [velocity]
            self.joint.publish(msg)
        elapsed = 0 if self.activation_time is None else time.monotonic()-self.activation_time
        # Arm occurs after 0.5 s of active zero hold, then a positive pulse and release.
        if self.drop_input and elapsed > 1.2:
            return
        angle = 20 if 1.0 < elapsed < 2.0 else 0
        msg = QuaternionStamped(); msg.header.stamp = stamp
        msg.quaternion.w = math.cos(math.radians(angle)/2)
        msg.quaternion.y = math.sin(math.radians(angle)/2)
        self.pose.publish(msg)
        status = DiagnosticStatus(); status.name = 'ctrboard_sensor_safety'; status.level = DiagnosticStatus.OK
        diag = DiagnosticArray(); diag.status = [status]; self.diag.publish(diag)


def main():
    rclpy.init()
    node = FakeManager()
    script = Path(__file__).resolve().parents[1]/'bench/run_desk_imu_follow.py'
    try:
        for flags, expected, drop in ((['--preflight'], 0, False),
                                      (['--execute', '--operator-supervised'], 0, False),
                                      (['--preflight'], 0, False),
                                      (['--execute', '--operator-supervised'], 1, True),
                                      (['--preflight'], 0, False)):
            node.drop_input = drop
            proc = subprocess.Popen([sys.executable, str(script), *flags], stdout=subprocess.PIPE,
                                    stderr=subprocess.STDOUT, text=True)
            deadline = time.monotonic()+20
            while proc.poll() is None and time.monotonic()<deadline:
                rclpy.spin_once(node, timeout_sec=0.002)
            if proc.poll() is None:
                proc.kill()
            output = proc.communicate(timeout=2)[0]
            print(output)
            assert proc.returncode == expected, (flags, output)
            if drop:
                assert 'stale sensor or motor feedback' in output
            assert not node.active and node.hw == State.PRIMARY_STATE_INACTIVE
        assert node.events == [('hardware', State.PRIMARY_STATE_ACTIVE), ('controller', True),
                               ('controller', False), ('hardware', State.PRIMARY_STATE_INACTIVE)]*2, node.events
        print('PASS synthetic desk runner: read-only preflights, activation, real ROS targets, '
              'bounded simulated motion, planned zero stop, stale-input abort and inactive cleanup')
    finally:
        node.destroy_node(); rclpy.shutdown()


if __name__ == '__main__':
    main()
