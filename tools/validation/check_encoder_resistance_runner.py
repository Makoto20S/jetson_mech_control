#!/usr/bin/env python3
"""Synthetic encoder resistance trial and fault cleanup; no device access."""
import subprocess
import sys
import time
from pathlib import Path
import rclpy
from sensor_msgs.msg import JointState
from controller_manager_msgs.msg import ControllerState
from std_msgs.msg import Float64
from check_desk_trial_runner import FakeManager


class EncoderManager(FakeManager):
    def __init__(self, drop):
        super().__init__()
        self.drop = drop
        self.commands = []
        self.create_subscription(Float64, '/dm4340_effort_controller/target_effort',
                                 lambda m: self.commands.append(m.data), 100)

    def controllers(self, request, response):
        response.controller = [ControllerState(name='dm4340_effort_controller',
                                               state='active' if self.active else 'inactive')]
        return response

    def tick(self):
        if self.hw != 3:
            return
        elapsed = 0 if self.activation_time is None else time.monotonic()-self.activation_time
        if self.drop and elapsed > 1:
            return
        velocity = .1 if elapsed > .3 else 0.
        self.position += velocity*.01
        msg = JointState(); msg.header.stamp = self.get_clock().now().to_msg()
        msg.name, msg.position, msg.velocity = ['dm4340_joint'], [self.position], [velocity]
        self.joint.publish(msg)


def run(drop):
    rclpy.init()
    node = EncoderManager(drop)
    command = [sys.executable, str(Path(__file__).resolve().parents[1]/'bench/run_encoder_resistance.py'),
               '--execute', '--operator-supervised']
    child = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    try:
        deadline = time.monotonic()+15
        while child.poll() is None and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.005)
        if child.poll() is None:
            child.kill()
            raise AssertionError('runner deadline')
        out = child.communicate()[0]
        print(out)
        assert child.returncode == (1 if drop else 0), child.returncode
        assert node.hw == 2 and not node.active, node.events
        assert node.commands and all(-.2 <= c <= 0 for c in node.commands)
        assert node.commands[-1] == 0, node.commands[-5:]
        if drop:
            assert 'stale' in out or 'fresh joint feedback unavailable' in out
        print('PASS encoder runner: '+('feedback loss cleanup' if drop else 'bounded resistance and cleanup'))
    finally:
        if child.poll() is None:
            child.kill();child.communicate()
        node.destroy_node();rclpy.shutdown()


if __name__ == '__main__':
    run(False)
    run(True)
