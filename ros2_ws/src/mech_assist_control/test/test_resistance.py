"""Resistance policy and preview-only ROS publication tests."""

import math
import sys
import time
import types
import unittest
from unittest.mock import patch

from mech_assist_control.resistance import (ResistanceConfig, ResistanceLogic,
                                            map_knee_effort_to_motor)


def ready(control, now=1.0, velocity=0.5):
    control.update_diagnostic(True, now)
    control.update_knee(0.5, velocity, now)
    assert control.arm(now)


class ResistanceTests(unittest.TestCase):
    def test_motor_torque_mapping_direction_limit_and_passivity(self):
        self.assertAlmostEqual(
            map_knee_effort_to_motor(-0.2, -2.0, -0.5, 0.3), 0.3)
        self.assertAlmostEqual(
            map_knee_effort_to_motor(0.2, -0.5, 0.5, 0.3), -0.1)
        self.assertEqual(
            map_knee_effort_to_motor(-0.2, 1.0, -0.01, 0.3), 0.0)
        with self.assertRaises(ValueError):
            map_knee_effort_to_motor(-0.2, 0.0, 0.5, 0.3)

    def test_torque_opposes_both_directions_and_is_bounded(self):
        control = ResistanceLogic()
        ready(control)
        for index in range(1, 20):
            torque = control.tick(1.0 + index * 0.02)
            self.assertLessEqual(torque * 0.5, 0.0)
            self.assertLessEqual(abs(torque), 0.3)
        control.update_knee(0.5, -0.5, 1.41)
        self.assertGreaterEqual(control.tick(1.42), 0.0)
        self.assertLessEqual(control.effort_nm * -0.5, 0.0)

    def test_rest_deadband_zero_and_limit(self):
        control = ResistanceLogic()
        ready(control)
        control.tick(1.02)
        control.update_knee(0.5, 0.01, 1.03)
        self.assertEqual(control.tick(1.04), 0.0)
        with self.assertRaises(ValueError):
            ResistanceConfig(max_torque_nm=math.nan)
        with self.assertRaises(ValueError):
            ResistanceConfig(max_torque_nm=3.0)

    def test_arm_requires_fresh_diagnostic_and_sample(self):
        control = ResistanceLogic()
        self.assertFalse(control.arm(1.0))
        control.update_diagnostic(True, 1.0)
        self.assertFalse(control.arm(1.0))
        control.update_knee(0.5, 0.2, 1.0)
        self.assertFalse(control.arm(1.3))
        self.assertTrue(control.arm(1.01))

    def test_faults_zero_and_latch_until_reset(self):
        control = ResistanceLogic()
        ready(control)
        self.assertLess(control.tick(1.02), 0.0)
        control.update_diagnostic(False, 1.03)
        self.assertEqual(control.tick(1.04), 0.0)
        self.assertEqual(control.state, 'fault')
        control.update_diagnostic(True, 1.05)
        control.update_knee(0.5, 0.2, 1.05)
        self.assertFalse(control.arm(1.06))
        control.reset()
        ready(control, 2.0)
        self.assertEqual(control.tick(2.21), 0.0)
        self.assertEqual(control.state, 'fault')

    def test_invalid_sample_and_run_limit(self):
        control = ResistanceLogic()
        ready(control)
        control.update_knee(math.nan, 0.5, 1.02)
        self.assertEqual(control.tick(1.03), 0.0)
        self.assertEqual(control.state, 'fault')
        control.reset()
        ready(control, 2.0)
        control.update_diagnostic(True, 11.99)
        control.update_knee(0.5, 0.5, 11.99)
        self.assertEqual(control.tick(12.0), 0.0)
        self.assertEqual(control.state, 'fault')

    def test_cyclic_motion_never_adds_mechanical_work(self):
        control = ResistanceLogic(ResistanceConfig(run_limit_s=20.0))
        ready(control)
        work = 0.0
        for index in range(1, 400):
            now = 1.0 + 0.02 * index
            velocity = 0.6 * math.sin(2 * math.pi * 0.5 * (now - 1.0))
            control.update_knee(0.5, velocity, now)
            control.update_diagnostic(True, now)
            torque = control.tick(now)
            self.assertLessEqual(torque * velocity, 1e-12)
            work += torque * velocity * 0.02
        self.assertLess(work, 0.0)


class FakePublisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class FakeNode:
    def __init__(self, name, **kwargs):
        self.name = name
        self.overrides = {}
        args = kwargs.get('cli_args', [])
        for index, arg in enumerate(args):
            if arg == '-p' and index + 1 < len(args):
                key, value = args[index + 1].split(':=', 1)
                self.overrides[key] = value
        self.parameters = {}
        self.publishers = {}
        self.services = []

    def declare_parameter(self, name, default):
        self.parameters[name] = self.overrides.get(name, default)

    def get_parameter(self, name):
        return types.SimpleNamespace(value=self.parameters[name])

    def create_publisher(self, kind, topic, depth):
        del kind, depth
        self.publishers[topic] = FakePublisher()
        return self.publishers[topic]

    def create_subscription(self, *args):
        return None

    def create_service(self, *args):
        self.services.append(args[1])
        return None

    def create_timer(self, *args):
        return None

    def add_on_set_parameters_callback(self, callback):
        self.parameter_callback = callback

    def get_clock(self):
        return types.SimpleNamespace(now=lambda: types.SimpleNamespace(
            nanoseconds=int(time.monotonic()*1e9), to_msg=lambda: types.SimpleNamespace()))


class FakeStatus:
    OK = 0
    WARN = 1
    ERROR = 2

    def __init__(self):
        self.name = ''
        self.level = self.WARN
        self.values = []


class ResistanceNodeTests(unittest.TestCase):
    def test_ros_preview_publishes_opposing_torque_then_zero_on_fault(self):
        def module(name, **symbols):
            result = types.ModuleType(name)
            vars(result).update(symbols)
            return result

        fake = {
            'rclpy': module('rclpy'),
            'rclpy.node': module('rclpy.node', Node=FakeNode),
            'rcl_interfaces': module('rcl_interfaces'),
            'rcl_interfaces.msg': module(
                'rcl_interfaces.msg', SetParametersResult=lambda **kwargs:
                types.SimpleNamespace(**kwargs)),
            'diagnostic_msgs': module('diagnostic_msgs'),
            'diagnostic_msgs.msg': module(
                'diagnostic_msgs.msg', DiagnosticStatus=FakeStatus,
                DiagnosticArray=lambda: types.SimpleNamespace(
                    header=types.SimpleNamespace(), status=[]),
                KeyValue=lambda **kwargs: types.SimpleNamespace(**kwargs)),
            'sensor_msgs': module('sensor_msgs'),
            'sensor_msgs.msg': module('sensor_msgs.msg', JointState=object),
            'std_msgs': module('std_msgs'),
            'std_msgs.msg': module('std_msgs.msg',
                                   Float64=lambda **kwargs: types.SimpleNamespace(**kwargs)),
            'std_srvs': module('std_srvs'),
            'std_srvs.srv': module('std_srvs.srv', Trigger=object),
        }
        with patch.dict(sys.modules, fake):
            sys.modules.pop('mech_assist_control.node', None)
            from mech_assist_control.node import MotorFollowNode
            default_node = MotorFollowNode()
            self.assertEqual(default_node.selected_mode, 'velocity_preview')
            self.assertEqual(set(default_node.publishers), {
                'knee_motor_follow/preview_velocity',
                'knee_motor_follow/diagnostics'})
            bench = MotorFollowNode(cli_args=[
                '--ros-args', '-p', 'output_mode:=motor',
                '-p', 'input_mode:=motor_feedback_simulation'])
            self.assertEqual(bench.selected_mode, 'periodic_bench')
            self.assertIn('/dm4340_velocity_controller/target_velocity',
                          bench.publishers)
            self.assertIn('knee_motor_follow/arm', bench.services)
            self.assertIn('knee_control/arm', bench.services)
            with self.assertRaises(ValueError):
                MotorFollowNode(cli_args=[
                    '--ros-args', '-p', 'control_strategy:=resistance',
                    '-p', 'output_mode:=motor'])
            node = MotorFollowNode(cli_args=['--ros-args', '-p',
                                             'control_strategy:=resistance'])
            self.assertFalse(node.parameter_callback([]).successful)
            self.assertEqual(set(node.publishers), {
                'knee_resistance/preview_effort_nm',
                'knee_resistance/diagnostics'})
            self.assertEqual(node.services, ['knee_control/arm',
                                             'knee_control/reset'])
            with patch('mech_assist_control.node.time.monotonic',
                       return_value=1.0):
                node._on_diagnostic(types.SimpleNamespace(
                    header=types.SimpleNamespace(stamp=types.SimpleNamespace(sec=1, nanosec=0)), status=[
                    types.SimpleNamespace(name='knee_kinematics',
                                          level=FakeStatus.OK)]))
                node._on_knee(types.SimpleNamespace(
                    name=['knee_joint'], position=[0.5], velocity=[0.5],
                    header=types.SimpleNamespace(stamp=types.SimpleNamespace(
                        sec=1, nanosec=0))))
                self.assertTrue(node._arm(None, types.SimpleNamespace()).success)
            with patch('mech_assist_control.node.time.monotonic',
                       return_value=1.02):
                node._tick()
            self.assertLess(node.command_pub.messages[-1].data, 0)
            with patch('mech_assist_control.node.time.monotonic',
                       return_value=1.03):
                node._on_diagnostic(types.SimpleNamespace(
                    header=types.SimpleNamespace(stamp=types.SimpleNamespace(sec=1, nanosec=0)), status=[]))
                node._tick()
            self.assertEqual(node.command_pub.messages[-1].data, 0)
            self.assertEqual(node.diagnostic_pub.messages[-1].status[0].level,
                             FakeStatus.ERROR)



if __name__ == '__main__':
    unittest.main()
