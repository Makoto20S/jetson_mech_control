"""Check the shadow policy and its ROS publication without actuator hardware."""

import math
import sys
import types
import unittest
from unittest.mock import patch


class Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class FakeNode:
    def __init__(self, name):
        self.name = name
        self.parameters = {}
        self.publishers = {}

    def declare_parameter(self, name, value):
        self.parameters[name] = value

    def get_parameter(self, name):
        return types.SimpleNamespace(value=self.parameters[name])

    def create_publisher(self, kind, topic, depth):
        del kind, depth
        publisher = Publisher()
        self.publishers[topic] = publisher
        return publisher

    def create_subscription(self, *args):
        return None

    def create_service(self, *args):
        return None

    def create_timer(self, *args):
        return None

    def get_clock(self):
        return types.SimpleNamespace(now=lambda: types.SimpleNamespace(
            to_msg=lambda: types.SimpleNamespace()))


class Status:
    OK = 0
    WARN = 1
    ERROR = 2

    def __init__(self):
        self.name = ''
        self.level = self.WARN
        self.values = []


def module(name, **symbols):
    result = types.ModuleType(name)
    vars(result).update(symbols)
    return result


def load_shadow():
    fake = {
        'rclpy': module('rclpy'),
        'rclpy.node': module('rclpy.node', Node=FakeNode),
        'diagnostic_msgs': module('diagnostic_msgs'),
        'diagnostic_msgs.msg': module(
            'diagnostic_msgs.msg', DiagnosticStatus=Status,
            DiagnosticArray=lambda: types.SimpleNamespace(
                header=types.SimpleNamespace(), status=[]),
            KeyValue=lambda **kwargs: types.SimpleNamespace(**kwargs)),
        'sensor_msgs': module('sensor_msgs'),
        'sensor_msgs.msg': module('sensor_msgs.msg', JointState=object),
        'std_msgs': module('std_msgs'),
        'std_msgs.msg': module('std_msgs.msg',
                               Float64=lambda: types.SimpleNamespace(data=0.0)),
        'std_srvs': module('std_srvs'),
        'std_srvs.srv': module('std_srvs.srv', Trigger=object),
    }
    with patch.dict(sys.modules, fake):
        sys.modules.pop('mech_knee_kinematics.shadow_assist', None)
        from mech_knee_kinematics.shadow_assist import ShadowAssist, ShadowAssistNode
        return ShadowAssist, ShadowAssistNode


class ShadowAssistTests(unittest.TestCase):
    def test_arm_requires_fresh_diagnostic_and_sample(self):
        ShadowAssist, _ = load_shadow()
        control = ShadowAssist()
        self.assertFalse(control.arm(1.0))
        control.update_diagnostic(True, 1.0)
        self.assertFalse(control.arm(1.0))
        control.update_knee(20.0, 0.0, 1.0)
        self.assertTrue(control.arm(1.01))
        self.assertEqual(control.tick(1.01), 0.0)
        control.update_knee(20.0, 0.0, 1.11)
        self.assertGreater(control.tick(1.11), 0.0)
        self.assertLessEqual(control.effort_nm, 0.35)

    def test_bad_diagnostic_and_timeout_latch_zero(self):
        ShadowAssist, _ = load_shadow()
        control = ShadowAssist()
        control.update_diagnostic(True, 1.0)
        control.update_knee(20.0, 0.0, 1.0)
        self.assertTrue(control.arm(1.01))
        control.update_knee(20.0, 0.0, 1.1)
        control.update_diagnostic(False, 1.11)
        self.assertEqual(control.tick(1.12), 0.0)
        self.assertEqual(control.state, 'fault')
        control.update_diagnostic(True, 1.2)
        control.update_knee(20.0, 0.0, 1.2)
        self.assertFalse(control.arm(1.2))
        control.reset()
        control.update_diagnostic(True, 2.0)
        control.update_knee(20.0, 0.0, 2.0)
        self.assertTrue(control.arm(2.01))
        self.assertEqual(control.tick(2.31), 0.0)
        self.assertEqual(control.state, 'fault')

    def test_invalid_knee_faults_and_parameter_validation(self):
        ShadowAssist, _ = load_shadow()
        with self.assertRaises(ValueError):
            ShadowAssist(target_deg=math.nan)
        with self.assertRaises(ValueError):
            ShadowAssist(timeout_s=0.0)
        control = ShadowAssist()
        control.update_diagnostic(True, 1.0)
        control.update_knee(20.0, 0.0, 1.0)
        self.assertTrue(control.arm(1.01))
        control.update_knee(math.nan, 0.0, 1.1)
        self.assertEqual(control.state, 'fault')
        self.assertEqual(control.tick(1.11), 0.0)

    def test_ros_node_only_publishes_virtual_output(self):
        _, Node = load_shadow()
        node = Node()
        self.assertEqual(set(node.publishers), {
            'assist_shadow/virtual_effort_nm', 'assist_shadow/diagnostics'})
        with patch('mech_knee_kinematics.shadow_assist.time.monotonic',
                   return_value=1.0):
            node._on_diagnostic(types.SimpleNamespace(status=[
                types.SimpleNamespace(name='knee_kinematics', level=Status.OK)]))
            node._on_knee(types.SimpleNamespace(
                name=['knee_joint'], position=[math.radians(20.0)], velocity=[0.0]))
        with patch('mech_knee_kinematics.shadow_assist.time.monotonic',
                   return_value=1.01):
            response = node._arm(None, types.SimpleNamespace())
        self.assertTrue(response.success)
        with patch('mech_knee_kinematics.shadow_assist.time.monotonic',
                   return_value=1.11):
            node._on_knee(types.SimpleNamespace(
                name=['knee_joint'], position=[math.radians(20.0)], velocity=[0.0]))
            node._tick()
        self.assertGreater(node.effort_pub.messages[-1].data, 0.0)
        with patch('mech_knee_kinematics.shadow_assist.time.monotonic',
                   return_value=1.12):
            node._on_diagnostic(types.SimpleNamespace(status=[
                types.SimpleNamespace(name='knee_kinematics', level=Status.WARN)]))
            node._tick()
        self.assertEqual(node.effort_pub.messages[-1].data, 0.0)
        self.assertEqual(node.diagnostic_pub.messages[-1].status[0].level, Status.ERROR)


if __name__ == '__main__':
    unittest.main()
