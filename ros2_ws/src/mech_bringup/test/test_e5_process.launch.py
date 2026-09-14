import math
import os
import pathlib
import subprocess
import time
import unittest

import launch
import launch.actions
import launch_testing.asserts
import launch_ros.actions
import launch_testing.actions
import rclpy
from controller_manager_msgs.srv import ListControllers, SwitchController
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, UInt64MultiArray
from std_srvs.srv import SetBool, Trigger


CONTROLLER = "motor1_position_controller"
MANAGER = "/controller_manager"
ROOT = pathlib.Path(__file__).resolve().parents[4]
ACTIVATE = ROOT / "tools" / "bench" / "activate_position_controller.sh"


def generate_test_description():
    source = pathlib.Path(__file__).resolve().parent
    manager_env = {}
    asan_runtime = os.environ.get("MECH_ASAN_RUNTIME")
    if asan_runtime:
        manager_env["LD_PRELOAD"] = asan_runtime
        manager_env["ASAN_OPTIONS"] = (
            os.environ.get("ASAN_OPTIONS", "") +
            ":new_delete_type_mismatch=0").lstrip(":")
    robot_description = (source / "e5_process.urdf").read_text(encoding="utf-8")
    manager = launch_ros.actions.Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[{"robot_description": robot_description},
                    str(source / "e5_process_controllers.yaml")],
        additional_env=manager_env,
        output="screen",
    )
    controller_spawner = launch_ros.actions.Node(
        package="controller_manager",
        executable="spawner",
        arguments=[CONTROLLER, "--inactive", "--controller-manager", MANAGER],
        output="screen",
    )
    broadcaster_spawner = launch_ros.actions.Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", MANAGER],
        output="screen",
    )
    spawner = launch.actions.TimerAction(
        period=1.0,
        actions=[broadcaster_spawner, controller_spawner],
    )
    return launch.LaunchDescription([
        manager,
        spawner,
        launch_testing.actions.ReadyToTest(),
    ]), {"manager_process": manager,
         "controller_spawner": controller_spawner,
         "broadcaster_spawner": broadcaster_spawner}


class E5ProcessTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node(f"e5_process_probe_{os.getpid()}")
        cls.list_client = cls.node.create_client(
            ListControllers, f"{MANAGER}/list_controllers")
        cls.switch_client = cls.node.create_client(
            SwitchController, f"{MANAGER}/switch_controller")
        cls.feedback_client = cls.node.create_client(
            SetBool, "/e5_test_system/set_feedback")
        cls.reset_client = cls.node.create_client(
            Trigger, "/e5_test_system/reset_trace")
        cls.publisher = cls.node.create_publisher(
            Float64, f"/{CONTROLLER}/target_position", 10)
        cls.trace_count = None
        cls.trace_id = None
        cls.trace_payload = None
        cls.trace_samples = 0
        cls.trace_reset = 0
        cls.observed_at_ns = 0
        cls.read_cycles = 0
        cls.error_reason = 0
        cls.error_quality = 0
        cls.joint_position = None
        cls.trace_subscription = cls.node.create_subscription(
            UInt64MultiArray, "/e5_test_system/trace", cls._trace, 10)
        cls.joint_subscription = cls.node.create_subscription(
            JointState, "/joint_states", cls._joint_state, 10)

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    @classmethod
    def _trace(cls, message):
        if len(message.data) == 15:
            cls.trace_count = message.data[0]
            cls.trace_id = message.data[1]
            cls.trace_payload = list(message.data[2:10])
            cls.trace_samples += 1
            cls.trace_reset = message.data[10]
            cls.observed_at_ns = message.data[11]
            cls.read_cycles = message.data[12]
            cls.error_reason = message.data[13]
            cls.error_quality = message.data[14]

    @classmethod
    def _joint_state(cls, message):
        if "motor1_joint" in message.name:
            cls.joint_position = message.position[message.name.index("motor1_joint")]

    def spin_until(self, predicate, timeout=5.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if predicate():
                return True
            rclpy.spin_once(self.node, timeout_sec=0.02)
        return predicate()

    def call(self, client, request, timeout=5.0):
        self.assertTrue(client.wait_for_service(timeout_sec=timeout))
        future = client.call_async(request)
        self.assertTrue(self.spin_until(future.done, timeout))
        return future.result()

    def spin_for(self, duration):
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.02)

    def state(self):
        response = self.call(self.list_client, ListControllers.Request())
        for controller in response.controller:
            if controller.name == CONTROLLER:
                return controller.state
        return None

    def wait_inactive(self):
        self.assertTrue(self.list_client.wait_for_service(timeout_sec=12.0))
        self.assertTrue(self.spin_until(lambda: self.state() == "inactive", 8.0))

    def set_feedback(self, enabled):
        request = SetBool.Request()
        request.data = enabled
        self.assertTrue(self.call(self.feedback_client, request).success)

    def reset_trace(self):
        previous_reset = self.trace_reset
        self.assertTrue(self.call(self.reset_client, Trigger.Request()).success)
        self.assertTrue(self.spin_until(
            lambda: self.trace_reset > previous_reset and
            self.trace_count == 0, 2.0))

    @staticmethod
    def decoded_canonical_position(payload):
        raw = (payload[3] << 8) | payload[4]
        device_position = -12.56 + (raw / 65535.0) * 25.12
        return device_position - 5.760604931781636

    def activate(self, timeout):
        return subprocess.run(
            ["bash", str(ACTIVATE), CONTROLLER, str(timeout), MANAGER],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            timeout=timeout + 3,
            env=os.environ.copy(),
            check=False,
        )

    def publish(self, value):
        message = Float64()
        message.data = value
        self.publisher.publish(message)
        rclpy.spin_once(self.node, timeout_sec=0.02)

    def publish_at_20_hz(self, value, duration):
        deadline = time.monotonic() + duration
        next_publish = time.monotonic()
        samples = 0
        while next_publish < deadline:
            wait = next_publish - time.monotonic()
            if wait > 0:
                time.sleep(wait)
            self.publish(value)
            samples += 1
            next_publish += 0.05
        return samples

    def deactivate(self):
        request = SwitchController.Request()
        request.deactivate_controllers = [CONTROLLER]
        request.strictness = SwitchController.Request.STRICT
        request.activate_asap = True
        self.assertTrue(self.call(self.switch_client, request).ok)
        self.assertEqual(self.state(), "inactive")

    def test_process_chain(self, proc_info, controller_spawner,
                           broadcaster_spawner):
        self.wait_inactive()
        proc_info.assertWaitForShutdown(controller_spawner, timeout=5)
        proc_info.assertWaitForShutdown(broadcaster_spawner, timeout=5)
        if os.environ.get("E5_SCENARIO") == "feedback_loss":
            self.run_feedback_loss()
        else:
            self.run_lifetime_and_restart()

    def run_lifetime_and_restart(self):
        refused = self.activate(1)
        self.assertEqual(refused.returncode, 4, refused.stdout + refused.stderr)
        self.assertEqual(self.state(), "inactive")

        self.set_feedback(True)
        expected_seed = math.pi / 2.0 - 5.760604931781636
        self.assertTrue(self.spin_until(
            lambda: self.joint_position is not None and
            math.isfinite(self.joint_position) and
            abs(self.joint_position - expected_seed) < 0.01, 2.0))
        seed = self.joint_position
        activated = self.activate(4)
        self.assertEqual(activated.returncode, 0,
                         activated.stdout + activated.stderr)
        self.assertEqual(self.state(), "active")
        self.reset_trace()
        time.sleep(0.03)
        rclpy.spin_once(self.node, timeout_sec=0.02)
        self.assertEqual(self.trace_count, 0)

        self.publish(0.5)
        self.assertTrue(self.spin_until(lambda: (self.trace_count or 0) > 0, 2.0))
        self.assertEqual(self.trace_id, 0x0868)
        commanded = self.decoded_canonical_position(self.trace_payload)
        self.assertGreater(commanded, seed)
        self.assertLess(commanded, 0.5)
        self.assertLess(commanded - seed, 0.05)
        # Sustain the slowest production cadence across several complete target
        # lifetimes. Same-value messages are refreshes and must keep both the
        # controller and the independent 4/6 ms hardware lease alive.
        before_sustained = self.trace_count
        self.assertGreaterEqual(self.publish_at_20_hz(0.5, 0.25), 5)
        self.assertTrue(self.spin_until(
            lambda: self.trace_count > before_sustained, 1.0))
        self.assertEqual(self.error_reason, 0)
        self.assertEqual(self.state(), "active")

        self.deactivate()
        self.reset_trace()
        self.publish(-0.5)
        activated = self.activate(4)
        self.assertEqual(activated.returncode, 0,
                         activated.stdout + activated.stderr)
        time.sleep(0.03)
        rclpy.spin_once(self.node, timeout_sec=0.02)
        self.assertEqual(self.trace_count, 0)
        self.publish(0.25)
        self.assertTrue(self.spin_until(lambda: self.trace_count > 0, 2.0))

        self.spin_for(0.2)
        self.assertEqual(self.error_reason, 1, "expected command-lease expiry")
        expired_count = self.trace_count
        expired_samples = self.trace_samples
        self.assertTrue(self.spin_until(
            lambda: self.trace_samples >= expired_samples + 2, 0.5))
        self.assertEqual(self.trace_count, expired_count)

    def run_feedback_loss(self):
        self.set_feedback(True)
        activated = self.activate(4)
        self.assertEqual(activated.returncode, 0,
                         activated.stdout + activated.stderr)
        self.assertEqual(self.state(), "active")
        self.assertTrue(self.spin_until(
            lambda: self.publisher.get_subscription_count() > 0, 2.0))
        self.assertGreaterEqual(self.publish_at_20_hz(0.25, 0.30), 6)
        self.assertGreater(self.trace_count or 0, 0)
        self.assertEqual(self.error_reason, 0)
        self.assertEqual(self.state(), "active")
        self.set_feedback(False)
        self.assertGreaterEqual(self.publish_at_20_hz(0.25, 0.12), 3)
        self.spin_for(0.1)
        self.assertEqual(self.error_reason, 5, "expected unusable feedback")
        self.assertEqual(self.error_quality, 3, "expected Stale sample")
        stopped_count = self.trace_count
        stopped_samples = self.trace_samples
        self.assertTrue(self.spin_until(
            lambda: self.trace_samples >= stopped_samples + 2, 0.5))
        self.assertEqual(self.trace_count, stopped_count)


@launch_testing.post_shutdown_test()
class E5ProcessExitTest(unittest.TestCase):
    def test_processes_exit_cleanly(self, proc_info, manager_process,
                                    controller_spawner,
                                    broadcaster_spawner):
        launch_testing.asserts.assertExitCodes(
            proc_info, allowable_exit_codes=[0, 130], process=manager_process)
        launch_testing.asserts.assertExitCodes(
            proc_info, process=controller_spawner)
        launch_testing.asserts.assertExitCodes(
            proc_info, process=broadcaster_spawner)
