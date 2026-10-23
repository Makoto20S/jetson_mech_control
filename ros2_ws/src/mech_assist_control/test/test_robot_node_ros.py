"""Synthetic ROS adapters and services, motor topics confined to /test/robot/."""
import json
import time
import unittest

try:
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from sensor_msgs.msg import JointState
    from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
    from std_msgs.msg import Float64, Float64MultiArray, String
    from std_srvs.srv import Trigger
    from mech_assist_control.robot_node import ExoskeletonNode
except ImportError:
    rclpy = None


@unittest.skipIf(rclpy is None, 'ROS 2 unavailable')
class RobotRosTests(unittest.TestCase):
    def test_encoder_source_age_is_preserved_in_supervision(self):
        rclpy.init()
        app = None
        try:
            app = ExoskeletonNode(use_global_arguments=False, parameter_overrides=[
                Parameter('input_source', value='motor_encoder'),
                Parameter('joint_names', value=['test_motor'])])
            msg = JointState()
            stamp = app.get_clock().now().nanoseconds - 140_000_000
            msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(stamp, 1_000_000_000)
            msg.name, msg.position, msg.velocity = ['test_motor'], [0.], [0.]
            app._encoder(msg)
            self.assertLessEqual(app.supervisor.observation.received_at, time.monotonic() - .139)
            self.assertFalse(app.supervisor.arm(time.monotonic() + .02))
        finally:
            if app: app.destroy_node()
            rclpy.shutdown()

    def test_legacy_permission_path_cannot_emit_motor_commands(self):
        rclpy.init()
        try:
            with self.assertRaisesRegex(ValueError, 'structured C'):
                ExoskeletonNode(use_global_arguments=False, parameter_overrides=[
                    Parameter('input_source', value='motor_encoder'),
                    Parameter('policy', value='resistance_training'),
                    Parameter('resistance_execution', value='controller'),
                    Parameter('training_damping', value=[8.]),
                    Parameter('output_mode', value='motor')])
        finally:
            rclpy.shutdown()

    def test_impedance_training_permission_uses_supervisor_and_stops_on_stale_feedback(self):
        rclpy.init()
        app = probe = executor = None
        try:
            values = dict(input_source='motor_encoder', policy='impedance_training',
                resistance_execution='application', output_mode='motor', allow_motor_output=True,
                joint_mapping_verified=True, joint_names=['test_motor'], training_damping=[8.0],
                actuator_joint_names=['test_motor'], controller_topics=['/test/robot/training_command'],
                joint_scales=[1.0], joint_offsets=[0.0], max_actuator_command=[8.0],
                feedback_topic='/test/robot/fast_feedback')
            app = ExoskeletonNode(use_global_arguments=False, parameter_overrides=[
                Parameter(k, value=v) for k, v in values.items()])
            probe = Node('fast_resistance_probe')
            feedback = probe.create_publisher(JointState, '/test/robot/fast_feedback', 1)
            outputs = []
            probe.create_subscription(Float64MultiArray, '/test/robot/training_command', lambda m: outputs.append(list(m.data)), 10)
            send = [True]
            def publish():
                if not send[0]: return
                msg = JointState(); msg.header.stamp = probe.get_clock().now().to_msg()
                msg.name, msg.position, msg.velocity = ['test_motor'], [.5], [0.0]
                feedback.publish(msg)
            probe.create_timer(.01, publish)
            executor = SingleThreadedExecutor(); executor.add_node(app); executor.add_node(probe)
            def wait(predicate):
                end = time.monotonic()+2
                while time.monotonic() < end:
                    executor.spin_once(timeout_sec=.002)
                    if predicate(): return True
                return False
            self.assertTrue(wait(lambda: app.supervisor.observation is not None))
            self.assertFalse(outputs)
            self.assertTrue(app.controller_impedance)
            self.assertFalse(app.controller_resistance)
            self.assertEqual(app.supervisor.policy.name, 'impedance_training')
            self.assertEqual(app.supervisor.policy.evaluate(app.supervisor.observation, .02).values, (0.0,))
            self.assertTrue(app._arm(Trigger.Request(), Trigger.Response()).success)
            self.assertTrue(wait(lambda: len(outputs) >= 3))
            self.assertTrue(all(len(frame) == 8 and frame[0] == 1. and frame[3] == 1.
                                and frame[4:] == [.5, 15., .5, .3] for frame in outputs))
            self.assertTrue(all(a[1] < b[1] and a[2] <= b[2] for a, b in zip(outputs, outputs[1:])))
            # Feedback motion changes neither the fixed reference nor gains.
            self.assertEqual(app.training_reference, (.5,))
            self.assertEqual(app.count_publishers('/dm4310_motor1_effort_controller/target_effort'), 0)
            send[0] = False
            self.assertTrue(wait(lambda: app.supervisor.state == 'fault' and outputs[-1][3] == 0.0))
            count = len(outputs)
            for _ in range(20): executor.spin_once(timeout_sec=.005)
            self.assertEqual(len(outputs), count)  # No zero stream refreshing a failed lease.
        finally:
            if executor: executor.shutdown()
            if app: app.stop(); app.destroy_node()
            if probe: probe.destroy_node()
            rclpy.shutdown()

    def test_fast_permission_rejects_a_torque_topic(self):
        rclpy.init()
        try:
            with self.assertRaises(ValueError):
                ExoskeletonNode(use_global_arguments=False, parameter_overrides=[
                    Parameter(k, value=v) for k,v in dict(input_source='motor_encoder',
                        policy='resistance_training', resistance_execution='controller',
                        training_damping=[8.0], output_mode='motor', allow_motor_output=True,
                        joint_mapping_verified=True, controller_topics=['/test/target_effort']).items()])
        finally:
            rclpy.shutdown()

    def test_impedance_rejects_estimator_source_and_effort_topic(self):
        rclpy.init()
        try:
            for values in (
                dict(policy='impedance_training'),
                dict(policy='impedance_training', input_source='motor_encoder',
                     output_mode='motor', allow_motor_output=True, joint_mapping_verified=True,
                     controller_topics=['/test/robot/target_effort'])):
                with self.assertRaises(ValueError):
                    ExoskeletonNode(use_global_arguments=False, parameter_overrides=[
                        Parameter(k, value=v) for k, v in values.items()])
        finally:
            rclpy.shutdown()

    def run_case(self, motor, encoder=False, assist=False, training=False):
        rclpy.init()
        app = probe = executor = None
        try:
            values = {'state_topic': '/test/robot/state', 'health_topic': '/test/robot/health'}
            if motor:
                values.update(output_mode='motor', allow_motor_output=True,
                              joint_mapping_verified=True, controller_topics=['/test/robot/target'],
                              actuator_joint_names=['test_motor'], joint_scales=[1.0], joint_offsets=[0.0])
            if assist:
                values.update(policy='motion_assist')
            if training:
                values.update(policy='resistance_training', training_damping=[1.0])
            if encoder:
                values.update(input_source='motor_encoder', joint_names=['test_motor'])
            app = ExoskeletonNode(use_global_arguments=False, parameter_overrides=[
                Parameter(k, value=v) for k, v in values.items()])
            probe = Node('exoskeleton_synthetic_probe')
            state = probe.create_publisher(JointState, '/test/robot/state', 1)
            health = probe.create_publisher(DiagnosticArray, '/test/robot/health', 1)
            feedback = probe.create_publisher(JointState, '/joint_states', 1)
            outputs, envelopes = [], []
            probe.create_subscription(Float64, '/test/robot/target', lambda m: outputs.append(m.data), 10)
            probe.create_subscription(String, 'exoskeleton/command_envelope',
                                      lambda m: envelopes.append(json.loads(m.data)), 10)
            send = {'state': True, 'health': not encoder}

            def publish():
                stamp = probe.get_clock().now().to_msg()
                if send['state']:
                    m = JointState(); m.header.stamp = stamp
                    m.name, m.position, m.velocity = ['knee_joint'], [.5], [.2]
                    state.publish(m)
                    f = JointState(); f.header.stamp = stamp
                    f.name, f.position, f.velocity = ['test_motor'], [.5], [.2]
                    feedback.publish(f)
                if send['health']:
                    d = DiagnosticArray(); d.header.stamp = stamp
                    d.status = [DiagnosticStatus(name='knee_kinematics', level=DiagnosticStatus.OK)]
                    health.publish(d)
            probe.create_timer(.02, publish)
            executor = SingleThreadedExecutor(); executor.add_node(app); executor.add_node(probe)

            def wait(predicate, timeout=2):
                end = time.monotonic()+timeout
                while time.monotonic() < end:
                    executor.spin_once(timeout_sec=.005)
                    if predicate(): return True
                return False

            def service(name):
                client = probe.create_client(Trigger, 'exoskeleton/'+name)
                self.assertTrue(client.wait_for_service(timeout_sec=1))
                future = client.call_async(Trigger.Request())
                self.assertTrue(wait(future.done))
                return future.result()

            self.assertTrue(wait(lambda: app.supervisor.observation is not None and
                                 not app.supervisor._health_error(time.monotonic()) and
                                 (not motor or not app._motor_error(time.monotonic()))))
            self.assertEqual(app.supervisor.state, 'standby')
            self.assertFalse(any(e['valid'] for e in envelopes))
            self.assertTrue(service('arm').success)
            self.assertFalse(service('reset').success)  # No live reset/rearm shortcut.
            self.assertTrue(wait(lambda: any(e['valid'] and (e['values'][0] > .01 if assist else e['values'][0] < -.01) for e in envelopes)))
            if motor:
                self.assertTrue(wait(lambda: len(outputs) >= 3))
                self.assertTrue(all(0 <= v <= .3 if assist else -.3 <= v <= 0 for v in outputs))
                # A second process/node claiming the output must withdraw permission.
                conflict = probe.create_publisher(Float64, '/test/robot/target', 1)
                self.assertTrue(wait(lambda: app.supervisor.state == 'fault'))
                self.assertTrue(wait(lambda: outputs[-1] == 0))
                probe.destroy_publisher(conflict)
                count = len(outputs)
                self.assertFalse(wait(lambda: len(outputs) > count, .15))
            else:
                self.assertEqual(app.count_publishers('/test/robot/target'), 0)
                self.assertEqual(app.count_publishers('/dm4340_effort_controller/target_effort'), 0)
                send['health' if not encoder else 'state'] = False
                self.assertTrue(wait(lambda: app.supervisor.state == 'fault'))
            self.assertFalse(service('arm').success)
            send['state'] = True
            send['health'] = not encoder
            self.assertTrue(service('reset').success)
            self.assertTrue(wait(lambda: app.supervisor.observation is not None and
                                 not app.supervisor._health_error(time.monotonic())))
            self.assertTrue(service('arm').success)
            self.assertTrue(service('disarm').success)
            self.assertEqual(app.supervisor.state, 'standby')
            self.assertTrue(service('fault_stop').success)
            self.assertEqual(app.supervisor.state, 'fault')
            self.assertFalse(service('arm').success)
            self.assertFalse(app.set_parameters([Parameter('policy', value='transparent')])[0].successful)
        finally:
            if executor: executor.shutdown()
            if app: app.stop(); app.destroy_node()
            if probe: probe.destroy_node()
            rclpy.shutdown()

    def test_preview_stale_health_and_explicit_recovery(self):
        self.run_case(False)


    def test_encoder_preview_without_imu_or_calibration_topic(self):
        self.run_case(False, encoder=True)


    def test_encoder_excursion_replay_nonfinite_and_deadline(self):
        rclpy.init()
        app = None
        try:
            app = ExoskeletonNode(use_global_arguments=False, parameter_overrides=[
                Parameter('input_source', value='motor_encoder'),
                Parameter('joint_names', value=['test_motor']),
                Parameter('position_min', value=[-3.0]),
                Parameter('position_max', value=[3.0]),
            ])

            def sample(position=0.0, velocity=.2):
                m = JointState()
                m.header.stamp = app.get_clock().now().to_msg()
                m.name, m.position, m.velocity = ['test_motor'], [position], [velocity]
                app._encoder(m)
                return m

            def arm():
                return app._arm(Trigger.Request(), Trigger.Response()).success

            sample()
            self.assertTrue(arm())
            sample(.6)
            self.assertEqual(app.supervisor.state, 'fault')
            self.assertIn('excursion', app.supervisor.reason)
            app._reset(Trigger.Request(), Trigger.Response())
            m = sample()
            self.assertTrue(arm())
            app._encoder(m)
            self.assertEqual(app.supervisor.state, 'fault')
            app._reset(Trigger.Request(), Trigger.Response())
            sample(velocity=float('nan'))
            self.assertFalse(arm())
            app._reset(Trigger.Request(), Trigger.Response())
            sample()
            self.assertTrue(arm())
            now = time.monotonic()
            for step in range(1, 10):
                e = app.supervisor.tick(now+step*.02)
            self.assertFalse(e.valid)
            self.assertEqual(app.supervisor.state, 'fault')
            self.assertEqual(app.count_publishers('/dm4340_effort_controller/target_effort'), 0)
        finally:
            if app:
                app.stop(); app.destroy_node()
            rclpy.shutdown()

    def test_encoder_assist_preview_and_stale_stop(self):
        self.run_case(False, encoder=True, assist=True)

    def test_application_feedback_laws_cannot_drive_motor_topics(self):
        rclpy.init()
        try:
            for policy in ('transparent', 'resistance', 'hold_velocity', 'motion_assist', 'resistance_training'):
                with self.subTest(policy=policy), self.assertRaisesRegex(ValueError, r'C\+\+ impedance training backend'):
                    ExoskeletonNode(use_global_arguments=False, parameter_overrides=[
                        Parameter('policy', value=policy),
                        Parameter('input_source', value='motor_encoder' if policy not in ('hold_velocity', 'transparent') else 'estimator'),
                        Parameter('training_damping', value=[1.]),
                        Parameter('output_mode', value='motor')])
        finally:
            rclpy.shutdown()

    def test_dual_training_preview_configuration(self):
        import yaml
        from pathlib import Path
        parameters = yaml.safe_load((Path(__file__).resolve().parents[1] /
            'config/dual_motor_resistance_training_preview.yaml').read_text())['exoskeleton_control']['ros__parameters']
        rclpy.init()
        app = None
        try:
            app = ExoskeletonNode(use_global_arguments=False, parameter_overrides=[
                Parameter(k, value=v) for k, v in parameters.items()])
            self.assertFalse(app.motor_mode)
            self.assertEqual(app.supervisor.policy.name, 'resistance_training')
            self.assertEqual(app.supervisor.policy.gains, (1.0, 1.0))
            self.assertEqual(app.supervisor.state, 'standby')
            self.assertFalse(app.command_publishers)
        finally:
            if app: app.stop(); app.destroy_node()
            rclpy.shutdown()

    def test_training_disables_relative_limit_but_retains_absolute_limits(self):
        from mech_assist_control.robot.contracts import JointObservation
        rclpy.init()
        app = None
        try:
            values = dict(input_source='motor_encoder', policy='resistance_training',
                          joint_names=['axis'], training_damping=[4.0],
                          position_min=[-1.0], position_max=[1.0],
                          max_velocity=[1.0], max_command=[2.0], max_slew=[0.5],
                          enforce_excursion_limit=False, max_excursion_rad=0.2)
            app = ExoskeletonNode(use_global_arguments=False, parameter_overrides=[
                Parameter(k, value=v) for k, v in values.items()])
            guard = app.supervisor.position_guard
            self.assertIsNone(guard.excursion)
            guard.arm((JointObservation('axis',0.0,0.0),))
            self.assertEqual(guard.error((JointObservation('axis',0.4,0.0),)), '')
            self.assertTrue(guard.error((JointObservation('axis',1.01,0.0),)))
            self.assertEqual(app.supervisor.policy.gains, (4.0,))
            self.assertFalse(app.motor_mode)
        finally:
            if app: app.stop(); app.destroy_node()
            rclpy.shutdown()
