"""ROS adapters for the whole-robot supervisor; default preview, no device I/O."""
import json
import math
import time
from dataclasses import asdict

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String
from std_srvs.srv import Trigger
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

from .robot.contracts import JointLimit, JointObservation, RobotObservation
from .robot.policies import (make_policy, DampingPolicy, MotionAssistPolicy,
                             ResistanceTrainingPolicy, ResistancePermissionPolicy)
from .robot.supervisor import ExoskeletonSupervisor
from .robot.routing import CommandRouter, ControllerRoute


def level_number(level):
    return level[0] if isinstance(level, (bytes, bytearray)) else int(level)


class ExoskeletonNode(Node):
    def __init__(self, **kwargs):
        super().__init__('exoskeleton_control', **kwargs)
        defaults = {
            'policy': 'resistance', 'output_mode': 'preview',
            'resistance_execution': 'application',
            'input_source': 'estimator', 'feedback_topic': '/joint_states',
            'damping': 0.5, 'velocity_deadband': 0.02,
            'impedance_stiffness': 15.0, 'impedance_damping': 0.5,
            'training_damping': [1.0, 1.0], 'training_velocity_filter_tau_s': 0.05,
            'training_start_velocity_rad_s': 0.0,  # 0: use the rest deadband.
            'max_excursion_rad': 0.5, 'enforce_excursion_limit': True,
            'position_soft_margin_rad': 0.1,
            'assist_gain': 0.3, 'assist_deadband': 0.04,
            'assist_taper_speed': 0.3, 'assist_cutoff_speed': 0.5,
            'assist_work_budget_j': 0.05,
            'joint_names': ['knee_joint'], 'position_min': [-0.2],
            'position_max': [2.4], 'max_velocity': [2.0],
            'max_command': [0.3], 'max_slew': [0.5], 'run_limit_s': 10.0,
            'state_topic': 'knee/joint_state', 'health_topic': 'knee/diagnostics',
            'calibration_status_names': ['knee_kinematics'],
            'allow_motor_output': False, 'joint_mapping_verified': False,
            'controller_topics': ['/unconfigured/target_effort'],
            'actuator_joint_names': ['unconfigured'],
            'joint_scales': [0.0], 'joint_offsets': [0.0],
            'max_actuator_command': [0.3],
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        get = lambda name: self.get_parameter(name).value
        names = tuple(get('joint_names'))
        arrays = [get(k) for k in ('position_min', 'position_max', 'max_velocity',
                                   'max_command', 'max_slew')]
        if any(len(a) != len(names) for a in arrays):
            raise ValueError('each joint requires explicit limits')
        limits = [JointLimit(name, *(a[i] for a in arrays)) for i, name in enumerate(names)]
        self.input_source = get('input_source')
        if self.input_source not in ('estimator', 'motor_encoder'):
            raise ValueError('unsupported input_source')
        self.encoder_only = self.input_source == 'motor_encoder'
        execution = get('resistance_execution')
        if execution not in ('application', 'controller'):
            raise ValueError('invalid resistance_execution')
        self.controller_resistance = execution == 'controller'
        if self.controller_resistance and (not self.encoder_only or get('policy') != 'resistance_training'):
            raise ValueError('controller resistance requires motor_encoder/resistance_training')
        self.controller_impedance = get('policy') == 'impedance_training'
        if self.controller_impedance and not self.encoder_only:
            raise ValueError('impedance_training requires motor_encoder feedback and C++ controller')
        self.controller_permission = self.controller_resistance or self.controller_impedance
        self.training_reference = None
        self.impedance_stiffness = get('impedance_stiffness')
        self.impedance_damping = get('impedance_damping')
        if (not math.isfinite(self.impedance_stiffness) or self.impedance_stiffness <= 0 or
                not math.isfinite(self.impedance_damping) or self.impedance_damping < 0):
            raise ValueError('invalid impedance plan gains')
        self.training_caps = tuple(limit.max_command for limit in limits)
        if self.encoder_only and get('policy') not in ('resistance', 'resistance_training', 'impedance_training', 'motion_assist'):
            raise ValueError('encoder-only entry supports resistance, resistance_training, impedance_training or motion_assist')
        self.enforce_excursion_limit = get('enforce_excursion_limit')
        self.max_excursion = get('max_excursion_rad')
        if not math.isfinite(self.max_excursion) or self.max_excursion <= 0:
            raise ValueError('invalid excursion bound')
        policy = (DampingPolicy(get('damping'), get('velocity_deadband'))
                  if get('policy') == 'resistance' else make_policy(get('policy')))
        if get('policy') == 'resistance_training':
            gains = get('training_damping')
            if len(gains) != len(names):
                raise ValueError('one training_damping value required per joint')
            start_velocity = get('training_start_velocity_rad_s')
            if not math.isfinite(start_velocity) or start_velocity < 0:
                raise ValueError('invalid training start velocity')
            policy = ResistanceTrainingPolicy(gains, get('velocity_deadband'),
                                              get('training_velocity_filter_tau_s'),
                                              start_velocity or None)
            if self.controller_resistance:
                policy = ResistancePermissionPolicy()
        if get('policy') == 'motion_assist':
            if not self.encoder_only:
                raise ValueError('motion_assist requires explicit motor_encoder source')
            policy = MotionAssistPolicy(*(get(k) for k in (
                'assist_gain', 'assist_deadband', 'assist_taper_speed',
                'assist_cutoff_speed', 'assist_work_budget_j')))
        self.supervisor = ExoskeletonSupervisor(
            limits, policy, run_limit=get('run_limit_s'),
            sample_timeout=0.15 if self.encoder_only else 0.3,
            soft_margin=get('position_soft_margin_rad'),
            max_excursion=(self.max_excursion if self.encoder_only and
                           self.enforce_excursion_limit else None))
        mode = get('output_mode')
        if mode not in ('preview', 'motor'):
            raise ValueError('output_mode must be preview or motor')
        self.motor_mode = mode == 'motor'
        if self.motor_mode and not self.controller_impedance:
            raise ValueError('motor output requires the structured C++ impedance training backend; '
                             'other Python policies are offline references only')
        self.status_names = tuple(get('calibration_status_names'))
        if not self.status_names or len(set(self.status_names)) != len(self.status_names):
            raise ValueError('explicit calibration/health status producers required')
        self.health = None
        self.joints = None
        self.motor = None
        self.source_sequence = 0
        self.last_state_stamp = self.last_health_stamp = self.last_motor_stamp = 0
        self.was_valid = False
        self.command_publishers = {}
        self.router = None
        if self.motor_mode:
            if get('allow_motor_output') is not True or get('joint_mapping_verified') is not True:
                raise ValueError('motor output requires explicit output permission and measured mapping')
            topics, self.actuator_names = tuple(get('controller_topics')), tuple(get('actuator_joint_names'))
            self.scales, self.offsets = tuple(get('joint_scales')), tuple(get('joint_offsets'))
            max_output = tuple(get('max_actuator_command'))
            suffix = '/training_command'
            if any(not t.endswith(suffix) for t in topics):
                raise ValueError('controller training requires '+suffix+' topics')
            if (any(len(a) != len(names) for a in (topics, self.actuator_names,
                                                  self.scales, self.offsets, max_output)) or
                    len(set(self.actuator_names)) != len(names) or
                    any(not n or n == 'unconfigured' for n in self.actuator_names) or
                    any(not math.isfinite(v) for v in self.offsets)):
                raise ValueError('complete unique actuator mapping required')
            if self.encoder_only and (self.actuator_names != names or
                                      any(s != 1.0 for s in self.scales) or
                                      any(o != 0.0 for o in self.offsets)):
                raise ValueError('encoder-only coordinates require identity shaft mapping')
            self.router = CommandRouter(names, policy.mode, [
                ControllerRoute(n, policy.mode, t, scale, cap)
                for n, t, scale, cap in zip(names, topics, self.scales, max_output)])
            self.training_caps = tuple(min(limit.max_command, cap)
                                       for limit, cap in zip(limits, max_output))
            self.command_publishers = {t: self.create_publisher(Float64MultiArray, t, 1)
                                       for t in topics}
            if not self.encoder_only:
                self.create_subscription(JointState, get('feedback_topic'), self._motor, 1)
        self.preview = self.create_publisher(JointState, 'exoskeleton/preview_command', 1)
        self.envelope = self.create_publisher(String, 'exoskeleton/command_envelope', 1)
        self.diagnostics = self.create_publisher(DiagnosticArray, 'exoskeleton/diagnostics', 1)
        if self.encoder_only:
            self.create_subscription(JointState, get('feedback_topic'), self._encoder, 1)
        else:
            self.create_subscription(JointState, get('state_topic'), self._state, 1)
            self.create_subscription(DiagnosticArray, get('health_topic'), self._health, 1)
        for service, handler in (('arm', self._arm), ('disarm', self._disarm), ('fault_stop', self._fault_stop), ('reset', self._reset)):
            self.create_service(Trigger, 'exoskeleton/'+service, handler)
        self.create_timer(0.02, self._tick)
        self.add_on_set_parameters_callback(lambda _: SetParametersResult(
            successful=False, reason='restart to change policy, mapping or limits'))

    def _stamp(self, stamp, previous, limit):
        value = stamp.sec*1_000_000_000+stamp.nanosec
        age = (self.get_clock().now().nanoseconds-value)/1e9
        if value <= previous or not 0 <= age <= limit:
            raise ValueError('replayed, future or stale ROS source stamp')
        return value

    def _sampled_at(self, stamp):
        age = (self.get_clock().now().nanoseconds - stamp) / 1e9
        return time.monotonic() - age

    @staticmethod
    def _joint_values(msg, names):
        if len(set(msg.name)) != len(msg.name):
            raise ValueError('duplicate joint names')
        values = []
        for name in names:
            if name not in msg.name:
                raise ValueError('required joint missing')
            i = msg.name.index(name)
            if i >= len(msg.position) or i >= len(msg.velocity):
                raise ValueError('incomplete joint sample')
            p, v = msg.position[i], msg.velocity[i]
            if not math.isfinite(p) or not math.isfinite(v):
                raise ValueError('nonfinite joint sample')
            values.append(JointObservation(name, p, v))
        return tuple(values)

    def _encoder(self, msg):
        """Shaft coordinates only: no IMU, anatomical calibration or drive-health claim."""
        try:
            self.last_motor_stamp = self._stamp(msg.header.stamp, self.last_motor_stamp, 0.15)
            joints = self._joint_values(msg, self.supervisor.names)
            arrival = self._sampled_at(self.last_motor_stamp)
            self.motor = self.joints = (joints, arrival)
            # This source's readiness means finite, fresh encoder coordinates.
            # It does not assert the drive is enabled, fault-free or body-calibrated.
            self.health = (True, arrival)
            self._observe()
        except ValueError as error:
            self.supervisor.fault(str(error))

    def _state(self, msg):
        try:
            self.last_state_stamp = self._stamp(msg.header.stamp, self.last_state_stamp, 0.3)
            self.joints = (self._joint_values(msg, self.supervisor.names),
                           self._sampled_at(self.last_state_stamp))
            self._observe()
        except ValueError as error:
            self.supervisor.fault(str(error))

    def _health(self, msg):
        try:
            self.last_health_stamp = self._stamp(msg.header.stamp, self.last_health_stamp, 0.3)
            statuses = {s.name: s for s in msg.status}
            if len(statuses) != len(msg.status):
                raise ValueError('duplicate health producer')
            healthy = all(n in statuses and level_number(statuses[n].level) == 0
                          for n in self.status_names)
            self.health = (healthy, self._sampled_at(self.last_health_stamp))
            if self.supervisor.state == 'active' and not healthy:
                self.supervisor.fault('estimator/calibration diagnostic unhealthy')
            self._observe()
        except ValueError as error:
            self.supervisor.fault(str(error))

    def _observe(self):
        if self.joints is None or self.health is None:
            return
        joints, arrival = self.joints
        # Diagnostic updates cannot make an old joint sample look new.
        if self.supervisor.last_received_at == arrival:
            return
        self.source_sequence += 1
        healthy, health_arrival = self.health
        self.supervisor.observe(RobotObservation(
            self.source_sequence, arrival, joints, healthy, health_arrival,
            healthy and not self.encoder_only,
            coordinates='motor_shaft' if self.encoder_only else 'calibrated_joint'))

    def _motor(self, msg):
        try:
            self.last_motor_stamp = self._stamp(msg.header.stamp, self.last_motor_stamp, 0.15)
            self.motor = (self._joint_values(msg, self.actuator_names),
                          self._sampled_at(self.last_motor_stamp))
        except ValueError as error:
            self.supervisor.fault(str(error))

    def _motor_error(self, now):
        if self.router is None:
            return ''
        if self.motor is None or not 0 <= now-self.motor[1] <= 0.15 or self.joints is None:
            return 'actuator feedback unavailable or stale'
        for measured, estimated, scale, offset, limit in zip(
                self.motor[0], self.joints[0], self.scales, self.offsets, self.supervisor.limits):
            if (abs(measured.position*scale+offset-estimated.position) > 0.15 or
                    abs(measured.velocity*scale) > limit.max_velocity or
                    abs(measured.velocity*scale-estimated.velocity) > 0.5):
                return 'actuator mapping/feedback disagrees with estimator'
        if any(self.count_publishers(topic) != 1 for topic in self.command_publishers):
            return 'controller target publisher conflict'
        return ''

    def _arm(self, request, response):
        now = time.monotonic()
        error = ('estimator health unavailable or stale'
                 if self.health is None or not self.health[0] or
                 not 0 <= now-self.health[1] <= 0.3 else self._motor_error(now))
        if not error and self.controller_impedance and self.joints is not None:
            if any(abs(j.velocity) > .05 for j in self.joints[0]):
                error = 'fixed-position plan requires stationary feedback (<=0.05 rad/s)'
        response.success = not error and self.supervisor.arm(now)
        if response.success and self.controller_impedance:
            self.training_reference = tuple(j.position for j in self.joints[0])
        response.message = error or self.supervisor.reason or 'armed'
        return response

    def _publish_training_plan(self, enabled):
        if self.training_reference is None:
            return
        now = time.monotonic()
        sequence = float(time.monotonic_ns() // 1000)
        for (topic, publisher), reference, cap in zip(
                self.command_publishers.items(), self.training_reference, self.training_caps):
            # One coherent reference/parameter/permission frame, never Python PD.
            publisher.publish(Float64MultiArray(data=[1., sequence, now, float(enabled),
                reference, self.impedance_stiffness, self.impedance_damping, cap]))

    def _neutral_once(self):
        if self.was_valid:
            self._publish_training_plan(False)
        self.was_valid = False

    def _disarm(self, request, response):
        self.supervisor.disarm()
        self._neutral_once()
        response.success, response.message = True, 'command permission withdrawn; verify physical stop separately'
        return response

    def _fault_stop(self, request, response):
        self.supervisor.fault('operator latched stop')
        self._neutral_once()
        response.success = True
        response.message = 'software stop latched; physical disable not confirmed by this service'
        return response

    def _reset(self, request, response):
        response.success = self.supervisor.reset()
        if response.success:
            self._neutral_once()
            self.joints = self.health = self.motor = None
        response.message = 'fresh observations and explicit arm required' if response.success else 'disarm first'
        return response

    def _tick(self):
        now = time.monotonic()
        if self.supervisor.state == 'active':
            if self.health is None or not self.health[0] or not 0 <= now-self.health[1] <= 0.3:
                self.supervisor.fault('estimator health stale or unhealthy')
            error = self._motor_error(now)
            if error:
                self.supervisor.fault(error)
        envelope = self.supervisor.tick(now)
        if envelope.valid and self.router:
            try:
                self._publish_training_plan(True)
                self.was_valid = True
            except ValueError as error:
                self.supervisor.fault(str(error))
                envelope = self.supervisor.tick(now)
        if not envelope.valid:
            self._neutral_once()
        preview = JointState()
        preview.header.stamp = self.get_clock().now().to_msg()
        preview.name = list(envelope.names)
        setattr(preview, envelope.mode.value, list(envelope.values))
        self.preview.publish(preview)
        payload = asdict(envelope)
        payload['resistance_execution'] = 'controller' if self.controller_resistance else 'application'
        payload['values_are'] = 'unused_permission_placeholder' if self.controller_permission else 'offline_reference_target'
        payload['execution'] = 'controller' if self.controller_permission else 'offline_reference'
        payload['controller_type'] = ('mech_controllers/FixedPositionImpedanceController'
                                      if self.controller_impedance else
                                      'mech_controllers/ResistanceTrainingController'
                                      if self.controller_resistance else '')
        payload['reference_capture'] = 'upper_arm' if self.controller_impedance else ''
        if self.controller_impedance:
            payload['training_plan'] = dict(reference=self.training_reference,
                stiffness=self.impedance_stiffness, damping=self.impedance_damping,
                torque_caps=self.training_caps, command_schema=1)
        # Monotonic fields are local-host diagnostic metadata, NOT ROS stamps.
        self.envelope.publish(String(data=json.dumps(payload, allow_nan=False)))
        status = DiagnosticStatus(name='exoskeleton_control', hardware_id='application_supervisor')
        status.level = (DiagnosticStatus.OK if envelope.valid else DiagnosticStatus.ERROR
                        if self.supervisor.state == 'fault' else DiagnosticStatus.WARN)
        status.message = self.supervisor.reason or self.supervisor.state
        status.values = [KeyValue(key='state', value=self.supervisor.state),
                         KeyValue(key='policy', value=self.supervisor.policy.name),
                         KeyValue(key='input_source', value=self.input_source),
                         KeyValue(key='command_work_j', value=str(self.supervisor.command_work)),
                         KeyValue(key='coordinates', value='motor_shaft' if self.encoder_only else 'calibrated_joint'),
                         KeyValue(key='output_mode', value='motor' if self.motor_mode else 'preview')]
        status.values.append(KeyValue(key='execution', value='controller' if self.controller_permission else 'offline_reference'))
        status.values.append(KeyValue(key='resistance_execution',
                                      value='controller' if self.controller_resistance else 'application'))
        self.diagnostics.publish(DiagnosticArray(header=preview.header, status=[status]))

    def stop(self):
        self.supervisor.disarm()
        self._neutral_once()


def main(args=None):
    from rclpy.executors import ExternalShutdownException
    rclpy.init(args=args)
    node = None
    try:
        node = ExoskeletonNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.stop()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
