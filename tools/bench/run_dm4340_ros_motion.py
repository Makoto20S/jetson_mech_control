#!/usr/bin/env python3
"""One bounded DM4340 ros2_control shaft-motion trial on an already launched manager."""

import argparse
from dataclasses import dataclass
import math
import sys
import time

import rclpy
from controller_manager_msgs.srv import (ListControllers, ListHardwareComponents,
                                         SetHardwareComponentState, SwitchController)
from lifecycle_msgs.msg import State
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.executors import SingleThreadedExecutor
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from std_srvs.srv import Trigger


HARDWARE = 'dm4340_system'
CONTROLLER = 'dm4340_velocity_controller'
EFFORT_CONTROLLER = 'dm4340_effort_controller'
JOINT = 'dm4340_joint'
MANAGER = '/controller_manager'
TARGET_TOPIC = f'/{CONTROLLER}/target_velocity'
EFFORT_TARGET_TOPIC = f'/{EFFORT_CONTROLLER}/target_effort'


@dataclass(frozen=True)
class TrialProfile:
    target_speed: float
    drive_seconds: float
    settle_seconds: float
    max_displacement: float
    measure_after_seconds: float = 0.0
    supervised: bool = False


PROFILES = {
    'short': TrialProfile(1.5, 0.5, 0.8, 0.35),
    'extended': TrialProfile(1.5, 0.8, 1.0, 0.8),
    'five_second': TrialProfile(1.5, 5.0, 2.0, 9.0, supervised=True),
    'feedforward_comparison': TrialProfile(0.8, 2.0, 1.0, 2.2, 1.0),
    'feedforward_five_second': TrialProfile(0.8, 5.0, 2.0, 5.0, 1.0, True),
}


def selected_profile(args):
    return next((PROFILES[name] for name in PROFILES if getattr(args, name, False)),
                PROFILES['short'])


class MotionTrial(Node):
    def __init__(self, publish_targets=True, observe_targets=False, effort_mode=False):
        super().__init__('dm4340_ros_motion_trial')
        self.controller = EFFORT_CONTROLLER if effort_mode else CONTROLLER
        self.target_topic = EFFORT_TARGET_TOPIC if effort_mode else TARGET_TOPIC
        self.target = 0.0
        self.feedback = None
        self.feedback_effort = None
        self.effort_samples = []
        self.record_effort = False
        self.peak_speed = 0.0
        self.measure_from = None
        self.steady_velocities = []
        self.min_position = None
        self.max_position = None
        self.min_target = 0.0
        self.max_target = 0.0
        self._trial_executor = None
        self.publisher = None
        if publish_targets or observe_targets:
            self.publisher = self.create_publisher(Float64, self.target_topic, 10)
        if publish_targets:
            self.create_timer(0.01, self.publish_target)
        self.create_subscription(JointState, '/joint_states', self.on_feedback, 10)
        if observe_targets:
            self.create_subscription(Float64, self.target_topic, self.on_target, 10)
        self.hardware_client = self.create_client(
            SetHardwareComponentState, f'{MANAGER}/set_hardware_component_state')
        self.hardware_list_client = self.create_client(
            ListHardwareComponents, f'{MANAGER}/list_hardware_components')
        self.controller_list_client = self.create_client(
            ListControllers, f'{MANAGER}/list_controllers')
        self.switch_client = self.create_client(
            SwitchController, f'{MANAGER}/switch_controller')
        self.arm_client = self.create_client(Trigger, '/knee_motor_follow/arm')

    def spin_once(self, timeout=0.01):
        if self._trial_executor is None:
            rclpy.spin_once(self, timeout_sec=timeout)
        else:
            self._trial_executor.spin_once(timeout_sec=timeout)

    def on_target(self, message):
        self.min_target = min(self.min_target, message.data)
        self.max_target = max(self.max_target, message.data)

    def publish_target(self):
        self.publisher.publish(Float64(data=self.target))

    def on_feedback(self, message):
        if JOINT not in message.name:
            return
        index = message.name.index(JOINT)
        if index >= len(message.position) or index >= len(message.velocity):
            return
        position = message.position[index]
        velocity = message.velocity[index]
        if math.isfinite(position) and math.isfinite(velocity):
            self.feedback = (position, velocity, time.monotonic())
            if index < len(message.effort) and math.isfinite(message.effort[index]):
                self.feedback_effort = (message.effort[index], time.monotonic())
                if self.record_effort:
                    self.effort_samples.append(message.effort[index])
            self.peak_speed = max(self.peak_speed, abs(velocity))
            self.min_position = (position if self.min_position is None else
                                 min(self.min_position, position))
            self.max_position = (position if self.max_position is None else
                                 max(self.max_position, position))
            if self.measure_from is not None and time.monotonic() >= self.measure_from:
                self.steady_velocities.append(velocity)

    def call(self, client, request, timeout=5.0):
        deadline = time.monotonic() + timeout
        while not client.wait_for_service(timeout_sec=0.05):
            if time.monotonic() >= deadline:
                raise RuntimeError(f'ROS service unavailable: {client.srv_name}')
        future = client.call_async(request)
        while not future.done() and time.monotonic() < deadline:
            self.spin_once()
        if not future.done() or future.result() is None:
            raise RuntimeError(f'ROS service timed out: {client.srv_name}')
        return future.result()

    def hardware_state(self):
        response = self.call(self.hardware_list_client, ListHardwareComponents.Request())
        return next((item.state.id for item in response.component
                     if item.name == HARDWARE), None)

    def controller_state(self):
        response = self.call(self.controller_list_client, ListControllers.Request())
        return next((item.state for item in response.controller
                     if item.name == self.controller), None)

    def set_hardware(self, state_id):
        request = SetHardwareComponentState.Request()
        request.name = HARDWARE
        request.target_state.id = state_id
        response = self.call(self.hardware_client, request, timeout=3.0)
        if not response.ok or response.state.id != state_id:
            raise RuntimeError(f'hardware transition failed: {response.state.label}')
        if self.hardware_state() != state_id:
            raise RuntimeError('hardware state readback mismatch')

    def switch_controller(self, activate):
        request = SwitchController.Request()
        request.activate_controllers = [self.controller] if activate else []
        request.deactivate_controllers = [] if activate else [self.controller]
        request.strictness = SwitchController.Request.STRICT
        request.activate_asap = True
        request.timeout.sec = 2
        response = self.call(self.switch_client, request, timeout=3.0)
        expected = 'active' if activate else 'inactive'
        if not response.ok or self.controller_state() != expected:
            raise RuntimeError(f'controller did not reach {expected}')

    def sample(self, timeout=1.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.spin_once()
            if self.feedback and time.monotonic() - self.feedback[2] < 0.1:
                return self.feedback
        raise RuntimeError('fresh joint feedback unavailable')

    def run_phase(self, seconds, start_position, max_displacement=0.35,
                  health_check=None, max_speed=2.0):
        deadline = time.monotonic() + seconds
        next_state_check = time.monotonic()
        while time.monotonic() < deadline:
            self.spin_once()
            if health_check is not None:
                health_check()
            if time.monotonic() >= next_state_check:
                if self.hardware_state() != State.PRIMARY_STATE_ACTIVE:
                    raise RuntimeError('hardware left active state')
                next_state_check = time.monotonic() + 0.05
            position, velocity, stamp = self.sample(timeout=0.1)
            if time.monotonic() - stamp > 0.1 or abs(position - start_position) > max_displacement:
                raise RuntimeError(
                    f'feedback stale or displacement exceeded {max_displacement:.2f} rad')
            if abs(velocity) > max_speed:
                raise RuntimeError(f'feedback speed exceeded {max_speed:.2f} rad/s')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    action = parser.add_mutually_exclusive_group(required=True)
    action.add_argument('--execute', action='store_true',
                        help='run one bounded hardware trial')
    action.add_argument('--preflight', action='store_true',
                        help='read ROS hardware/controller state without activation or motion')
    parser.add_argument('--diagnose-only', action='store_true',
                        help='observe the active controller with a zero target only')
    profile = parser.add_mutually_exclusive_group()
    profile.add_argument('--extended', action='store_true',
                         help='one 0.8 s drive with a 0.8 rad displacement limit')
    profile.add_argument('--five-second', action='store_true',
                         help='one 5 s drive with a 9 rad displacement limit')
    profile.add_argument('--feedforward-comparison', action='store_true',
                         help='one 2 s ROS drive at 0.8 rad/s with steady feedback sampling')
    profile.add_argument('--feedforward-five-second', action='store_true',
                         help='one 5 s ROS drive at 0.8 rad/s with a 5 rad displacement limit')
    profile.add_argument('--follower-simulation', action='store_true',
                         help='one 4 s periodic ROS follower trial using motor feedback as virtual knee')
    profile.add_argument('--follower-extended', action='store_true',
                         help='two 4 s periodic cycles at 20 deg using motor feedback as virtual knee')
    profile.add_argument('--follower-40deg', action='store_true',
                         help='three 4 s periodic cycles at 40 deg on the unloaded bench')
    profile.add_argument('--effort-pulse', action='store_true',
                         help='one 0.08 Nm, 0.5 s ROS effort pulse on an unloaded motor')
    parser.add_argument('--effort-controller', action='store_true',
                        help='select effort controller for read-only --preflight')
    parser.add_argument('--operator-supervised', action='store_true',
                        help='confirm an operator is present with immediate motor power cutoff')
    parser.add_argument('--reverse', action='store_true',
                        help='drive toward decreasing motor position with the same speed magnitude')
    args = parser.parse_args()
    if args.preflight and (args.diagnose_only or args.reverse or args.operator_supervised or
                           args.follower_simulation or args.follower_extended or
                           args.follower_40deg or args.effort_pulse or
                           any(getattr(args, name) for name in PROFILES if name != 'short')):
        parser.error('--preflight cannot be combined with motion options')
    if args.effort_controller and not args.preflight:
        parser.error('--effort-controller is only valid with --preflight')
    if args.execute and args.effort_pulse and not args.operator_supervised:
        parser.error('--effort-pulse requires --operator-supervised')
    if args.effort_pulse and args.reverse:
        parser.error('--effort-pulse cannot use --reverse')
    profile = selected_profile(args)
    if args.execute and profile.supervised and not args.operator_supervised:
        parser.error('five-second trials require --operator-supervised')
    follower_mode = (args.follower_simulation or args.follower_extended or
                     args.follower_40deg)
    if follower_mode and (args.diagnose_only or args.reverse):
        parser.error('periodic follower trials cannot use --diagnose-only or --reverse')
    if args.execute and follower_mode and not args.operator_supervised:
        parser.error('periodic follower trials require --operator-supervised')

    rclpy.init()
    trial = MotionTrial(publish_targets=not (args.preflight or follower_mode),
                        observe_targets=follower_mode,
                        effort_mode=args.effort_pulse or args.effort_controller)
    if args.preflight:
        try:
            hardware = trial.hardware_state()
            controller = trial.controller_state()
            print(f'ROS preflight: hardware={hardware} controller={controller}; '
                  'no activation or motor command sent')
            return 0 if hardware == State.PRIMARY_STATE_INACTIVE and controller == 'inactive' else 1
        except RuntimeError as error:
            print(f'ROS preflight failed: {error}', file=sys.stderr)
            return 1
        finally:
            trial.destroy_node()
            rclpy.shutdown()
    activated_hardware = False
    start_position = None
    follower = None
    zero_hold_started = False
    result = 1
    try:
        if trial.hardware_state() != State.PRIMARY_STATE_INACTIVE:
            raise RuntimeError('expected inactive DM4340 hardware')
        if trial.controller_state() != 'inactive':
            raise RuntimeError(f'expected inactive {trial.controller}')
        trial.set_hardware(State.PRIMARY_STATE_ACTIVE)
        activated_hardware = True
        start_position = trial.sample()[0]
        trial.min_position = start_position
        trial.max_position = start_position
        max_displacement = (0.75 if args.follower_40deg else 0.48 if follower_mode
                            else profile.max_displacement)
        if not (-12.5 <= start_position - max_displacement and
                start_position + max_displacement <= 12.5):
            raise RuntimeError('insufficient feedback range for trial displacement limit')
        if follower_mode:
            from mech_assist_control.node import MotorFollowNode
            follower = MotorFollowNode(parameter_overrides=[
                Parameter('output_mode', value='motor'),
                Parameter('input_mode', value='motor_feedback_simulation'),
                Parameter('amplitude_deg', value=(40.0 if args.follower_40deg else
                                                  20.0 if args.follower_extended else 15.0)),
                Parameter('motor_run_limit_s', value=(13.0 if args.follower_40deg else
                                                       9.0 if args.follower_extended else 5.0)),
                Parameter('simulated_travel_limit_rad', value=max_displacement),
            ])
            trial._trial_executor = SingleThreadedExecutor()
            trial._trial_executor.add_node(trial)
            trial._trial_executor.add_node(follower)
        trial.run_phase(2.0 if args.diagnose_only else 0.5, start_position)
        trial.switch_controller(True)
        trial.run_phase(2.0 if args.diagnose_only else 0.5, start_position)
        if args.diagnose_only:
            trial.switch_controller(False)
            trial.set_hardware(State.PRIMARY_STATE_INACTIVE)
            activated_hardware = False
            print('ROS DM4340 zero-target controller observation passed')
            return 0
        if args.effort_pulse:
            if (trial.feedback_effort is None or
                    time.monotonic() - trial.feedback_effort[1] > 0.1):
                raise RuntimeError('fresh effort feedback unavailable')
            baseline = trial.feedback_effort[0]
            trial.effort_samples.clear()
            trial.record_effort = True
            trial.target = 0.08
            trial.run_phase(0.5, start_position, 0.15, max_speed=0.5)
            trial.target = 0.0
            trial.record_effort = False
            trial.run_phase(0.5, start_position, 0.15, max_speed=0.5)
            end_position, end_velocity, _ = trial.sample()
            if not trial.effort_samples:
                raise RuntimeError('no effort feedback during pulse')
            print(f'ROS DM4340 effort pulse: target=0.08 Nm, '
                  f'baseline={baseline:.4f} Nm, '
                  f'feedback=[{min(trial.effort_samples):.4f}, '
                  f'{max(trial.effort_samples):.4f}] Nm, '
                  f'delta_position={end_position - start_position:.4f} rad, '
                  f'end_velocity={end_velocity:.4f} rad/s')
            result = 0
        elif follower_mode:
            armed = trial.call(trial.arm_client, Trigger.Request())
            if not armed.success:
                raise RuntimeError(
                    f'periodic follower did not arm: {armed.message}; '
                    f'knee={follower.control.knee} motor={follower.control.motor} '
                    f'zero={follower.sim_motor_zero} '
                    f'diagnostic={follower.control.diagnostic_ok} '
                    f'state={follower.control.state}')

            def check_follower():
                if follower.control.state != 'armed':
                    raise RuntimeError(f'periodic follower stopped: {follower.control.reason}')

            trial.run_phase(12.0 if args.follower_40deg else
                            8.0 if args.follower_extended else 4.0,
                            start_position, max_displacement, check_follower)
            follower.command_pub.publish(Float64(data=0.0))
            follower.control.reset()
            follower._was_armed = False
            trial.create_timer(0.01, trial.publish_target)
            zero_hold_started = True
            trial.run_phase(1.0, start_position, max_displacement)
            end_position, end_velocity, _ = trial.sample()
            span = trial.max_position - trial.min_position
            print(f'ROS periodic follower: start={start_position:.4f} rad, '
                  f'end={end_position:.4f} rad, span={span:.4f} rad, '
                  f'position=[{trial.min_position:.4f}, {trial.max_position:.4f}] rad, '
                  f'target=[{trial.min_target:.4f}, {trial.max_target:.4f}] rad/s, '
                  f'peak_speed={trial.peak_speed:.4f} rad/s, '
                  f'end_velocity={end_velocity:.4f} rad/s')
            if (trial.min_target >= -0.01 or trial.max_target <= 0.01 or
                    span < 0.02 or abs(end_velocity) > 0.05):
                raise RuntimeError('periodic target, shaft motion or final rest unconfirmed')
            result = 0
        else:
            trial.target = -profile.target_speed if args.reverse else profile.target_speed
            trial.measure_from = (time.monotonic() + profile.measure_after_seconds
                                  if profile.measure_after_seconds else None)
            trial.run_phase(profile.drive_seconds, start_position,
                            profile.max_displacement)
            trial.target = 0.0
            trial.measure_from = None
            trial.run_phase(profile.settle_seconds, start_position,
                            profile.max_displacement)
            end_position, end_velocity, _ = trial.sample()
            displacement = end_position - start_position
            print(f'ROS DM4340 trial: start={start_position:.4f} rad, '
                  f'end={end_position:.4f} rad, delta={displacement:.4f} rad, '
                  f'end_velocity={end_velocity:.4f} rad/s, '
                  f'peak_speed={trial.peak_speed:.4f} rad/s')
            if profile.measure_after_seconds:
                if not trial.steady_velocities:
                    raise RuntimeError('no steady-state feedback samples')
                print(f'ROS steady velocity (after first second): '
                      f'{sum(trial.steady_velocities) / len(trial.steady_velocities):.4f} '
                      f'rad/s over {len(trial.steady_velocities)} samples')
            progress = -displacement if args.reverse else displacement
            if not 0.005 <= progress <= profile.max_displacement or abs(end_velocity) > 0.05:
                raise RuntimeError('motion or final rest not confirmed')
            result = 0
    except (RuntimeError, KeyboardInterrupt) as error:
        print(f'ROS DM4340 trial failed: {error}', file=sys.stderr)
        if follower is not None:
            print(f'Follower state={follower.control.state} '
                  f'reason={follower.control.reason} '
                  f'target=[{trial.min_target:.4f}, {trial.max_target:.4f}] rad/s',
                  file=sys.stderr)
    finally:
        trial.target = 0.0
        if follower is not None:
            follower.command_pub.publish(Float64(data=0.0))
            follower.control.reset()
            follower._was_armed = False
            if not zero_hold_started:
                trial.create_timer(0.01, trial.publish_target)
                for _ in range(12):
                    trial.spin_once()
        try:
            if (args.effort_pulse and start_position is not None and
                    trial.controller_state() == 'active'):
                trial.run_phase(0.2, start_position, 0.15, max_speed=0.5)
        except RuntimeError as error:
            print(f'zero-effort hold unconfirmed: {error}; cut motor power',
                  file=sys.stderr)
            result = 1
        try:
            if trial.controller_state() == 'active':
                trial.switch_controller(False)
        except RuntimeError as error:
            print(f'controller cleanup failed: {error}', file=sys.stderr)
            result = 1
        if activated_hardware:
            try:
                trial.set_hardware(State.PRIMARY_STATE_INACTIVE)
            except RuntimeError as error:
                print(f'hardware disable unconfirmed: {error}; cut motor power',
                      file=sys.stderr)
                result = 1
        if trial._trial_executor is not None:
            trial._trial_executor.remove_node(trial)
            trial._trial_executor.remove_node(follower)
            trial._trial_executor.shutdown()
        if follower is not None:
            follower.destroy_node()
        trial.destroy_node()
        rclpy.shutdown()
    return result


if __name__ == '__main__':
    raise SystemExit(main())
