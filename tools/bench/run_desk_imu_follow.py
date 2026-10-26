#!/usr/bin/env python3
"""One supervised desktop-IMU -> DM4340 velocity trial; manager starts inactive."""
import argparse
import sys
import time
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from std_msgs.msg import Float64
from std_srvs.srv import Trigger
from run_dm4340_ros_motion import MotionTrial, State, TARGET_TOPIC


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument('--preflight', action='store_true')
    group.add_argument('--execute', action='store_true')
    parser.add_argument('--operator-supervised', action='store_true')
    parser.add_argument('--countdown-seconds', type=int, default=0,
                        help='supervised zero-target countdown before arm (0..15 seconds)')
    args = parser.parse_args()
    if not 0 <= args.countdown_seconds <= 15:
        parser.error('countdown must be 0..15 seconds')
    if args.execute and not args.operator_supervised:
        parser.error('execution requires current on-site authorization and --operator-supervised')
    rclpy.init()
    trial = MotionTrial(publish_targets=False)
    app = None
    activated = False
    result = 1
    try:
        if trial.hardware_state() != State.PRIMARY_STATE_INACTIVE or trial.controller_state() != 'inactive':
            raise RuntimeError('hardware and velocity controller must both start inactive')
        if args.preflight:
            print('PASS inactive preflight; no activation, target publication or arming')
            return 0
        from mech_assist_control.desk_node import DeskFollowNode
        app = DeskFollowNode(parameter_overrides=[Parameter('output_mode', value='motor')])
        trial._trial_executor = SingleThreadedExecutor()
        trial._trial_executor.add_node(trial)
        trial._trial_executor.add_node(app)
        trial.create_subscription(Float64, TARGET_TOPIC, trial.on_target, 10)
        activated = True  # Cleanup even if service response fails after activation.
        trial.set_hardware(State.PRIMARY_STATE_ACTIVE)
        start = trial.sample()[0]
        if not -11.9 <= start <= 11.9:
            raise RuntimeError('insufficient protocol position range for 0.6 rad envelope')
        trial.run_phase(0.5, start, 0.1, max_speed=0.1)
        if not app.logic.available(time.monotonic()):
            raise RuntimeError('fresh real IMUs, healthy sensor diagnostic and motor feedback required')
        trial.switch_controller(True)
        trial.run_phase(0.5, start, 0.1, max_speed=0.1)
        for remaining in range(args.countdown_seconds, 0, -1):
            print(f'静止准备：{remaining} 秒；倒计时后才转动 IMU', flush=True)
            trial.run_phase(1.0, start, 0.1, max_speed=0.1)
        client = trial.create_client(Trigger, '/desk_imu_follow/arm')
        armed = trial.call(client, Trigger.Request())
        if not armed.success:
            raise RuntimeError('desk arm refused: '+armed.message)
        print('ARMED: gently rotate desktop IMU about standard Y within +/-30 deg; 4 s', flush=True)
        def check():
            if app.logic.state != 'armed':
                raise RuntimeError(app.logic.reason)
        trial.run_phase(4.0, start, 0.6, check, max_speed=1.0)
        app.logic.state, app.logic.target, app.was_armed = 'complete', 0.0, False
        # Planned stop uses the same single publisher; faults never enter this path.
        app.create_timer(0.01, lambda: app.output.publish(Float64(data=0.0)))
        app.output.publish(Float64(data=0.0))
        trial.run_phase(0.7, start, 0.6, max_speed=1.0)
        end, speed, _ = trial.sample()
        if abs(speed) > 0.1:
            raise RuntimeError('zero-target stop not confirmed; cut motor power')
        if max(abs(trial.min_target), abs(trial.max_target)) < 0.02:
            raise RuntimeError('no meaningful IMU velocity target observed')
        if trial.peak_speed < 0.02 and abs(end-start) < 0.005:
            raise RuntimeError('commands observed but shaft motion not detected; do not increase gains automatically')
        print(f'Desk trial completed: target=[{trial.min_target:.3f}, {trial.max_target:.3f}] rad/s, '
              f'displacement={end-start:.4f} rad, peak_speed={trial.peak_speed:.4f}, end_speed={speed:.4f}')
        result = 0
    except (RuntimeError, KeyboardInterrupt) as error:
        print(f'Trial failed/interrupted: {error}', file=sys.stderr)
    finally:
        print(f'Observed targets=[{trial.min_target:.4f}, {trial.max_target:.4f}] rad/s; '
              f'peak speed={trial.peak_speed:.4f} rad/s; '
              f'position range=[{trial.min_position}, {trial.max_position}]', flush=True)
        if app is not None and app.logic.state == 'armed':
            app.logic.fault('runner stopping')
            app.output.publish(Float64(data=0.0))
        if activated:
            try:
                if trial.controller_state() == 'active':
                    trial.switch_controller(False)
                trial.set_hardware(State.PRIMARY_STATE_INACTIVE)
                print('Hardware/controller inactive lifecycle cleanup confirmed; independently verify motor status0')
            except RuntimeError as error:
                print(f'DISABLE UNCONFIRMED: {error}; cut motor power', file=sys.stderr)
                result = 1
        if trial._trial_executor is not None:
            trial._trial_executor.shutdown()
        if app is not None:
            app.destroy_node()
        trial.destroy_node()
        rclpy.shutdown()
    return result


if __name__ == '__main__':
    raise SystemExit(main())
