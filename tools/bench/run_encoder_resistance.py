#!/usr/bin/env python3
"""One supervised encoder-only resistance trial on an already inactive manager."""
import argparse
import json
import time

import rclpy
from lifecycle_msgs.msg import State
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from std_srvs.srv import Trigger
from mech_assist_control.robot_node import ExoskeletonNode
from run_dm4340_ros_motion import MotionTrial


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute', action='store_true', required=True)
    parser.add_argument('--operator-supervised', action='store_true', required=True)
    args = parser.parse_args()
    rclpy.init()
    executor = SingleThreadedExecutor()
    trial = MotionTrial(publish_targets=False, effort_mode=True)
    app = None
    activated = controller = False
    result = 1
    samples = []
    try:
        if trial.hardware_state() != State.PRIMARY_STATE_INACTIVE or trial.controller_state() != 'inactive':
            raise RuntimeError('require inactive hardware and effort controller')
        app = ExoskeletonNode(use_global_arguments=False, parameter_overrides=[
            Parameter(k, value=v) for k, v in {
                'input_source': 'motor_encoder', 'policy': 'resistance',
                'output_mode': 'motor', 'allow_motor_output': True,
                'joint_mapping_verified': True, 'joint_names': ['dm4340_joint'],
                'actuator_joint_names': ['dm4340_joint'],
                'controller_topics': ['/dm4340_effort_controller/target_effort'],
                'joint_scales': [1.0], 'joint_offsets': [0.0],
                'position_min': [-12.5], 'position_max': [12.5],
                'max_velocity': [1.0], 'max_command': [0.2],
                'max_actuator_command': [0.2], 'max_slew': [0.5],
                'damping': 0.5, 'velocity_deadband': 0.02,
                'max_excursion_rad': 0.5, 'run_limit_s': 5.0,
            }.items()])
        executor.add_node(app); executor.add_node(trial)
        trial._trial_executor = executor
        activated = True  # Cleanup even if transition response is lost.
        trial.set_hardware(State.PRIMARY_STATE_ACTIVE)
        initial = trial.sample()
        if abs(initial[1]) > .05:
            raise RuntimeError('shaft must start stationary')
        controller = True
        trial.switch_controller(True)
        deadline = time.monotonic()+2
        while time.monotonic() < deadline and app.supervisor.observation is None:
            trial.spin_once()
        response = app._arm(Trigger.Request(), Trigger.Response())
        if not response.success:
            raise RuntimeError(response.message)
        print('ARMED: 5 seconds, gently move fixture within 0.5 rad; no automatic retry', flush=True)
        deadline = time.monotonic()+5
        while time.monotonic() < deadline:
            trial.spin_once(.005)
            if app.supervisor.state != 'active':
                if app.supervisor.reason == 'run limit reached': break
                raise RuntimeError('application stopped: '+app.supervisor.reason)
            feedback = trial.sample(.1)
            if abs(feedback[0]-initial[0]) > .5 or abs(feedback[1]) > 1:
                raise RuntimeError('independent shaft envelope exceeded')
            samples.append((feedback[0], feedback[1], app.supervisor.previous[0]))
        app.stop()
        trial.switch_controller(False); controller = False
        trial.set_hardware(State.PRIMARY_STATE_INACTIVE); activated = False
        meaningful = any(abs(v) > .04 and abs(t) > .005 for _, v, t in samples)
        print(json.dumps({'samples': len(samples), 'meaningful_resistance_command': meaningful,
                          'position_range': [min(p for p,_,_ in samples),max(p for p,_,_ in samples)] if samples else [],
                          'peak_speed': max((abs(v) for _,v,_ in samples), default=0),
                          'peak_command_nm': max((abs(t) for _,_,t in samples), default=0),
                          'lifecycle_inactive': True,
                          'physical_disabled_requires_independent_query': True}), flush=True)
        result = 0 if meaningful else 1
    except Exception as error:
        print('FAIL: '+str(error), flush=True)
    finally:
        if app: app.stop()
        try:
            if controller: trial.switch_controller(False)
            if activated: trial.set_hardware(State.PRIMARY_STATE_INACTIVE)
        except Exception as error:
            print('STOP UNCONFIRMED: operator disconnect power: '+str(error), flush=True)
            result = 2
        executor.shutdown()
        if app: app.destroy_node()
        trial.destroy_node()
        rclpy.shutdown()
    return result


if __name__ == '__main__':
    raise SystemExit(main())
