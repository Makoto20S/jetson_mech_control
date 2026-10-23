#!/usr/bin/env python3
"""Upper plan -> installed C++ controllers -> GenericSystem; no device backend."""
import argparse
import json
from pathlib import Path
import signal
import subprocess
import time

from ament_index_python.packages import get_package_prefix
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState
from control_msgs.msg import DynamicJointState
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import Trigger
import yaml

from mech_assist_control.robot_node import ExoskeletonNode


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir', type=Path, required=True,
                        help='New external evidence directory; must not exist')
    args = parser.parse_args()
    root = args.output_dir.resolve()
    root.mkdir(parents=True, exist_ok=False)
    ns = '/training_architecture_shadow'
    names = ['joint_1', 'joint_2']
    controllers = ['joint_1_training', 'joint_2_training']
    joints = ''.join(f'''<joint name="{name}">
      <command_interface name="effort"/><command_interface name="command_generation"/>
      <state_interface name="position"><param name="initial_value">0</param></state_interface>
      <state_interface name="velocity"><param name="initial_value">0</param></state_interface>
      <state_interface name="effort"><param name="initial_value">0</param></state_interface>
      <state_interface name="command_generation"><param name="initial_value">0</param></state_interface>
      </joint>''' for name in names)
    # This executable never reads a real URDF or accepts a hardware plugin option.
    links = '<link name="base"/>'+''.join(
        f'<link name="link_{i}"/><joint name="{name}" type="revolute">'
        f'<parent link="base"/><child link="link_{i}"/><axis xyz="0 0 1"/>'
        '<limit effort="7" lower="-1" upper="1" velocity="1"/></joint>'
        for i, name in enumerate(names))
    xml = ('<robot name="mock_only">'+links+'<ros2_control name="shadow_only" type="system">'
           '<hardware><plugin>mock_components/GenericSystem</plugin></hardware>'+joints+
           '</ros2_control></robot>')
    params = {ns+'/controller_manager': {'ros__parameters': {
        'update_rate': 500, 'joint_state_broadcaster': {
            'type': 'joint_state_broadcaster/JointStateBroadcaster'},
        **{c: {'type': 'mech_controllers/FixedPositionImpedanceController'} for c in controllers}}}}
    for name, controller in zip(names, controllers):
        params[ns+'/'+controller] = {'ros__parameters': dict(joint=name,
            command_source='training_command', minimum=0., maximum=1.,
            max_slew_per_second=1000000., stiffness_nm_per_rad=15., damping=.5,
            torque_limit_nm=7., position_min_rad=-1., position_max_rad=1., run_limit_s=5.)}
    path = root/'parameters.yaml'
    path.write_text(yaml.safe_dump(params))
    manager = app = probe = executor = None
    log = (root/'manager.log').open('w')
    initialized = False
    try:
        manager = subprocess.Popen([str(Path(get_package_prefix('controller_manager')) /
            'lib/controller_manager/ros2_control_node'), '--ros-args', '-r', '__ns:='+ns,
            '--params-file', str(path), '-p', 'robot_description:='+xml],
            stdout=log, stderr=subprocess.STDOUT)
        for name in ('joint_state_broadcaster', *controllers):
            subprocess.run(['ros2', 'run', 'controller_manager', 'spawner', name,
                '--controller-manager', ns+'/controller_manager'], check=True,
                timeout=20, capture_output=True)
        rclpy.init()
        initialized = True
        values = dict(policy='impedance_training', input_source='motor_encoder',
            output_mode='motor', allow_motor_output=True, joint_mapping_verified=True,
            joint_names=names, actuator_joint_names=names, position_min=[-1., -1.],
            position_max=[1., 1.], max_velocity=[1., 1.], max_command=[2., 2.],
            max_slew=[1000., 1000.], max_actuator_command=[2., 2.],
            joint_scales=[1., 1.], joint_offsets=[0., 0.], enforce_excursion_limit=False,
            controller_topics=[ns+'/'+c+'/training_command' for c in controllers],
            feedback_topic=ns+'/relayed_joint_states', run_limit_s=5.,
            impedance_stiffness=10., impedance_damping=.25)
        app = ExoskeletonNode(namespace=ns, use_global_arguments=False,
            parameter_overrides=[Parameter(k, value=v) for k, v in values.items()])
        probe = Node('training_graph_probe', namespace=ns)
        relay = probe.create_publisher(JointState, 'relayed_joint_states', 1)
        forward = [True]
        probe.create_subscription(JointState, 'joint_states',
            lambda msg: relay.publish(msg) if forward[0] else None, 1)
        latest = {}
        frames = {c: [] for c in controllers}
        def states(msg):
            for name, interfaces in zip(msg.joint_names, msg.interface_values):
                if 'command_generation' in interfaces.interface_names:
                    latest[name] = interfaces.values[interfaces.interface_names.index('command_generation')]
        probe.create_subscription(DynamicJointState, 'dynamic_joint_states', states, 10)
        for controller in controllers:
            probe.create_subscription(Float64MultiArray, controller+'/training_command',
                lambda msg, c=controller: frames[c].append(list(msg.data)), 10)
        executor = SingleThreadedExecutor()
        executor.add_node(app)
        executor.add_node(probe)
        def wait(predicate, seconds=2.):
            end = time.monotonic()+seconds
            while time.monotonic() < end:
                executor.spin_once(timeout_sec=.002)
                if predicate():
                    return True
            return False
        assert wait(lambda: app.supervisor.observation is not None and len(latest) == 2)
        assert not any(frames.values()), 'activation must not authorize output'
        initial = dict(latest)
        response = app._arm(Trigger.Request(), Trigger.Response())
        assert response.success, response.message
        assert wait(lambda: all(latest[n]-initial[n] > 100 for n in names)), (initial, latest)
        following = dict(latest)
        for messages in frames.values():
            assert messages and all(f[3:] == [1., 0., 10., .25, 2.] for f in messages)
        forward[0] = False  # Loss of upstream feedback, not a synthetic command stop.
        assert wait(lambda: app.supervisor.state == 'fault' and
            all(messages[-1][3] == 0. for messages in frames.values())), app.supervisor.reason
        counts = {c: len(messages) for c, messages in frames.items()}
        wait(lambda: False, .3)
        stopped = dict(latest)
        wait(lambda: False, .12)
        assert latest == stopped, (stopped, latest)
        assert all(len(frames[c]) == counts[c] for c in controllers), 'zero stream renewed lease'
        result = dict(mock_only=True, axes=2, actual_cpp_plugin=True,
            initial_generation=initial, following_generation=following,
            stopped_generation=stopped, stale_feedback_fault=app.supervisor.reason,
            upper_sends_reference_and_parameters=True, no_python_pd=True,
            command_refresh_stopped=True, nonzero_pd_covered_by_cpp_unit_tests=True)
        (root/'summary.json').write_text(json.dumps(result, indent=2))
        print(json.dumps(result, indent=2))
    finally:
        if app:
            app.stop()
        if executor:
            executor.shutdown()
        if app:
            app.destroy_node()
        if probe:
            probe.destroy_node()
        if initialized and rclpy.ok():
            rclpy.shutdown()
        if manager:
            manager.send_signal(signal.SIGINT)
            try:
                manager.wait(timeout=5)
            except subprocess.TimeoutExpired:
                manager.terminate()
                manager.wait(timeout=5)
        log.close()


if __name__ == '__main__':
    main()
