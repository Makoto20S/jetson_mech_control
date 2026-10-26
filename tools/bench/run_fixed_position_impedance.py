#!/usr/bin/env python3
"""Dual DM4310 fixed-anchor impedance, explicit operator start and group cleanup.

--continuous removes only the duration stop. It does not relax position,
feedback, command/permission TTL or update-deadline protection. --shadow uses
ONLY GenericSystem and never invokes the supplied query tool.
"""
import argparse
import importlib.util
import json
import math
import os
from pathlib import Path
import re
import signal
import subprocess
import sys
import tempfile
import time
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.signals import SignalHandlerOptions
from lifecycle_msgs.msg import State
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
import xacro
import yaml

from controller_log_guard import ControllerLogGuard
import run_dm4340_ros_motion as motion

NAMES = [f'dm4310_motor{i}_joint' for i in (1, 2)]
CONTROLLERS = [f'dm4310_motor{i}_effort_controller' for i in (1, 2)]


def read_devices(tool, destination, mapping, firmware):
    """Known IDs only; audited external CC/33 read-only probe, no retry."""
    with destination.open('w') as log:
        result = subprocess.run([str(tool)], stdout=log, stderr=subprocess.STDOUT, timeout=15)
    text = destination.read_text()
    if result.returncode or 'PARAM_SUMMARY matched=18 expected=18' not in text:
        raise RuntimeError('independent device query failed: '+str(destination))
    for axis in (1, 2):
        found = re.findall(rf'MATCH motor={axis} feedback_can={axis+3} status=(\d+) position=([-\d.]+) velocity=([-\d.]+)', text)
        if len(found) != 1 or found[0][0] != '0':
            raise RuntimeError(f'axis{axis}: disabled state not independently confirmed')
        q, v = map(float, found[0][1:])
        if not all(math.isfinite(x) for x in (q, v)):
            raise RuntimeError('nonfinite query feedback')
        # Final disabled feedback need not be stationary: an operator may still
        # be moving the fixture. Only initial acquisition captures an anchor.
        if destination.name.startswith('initial'):
            lo, hi = (float(mapping[f'motor{axis}_position_{side}_rad']) for side in ('min', 'max'))
            if not lo+.02 <= q <= hi-.02 or abs(v) > .05:
                raise RuntimeError(f'axis{axis}: initial anchor must be stationary inside bounds')
        expected = {10: 1, 13: 0x56303033, 14: firmware, 20: 0x41200000,
                    21: 0x41480000, 22: 0x42480000, 23: 0x41200000}
        for rid, raw in expected.items():
            if f'PARAM motor={axis} rid={rid} raw=0x{raw:08X}' not in text:
                raise RuntimeError(f'axis{axis} RID{rid}: configured device identity/range differs')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--continuous', action='store_true', required=True)
    parser.add_argument('--operator-supervised', action='store_true', required=True,
                        help='Current off-body supported fixture, no person attached, unchanged verified CAN/mapping/reference, immediate power cutoff')
    parser.add_argument('--query-tool', type=Path, required=True,
                        help='Audited known-ID disabled-status and 18-parameter read-only executable')
    parser.add_argument('--firmware-id', type=int, required=True)
    parser.add_argument('--range-file', type=Path,
                        default=Path(os.environ.get('XDG_CONFIG_HOME', Path.home()/'.config'))/'jetson_mech_control/dm4310_limits.json')
    parser.add_argument('--shadow', action='store_true')
    parser.add_argument('--output-dir', type=Path, required=True, help='New external result directory')
    parser.add_argument('--shadow-duration-s', type=float, default=0., help='Mock-only timed exit; 0 waits for Ctrl+C')
    args = parser.parse_args()
    if (not math.isfinite(args.shadow_duration_s) or args.shadow_duration_s < 0 or
            (args.shadow_duration_s != 0 and not args.shadow)):
        parser.error('a timed exit here is mock-only')
    root = args.output_dir.resolve(); root.mkdir(parents=True, exist_ok=False)
    share = Path(get_package_share_directory('mech_bringup'))
    spec = importlib.util.spec_from_file_location('fixed_dm4310_launch', share/'launch/dm4310_dual_bringup.launch.py')
    launch = importlib.util.module_from_spec(spec); spec.loader.exec_module(launch)
    mapping = launch.range_mappings(args.range_file)
    if float(mapping['limit_velocity_rad_s']) != 50.:
        raise RuntimeError('this explicitly reviewed entry expects current P/V/T=12.5/50/10')
    mapping.update(fixed_position_impedance='true', effort_limit_nm='7', feedback_torque_limit_nm='8',
                   feedback_velocity_limit_rad_s='50', command_mode='effort', enable_on_activate='true',
                   board_version_verified='true', can_bitrate_verified='true', motor_side_timeout_verified='true',
                   joint_mapping_verified='true', firmware_evidence_declared='true', limits_evidence_declared='true')
    for axis in (1, 2): mapping[f'motor{axis}_firmware_id'] = str(args.firmware_id)
    params = launch.controller_parameters(mapping)
    for controller in CONTROLLERS:
        params[controller]['ros__parameters'].update(run_limit_s=0., allow_continuous_operation=True)
    robot = xacro.process_file(str(share/'config/dm4310_dual.urdf.xacro'), mappings=mapping)
    # xacro returns a minidom document; use ElementTree for mock-only replacement.
    robot = ET.fromstring(robot.toxml())
    if args.shadow:
        hardware = robot.find('ros2_control/hardware'); hardware.clear()
        ET.SubElement(hardware, 'plugin').text = 'mock_components/GenericSystem'
        for axis, joint in enumerate(robot.findall('ros2_control/joint')):
            for param in list(joint.findall('param')): joint.remove(param)
            for state in joint.findall('state_interface'):
                ET.SubElement(state, 'param', name='initial_value').text = str([0., -1.2][axis]) if state.attrib['name'] == 'position' else '0'
        assert robot.find('ros2_control/hardware/plugin').text == 'mock_components/GenericSystem'
    else:
        read_devices(args.query_tool, root/'initial_status.log', mapping, args.firmware_id)
    parameter_path = root/'controllers.yaml'; parameter_path.write_text(yaml.safe_dump(params))
    description = ET.tostring(robot, encoding='unicode'); (root/'robot.urdf').write_text(description)
    # No long-run raw trace: do not grow disk/memory without bound.
    config = root/'manager.yaml'
    config.write_text(yaml.safe_dump({'controller_manager': {'ros__parameters': {
        'robot_description': description, 'hardware_components_initial_state.inactive': ['dm4310_system']}}}))
    log = (root/'manager.log').open('w'); manager = None; trial = None; initialized = False
    armed = claimed = False; result = 1; permissions = [0., 0.]; pubs = []; timer = None
    latest = [None, None]; peak_v = [0., 0.]; peak_t = [0., 0.]; start = None
    def publish():
        for pub, permission in zip(pubs, permissions): pub.publish(Float64(data=permission))
    def observe(msg):
        if any(name not in msg.name for name in NAMES): return
        now = time.monotonic()
        for axis, name in enumerate(NAMES):
            j = msg.name.index(name)
            if any(j >= len(values) for values in (msg.position, msg.velocity, msg.effort)): return
            latest[axis] = (msg.position[j], msg.velocity[j], msg.effort[j], now)
    def healthy(stationary=False):
        now = time.monotonic()
        for axis, sample in enumerate(latest, 1):
            if sample is None: raise RuntimeError(f'axis{axis}: no fresh sample')
            q, v, torque, stamp = sample
            lo, hi = (float(mapping[f'motor{axis}_position_{side}_rad']) for side in ('min', 'max'))
            margin = float(mapping[f'motor{axis}_position_guard_margin_rad']) if stationary else 0.
            if (not all(math.isfinite(x) for x in sample) or not 0 <= now-stamp <= .1 or
                    not lo+margin <= q <= hi-margin or abs(v) > (0.05 if stationary else 50.) or abs(torque) > 8.):
                raise RuntimeError(f'axis{axis}: feedback outside reviewed range {sample}')
            peak_v[axis-1] = max(peak_v[axis-1], abs(v)); peak_t[axis-1] = max(peak_t[axis-1], abs(torque))
    def switch(active):
        req = motion.SwitchController.Request(); req.strictness = req.STRICT
        req.activate_asap = True; req.timeout.sec = 2
        req.activate_controllers = CONTROLLERS if active else []
        req.deactivate_controllers = [] if active else CONTROLLERS
        if not trial.call(trial.switch_client, req, 3.).ok: raise RuntimeError('group controller switch rejected')
    try:
        manager = subprocess.Popen(['ros2', 'run', 'controller_manager', 'ros2_control_node', '--ros-args',
                                    '--params-file', str(parameter_path), '--params-file', str(config)],
                                   stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        # Sequential broadcaster creation is intentional; no competing JSB activation.
        for controller in ['joint_state_broadcaster', *CONTROLLERS]:
            command = ['ros2', 'run', 'controller_manager', 'spawner', controller]
            if controller in CONTROLLERS: command.append('--inactive')
            spawned = subprocess.run(command, capture_output=True, text=True, timeout=20)
            (root/(controller+'.log')).write_text(spawned.stdout+spawned.stderr); spawned.check_returncode()
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO); initialized = True; motion.HARDWARE = 'dm4310_system'
        trial = motion.MotionTrial(publish_targets=False, effort_mode=True)
        trial.create_subscription(JointState, '/joint_states', observe, 10)
        pubs = [trial.create_publisher(Float64, f'/{controller}/enable', 1) for controller in CONTROLLERS]
        timer = trial.create_timer(.02, publish)
        if trial.hardware_state() != State.PRIMARY_STATE_INACTIVE: raise RuntimeError('require inactive hardware')
        guard = ControllerLogGuard(root/'manager.log')
        armed = True; trial.set_hardware(State.PRIMARY_STATE_ACTIVE)
        latest[:] = [None, None]; deadline = time.monotonic()+1.; stable = None
        while time.monotonic() < deadline:
            trial.spin_once(.002)
            if guard.poll(): raise RuntimeError(guard.failure)
            try: healthy(stationary=True)
            except RuntimeError: stable = None; continue
            if stable is None: stable = time.monotonic()
            if time.monotonic()-stable >= .05: break
        else: raise RuntimeError('no fresh stationary 50ms anchor window')
        print('Anchor observations: '+repr(latest), flush=True)
        claimed = True; switch(True)
        if any(pub.get_subscription_count() != 1 for pub in pubs): raise RuntimeError('expected exactly one permission subscriber per axis')
        healthy(stationary=True)
        permissions[:] = [1., 1.]; publish(); start = time.monotonic()
        print('ARMED BOTH: K15 D0.5 cap7Nm each; continuous fixed-anchor spring. Ctrl+C stops and disables BOTH.', flush=True)
        next_guard = start; next_display = start+1.
        while True:
            trial.spin_once(.001); healthy(); now = time.monotonic()
            if now >= next_guard:
                if guard.poll(): raise RuntimeError(guard.failure)
                next_guard = now+.02
            if now >= next_display:
                # Overwrite a small current-state file; never append an unbounded high-rate log.
                (root/'current_state.json').write_text(json.dumps({'elapsed_s': now-start, 'latest': latest}))
                next_display = now+1.
            if args.shadow_duration_s and now-start >= args.shadow_duration_s: break
        result = 0
    except KeyboardInterrupt:
        print('Stopping on Ctrl+C; clearing commands and disabling both motors...', flush=True); result = 0
    except BaseException as exc:
        print('FAIL '+repr(exc), flush=True)
    finally:
        # A second Ctrl+C must not interrupt the bounded shutdown sequence.
        previous_handler = signal.signal(signal.SIGINT, signal.SIG_IGN)
        permissions[:] = [0., 0.]; publish()
        if timer: timer.cancel()
        if trial and claimed:
            try: switch(False)
            except BaseException as exc: print('controller cleanup '+repr(exc), flush=True); result = 2
        if trial and armed:
            try: trial.set_hardware(State.PRIMARY_STATE_INACTIVE)
            except BaseException as exc: print('CUT POWER: disable unconfirmed '+repr(exc), flush=True); result = 2
        if trial: trial.destroy_node()
        if initialized: rclpy.shutdown()
        if manager:
            try:
                os.killpg(manager.pid, signal.SIGINT); manager.wait(timeout=6)
            except subprocess.TimeoutExpired:
                os.killpg(manager.pid, signal.SIGTERM); manager.wait(timeout=5)
        log.close()
        disabled = None
        if not args.shadow:
            try:
                read_devices(args.query_tool, root/'final_status.log', mapping, args.firmware_id)
                disabled = True; print('Independent query: BOTH motors disabled.', flush=True)
            except BaseException as exc:
                disabled = False; result = 2; print('CUT POWER: final disabled verification failed '+repr(exc), flush=True)
        summary = dict(result=result, mock_only=args.shadow, continuous=True, axes=[1, 2],
                       elapsed_s=time.monotonic()-start if start else 0.,
                       peak_velocity_rad_s=peak_v, peak_feedback_torque_nm=peak_t,
                       independently_disabled=disabled)
        (root/'summary.json').write_text(json.dumps(summary, indent=2)); print(summary, flush=True)
        signal.signal(signal.SIGINT, previous_handler)
    return result


if __name__ == '__main__': sys.exit(main())
