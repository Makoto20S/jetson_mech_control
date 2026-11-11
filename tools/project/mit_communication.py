#!/usr/bin/env python3
"""Bounded AK3.0 ROS communication test. Only nominal zero effort is permitted.

--check is Windows-portable and performs no device I/O. --execute opens physical
ports, sends mode-8 frames and sends mode-15 disable once during cleanup.
The diagnostic joint names/topics deliberately cannot feed REF01 state estimation:
the legacy Torque profile's position/velocity fields are unsupported placeholders.
"""
from __future__ import annotations

import argparse
import copy
import json
import math
import os
from pathlib import Path
import signal
import struct
import subprocess
import sys
import time
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[2]
NS = '/mit_comm'
ZERO_WIRE = '0000007fff7ff7ff'



class HardwareMonitor:
    """Bounded asynchronous supervision; a fault permanently invalidates this run."""
    def __init__(self, names, request, record, period=.2, timeout=1.):
        self.expected = {name + '/effort' for name in names}
        self.request, self.record = request, record
        self.period, self.timeout = period, timeout
        self.future = None
        self.next_check = 0.
        self.deadline = 0.
        self.error = None
        self.confirmed = False

    def fail(self, reason, now):
        if self.error is None:
            self.error = reason
            self.confirmed = False
            self.record('first_monitor_fault', reason=reason, monotonic_s=now)
        raise RuntimeError(self.error)

    def accept(self, response, now):
        if self.error:
            raise RuntimeError(self.error)
        seen = set()
        for component in response.component:
            selected = [i for i in component.command_interfaces if i.name in self.expected]
            if not selected:
                continue
            for interface in selected:
                if (interface.name in seen or component.state.id != 3 or
                        not interface.is_available or not interface.is_claimed):
                    self.fail('Hardware inactive or effort interface unavailable/unclaimed', now)
                seen.add(interface.name)
        if seen != self.expected:
            self.fail('Missing selected effort interfaces', now)
        self.confirmed = True

    def poll(self, now):
        if self.error:
            raise RuntimeError(self.error)
        if self.future is not None:
            if now >= self.deadline:
                self.fail('Hardware status response timeout', now)
            if self.future.done():
                try:
                    response = self.future.result()
                except Exception as exc:
                    self.fail('Hardware status request failed: ' + str(exc), now)
                self.future = None
                self.accept(response, now)
        if self.future is None and now >= self.next_check:
            try:
                self.future = self.request()
            except Exception as exc:
                self.fail('Hardware status unavailable: ' + str(exc), now)
            self.deadline = now + self.timeout
            self.next_check = now + self.period


def selected_config(config, ids):
    if not ids or len(ids) != len(set(ids)) or not set(ids) <= {104, 105}:
        raise ValueError('Select distinct motor IDs from 104 and 105')
    result = copy.deepcopy(config)
    result['motors'] = [m for m in result['motors'] if m['id'] in ids]
    if {m['id'] for m in result['motors']} != set(ids):
        raise ValueError('Missing motor configuration')
    buses = {m['logical_bus'] for m in result['motors']}
    if len(buses) != len(ids):
        raise ValueError('This test requires independent physical buses')
    result['buses'] = [b for b in result['buses'] if b['logical_bus'] in buses]
    if len(result['buses']) != len(ids):
        raise ValueError('Missing/duplicate bus configuration')
    paths = [b['device_path'] for b in result['buses']]
    if len(set(paths)) != len(paths) or any(not p.startswith('/dev/serial/by-id/') for p in paths):
        raise ValueError('Distinct stable device paths are required')
    return result


def artifacts(config, out):
    robot = ET.Element('robot', name='mit_communication_only')
    ET.SubElement(robot, 'link', name='world')
    manager = {'update_rate': 500,
               'joint_state_broadcaster': {'type': 'joint_state_broadcaster/JointStateBroadcaster'}}
    params = {NS + '/joint_state_broadcaster': {'ros__parameters': {'use_local_topics': False}}}
    buses = {b['logical_bus']: b['device_path'] for b in config['buses']}
    for motor in config['motors']:
        mid, bus = motor['id'], motor['logical_bus']
        joint = f'mit_comm_motor{mid}'
        ET.SubElement(robot, 'link', name=joint + '_link')
        j = ET.SubElement(robot, 'joint', name=joint, type='continuous')
        ET.SubElement(j, 'parent', link='world')
        ET.SubElement(j, 'child', link=joint + '_link')
        ET.SubElement(j, 'axis', xyz='0 0 1')
        control = ET.SubElement(robot, 'ros2_control', name=f'mit_comm_{mid}', type='system')
        hw = ET.SubElement(control, 'hardware')
        ET.SubElement(hw, 'plugin').text = 'mech_bringup/Ak30System'
        hardware = dict(feedback_telemetry_log='true', device_path=buses[bus], logical_bus=bus, drive_id=mid,
                        device_id=mid, sub_mode='torque', kp=0.0, kd=0.0,
                        torque_max_abs_erpm=100.0, control_period_ns=2000000,
                        command_ttl_ns=4000000, command_hard_ttl_ns=6000000,
                        feedback_period_ns=20000000, feedback_ttl_ns=60000000,
                        command_trace_path=str(out / f'bus{bus}_motor{mid}.jsonl'))
        for key, value in hardware.items():
            ET.SubElement(hw, 'param', name=key).text = str(value)
        j = ET.SubElement(control, 'joint', name=joint)
        for name in ('effort', 'command_generation'):
            ET.SubElement(j, 'command_interface', name=name)
        for name in ('position', 'velocity', 'effort', 'mit_drive_id', 'mit_rx_sequence', 'mit_rx_us',
                     'mit_valid', 'mit_position_deg', 'mit_erpm', 'mit_iq_a', 'mit_temperature_c', 'mit_fault_code'):
            ET.SubElement(j, 'state_interface', name=name)
        controller = f'zero_effort_{mid}'
        manager[controller] = {'type': 'mech_controllers/EffortCommandController'}
        params[NS + '/' + controller] = {'ros__parameters': dict(
            joint=joint, minimum=0.0, maximum=0.0, max_slew_per_second=0.1,
            target_ttl_nanoseconds=100000000, target_hard_ttl_nanoseconds=106000000)}
    xml = ET.tostring(robot, encoding='unicode')
    manager['robot_description'] = xml
    params[NS + '/controller_manager'] = {'ros__parameters': manager}
    return xml, params


def shutdown_urdf(config):
    # Reuse the verified pair generator, then select complete independent buses.
    # The legacy operator intentionally refuses a one-motor persistent config.
    sys.path.insert(0, str(ROOT / 'tools/servo'))
    import servo_operator
    full = json.loads((ROOT/'config/parallel_exo/servo_pair.ref01.json').read_text())
    root = ET.fromstring(servo_operator.generate_urdf(full, observe=True))
    wanted = {m['id'] for m in config['motors']}
    for hardware in list(root.findall('ros2_control')):
        ids = {int(j.find("param[@name='drive_id']").text) for j in hardware.findall('joint')}
        if not ids <= wanted:
            root.remove(hardware)
    return ET.tostring(root, encoding='unicode')


def interval_stats(ns):
    gaps = sorted((b-a)/1e6 for a, b in zip(ns, ns[1:]))
    if not gaps:
        return {'count': len(ns)}
    def percentile(p):
        return gaps[min(len(gaps)-1, math.ceil(len(gaps)*p)-1)]
    return dict(count=len(ns), p50_ms=percentile(.5), p95_ms=percentile(.95),
                p99_ms=percentile(.99), max_ms=max(gaps),
                average_hz=(len(ns)-1)*1e9/(ns[-1]-ns[0]) if ns[-1]>ns[0] else None)


def decode_rx(record):
    p, v, iq, temp, status = struct.unpack('>hhhbB', bytes.fromhex(record['hex']))
    return dict(ns=record['ns'], position_deg=p*.1, erpm=v*10.,
                iq_a=iq*.01, temperature_c=temp, status=status)


def analyze_trace(path, mid, bus):
    rows = [json.loads(line) for line in path.read_text().splitlines()]
    header, records = rows[0], rows[1:]
    tx = [r for r in records if r['stage'] == 'transport_result']
    rx = [r for r in records if r['stage'] == 'transport_receive' and r['id'] == 0x2900+mid]
    invalid_tx = [r for r in tx if r['result'] != 0 or r['id'] != 0x800+mid or
                  r['flags'] != 4 or r['deadline'] != bus or r['hex'] != ZERO_WIRE]
    samples = []
    invalid_rx = 0
    for r in rx:
        if r['flags'] != 4 or r['deadline'] != bus or len(r['hex']) != 16:
            invalid_rx += 1
            continue
        samples.append(decode_rx(r))
    def bounds(key):
        values = [s[key] for s in samples]
        return [min(values), max(values)] if values else None
    return dict(motor_id=mid, bus=bus, trace_complete=header['dropped']==0 and
                header['total']==len(records), tx=interval_stats([r['ns'] for r in tx]),
                rx=interval_stats([r['ns'] for r in rx]), invalid_tx=len(invalid_tx),
                invalid_rx=invalid_rx, tx_all_nominal_zero=bool(tx) and not invalid_tx,
                position_deg=bounds('position_deg'), erpm=bounds('erpm'),
                iq_a=bounds('iq_a'), temperature_c=bounds('temperature_c'),
                statuses=sorted({s['status'] for s in samples}),
                syscall_writes=sum(r['stage']=='syscall_write' for r in records),
                syscall_write_errors=sum(r['stage']=='syscall_write' and
                    (r['result'] < 0 or r['result'] != r['requested']) for r in records),
                first_tx_ns=tx[0]['ns'] if tx else None,
                last_tx_ns=tx[-1]['ns'] if tx else None,
                position_velocity_ros_valid=False,
                torque_mapping_validated=False, device_mode_ack_available=False)


def complete_window(summary):
    if summary.get('error') or summary.get('disable_returncode') != 0:
        return False
    motors = summary.get('motors', [])
    if len(motors) != len(summary['selected']['motors']):
        return False
    for m in motors:
        if not (m.get('trace_complete') and m.get('tx_all_nominal_zero') and
                m.get('invalid_rx') == 0 and m.get('syscall_write_errors') == 0 and
                m.get('syscall_writes', 0) >= m.get('tx', {}).get('count', 1) and
                m.get('rx', {}).get('count', 0) > 10 and m.get('statuses') == [0]):
            return False
        if (m['last_tx_ns']-m['first_tx_ns'])/1e9 < .9*summary['duration_s']:
            return False
        if max(abs(x) for x in m['erpm']) > 100 or max(abs(x) for x in m['iq_a']) > .5:
            return False
    return bool(motors)


def stop_child(child):
    if child is None or child.poll() is not None:
        return
    os.killpg(child.pid, signal.SIGINT)
    try:
        child.wait(timeout=5)
    except subprocess.TimeoutExpired:
        os.killpg(child.pid, signal.SIGTERM)
        child.wait(timeout=3)


def execute(config, out, duration, timing_nonce=None):
    import fcntl
    import rclpy
    from controller_manager_msgs.srv import ListControllers, ListHardwareComponents, SwitchController, SetHardwareComponentState
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Float64
    from ament_index_python.packages import get_package_prefix

    # This unique directory is a non-replay record. A rerun requires a new path.
    out.mkdir(parents=True, exist_ok=False)
    lock = (ROOT / '.local/mit_communication.lock').open('a+')
    fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    sync_lock = (ROOT / '.local/sync.lock').open('a+')
    fcntl.flock(sync_lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    if (ROOT / '.local/sync/incomplete.json').exists():
        raise RuntimeError('Interrupted source synchronization')
    os.environ['ROS_DOMAIN_ID'] = '173'
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    os.environ['ROS_LOG_DIR'] = str(out / 'ros_logs')
    os.environ['TMPDIR'] = str(ROOT / '.local/tmp')
    (out / 'ros_logs').mkdir()
    (ROOT / '.local/tmp').mkdir(exist_ok=True)
    xml, params = artifacts(config, out)
    (out / 'robot.urdf').write_text(xml)
    (out / 'controllers.json').write_text(json.dumps(params, indent=2))
    servo_xml = shutdown_urdf(config)
    (out / 'disable.urdf').write_text(servo_xml)
    prefix = Path(get_package_prefix('mech_bringup'))
    if not prefix.is_relative_to(ROOT / '.local/ros'):
        raise RuntimeError('Wrong ROS overlay: ' + str(prefix))
    binary = prefix / 'lib/mech_bringup'
    summary = dict(schema_version=1, status='INCOMPLETE', selected=config,
                   duration_s=duration, nominal_effort_nm=0.0, kp=0.0, kd=0.0,
                   wire_hex=ZERO_WIRE, wire_torque_quantization_nm=-15/4095,
                   runtime_overlay=str(prefix), report_clock='host monotonic',
                   ros_state_warning='Torque position/velocity are unsupported placeholders',
                   started_wall_ns=time.time_ns(), events=[])
    (out / 'start.json').write_text(json.dumps(summary, indent=2))
    manager = None
    node = None
    touched = False
    error = None
    messages = []
    joint_rows = []
    controllers = [f'zero_effort_{m["id"]}' for m in config['motors']]
    targets = 0
    timing_publications = []
    timing_feedback = []
    log = (out / 'manager.log').open('w')
    def event(name, **values):
        row = dict(event=name, monotonic_ns=time.monotonic_ns(), **values)
        summary['events'].append(row)
        print(json.dumps(row), flush=True)
    try:
        # Validates the independent shutdown resource list before device access.
        subprocess.run([str(binary/'servo_disable'), '--urdf', str(out/'disable.urdf'),
                        '--check'], check=True, timeout=5)
        with (out / 'passive.jsonl').open('w') as passive:
            touched = True
            pre = subprocess.run([str(binary/'servo_feedback_reader'), '--urdf',
                                  str(out/'disable.urdf'), '--duration', '2'],
                                 stdout=passive, stderr=subprocess.STDOUT, timeout=6)
        if pre.returncode:
            raise RuntimeError('Passive feedback preflight failed; no MIT target sent')
        event('passive_preflight_complete')
        manager_binary = Path(get_package_prefix('controller_manager')) / 'lib/controller_manager/ros2_control_node'
        manager = subprocess.Popen([str(manager_binary),
                                    '--ros-args', '-r', '__ns:='+NS, '--params-file',
                                    str(out/'controllers.json')], stdout=log,
                                   stderr=subprocess.STDOUT, start_new_session=True)
        rclpy.init()
        node = rclpy.create_node('mit_communication_observer', namespace=NS)
        state = {'last': None}
        def callback(msg):
            now = time.monotonic_ns()
            state['last'] = now
            messages.append(now)
            joint_rows.append(dict(ns=now, stamp_ns=msg.header.stamp.sec*10**9+msg.header.stamp.nanosec,
                                   names=list(msg.name), position_placeholder=list(msg.position),
                                   velocity_placeholder=list(msg.velocity), effort_unvalidated=list(msg.effort)))
        node.create_subscription(JointState, NS+'/joint_states', callback, 100)
        if timing_nonce is not None:
            from std_msgs.msg import UInt64MultiArray
            from control_msgs.msg import DynamicJointState
            last_feedback_sequence = {}
            def timing_callback(msg):
                now = time.monotonic_ns()
                for name, values in zip(msg.joint_names, msg.interface_values):
                    d = dict(zip(values.interface_names, values.values))
                    keys = ('timing_rx_sequence', 'timing_rx_ns', 'timing_read_ns')
                    if not all(k in d and math.isfinite(d[k]) for k in keys):
                        continue
                    seq, received, read_done = [int(d[k]) for k in keys]
                    if seq > last_feedback_sequence.get(name, 0):
                        last_feedback_sequence[name] = seq
                        timing_feedback.append(dict(joint=name,seq=seq,rx_ns=received,
                                                    read_ns=read_done,callback_ns=now))
            node.create_subscription(DynamicJointState, NS+'/dynamic_joint_states', timing_callback, 100)
        message_type = Float64 if timing_nonce is None else UInt64MultiArray
        suffix = '/target_effort' if timing_nonce is None else '/timed_zero'
        publishers = [node.create_publisher(message_type, NS+'/'+c+suffix, 10) for c in controllers]
        publication_sequence = 0
        def publish_zero():
            nonlocal publication_sequence
            publication_sequence += 1
            for motor, pub in zip(config['motors'], publishers):
                if timing_nonce is None:
                    pub.publish(Float64(data=0.0))
                else:
                    before = time.monotonic_ns()
                    pub.publish(UInt64MultiArray(data=[timing_nonce, publication_sequence, before]))
                    timing_publications.append(dict(motor=motor['id'],seq=publication_sequence,
                        publish_ns=before,publish_return_ns=time.monotonic_ns()))
        def call(kind, suffix, request, timeout=5.):
            client = node.create_client(kind, NS+'/controller_manager/'+suffix)
            try:
                if not client.wait_for_service(timeout_sec=timeout):
                    raise RuntimeError('Service unavailable: '+suffix)
                future = client.call_async(request)
                rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
                if not future.done() or future.exception():
                    raise RuntimeError('Service result uncertain: '+suffix)
                return future.result()
            finally:
                node.destroy_client(client)
        call(ListHardwareComponents, 'list_hardware_components', ListHardwareComponents.Request())
        for name in ['joint_state_broadcaster'] + controllers:
            cmd = ['ros2', 'run', 'controller_manager', 'spawner', name,
                   '--controller-manager', NS+'/controller_manager', '--controller-manager-timeout', '10']
            if name != 'joint_state_broadcaster':
                cmd.append('--inactive')
            with (out / ('spawn_'+name+'.log')).open('w') as f:
                subprocess.run(cmd, stdout=f, stderr=subprocess.STDOUT, check=True, timeout=20)
        listed = call(ListControllers, 'list_controllers', ListControllers.Request())
        states = {c.name: c.state for c in listed.controller}
        if any(states.get(c) != 'inactive' for c in controllers):
            raise RuntimeError('Effort controller was not initially inactive')
        deadline = time.monotonic()+3
        expected = {f'mit_comm_motor{m["id"]}' for m in config['motors']}
        while (not joint_rows or not expected <= set(joint_rows[-1]['names'])) and time.monotonic()<deadline:
            rclpy.spin_once(node, timeout_sec=.02)
        if not joint_rows or not expected <= set(joint_rows[-1]['names']):
            raise RuntimeError('Missing ROS joint state channels')
        req = SwitchController.Request()
        req.activate_controllers = controllers
        req.strictness = SwitchController.Request.STRICT
        req.timeout.sec = 3
        if not call(SwitchController, 'switch_controller', req).ok:
            raise RuntimeError('Controller activation refused')
        health_client = node.create_client(ListHardwareComponents, NS+'/controller_manager/list_hardware_components')
        def request_health():
            if not health_client.service_is_ready():
                raise RuntimeError('hardware service unavailable')
            return health_client.call_async(ListHardwareComponents.Request())
        monitor = HardwareMonitor(expected, request_health, event)
        monitor.accept(call(ListHardwareComponents, 'list_hardware_components',
                            ListHardwareComponents.Request()), time.monotonic())
        event('zero_effort_window_start')
        began = time.monotonic()
        next_publish = began
        while time.monotonic()-began < duration:
            if manager.poll() is not None:
                raise RuntimeError('Controller manager exited')
            now = time.monotonic()
            monitor.poll(now)
            if now >= next_publish:
                publish_zero()
                targets += 1
                next_publish = now + .02
            rclpy.spin_once(node, timeout_sec=.002)
            monitor.poll(time.monotonic())
            if state['last'] is None or time.monotonic_ns()-state['last'] > 200000000:
                raise RuntimeError('ROS observer stale')
            # This division only reverses the existing software scaling to Iq;
            # it does not qualify the physical torque calibration.
            if any(not math.isfinite(x) or abs(x/0.7382) > .5
                   for x in joint_rows[-1]['effort_unvalidated']):
                raise RuntimeError('Current diagnostic exceeded 0.5 A or became invalid')
        event('zero_effort_window_end', published_batches=targets)
        # Continue zero targets while the asynchronous deactivation is pending.
        timer = node.create_timer(.02, publish_zero)
        hardware = call(ListHardwareComponents, 'list_hardware_components', ListHardwareComponents.Request())
        if any(h.state.id != 3 for h in hardware.component):
            raise RuntimeError('Hardware left ACTIVE during communication window')
        active = call(ListControllers, 'list_controllers', ListControllers.Request())
        states = {c.name: c.state for c in active.controller}
        if any(states.get(c) != 'active' for c in controllers):
            raise RuntimeError('Effort controller stopped during communication window')
        req = SwitchController.Request()
        req.deactivate_controllers = controllers
        req.strictness = SwitchController.Request.STRICT
        req.timeout.sec = 3
        if not call(SwitchController, 'switch_controller', req).ok:
            raise RuntimeError('Controller deactivation refused')
        node.destroy_timer(timer)
        event('controllers_inactive')
        for motor in config['motors']:
            req = SetHardwareComponentState.Request()
            req.name = f'mit_comm_{motor["id"]}'
            req.target_state.id = 2  # INACTIVE
            if not call(SetHardwareComponentState, 'set_hardware_component_state', req).ok:
                raise RuntimeError('Hardware deactivation refused')
            req.target_state.id = 1  # UNCONFIGURED: closes ports and flushes traces
            if not call(SetHardwareComponentState, 'set_hardware_component_state', req).ok:
                raise RuntimeError('Hardware cleanup refused')
        event('hardware_ports_released')
    except BaseException as exc:
        error = f'{type(exc).__name__}: {exc}'
        event('test_error', message=error)
    finally:
        try:
            stop_child(manager)
        except BaseException as exc:
            error = (error or '')+'; manager termination uncertain: '+str(exc)
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        log.close()
        if touched and (manager is None or manager.poll() is not None):
            # Exactly one shutdown attempt. No retry or automatic reactivation.
            with (out/'disable.log').open('w') as f:
                try:
                    result = subprocess.run([str(binary/'servo_disable'), '--urdf',
                                             str(out/'disable.urdf'), '--trace',
                                             str(out/'disable_trace.jsonl'), '--timeout', '2'],
                                            stdout=f, stderr=subprocess.STDOUT, timeout=7)
                    summary['disable_returncode'] = result.returncode
                except BaseException as exc:
                    summary['disable_error'] = str(exc)
        if timing_nonce is not None:
            summary['timing_nonce'] = timing_nonce
            (out/'timing_publications.json').write_text(json.dumps(timing_publications))
            (out/'timing_feedback.json').write_text(json.dumps(timing_feedback))
        summary['error'] = error
        summary['ros_observer'] = interval_stats(messages)
        summary['published_zero_batches'] = targets
        summary['manager_returncode'] = None if manager is None else manager.poll()
        with (out/'ros_states.jsonl').open('w') as f:
            for row in joint_rows:
                f.write(json.dumps(row)+'\n')
        summary['motors'] = []
        for motor in config['motors']:
            mid, bus = motor['id'], motor['logical_bus']
            try:
                summary['motors'].append(analyze_trace(out/f'bus{bus}_motor{mid}.jsonl', mid, bus))
            except Exception as exc:
                summary['motors'].append(dict(motor_id=mid, error=str(exc)))
        passed = complete_window(summary)
        summary['status'] = 'COMMUNICATION_CHECK_COMPLETE' if passed else 'INCOMPLETE'
        summary['finished_wall_ns'] = time.time_ns()
        (out/'summary.json').write_text(json.dumps(summary, indent=2))
        event('finished', status=summary['status'], disable_returncode=summary.get('disable_returncode'))
        print('RESULT_DIRECTORY='+str(out), flush=True)
        if summary.get('disable_returncode') != 0:
            print('DISABLE NOT CONFIRMED: CUT MOTOR POWER. Do not retry automatically.', flush=True)
        fcntl.flock(sync_lock, fcntl.LOCK_UN)
        fcntl.flock(lock, fcntl.LOCK_UN)
        sync_lock.close()
        lock.close()
    return 0 if passed else 2


def main():
    p = argparse.ArgumentParser(description=__doc__)
    mode = p.add_mutually_exclusive_group(required=True)
    mode.add_argument('--check', action='store_true')
    mode.add_argument('--execute', action='store_true')
    p.add_argument('--motors', default='105,104')
    p.add_argument('--duration', type=float, default=5.)
    p.add_argument('--output', type=Path)
    a = p.parse_args()
    if not math.isfinite(a.duration) or not 0<a.duration<=20:
        p.error('duration must be within (0,20] seconds')
    config = selected_config(json.loads((ROOT/'config/parallel_exo/servo_pair.ref01.json').read_text()),
                             [int(x) for x in a.motors.split(',')])
    out = a.output or ROOT/'.local/validation/mit_communication_check'
    xml, params = artifacts(config, out)
    if a.check:
        shutdown = ET.fromstring(shutdown_urdf(config))
        if len(shutdown.findall('ros2_control')) != len(config['motors']):
            raise ValueError('Shutdown resource selection mismatch')
        print(json.dumps(dict(configuration_valid=True, motors=[m['id'] for m in config['motors']],
                             nominal_effort_nm=0.0, kp=0.0, kd=0.0, wire_hex=ZERO_WIRE,
                             motor_command_frames=0, physical_ports_opened=0,
                             ros_position_velocity_valid=False), indent=2))
        return 0
    out = out.resolve()
    if not out.is_relative_to((ROOT/'.local/validation').resolve()):
        p.error('Hardware output must be below this repository .local/validation')
    def interrupted(signum, frame):
        raise KeyboardInterrupt('signal '+str(signum))
    signal.signal(signal.SIGTERM, interrupted)
    signal.signal(signal.SIGHUP, interrupted)
    return execute(config, out, a.duration)


if __name__ == '__main__':
    raise SystemExit(main())
