"""Bounded ROS verification: PTY synthetic data or the selected CtrBoard port.

Only this script's bridge child is stopped. No motor process is started.
"""
import argparse
import binascii
from collections import Counter
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import pty
import signal
import struct
import subprocess
import time

import rclpy
from rclpy.qos import qos_profile_sensor_data
from diagnostic_msgs.msg import DiagnosticArray
from sensor_msgs.msg import Imu
from std_msgs.msg import UInt32

from check_imu_device import DEFAULT_DEVICE, check


def crc_reflected(data, initial, polynomial):
    crc = initial
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ (polynomial if crc & 1 else 0)
    return crc


def wire_packet(timestamp, corrupt=False, can_id=0x621):
    # IMU1 absent; IMU2 has known SI conversions and identity orientation.
    imu2 = [0., 0., 1., 180., 0., -90., 0., 0., 0., 1., 0., 0., 0.]
    payload = struct.pack('<I26f4B2H40H', timestamp, *([0.] * 13 + imu2),
                          0, 0, 0, 0, 0, 0, *([0] * 40))
    application = b'\xeb\x90\x05\xc4' + payload
    application += struct.pack('<H', binascii.crc_hqx(application, 0xffff))
    if corrupt:
        application = application[:-1] + bytes([application[-1] ^ 0xff])
    records = b''.join(struct.pack('<IBB', can_id, 0, len(application[i:i+8]))
                       + application[i:i+8] for i in range(0, len(application), 8))
    header = b'\x12' + struct.pack('<H', len(records))
    return b'\xf7' + header + bytes([crc_reflected(header, 0xff, 0x8c)]) + \
        struct.pack('<H', crc_reflected(records, 0xffff, 0x8408)) + records


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--synthetic', action='store_true')
    parser.add_argument('--device', default=DEFAULT_DEVICE)
    parser.add_argument('--motor-config', type=Path, default=Path(__file__).resolve().parents[2] / 'config/parallel_exo/servo_pair.local.json')
    parser.add_argument('--duration', type=float, default=15.)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if not 5 <= args.duration <= 60:
        parser.error('duration must be between 5 and 60 seconds')
    args.output.mkdir(parents=True, exist_ok=False)
    master = slave = None
    proc = node = None
    results = {'kind': 'synthetic_pty' if args.synthetic else 'live_hardware',
               'started_at': datetime.now(timezone.utc).isoformat(),
               'duration_seconds': args.duration, 'passed': False}
    records = []
    child_log = (args.output / 'bridge.log').open('w')
    raw = (args.output / 'messages.jsonl').open('w')
    counts = Counter()
    times = {}
    device_times = []
    imu_samples = {1: [], 2: []}
    diagnostics = []
    try:
        if args.synthetic:
            master, slave = pty.openpty()
            os.set_blocking(master, False)
            device = os.ttyname(slave)
        else:
            results['device_check'] = check(args.device, args.motor_config)
            device = args.device
        rclpy.init()
        node = rclpy.create_node('ctrboard_integration_observer')

        def record(topic, value):
            now = time.monotonic()
            counts[topic] += 1
            times.setdefault(topic, []).append(now)
            item = {'monotonic_seconds': now, 'topic': topic, **value}
            records.append(item)
            raw.write(json.dumps(item, allow_nan=False) + '\n')

        def on_imu(index, msg):
            value = {'frame_id': msg.header.frame_id,
                     'stamp': [msg.header.stamp.sec, msg.header.stamp.nanosec],
                     'quaternion_xyzw': [msg.orientation.x, msg.orientation.y,
                                         msg.orientation.z, msg.orientation.w],
                     'angular_velocity': [msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z],
                     'acceleration': [msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z],
                     'covariance_first': [msg.orientation_covariance[0], msg.angular_velocity_covariance[0], msg.linear_acceleration_covariance[0]]}
            imu_samples[index].append(value)
            record(f'imu/imu{index}/data', value)

        def on_timestamp(msg):
            device_times.append(msg.data)
            record('ctrboard/timestamp_ms', {'timestamp_ms': msg.data})

        def on_diagnostic(msg):
            for status in msg.status:
                level = int.from_bytes(status.level, 'little') if isinstance(status.level, bytes) else int(status.level)
                value = {'level': level, 'message': status.message,
                         'values': {pair.key: pair.value for pair in status.values}}
                diagnostics.append(value)
                record('ctrboard/diagnostics', value)

        subscriptions = [node.create_subscription(Imu, f'imu/imu{i}/data',
                         lambda message, index=i: on_imu(index, message), qos_profile_sensor_data)
                         for i in (1, 2)]
        subscriptions += [node.create_subscription(UInt32, 'ctrboard/timestamp_ms', on_timestamp, qos_profile_sensor_data),
                          node.create_subscription(DiagnosticArray, 'ctrboard/diagnostics', on_diagnostic, 10)]
        prefix = subprocess.check_output(['ros2', 'pkg', 'prefix', 'mech_ctrboard_bridge'], text=True).strip()
        executable = str(Path(prefix) / 'lib/mech_ctrboard_bridge/ctrboard_bridge_node')
        command = [executable, '--ros-args', '-r', '__node:=stm32_ctrboard',
                   '-p', 'transport:=usb_cdc', '-p', f'device_path:={device}',
                   '-p', 'logical_bus:=5', '-p', 'imu_1_frame_id:=imu_1_link',
                   '-p', 'imu_2_frame_id:=imu_2_link']
        results['command'] = command
        proc = subprocess.Popen(command, stdout=child_log, stderr=subprocess.STDOUT)
        start = time.monotonic()
        next_send = start + 2.
        sent = 0
        init = bytearray()
        while time.monotonic() - start < args.duration:
            if proc.poll() is not None:
                raise RuntimeError(f'Bridge exited unexpectedly with {proc.returncode}')
            now = time.monotonic()
            if master is not None:
                try:
                    init.extend(os.read(master, 4096))
                except BlockingIOError:
                    pass
                if now >= next_send and now - start < args.duration - 1.5:
                    # Include bad-CRC and unrelated-CAN packets among good data.
                    sent += 1
                    packet = wire_packet(sent * 100, corrupt=sent % 7 == 0,
                                         can_id=0x123 if sent % 11 == 0 else 0x621)
                    os.write(master, packet[:5])
                    os.write(master, packet[5:])
                    next_send = now + .1
            rclpy.spin_once(node, timeout_sec=.02)
        results['counts'] = dict(counts)
        results['rates_hz'] = {topic: (len(points)-1)/(points[-1]-points[0])
                               for topic, points in times.items() if len(points) > 1}
        results['device_timestamp_samples'] = len(device_times)
        deltas = [(b-a) & 0xffffffff for a, b in zip(device_times, device_times[1:])]
        results['device_timestamp_delta_ms'] = dict(Counter(deltas))
        results['imu_valid_samples'] = {str(index): sum(all(x != -1 for x in v['covariance_first']) for v in values)
                                        for index, values in imu_samples.items()}
        results['last_imu'] = {str(index): values[-1] if values else None for index, values in imu_samples.items()}
        results['diagnostic_states'] = dict(Counter(v['values'].get('state', '') for v in diagnostics))
        results['last_diagnostic'] = diagnostics[-1] if diagnostics else None
        if args.synthetic:
            assert bytes(init) == bytes.fromhex('f7 12 06 00 7d 70 08 00 00 00 00 00 00'), init.hex()
            assert len(imu_samples[2]) >= 10, 'No synthetic IMU stream'
            assert all(v['covariance_first'] == [-1., -1., -1.] for v in imu_samples[1])
            assert all(math.isclose(v['angular_velocity'][0], math.pi, abs_tol=1e-6)
                       and math.isclose(v['acceleration'][2], 9.80665, abs_tol=1e-6)
                       and v['quaternion_xyzw'] == [0., 0., 0., 1.] for v in imu_samples[2])
            assert all(t // 100 % 7 != 0 and t // 100 % 11 != 0 for t in device_times)
            assert any(v['values'].get('state') == 'timed_out' for v in diagnostics)
            assert any(int(v['values'].get('crc_errors', '0')) > 0 for v in diagnostics)
            results['host_write_bytes_hex'] = init.hex()
            results['passed'] = True
        else:
            results['transport_passed'] = len(device_times) >= 10 and any(0 < d < 10000 for d in deltas)
            results['passed'] = results['transport_passed'] and any(n > 0 for n in results['imu_valid_samples'].values())
            results['both_imus_valid'] = all(n > 0 for n in results['imu_valid_samples'].values())
    except Exception as error:
        results['error'] = f'{type(error).__name__}: {error}'
    finally:
        if proc is not None and proc.poll() is None:
            proc.send_signal(signal.SIGINT)
            try:
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                proc.terminate()
                try:
                    proc.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    proc.kill()
                    proc.wait(timeout=3)
        results['bridge_exit_code'] = proc.returncode if proc is not None else None
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        for fd in (master, slave):
            if fd is not None:
                os.close(fd)
        child_log.close()
        raw.close()
        (args.output / 'result.json').write_text(json.dumps(results, indent=2) + '\n')
        print(json.dumps(results, indent=2))
    return 0 if results['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
