"""Orin software test. The only device endpoints are the PTYs owned by this process."""
import json
import os
from pathlib import Path
import struct
import subprocess
import sys
import tempfile
import time
from uuid import uuid4
import xml.etree.ElementTree as ET

import benchmark_mit_pty as emu
import mit_communication as mit


class PairGateway(emu.Gateway):
    def __init__(self, mid):
        self.iq_word = 0
        self.connected = True
        super().__init__(mid)

    def validate_command(self, body):
        base = bytes.fromhex(mit.ZERO_WIRE)
        if (len(body) != 14 or body[4:6] != bytes.fromhex('0c08') or
                body[6:12] != base[:6] or body[12] >> 4 != base[6] >> 4):
            raise ValueError('Unexpected MIT mode, gain, position or velocity')
        code = int.from_bytes(body[-2:], 'big') & 4095
        if abs(code - 2047) > 29:
            raise ValueError('Effort exceeds the +/-0.20 Nm fixture envelope')

    def send_feedback(self, acknowledge=False):
        if self.connected:
            body = struct.pack('<IBB', 0x2900+self.drive, 4, 8)
            data = struct.pack('>hhhBB', 700, 0, self.iq_word, 41, 0x77 if acknowledge else 0)
            os.write(self.master, emu.packet(body + data))


def verify_devices(xml, owners):
    controls = ET.fromstring(xml).findall('ros2_control')
    found = {}
    for control in controls:
        hw = control.find('hardware')
        mid = hw.find("param[@name='drive_id']")
        if mid is None:
            mid = control.find("joint/param[@name='drive_id']")
        if mid is None or int(mid.text) in found:
            raise ValueError('Missing or duplicate drive ID')
        found[int(mid.text)] = hw.find("param[@name='device_path']").text
    if found != owners or not all(p.startswith('/dev/pts/') for p in found.values()):
        raise ValueError('Device path must match its newly allocated PTY')


def code(row):
    return int(row['hex'][-3:], 16)


def run_case(work, mode, node, manager_bin, disable_bin):
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
    from std_srvs.srv import Trigger
    import rclpy
    sys.path.insert(0, str(mit.ROOT / 'tools/servo'))
    import servo_operator

    # Unique ROS namespace prevents discovery of a different controller manager.
    mit.NS = '/mit_pair_pty_' + uuid4().hex
    joint_names = ['mit_comm_motor104', 'mit_comm_motor105']
    gateways = []
    manager = None
    pub = None
    client = None
    try:
        for mid in (104, 105):
            gateways.append(PairGateway(mid))
        owned = {g.drive: g.path for g in gateways}
        cfg = json.loads((mit.ROOT/'config/parallel_exo/servo_pair.ref01.json').read_text())
        for bus in cfg['buses']:
            mid = next(m['id'] for m in cfg['motors'] if m['logical_bus'] == bus['logical_bus'])
            bus['device_path'] = owned[mid]
        xml, params = mit.artifacts(cfg, work)
        shutdown = servo_operator.generate_urdf(cfg, observe=True)
        for description in (xml, shutdown):
            verify_devices(description, owned)
        (work/'disable.urdf').write_text(shutdown)
        ctrl = 'pair_effort'
        manager_params = params[mit.NS+'/controller_manager']['ros__parameters']
        for mid in owned:
            del manager_params['zero_effort_'+str(mid)]
            del params[mit.NS+'/zero_effort_'+str(mid)]
        manager_params[ctrl] = {'type': 'mech_bringup/MitPairEffortController'}
        params[mit.NS+'/'+ctrl] = {'ros__parameters': dict(max_effort_nm=.2, max_slew_nm_s=5.,
            max_abs_erpm=100., max_abs_iq_a=.5, max_temperature_c=60.)}
        paramfile = work/'params.json'
        paramfile.write_text(json.dumps(params))
        topic = mit.NS+'/'+ctrl
        pub = node.create_publisher(JointTrajectory, topic+'/target', 1)
        client = node.create_client(Trigger, topic+'/status')
        def publish(names=joint_names, delta_ns=0):
            stamp = node.get_clock().now().nanoseconds + delta_ns
            msg = JointTrajectory(joint_names=names)
            msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(stamp, 10**9)
            msg.points = [JointTrajectoryPoint(effort=[.5, -.1])]
            pub.publish(msg)

        with (work/'manager.log').open('w') as log:
            manager = subprocess.Popen([str(manager_bin),'--ros-args','-r','__ns:='+mit.NS,
                                        '--params-file',str(paramfile)], stdout=log, stderr=log,
                                       start_new_session=True)
            subprocess.run(['ros2','run','controller_manager','spawner',ctrl,'-c',
                            mit.NS+'/controller_manager','--controller-manager-timeout','15',
                            '--param-file',str(paramfile)], check=True, timeout=25, stdout=log, stderr=log)
            deadline = time.monotonic()+5
            while not pub.get_subscription_count() and time.monotonic()<deadline:
                rclpy.spin_once(node,timeout_sec=.01)
            assert pub.get_subscription_count() == 1
            for _ in range(4):
                publish(joint_names[::-1])
                publish(delta_ns=-2_000_000_000)
                rclpy.spin_once(node,timeout_sec=.012)
            assert not any(g.rows for g in gateways), 'Invalid/old commands armed a drive'

            expected = [int((v+15.)/30*4095) for v in (.2,-.1)]
            def at_targets():
                return all(any(r['id']==0x800+g.drive and code(r)==want for r in g.rows)
                           for g,want in zip(gateways,expected))
            deadline = time.monotonic()+1
            while time.monotonic()<deadline and not at_targets():
                publish()
                rclpy.spin_once(node,timeout_sec=.01)
            assert at_targets(), 'Did not observe distinct signed clipped targets on both wires'
            injected = time.monotonic_ns()
            if mode == 'overcurrent':
                gateways[1].iq_word = 75
            if mode == 'loss':
                gateways[1].connected = False
            deadline = time.monotonic()+.35
            while time.monotonic()<deadline:
                if mode != 'expiry':
                    publish()
                rclpy.spin_once(node,timeout_sec=.005)
            # Even new valid targets and restored current cannot restart a trip.
            gateways[1].iq_word = 0
            fence = time.monotonic_ns()
            for _ in range(15):
                publish()
                rclpy.spin_once(node,timeout_sec=.006)
            assert all(not any(r['ns'] >= fence for r in g.rows) for g in gateways), 'Lease restarted'
            assert client.wait_for_service(timeout_sec=2.)
            future = client.call_async(Trigger.Request())
            rclpy.spin_until_future_complete(node,future,timeout_sec=2.)
            status = future.result()
            assert status and not status.success
            expected_stop = {'overcurrent': {'feedback_invalid','inactive'},
                             'loss': {'feedback_invalid','feedback_skew','inactive'},
                             'expiry': {'target_expired','inactive'}}
            assert status.message in expected_stop[mode], (mode,status.message)
            final_rows = [[r for r in g.rows if r['id']==0x800+g.drive] for g in gateways]
            # Shutdown/manager errors may pre-empt a final zero on an unavailable drive.
            # The healthy axis must stop nonzero traffic within the fixture observation window.
            stop_ms = [round((rows[-1]['ns']-injected)/1e6,2) for rows in final_rows]
            assert max(stop_ms) < 220, stop_ms
            if mode in ('expiry','overcurrent'):
                assert all(code(rows[-1])==2047 for rows in final_rows), 'Missing final nominal zero'
        mit.stop_child(manager)
        assert manager.poll() is not None
        manager = None
        shutdown_result = subprocess.run([str(disable_bin),'--urdf',str(work/'disable.urdf'),
            '--trace',str(work/'disable.jsonl'),'--timeout','1'],capture_output=True,text=True,timeout=6)
        events = [json.loads(line) for line in shutdown_result.stdout.splitlines() if line.startswith('{')]
        summary = next(e for e in events if e['event']=='disable_complete')
        ack = sorted(e['id'] for e in events if e['event']=='disable_ack')
        assert ack == ([104] if mode=='loss' else [104,105]), (events,shutdown_result.stderr)
        assert summary['missing'] == ([105] if mode=='loss' else [])
        assert summary['confirmed'] == (mode!='loss')
        assert shutdown_result.returncode == (2 if mode=='loss' else 0)
        for g in gateways:
            assert not g.errors and sum(r['id']==0xf00+g.drive for r in g.rows)==1, (g.drive,g.errors)
        print(json.dumps(dict(case=mode, codes=expected, status=status.message, last_frame_ms=stop_ms,
                              disable_ack=ack, missing=summary['missing'])), flush=True)
    except BaseException:
        if (work/'manager.log').exists():
            print((work/'manager.log').read_text(errors='replace')[-3400:],flush=True)
        raise
    finally:
        if manager is not None:
            mit.stop_child(manager)
        if client is not None:
            node.destroy_client(client)
        if pub is not None:
            node.destroy_publisher(pub)
        for g in gateways:
            g.close()


def main():
    from ament_index_python.packages import get_package_prefix
    import rclpy
    os.environ.update(ROS_DOMAIN_ID='178', ROS_LOCALHOST_ONLY='1')
    overlay = (mit.ROOT/'.local/ros/servo_imu/install').resolve()
    bringup = Path(get_package_prefix('mech_bringup')).resolve()
    if bringup.parent != overlay:
        raise RuntimeError('Source the PH-EXO development overlay')
    manager_bin = Path(get_package_prefix('controller_manager'))/'lib/controller_manager/ros2_control_node'
    rclpy.init()
    node = rclpy.create_node('ph_exo_pair_pty_test_'+uuid4().hex)
    try:
        for mode in ('expiry','overcurrent','loss'):
            with tempfile.TemporaryDirectory(dir=mit.ROOT/'.local/tmp') as temp:
                run_case(Path(temp),mode,node,manager_bin,bringup/'lib/mech_bringup/servo_disable')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__=='__main__':
    main()
