"""Orin-only passive ROS integration test. Only two freshly allocated PTYs are usable."""
import json
import os
from pathlib import Path
import struct
import subprocess
import tempfile
import time
from unittest.mock import patch
import xml.etree.ElementTree as ET


def main():
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from ament_index_python.packages import get_package_prefix, get_package_share_directory
    import benchmark_mit_pty as emu
    import mit_communication as mit
    from mech_exo_state.msg import RobotState

    os.environ.update(ROS_DOMAIN_ID='177', ROS_LOCALHOST_ONLY='1')
    # Distinct 0x29 payloads recorded on November 6: bus 2/104 and bus 1/105.
    frames = {104: '02d00000000c2a00', 105: '01c20000fffe2a00'}
    def replay(drive):
        return emu.packet(struct.pack('<IBB', 0x2900+drive, 4, 8) + bytes.fromhex(frames[drive]))
    overlay = (mit.ROOT / '.local/ros/servo_imu/install').resolve()
    for package in ('mech_bringup', 'mech_exo_state'):
        if Path(get_package_prefix(package)).resolve().parent != overlay:
            raise RuntimeError('Source this development overlay first: ' + str(overlay))
    manager_bin = Path(get_package_prefix('controller_manager')) / 'lib/controller_manager/ros2_control_node'
    state_bin = Path(get_package_prefix('mech_exo_state')) / 'lib/mech_exo_state/state_node.py'
    yaml = Path(get_package_share_directory('mech_exo_state')) / 'config/ph_exo_mit_state.yaml'
    gateways, children = [], []
    with tempfile.TemporaryDirectory(dir=mit.ROOT / '.local/tmp') as temp, patch.object(emu, 'feedback', replay):
        work = Path(temp)
        config = {'motors': [], 'buses': []}
        rclpy.init()
        observer = rclpy.create_node('mit_pty_observer')
        seen = []
        observer.create_subscription(RobotState, '/ph_exo/state', seen.append, qos_profile_sensor_data)
        try:
            for mid, bus in ((104,2),(105,1)):
                gateway = emu.Gateway(mid)
                gateways.append(gateway)
                config['motors'].append(dict(id=mid, logical_bus=bus))
                config['buses'].append(dict(device_path=gateway.path, logical_bus=bus))
            xml, params = mit.artifacts(config, work)
            devices = [e.text for e in ET.fromstring(xml).findall("ros2_control/hardware/param[@name='device_path']")]
            assert devices == [g.path for g in gateways] and all(p.startswith('/dev/pts/') for p in devices)
            paramfile = work / 'params.json'
            paramfile.write_text(json.dumps(params))
            with (work / 'nodes.log').open('w+') as log:
                try:
                    for cmd in ([str(manager_bin), '--ros-args','-r','__ns:='+mit.NS,'--params-file',str(paramfile)],
                                [str(state_bin),'--ros-args','--params-file',str(yaml)]):
                        children.append(subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT, start_new_session=True))
                    subprocess.run(['ros2','run','controller_manager','spawner','joint_state_broadcaster',
                                    '-c',mit.NS+'/controller_manager','--controller-manager-timeout','12'],
                                   check=True, timeout=20, stdout=log, stderr=subprocess.STDOUT)
                    deadline = time.monotonic()+10
                    while time.monotonic() < deadline:
                        rclpy.spin_once(observer, timeout_sec=.05)
                        if seen and seen[-1].sources[0].valid:
                            break
                    assert seen and seen[-1].sources[0].valid, 'Missing valid pair at /ph_exo/state'
                    state = seen[-1]
                    actual = [(a.drive_id, a.position_deg, a.erpm, a.iq_a, a.temperature_c,
                               a.fault_code, a.valid) for a in state.mit_motors]
                    assert actual == [(104,72.,0.,.12,42.,0,True),(105,45.,0.,-.02,42.,0,True)], actual
                    assert all(a.rx_sequence > 0 and 0 <= a.age_ms <= 60 for a in state.mit_motors)
                    assert not state.q_valid and not state.joints.effort and not state.joints.velocity
                    print('PTY -> Ak30System -> joint_state_broadcaster -> /ph_exo/state:', actual)
                except Exception:
                    log.flush()
                    log.seek(0)
                    print(log.read()[-2400:])
                    raise
                finally:
                    for child in reversed(children):
                        mit.stop_child(child)
        finally:
            for gateway in gateways:
                gateway.close()
            observer.destroy_node()
            rclpy.shutdown()
        assert len(gateways) == 2 and not any(g.rows or g.errors for g in gateways)
        print('Private PTYs observed zero motor command frames.')


if __name__ == '__main__':
    main()
