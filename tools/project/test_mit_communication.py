import json
from pathlib import Path
import tempfile
import unittest
import xml.etree.ElementTree as ET

import mit_communication as mit


class MitCommunicationTest(unittest.TestCase):
    def setUp(self):
        self.config = json.loads((mit.ROOT/'config/parallel_exo/servo_pair.ref01.json').read_text())

    def test_each_selection_has_zero_only_controller_and_matching_shutdown(self):
        for ids in ([104], [105], [104, 105]):
            selected = mit.selected_config(self.config, ids)
            xml, params = mit.artifacts(selected, Path('/tmp/offline'))
            robot = ET.fromstring(xml)
            shutdown = ET.fromstring(mit.shutdown_urdf(selected))
            self.assertEqual(len(shutdown.findall('ros2_control')), len(ids))
            actual = set()
            for hw in robot.findall('ros2_control/hardware'):
                p = {x.attrib['name']: x.text for x in hw.findall('param')}
                mid = int(p['drive_id'])
                actual.add(mid)
                self.assertEqual(p['sub_mode'], 'torque')
                self.assertEqual(p['feedback_telemetry_log'], 'true')
                self.assertEqual(float(p['kp']), 0)
                self.assertEqual(float(p['kd']), 0)
                self.assertEqual(int(p['logical_bus']), 2 if mid==104 else 1)
                controller = params[f'/mit_comm/zero_effort_{mid}']['ros__parameters']
                self.assertEqual(controller['minimum'], 0)
                self.assertEqual(controller['maximum'], 0)
                self.assertNotEqual(controller['joint'], f'motor{mid}_joint')
            self.assertEqual(actual, set(ids))

    def test_selection_rejects_ambiguous_or_unknown_routes(self):
        for ids in ([], [104,104], [1], [104,106]):
            with self.assertRaises(ValueError):
                mit.selected_config(self.config, ids)
        self.config['motors'][0]['logical_bus'] = self.config['motors'][1]['logical_bus']
        with self.assertRaises(ValueError):
            mit.selected_config(self.config, [104,105])

    def test_zero_wire_is_midpoint_not_all_zero_bytes(self):
        data = bytes.fromhex(mit.ZERO_WIRE)
        self.assertEqual(data[:3], bytes(3))
        p = int.from_bytes(data[3:5], 'big')
        v = (data[5]<<4)|(data[6]>>4)
        t = ((data[6]&15)<<8)|data[7]
        self.assertEqual((p,v,t),(32767,2047,2047))
        self.assertAlmostEqual(t*30/4095-15, -15/4095)

    def trace(self, **change):
        tx = dict(stage='transport_result', ns=2000000, id=0x868, flags=4,
                  deadline=2, result=0, hex=mit.ZERO_WIRE)
        tx.update(change)
        rx = dict(stage='transport_receive', ns=3000000, id=0x2968, flags=4,
                  deadline=2, result=0, hex='0384000000002800')
        with tempfile.TemporaryDirectory() as d:
            path = Path(d)/'trace.jsonl'
            path.write_text('\n'.join(json.dumps(r) for r in
                            [dict(total=2,dropped=0), tx, rx]))
            return mit.analyze_trace(path,104,2)

    def test_trace_proves_actual_ids_payload_and_bus(self):
        r = self.trace()
        self.assertTrue(r['trace_complete'])
        self.assertTrue(r['tx_all_nominal_zero'])
        self.assertEqual(r['position_deg'],[90.,90.])
        self.assertEqual(r['statuses'],[0])
        self.assertFalse(r['position_velocity_ros_valid'])

    def test_trace_rejects_wrong_motor_bus_mode_payload_and_failed_write(self):
        for bad in (dict(id=0x869),dict(id=0x668),dict(deadline=1),
                    dict(hex='0000000000000000'),dict(result=1),dict(flags=0)):
            self.assertFalse(self.trace(**bad)['tx_all_nominal_zero'])

    def test_intervals_do_not_use_ros_display_rate(self):
        r=mit.interval_stats([0,20000000,40000000,61000000])
        self.assertEqual(r['p50_ms'],20)
        self.assertEqual(r['max_ms'],21)

    def test_short_send_burst_is_not_a_complete_window(self):
        motor = dict(trace_complete=True, tx_all_nominal_zero=True, invalid_rx=0,
                     syscall_write_errors=0, syscall_writes=2501, tx={'count':2500},
                     rx={'count':250}, statuses=[0], first_tx_ns=1_000_000_000,
                     last_tx_ns=6_000_000_000, erpm=[0,0], iq_a=[0,.01])
        summary = dict(error=None, disable_returncode=0, selected={'motors':[{}]},
                       motors=[motor], duration_s=5)
        self.assertTrue(mit.complete_window(summary))
        motor['last_tx_ns'] = 1_020_000_000
        self.assertFalse(mit.complete_window(summary))
        motor['last_tx_ns'] = 6_000_000_000
        summary['disable_returncode'] = 2
        self.assertFalse(mit.complete_window(summary))



    def test_health_fault_latches_despite_cached_messages(self):
        from types import SimpleNamespace as N
        events=[]
        monitor=mit.HardwareMonitor(['motor'], lambda: None, lambda *a, **k: events.append(k))
        def reply(state=3, claimed=True):
            return N(component=[N(state=N(id=state), command_interfaces=[
                N(name='motor/effort',is_available=True,is_claimed=claimed)])])
        monitor.accept(reply(),0)
        self.assertTrue(monitor.confirmed)
        with self.assertRaises(RuntimeError): monitor.accept(reply(state=1),.1)
        with self.assertRaises(RuntimeError): monitor.accept(reply(),.2)
        self.assertFalse(monitor.confirmed)
        self.assertEqual(len(events),1)

    def test_health_missing_duplicate_unclaimed_interfaces_fail(self):
        from types import SimpleNamespace as N
        for interfaces in ([],[N(name='motor/effort',is_available=True,is_claimed=False)],
                           [N(name='motor/effort',is_available=True,is_claimed=True)]*2):
            monitor=mit.HardwareMonitor(['motor'],lambda:None,lambda *a,**k:None)
            with self.assertRaises(RuntimeError):
                monitor.accept(N(component=[N(state=N(id=3),command_interfaces=interfaces)]),0)

    def test_health_poll_is_nonblocking_and_has_deadline(self):
        from concurrent.futures import Future
        future=Future()
        requests=[]
        monitor=mit.HardwareMonitor(['motor'],lambda: requests.append(1) or future,lambda *a,**k:None)
        monitor.poll(0)
        monitor.poll(.5)
        self.assertEqual(len(requests),1)
        with self.assertRaisesRegex(RuntimeError,'timeout'): monitor.poll(1.)
        with self.assertRaises(RuntimeError): monitor.poll(2.)

    def test_health_service_failure_latches(self):
        def unavailable(): raise RuntimeError('offline')
        monitor=mit.HardwareMonitor(['motor'],unavailable,lambda *a,**k:None)
        with self.assertRaisesRegex(RuntimeError,'offline'): monitor.poll(0)


if __name__ == '__main__':
    unittest.main()
