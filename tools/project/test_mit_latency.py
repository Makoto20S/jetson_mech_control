import copy
import json
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET
import mit_latency as latency

class TimingTests(unittest.TestCase):
    def fixture(self):
        p=[dict(motor=105,seq=7,publish_ns=1000000)]
        c=[dict(seq=7,publish_ns=1000000,receive_ns=1200000,update_ns=2000000,command_generation=19)]
        r=[dict(stage='transport_result',result=0,id=0x869,hex=latency.mit.ZERO_WIRE,generation=19,ns=4000000)]
        return p,c,r
    def test_exact_identity_and_segments(self):
        result=latency.correlate(*self.fixture(),105)
        self.assertEqual(result['matched'],1)
        self.assertEqual(result['metrics']['publish_to_usb_ms']['p50_ms'],3)
        self.assertEqual(result['samples'][0]['callback_to_update_ms'],.8)
    def test_same_zero_wrong_generation_not_matched(self):
        p,c,r=self.fixture();r[0]['generation']=20
        result=latency.correlate(p,c,r,105)
        self.assertEqual(result['matched'],0)
        self.assertEqual(result['unmatched_published'],[7])
    def test_clock_and_source_mismatch_rejected(self):
        for key,value in [('publish_ns',999999),('receive_ns',500000)]:
            p,c,r=self.fixture();c[0][key]=value
            with self.assertRaises(ValueError):latency.correlate(p,c,r,105)
    def test_missing_and_duplicate_targets(self):
        p,c,r=self.fixture()
        with self.assertRaises(ValueError):latency.correlate(p+p,c,r,105)
        with self.assertRaises(ValueError):latency.correlate(p,c+c,r,105)
        result=latency.correlate(p,[],r,105)
        self.assertEqual(result['matched'],0)
    def test_nonzero_wire_rejected(self):
        p,c,r=self.fixture();r[0]['hex']='0000000000000000'
        with self.assertRaises(ValueError):latency.correlate(p,c,r,105)
    def test_window_excludes_startup_and_shutdown(self):
        rows=[dict(rx_ns=x) for x in (9,10,15,20,21)]
        self.assertEqual(latency.window_records(rows,'rx_ns',10,20),rows[1:4])
    def test_instrumentation_preserves_zero_configuration(self):
        mit=latency.mit
        old_artifacts,old_execute=mit.artifacts,mit.execute
        try:
            latency.install_timing(1234)
            config=mit.selected_config(json.loads((mit.ROOT/'config/parallel_exo/servo_pair.ref01.json').read_text()),[104,105])
            base_xml, _ = old_artifacts(config, Path('/tmp/test'))
            base_states = {
                joint.get('name'): [state.get('name') for state in joint.findall('state_interface')]
                for joint in ET.fromstring(base_xml).findall('ros2_control/joint')
            }
            xml,params=mit.artifacts(config,Path('/tmp/test'))
            root=ET.fromstring(xml)
            for control in root.findall('ros2_control'):
                hw=control.find('hardware')
                self.assertEqual(hw.find('plugin').text,'mech_bringup/MitTimingSystem')
                for key in ('kp','kd'): self.assertEqual(float(hw.find("param[@name='%s']"%key).text),0)
                joint = control.find('joint')
                names = [state.get('name') for state in joint.findall('state_interface')]
                timing = ['timing_rx_sequence', 'timing_rx_ns', 'timing_read_ns']
                self.assertCountEqual(names, base_states[joint.get('name')] + timing)
            for mid in (104,105):
                settings=params[f'/mit_comm/zero_effort_{mid}']['ros__parameters']
                self.assertEqual((settings['minimum'],settings['maximum']),(0,0))
                self.assertEqual(settings['timing_nonce'],1234)
        finally:mit.artifacts,mit.execute=old_artifacts,old_execute

if __name__=='__main__':unittest.main()
