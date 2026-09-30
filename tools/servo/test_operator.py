import copy
import importlib.util
import json
import math
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

SPEC = importlib.util.spec_from_file_location('servo_operator', Path(__file__).with_name('operator.py'))

class OperatorTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.op = importlib.util.module_from_spec(SPEC)
        SPEC.loader.exec_module(cls.op)

    def config(self):
        c = self.op.default_config('/dev/ttyTEST')
        for m in c['motors']:
            m.update(target_mapping_verified=True, feedback_mapping_verified=True,
                     mapping_evidence='bounded experiment reference', position_min_rad=-2, position_max_rad=2)
        return c

    def frame(self, t, p=0, status=0):
        return {'schema_version': 1, 'monotonic_ns': t, 'motor_command_frames': 0,
                'motors':[dict(id=i,position_rad=p,electrical_speed_erpm=0,current_iq_a=1,
                               temperature_c=22,raw_status=status,availability='fresh',sequence=t,
                               host_rx_ns=t) for i in (104,105)]}

    def test_margin_is_device_degrees_and_negative_mapping_sorts(self):
        c=self.config(); c['motors'][1]['feedback_scale']=-math.pi/180
        s=self.op.Calibration(c)
        for t,p in [(1_000_000_000,-1),(1_100_000_000,1)]: s.ingest(self.frame(t,p),t)
        s.captures={'right':{'104':1,'105':1},'left':{'104':-1,'105':-1}}
        result=s.finish(10)
        self.assertAlmostEqual(result['motors'][0]['position_min_rad'],-1+math.pi/18)
        self.assertAlmostEqual(result['motors'][1]['position_min_rad'],-1+math.pi/18)
        with self.assertRaises(ValueError): s.finish(60)

    def test_missing_stale_fault_or_gap_invalidates_sweep(self):
        for mode in ('missing','stale','fault','gap'):
            s=self.op.Calibration(self.config()); t=1_000_000_000
            s.ingest(self.frame(t),t)
            f=self.frame(t+100_000_000)
            if mode=='missing': f['motors'].pop()
            if mode=='stale': f['motors'][0]['host_rx_ns']=t-300_000_000
            if mode=='fault': f['motors'][0]['raw_status']=1
            now=t+500_000_000 if mode=='gap' else t+100_000_000
            with self.assertRaises(ValueError): s.ingest(f,now)
            with self.assertRaises(ValueError): s.finish(10)

    def test_capture_needs_stationary_window_and_current_pair(self):
        s=self.op.Calibration(self.config()); t=1_000_000_000
        s.ingest(self.frame(t),t)
        with self.assertRaises(ValueError): s.capture('right',t)
        for k in range(1,7): s.ingest(self.frame(t+k*100_000_000),t+k*100_000_000)
        s.capture('right',t+600_000_000)
        with self.assertRaises(ValueError): s.capture('left',t+900_000_000)

    def test_monitor_marks_old_values_stale_without_zero(self):
        f=self.frame(1_000_000_000,p=1)
        text=self.op.format_monitor(f,1_300_000_000,1_000_000_000)
        self.assertIn('过期',text); self.assertIn('57.296',text)
        f['motors'][0]['current_iq_a']=None
        self.assertIn('未知',self.op.format_monitor(f,1_000_000_000,1_000_000_000))

    def test_generated_urdf_contains_actual_limits_and_interfaces(self):
        import xml.etree.ElementTree as ET
        c=self.config(); c['calibrated']=True; c['motors'][0]['position_min_rad']=-.7
        root=ET.fromstring(self.op.generate_urdf(c))
        joint=root.find('.//ros2_control/joint')
        self.assertEqual(joint.find("param[@name='position_min_rad']").text,'-0.7')
        self.assertEqual({x.attrib['name'] for x in joint.findall('command_interface')},{'position','command_generation'})
        c['motors'][0]['target_mapping_verified']=False
        with self.assertRaises(ValueError): self.op.generate_urdf(c)

    def test_startup_unknown_waits_but_fault_fails(self):
        s=self.op.Calibration(self.config()); f=self.frame(1_000_000_000)
        f['motors'][0].update(availability='unknown',position_rad=None,host_rx_ns=0)
        self.assertFalse(s.ingest(f,1_000_000_000)); self.assertFalse(s.invalid)
        f['motors'][1]['raw_status']=1
        with self.assertRaises(ValueError): s.ingest(f,1_000_000_000)

    def test_interior_peak_sets_range_even_when_endpoint_is_lower(self):
        s=self.op.Calibration(self.config())
        for t,p in [(1_000_000_000,-1),(1_100_000_000,1.5),(1_200_000_000,1)]:
            s.ingest(self.frame(t,p),t)
        s.captures={'right':{'104':1,'105':1},'left':{'104':-1,'105':-1}}
        result=s.finish(10)
        self.assertAlmostEqual(result['motors'][0]['position_max_rad'],1.5-math.pi/18)
        self.assertAlmostEqual(result['motors'][1]['position_min_rad'],-1+math.pi/18)

    def test_identical_endpoint_captures_do_not_save_arbitrary_sweep(self):
        s=self.op.Calibration(self.config())
        s.ingest(self.frame(1_000_000_000,-1),1_000_000_000)
        s.ingest(self.frame(1_100_000_000,1),1_100_000_000)
        s.captures={'right':{'104':1,'105':1},'left':{'104':1,'105':1}}
        with self.assertRaises(ValueError): s.finish(10)

    def test_feedback_device_degrees_preferred_for_display(self):
        f=self.frame(1_000_000_000,1); f['motors'][0]['feedback_position_deg']=123
        self.assertIn('123.000 deg',self.op.format_monitor(f,1_000_000_000,1_000_000_000))

    def test_joint_names_match_deployment_controller(self):
        c=self.config(); c['motors'][0]['joint_name']='wrong'
        with self.assertRaises(ValueError): self.op.validate_config(c,True)

    def test_buffered_lines_are_all_decoded_without_another_pipe_event(self):
        frames,tail=self.op.decode_lines(b'{"schema_version":1}\n{"x":2}\npartial')
        self.assertEqual(frames,[{'schema_version':1},{'x':2}]); self.assertEqual(tail,b'partial')

    def test_live_reader_initial_unknown_manual_capture_and_confirmed_save(self):
        import subprocess
        import sys
        import time
        with tempfile.TemporaryDirectory() as d:
            config=Path(d)/'config.json'; config.write_text(json.dumps(self.config()))
            reader=Path(d)/'reader'
            reader.write_text("#!/usr/bin/env python3\nimport json,time\nstart=time.monotonic()\nwhile True:\n t=time.monotonic_ns(); elapsed=time.monotonic()-start\n p=1 if elapsed<1.2 else -1\n unknown=elapsed<.1\n print(json.dumps({'schema_version':1,'monotonic_ns':t,'motor_command_frames':0,'motors':[{'id':i,'position_rad':None if unknown else p,'electrical_speed_erpm':0,'raw_status':0,'availability':'unknown' if unknown else 'fresh','host_rx_ns':t,'sequence':t} for i in (104,105)]}),flush=True)\n time.sleep(.05)\n")
            reader.chmod(0o755)
            proc=subprocess.Popen([sys.executable,str(Path(__file__).with_name('operator.py')),'calibrate','--config',str(config),'--reader',str(reader)],stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True)
            try:
                time.sleep(.85); proc.stdin.write('\n'); proc.stdin.flush()
                time.sleep(1); proc.stdin.write('\n'); proc.stdin.flush()
                time.sleep(.15)
                out,err=proc.communicate('SAVE\n',timeout=5)
                self.assertEqual(proc.returncode,0,err+out)
                self.assertIn('新鲜',out)
                saved=json.loads(config.read_text()); self.assertTrue(saved['calibrated'])
                self.assertEqual(saved['calibration']['paired_endpoints_rad']['right']['104'],1)
                self.assertEqual(saved['calibration']['paired_endpoints_rad']['left']['105'],-1)
                self.assertTrue(list(Path(d).glob('*.bak')))
                self.assertTrue(list(Path(d).glob('*.report.*.json')))
            finally:
                if proc.poll() is None: proc.kill(); proc.wait()

    def test_tty_status_redraw_preserves_events_without_scrolling(self):
        import io
        class Tty(io.StringIO):
            def isatty(self): return True
        stream=Tty(); display=self.op.TerminalDisplay(stream)
        display.update('电机104\n电机105','向右停稳后回车')
        display.update('更新104\n更新105','向左停稳后回车')
        display.message('右端点已记录')
        text=stream.getvalue()
        self.assertIn('\x1b7',text); self.assertIn('\x1b8\x1b[J',text)
        self.assertIn('右端点已记录',text)
        plain=io.StringIO(); self.op.TerminalDisplay(plain).update('状态','提示')
        self.assertNotIn('\x1b',plain.getvalue())

    def test_interrupted_save_keeps_previous_config(self):
        with tempfile.TemporaryDirectory() as d:
            p=Path(d)/'config.json'; p.write_text('{"old":true}')
            with patch.object(self.op.os,'replace',side_effect=KeyboardInterrupt):
                with self.assertRaises(KeyboardInterrupt): self.op.atomic_save(p,self.config())
            self.assertEqual(json.loads(p.read_text()),{'old':True})
            self.assertFalse(list(Path(d).glob('*.tmp')))

if __name__=='__main__': unittest.main()
