"""Portable checks for explicit bus routing and bounded independent captures."""
import copy
import importlib.util
import json
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parent


def load(name):
    spec = importlib.util.spec_from_file_location(name, ROOT/(name+'.py'))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


op = load('servo_operator')
storage = load('trace_storage')


def config_for(paths):
    config = json.loads((ROOT/'config.example.json').read_text())
    config.pop('device_path')
    config.update(schema_version=2, calibrated=True,
                  buses=[dict(logical_bus=i+1, device_path=path) for i, path in enumerate(paths)])
    for motor in config['motors']:
        motor.update(logical_bus=1 if motor['id']==105 else 2,
                     position_min_rad=-2., position_max_rad=2.)
    return config


class TwoBusConfigTests(unittest.TestCase):
    def test_migration_preserves_every_mapping_and_limit(self):
        original=json.loads((ROOT/'config.example.json').read_text())
        before=copy.deepcopy(original)
        converted=op.split_buses(original,'/dev/port104','/dev/port105')
        self.assertEqual(original,before)
        for old,new in zip(original['motors'],converted['motors']):
            self.assertEqual(old,{k:v for k,v in new.items() if k!='logical_bus'})
        self.assertEqual(converted['calibrated'],original['calibrated'])
        self.assertEqual(converted['identity'],original['identity'])

    def test_routing_is_explicit_and_calibration_is_preserved(self):
        config = config_for(['/dev/port105', '/dev/port104'])
        before = copy.deepcopy(config)
        controls = ET.fromstring(op.generate_urdf(config)).findall('ros2_control')
        self.assertEqual(len(controls), 2)
        for control, drive, path in zip(controls, (105,104), ('/dev/port105','/dev/port104')):
            self.assertEqual(control.findtext("hardware/param[@name='device_path']"), path)
            self.assertEqual(control.findtext("joint/param[@name='drive_id']"), str(drive))
            self.assertEqual(len(control.findall('joint')), 1)
        self.assertEqual(config, before)
        selected = ET.fromstring(op.generate_urdf(config, motor_id=105)).findall('ros2_control')
        self.assertEqual(len(selected), 1)
        self.assertEqual(selected[0].findtext("hardware/param[@name='trace_name']"), 'bus-1')

    def test_rejects_ambiguous_and_unassigned_routes_before_generation(self):
        for mode in ('path', 'id', 'missing', 'unused', 'mixed', 'bool', 'version'):
            with self.subTest(mode=mode):
                config = config_for(['/dev/port105', '/dev/port104'])
                if mode=='path': config['buses'][1]['device_path']='/dev/port105'
                if mode=='id': config['buses'][1]['logical_bus']=1
                if mode=='missing': config['motors'][0].pop('logical_bus')
                if mode=='unused': config['motors'][0]['logical_bus']=1
                if mode=='mixed': config['device_path']='/dev/other'
                if mode=='bool': config['motors'][0]['logical_bus']=True
                if mode=='version': config['schema_version']=1
                with self.assertRaises(ValueError): op.generate_urdf(config, observe=True)

    def test_legacy_layout_and_total_multi_bus_budget(self):
        config = json.loads((ROOT/'config.example.json').read_text())
        self.assertEqual(len(ET.fromstring(op.generate_urdf(config, True)).findall('ros2_control')), 1)
        self.assertEqual(storage.snapshot_layout([''])[0][0], 'chain.snapshot')
        layout = storage.snapshot_layout(['bus-1', 'bus-2'])
        self.assertEqual(len({row[0] for row in layout}), 4)
        self.assertEqual(sum(row[3] for row in layout), 112*storage.MIB)
        self.assertLess(2*storage.capture_bytes(storage.CHAIN_CAPACITY//2, storage.RAW_CAPACITY//2),
                        storage.SNAPSHOT_LIMIT)
        for names in (['bus-1','bus-1'], ['../escape'], ['', 'bus-2']):
            with self.assertRaises(ValueError): storage.snapshot_layout(names)


if __name__ == '__main__': unittest.main()
