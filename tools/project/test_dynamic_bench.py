import math,unittest
from unittest.mock import patch
from pathlib import Path
import json,xml.etree.ElementTree as ET
from ref01_dynamic_bench import Trajectory,confirm,GAINS,LIMITS
from ref01_bench_hold import ROOT,PATHS,configuration,DEFAULT_LIMITS
class Contract(unittest.TestCase):
 def test_exact_start(self):
  for answer in ('','yes','start'):
   with patch('sys.stdin.isatty',return_value=True),patch('builtins.input',return_value=answer),patch('builtins.print'):
    with self.assertRaises(RuntimeError):confirm(26,{104:80,105:81},[0,.35],False,GAINS)
  with patch('sys.stdin.isatty',return_value=False),patch('builtins.input') as inp:
   with self.assertRaises(RuntimeError):confirm(26,{104:80,105:81},[0,.35],False,GAINS)
   inp.assert_not_called()
 def test_start_and_axis_order_and_cut(self):
  tr=Trajectory();home=[.1,.35]
  for t in (0.,1.,7.,12.,13.,19.,21.9):
   self.assertEqual(tr.sample(t,home),(home,[0.,0.]))
  q,dq=tr.sample(4.,home)
  self.assertAlmostEqual(q[0]-home[0],math.radians(3));self.assertEqual(q[1],home[1])
  q,dq=tr.sample(16.,home)
  self.assertEqual(q[0],home[0]);self.assertAlmostEqual(q[1]-home[1],math.radians(3))
  self.assertIsNone(tr.sample(22.,home))
  self.assertEqual(home,[.1,.35])
 def test_out_of_range_before_start(self):
  tr=Trajectory()
  for q in ([0,-.01],[math.pi/4,.35],[0,math.pi/4]):
   with self.assertRaises(RuntimeError):tr.validate(q)
 def test_dynamic_config_keeps_cap_and_ports(self):
  cfg=json.loads((ROOT/'config/parallel_exo/servo_pair.ref01.json').read_text())
  p,_=configuration(cfg,PATHS,{104:80,105:81},Path.cwd(),'/bench',False,limits=LIMITS,dynamic=True)
  self.assertEqual(p['/bench/ref01_impedance']['ros__parameters']['command_source'],'dynamic_ref_v1')
  root=ET.fromstring(p['/bench/controller_manager']['ros__parameters']['robot_description'])
  self.assertEqual(LIMITS,DEFAULT_LIMITS)
  self.assertEqual(LIMITS,dict(torque_nm=10.,current_a=6.,travel_deg=90.))
  self.assertEqual(root.findall("./ros2_control/hardware/param[@name='bench_torque_limit_nm']"),[])
  self.assertEqual({v.text for v in root.findall("./ros2_control/hardware/param[@name='device_path']")},set(PATHS.values()))
if __name__=='__main__':unittest.main()
