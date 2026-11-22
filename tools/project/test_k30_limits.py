import unittest,sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'tools/bench_panel'))
sys.path.insert(0,str(ROOT/'ros2_ws/src/mech_assist_control'))
import ph_exo_bench_worker as worker
import ph_exo_ros_client as client
from ref01_bench_hold import validate_gains
class Limits(unittest.TestCase):
 def test_accepted(self):
  for a,b in [(30,30),(15,25),(.1,30)]:
   self.assertEqual(worker.validate_config(dict(worker.DEFAULT,k_psi=a,k_beta=b))['k_psi'],a)
   self.assertEqual(client.validate_config(dict(k=[a,b],b=[0,1],tau=[10,10]))['k'],[a,b])
   self.assertEqual(validate_gains([a,b,0,1]),(a,b,0,1))
 def test_rejected(self):
  for bad in [0,-1,30.001,float('nan'),float('inf')]:
   with self.assertRaises(ValueError):worker.validate_config(dict(worker.DEFAULT,k_psi=bad))
   with self.assertRaises(ValueError):client.validate_config(dict(k=[bad,30],b=[0,1],tau=[10,10]))
   with self.assertRaises(RuntimeError):validate_gains([bad,30,0,1])
 def test_profile(self):
  from mech_assist_control.ph_exo_contract import load_profile,validate_profile
  import json
  # Reuse the same production contract fixture as the existing test.
  from test_ph_exo_contract import Contract
  c=Contract();c.setUp();d=c.value;d['stiffness_nm_per_rad']=[30,30]
  self.assertEqual(validate_profile(d)['stiffness_nm_per_rad'],[30,30])
if __name__=='__main__':unittest.main()
