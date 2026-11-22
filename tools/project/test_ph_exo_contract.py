import unittest, sys, json
from copy import deepcopy
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/"ros2_ws/src/mech_assist_control"))
from mech_assist_control.ph_exo_contract import load_profile,validate_profile
class Contract(unittest.TestCase):
 def setUp(self):
  self.value=load_profile(ROOT/"ros2_ws/src/mech_assist_control/config/ph_exo_fixed_pose.json")
 def test_baseline(self):
  self.assertEqual(self.value["damping_nm_s_per_rad"],[0.,0.])
  self.assertEqual(self.value["stiffness_nm_per_rad"],[10.,10.])
 def test_reject_invalid(self):
  changes=[("schema_version",True),("mode","predictive"),("require_terminal_start",False),
   ("duration_s",31),("control_period_s",.020),("expected_feedback_period_s",0),
   ("stiffness_nm_per_rad",[10,float("nan")]),("damping_nm_s_per_rad",[-.1,0]),
   ("motor_torque_cap_nm",[10,10.1]),("feedback_current_limit_a",[6,6.1]),
   ("relative_travel_limit_deg",[60,60.1])]
  for key,value in changes:
   with self.subTest(key=key):
    d=deepcopy(self.value);d[key]=value
    with self.assertRaises(ValueError):validate_profile(d)
 def test_copy_and_frequency(self):
  d=validate_profile(self.value);d["stiffness_nm_per_rad"][0]=1
  self.assertEqual(self.value["stiffness_nm_per_rad"][0],10)
  self.value["expected_feedback_period_s"]=.005;validate_profile(self.value)
 def test_fields(self):
  for key in self.value:
   d=deepcopy(self.value);del d[key]
   with self.assertRaises(ValueError):validate_profile(d)
if __name__=="__main__":unittest.main()
