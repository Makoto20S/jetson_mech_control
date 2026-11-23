"""No ROS/device I/O: bench scope and config defaults."""
import unittest,json,xml.etree.ElementTree as ET
from pathlib import Path
from ref01_bench_hold import ROOT,configuration,PATHS,PROFILE
class BenchConfig(unittest.TestCase):
 def setUp(self):
  self.cfg=json.loads((ROOT/"config/parallel_exo/servo_pair.ref01.json").read_text())
 def test_explicit_profile(self):
  p,d=configuration(self.cfg,PATHS,{104:80.6,105:81.3},Path.cwd(),"/bench",False)
  c=p["/bench/ref01_impedance"]["ros__parameters"]
  self.assertFalse(c["simulation_only"]);self.assertEqual(c["bench_profile"],PROFILE)
  r=ET.fromstring(p["/bench/controller_manager"]["ros__parameters"]["robot_description"])
  self.assertEqual({x.text for x in r.findall("./ros2_control/hardware/param[@name='device_path']")},set(PATHS.values()))
  self.assertEqual({x.text for x in r.findall("./ros2_control/hardware/param[@name='torque_max_abs_erpm']")},{"327680"})
  self.assertEqual({x.text for x in r.findall("./ros2_control/hardware/param[@name='kp']")},{"0.0"})
 def test_physical_forbidden_in_rehearsal(self):
  with self.assertRaises(RuntimeError):configuration(self.cfg,PATHS,{104:80,105:80},Path.cwd(),"/bench",True)
 def test_pty_forbidden_in_real(self):
  with self.assertRaises(RuntimeError):configuration(self.cfg,{104:"/dev/pts/4",105:"/dev/pts/5"},{104:80,105:80},Path.cwd(),"/bench",False)
 def test_wrong_bus_forbidden(self):
  with self.assertRaises(RuntimeError):configuration(self.cfg,{104:PATHS[105],105:PATHS[104]},{104:80,105:80},Path.cwd(),"/bench",False)

 def test_predictive_explicit_receipts(self):
  for rehearsal,paths in ((False,PATHS),(True,{104:"/dev/pts/4",105:"/dev/pts/5"})):
   p,_=configuration(self.cfg,paths,{104:80.6,105:81.3},Path.cwd(),"/bench",rehearsal,True)
   c=p["/bench/ref01_impedance"]["ros__parameters"]
   self.assertTrue(c["predictive_impedance"]);self.assertEqual(c["receive_time_uncertainty_s"],.007)
   robot=ET.fromstring(p["/bench/controller_manager"]["ros__parameters"]["robot_description"])
   for joint in robot.findall("./ros2_control/joint"):
    names=[x.get("name") for x in joint.findall("state_interface")]
    for name in ("accepted_effort","accepted_time_s","accepted_sequence"):
     self.assertEqual(names.count(name),1)
 def test_ordinary_default_unchanged(self):
  p,_=configuration(self.cfg,PATHS,{104:80.6,105:81.3},Path.cwd(),"/bench",False)
  self.assertFalse(p["/bench/ref01_impedance"]["ros__parameters"]["predictive_impedance"])
  robot=ET.fromstring(p["/bench/controller_manager"]["ros__parameters"]["robot_description"])
  self.assertEqual(robot.findall("./ros2_control/joint/state_interface[@name='accepted_effort']"),[])
 def test_predictive_fixture_scope(self):
  from ref01_mit_fixture import MitFixture,Plant
  f=MitFixture(predictive=True);f.plant=Plant(.3)
  f.ports=[dict(mid=104,path="/dev/pts/4"),dict(mid=105,path="/dev/pts/5")]
  xml,p=f.configuration(Path.cwd(),"psi",6.,"/ph_exo_manager_config")
  self.assertTrue(p["/ph_exo_manager_config/ref01_impedance"]["ros__parameters"]["predictive_impedance"])
  self.assertEqual(f.cap,5.)
  robot=ET.fromstring(xml)
  self.assertEqual({x.text for x in robot.findall("./ros2_control/hardware/param[@name='bench_rehearsal']")},{"true"})
  f.ports[0]["path"]=PATHS[104]
  with self.assertRaisesRegex(ValueError,"physical device forbidden"):f.configuration(Path.cwd(),"psi",6.,"/ph_exo_manager_config")

class ManualSettings(unittest.TestCase):
 def test_manual_limits_reach_hardware(self):
  cfg=json.loads((ROOT/"config/parallel_exo/servo_pair.ref01.json").read_text())
  limits=dict(torque_nm=2.,current_a=3.,travel_deg=15.)
  params,_=configuration(cfg,{104:"/dev/pts/4",105:"/dev/pts/5"},{104:80.,105:80.},Path.cwd(),"/bench",True,manual_stop=True,limits=limits)
  robot=ET.fromstring(params["/bench/controller_manager"]["ros__parameters"]["robot_description"])
  for hw in robot.findall("./ros2_control/hardware"):
   for name,value in (("bench_manual_stop","true"),("bench_torque_limit_nm","2.0"),
                      ("bench_current_limit_a","3.0"),("bench_travel_limit_deg","15.0")):
    self.assertEqual(hw.find("param[@name='"+name+"']").text,value)
   self.assertIsNone(hw.find("param[@name='command_trace_path']"))
 def test_feedback_uses_custom_limits(self):
  from ref01_bench_hold import check_feedback,validate_limits
  limits=validate_limits(dict(torque_nm=2.,current_a=3.,travel_deg=15.))
  d=dict(mit_rx_us=1000000,mit_position_deg=95,mit_drive_id=104,mit_valid=1,
         mit_fault_code=0,mit_erpm=100,mit_iq_a=3,mit_temperature_c=40)
  check_feedback(104,d,80,1.001,limits)
  d["mit_iq_a"]=3.01
  with self.assertRaisesRegex(RuntimeError,"current_limit"):check_feedback(104,d,80,1.001,limits)
  d["mit_iq_a"]=0;d["mit_position_deg"]=95.01
  with self.assertRaisesRegex(RuntimeError,"travel_limit"):check_feedback(104,d,80,1.001,limits)
 def test_manual_requires_callback_before_ros_import(self):
  from ref01_bench_hold import run
  with self.assertRaisesRegex(RuntimeError,"完整界面"):run(Path.cwd(),False,None,"normal",manual_stop=True)
 def test_invalid_limit_rejected(self):
  from ref01_bench_hold import validate_limits
  with self.assertRaises(RuntimeError):validate_limits(dict(torque_nm=2,current_a=0,travel_deg=15))


class OperatorGate(unittest.TestCase):
 def test_terminal_input_required(self):
  from unittest.mock import patch
  from ref01_bench_hold import confirm_terminal
  with patch("sys.stdin.isatty",return_value=False),patch("builtins.input") as inp:
   with self.assertRaises(RuntimeError):confirm_terminal(30,{104:80,105:81},[0,.1])
   inp.assert_not_called()
 def test_exact_start_required(self):
  from unittest.mock import patch
  from ref01_bench_hold import confirm_terminal
  for answer in ("","yes","start","CANCEL"):
   with patch("sys.stdin.isatty",return_value=True),patch("builtins.input",return_value=answer),patch("builtins.print"):
    with self.assertRaises(RuntimeError):confirm_terminal(30,{104:80,105:81},[0,.1])
  with patch("sys.stdin.isatty",return_value=True),patch("builtins.input",return_value="START"),patch("builtins.print"):
   self.assertGreater(confirm_terminal(30,{104:80,105:81},[0,.1]),0)

class FeedbackDiagnostics(unittest.TestCase):
 def sample(self):
  return dict(mit_drive_id=105,mit_valid=1,mit_fault_code=0,mit_rx_us=1000000.,
              mit_erpm=3400.,mit_iq_a=.44,mit_temperature_c=43.,mit_position_deg=97.)
 def test_new_travel_failure_is_not_stale(self):
  from ref01_bench_hold import check_feedback
  with self.assertRaisesRegex(RuntimeError,r"105 travel_limit:.*delta_deg=90.200"):
   check_feedback(105,self.sample(),6.8,1.001)
 def test_stale_is_separate(self):
  from ref01_bench_hold import check_feedback
  with self.assertRaisesRegex(RuntimeError,"105 feedback_stale"):
   check_feedback(105,self.sample(),66.8,1.061)
 def test_current_is_separate(self):
  from ref01_bench_hold import check_feedback
  d=self.sample();d["mit_iq_a"]=6.01;d["mit_position_deg"]=66.8
  with self.assertRaisesRegex(RuntimeError,"105 current_limit"):check_feedback(105,d,66.8,1.001)
 def test_boundary_and_finite_speed(self):
  from ref01_bench_hold import check_feedback
  d=self.sample();d["mit_position_deg"]=126.8;d["mit_iq_a"]=6.
  check_feedback(105,d,66.8,1.001)
  d["mit_erpm"]=float("nan")
  with self.assertRaisesRegex(RuntimeError,"105 nonfinite_speed"):check_feedback(105,d,66.8,1.001)
 def test_unique_output_is_nonmutating(self):
  from ref01_bench_hold import new_output_path,ROOT
  a,b=new_output_path(),new_output_path()
  self.assertNotEqual(a,b);self.assertEqual(a.parent,ROOT/".local/validation")
  self.assertFalse(a.exists());self.assertFalse(b.exists())


class ModeSelection(unittest.TestCase):
 def test_explicit_modes_and_default(self):
  from ref01_bench_hold import parse_args
  self.assertFalse(parse_args(["--execute"]).predictive)
  self.assertFalse(parse_args(["--execute","--ordinary"]).predictive)
  self.assertTrue(parse_args(["--execute","--predictive"]).predictive)
 def test_modes_cannot_mix(self):
  import contextlib,io
  from ref01_bench_hold import parse_args
  with contextlib.redirect_stderr(io.StringIO()),self.assertRaises(SystemExit):
   parse_args(["--execute","--ordinary","--predictive"])
 def test_ordinary_uses_same_gains_and_no_receipt_inputs(self):
  from ref01_bench_hold import parse_args,GAINS
  cfg=json.loads((ROOT/"config/parallel_exo/servo_pair.ref01.json").read_text())
  a=parse_args(["--execute","--ordinary","--duration","30"])
  p,_=configuration(cfg,PATHS,{104:64.,105:56.6},Path.cwd(),"/bench",False,a.predictive)
  c=p["/bench/ref01_impedance"]["ros__parameters"]
  self.assertFalse(c["predictive_impedance"]);self.assertEqual(c["receive_time_uncertainty_s"],0.)
  self.assertEqual(GAINS,(10.,10.,1.,1.))
  robot=ET.fromstring(p["/bench/controller_manager"]["ros__parameters"]["robot_description"])
  self.assertEqual(robot.findall("./ros2_control/joint/state_interface[@name='accepted_sequence']"),[])


class DampingSelection(unittest.TestCase):
 def test_zero_and_default_gains(self):
  from ref01_bench_hold import parse_args
  self.assertEqual(parse_args(["--execute","--ordinary","--damping","0"]).gains,(10.,10.,0.,0.))
  self.assertEqual(parse_args(["--execute"]).gains,(10.,10.,1.,1.))
 def test_invalid_or_predictive_override_rejected(self):
  import contextlib,io
  from ref01_bench_hold import parse_args
  for value in ("nan","inf","-1","1.1"):
   with contextlib.redirect_stderr(io.StringIO()),self.assertRaises(SystemExit):
    parse_args(["--execute","--damping",value])
  with contextlib.redirect_stderr(io.StringIO()),self.assertRaises(SystemExit):
   parse_args(["--execute","--predictive","--damping","0"])
 def test_main_passes_zero_without_device_call(self):
  from unittest.mock import patch
  import ref01_bench_hold as bench
  with patch("sys.argv",["bench","--execute","--ordinary","--damping","0"]),patch("sys.platform","linux"),patch.object(bench.signal,"signal"),patch.object(bench.signal,"SIGHUP",1,create=True),patch.object(bench,"run") as run,patch("builtins.print"):
   bench.main()
   self.assertEqual(run.call_args.args[-1],(10.,10.,0.,0.))
 def test_terminal_displays_selected_gains(self):
  from unittest.mock import patch
  from ref01_bench_hold import confirm_terminal
  with patch("sys.stdin.isatty",return_value=True),patch("builtins.input",return_value="START"),patch("builtins.print") as output:
   confirm_terminal(30,{104:80,105:81},[0,.1],False,(10.,10.,0.,0.))
   output.assert_any_call("输出轴 [ψ,β] 刚度(N·m/rad)：",(10.,10.),"阻尼(N·m·s/rad)：",(0.,0.),flush=True)

if __name__=="__main__":unittest.main()
