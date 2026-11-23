
"""Live mapping and display contracts; no device access."""
import copy,importlib.util,json,math,subprocess,sys
from pathlib import Path
from types import ModuleType,SimpleNamespace as NS
from unittest.mock import patch
import pytest,yaml
import ref01_bench_hold as bench
import ph_exo_bench_worker as worker
from ph_exo_ros_client import sample
ROOT=bench.ROOT

def message():
    return NS(observed_at_monotonic_s=1.01,position_rad=[0,0],velocity_rad_s=[0,0],
              reference_rad=[0,0],mode=2,reason="",state_valid=True,motor_target_nm=[0,0])

def feedback(**kwargs):
    return dict(dict(velocity=math.pi/3,mit_erpm=1120,mit_valid=1,mit_fault_code=0,mit_rx_us=1000000),**kwargs)

def test_live_config_and_state_conversion_agree():
    spec=importlib.util.spec_from_file_location("_erpm_state_check",ROOT/"ros2_ws/src/mech_exo_state/scripts/mit_feedback.py")
    module=importlib.util.module_from_spec(spec)
    stub=ModuleType("mech_exo_state.msg");stub.MitMotorFeedback=NS
    with patch.dict(sys.modules,{"mech_exo_state.msg":stub}):spec.loader.exec_module(module)
    p=yaml.safe_load(bench.SPEED_CONFIG.read_text())["ph_exo_state"]["ros__parameters"]
    state=module.MitFeedback(p["mit_joint_names"],p["mit_feedback_ttl_ms"],
        p["mit_pole_pairs"],p["mit_gear_ratios"],p["mit_speed_directions"],p["mit_speed_confirmed"])
    mapping=bench.feedback_mapping()
    assert list(mapping["axes"])==["104","105"]
    for i,mid in enumerate(("104","105")):
        assert mapping["axes"][mid]["rad_s_per_erpm"]==pytest.approx(state.speed_scales[i])
        assert state.speed_scales[i]*1120==pytest.approx(math.pi/3)
        assert mapping["axes"][mid]["confirmed"] is False

def test_generated_hardware_uses_each_axis_config(tmp_path):
    import xml.etree.ElementTree as ET
    cfg=json.loads((ROOT/"config/parallel_exo/servo_pair.ref01.json").read_text())
    custom=copy.deepcopy(bench.feedback_mapping())
    custom["axes"]["105"]["rad_s_per_erpm"]=.002
    with patch.object(bench,"feedback_mapping",return_value=custom):
        result,_=bench.configuration(cfg,bench.PATHS,{104:80,105:81},tmp_path,"/check",False)
    robot=ET.fromstring(result["/check/controller_manager"]["ros__parameters"]["robot_description"])
    for hw in robot.findall("./ros2_control/hardware"):
        mid=hw.find("param[@name='drive_id']").text
        assert float(hw.find("param[@name='kinematic_erpm_to_rad_s']").text)==custom["axes"][mid]["rad_s_per_erpm"]

@pytest.mark.parametrize("key,value",[("mit_pole_pairs",[0,14]),("mit_pole_pairs",[True,14]),
    ("mit_gear_ratios",[8,float("nan")]),("mit_speed_directions",[-1,1]),
    ("mit_speed_confirmed",[False,1]),("mit_gear_ratios",[8])])
def test_bad_mapping_fails_before_device_access(tmp_path,key,value):
    cfg=yaml.safe_load(bench.SPEED_CONFIG.read_text())
    cfg["ph_exo_state"]["ros__parameters"][key]=value
    p=tmp_path/"bad.yaml";p.write_text(yaml.safe_dump(cfg))
    with pytest.raises(ValueError):bench.feedback_mapping(p)

def test_mapping_in_handshake():
    original=Path.read_bytes
    def changed(path):
        data=original(path)
        return data+b"\n# version check" if path==bench.SPEED_CONFIG else data
    first=worker.source_signature()
    with patch.object(Path,"read_bytes",changed):assert worker.source_signature()!=first

def test_display_uses_actual_velocity_not_raw_erpm():
    raw=feedback(mit_erpm=99999)
    row=bench.panel_observation(message(),{104:raw})
    assert row["feedback"]["104"]["rpm"]==pytest.approx(10)
    assert "rpm" not in raw

@pytest.mark.parametrize("bad",[{"velocity":math.nan},{"mit_valid":0},{"mit_fault_code":1},
    {"mit_rx_us":900000},{"mit_rx_us":2000000}, {"velocity":None}])
def test_invalid_bench_velocity_stays_missing(bad):
    assert bench.panel_observation(message(),{104:feedback(**bad)})["feedback"]["104"]["rpm"] is None

def test_ros_invalid_velocity_stays_missing():
    m=NS(valid=True,age_ms=1,drive_id=104,position_deg=80,output_velocity_rad_s=math.pi,
         output_velocity_valid=False,erpm=1120,iq_a=0)
    robot=NS(sources=[NS(valid=True,source_age_ms=1)],mit_motors=[m])
    assert sample(None,robot,1.)["feedback"]["104"]["rpm"] is None

def test_js_plot_and_csv_do_not_reinterpret_invalid_or_raw_erpm():
    html=(ROOT/"tools/bench_panel/ph_exo_bench_panel.html").read_text(encoding="utf-8")
    fn=html[html.index("function motorValue("):html.index("function values(")]
    code=fn+"""
const assert=require('node:assert/strict');
const row=d=>({observed_s:1.01,feedback:{104:d}});
assert.equal(motorValue(row({rpm:10,mit_erpm:9999}),'speed','104'),10);
assert.equal(motorValue(row({rpm:null,velocity:1,mit_erpm:1120}),'speed','104'),undefined);
assert.equal(motorValue(row({mit_erpm:1120}),'speed','104'),undefined);
const d={velocity:Math.PI/3,mit_valid:1,mit_fault_code:0,mit_rx_us:1e6};
assert.ok(Math.abs(motorValue(row(d),'speed','104')-10)<1e-12);
assert.equal(motorValue(row({...d,mit_valid:0}),'speed','104'),undefined);
assert.equal(motorValue(row({...d,mit_rx_us:0}),'speed','104'),undefined);
console.log('6 JS speed assertions passed');
"""
    result=subprocess.run(["node","-e",code],capture_output=True,text=True)
    assert result.returncode==0,result.stderr
