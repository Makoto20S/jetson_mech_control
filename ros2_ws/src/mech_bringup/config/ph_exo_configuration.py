import math
from pathlib import Path
import xml.etree.ElementTree as ET
CONTROLLER = "ref01_impedance"
HARDWARE = "ref01_mujoco"
TYPE = "mech_controllers/Ref01ImpedanceController"
PLUGIN = "mech_simulation/Ref01MujocoSystem"
JOINTS = ("motor104_joint", "motor105_joint")
STATES = ("position", "velocity", "sample_time_s", "sequence")
COMMANDS = ("effort", "command_generation")

def build_configuration(model, output, case, duration, namespace):
    if case not in ("psi", "beta", "loss", "duplicate", "reference"):
        raise ValueError("unknown scenario")
    if not math.isfinite(duration) or not 2.5 <= duration <= 120:
        raise ValueError("duration must be 2.5..120s")
    if abs(duration/.005-round(duration/.005)) > 1e-7:
        raise ValueError("duration must be a multiple of 5 ms")
    if not namespace.startswith("/ph_exo_manager_") or "/" in namespace[1:]:
        raise ValueError("isolated namespace required")
    if not Path(model).is_absolute() or not Path(output).is_absolute():
        raise ValueError("model/output must be absolute")
    robot = ET.Element("robot", name="ph_exo_ref01_simulation")
    ET.SubElement(robot, "link", name="base")
    for index, name in enumerate(JOINTS):
        ET.SubElement(robot, "link", name="motor_link_"+str(index))
        joint = ET.SubElement(robot, "joint", name=name, type="continuous")
        ET.SubElement(joint, "parent", link="base")
        ET.SubElement(joint, "child", link="motor_link_"+str(index))
        ET.SubElement(joint, "axis", xyz="0 0 1")
    control = ET.SubElement(robot, "ros2_control", name=HARDWARE, type="system")
    hardware = ET.SubElement(control, "hardware")
    ET.SubElement(hardware, "plugin").text = PLUGIN
    for name, value in dict(simulation_only="true", model_path=str(model), output_dir=str(output),
                            scenario=case, duration_s=str(duration),
                            max_samples=str(round(duration/.005)+1)).items():
        ET.SubElement(hardware, "param", name=name).text = value
    for name in JOINTS:
        joint = ET.SubElement(control, "joint", name=name)
        for interface in STATES:
            ET.SubElement(joint, "state_interface", name=interface)
        for interface in COMMANDS:
            ET.SubElement(joint, "command_interface", name=interface)
    urdf = ET.tostring(robot, encoding="unicode")
    params = {
        namespace+"/controller_manager": {"ros__parameters": {
            "robot_description": urdf, "update_rate": 200, "use_sim_time": False,
            "lock_memory": False, CONTROLLER: {"type": TYPE}}},
        namespace+"/"+CONTROLLER: {"ros__parameters": {
            "simulation_only": True, "command_source": "fixed_pose_v1", "model_path": str(model), "joints": list(JOINTS)}}}
    return urdf, params
