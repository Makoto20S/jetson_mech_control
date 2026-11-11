"""Actual MuJoCo FK plus portable state logic with message fixtures; no ROS runtime."""
import argparse
import ast
from dataclasses import dataclass, field
from pathlib import Path
import sys
from types import ModuleType
import unittest
import xml.etree.ElementTree as ET


@dataclass
class Stamp:
    sec: int = 0
    nanosec: int = 0


@dataclass
class Header:
    stamp: Stamp = field(default_factory=Stamp)
    frame_id: str = ''


@dataclass
class Vector:
    x: float = 0.
    y: float = 0.
    z: float = 0.
    w: float = 0.


@dataclass
class JointState:
    header: Header = field(default_factory=Header)
    name: list = field(default_factory=list)
    position: list = field(default_factory=list)
    velocity: list = field(default_factory=list)
    effort: list = field(default_factory=list)


@dataclass
class Imu:
    header: Header = field(default_factory=Header)
    orientation: Vector = field(default_factory=Vector)
    angular_velocity: Vector = field(default_factory=Vector)
    linear_acceleration: Vector = field(default_factory=Vector)
    orientation_covariance: list = field(default_factory=lambda: [0.]*9)
    angular_velocity_covariance: list = field(default_factory=lambda: [0.]*9)
    linear_acceleration_covariance: list = field(default_factory=lambda: [0.]*9)


@dataclass
class SourceStatus:
    source_stamp: Stamp = field(default_factory=Stamp)
    received_stamp: Stamp = field(default_factory=Stamp)
    source_age_ms: float = 0.
    receive_age_ms: float = 0.
    accepted_samples: int = 0
    valid: bool = False
    sample_time_verified: bool = False
    reason: str = ''


@dataclass
class MitMotorFeedback:
    drive_id: int = 0
    has_sample: bool = False
    rx_sequence: int = 0
    host_rx_us: int = 0
    age_ms: float = 0.
    position_deg: float = 0.
    erpm: float = 0.
    output_velocity_rad_s: float = 0.
    output_velocity_valid: bool = False
    iq_a: float = 0.
    temperature_c: float = 0.
    fault_code: int = 0
    valid: bool = False
    reason: str = ''


@dataclass
class RobotState:
    mit_motors: list = field(default_factory=list)
    header: Header = field(default_factory=Header)
    sources: list = field(default_factory=lambda: [SourceStatus() for _ in range(3)])
    joints: JointState = field(default_factory=JointState)
    imus: list = field(default_factory=lambda: [Imu(), Imu()])
    imu_orientation_valid: list = field(default_factory=lambda: [False]*2)
    imu_angular_velocity_valid: list = field(default_factory=lambda: [False]*2)
    imu_linear_acceleration_valid: list = field(default_factory=lambda: [False]*2)
    alpha_position: list = field(default_factory=lambda: [0.]*2)
    alpha_velocity: list = field(default_factory=lambda: [0.]*2)
    alpha_valid: bool = False
    alpha_velocity_valid: bool = False
    q_position: list = field(default_factory=lambda: [0.]*2)
    q_valid: bool = False
    q_reason: str = ''
    q_closure_residual_m: float = 0.
    mapping_confirmed: bool = False
    messages_aligned: bool = False
    source_skew_ms: float = 0.
    hardware_freshness_verified: bool = False


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mit-replay", type=Path, help="Archived bus traces with adjacent remote manifest")
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[2]
    package = root / 'ros2_ws/src/mech_exo_state'
    for path in package.rglob('*.py'):
        ast.parse(path.read_text(encoding='utf-8'), filename=str(path))
    ET.parse(package / 'package.xml')
    # Fixtures are used only in this dedicated process; production modules are
    # unmodified and Orin separately tests generated ROS messages and DDS.
    for module_name, values in [('sensor_msgs.msg', {'JointState': JointState, 'Imu': Imu}),
                                ('mech_exo_state.msg', {'RobotState': RobotState, 'SourceStatus': SourceStatus, 'MitMotorFeedback': MitMotorFeedback})]:
        parent_name = module_name.split('.')[0]
        parent = ModuleType(parent_name)
        module = ModuleType(module_name)
        for key, value in values.items():
            setattr(module, key, value)
        parent.msg = module
        sys.modules[parent_name], sys.modules[module_name] = parent, module
    sys.path.insert(0, str(package / 'scripts'))
    sys.path.insert(0, str(root / 'mujoco/ref01'))
    suite = unittest.defaultTestLoader.discover(str(package / 'test'), pattern='test_state_core.py')
    suite.addTests(unittest.defaultTestLoader.discover(str(package / 'test'), pattern='test_ref01_fk.py'))
    result = unittest.TextTestRunner(stream=sys.stdout, verbosity=1).run(suite)
    print('Actual MuJoCo FK + portable state logic/message fixtures + Python/XML syntax; NOT ROS/DDS or hardware validation.')
    if not result.wasSuccessful():
        return 1
    if args.mit_replay:
        sys.path.insert(0, str(root / 'tools/project'))
        from replay_mit_feedback import replay
        replay(args.mit_replay)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
