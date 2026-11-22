"""Pure fixed-pose configuration contract: no ROS, ports or process launches."""
import json
import math
from pathlib import Path

def validate_profile(value):
    expected = {"schema_version", "mode", "stiffness_nm_per_rad",
        "damping_nm_s_per_rad", "motor_torque_cap_nm", "feedback_current_limit_a",
        "relative_travel_limit_deg", "duration_s", "control_period_s",
        "expected_feedback_period_s", "reference_policy", "require_terminal_start"}
    if not isinstance(value, dict) or set(value) != expected:
        raise ValueError("配置字段缺失或含未知字段")
    if type(value["schema_version"]) is not int or value["schema_version"] != 1:
        raise ValueError("不支持的配置版本")
    if value["mode"] != "ordinary_fixed_pose":
        raise ValueError("首版只允许普通固定姿态阻抗")
    if value["reference_policy"] != "capture_at_start" or value["require_terminal_start"] is not True:
        raise ValueError("必须捕获启动姿态并保留终端START确认")
    bounds = {"stiffness_nm_per_rad": (0, 30), "damping_nm_s_per_rad": (0, 1),
              "motor_torque_cap_nm": (0, 10), "feedback_current_limit_a": (0, 6),
              "relative_travel_limit_deg": (0, 60)}
    for key, (lo, hi) in bounds.items():
        pair = value[key]
        if not isinstance(pair, list) or len(pair) != 2:
            raise ValueError(key + ": 必须为两轴")
        for x in pair:
            if type(x) not in (float, int) or not math.isfinite(x) or x < lo or x > hi or (x == 0 and key != "damping_nm_s_per_rad"):
                raise ValueError(key + ": 数值超出当前合同")
    for key in ("duration_s", "control_period_s", "expected_feedback_period_s"):
        x = value[key]
        if type(x) not in (float, int) or not math.isfinite(x):
            raise ValueError(key + ": 非有限数值")
    if not 2 <= value["duration_s"] <= 30 or value["control_period_s"] != .005:
        raise ValueError("当前台架只支持2～30秒和5ms控制周期")
    if not .005 <= value["expected_feedback_period_s"] <= .020:
        raise ValueError("预期反馈周期只接受5～20ms；该字段不会设置设备频率")
    return json.loads(json.dumps(value, allow_nan=False))

def load_profile(path):
    return validate_profile(json.loads(Path(path).read_text(encoding="utf-8")))
