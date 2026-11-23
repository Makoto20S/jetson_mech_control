"""Windows display calculations. Inputs use the same model branch as the controller."""
import math
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "mujoco/ref01"))
from ref01_kinematics import inverse_kinematics
from ph_exo_bench_worker import DEFAULT, make_trajectory
from mech_assist_control.robot.ph_exo_reference import pose


def encoder_reference(q):
    """Convert [psi, beta] rad to encoder coordinates [104, 105] deg."""
    if not isinstance(q, (tuple, list)) or len(q) != 2:
        return None
    try:
        alpha1, alpha2 = inverse_kinematics(*q, unit="rad")
    except ValueError:
        return None
    return [45 + math.degrees(alpha2), 45 + math.degrees(alpha1)]


def preview_plan(request, ros=False):
    cfg = request["config"]
    if ros:
        from ph_exo_ros_client import validate_config
        checked = validate_config(cfg)
        cfg = dict(DEFAULT, **{key: checked[key] for key in
                   ("pose_deg", "move_s", "amp_deg", "period_s") if key in checked})
    home = request["home_deg"]
    if (not isinstance(home, list) or len(home) != 2 or
            any(type(x) not in (int, float) or not math.isfinite(x) for x in home)):
        raise ValueError("请填写预览起点的 ψ 和 β")
    q0 = pose([math.radians(x) for x in home])
    plan = make_trajectory(cfg)
    if plan:
        plan.validate(q0)
    periods = [p for p, a in zip(cfg.get("period_s", []), cfg.get("amp_deg", [])) if a]
    duration = (plan.startup_s if plan else 1.) + max(periods, default=1.)
    if cfg.get("wave") == "sine":
        duration += cfg["period_s"][0]  # Include one full cycle after the sine envelope.
    count = math.ceil(duration * 40)
    rows = []
    for i in range(count + 1):
        t = duration * i / count
        q = plan.sample(t, q0)[0] if plan else q0
        rows.append(dict(observed_s=t, reference=q, encoder_ref_deg=encoder_reference(q)))
    return rows
