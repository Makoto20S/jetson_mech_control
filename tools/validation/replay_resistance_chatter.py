"""Replay recorded encoder states through the C++ controller, with no device I/O.

The fixed input is not a closed-loop prediction of motor motion or human feel.
Build resistance_controller_replay in mech_controllers with BUILD_TESTING enabled.
"""
import argparse
import csv
import json
from pathlib import Path
import subprocess

import numpy as np


def metrics(path):
    data = np.genfromtxt(path, delimiter=",", names=True)
    torque, velocity = data["effort_nm"], data["velocity_rad_s"]
    assert np.isfinite(torque).all() and np.isfinite(velocity).all()
    assert (np.abs(torque) <= 8.0 + 1e-12).all()
    assert (torque * velocity <= 1e-12).all(), "command assists measured motion"
    steps = np.diff(torque)
    return {
        "rows": len(data),
        "peak_effort_nm": float(np.max(np.abs(torque))),
        "mean_abs_effort_nm": float(np.mean(np.abs(torque))),
        "max_step_nm": float(np.max(np.abs(steps))),
        "steps_over_0p25_nm": int(np.count_nonzero(np.abs(steps) > .25)),
        "total_variation_nm": float(np.sum(np.abs(steps))),
        "no_assisting_command": True,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input-dir", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--probe", type=Path, required=True)
    parser.add_argument("--reference-dir", type=Path)
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=False)
    data = np.genfromtxt(args.input_dir / "real_feedback.csv", delimiter=",", names=True)
    tx = json.loads((args.input_dir / "tx_effort_summary.json").read_text())
    result = {"fixed_input_only": True, "hardware_access": False, "axes": {}}
    for axis in (1, 2):
        timing = tx[str(axis)]
        start, end = timing["first_monotonic_s"], timing["last_monotonic_s"]
        times = np.arange(int((end - start) / .002) + 1) * .002
        # Hold the most recently received sample; never interpolate future data.
        indices = np.searchsorted(data["receive_monotonic_s"], start + times, side="right") - 1
        assert (indices >= 0).all()
        q, v = data[f"q{axis}"][indices], data[f"v{axis}"][indices]
        assert np.isfinite(q).all() and np.isfinite(v).all()
        prepared = args.output_dir / f"input_axis{axis}.csv"
        output = args.output_dir / f"output_axis{axis}.csv"
        with prepared.open("w") as stream:
            writer = csv.writer(stream)
            writer.writerow(["elapsed_s", "position_rad", "velocity_rad_s"])
            writer.writerows(zip(times, q, v))
        subprocess.run([str(args.probe.resolve()), str(prepared), str(output)], check=True)
        actual = metrics(output)
        if args.reference_dir:
            baseline = metrics(args.reference_dir / f"output_axis{axis}.csv")
            assert actual["rows"] == baseline["rows"]
            actual["reference"] = baseline
            actual["total_variation_ratio"] = actual["total_variation_nm"] / baseline["total_variation_nm"]
        result["axes"][str(axis)] = actual
    (args.output_dir / "summary.json").write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
