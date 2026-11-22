
"""Render and audit retained Windows robustness CSVs; no simulations or devices."""
import argparse
import csv
import hashlib
import json
from collections import Counter, defaultdict
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

MODES = ["ordinary_B1", "ordinary_B02", "predict_B1"]
LABELS = ["Ordinary K=10, B=1", "Ordinary K=10, B=0.2", "Predictive K=10, B=1"]

def read_csv(folder, case):
    return np.genfromtxt(folder / (case["id"] + ".csv"), delimiter=",", names=True)

def audit(folder, cases):
    checks = []
    delay = {r[0]: r[5] for r in json.loads((folder / "design.json").read_text())["scenarios"]}
    for r in cases:
        a = read_csv(folder, r)
        err = np.column_stack([np.rad2deg(a["psi"])-r["reference_deg"][0],
                               np.rad2deg(a["beta"])-r["reference_deg"][1]])
        speed = np.rad2deg(np.column_stack([a["dpsi"], a["dbeta"]]))
        tail = a["t"] >= 2.5
        ok = (np.abs(err).max(axis=1) <= .5) & (np.abs(speed).max(axis=1) <= 1)
        passed = bool(r["complete"] and tail.any() and ok[tail].all())
        assert passed == r["passed"], r["id"]
        assert len(a) == r["samples"]
        assert np.isfinite(np.column_stack([a[n] for n in a.dtype.names])).all()
        assert np.allclose(np.diff(a["t"]), .005, atol=1e-10)
        assert np.allclose([max(abs(a["sent104"])), max(abs(a["sent105"]))],
                           r["peak_motor_torque_nm"], atol=1e-10)
        # Startup has an immediately available initial sample. Audit only steady sampling.
        steady = a["t"] >= .1
        expected = np.arange(delay[r["scenario"]], delay[r["scenario"]]+20, 5)
        observed = sorted(set(a["age_ms"][steady]))
        assert set(observed).issubset(set(expected)), (r["id"], observed, expected)
        if r["complete"]:
            assert observed == list(expected), (r["id"], observed, expected)
        checks.append(dict(id=r["id"], recovery_recomputed=passed,
                           feedback_age_ms=observed, samples=len(a)))
    return checks

def curves(folder, cases, label, scenario, axis, dest):
    chosen = [next(r for r in cases if r["id"] == f"{label}_{scenario}_{m}_{axis}_1") for m in MODES]
    fig, axes = plt.subplots(3, 3, figsize=(15, 9), sharex=True, constrained_layout=True)
    for col, (r, title) in enumerate(zip(chosen, LABELS)):
        a = read_csv(folder, r)
        signals = [
            [np.rad2deg(a["psi"])-r["reference_deg"][0], np.rad2deg(a["beta"])-r["reference_deg"][1]],
            [np.rad2deg(a["dpsi"]), np.rad2deg(a["dbeta"])],
            [a["sent104"], a["sent105"]],
        ]
        for row in range(3):
            ax = axes[row, col]
            names = ["psi", "beta"] if row < 2 else ["motor 104", "motor 105"]
            for y, name, color in zip(signals[row], names, ["#0072B2", "#D55E00"]):
                ax.plot(a["t"], y, label=name, color=color, linewidth=.9)
            if row < 2:
                bound = .5 if row == 0 else 1.
                for b in [-bound, bound]:
                    ax.axhline(b, color="#777777", linestyle=":", linewidth=.8)
                if max(np.max(abs(y)) for y in signals[row]) > 20:
                    ax.set_yscale("symlog", linthresh=1.)
                    ax.text(.98,.04,"symlog axis",transform=ax.transAxes,ha="right",fontsize=8)
            ax.axvspan(.4, 1., color="#E6AB02", alpha=.13)
            ax.axvline(2.5, color="#666666", linestyle="--", linewidth=.7)
            ax.grid(alpha=.18)
            ax.set_xlim(0, r["duration"])
            if not r["complete"]:
                ax.axvline(a["t"][-1], color="crimson", linewidth=1.)
                ax.axvspan(a["t"][-1], r["duration"], color="crimson", alpha=.04)
            if col == 0:
                ax.set_ylabel(["Pose error (deg)", "Velocity (deg/s)", "Encoded torque (N m)"][row])
            if row == 2:
                ax.set_xlabel("Model time (s)")
            if col == 2:
                ax.legend(fontsize=8,loc="upper right")
        status = "PASS" if r["passed"] else ("ABORT: "+r["failure"] if r["failure"] else "RECOVERY CRITERION NOT MET")
        axes[0,col].set_title(title+"\n"+status,fontsize=10)
        if not r["complete"]:
            axes[0,col].text(.04,.08,f"Stopped at {a['t'][-1]:.3f} s\nNo post-stop trajectory simulated",
                             transform=axes[0,col].transAxes,color="crimson",fontsize=8)
    axis_name = ["psi", "beta"][axis]
    fig.suptitle(f"{scenario} | +0.3 N m {axis_name} disturbance, 0.4-1.0 s | {label}\n"
                 "5 ms control, 20 ms quantized observations; dashed line: recovery evaluation begins",
                 fontsize=12)
    fig.savefig(dest, dpi=140)
    plt.close(fig)

def main():
    p = argparse.ArgumentParser()
    p.add_argument("--grid", type=Path, required=True)
    p.add_argument("--long", type=Path, required=True)
    p.add_argument("--output", type=Path, required=True)
    args = p.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    groups = {}
    audit_results = {}
    for name, folder in [("grid", args.grid), ("long", args.long)]:
        summary = json.loads((folder/"summary.json").read_text())
        assert summary["complete"]
        groups[name] = summary["cases"]
        audit_results[name] = audit(folder, summary["cases"])
    (args.output/"csv_audit.json").write_text(json.dumps(audit_results,indent=2))

    scenarios = [r[0] for r in json.loads((args.grid/"design.json").read_text())["scenarios"]]
    heat = np.zeros((len(scenarios),3))
    counts = []
    for i,s in enumerate(scenarios):
        for j,m in enumerate(MODES):
            rr = [r for r in groups["grid"] if r["id"].startswith("grid_") and r["scenario"]==s and r["mode"]==m]
            assert len(rr)==4
            heat[i,j] = sum(r["passed"] for r in rr)
            counts.append(dict(scenario=s,mode=m,passed=int(heat[i,j]),total=4,
                               completed=sum(r["complete"] for r in rr)))
    fig,ax=plt.subplots(figsize=(10,10),constrained_layout=True)
    im=ax.imshow(heat,vmin=0,vmax=4,cmap="YlGn",aspect="auto")
    ax.set_xticks(range(3),["Ordinary B=1","Ordinary B=0.2","Predictive B=1"])
    ax.set_yticks(range(len(scenarios)),scenarios)
    for i in range(len(scenarios)):
        for j in range(3):
            c=counts[i*3+j]
            ax.text(j,i,f"{c['passed']}/4 pass; {c['completed']}/4 ran",ha="center",va="center",fontsize=9,color="white" if c["passed"] >= 3 else "black")
    ax.set_title("6 s recovery grid at psi=0, beta=20 deg; K=10\n"
                 "Each cell: both axes, both disturbance signs. 'ran' means no early abort.")
    fig.colorbar(im,ax=ax,label="Cases meeting BOTH <=0.5 deg and <=1 deg/s from 2.5 s")
    fig.savefig(args.output/"recovery_matrix.png",dpi=150);plt.close(fig)

    for label, folder, cases, scenarios_to_plot in [
        ("grid",args.grid,groups["grid"],["nominal","rotor_005","delay_10","combined_rotor"]),
        ("long",args.long,groups["long"],["nominal","rotor_005","delay_10","combined_rotor"]),
    ]:
        for scenario in scenarios_to_plot:
            for axis in (0,1):
                curves(folder,cases,label,scenario,axis,args.output/f"{label}_{scenario}_{['psi','beta'][axis]}.png")
    aggregates={}
    for name,cases in groups.items():
        aggregates[name]={m:dict(total=sum(r["mode"]==m for r in cases),
                         passed=sum(r["mode"]==m and r["passed"] for r in cases),
                         completed=sum(r["mode"]==m and r["complete"] for r in cases),
                         failures=dict(Counter(r["failure"] for r in cases if r["mode"]==m and r["failure"])))
                         for m in MODES}
    (args.output/"aggregates.json").write_text(json.dumps(aggregates,indent=2))
    with (args.output/"case_metrics.csv").open("w",newline="") as f:
        cols=["id","mode","scenario","axis","sign","duration","complete","passed","failure","tail_error_deg","tail_speed_deg_s","settling_s","saturation_fraction","peak_motor_torque_nm","model_mass_diagonal_ratio"]
        w=csv.DictWriter(f,fieldnames=cols,extrasaction="ignore");w.writeheader()
        w.writerows(groups["grid"]+groups["long"])
    hashes={}
    for folder in [args.grid,args.long,args.output]:
        for file in sorted(folder.iterdir()):
            if file.is_file() and file.name not in ("probe.exe","mujoco.dll","artifact_hashes.json"):
                hashes[str(file)]=hashlib.sha256(file.read_bytes()).hexdigest()
    (args.output/"artifact_hashes.json").write_text(json.dumps(hashes,indent=2))
    print(json.dumps(aggregates,indent=2))
    print("Independent CSV checks passed:",sum(len(v) for v in audit_results.values()))
if __name__ == "__main__":
    main()
