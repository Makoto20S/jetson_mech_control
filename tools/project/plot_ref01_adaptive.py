
"""Audit final Windows numerical runs and plot old/new controller evidence."""
import argparse
import csv
import hashlib
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from plot_ref01_robustness import audit, read_csv

def load_cases(folder):
    data=json.loads((folder/"summary.json").read_text())
    assert data["complete"]
    return [r for r in data["cases"] if r["mode"]=="predict_B1"]

def draw_pair(old_folder,new_folder,old,new,path):
    fig,axes=plt.subplots(3,2,figsize=(12,9),sharex=True,constrained_layout=True)
    for col,(folder,r,title) in enumerate([(old_folder,old,"Original fixed-model predictor"),
                                         (new_folder,new,"Adaptive + joint wire selection")]):
        a=read_csv(folder,r)
        data=[[np.rad2deg(a["psi"])-r["reference_deg"][0],np.rad2deg(a["beta"])-r["reference_deg"][1]],
              [np.rad2deg(a["dpsi"]),np.rad2deg(a["dbeta"])],[a["sent104"],a["sent105"]]]
        for row in range(3):
            ax=axes[row,col]
            names=["psi","beta"] if row<2 else ["104","105"]
            for y,name,color in zip(data[row],names,["#0072B2","#D55E00"]):
                ax.plot(a["t"],y,label=name,color=color,lw=.9)
            if row<2:
                bound=.5 if row==0 else 1.
                for v in [-bound,bound]: ax.axhline(v,color="#888",ls=":",lw=.8)
                if max(np.max(abs(y)) for y in data[row])>20:
                    ax.set_yscale("symlog",linthresh=1.)
                    ax.text(.98,.02,"symlog",transform=ax.transAxes,ha="right",fontsize=8)
            ax.axvspan(.4,1.,color="#E6AB02",alpha=.12)
            ax.axvline(2.5,color="#888",ls="--",lw=.8)
            ax.set_xlim(0,r["duration"]);ax.grid(alpha=.2);ax.legend(fontsize=8)
            if not r["complete"]:
                ax.axvline(a["t"][-1],color="crimson")
                ax.axvspan(a["t"][-1],r["duration"],color="crimson",alpha=.05)
            if col==0: ax.set_ylabel(["Pose error (deg)","Velocity (deg/s)","Encoded torque (N m)"][row])
            if row==2: ax.set_xlabel("Model time (s)")
        status="PASS" if r["passed"] else ("ABORT: "+str(r["failure"]) if r["failure"] else "Criterion not met")
        axes[0,col].set_title(title+"\n"+status)
        if not r["complete"]:
            axes[0,col].text(.04,.08,f"Stopped at {a['t'][-1]:.3f} s; no post-stop trajectory",
                             color="crimson",transform=axes[0,col].transAxes,fontsize=8)
    fig.suptitle(f"{new['scenario']} | +0.3 N m {['psi','beta'][new['axis']]} disturbance\n"
                 "Same K=10, plant, 20 ms feedback, MIT format and recovery criterion; new B=0.8-1.0")
    fig.savefig(path,dpi=140);plt.close(fig)

def main():
    p=argparse.ArgumentParser()
    for name in ["before-grid","before-long","grid","long","cross","output"]:
        p.add_argument("--"+name,type=Path,required=True)
    args=p.parse_args();out=args.output;out.mkdir(parents=True,exist_ok=False)
    groups={};folders={}
    for name in ["before_grid","before_long","grid","long","cross"]:
        folder=getattr(args,name);folders[name]=folder;groups[name]=load_cases(folder)
    checks={name:audit(folders[name],groups[name]) for name in ["grid","long","cross"]}
    for name in ["grid","long","cross"]:
        for r in groups[name]:
            a=read_csv(folders[name],r)
            for field in ["sent104","sent105"]:
                code=(a[field]+15.)/(30./4095.)
                assert np.max(abs(code-np.round(code)))<1e-9
                assert np.max(abs(a[field]))<=5.+1e-10
    summary={}
    for name,rr in groups.items():
        summary[name]=dict(total=len(rr),passed=sum(r["passed"] for r in rr),
                           completed=sum(r["complete"] for r in rr),
                           tail_error_deg=max((r["tail_error_deg"] or 0 for r in rr)),
                           tail_speed_deg_s=max((r["tail_speed_deg_s"] or 0 for r in rr)))
    (out/"summary.json").write_text(json.dumps(summary,indent=2))
    (out/"audit.json").write_text(json.dumps(checks,indent=2))
    with (out/"case_metrics.csv").open("w",newline="") as f:
        keys=["variant","id","scenario","axis","sign","passed","complete","failure","tail_error_deg",
              "tail_speed_deg_s","settling_s","peak_motor_torque_nm","max_closure_m"]
        w=csv.DictWriter(f,fieldnames=keys,extrasaction="ignore");w.writeheader()
        for name,rr in groups.items():
            for r in rr:w.writerow(dict(variant=name,**r))
    for scenario in ["nominal","rotor_020","combined_rotor"]:
        for axis in (0,1):
            ident=f"grid_{scenario}_predict_B1_{axis}_1"
            old=next(r for r in groups["before_grid"] if r["id"]==ident)
            new=next(r for r in groups["grid"] if r["id"]==ident)
            draw_pair(args.before_grid,args.grid,old,new,out/f"{scenario}_{['psi','beta'][axis]}.png")
    scenarios=[r[0] for r in json.loads((args.grid/"design.json").read_text())["scenarios"]]
    matrix=np.zeros((len(scenarios),2))
    for i,s in enumerate(scenarios):
        for j,name in enumerate(["before_grid","grid"]):
            matrix[i,j]=sum(r["passed"] for r in groups[name] if r["scenario"]==s and r["id"].startswith("grid_"))
    fig,ax=plt.subplots(figsize=(8,9),constrained_layout=True)
    ax.imshow(matrix,vmin=0,vmax=4,cmap="YlGn",aspect="auto")
    ax.set_xticks([0,1],["Original predictor","Final adaptive candidate"])
    ax.set_yticks(range(len(scenarios)),scenarios)
    for i in range(len(scenarios)):
        for j in range(2):
            ax.text(j,i,f"{int(matrix[i,j])}/4",ha="center",va="center",
                    color="white" if matrix[i,j]>=3 else "black")
    ax.set_title("6 s recovery at main pose: both axes, both signs\nSame <=0.5 deg and <=1 deg/s from 2.5 s")
    fig.savefig(out/"before_after_matrix.png",dpi=150);plt.close(fig)
    fig,ax=plt.subplots(figsize=(12,5),constrained_layout=True)
    for axis,color in [(0,"#0072B2"),(1,"#D55E00")]:
        rr=[r for r in groups["long"] if r["axis"]==axis]
        ax.plot(range(len(rr)),[r["tail_speed_deg_s"] for r in rr],"o-",color=color,label=["psi disturbance","beta disturbance"][axis])
    ax.axhline(1.,color="crimson",ls="--",label="unchanged speed limit")
    ax.set_xticks(range(len(scenarios)),scenarios,rotation=65,ha="right")
    ax.set_ylabel("Max speed of BOTH coordinates after 2.5 s (deg/s)")
    ax.set_title("30 s extension: every scenario, positive disturbances")
    ax.legend(fontsize=8);ax.grid(alpha=.2)
    fig.savefig(out/"long_tail_speed.png",dpi=150);plt.close(fig)
    # Inertia estimates are controller diagnostics, not accepted physical parameters.
    fig,axes=plt.subplots(2,2,figsize=(12,7),sharex=True,constrained_layout=True)
    for col,axis in enumerate((0,1)):
        ident=f"grid_rotor_020_predict_B1_{axis}_1"
        a=np.loadtxt(args.grid/(ident+".log"),delimiter=",")
        for row in (0,1):
            axes[row,col].plot(a[:,0],a[:,2+3*row],label="estimated diagonal inertia")
            axes[row,col].axhline(a[0,2+3*row],ls=":",color="#777",label="nominal model")
            axes[row,col].set_ylabel(f"M{row}{row} (kg m2)")
            axes[row,col].set_xlabel("Model time (s)");axes[row,col].grid(alpha=.2)
        axes[0,col].set_title(["psi disturbance","beta disturbance"][axis])
    axes[0,0].legend(fontsize=8);fig.suptitle("Online effective inertia: no plant parameters supplied")
    fig.savefig(out/"inertia_estimates.png",dpi=150);plt.close(fig)
    hashes={}
    for name in ["grid","long","cross"]:
        folder=folders[name]
        for file in folder.iterdir():
            if file.suffix in (".json",".csv",".log",".hpp"):
                hashes[str(file)]=hashlib.sha256(file.read_bytes()).hexdigest()
    (out/"input_hashes.json").write_text(json.dumps(hashes,indent=2))
    print(json.dumps(summary,indent=2))
    print("CSV and encoded-torque audit passed",sum(len(v) for v in checks.values()))
if __name__=="__main__":
    main()
