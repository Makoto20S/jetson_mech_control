from pathlib import Path
import sys,json,hashlib,csv,os
import numpy as np
R=Path(__file__).resolve().parents[2];sys.path.insert(0,str(R/'tools/project'))
import verify_ref01_5ms_damping as v
from sync_orin import safe_path
out=safe_path(R,'tools/bench_panel/data/kb_5ms_20261114_v2');out.mkdir(parents=True,exist_ok=False)
v.compile_probe(out)
results=[]
# One LSB reserved so truncated MIT wire torque stays within each selected tier.
tiers={10:4,15:6,20:6,25:8,30:10}
jobs=[(k,b,axis,sign) for k in tiers for b in v.BS for axis in (0,1) for sign in (-1,1)]
for n,(k,b,axis,sign) in enumerate(jobs,1):
    row=v.run_case(out,None,'pipeline5',b,axis,sign,K=k,cap=tiers[k]-30/4095)
    row['tier_nm']=tiers[k]
    a=np.loadtxt(out/(row['id']+'.csv'),delimiter=',',skiprows=1,ndmin=2)
    valid=a[:,10]==2
    assert abs(a[:,5:9]).max()<=tiers[k]+1e-10
    row['saturation_fraction']=float(np.mean(a[valid,27]<.999999))
    row['min_scale']=float(a[valid,27].min())
    results.append(row);v.save(out/'summary.json',dict(complete=n==len(jobs),cases=results))
    if n%12==0:print(f'{n}/{len(jobs)} cases, {sum(x["passed"] for x in results)} recovery pass',flush=True)
v.validate(out,results)
v.save(out/'design.json',dict(K=list(tiers),B=v.BS,tiers=tiers,feedback_ms=5,command_delay_ms=5,MIT_bits=12,MIT_range_nm=[-15,15],quantization_reserve_nm=30/4095,reference_deg=np.rad2deg(v.REF).tolist(),perturbation_nm=.3,perturbation_s=[.4,1],duration_s=6,recovery=dict(after_s=2.5,error_deg=.5,speed_deg_s=1),scope='Windows software only; ordinary fixed-pose, not gait tracking'))
os.environ['MPLCONFIGDIR']=str(out/'mplconfig')
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
plt.rcParams.update({'font.family':'Microsoft YaHei','axes.unicode_minus':False})
groups=[]
for k in tiers:
    rr=[r for r in results if r['K']==k]
    for b in v.BS:
        g=[r for r in rr if r['B']==b]
        groups.append(dict(K=k,B=b,tier_nm=tiers[k],passed=sum(r['passed'] for r in g),complete=sum(r['complete'] for r in g),
            worst_tail_speed=max((r['tail_speed_deg_s'] or 0) for r in g),max_jump=max(max(r['max_jump_nm']) for r in g),
            max_command=max(max(r['peak_command_nm']) for r in g),saturation_fraction=max(r['saturation_fraction'] for r in g)))
    fig,axs=plt.subplots(2,4,figsize=(17,7),layout='constrained')
    for r in rr:
        if r['sign']!=1:continue
        a=np.loadtxt(out/(r['id']+'.csv'),delimiter=',',skiprows=1,ndmin=2);i=r['axis']
        for j,y in enumerate([np.rad2deg(a[:,1+i]-v.REF[i]),np.rad2deg(a[:,3+i]),a[:,5],a[:,6]]):
            axs[i,j].plot(a[:,0],y,lw=.8,label=f'B={r["B"]:g}')
    for i in (0,1):
        for j,title in enumerate(['误差 °','速度 °/s','104目标 N·m','105目标 N·m']):
            ax=axs[i,j];ax.set_title(('ψ' if i==0 else 'β')+'扰动 · '+title);ax.grid(alpha=.2);ax.set_xlim(0,6);ax.axvspan(.4,1,color='gray',alpha=.15)
            if j<2:
                for sign in [-1,1]:ax.axhline(sign*(.5 if j==0 else 1),color='black',ls='--',lw=.5)
            ax.set_xlabel('模型时间 s')
    axs[0,0].legend(ncol=3,fontsize=8);fig.suptitle(f'PH-EXO K={k}，每台{tiers[k]} N·m档｜5ms反馈+5ms提交+MIT量化；提前结束即失败')
    fig.savefig(out/f'K{k}_curves.png',dpi=150);plt.close(fig)
v.save(out/'groups.json',groups)
with (out/'groups.csv').open('w',newline='',encoding='utf-8-sig') as f:
    w=csv.DictWriter(f,fieldnames=groups[0]);w.writeheader();w.writerows(groups)
v.save(out/'hashes.json',{p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in out.iterdir() if p.is_file()})
print(json.dumps(dict(cases=len(results),passed=sum(r['passed'] for r in results),complete=sum(r['complete'] for r in results),groups=groups),ensure_ascii=False),flush=True)
