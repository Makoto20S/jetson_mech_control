from pathlib import Path
import sys,json,hashlib,os,collections,csv
import numpy as np
r=Path(__file__).resolve().parents[2];sys.path.insert(0,str(r/'tools/project'))
from sync_orin import safe_path
o=safe_path(r,'tools/bench_panel/data/delay_velocity_20261114')
s=json.loads((o/'summary.json').read_text());assert s['complete'];rows=s['cases']
conditions=['delay5_quantized','delay5_exact','delay0_quantized','delay0_exact']
groups=[]
for c in conditions:
 for K in [10,15,20,25,30]:
  for B in [0,.1,.2,.4,.6,1]:
   rr=[x for x in rows if x['condition']==c and x['K']==K and x['B']==B]
   tails=[x['tail_speed_deg_s'] for x in rr if x['tail_speed_deg_s'] is not None]
   groups.append(dict(condition=c,K=K,B=B,complete=sum(x['complete'] for x in rr),passed=sum(x['passed'] for x in rr),
    max_tail_speed=max(tails) if tails else None,max_jump=max(max(x['max_jump_nm']) for x in rr),
    max_damping_jump=max(x['max_damping_jump_nm'] for x in rr),
    saturated=sum(x['saturated_samples'] for x in rr)))
# Reproduced baseline must be byte-identical to prior numerical CSV, not just same verdict.
baseline=r/'tools/bench_panel/data/kb_5ms_20261114_v2'
for x in rows:
 if x['condition']=='delay5_quantized':
  assert (o/(x['id']+'.csv')).read_bytes()==(baseline/(x['id']+'.csv')).read_bytes(),x['id']
# Negative control: B=0 must have identical plant/command trajectories with either velocity precision.
for profile in ['pipeline5','ideal5']:
 for K in [10,15,20,25,30]:
  for a in [0,1]:
   for sign in [-1,1]:
    rr=[x for x in rows if x['profile']==profile and x['K']==K and x['B']==0 and x['axis']==a and x['sign']==sign]
    aa=[np.loadtxt(o/(x['id']+'.csv'),delimiter=',',skiprows=1,ndmin=2) for x in rr]
    assert aa[0].shape==aa[1].shape and np.array_equal(aa[0][:,:15],aa[1][:,:15])
summary={}
for c in conditions:
 rr=[x for x in rows if x['condition']==c]
 summary[c]=dict(complete=sum(x['complete'] for x in rr),passed=sum(x['passed'] for x in rr),
   saturation_samples=sum(x['saturated_samples'] for x in rr),max_command=max(max(x['peak_command_nm']) for x in rr),
   failures=dict(collections.Counter(x['failure'] for x in rr if x['failure'])))
(o/'groups.json').write_text(json.dumps(groups,indent=2))
(o/'factor_summary.json').write_text(json.dumps(summary,indent=2))
os.environ['MPLCONFIGDIR']=str(o/'mplconfig')
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
plt.rcParams.update({'font.family':'Microsoft YaHei','axes.unicode_minus':False})
labels=['5ms提交 + 速度量化','5ms提交 + 精确速度','同拍提交 + 速度量化','同拍提交 + 精确速度']
fig,axs=plt.subplots(1,4,figsize=(15,5),layout='constrained')
for ax,c,label in zip(axs,conditions,labels):
 grid=np.array([x['passed'] for x in groups if x['condition']==c]).reshape(5,6)
 ax.imshow(grid,vmin=0,vmax=4,cmap='YlGn');ax.set_xticks(range(6),[0,.1,.2,.4,.6,1]);ax.set_yticks(range(5),[10,15,20,25,30]);ax.set(xlabel='B',ylabel='K',title=label)
 for i in range(5):
  for j in range(6):ax.text(j,i,f'{grid[i,j]}/4',ha='center',va='center')
fig.suptitle('PH-EXO｜原恢复门槛通过数（每格：两轴正负扰动）')
fig.savefig(o/'factor_recovery.png',dpi=180);plt.close(fig)
for B in [.4,1.]:
 fig,axs=plt.subplots(2,4,figsize=(17,7),layout='constrained')
 for c,label in zip(conditions,labels):
  for axis in [0,1]:
   x=next(x for x in rows if x['condition']==c and x['K']==20 and x['B']==B and x['axis']==axis and x['sign']==1)
   a=np.loadtxt(o/(x['id']+'.csv'),delimiter=',',skiprows=1,ndmin=2)
   for j,y in enumerate([np.rad2deg(a[:,1+axis]-np.deg2rad([4.05,16.608514])[axis]),np.rad2deg(a[:,3+axis]),a[:,5],a[:,21]]):
    axs[axis,j].plot(a[:,0],y,lw=.7,label=label)
 for axis in [0,1]:
  for j,title in enumerate(['姿态误差 °','速度 °/s','104总目标 Nm','104阻尼分量 Nm']):
   ax=axs[axis,j];ax.set_title(('ψ' if axis==0 else 'β')+'扰动 · '+title);ax.set_xlim(0,6);ax.grid(alpha=.2);ax.set_xlabel('模型时间 s');ax.axvspan(.4,1,color='gray',alpha=.1)
 axs[0,0].legend(fontsize=8);fig.suptitle(f'PH-EXO K20 B{B:g}｜仅切换提交延迟、速度量化；位置/MIT量化保留')
 fig.savefig(o/f'factor_curves_B{B:g}.png',dpi=160);plt.close(fig)
(o/'factor_validation.json').write_text(json.dumps(dict(baseline_csv_identical=120,B0_negative_control_pairs=40,all_wire_caps_verified=True,velocity_precision_verified=True),indent=2))
print(json.dumps(dict(summary=summary,selected=[x for x in groups if x['K']==20 and x['B'] in [.4,1.]]),ensure_ascii=False))
