
import argparse,csv,hashlib,json,collections,os
from pathlib import Path
import numpy as np
p=argparse.ArgumentParser();p.add_argument('--base',type=Path,required=True);p.add_argument('--output',type=Path,required=True);p.add_argument('--root',type=Path,required=True);a=p.parse_args()
out=a.output;out.mkdir(parents=True,exist_ok=True)
sets={name:a.base/f'feedback_timing_20261111_{name}' for name in ('grid','long')}
all_results={};audit=[]
for name,directory in sets.items():
 s=json.loads((directory/'summary.json').read_text());assert s['complete']
 results=s['cases'];all_results[name]=results
 for r in results:
  f=directory/(r['id']+'.csv');x=np.genfromtxt(f,delimiter=',',names=True)
  e=list(csv.DictReader((directory/(r['id']+'_events.csv')).open()));at=collections.defaultdict(list)
  latest=[0,0];hist=[[0],[0]]
  for event in e:at[int(event['arrival_tick'])].append({k:int(v) for k,v in event.items()})
  for tick,row in enumerate(x):
   assert abs(row['t']-tick*.005)<1e-8
   for ev in at[tick]:
    ax=ev['motor_index'];src=ev['source_tick']
    assert src<=ev['scheduled_arrival_tick']==tick
    assert ev['accepted']==int(src>latest[ax])
    if ev['accepted']:latest[ax]=src
   for ax in (0,1):
    if latest[ax]!=hist[ax][-1]:hist[ax].append(latest[ax]);hist[ax]=hist[ax][-16:]
   assert [round(row['source104']/.005),round(row['source105']/.005)]==latest
   if row['fault']:continue
   common=min(latest);assert round(row['aligned_source']/.005)==common
   assert row['age_ms']==(tick-common)*5
   for ax,motor in enumerate((104,105)):
    assert row[f'age{motor}_ms']==(tick-latest[ax])*5<60
    if common in hist[ax]:width=0
    else:
     left=max(v for v in hist[ax] if v<common);right=min(v for v in hist[ax] if v>common)
     width=(right-left)*5
    assert row[f'bracket{motor}_ms']==width
  err=np.abs(np.rad2deg(np.column_stack((x['psi'],x['beta'])))-r['reference_deg'])
  vel=np.abs(np.rad2deg(np.column_stack((x['dpsi'],x['dbeta']))))
  tail=x['t']>=2.5
  complete=not r['failure'] and len(x)==round(r['duration']/.005)
  passed=bool(complete and tail.any() and (err[tail]<=.5).all() and (vel[tail]<=1).all())
  assert passed==r['passed'] and complete==r['complete']
  if tail.any():
   assert abs(err[tail].max()-r['tail_error_deg'])<1e-10
   assert abs(vel[tail].max()-r['tail_speed_deg_s'])<1e-10
  audit.append(dict(set=name,id=r['id'],rows=len(x),csv_sha256=hashlib.sha256(f.read_bytes()).hexdigest(),
   events_sha256=hashlib.sha256((directory/(r['id']+'_events.csv')).read_bytes()).hexdigest()))
 aggregate={}
 for r in results:
  k=r['plant']+'/'+r['timing'];aggregate.setdefault(k,collections.Counter())[r['category']]+=1
 print(name,'TOTAL',dict(collections.Counter(r['category'] for r in results)))
 for k,v in aggregate.items():print(k,dict(v))
 good=[r for r in results if r['passed']]
 print('worst pass speed',max(good,key=lambda r:r['tail_speed_deg_s']))
 print('failures',dict(collections.Counter(r['failure'] for r in results if r['failure'])))
 (out/(name+'_aggregate.json')).write_text(json.dumps(aggregate,indent=2))
(out/'audit.json').write_text(json.dumps(dict(passed=True,cases=len(audit),samples=sum(r['rows'] for r in audit),files=audit),indent=2))
print('AUDIT',len(audit),'cases',sum(r['rows'] for r in audit),'rows')
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
plt.rcParams.update({'font.size':9,'axes.grid':True,'grid.alpha':.2})
grid=all_results['grid'];plants=list(dict.fromkeys(r['plant'] for r in grid));timings=list(dict.fromkeys(r['timing'] for r in grid))
matrix=np.zeros((len(timings),len(plants)));labels=[]
for i,t in enumerate(timings):
 lab=[]
 for j,pn in enumerate(plants):
  rr=[r for r in grid if r['plant']==pn and r['timing']==t];n=sum(r['passed'] for r in rr);matrix[i,j]=n/len(rr);lab.append(f'{n}/{len(rr)}')
 labels.append(lab)
fig,ax=plt.subplots(figsize=(10,10));im=ax.imshow(matrix,vmin=0,vmax=1,cmap='RdYlGn',aspect='auto')
ax.set_xticks(range(len(plants)),['Nominal','Body I x 0.5','Armature 0.020','Combined']);ax.set_yticks(range(len(timings)),timings);ax.grid(False)
for i in range(len(timings)):
 for j in range(len(plants)):ax.text(j,i,labels[i][j],ha='center',va='center')
ax.set_title('Frozen predictive impedance: strict recovery pass / total (6 s)\n0.5 deg and 1 deg/s, every sample from t=2.5 s; failures include stops')
fig.colorbar(im,ax=ax,label='Recovery pass fraction');fig.tight_layout();fig.savefig(out/'timing_matrix.png',dpi=160);plt.close(fig)
chosen=[('nominal','shared_20_j10'),('nominal','phase10_0'),('combined_rotor','phase10_20_j10'),('combined_rotor','phase10_40')]
fig,axs=plt.subplots(3,4,figsize=(16,9),sharex='col')
for j,(pn,tn) in enumerate(chosen):
 r=next(r for r in grid if r['plant']==pn and r['timing']==tn and r['axis']==0 and r['sign']==1 and r['seed']==11)
 x=np.genfromtxt(sets['grid']/(r['id']+'.csv'),delimiter=',',names=True);t=x['t']
 for field,ref,label in [('psi',0,'psi'),('beta',20,'beta')]:axs[0,j].plot(t,np.rad2deg(x[field])-ref,label=label)
 axs[0,j].axhspan(-.5,.5,color='green',alpha=.1);axs[0,j].set_title(pn+'\n'+tn+' / '+r['category'])
 for f in ('dpsi','dbeta'):axs[1,j].plot(t,np.rad2deg(x[f]),label=f)
 axs[1,j].axhspan(-1,1,color='green',alpha=.1)
 for m in (104,105):
  axs[2,j].plot(t,x['sent'+str(m)],label=str(m))
  axs[2,j].plot(t,x['desired'+str(m)],ls=':',alpha=.5)
 for ax in axs[:,j]:
  ax.axvspan(.4,1.,color='orange',alpha=.12);ax.axvline(2.5,color='gray',ls='--')
  if r['failure']:ax.axvline(r['end_s'],color='red',ls='--')
  ax.legend(fontsize=7)
 axs[2,j].set_xlabel('Model time (s)')
axs[0,0].set_ylabel('Pose error (deg)');axs[1,0].set_ylabel('Speed (deg/s)');axs[2,0].set_ylabel('Motor torque (N m)')
fig.suptitle('Representative psi disturbances; solid = encoded torque, dotted = requested; red = experiment stopped')
fig.tight_layout();fig.savefig(out/'timing_curves.png',dpi=150);plt.close(fig)
r=max([r for r in all_results['long'] if r['passed']],key=lambda r:r['tail_speed_deg_s'])
x=np.genfromtxt(sets['long']/(r['id']+'.csv'),delimiter=',',names=True)
fig,axs=plt.subplots(4,1,figsize=(12,10),sharex=True);t=x['t']
for f,ref in [('psi',0),('beta',20)]:axs[0].plot(t,np.rad2deg(x[f])-ref,label=f)
for f in ('dpsi','dbeta'):axs[1].plot(t,np.rad2deg(x[f]),label=f)
for m in (104,105):axs[2].plot(t,x['sent'+str(m)],label=str(m))
for f in ('age104_ms','age105_ms'):axs[3].plot(t,x[f],label=f,alpha=.7)
axs[0].axhspan(-.5,.5,color='green',alpha=.1);axs[1].axhspan(-1,1,color='green',alpha=.1)
for ax,label in zip(axs,['Error (deg)','Speed (deg/s)','Torque (N m)','Original age (ms)']):
 ax.set_ylabel(label);ax.legend();ax.axvline(2.5,color='gray',ls='--')
axs[-1].set_xlabel('Model time (s)');fig.suptitle('Worst passing 30 s case: '+r['id'])
fig.tight_layout();fig.savefig(out/'timing_long.png',dpi=150);plt.close(fig)

root=a.root;base=a.base;d=sets['long'];cases=json.loads((d/'summary.json').read_text())['cases']
fig,axs=plt.subplots(4,2,figsize=(14,10),sharex='col')
for j,timing in enumerate(('shared_20_j10','independent_20_j10')):
 r=next(r for r in cases if r['plant']=='nominal' and r['timing']==timing and r['axis']==0 and r['seed']==47)
 x=np.genfromtxt(d/(r['id']+'.csv'),delimiter=',',names=True)
 x=x[x['t']>=2.5]
 for f,ref in [('psi',0),('beta',20)]:axs[0,j].plot(x['t'],np.rad2deg(x[f])-ref,label=f)
 for f in ('dpsi','dbeta'):axs[1,j].plot(x['t'],np.rad2deg(x[f]),label=f)
 for m in (104,105):axs[2,j].plot(x['t'],x['sent'+str(m)],label=str(m))
 axs[3,j].plot(x['t'],np.maximum(x['bracket104_ms'],x['bracket105_ms']),label='Interpolation span')
 axs[0,j].axhspan(-.5,.5,color='green',alpha=.1);axs[1,j].axhspan(-1,1,color='green',alpha=.1)
 axs[0,j].set_title(timing+' / '+r['category'])
 for ax in axs[:,j]:ax.axvline(2.5,color='gray',ls='--');ax.legend(fontsize=8);ax.grid(alpha=.2)
 axs[3,j].set_xlabel('Model time (s)')
for ax,label in zip(axs[:,0],['Error (deg)','Speed (deg/s)','Encoded torque (N m)','Span (ms)']):ax.set_ylabel(label)
fig.suptitle('Recovery window t >= 2.5 s: same nominal plant / psi disturbance / seed 47')
fig.tight_layout();fig.savefig(out/'timing_late_failure.png',dpi=150);plt.close(fig)
prior=json.loads((base/'inertia_20261111_final_grid/source_hashes.json').read_text())
now=json.loads((base/'feedback_timing_20261111_grid/source_hashes.json').read_text())
same={k:v for k,v in now.items() if k in prior}
assert same and all(prior[k]==v for k,v in same.items())
assert all(hashlib.sha256((root/k).read_bytes()).hexdigest()==v for k,v in now.items())
(out/'frozen_sources.json').write_text(json.dumps(dict(prior_run='inertia_20261111_final_grid',unchanged=same,current_hashes=now),indent=2))
print('Frozen overlapping source hashes',list(same))
for name in ('grid','long'):
 rr=json.loads((base/f'feedback_timing_20261111_{name}/summary.json').read_text())['cases']
 print(name,'max pass torque',max(max(r['peak_motor_torque_nm']) for r in rr if r['passed']))
 print(name,'pre-perturbation aborts',[(r['id'],r['failure'],r['end_s']) for r in rr if r['category']=='abort' and r['end_s']<.4])
print('plots complete')
