
"""PH-EXO ordinary impedance: recorded-feedback B sweep, no hardware access."""
import argparse,csv,hashlib,json,os,shutil,subprocess,sys
from pathlib import Path
import numpy as np
import mujoco
ROOT=Path(__file__).resolve().parents[2]
BS=(0.,.1,.2,.4,.6,1.)
COLS=['time','mode','refresh','reason','tau104','tau105','psi','beta','dpsi','dbeta','spring_psi','spring_beta','damping_psi','damping_beta','gravity_psi','gravity_beta','scale','a00','a01','a10','a11','raw104','raw105']
def compile_probe(out):
 sdk=Path(mujoco.__file__).parent
 cmd=[str(ROOT/'.local/toolchains/zig-0.13.0/ziglang/zig.exe'),'c++','-std=c++17','-O2','-Wall','-Wextra','-Werror']
 for inc in ['mujoco/ref01/cpp','ros2_ws/src/mech_controllers/include/mech_controllers','ros2_ws/src/mech_protocol_cubemars/include']:
  cmd+=['-I'+str(ROOT/inc)]
 cmd+=['-I'+str(sdk/'include'),str(ROOT/'tools/project/replay_ref01_ordinary.cpp'),str(ROOT/'ros2_ws/src/mech_protocol_cubemars/src/ak30_force_wire.cpp'),str(sdk/'mujoco.dll'),'-o',str(out/'replay.exe')]
 env=dict(os.environ,ZIG_GLOBAL_CACHE_DIR=str(ROOT/'.local/cache/zig'),ZIG_LOCAL_CACHE_DIR=str(ROOT/'.local/cache/zig/local'))
 run=subprocess.run(cmd,capture_output=True,text=True,env=env,timeout=180)
 (out/'compile.log').write_text(run.stdout+run.stderr,encoding='utf-8')
 (out/'compile_command.json').write_text(json.dumps(cmd,indent=2))
 if run.returncode:raise RuntimeError(run.stderr)
 shutil.copy2(sdk/'mujoco.dll',out/'mujoco.dll')
def write_csv(path,cols,rows):
 with path.open('w',newline='') as f:
  w=csv.writer(f);w.writerow(cols);w.writerows(rows)
def analyze(raw,out):
 d=json.loads((raw/'result.json').read_text());t0=d['events']['first_plan_s']
 samples=[[v for v in d['samples'] if v['id']==mid] for mid in (104,105)]
 sources=[np.array([v['mit_rx_us']*1e-6-t0+1 for v in s]) for s in samples]
 start=max(1.,max(s[np.flatnonzero(s>.99)[0]] for s in sources))+.001
 times=np.arange(start,30.95,.005)
 inputs=[]
 for now in times:
  row=[now]
  for i in range(2):
   j=int(np.searchsorted(sources[i],now,side='right')-1);assert j>=0
   v=samples[i][j];row +=[v['position'],v['velocity'],sources[i][j],v['sequence']]
  inputs.append(row)
 text='\n'.join(' '.join(map(str,v)) for v in inputs)+'\n'
 (out/'input.txt').write_text(text)
 all_rows={};summary=[]
 for b in BS:
  run=subprocess.run([str(out/'replay.exe'),str(ROOT/'mujoco/ref01/config/mujoco/ref01_dynamics_cad_v1.xml'),*map(str,d['reference_q_rad']),str(b)],input=text,capture_output=True,text=True,timeout=90)
  assert run.returncode==0,run.stderr
  rows=[line.split() for line in run.stdout.splitlines()]
  assert len(rows)==len(times) and all(v[1]=='2' and v[3]=='none' for v in rows)
  a=np.array([[float(x) if j!=3 else 0 for j,x in enumerate(row)] for row in rows]);all_rows[b]=a
  write_csv(out/f'replay_B{b:g}.csv',COLS,rows)
  mask=(a[1:,0]-1>=6)&(a[1:,0]-1<29.5);delta=np.diff(a[:,4:6],axis=0)[mask]
  summary.append(dict(B=b,rows=len(rows),refresh_count=int(a[:,2].sum()),peak_torque=np.max(abs(a[:,4:6]),axis=0).tolist(),
   jump_max=np.max(abs(delta),axis=0).tolist(),jump_p99=np.percentile(abs(delta),99,axis=0).tolist(),
   jumps_ge_0p1=np.sum(abs(delta)>=.1,axis=0).tolist(),saturated=int(np.sum(a[:,16]<1))))
 a=all_rows[1.]; consumed=np.array(inputs)[np.maximum.accumulate(np.where(a[:,2]==1,np.arange(len(a)),0))];H=np.linalg.inv(a[:,17:21].reshape(-1,2,2).transpose(0,2,1))[:,::-1,:]
 spring=np.einsum('nij,nj->ni',H,a[:,10:12]);damper=np.einsum('nij,nj->ni',H,a[:,12:14]);gravity=np.einsum('nij,nj->ni',H,a[:,14:16])
 rawsum=spring+damper+gravity
 err=float(np.max(abs(rawsum*a[:,16,None]-a[:,21:23])));assert err<1e-10
 for b in BS:
  aa=all_rows[b]
  demand=spring+b*damper+gravity
  scale=np.minimum(1,5/np.maximum(abs(demand).max(axis=1),1e-30))
  assert np.max(abs(demand*scale[:,None]-aa[:,21:23]))<1e-9
  # Independent ABI check: interval coding is floor to 12 bits, per production codec.
  expect=np.floor((aa[:,21:23]+15)*4095/30)*30/4095-15
  assert np.max(abs(expect-aa[:,4:6]))<1e-10
 write_csv(out/'components_B1.csv',['t','spring104','spring105','damper104','damper105','gravity104','gravity105','raw104','raw105','wire104','wire105'],
  np.column_stack((a[:,0]-1,spring,damper,gravity,rawsum,a[:,4:6])))
 delta=np.diff(a[:,4:6],axis=0);valid=(a[1:,0]-1>=6)&(a[1:,0]-1<29.5)
 jumps=[]
 for axis,mid in enumerate((104,105)):
  idx=np.flatnonzero(valid)[np.argmax(abs(delta[valid,axis]))]+1
  velpart=-np.einsum('ij,j->i',(H[idx]+H[idx-1])/2,a[idx,8:10]-a[idx-1,8:10])
  geompart=-np.einsum('ij,j->i',H[idx]-H[idx-1],(a[idx,8:10]+a[idx-1,8:10])/2)
  comp=np.array([spring[idx,axis]-spring[idx-1,axis],damper[idx,axis]-damper[idx-1,axis],gravity[idx,axis]-gravity[idx-1,axis]])
  jumps.append(dict(motor=mid,t=float(a[idx,0]-1),target_before_after=a[idx-1:idx+1,4+axis].tolist(),delta=float(delta[idx-1,axis]),
   spring_delta=float(comp[0]),damper_delta=float(comp[1]),gravity_delta=float(comp[2]),quantization_delta=float(delta[idx-1,axis]-comp.sum()),
   damper_velocity_delta=float(velpart[axis]),damper_geometry_delta=float(geompart[axis]),q_before_after_deg=np.rad2deg(a[idx-1:idx+1,6:8]).tolist(),
   dq_before_after_deg_s=np.rad2deg(a[idx-1:idx+1,8:10]).tolist(),
   motor_velocity_before_after_deg_s=np.rad2deg(consumed[idx-1:idx+1][:,[2,6]]).tolist()))
 # Match each reconstructed new-pair target to measured TX within 12 ms; not a cycle-perfect replay.
 match=[]
 for axis,(mid,bus) in enumerate(((104,2),(105,1))):
  trace=[json.loads(x) for x in (raw/f'bus{bus}_motor{mid}.jsonl').read_text().splitlines()]
  tx=[v for v in trace if v.get('stage')=='transport_result' and v.get('id')==0x800+mid]
  tt=np.array([v['ns']*1e-9-t0 for v in tx]);tau=np.array([int(v['hex'][-3:],16)*30/4095-15 for v in tx])
  errors=[]
  for i in np.flatnonzero((a[:,2]==1)&(a[:,0]>1.5)&(a[:,0]<30.5)):
   sel=abs(tt-(a[i,0]-1))<=.012
   if sel.any():errors.append(float(abs(tau[sel]-a[i,4+axis]).min()))
  match.append(dict(motor=mid,comparisons=len(errors),exact_within_12ms=int(np.sum(np.array(errors)<1e-8)),maximum_best_error=max(errors)))
 # Encoder differences provide an independent kinematic consistency diagnostic, not speed truth.
 velocity=[]
 for mid,s in zip((104,105),samples):
  t=np.array([x['mit_rx_us']*1e-6-t0 for x in s]);p=np.array([x['mit_position_deg'] for x in s]);v=np.rad2deg([x['velocity'] for x in s])
  fd=np.diff(p)/np.diff(t);average=(v[1:]+v[:-1])/2
  use=(t[1:]>=6)&(t[1:]<29.5)
  velocity.append(dict(motor=mid,erpm_step_rad_s=10*2*np.pi/(60*21*8),erpm_step_deg_s=10*360/(60*21*8),
   peak_speed_deg_s=float(max(abs(v))),max_sample_velocity_change_deg_s=float(max(abs(np.diff(v)[use]))),
   midpoint_fd_rmse_deg_s=float(np.sqrt(np.mean((fd[use]-average[use])**2))),midpoint_fd_correlation=float(np.corrcoef(fd[use],average[use])[0,1])))
 report=dict(K=[10,10],B_sweep=summary,component_sum_max_error=err,largest_jumps=jumps,replay_match=match,velocity=velocity,
  boundary='Recorded-feedback counterfactual; identical trajectory for B comparisons, no closed-loop recovery implication. Host grid cannot reproduce exact ROS scheduling.',
  reference_q_rad=d['reference_q_rad'],feedback_period_s=.02,command_period_s=.005)
 (out/'replay_summary.json').write_text(json.dumps(report,indent=2))
 hashes={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in [raw/'result.json',raw/'bus1_motor105.jsonl',raw/'bus2_motor104.jsonl',Path(__file__),ROOT/'tools/project/replay_ref01_ordinary.cpp',ROOT/'ros2_ws/src/mech_controllers/include/mech_controllers/ref01_impedance_core.hpp',ROOT/'mujoco/ref01/cpp/ref01_model.hpp',ROOT/'mujoco/ref01/config/mujoco/ref01_dynamics_cad_v1.xml',ROOT/'tools/project/verify_ref01_robustness.py',ROOT/'tools/project/ref01_robustness_probe.cpp',ROOT/'ros2_ws/src/mech_protocol_cubemars/src/ak30_force_wire.cpp']}
 (out/'source_hashes.json').write_text(json.dumps(hashes,indent=2))
 print(json.dumps(report,indent=2),flush=True)
def simulate(raw,out):
 import verify_ref01_robustness as vr
 sim=out/'simulation';sim.mkdir()
 vr.compile_probe(sim)
 reference=json.loads((raw/'result.json').read_text())['reference_q_rad']
 specs=[s for s in vr.SCENARIOS if s[0] in ('nominal','delay_10','body_I_x4','rotor_005')]
 results=[]
 for spec in specs:
  for b in BS:
   for axis in (0,1):
    result=vr.run_case(sim,spec,(f'ordinary_B{b:g}','ordinary',b),axis,1,np.array(reference),6.,'damping')
    result['B']=b;results.append(result)
    (out/'simulation_summary.json').write_text(json.dumps(dict(cases=results,complete=len(results)==48,expected_cases=48),indent=2))
    print(json.dumps({k:result[k] for k in ('id','passed','failure','tail_speed_deg_s')}),flush=True)
def plot(out):
 os.environ['MPLCONFIGDIR']=str(out/'mplconfig')
 import matplotlib
 matplotlib.use('Agg')
 import matplotlib.pyplot as plt
 plt.rcParams.update({'font.family':'DejaVu Sans','font.size':10})
 data=np.genfromtxt(out/'components_B1.csv',delimiter=',',names=True)
 fig,axes=plt.subplots(3,1,figsize=(11,9),sharex=True)
 a=np.genfromtxt(out/'replay_B1.csv',delimiter=',',names=True)
 mask=(data['t']>=14.8)&(data['t']<=15.6)
 for axis,mid in enumerate((104,105)):
  ax=axes[axis]
  for term in ('spring','damper','gravity','wire'):ax.step(data['t'][mask],data[term+str(mid)][mask],where='post',label=term)
  ax.set_ylabel(f'Motor {mid} [Nm]');ax.legend(ncol=4);ax.grid(alpha=.25)
 for name in ('dpsi','dbeta'):axes[2].step(data['t'][mask],np.rad2deg(a[name][mask]),where='post',label=name)
 axes[2].set_ylabel('Task velocity [deg/s]');axes[2].set_xlabel('Time after first plan [s]');axes[2].legend();axes[2].grid(alpha=.25)
 fig.suptitle('PH-EXO | Ordinary impedance B=1 | Recorded-feedback reconstruction')
 fig.tight_layout();fig.savefig(out/'jump_components.png',dpi=170);fig.savefig(out/'jump_components.svg');plt.close(fig)
 fig,axes=plt.subplots(2,1,figsize=(11,7),sharex=True)
 for b in BS:
  a=np.genfromtxt(out/f'replay_B{b:g}.csv',delimiter=',',names=True)
  mask=(a['time']-1>=14.8)&(a['time']-1<=15.6)
  for axis,mid in enumerate((104,105)):axes[axis].step(a['time'][mask]-1,a[f'tau{mid}'][mask],where='post',label=f'B={b:g}')
 for axis,mid in enumerate((104,105)):
  axes[axis].set_ylabel(f'Motor {mid} [Nm]');axes[axis].legend(ncol=6);axes[axis].grid(alpha=.25)
 axes[-1].set_xlabel('Time after first plan [s]');fig.suptitle('Same measured trajectory, K=10 | B counterfactual (not a new closed loop)')
 fig.tight_layout();fig.savefig(out/'B_torque_comparison.png',dpi=170);plt.close(fig)
 if (out/'simulation_summary.json').exists():
  summary=json.loads((out/'simulation_summary.json').read_text())
  fig,axes=plt.subplots(2,2,figsize=(12,8),sharex=True)
  ref=np.rad2deg(json.loads((out/'replay_summary.json').read_text())['reference_q_rad'])
  for b in BS:
   for axis in (0,1):
    r=next(r for r in summary['cases'] if r['scenario']=='nominal' and r['B']==b and r['axis']==axis)
    a=np.genfromtxt(out/'simulation'/(r['id']+'.csv'),delimiter=',',names=True)
    axes[axis,0].plot(a['t'],np.rad2deg(a['psi' if axis==0 else 'beta'])-ref[axis],label=f'B={b:g}')
    axes[axis,1].plot(a['t'],a['sent104' if axis==0 else 'sent105'],label=f'B={b:g}')
  for row in range(2):
   axes[row,0].set_ylabel(('psi' if row==0 else 'beta')+' error [deg]');axes[row,1].set_ylabel(('104' if row==0 else '105')+' torque [Nm]')
   for col in range(2):axes[row,col].grid(alpha=.25);axes[row,col].legend(ncol=3)
  axes[1,0].set_xlabel('Model time [s]');axes[1,1].set_xlabel('Model time [s]')
  fig.suptitle('MuJoCo nominal plant | 20 ms feedback, MIT quantization, K=10 | 0.3 Nm pulse 0.4-1.0 s')
  fig.tight_layout();fig.savefig(out/'nominal_recovery.png',dpi=170);plt.close(fig)
def main():
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('--raw',type=Path,required=True);p.add_argument('--output',type=Path,required=True);p.add_argument('--plot-only',action='store_true');p.add_argument('--replay-only',action='store_true');a=p.parse_args()
 if a.plot_only:plot(a.output);return
 if a.replay_only:analyze(a.raw,a.output);plot(a.output);return
 a.output.mkdir(parents=True,exist_ok=False);compile_probe(a.output);analyze(a.raw,a.output);simulate(a.raw,a.output);plot(a.output)
if __name__=='__main__':main()
