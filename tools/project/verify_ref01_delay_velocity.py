"""Explicit Windows delay/velocity factorial; no hardware or deployment."""
from pathlib import Path
import sys,json,hashlib,collections
import numpy as np
R=Path(__file__).resolve().parents[2];sys.path.insert(0,str(R/'tools/project'))
import verify_ref01_5ms_damping as v
from sync_orin import safe_path
out=safe_path(R,'tools/bench_panel/data/delay_velocity_20261114');out.mkdir(parents=True,exist_ok=False)
sources=['tools/project/verify_ref01_delay_velocity.py','tools/project/verify_ref01_5ms_damping.py',
'tools/project/replay_ref01_ordinary.cpp','mujoco/ref01/cpp/ref01_model.hpp',
'ros2_ws/src/mech_controllers/include/mech_controllers/ref01_impedance_core.hpp',
'ros2_ws/src/mech_protocol_cubemars/src/ak30_force_wire.cpp','mujoco/ref01/ref01_dynamics_api.py',
'mujoco/ref01/config/mujoco/ref01_dynamics_cad_v1.xml']
hashes={s:hashlib.sha256((R/s).read_bytes()).hexdigest() for s in sources}
v.save(out/'source_hashes.json',hashes);v.compile_probe(out)
tiers={10:4,15:6,20:6,25:8,30:10};results=[]
jobs=[(profile,quant,k,b,a,sign) for profile in ['pipeline5','ideal5'] for quant in [True,False] for k in tiers for b in v.BS for a in (0,1) for sign in (-1,1)]
for n,(profile,quant,k,b,a,sign) in enumerate(jobs,1):
 row=v.run_case(out,None,profile,b,a,sign,K=k,cap=tiers[k]-30/4095,velocity_quantized=quant)
 row['tier_nm']=tiers[k];row['condition']=f'delay{5 if profile=="pipeline5" else 0}_'+('quantized' if quant else 'exact')
 data=np.loadtxt(out/(row['id']+'.csv'),delimiter=',',skiprows=1,ndmin=2)
 velocity=np.loadtxt(out/(row['id']+'_velocity.csv'),delimiter=',',skiprows=1,ndmin=2)
 assert abs(data[:,5:9]).max()<=tiers[k]+1e-10
 delta=velocity[:,1:3]-velocity[:,3:5]
 if quant:
  assert abs(delta).max()<=5*v.ERPM_SCALE+1e-12
 else:assert abs(delta).max()<1e-12
 row['velocity_feedback_error_max_rad_s']=float(abs(delta).max())
 valid=data[:,10]==2;pair=valid[1:]&valid[:-1]
 row['max_damping_jump_nm']=float(abs(np.diff(data[:,21:23],axis=0)[pair]).max(initial=0))
 row['saturation_fraction']=float(np.mean(data[valid,27]<.999999))
 # Work of held motor-space damper over actual shaft displacement, model sample approximation.
 # Pipeline delay is applied to the damper component as to the total command.
 damper=data[:,21:23].copy()
 if profile=='pipeline5':damper=np.vstack((np.zeros((1,2)),damper[:-1]))
 power=np.sum(damper*velocity[:,3:5],axis=1)
 tail=data[:,0]>=2.5
 row['damper_positive_power_tail_fraction']=float(np.mean(power[tail]>1e-9)) if tail.any() else None
 row['damper_tail_work_j_approx']=float(np.sum(power[tail])*.005) if tail.any() else None
 results.append(row);v.save(out/'summary.json',dict(complete=n==len(jobs),cases=results))
 if n%24==0:print(f'{n}/{len(jobs)}, recovered {sum(x["passed"] for x in results)}',flush=True)
v.validate(out,results)
assert all(hashlib.sha256((R/s).read_bytes()).hexdigest()==h for s,h in hashes.items())
v.save(out/'design.json',dict(conditions=['delay5_quantized','delay5_exact','delay0_quantized','delay0_exact'],K=list(tiers),B=v.BS,tiers=tiers,feedback_ms=5,position_quantization_deg=.1,velocity_quantization_ERPM=10,MIT_bits=12,MIT_range_nm=[-15,15],duration_s=6,perturbation_nm=.3,perturbation_s=[.4,1],reference_deg=np.rad2deg(v.REF).tolist(),recovery=dict(after_s=2.5,error_deg=.5,speed_deg_s=1),limits='unchanged from preceding KB matrix; all <=10 Nm',scope='Numerical factor ablation, exact velocity is simulated, not a device capability'))
print('FINISHED '+json.dumps({c:dict(cases=sum(x['condition']==c for x in results),passed=sum(x['passed'] for x in results if x['condition']==c),complete=sum(x['complete'] for x in results if x['condition']==c)) for c in sorted(set(x['condition'] for x in results))}),flush=True)
