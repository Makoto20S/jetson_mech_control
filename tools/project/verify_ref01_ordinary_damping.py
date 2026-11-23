
from pathlib import Path
import numpy as np,json,hashlib,csv,sys
import argparse
p=argparse.ArgumentParser(description='Independently audit ordinary damping replay and MuJoCo CSV metrics.')
p.add_argument('--output',type=Path,required=True);p.add_argument('--raw',type=Path,required=True);args=p.parse_args()
out=args.output.resolve()
rep=json.loads((out/'replay_summary.json').read_text())
sim=json.loads((out/'simulation_summary.json').read_text())
checks=[];total=0
assert sim['complete'] and len(sim['cases'])==48
ref=np.array(rep['reference_q_rad'])
for r in sim['cases']:
 a=np.genfromtxt(out/'simulation'/(r['id']+'.csv'),delimiter=',',names=True);total+=len(a)
 assert len(a)==r['samples']
 error=np.rad2deg(np.column_stack([a['psi'],a['beta']])-ref)
 speed=np.rad2deg(np.column_stack([a['dpsi'],a['dbeta']]))
 tail=a['t']>=2.5
 good=(np.max(abs(error),axis=1)<=.5)&(np.max(abs(speed),axis=1)<=1)
 passed=bool(r['complete'] and tail.any() and good[tail].all())
 assert passed==r['passed'],r['id']
 if tail.any():
  assert abs(np.max(abs(error[tail]))-r['tail_error_deg'])<1e-9
  assert abs(np.max(abs(speed[tail]))-r['tail_speed_deg_s'])<1e-9
 for mid in (104,105):
  v=a[f'sent{mid}'];assert np.max(abs((v+15)*4095/30-np.rint((v+15)*4095/30)))<1e-9
 checks.append(r['id'])
a=np.genfromtxt(out/'replay_B1.csv',delimiter=',',names=True)
inp=np.loadtxt(out/'input.txt');idx=np.maximum.accumulate(np.where(a['refresh']==1,np.arange(len(a)),0));used=inp[idx]
A=np.column_stack([a[k] for k in ('a00','a01','a10','a11')]).reshape(-1,2,2)
dq=np.column_stack([a['dpsi'],a['dbeta']]);alpha_v=used[:,[6,2]]
velocity_error=float(np.max(abs(np.einsum('nij,nj->ni',A,dq)-alpha_v)));assert velocity_error<1e-8
components=np.genfromtxt(out/'components_B1.csv',delimiter=',',names=True)
D=np.column_stack([components['damper104'],components['damper105']])
power=(D*used[:,[2,6]]).sum(axis=1)
power_error=float(np.max(abs(power+(dq*dq).sum(axis=1))));assert power_error<1e-8
# B=1 motor-target change due to one ERPM code (10 ERPM), frozen pose.
Hi=np.linalg.inv(A.transpose(0,2,1))[:,::-1,:]
gain=-Hi@np.linalg.inv(A)[:,:,::-1]
one_lsb=np.max(abs(gain),axis=(1,2))*10*2*np.pi/(60*21*8)
hashes=json.loads((out/'source_hashes.json').read_text())
assert all(hashlib.sha256(Path(p).read_bytes()).hexdigest()==h for p,h in hashes.items())
raw=args.raw.resolve()
ar=json.loads((raw/'_archive_hashes.json').read_text())
assert all(hashlib.sha256((raw/p).read_bytes()).hexdigest()==h for p,h in ar.items())
result=dict(passed=True,simulation_cases=len(checks),simulation_csv_rows=total,simulation_recovery_pass=sum(r['passed'] for r in sim['cases']),
 replay_cases=6,replay_rows=len(a)*6,velocity_projection_max_error_rad_s=velocity_error,
 instantaneous_damping_power_identity_error_w=power_error,velocity_one_code_max_motor_torque_change_nm=float(one_lsb.max()),
 source_hashes_verified=len(hashes),archived_files_reverified=len(ar),all_checks='metrics independently recomputed from CSV, source hashes unchanged')
(out/'validation.json').write_text(json.dumps(result,indent=2))
print(json.dumps(result,indent=2))
print('B table')
for b in (0,.1,.2,.4,.6,1):
 rows=[r for r in sim['cases'] if r['B']==b]
 print(b,'pass',sum(x['passed'] for x in rows),'complete',sum(x['complete'] for x in rows),
  'nominal',[round(x['tail_speed_deg_s'],3) if x['tail_speed_deg_s'] is not None else x['failure'] for x in rows if x['scenario']=='nominal'])
print('consumed motor speed',rep['largest_jumps'][0]['motor_velocity_before_after_deg_s'])
