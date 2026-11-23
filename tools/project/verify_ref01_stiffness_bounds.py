from pathlib import Path
import argparse,csv,hashlib,json,itertools,sys,math
from datetime import datetime
import numpy as np
root=Path(__file__).resolve().parents[2];sys.path.insert(0,str(root/'tools/project'))
from sync_orin import safe_path
sys.path.insert(0,str(root/'mujoco/ref01'))
from ref01_dynamics_api import REF01Dynamics
from fixed_pose_impedance import FixedPoseImpedance
parser=argparse.ArgumentParser(description='Offline independent PH-EXO stiffness capacity audit')
parser.add_argument('--input',required=True,help='Explicit analysis result directory relative to repository')
args=parser.parse_args()
out=safe_path(root,args.input)
s=json.loads((out/'summary.json').read_text(encoding='utf-8'))
with (out/'pose_metrics.csv').open(encoding='utf-8-sig') as f: rows=list(csv.DictReader(f))
E=np.radians(s['errors_deg']);api=REF01Dynamics()
direct_error=0.;direct_count=0;boundary_peak=0.;above_peak=0.
# Independent scalar closed-form row audit, not the sweep's matrix reduction.
for r in rows:
    k=float(r['k']);g=[float(r['gravity104_nm']),float(r['gravity105_nm'])]
    K=(10-max(abs(x) for x in g))/(E[0]/2+E[1]/(2*k))
    assert abs(K-float(r['equal_K_10Nm_B0']))<1e-10
# Exercise the existing controller over every sign corner at independent poses.
for index in np.linspace(0,len(rows)-1,80,dtype=int):
    r=rows[index];q=np.radians([float(r['psi_deg']),float(r['beta_deg'])])
    K=np.array([20.,30.]);B=np.array([.2,.4]);V=np.radians([60.,30.])
    expected=np.abs([float(r['gravity104_nm']),float(r['gravity105_nm'])])+np.array(
        [[abs(float(r['h104_psi'])),abs(float(r['h104_beta']))],
         [abs(float(r['h105_psi'])),abs(float(r['h105_beta']))]])@(K*E+B*V)
    actual=np.zeros(2)
    for es,vs in itertools.product(itertools.product([-1,1],repeat=2),repeat=2):
        control=FixedPoseImpedance(api,q+np.array(es)*E,K,B,[1000,1000])
        tau=control.command(q,np.array(vs)*V).motor104_105_nm
        actual=np.maximum(actual,abs(tau));direct_count+=1
    direct_error=max(direct_error,float(max(abs(actual-expected))))
assert direct_error<1e-10
# Refine the sampled limiting beta; endpoints in psi are retained.
refined=(float('inf'),None)
for psi in [-45.,45.]:
    for beta in np.arange(17.,19.00001,.002):
        t=api.dynamics_terms(np.radians([psi,beta]));H=np.linalg.inv(t['A'].T)[::-1]
        g=H@t['bias'];value=float(min((10-abs(g))/(np.abs(H)@E)))
        if value<refined[0]: refined=(value,[psi,float(beta)])
assert abs(refined[0]-s['equal_K_static_boundary'])<.001
# Verify reported data/source hashes, and tier boundaries at all sampled poses.
manifest=json.loads((out/'artifacts_sha256.json').read_text(encoding='utf-8'))
for name,value in manifest.items():
    assert hashlib.sha256((out/name).read_bytes()).hexdigest()==value,name
for name,value in s['source_hashes'].items():
    assert hashlib.sha256((root/name).read_bytes()).hexdigest()==value,name
for tier in s['tiers']:
    K=tier['equal_K_static_boundary'];cap=tier['motor_cap_nm']
    high=max(max(abs(float(r['gravity104_nm'])),abs(float(r['gravity105_nm'])))+
        K*(E[0]/2+E[1]/(2*float(r['k']))) for r in rows)
    assert abs(high-cap)<1e-10,(cap,high)
# Conservative one-LSB budget, computed from the inspected production codec.
lsb=30/4095
quant_tiers=[]
for tier in s['tiers']:
    cap=tier['motor_cap_nm']
    K=min((cap-lsb-max(abs(float(r['gravity104_nm'])),abs(float(r['gravity105_nm']))))/
          (E[0]/2+E[1]/(2*float(r['k']))) for r in rows)
    quant_tiers.append(dict(motor_cap_nm=cap,target_budget_nm=cap-lsb,equal_K_boundary=K))
codec_paths=['ros2_ws/src/mech_protocol_cubemars/src/ak30_force_wire.cpp',
             'ros2_ws/src/mech_protocol_cubemars/include/mech_protocol_cubemars/ak30_force_wire.hpp']
codec_hashes={name:hashlib.sha256((root/name).read_bytes()).hexdigest() for name in codec_paths}
audit=dict(status='PASS',created_at=datetime.now().astimezone().isoformat(),
  rows_independently_recomputed=len(rows),existing_controller_corner_calls=direct_count,
  controller_max_difference_nm=direct_error,verified_artifacts=len(manifest),
  verified_sources=len(s['source_hashes']),tier_boundary_checks=len(s['tiers']),
  local_refinement=dict(beta_step_deg=.002,beta_range_deg=[17,19],psi_deg=[-45,45],
      equal_K_boundary=refined[0],pose_deg=refined[1],
      difference_from_grid=refined[0]-s['equal_K_static_boundary']),
  quantization=dict(range_nm=[-15,15],bits=12,step_nm=lsb,
      negative_10_encoded_nm=-15+math.floor((-10+15)*4095/30)*30/4095,
      positive_10_encoded_nm=-15+math.floor((10+15)*4095/30)*30/4095,
      conservative_tiers=quant_tiers,codec_source_hashes=codec_hashes,
      evidence='Scalar calculation from inspected codec, not a hardware torque measurement'),
  scope='Offline ideal-model capacity audit; refined local grid, not a continuous workspace proof')
(out/'independent_audit.json').write_text(json.dumps(audit,ensure_ascii=False,indent=2),encoding='utf-8')
print(json.dumps(audit,ensure_ascii=False))
print('selected damping sensitivity:',json.dumps([x for x in s['damping_speed_sensitivity'] if x['B_each'] in [.2,.4,1.] and x['speed_bound_each_deg_s']==60],ensure_ascii=False))
print('candidate gains:',json.dumps(s['candidate_gains'],ensure_ascii=False))
