"""Independent KKT and frozen-trajectory checks for the callable dynamics API."""
from pathlib import Path
import sys,json
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
import numpy as np
from ref01_dynamics_api import REF01Dynamics

r=REF01Dynamics();rng=np.random.default_rng(20261103)
roundtrip_error=0.;kkt_error=0.;power_error=0.
for _ in range(100):
    q=np.deg2rad(rng.uniform([-44,1],[44,44]));v=rng.uniform(-.8,.8,2);a=rng.uniform(-2,2,2)
    ext=rng.uniform(-.03,.03,2)
    tau=r.inverse_dynamics(q,v,a,external_torque=ext)
    forward=r.forward_dynamics(q,v,tau,external_torque=ext)
    roundtrip_error=max(roundtrip_error,float(np.max(np.abs(forward-a))))
    t=r._terms(q,v);force=np.zeros(8);force[r._active]=tau;force[r._out]+=ext
    block=np.block([[t['full_M'],-t['J'].T],[t['J'],np.zeros((6,6))]])
    solution=np.linalg.solve(block,np.r_[force-t['full_bias'],-t['jdotv']])
    kkt_error=max(kkt_error,float(np.max(np.abs(solution[:8][r._out]-forward))))
    power_error=max(power_error,abs(float(tau@(t['A']@v)-(t['A'].T@tau)@v)))
assert roundtrip_error<1e-9 and kkt_error<1e-9 and power_error<1e-12

trajectory_error=0.;samples=0
for period in (12,6,3):
    data=np.loadtxt(ROOT/f'fixtures/ref01_dynamics_cad_v1/trajectory_{period}s.csv',delimiter=',',skiprows=1)
    for row in data[::10]:
        phase=2*np.pi*row[0]/period;omega=2*np.pi/period
        q=np.deg2rad([15*np.sin(phase),28-8*np.cos(phase)])
        dq=np.deg2rad([15*omega*np.cos(phase),8*omega*np.sin(phase)])
        ddq=np.deg2rad([-15*omega**2*np.sin(phase),8*omega**2*np.cos(phase)])
        tau=r.inverse_dynamics(q,dq,ddq)
        trajectory_error=max(trajectory_error,float(np.max(np.abs(tau-row[3:5]))));samples+=1
assert trajectory_error<2e-7

for q in ([0,0],[45,45],[-45,0]):
    q=np.deg2rad(q);tau=r.inverse_dynamics(q,[.1,-.1],[.2,.1])
    assert np.max(np.abs(r.forward_dynamics(q,[.1,-.1],tau)-[.2,.1]))<1e-9
bad=0
for func,args in [(r.forward_dynamics,([0,2],[0,0],[0,0])),
                  (r.inverse_dynamics,([0,.3],[float('nan'),0],[0,0])),
                  (r.forward_dynamics,([0,.3],[0,0],[1,2,3])),
                  (r.forward_dynamics,([0,.3],[0,0],[float('inf'),0]))]:
    try: func(*args)
    except ValueError:bad+=1
    else:raise AssertionError('Invalid input accepted')
try:r.step([0,0])
except RuntimeError:bad+=1
else:raise AssertionError('Uninitialized simulation accepted')
q=np.deg2rad([10,20]);tau=r.gravity_compensation(q);r.reset(q)
before=r.simulation_state()
r.forward_dynamics(np.deg2rad([20,30]),[0,0],[0,0])
assert np.array_equal(before['q'],r.simulation_state()['q'])
native=r.step(tau,steps=100)
assert native['warnings']==0 and native['max_closure_m']<1e-4 and np.max(np.abs(native['q']-q))<1e-3
example=dict(q=q.tolist(),gravity_tau_Nm=tau.tolist(),
             acceleration_with_compensation=r.forward_dynamics(q,[0,0],tau).tolist())
record=dict(status='PASS',model=r.metadata,random_cases=100,roundtrip_error_rad_s2=roundtrip_error,
            independent_KKT_error_rad_s2=kkt_error,power_mapping_error_W=power_error,
            frozen_trajectory_samples=samples,trajectory_torque_error_Nm=trajectory_error,
            invalid_cases_rejected=bad,native_step_duration_s=native['time_s'],
            native_max_closure_m=native['max_closure_m'],example=example)
out=ROOT/'output/ref01_dynamics_api';out.mkdir(exist_ok=True)
(out/'verification.json').write_text(json.dumps(record,ensure_ascii=False,indent=2),encoding='utf-8')
print(json.dumps(record,ensure_ascii=False,indent=2))
