"""Dynamic-reference software test: Windows native probe, Orin isolated ROS/MuJoCo."""
import argparse
import csv
import json
import os
from pathlib import Path
import sys
import numpy as np
from ref01_dynamic_validation import reference,assess_trace,HOME,DURATION,K,B,CAP

ROOT=Path(__file__).resolve().parents[2]

def windows(output,probe_path):
    sys.path.insert(0,str(ROOT/'mujoco/ref01'))
    from ref01_dynamics_api import REF01Dynamics
    from test_ref01_native import Probe
    plant=REF01Dynamics()
    probe=Probe(probe_path,'mujoco/ref01/config/mujoco/ref01_dynamics_cad_v1.xml',ROOT)
    results=[]
    max_oracle=0.
    try:
        for axis,name in enumerate(('psi','beta')):
            state=plant.reset(HOME);rows=[]
            for tick in range(round(DURATION/.005)):
                t=tick*.005
                # 20ms publisher, 5ms controller/feedback, held atomic q,dq.
                qref,vref=reference((tick//4)*.02,axis)
                result=probe.evaluate([*state['alpha'],*state['alpha_velocity']],[*qref,*vref])
                if result is None:raise RuntimeError('native probe rejected '+name)
                q=np.array(result['q']);dq=np.array(result['dq'])
                A=np.array(result['A'])
                expected=np.linalg.solve(A.T,np.array(result['gravity'])+K*(qref-q)+B*(vref-dq))
                expected/=max(1.,max(abs(expected)/CAP))
                max_oracle=max(max_oracle,float(np.max(abs(expected-result['torque']))))
                motor=np.array(result['torque'])[::-1]
                rows.append([t,*state['q'],*state['dq'],*qref,*vref,*motor,state['max_closure_m']])
                state=plant.step(result['torque'],steps=10)
            with (output/(name+'.csv')).open('w',newline='',encoding='utf-8') as f:
                w=csv.writer(f);w.writerow(['t','psi','beta','dpsi','dbeta','qref_psi','qref_beta','vref_psi','vref_beta','tau104','tau105','closure']);w.writerows(rows)
            report=assess_trace(rows)
            report.update(case=name,samples=len(rows),warnings=state['warnings'])
            report['passed']=report['passed'] and state['warnings']==0
            results.append(report)
    finally:probe.close()
    return dict(passed=all(r['passed'] for r in results) and max_oracle<1e-10,
                cases=results,max_torque_oracle_error=max_oracle,evidence_kind='Windows_native_MuJoCo',
                hardware_verified=False,performance_verified=False)

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output',required=True,type=Path)
    p.add_argument('--probe',type=Path)
    p.add_argument('--ros',action='store_true')
    p.add_argument('--cases',nargs='+',default=['psi','beta','drop','replay','nan','gain','session','stop'])
    a=p.parse_args()
    out=a.output.resolve()
    if not out.is_relative_to(ROOT/'.local/validation'):raise ValueError('project validation output required')
    if any(x.is_symlink() for x in (out,*out.parents)):raise ValueError('symlink output')
    if a.ros:
        if sys.platform!='linux' or os.environ.get('EXO_SYNC_LOCK_HELD')!='1' or os.environ.get('ROS_DOMAIN_ID')!='179' or os.environ.get('ROS_LOCALHOST_ONLY')!='1':
            raise RuntimeError('Orin project lock and isolated localhost domain 179 required')
        if (ROOT/'.local/sync/incomplete.json').exists():raise RuntimeError('incomplete sync')
    elif not a.probe:raise ValueError('--probe required for native test')
    out.mkdir(parents=True,exist_ok=False)
    if a.ros:
        from test_ref01_manager import run_case
        results=[]
        for case in a.cases:
            r=run_case(out/case,'reference',DURATION if case in ('psi','beta') else 7.,dynamic_case=case)
            results.append(r)
            print(json.dumps(r),flush=True)
        result=dict(passed=all(r['passed'] for r in results),cases=results,hardware_verified=False)
    else:result=windows(out,a.probe.resolve())
    (out/'summary.json').write_text(json.dumps(result,indent=2,allow_nan=False),encoding='utf-8')
    print(json.dumps(result),flush=True)
    return 0 if result['passed'] else 1
if __name__=='__main__':raise SystemExit(main())
