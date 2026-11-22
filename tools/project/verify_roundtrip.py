"""Device-free panel trajectory -> production C++ impedance/codec -> MuJoCo.

Windows first; identical numerical test can run on Orin. Separate from the
panel/ROS private-PTY test. No serial, CAN or physical backend is opened.
"""
import argparse,csv,json,os,shutil,subprocess,sys,math
from pathlib import Path
import numpy as np
from ph_exo_bench_worker import DEFAULT,make_trajectory
from ref01_mit_fixture import ROOT,Plant,feedback_words,LSB
from ref01_dynamic_mit import current_scale

def build(out):
    if sys.platform=='win32':
        from test_ref01_dynamic_mit import build as windows_build
        return windows_build(out)
    import mujoco
    sdk=Path(mujoco.__file__).parent
    exe=out/'dynamic_mit_probe'
    lib=next(sdk.glob('libmujoco.so.*'))
    cmd=['g++','-std=c++17','-O2','-Wall','-Wextra','-Werror',
        '-I'+str(ROOT/'ros2_ws/src/mech_controllers/include/mech_controllers'),
        '-I'+str(ROOT/'ros2_ws/src/mech_protocol_cubemars/include'),
        '-I'+str(ROOT/'mujoco/ref01/cpp'),'-I'+str(sdk/'include'),
        str(ROOT/'tools/project/ref01_dynamic_mit_probe.cpp'),
        str(ROOT/'ros2_ws/src/mech_protocol_cubemars/src/ak30_force_wire.cpp'),
        str(lib),'-Wl,-rpath,'+str(sdk),'-o',str(exe)]
    r=subprocess.run(cmd,capture_output=True,text=True,timeout=300)
    (out/'compile.log').write_text(r.stdout+r.stderr)
    if r.returncode:raise RuntimeError(r.stderr)
    return exe

def run(out,exe,case):
    amp={'psi':[3,0],'beta':[0,3],'both':[3,-2],'drop':[3,-2]}[case]
    cfg=dict(DEFAULT,amp_deg=amp,period_s=[6,4])
    tr=make_trajectory(cfg);home=[0,math.radians(20)];tr.validate(home)
    scale,_=current_scale();plant=Plant(pulse_nm=0)
    proc=subprocess.Popen([str(exe),str(ROOT/'mujoco/ref01/config/mujoco/ref01_dynamics_cad_v1.xml'),
        str(scale),'20','20','10','.3','.3'],stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True,bufsize=1)
    rows=[];seq=0;effort=[0.,0.];last=-1;tx=[]
    try:
        for tick in range(3201 if case!='drop' else 1401):
            t=tick*.005;rt=(tick//4)*.02
            # Two/three complete periods, then diagnostic static tail only.
            # The product itself runs indefinitely until the operator stops.
            q,dq=tr.sample(min(rt,13.),home)
            send=tick%4==0 and not (case=='drop' and 4<=t<4.6)
            if send:seq+=1
            pos,vel=feedback_words(plant.state['alpha'],plant.state['alpha_velocity'],scale)
            proc.stdin.write(' '.join(map(str,[t,*pos,*vel,int(send),seq,1.005+t,*q,*dq]))+'\n');proc.stdin.flush()
            parsed=list(map(float,proc.stdout.readline().split()))
            if len(parsed)!=15:raise RuntimeError('probe output invalid: '+proc.stderr.read())
            if parsed[2]:effort=[parsed[9],parsed[12]];last=t;tx.append(t)
            if last<0 or t-last>=.060:effort=[0.,0.]
            r=plant.advance(effort,t,case)
            rows.append([t,r['psi_rad'],r['beta_rad'],r['dpsi_rad_s'],r['dbeta_rad_s'],*q,*dq,*effort,r['closure_m'],*parsed[:3],parsed[14],parsed[8],parsed[11]])
        proc.stdin.close();rc=proc.wait(timeout=10)
        if rc:raise RuntimeError(proc.stderr.read())
    finally:
        if proc.poll() is None:proc.kill();proc.wait()
    with (out/(case+'.csv')).open('w',newline='') as f:
        w=csv.writer(f);w.writerow(['t','psi','beta','dpsi','dbeta','qref_psi','qref_beta','vref_psi','vref_beta','tau104','tau105','closure','mode','reason','refresh','scale','req104','req105']);w.writerows(rows)
    a=np.asarray(rows);err=np.rad2deg(a[:,1:3]-a[:,5:7]);tail=a[:,0]>=13.6;moving=(a[:,0]>=1)&(a[:,0]<=13)
    checks=dict(finite=bool(np.isfinite(a).all()),no_solver_warnings=plant.state['warnings']==0,
                closure=bool(abs(a[:,11]).max()<1e-4),command_cap=bool(abs(a[:,16:18]).max()<=10+1e-12),
                decoded_cap=bool(abs(a[:,9:11]).max()<=10+LSB),commands_present=bool(tx))
    if case=='drop':
        faults=a[a[:,12]==3]
        checks.update(expired=bool(len(faults)) and bool((faults[:,13]==8).all()),stop_within_120ms=3.97<=tx[-1]<=4.12,
                      no_restart=not any(x>=4.45 for x in tx),watchdog_zero=bool((a[a[:,0]>=4.3,9:11]==0).all()))
        metrics=dict(last_tx_after_cut_ms=(tx[-1]-4)*1000)
    else:
        checks['no_fault']=bool((a[:,12]==2).all())
        metrics=dict(rmse_deg=np.sqrt(np.mean(err[moving]**2,axis=0)).tolist(),tail_error_deg=abs(err[tail]).max(axis=0).tolist(),
                     tail_speed_deg_s=abs(np.rad2deg(a[tail,3:5])).max(axis=0).tolist())
        metrics['tracking_passed']=max(metrics['rmse_deg'])<.15 and max(metrics['tail_error_deg'])<=.5 and max(metrics['tail_speed_deg_s'])<=1.
    metrics.update(peak_nm=abs(a[:,9:11]).max(axis=0).tolist(),max_step_nm=abs(np.diff(a[:,9:11],axis=0)).max(axis=0).tolist(),saturated=int((a[:,15]<1).sum()))
    return dict(case=case,config=cfg,checks=checks,functional_passed=all(checks.values()),metrics=metrics)

def main():
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);a=p.parse_args();out=a.output.resolve()
    if not out.is_relative_to(ROOT/'.local/validation') or any(x.is_symlink() for x in (out,*out.parents)):raise ValueError('project validation output required')
    out.mkdir(parents=True,exist_ok=False);exe=build(out)
    results=[]
    for case in ('psi','beta','both','drop'):
        r=run(out,exe,case);results.append(r);print(json.dumps(r),flush=True)
    summary=dict(platform=sys.platform,hardware_started=False,cases=results,functional_passed=all(r['functional_passed'] for r in results),
                 criteria=dict(rmse_deg=.15,tail_error_deg=.5,tail_speed_deg_s=1.),feedback_s=.005,reference_s=.02)
    (out/'summary.json').write_text(json.dumps(summary,indent=2),encoding='utf-8')
    return 0 if summary['functional_passed'] else 1
if __name__=='__main__':raise SystemExit(main())
