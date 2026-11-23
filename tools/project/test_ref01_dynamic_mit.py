"""Windows production-codec loop, then Orin isolated MIT/PTY dynamic-reference loop."""
import argparse,csv,hashlib,json,os,shutil,subprocess,sys
from pathlib import Path
import numpy as np
from ref01_dynamic_validation import reference,K,B,CAP
from ref01_dynamic_mit import tracking,current_scale,fixture
from ref01_mit_fixture import ROOT,Plant,feedback_words

def save(p,x):p.write_text(json.dumps(x,indent=2,allow_nan=False),encoding='utf-8')

def build(out, *, offline_gain_sweep=False):
    import mujoco
    sdk=Path(mujoco.__file__).parent
    compiler=ROOT/'.local/toolchains/zig-0.13.0/ziglang/zig.exe'
    env=dict(os.environ,PYTHONDONTWRITEBYTECODE='1',
        ZIG_GLOBAL_CACHE_DIR=str(ROOT/'.local/cache/zig'),ZIG_LOCAL_CACHE_DIR=str(ROOT/'.local/cache/zig/local'))
    exe=out/'dynamic_mit_probe.exe'
    cmd=[str(compiler),'c++','-std=c++17','-O2','-DNDEBUG','-Wall','-Wextra','-Werror',
        '-I'+str(ROOT/'ros2_ws/src/mech_controllers/include/mech_controllers'),
        '-I'+str(ROOT/'ros2_ws/src/mech_protocol_cubemars/include'),
        '-I'+str(ROOT/'mujoco/ref01/cpp'),'-I'+str(sdk/'include'),
        str(ROOT/'tools/project/ref01_dynamic_mit_probe.cpp'),
        str(ROOT/'ros2_ws/src/mech_protocol_cubemars/src/ak30_force_wire.cpp'),
        str(sdk/'mujoco.dll'),'-o',str(exe)]
    if offline_gain_sweep:
        cmd.insert(2, '-DPH_EXO_OFFLINE_GAIN_SWEEP')
    r=subprocess.run(cmd,cwd=ROOT,env=env,capture_output=True,text=True,timeout=300)
    (out/'compile.log').write_text(r.stdout+r.stderr,encoding='utf-8')
    save(out/'compile.json',dict(command=cmd,returncode=r.returncode))
    if r.returncode:raise RuntimeError(r.stderr[-3000:])
    shutil.copy2(sdk/'mujoco.dll',out/'mujoco.dll')
    return exe

def windows_case(out,exe,case,scale,k,cap=.2):
    plant=Plant(pulse_nm=0.);rows=[];wire_rows=[];seq=0;effort=[0.,0.];last_tx=-1.
    trace=[]
    duration=12. if case in ('psi','beta') else 7.
    log=(out/(case+'_probe.log')).open('w')
    proc=subprocess.Popen([str(exe),'mujoco/ref01/config/mujoco/ref01_dynamics_cad_v1.xml',str(scale),*map(str,k),str(cap)],
        cwd=ROOT,stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=log,text=True,bufsize=1)
    try:
        for tick in range(round(duration/.005)+1):
            t=tick*.005
            q,dq=reference((tick//4)*.02,1 if case=='beta' else 0)
            send=tick%4==0 and not (case=='drop' and 4.<=t<4.6)
            if send:seq+=1
            pos,vel=feedback_words(plant.state['alpha'],plant.state['alpha_velocity'],scale)
            values=[t,*pos,*vel,int(send),seq,1.005+t,*q,*dq]
            proc.stdin.write(' '.join(map(str,values))+'\n');proc.stdin.flush()
            parsed=list(map(float,proc.stdout.readline().split()))
            if len(parsed)!=15:raise RuntimeError('native MIT probe failed')
            mode,reason,refresh,generation=parsed[:4]
            if refresh:
                effort=[parsed[9],parsed[12]];last_tx=t
                wire_rows.append(dict(t=t,mode=mode,generation=generation,tau104=effort[0],tau105=effort[1],
                    req104=parsed[8],req105=parsed[11],code104=int(parsed[10]),code105=int(parsed[13])))
            if last_tx<0. or t-last_tx>=.060:effort=[0.,0.]
            r=plant.advance(effort,t,case)
            rows.append([t,r['psi_rad'],r['beta_rad'],r['dpsi_rad_s'],r['dbeta_rad_s'],
                         *q,*dq,*effort,r['closure_m']])
            trace.append(dict(t=t,mode=int(mode),reason=int(reason),generation=int(generation),send=int(send),seq=seq,scale=parsed[14]))
        proc.stdin.close()
        if proc.wait(timeout=10):raise RuntimeError('native exit failure')
    finally:
        if proc.poll() is None:proc.kill();proc.wait()
        log.close()
    for name,data in ((case+'_wire',wire_rows),(case+'_status',trace)):
        with (out/(name+'.csv')).open('w',newline='') as f:
            w=csv.DictWriter(f,fieldnames=list(data[0]));w.writeheader();w.writerows(data)
    with (out/(case+'.csv')).open('w',newline='') as f:
        w=csv.writer(f);w.writerow(['t','psi','beta','dpsi','dbeta','qref_psi','qref_beta','vref_psi','vref_beta','tau104','tau105','closure']);w.writerows(rows)
    checks=dict(no_warnings=plant.state['warnings']==0,command_seen=bool(wire_rows))
    if case=='drop':
        fault=[r for r in trace if r['mode']==3]
        checks.update(reason_expired=bool(fault) and all(r['reason']==8 for r in fault),
            stop_within_120ms=bool(wire_rows) and 3.97<=wire_rows[-1]['t']<=4.12,
            no_restart=not any(r['t']>=4.45 for r in wire_rows),
            watchdog_zero=all(r[9]==0. and r[10]==0. for r in rows if r[0]>=4.3))
        result=dict(case=case,checks=checks,transport_passed=all(checks.values()),
                    last_tx_after_cut_ms=(wire_rows[-1]['t']-4.)*1000)
    else:
        checks['no_fault']=all(r['mode']==2 for r in trace)
        result=dict(case=case,checks=checks,transport_passed=all(checks.values()),tracking=tracking(rows,cap=cap))
        result['tracking_passed']=result['tracking']['passed']
    scales=[r['scale'] for r in trace if r['mode']==2]
    result['saturation']=dict(observed=len(scales),saturated=sum(x<1. for x in scales),min_scale=min(scales))
    result['passed']=result['transport_passed'] and result.get('tracking_passed',True)
    return result

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output',required=True,type=Path);p.add_argument('--ros',action='store_true')
    p.add_argument('--cases',nargs='+',choices=['psi','beta','drop'],default=['psi','beta','drop'])
    p.add_argument('--k',type=float,nargs=2,default=K)
    a=p.parse_args();out=a.output.resolve()
    if any(not np.isfinite(k) or not 0<k<=10 for k in a.k):p.error('K must be in (0,10]')
    if not out.is_relative_to(ROOT/'.local/validation') or any(x.is_symlink() for x in (out,*out.parents)):raise ValueError('project validation output required')
    if a.ros:
        if sys.platform!='linux' or os.environ.get('EXO_SYNC_LOCK_HELD')!='1' or os.environ.get('ROS_DOMAIN_ID')!='179' or os.environ.get('ROS_LOCALHOST_ONLY')!='1':raise RuntimeError('isolated Orin project lock required')
        if (ROOT/'.local/sync/incomplete.json').exists():raise RuntimeError('incomplete sync')
    out.mkdir(parents=True,exist_ok=False)
    scale,mapping=current_scale()
    save(out/'invocation.json',dict(k=a.k,b=B,tau=CAP,mapping=mapping,feedback_s=.005,reference_s=.02,ros=a.ros,cases=a.cases))
    results=[]
    if not a.ros:exe=build(out)
    for case in a.cases:
        if a.ros:
            from test_ref01_manager import run_case
            f=fixture(a.k)
            try:r=run_case(out/case,'beta' if case=='beta' else 'psi',12. if case!='drop' else 7.,fixture=f,dynamic_case=case,dynamic_gains=a.k)
            except Exception:
                f.close();raise
        else:r=windows_case(out,exe,case,scale,a.k)
        results.append(r);print(json.dumps(r),flush=True)
        save(out/'summary.json',dict(passed=all(x['passed'] for x in results),
            transport_passed=all(x['transport_passed'] for x in results),cases=results,hardware_verified=False))
    return 0 if all(r['passed'] for r in results) else 1
if __name__=='__main__':raise SystemExit(main())
