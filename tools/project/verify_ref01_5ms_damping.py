"""Windows numerical audit of ordinary PH-EXO impedance at 200 Hz.
Explicit trace input only; never opens a port, SSH connection or ROS node.
"""
import argparse, csv, hashlib, json, os, subprocess, sys
from pathlib import Path
import numpy as np
ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT/'tools/project'), str(ROOT/'mujoco/ref01')]
from analyze_ref01_ordinary_damping import compile_probe
from ref01_dynamics_api import REF01Dynamics
from ref01_mit_fixture import feedback_words, ERPM_SCALE
from sync_orin import safe_path

BS = (0., .1, .2, .4, .6, 1.)
REF = np.deg2rad([4.05, 16.608514])
HNS = 5_000_000
COLS = ['t','psi','beta','dpsi','dbeta','cmd104','cmd105','applied104','applied105',
        'refresh','mode','seq104','seq105','source104','source105',
        'estimated_psi','estimated_beta','estimated_dpsi','estimated_dbeta',
        'spring104','spring105','damper104','damper105','gravity104','gravity105',
        'raw104','raw105','scale','closure','warnings']
PROFILES = {
    'ideal5': dict(feedback_ms=5, command_delay_ticks=0, trace=False, phase_ms=0),
    'pipeline5': dict(feedback_ms=5, command_delay_ticks=1, trace=False, phase_ms=0),
    'receipt5_phase0': dict(feedback_ms=5, command_delay_ticks=1, trace=True, phase_ms=0),
    'receipt5_phase2p5': dict(feedback_ms=5, command_delay_ticks=1, trace=True, phase_ms=2.5),
    'pipeline20': dict(feedback_ms=20, command_delay_ticks=1, trace=False, phase_ms=0),
}

def save(path, data):
    path.write_text(json.dumps(data, ensure_ascii=False, indent=2), encoding='utf-8')

def schedules(trace, profile, ticks):
    spec = PROFILES[profile]
    if not spec['trace']:
        capture = np.arange(0, ticks*HNS, spec['feedback_ms']*1_000_000, dtype=np.int64)
        return [(capture, capture.copy()) for _ in range(2)]
    result = []
    for axis, mid in enumerate((104,105)):
        rows = json.loads((trace/f'frames_{mid}.json').read_text(encoding='utf-8'))
        received = np.array([int(r['ns']) for r in rows], dtype=np.int64)
        received -= received[0]
        capture = np.arange(len(received), dtype=np.int64)*HNS
        residual = received-capture
        # Arrival trace contains polling phase and clock drift. Only a synthetic
        # causal schedule can be inferred; never label these delays as measured.
        delay = residual - min(0, int(residual.min()))
        arrival = capture + delay + int(spec['phase_ms']*1e6 if axis else 0)
        assert np.all(arrival>=capture) and np.all(np.diff(arrival)>0)
        assert capture[-1] >= (ticks-1)*HNS, 'trace too short; no cycling'
        result.append((capture,arrival))
    return result

def metrics(a, complete, failure):
    t=a[:,0];error=np.rad2deg(a[:,1:3]-REF);speed=np.rad2deg(a[:,3:5])
    tail=t>=2.5
    good=(abs(error).max(axis=1)<=.5)&(abs(speed).max(axis=1)<=1)
    settled=np.flatnonzero((t>=1)&np.logical_and.accumulate(good[::-1])[::-1])
    refresh=t[a[:,9]==1]; gaps=np.diff(refresh)*1000
    cmd=a[:,5:7]; valid=a[:,10]==2
    pair_valid=valid[1:] & valid[:-1]
    all_jump=np.diff(cmd,axis=0); jump=all_jump[pair_valid]
    pre_release=(t[1:]>=.4)&(t[1:]<=1)&pair_valid
    return dict(complete=complete, failure=failure,
        passed=bool(complete and tail.any() and good[tail].all()),
        tail_error_deg=float(abs(error[tail]).max()) if tail.any() else None,
        tail_speed_deg_s=float(abs(speed[tail]).max()) if tail.any() else None,
        peak_error_deg=float(abs(error).max()), peak_speed_deg_s=float(abs(speed).max()),
        settling_after_release_s=float(t[settled[0]]-1) if complete and len(settled) else None,
        peak_command_nm=abs(cmd[valid]).max(axis=0).tolist(),
        max_jump_nm=abs(jump).max(axis=0).tolist() if len(jump) else [0,0],
        p99_jump_nm=np.percentile(abs(jump),99,axis=0).tolist() if len(jump) else [0,0],
        max_push_jump_nm=abs(all_jump[pre_release]).max(axis=0).tolist() if pre_release.any() else None,
        refresh_count=len(refresh),
        refresh_hz=float((len(refresh)-1)/(refresh[-1]-refresh[0])) if len(refresh)>1 else None,
        max_refresh_gap_ms=float(gaps.max()) if len(gaps) else None,
        refresh_gaps_over_5ms=int((gaps>5.000001).sum()),
        saturated_samples=int((a[:,27]<.999999).sum()),
        samples=len(a))

def run_case(out, trace, profile, B, axis, sign, ticks=1200, K=10., cap=10., velocity_quantized=True):
    ident=f'{profile}_K{K:g}_B{B:g}_axis{axis}_sign{sign}' + ('' if velocity_quantized else '_exactv')
    plant=REF01Dynamics(); state=plant.reset(REF)
    clocks=schedules(trace,profile,ticks)
    assert abs(plant.model.opt.timestep-.0005)<1e-12
    feedback=np.zeros((2,4)); queues=[[],[]]; captured=[0,0]
    rows=[]; velocity_rows=[]; failure=None; pending=np.zeros(2)
    p0,v0=feedback_words(state['alpha'],state['alpha_velocity'])
    if not velocity_quantized: v0=np.asarray(state['alpha_velocity'])[::-1]/(10*ERPM_SCALE)
    feedback[:,0]=np.deg2rad(np.array(p0)/10)
    feedback[:,1]=np.array(v0)*10*ERPM_SCALE
    feedback[:,2]=1.; feedback[:,3]=1.
    with (out/(ident+'.log')).open('w',encoding='utf-8') as log:
        proc=subprocess.Popen([str(out/'replay.exe'),
            str(ROOT/'mujoco/ref01/config/mujoco/ref01_dynamics_cad_v1.xml'),
            *map(str,REF),str(B),str(cap),str(K)],cwd=ROOT,stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,stderr=log,text=True,bufsize=1)
        try:
            for tick in range(ticks):
                ns=tick*HNS; t=ns*1e-9
                pos,vel=feedback_words(state['alpha'],state['alpha_velocity'])
                if not velocity_quantized: vel=np.asarray(state['alpha_velocity'])[::-1]/(10*ERPM_SCALE)
                if max(abs(v) for v in list(pos)+list(vel))>32767: failure='feedback_range'; break
                for j in range(2):
                    caps,arrivals=clocks[j]
                    i=captured[j]
                    if i<len(caps) and caps[i]==ns:
                        # Both axes' hypothetical capture clocks are regular.
                        # The added phase changes RECEIPT, not a measured capture clock.
                        queues[j].append((int(arrivals[i]),pos[j],vel[j],i+2))
                        captured[j]+=1
                    while queues[j] and queues[j][0][0]<=ns:
                        arrival,p,v,seq=queues[j].pop(0)
                        feedback[j]=[np.deg2rad(p/10),v*10*ERPM_SCALE,1.+arrival*1e-9,seq]
                proc.stdin.write(' '.join(map(str,[1.+t,*feedback.ravel()]))+'\n')
                proc.stdin.flush()
                raw=proc.stdout.readline().split()
                if len(raw)!=23: raise RuntimeError(f'{ident} tick {tick}: {raw!r}')
                velocity_rows.append([t,*feedback[:,1],*np.asarray(state['alpha_velocity'])[::-1]])
                mode=int(raw[1]); refresh=int(raw[2])
                values=np.array([float(x) for x in raw[4:]])
                command=values[:2]
                # Runtime stores write(n), submits it in read(n+1).
                applied=pending.copy() if PROFILES[profile]['command_delay_ticks'] else command.copy()
                pending=command.copy()
                A=values[13:17].reshape(2,2)
                components=[]
                for pair in (values[6:8],values[8:10],values[10:12]):
                    components.extend(np.linalg.solve(A.T,pair)[::-1]*values[12])
                rows.append([t,*state['q'],*state['dq'],*command,*applied,refresh,mode,
                    *feedback[:,3],*feedback[:,2],*values[2:6],*components,
                    *values[17:19],values[12],state['max_closure_m'],state['warnings']])
                if mode!=2: failure=raw[3]; break
                ext=np.zeros(2)
                if .4<=t<1.: ext[axis]=sign*.3
                state=plant.step(applied[::-1],steps=10,external_torque=ext)
                if not np.isfinite(state['q']).all() or not np.isfinite(state['dq']).all():
                    failure='nonfinite'; break
                if abs(state['q'][0])>np.pi/4 or not 0<=state['q'][1]<=np.pi/4:
                    failure='historical_plant_domain_exit'; break
                if state['warnings'] or state['max_closure_m']>=1e-4:
                    failure='plant_warning_or_closure'; break
        finally:
            proc.stdin.close()
            try: code=proc.wait(timeout=15)
            except subprocess.TimeoutExpired: proc.kill();proc.wait();raise
            if code: raise RuntimeError(f'probe exit {code}')
    a=np.array(rows)
    with (out/(ident+'_velocity.csv')).open('w',newline='') as f:
        w=csv.writer(f);w.writerow(['t','feedback104','feedback105','true104','true105']);w.writerows(velocity_rows)
    with (out/(ident+'.csv')).open('w',newline='',encoding='utf-8') as f:
        w=csv.writer(f); w.writerow(COLS); w.writerows(rows)
    r=dict(id=ident,profile=profile,K=K,cap=cap,velocity_quantized=velocity_quantized,B=B,axis=axis,sign=sign,
           **metrics(a,len(rows)==ticks and failure is None,failure))
    return r

def timing_audit(out,trace):
    # Stationary input isolates generation gating from plant stability.
    plant=REF01Dynamics();s=plant.reset(REF)
    p,v=feedback_words(s['alpha'],s['alpha_velocity'])
    result=[]
    for profile in PROFILES:
        clocks=schedules(trace,profile,1200);lines=[]
        feedback=np.array([[np.deg2rad(p[j]/10),v[j]*10*ERPM_SCALE,1,1] for j in range(2)])
        indices=[0,0]
        for tick in range(1200):
            ns=tick*HNS
            for j,(caps,arrivals) in enumerate(clocks):
                while indices[j]<len(arrivals) and arrivals[indices[j]]<=ns:
                    i=indices[j]; feedback[j,2:]=[1+int(arrivals[i])*1e-9,i+2];indices[j]+=1
            lines.append([1+ns*1e-9,*feedback.ravel()])
        r=subprocess.run([str(out/'replay.exe'),str(ROOT/'mujoco/ref01/config/mujoco/ref01_dynamics_cad_v1.xml'),
            *map(str,REF),'0','10'],input='\n'.join(' '.join(map(str,row)) for row in lines)+'\n',
            cwd=ROOT,capture_output=True,text=True,timeout=60)
        assert r.returncode==0,r.stderr
        raw=[line.split() for line in r.stdout.splitlines()]
        assert len(raw)==1200 and all(x[1]=='2' for x in raw)
        last=[0,0]; count=0;times=[]
        records=[]
        for data,answer in zip(lines,raw):
            seq=[data[4],data[8]]
            expected=all(seq[j]>last[j] for j in range(2))
            assert int(answer[2])==int(expected), (profile,data,answer)
            if expected: last=seq;count+=1;times.append(data[0])
            records.append([data[0]-1,*seq,int(answer[2]),*data[3:4],*data[7:8]])
        with (out/(profile+'_timing.csv')).open('w',newline='') as f:
            w=csv.writer(f);w.writerow(['t','seq104','seq105','refresh','source104','source105']);w.writerows(records)
        gaps=np.diff(times)*1000
        result.append(dict(profile=profile,cycles=1200,refreshes=count,
            refresh_hz=(count-1)/(times[-1]-times[0]),max_refresh_gap_ms=float(gaps.max()),
            gaps_over_5ms=int((gaps>5.000001).sum()),sequence_gate_verified=True))
    save(out/'timing_audit.json',result)
    return result

def validate(out,results):
    count=0;max_component_error=0.
    for r in results:
        a=np.loadtxt(out/(r['id']+'.csv'),delimiter=',',skiprows=1,ndmin=2);count+=len(a)
        # Independent named-column checks; no call to metrics().
        d={name:a[:,i] for i,name in enumerate(COLS)}
        err=np.rad2deg(np.c_[d['psi'],d['beta']]-REF)
        speed=np.rad2deg(np.c_[d['dpsi'],d['dbeta']])
        tail=d['t']>=2.5
        passed=bool(r['complete'] and tail.any() and (abs(err[tail])<=.5).all() and (abs(speed[tail])<=1).all())
        assert passed==r['passed']
        if tail.any():
            assert abs(float(abs(speed[tail]).max())-r['tail_speed_deg_s'])<1e-9
            assert abs(float(abs(err[tail]).max())-r['tail_error_deg'])<1e-9
        valid=d['mode']==2
        for mid in (104,105):
            code=(d[f'cmd{mid}'][valid]+15)*4095/30
            assert abs(code-np.rint(code)).max()<1e-9
            delta=abs(np.diff(d[f'cmd{mid}']))[valid[1:] & valid[:-1]]
            assert abs(float(delta.max(initial=0))-r['max_jump_nm'][mid-104])<1e-10
            component=d[f'spring{mid}']+d[f'damper{mid}']+d[f'gravity{mid}']
            error=abs(component[valid]-d[f'raw{mid}'][valid]).max()
            max_component_error=max(max_component_error,float(error));assert error<1e-10
            if PROFILES[r['profile']]['command_delay_ticks']:
                assert abs(d[f'applied{mid}'][1:]-d[f'cmd{mid}'][:-1]).max(initial=0)<1e-12
            else: assert (d[f'applied{mid}']==d[f'cmd{mid}']).all()
        assert np.all(np.diff(d['seq104'])>=0) and np.all(np.diff(d['seq105'])>=0)
    result=dict(passed=True,cases=len(results),csv_rows=count,
                component_sum_max_error_nm=max_component_error,
                checks=['original recovery bands','MIT grid','torque jumps','component decomposition',
                        'one-cycle command pipeline','nondecreasing feedback sequences'])
    save(out/'validation.json',result)
    return result

def plots(out,results):
    os.environ['MPLCONFIGDIR']=str(out/'mplconfig')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    plt.rcParams.update({'font.family':'Microsoft YaHei','axes.unicode_minus':False,'font.size':10})
    for profile in PROFILES:
        fig,axs=plt.subplots(2,4,figsize=(17,7),sharex=True)
        for r in results:
            if r['profile']!=profile or r['sign']!=1: continue
            a=np.loadtxt(out/(r['id']+'.csv'),delimiter=',',skiprows=1,ndmin=2)
            axis=r['axis'];t=a[:,0]
            ys=[np.rad2deg(a[:,1+axis]-REF[axis]),np.rad2deg(a[:,3+axis]),a[:,5],a[:,6]]
            for j,y in enumerate(ys):axs[axis,j].plot(t,y,lw=.85,label=f"B={r['B']:g}")
        for axis in (0,1):
            for col,label in enumerate(['姿态误差 / °','角速度 / °/s','104 目标力矩 / N·m','105 目标力矩 / N·m']):
                ax=axs[axis,col];ax.set_title(('ψ扰动' if axis==0 else 'β扰动')+' · '+label)
                ax.axvspan(.4,1,color='gray',alpha=.13);ax.axvline(2.5,color='black',ls=':',lw=.8)
                ax.grid(alpha=.2);ax.set_xlabel('模型时间 / s');ax.set_xlim(0,6)
                if col<2:
                    band=.5 if col==0 else 1
                    ax.axhline(band,color='black',ls='--',lw=.6);ax.axhline(-band,color='black',ls='--',lw=.6)
        axs[0,0].legend(ncol=3,fontsize=8)
        fig.suptitle(f'PH-EXO 普通阻抗：K=10，{profile}\n灰区：+0.3 N·m扰动；1 s撤力；2.5 s后误差≤0.5°且速度≤1°/s；提前终止曲线=失败',fontsize=12)
        fig.tight_layout();fig.savefig(out/f'curves_{profile}.png',dpi=160);plt.close(fig)
    fig,axs=plt.subplots(1,2,figsize=(12,4.5))
    for profile in PROFILES:
        rows=[r for r in results if r['profile']==profile]
        if not rows:continue
        xs=sorted(set(r['B'] for r in rows))
        speed=[max((r['tail_speed_deg_s'] if r['tail_speed_deg_s'] is not None else np.nan) for r in rows if r['B']==b) for b in xs]
        jump=[max(max(r['max_jump_nm']) for r in rows if r['B']==b) for b in xs]
        axs[0].plot(xs,speed,'o-',label=profile);axs[1].plot(xs,jump,'o-',label=profile)
    axs[0].axhline(1,color='black',ls='--');axs[0].set_ylabel('尾段两轴最大速度 / °/s');axs[0].set_yscale('log')
    axs[1].set_ylabel('两电机最大相邻目标变化 / N·m')
    for ax in axs:ax.set_xlabel('两轴阻尼 B / N·m·s/rad');ax.grid(alpha=.2);ax.legend(fontsize=8)
    fig.suptitle('PH-EXO：保持K=10的阻尼与时序对照（无尾段的数据不绘速度）')
    fig.tight_layout();fig.savefig(out/'comparison.png',dpi=180);plt.close(fig)

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output',type=Path,required=True);p.add_argument('--trace',type=Path,required=True)
    p.add_argument('--smoke',action='store_true');p.add_argument('--plots-only',action='store_true')
    args=p.parse_args()
    out=safe_path(ROOT,str(args.output));trace=safe_path(ROOT,str(args.trace))
    if args.plots_only:
        summary=json.loads((out/'summary.json').read_text(encoding='utf-8'))
        validate(out,summary['cases']);plots(out,summary['cases']);return
    out.mkdir(parents=True,exist_ok=False)
    sources=['tools/project/verify_ref01_5ms_damping.py','tools/project/replay_ref01_ordinary.cpp',
        'tools/project/analyze_ref01_ordinary_damping.py','tools/project/ref01_mit_fixture.py',
        'ros2_ws/src/mech_controllers/include/mech_controllers/ref01_impedance_core.hpp',
        'ros2_ws/src/mech_controllers/src/ref01_impedance_controller.cpp',
        'mujoco/ref01/cpp/ref01_model.hpp','mujoco/ref01/ref01_dynamics_api.py',
        'mujoco/ref01/config/mujoco/ref01_dynamics_cad_v1.xml',
        'ros2_ws/src/mech_protocol_cubemars/src/ak30_force_wire.cpp']
    hashes={s:hashlib.sha256((ROOT/s).read_bytes()).hexdigest() for s in sources}
    save(out/'source_hashes.json',hashes)
    save(out/'input_hashes.json',{str((trace/f'frames_{mid}.json').relative_to(ROOT)):
         hashlib.sha256((trace/f'frames_{mid}.json').read_bytes()).hexdigest() for mid in (104,105)})
    save(out/'design.json',dict(K=[10,10],B=BS,reference_deg=np.rad2deg(REF).tolist(),duration_s=6,
        perturbation_nm=.3,perturbation_s=[.4,1],recovery=dict(after_s=2.5,error_deg=.5,speed_deg_s=1),
        command_cap_nm=10,profiles=PROFILES,simulation_only=True,
        limitations=['No ROS process or device access','No physical torque/current dynamics',
          'Sequential traces do not measure inter-axis phase','Capture clocks and causal delivery offsets are assumed',
          'Host polling jitter is not intrinsic device jitter','Internal velocity-estimator delay is unknown',
          'Original recovery criterion and historical simulation domain retained']))
    compile_probe(out);print('Native Session/model/MIT codec compiled',flush=True)
    audit=timing_audit(out,trace);print(json.dumps(audit),flush=True)
    jobs=[(profile,b,axis,sign) for profile in PROFILES for b in BS for axis in (0,1) for sign in (-1,1)]
    if args.smoke:jobs=[('ideal5',.2,0,1),('pipeline5',.2,1,1),('receipt5_phase2p5',.2,0,1)]
    results=[]
    for n,job in enumerate(jobs,1):
        r=run_case(out,trace,*job);results.append(r)
        save(out/'summary.json',dict(complete=n==len(jobs),cases=results))
        print(json.dumps(dict(n=n,total=len(jobs),id=r['id'],passed=r['passed'],failure=r['failure'],
              speed=r['tail_speed_deg_s'],refresh_hz=r['refresh_hz'])),flush=True)
    validate(out,results)
    assert all(hashlib.sha256((ROOT/s).read_bytes()).hexdigest()==h for s,h in hashes.items())
    plots(out,results)
    print(json.dumps(dict(complete=True,cases=len(results),recovery_pass=sum(r['passed'] for r in results),
                         verification_pass=True)),flush=True)

if __name__=='__main__':main()
