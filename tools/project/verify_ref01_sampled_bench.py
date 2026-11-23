"""Synchronous model-time bench verifier; no ROS or physical device access."""
import argparse,csv,json,os,shutil,subprocess,sys
from pathlib import Path
import numpy as np
import mujoco
ROOT=Path(__file__).resolve().parents[2]
sys.path[:0]=[str(ROOT/'tools/project'),str(ROOT/'mujoco/ref01')]
from ref01_mit_fixture import feedback_words
from ref01_dynamics_api import REF01Dynamics
def main():
 p=argparse.ArgumentParser();p.add_argument('--output',required=True,type=Path);p.add_argument('--duration',type=float,default=30.);a=p.parse_args()
 out=a.output.resolve();out.mkdir(parents=True,exist_ok=False);sdk=Path(mujoco.__file__).parent
 win=os.name=='nt';exe=out/('sampled_probe.exe' if win else 'sampled_probe')
 cmd=([str(ROOT/'.local/toolchains/zig-0.13.0/ziglang/zig.exe'),'c++'] if win else ['g++'])+['-std=c++17','-O2','-DNDEBUG','-Wall','-Wextra','-Werror']
 for d in ['ros2_ws/src/mech_controllers/include/mech_controllers','ros2_ws/src/mech_bringup/include','ros2_ws/src/mech_protocol_cubemars/include','mujoco/ref01/cpp']:cmd+=['-I'+str(ROOT/d)]
 cmd+=['-I'+str(sdk/'include'),str(ROOT/'tools/project/ref01_sampled_bench_probe.cpp'),str(ROOT/'ros2_ws/src/mech_protocol_cubemars/src/ak30_force_wire.cpp')]
 if win:cmd+=[str(sdk/'mujoco.dll')]
 else:cmd+=[str(next(sdk.glob('libmujoco.so*'))),'-Wl,-rpath,'+str(sdk)]
 cmd+=['-o',str(exe)]
 env=dict(os.environ,PYTHONDONTWRITEBYTECODE='1',ZIG_GLOBAL_CACHE_DIR=str(ROOT/'.local/cache/zig'),ZIG_LOCAL_CACHE_DIR=str(ROOT/'.local/cache/zig/local'))
 r=subprocess.run(cmd,cwd=ROOT,env=env,capture_output=True,text=True,timeout=300)
 (out/'compile.log').write_text(r.stdout+r.stderr,encoding='utf-8')
 if r.returncode:raise RuntimeError(r.stderr)
 if win:shutil.copy2(sdk/'mujoco.dll',out/'mujoco.dll')
 contract=out/('predictor_contract.exe' if win else 'predictor_contract')
 common=cmd[:cmd.index(str(ROOT/'tools/project/ref01_sampled_bench_probe.cpp'))]
 for source,name in [('test_ref01_bench_predictor.cpp','predictor_contract'),('test_kinematic_lease.cpp','lease_contract')]:
  binary=out/(name+('.exe' if win else ''))
  r=subprocess.run(common+[str(ROOT/'tools/project'/source),'-o',str(binary)],cwd=ROOT,env=env,capture_output=True,text=True,timeout=300)
  (out/(name+'_compile.log')).write_text(r.stdout+r.stderr,encoding='utf-8')
  if r.returncode:raise RuntimeError(r.stderr)
  r=subprocess.run([str(binary)],cwd=ROOT,env=env,capture_output=True,text=True,timeout=30)
  (out/(name+'.log')).write_text(r.stdout+r.stderr,encoding='utf-8')
  if r.returncode:raise RuntimeError(r.stdout+r.stderr)
  print(r.stdout.strip(),flush=True)

 cases=[(f'p{ref_id}_{axis}_{sign}',ref_id,axis,sign,'pulse') for ref_id in (0,1) for axis in (0,1) for sign in (-1,1)]
 cases +=[(f'hold_{i}',i,0,0,'hold') for i in (0,1)]
 cases +=[(f'load_{i}',0,i,1,'load') for i in (0,1)]
 cases +=[(f'fault_{fault}',0,0,0,fault) for fault in ('loss','plan_loss','disabled','rollback','mutation')]
 result=[]
 for name,ref_id,axis,sign,kind in cases:
  ref=np.deg2rad([0,20] if ref_id==0 else [.55,15.288231]);plant=REF01Dynamics();state=plant.reset(ref)
  rows=[];duration=a.duration if kind in ('pulse','hold') else (7 if kind=='load' else 2)
  with (out/(name+'.log')).open('w') as log:
   proc=subprocess.Popen([str(exe),'mujoco/ref01/config/mujoco/ref01_dynamics_cad_v1.xml',*map(str,ref)],cwd=ROOT,stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=log,text=True,bufsize=1)
   faulted=False
   try:
    for tick in range(round(duration/.005)):
     t=tick*.005;now=1.+(tick+1)*.005
     if tick%4==0 and not(kind=='loss' and 1.<=t<1.4):
      pos,vel=feedback_words(state['alpha'],state['alpha_velocity']);seq=tick//4+1;source=now-1e-10
     if kind=='rollback' and t>=1. and t<1.02:seq=1
     if kind=='mutation' and tick==201:pos[0]+=1
     issue=1.5 if kind=='plan_loss' and t>=.5 else now-1e-10
     enabled=0 if kind=='disabled' and t>=1 else 1
     proc.stdin.write(' '.join(map(str,pos+vel+[source,seq,issue,enabled]))+'\n');proc.stdin.flush()
     raw=proc.stdout.readline().split()
     if len(raw)!=16:raise RuntimeError(name+' probe exited: '+repr(raw)+' tick='+str(tick)+' code='+str(proc.poll())+' input='+repr(pos+vel)+' previous='+repr(rows[-1:]))
     v=list(map(float,raw[:11]));mode=int(v[10]);faulted|=mode==3
     applied=np.array([v[5],v[8]]) if mode==2 else np.zeros(2)
     external=np.zeros(2)
     if .4<=t<(4. if kind=='load' else 1.):external[axis]=sign*.3
     rows.append([t,*state['q'],*state['dq'],v[4],v[7],*applied,mode,raw[11],float(raw[12]),*map(float,raw[13:15]),state['max_closure_m'],state['warnings'],int(raw[15])])
     state=plant.step(applied[::-1],steps=10,external_torque=external)
    proc.stdin.close()
    if proc.wait(timeout=10):raise RuntimeError(name+' native exit')
   finally:
    if proc.poll() is None:proc.kill();proc.wait()
  with (out/(name+'.csv')).open('w',newline='') as f:
   w=csv.writer(f);w.writerow(['t','psi','beta','dpsi','dbeta','desired104','desired105','sent104','sent105','mode','reason','source','external_psi','external_beta','closure','warnings','sent_mask']);w.writerows(rows)
  time=np.array([r[0] for r in rows]);q=np.array([r[1:3] for r in rows]);vels=np.array([r[3:5] for r in rows]);torque=np.array([r[7:9] for r in rows]);modes=np.array([r[9] for r in rows])
  normal=kind in ('pulse','hold','load');tail=time>=(5.5 if kind=='load' else 2.5)
  checks={'finite':bool(np.isfinite(q).all() and np.isfinite(vels).all()),'no_warnings':all(r[15]==0 for r in rows)}
  item=dict(name=name,kind=kind,checks=checks)
  if normal:
   err=np.rad2deg(q-ref);speed=np.rad2deg(vels)
   checks.update(following=bool(np.all(modes==2)),writes=all(r[16]==3 for r in rows),tail_error=bool(abs(err[tail]).max()<=.5),tail_speed=bool(abs(speed[tail]).max()<=1.),closure=max(r[14] for r in rows)<1e-4,torque=bool(abs(torque).max()<=5+30/4095))
   item.update(tail_error_deg=float(abs(err[tail]).max()),tail_speed_deg_s=float(abs(speed[tail]).max()))
   if kind=='load':
    settled=(time>=3.5)&(time<4.)
    deflect=float(np.rad2deg(q[settled,axis]-ref[axis]).mean());expected=np.rad2deg(.3/10)
    checks['static_stiffness']=bool(abs(deflect-expected)<=.25);item.update(steady_deflection_deg=deflect,expected_deg=float(expected))
  else:
   trip=np.flatnonzero(modes==3)
   checks.update(faulted=bool(len(trip)),latched=bool(len(trip) and np.all(modes[trip[0]:]==3)),stop_writes=bool(len(trip) and all(r[16]==0 for r in rows[trip[0]+1:])))
   item['first_fault_s']=float(time[trip[0]]) if len(trip) else None
   item['reason']=rows[trip[0]][10] if len(trip) else None
  item['passed']=all(checks.values());result.append(item);print(json.dumps(item),flush=True)
  (out/'summary.json').write_text(json.dumps(dict(passed=all(x['passed'] for x in result),complete=len(result)==len(cases),cases=result,feedback_period_s=.020,control_period_s=.005,hardware=False,ros=False,boundary='Synchronous model-time C++ core/model + production MIT codec; no host realtime or device timing acceptance.'),indent=2),encoding='utf-8')
 return 0 if all(x['passed'] for x in result) else 1
if __name__=='__main__':raise SystemExit(main())
