"""Windows numerical experiment, frozen controller model, plant-only perturbations."""
import argparse,csv,hashlib,json,os,subprocess,sys,shutil
from pathlib import Path
import numpy as np
import mujoco
ROOT=Path(__file__).resolve().parents[2]
sys.path[:0]=[str(ROOT/'tools/project'),str(ROOT/'mujoco/ref01')]
from ref01_mit_fixture import feedback_words
from ref01_dynamics_api import REF01Dynamics
SCENARIOS=[('nominal',1,0,0,0,0),
 ('body_I_half',.5,0,0,0,0),('body_I_x2',2,0,0,0,0),('body_I_x4',4,0,0,0,0),
 ('rotor_001',1,.001,0,0,0),('rotor_005',1,.005,0,0,0),('rotor_020',1,.02,0,0,0),
 ('visc_002',1,0,.02,0,0),('visc_010',1,0,.1,0,0),('visc_030',1,0,.3,0,0),
 ('coulomb_002',1,0,0,.02,0),('coulomb_010',1,0,0,.1,0),
 ('delay_5',1,0,0,0,5),('delay_10',1,0,0,0,10),('delay_20',1,0,0,0,20),('delay_40',1,0,0,0,40),
 ('combined_light',.5,0,.02,.02,10),('combined_rotor',1,.005,.1,.02,20)]
MODES=[('ordinary_B1','ordinary',1.),('ordinary_B02','ordinary',.2),('predict_B1','predict',1.)]
def compile_probe(out):
 sdk=Path(mujoco.__file__).parent
 cmd=[str(ROOT/'.local/toolchains/zig-0.13.0/ziglang/zig.exe'),'c++','-std=c++17','-O2','-DNDEBUG','-Wall','-Wextra','-Werror']
 for d in ['ros2_ws/src/mech_controllers/include/mech_controllers','ros2_ws/src/mech_protocol_cubemars/include','mujoco/ref01/cpp']:cmd+=['-I'+str(ROOT/d)]
 cmd+=['-I'+str(sdk/'include'),str(ROOT/'tools/project/ref01_robustness_probe.cpp'),str(ROOT/'ros2_ws/src/mech_protocol_cubemars/src/ak30_force_wire.cpp'),str(sdk/'mujoco.dll'),'-o',str(out/'probe.exe')]
 env=dict(os.environ,PYTHONDONTWRITEBYTECODE='1',ZIG_GLOBAL_CACHE_DIR=str(ROOT/'.local/cache/zig'),ZIG_LOCAL_CACHE_DIR=str(ROOT/'.local/cache/zig/local'))
 r=subprocess.run(cmd,cwd=ROOT,env=env,capture_output=True,text=True,timeout=300)
 (out/'compile.log').write_text(r.stdout+r.stderr,encoding='utf-8')
 if r.returncode:raise RuntimeError(r.stderr)
 shutil.copy2(sdk/'mujoco.dll',out/'mujoco.dll')
def run_case(out,spec,mode,axis,sign,reference,duration,label):
 name,I,rotor,visc,coulomb,delay=spec;mode_name,kind,B=mode
 ident=f'{label}_{name}_{mode_name}_{axis}_{sign}'
 plant=REF01Dynamics();nominal=plant.dynamics_terms(reference)['M']
 plant.model.body_inertia[:]*=I
 plant.model.dof_armature[plant._active]+=rotor
 plant.model.dof_damping[plant._active]+=visc
 plant.model.dof_frictionloss[plant._active]+=coulomb
 mujoco.mj_setConst(plant.model,plant._data)
 state=plant.reset(reference);actual=plant.dynamics_terms(reference)['M']
 rows=[];failure=None;queue=[];feedback=None
 with (out/(ident+'.log')).open('w') as log:
  proc=subprocess.Popen([str(out/'probe.exe'),'mujoco/ref01/config/mujoco/ref01_dynamics_cad_v1.xml',kind,*map(str,reference),str(B)],cwd=ROOT,stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=log,text=True,bufsize=1)
  try:
   for tick in range(round(duration/.005)):
    t=tick*.005
    if tick%4==0:
     p,v=feedback_words(state['alpha'],state['alpha_velocity']);packet=(tick,p,v)
     if tick==0:feedback=packet
     else:queue.append((tick+delay//5,packet))
    while queue and queue[0][0]<=tick:feedback=queue.pop(0)[1]
    source,p,v=feedback
    if max(abs(x) for x in p+v)>32767:failure='feedback_range';break
    proc.stdin.write(' '.join(map(str,[source]+p+v))+'\n');proc.stdin.flush()
    raw=proc.stdout.readline().split()
    if len(raw)!=10:raise RuntimeError('probe failure '+ident+repr(raw))
    values=list(map(float,raw[:9]));requested=np.array([values[0],values[2]]);sent=np.array([values[1],values[3]])
    rows.append([t,*state['q'],*state['dq'],*requested,*sent,*values[4:8],(tick-source)*5,state['max_closure_m'],state['warnings'],values[8]])
    if values[8]:failure=raw[9];break
    ext=np.zeros(2)
    if .4<=t<1.:ext[axis]=sign*.3
    state=plant.step(sent[::-1],steps=10,external_torque=ext)
    if not np.isfinite(state['q']).all() or not np.isfinite(state['dq']).all():failure='nonfinite';break
    if abs(state['q'][0])>np.pi/4 or not 0<=state['q'][1]<=np.pi/4:failure='plant_domain_exit';break
    if state['warnings'] or state['max_closure_m']>=1e-4:failure='plant_warning_or_closure';break
  finally:
   proc.stdin.close()
   try:code=proc.wait(timeout=10)
   except subprocess.TimeoutExpired:proc.kill();proc.wait();raise
   if code and not failure:raise RuntimeError('native exit '+str(code))
 with (out/(ident+'.csv')).open('w',newline='') as f:
  w=csv.writer(f);w.writerow(['t','psi','beta','dpsi','dbeta','desired104','desired105','sent104','sent105','estimated_psi','estimated_beta','estimated_dpsi','estimated_dbeta','age_ms','closure','warnings','fault']);w.writerows(rows)
 a=np.array(rows);err=np.rad2deg(a[:,1:3]-reference);speed=np.rad2deg(a[:,3:5]);tail=a[:,0]>=2.5
 good=(abs(err).max(axis=1)<=.5)&(abs(speed).max(axis=1)<=1.)
 settle=np.flatnonzero((a[:,0]>=1.)&np.logical_and.accumulate(good[::-1])[::-1])
 complete=not failure and len(rows)==round(duration/.005)
 return dict(id=ident,scenario=name,mode=mode_name,axis=axis,sign=sign,reference_deg=np.rad2deg(reference).tolist(),duration=duration,complete=complete,failure=failure,
 passed=bool(complete and tail.any() and good[tail].all()),tail_error_deg=float(abs(err[tail]).max()) if tail.any() else None,tail_speed_deg_s=float(abs(speed[tail]).max()) if tail.any() else None,
 peak_error_deg=float(abs(err).max()),peak_speed_deg_s=float(abs(speed).max()),peak_motor_torque_nm=abs(a[:,7:9]).max(axis=0).tolist(),
 saturation_fraction=float((abs(a[:,5:7]).max(axis=1)>=4.999).mean()),settling_s=float(a[settle[0],0]-1) if complete and len(settle) else None,
 model_mass_diagonal_ratio=(np.diag(actual)/np.diag(nominal)).tolist(),final_error_deg=err[-1].tolist(),max_closure_m=float(a[:,14].max()),samples=len(rows))
def main():
 p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);p.add_argument('--smoke',action='store_true');p.add_argument('--long',action='store_true');args=p.parse_args()
 out=args.output.resolve();out.mkdir(parents=True,exist_ok=False);compile_probe(out)
 results=[];specs=SCENARIOS[:1] if args.smoke else SCENARIOS
 if args.long:
  specs=[s for s in SCENARIOS if s[0] in ('nominal','rotor_005','delay_10','combined_rotor')]
  jobs=[(s,m,ax,1,np.deg2rad([0,20]),30.,'long') for s in specs for m in MODES for ax in (0,1)]
 else:
  jobs=[(s,m,ax,sign,np.deg2rad([0,20]),6.,'grid') for s in specs for m in MODES for ax in (0,1) for sign in (-1,1)]
  if not args.smoke:jobs +=[(s,m,ax,sign,np.deg2rad([.55,15.288231]),6.,'recorded') for s in SCENARIOS if s[0] in ('nominal','delay_10','combined_rotor') for m in MODES for ax in (0,1) for sign in (-1,1)]
 metadata=dict(scenarios=SCENARIOS,modes=MODES,controller_h=.005,feedback_h=.02,reference_model_frozen=True,friction_units='added alpha-shaft viscous Nms/rad and native Coulomb Nm',inertia_units='body rotational inertia multiplier; added alpha-shaft armature kg m2',boundary='Windows numerical robustness grid; no ROS/device/physical stability proof',expected_cases=len(jobs))
 (out/'design.json').write_text(json.dumps(metadata,indent=2))
 for i,job in enumerate(jobs):
  r=run_case(out,*job);results.append(r)
  (out/'summary.json').write_text(json.dumps(dict(complete=len(results)==len(jobs),cases=results),indent=2))
  print(json.dumps(dict(index=i+1,total=len(jobs),id=r['id'],passed=r['passed'],failure=r['failure'],tail_speed=r['tail_speed_deg_s'])),flush=True)
 sources=['tools/project/ref01_robustness_probe.cpp','tools/project/verify_ref01_robustness.py','mujoco/ref01/cpp/ref01_model.hpp','ros2_ws/src/mech_controllers/include/mech_controllers/ref01_bench_predictor.hpp','mujoco/ref01/config/mujoco/ref01_dynamics_cad_v1.xml']
 (out/'source_hashes.json').write_text(json.dumps({s:hashlib.sha256((ROOT/s).read_bytes()).hexdigest() for s in sources},indent=2))
 print('Completed',len(results),'cases; strict recovery pass',sum(r['passed'] for r in results))
if __name__=='__main__':main()
