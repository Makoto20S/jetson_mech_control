
"""Windows numerical timing margin experiment. Frozen adaptive core, causal alignment."""
import argparse,csv,hashlib,json,os,random,shutil,subprocess
from pathlib import Path
import numpy as np
import mujoco
import verify_ref01_robustness as base
ROOT=Path(__file__).resolve().parents[2]
# delay/jitter/phase/source-period jitter, all in 5 ms ticks.
TIMINGS=[
 ("sync_0",0,0,0,0,True),("sync_20",4,0,0,0,True),
 ("shared_20_j5",4,1,0,0,True),("shared_20_j10",4,2,0,0,True),
 ("independent_10_j5",2,1,0,0,False),("independent_20_j5",4,1,0,0,False),
 ("independent_20_j10",4,2,0,0,False),
 ("phase5_0",0,0,1,0,False),("phase10_0",0,0,2,0,False),
 ("phase15_0",0,0,3,0,False),("phase10_20",4,0,2,0,False),
 ("phase15_20",4,0,3,0,False),
 ("phase10_10_j5",2,1,2,0,False),("phase10_20_j10",4,2,2,0,False),
 ("independent_30_j10",6,2,0,0,False),("phase10_40",8,0,2,0,False),
 ("expiry_45_j10",9,2,2,0,False),("source_j5_delay10",2,0,0,1,False)]
PLANTS=[s for s in base.SCENARIOS if s[0] in ("nominal","body_I_half","rotor_020","combined_rotor")]
# The original plant tuple's transport delay is replaced by TIMINGS, not added twice.
def compile_probe(out):
 sdk=Path(mujoco.__file__).parent
 cmd=[str(ROOT/'.local/toolchains/zig-0.13.0/ziglang/zig.exe'),'c++','-std=c++17','-O2','-DNDEBUG','-Wall','-Wextra','-Werror']
 for d in ['ros2_ws/src/mech_controllers/include/mech_controllers','ros2_ws/src/mech_protocol_cubemars/include','mujoco/ref01/cpp']:
  cmd+=['-I'+str(ROOT/d)]
 cmd+=['-I'+str(sdk/'include'),str(ROOT/'tools/project/ref01_feedback_timing_probe.cpp'),
       str(ROOT/'ros2_ws/src/mech_protocol_cubemars/src/ak30_force_wire.cpp'),str(sdk/'mujoco.dll'),'-o',str(out/'probe.exe')]
 env=dict(os.environ,ZIG_GLOBAL_CACHE_DIR=str(ROOT/'.local/cache/zig'),ZIG_LOCAL_CACHE_DIR=str(ROOT/'.local/cache/zig/local'))
 r=subprocess.run(cmd,capture_output=True,text=True,env=env,timeout=300)
 (out/'compile.log').write_text(r.stdout+r.stderr)
 if r.returncode:raise RuntimeError(r.stderr)
 shutil.copy2(sdk/'mujoco.dll',out/'mujoco.dll')

def run_case(out,spec,timing,axis,sign,seed,duration=6.):
 name,I,rotor,visc,coulomb,_=spec
 tname,delay,jitter,phase,source_jitter,shared=timing
 ident=f'{name}__{tname}__a{axis}_s{sign}_r{seed}'
 reference=np.deg2rad([0.,20.]);plant=base.REF01Dynamics()
 plant.model.body_inertia[:]*=I
 plant.model.dof_armature[plant._active]+=rotor
 plant.model.dof_damping[plant._active]+=visc
 plant.model.dof_frictionloss[plant._active]+=coulomb
 mujoco.mj_setConst(plant.model,plant._data)
 state=plant.reset(reference);queue=[];rows=[];failure=None;events=[]
 # Separate PRNG per stream; reproducible schedules independent of plant response.
 rng=[random.Random(seed),random.Random(seed+100003)]
 common=random.Random(seed+200003)
 next_sample=[4,phase if phase else 4];p,v=base.feedback_words(state['alpha'],state['alpha_velocity'])
 latest=[(0,p[0],v[0]),(0,p[1],v[1])]
 consumed=0;discarded=0
 with (out/(ident+'.log')).open('w') as log:
  proc=subprocess.Popen([str(out/'probe.exe'),'mujoco/ref01/config/mujoco/ref01_dynamics_cad_v1.xml',
    'predict',*map(str,reference),'1.'],cwd=ROOT,stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=log,text=True,bufsize=1)
  try:
   for tick in range(round(duration/.005)):
    t=tick*.005
    common_delay=max(0,delay+common.randint(-jitter,jitter)) if shared and tick%4==0 else None
    for motor in (0,1):
     if tick==next_sample[motor]:
      p,v=base.feedback_words(state['alpha'],state['alpha_velocity'])
      latency=common_delay if common_delay is not None else max(0,delay+rng[motor].randint(-jitter,jitter))
      queue.append((tick+latency,motor,tick,p[motor],v[motor]))
      next_sample[motor]+=4+rng[motor].randint(-source_jitter,source_jitter)
    remaining=[]
    for due,motor,source,pos,vel in queue:
     if due<=tick:
      accepted=source>latest[motor][0]
      if accepted:latest[motor]=(source,pos,vel);consumed+=1
      else:discarded+=1
      events.append((tick,motor,source,due,int(accepted)))
     else:remaining.append((due,motor,source,pos,vel))
    queue=remaining
    proc.stdin.write(' '.join(map(str,[*latest[0],*latest[1]]))+'\n');proc.stdin.flush()
    raw=proc.stdout.readline().split()
    if len(raw)!=15:raise RuntimeError(f'probe output {ident}: {raw}')
    values=list(map(float,raw[:9]));requested=[values[0],values[2]];sent=np.array([values[1],values[3]])
    source,age0,age1,width0,width1=map(int,raw[10:])
    rows.append([t,*state['q'],*state['dq'],*requested,*sent,*values[4:8],(tick-source)*5,
                 state['max_closure_m'],state['warnings'],values[8],
                 latest[0][0]*.005,latest[1][0]*.005,source*.005,age0*5,age1*5,width0*5,width1*5])
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
   if code and not failure:raise RuntimeError('probe exit '+str(code))
 with (out/(ident+'.csv')).open('w',newline='') as f:
  w=csv.writer(f);w.writerow(['t','psi','beta','dpsi','dbeta','desired104','desired105','sent104','sent105',
   'estimated_psi','estimated_beta','estimated_dpsi','estimated_dbeta','age_ms','closure','warnings','fault',
   'source104','source105','aligned_source','age104_ms','age105_ms','bracket104_ms','bracket105_ms']);w.writerows(rows)
 with (out/(ident+'_events.csv')).open('w',newline='') as f:
  w=csv.writer(f);w.writerow(['arrival_tick','motor_index','source_tick','scheduled_arrival_tick','accepted']);w.writerows(events)
 a=np.array(rows);err=np.rad2deg(a[:,1:3]-reference);speed=np.rad2deg(a[:,3:5]);tail=a[:,0]>=2.5
 good=(abs(err).max(axis=1)<=.5)&(abs(speed).max(axis=1)<=1.)
 complete=not failure and len(rows)==round(duration/.005)
 passed=bool(complete and tail.any() and good[tail].all())
 settle=np.flatnonzero((a[:,0]>=1.)&np.logical_and.accumulate(good[::-1])[::-1])
 category='pass' if passed else ('freshness_stop' if failure in ('alignment_missing_or_expired','source_invalid_or_expired') else ('abort' if failure else 'recovery_fail'))
 return dict(id=ident,plant=name,timing=tname,axis=axis,sign=sign,seed=seed,duration=duration,reference_deg=[0.,20.],
  passed=passed,complete=complete,failure=failure,category=category,samples=len(rows),end_s=float(a[-1,0]),
  tail_error_deg=float(abs(err[tail]).max()) if tail.any() else None,
  tail_speed_deg_s=float(abs(speed[tail]).max()) if tail.any() else None,
  peak_motor_torque_nm=abs(a[:,7:9]).max(axis=0).tolist(),
  max_age_ms=float(a[:,13].max()),max_skew_ms=float(abs(a[:,17]-a[:,18]).max()*1000),
  max_bracket_ms=float(a[:,22:24].max()),discarded_old_packets=discarded,accepted_packets=consumed,
  settling_s=float(a[settle[0],0]-1) if complete and len(settle) else None)

def main():
 p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True)
 p.add_argument('--smoke',action='store_true');p.add_argument('--long',action='store_true')
 args=p.parse_args();out=args.output.resolve();out.mkdir(parents=True,exist_ok=False);compile_probe(out)
 sources=['tools/project/ref01_feedback_alignment.hpp','tools/project/ref01_feedback_timing_probe.cpp',
 'tools/project/verify_ref01_feedback_timing.py','tools/project/ref01_pair_quantization.hpp',
 'ros2_ws/src/mech_controllers/include/mech_controllers/ref01_adaptive_predictor.hpp',
 'mujoco/ref01/cpp/ref01_model.hpp','mujoco/ref01/ref01_dynamics_api.py']
 hashes={s:hashlib.sha256((ROOT/s).read_bytes()).hexdigest() for s in sources}
 (out/'source_hashes.json').write_text(json.dumps(hashes,indent=2))
 if args.smoke:
  jobs=[(s,t,a,1,11,6.) for s in PLANTS if s[0] in ('nominal','combined_rotor') for t in TIMINGS if t[0] in ('sync_0','sync_20','phase10_0','phase10_20_j10') for a in (0,1)]
 elif args.long:
  jobs=[(s,t,a,1,seed,30.) for s in PLANTS if s[0] in ('nominal','combined_rotor')
    for t in TIMINGS if t[0] in ('shared_20_j10','independent_20_j10','phase10_0','phase10_20_j10') for a in (0,1) for seed in (11,47)]
 else:
  jobs=[(s,t,a,sign,seed,6.) for s in PLANTS for t in TIMINGS for a in (0,1) for sign in (-1,1)
        for seed in ((11,23,47) if t[2] or t[4] else (11,))]
 design=dict(timings=TIMINGS,plants=PLANTS,expected_cases=len(jobs),step_s=.005,plant_step_s=.0005,
  source_period_s=.020,startup='two simultaneous t0 samples immediately available',
  alignment='causal linear interpolation of decoded channels to min(latest source times)',
  boundary='Windows only; frozen controller; synthetic timestamps; no ROS/devices')
 (out/'design.json').write_text(json.dumps(design,indent=2))
 results=[]
 for i,job in enumerate(jobs):
  r=run_case(out,*job);results.append(r)
  (out/'summary.json').write_text(json.dumps(dict(complete=len(results)==len(jobs),cases=results),indent=2))
  print(i+1,len(jobs),r['id'],r['category'],r['failure'],r['tail_speed_deg_s'],flush=True)
 assert hashes=={s:hashlib.sha256((ROOT/s).read_bytes()).hexdigest() for s in sources}
 print('Experiment completed; recovery passed',sum(r['passed'] for r in results),'/',len(results))
if __name__=='__main__':main()
