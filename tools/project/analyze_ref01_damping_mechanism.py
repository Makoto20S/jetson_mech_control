
"""Windows-only damping timing diagnosis. No hardware, ROS, or production gain edits."""
import argparse,csv,hashlib,json,os,subprocess,sys
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[2]
sys.path[:0]=[str(ROOT/'tools/project'),str(ROOT/'mujoco/ref01')]
from ref01_dynamics_api import REF01Dynamics
from ref01_mit_fixture import feedback_words
from verify_ref01_robustness import compile_probe
REF=np.deg2rad([4.05,16.608514])
def linear(M,T,K,B):
 return np.array([[1-K*T*T/(2*M),T-B*T*T/(2*M)],[-K*T/M,1-B*T/M]])
def run_case(out,interval,b,axis):
 plant=REF01Dynamics();state=plant.reset(REF);rows=[];failure=None;ident=f'T{interval*5}_B{b:g}_axis{axis}'
 with (out/(ident+'.log')).open('w') as log:
  proc=subprocess.Popen([str(out/'probe.exe'),str(ROOT/'mujoco/ref01/config/mujoco/ref01_dynamics_cad_v1.xml'),'ordinary',*map(str,REF),str(b)],
   cwd=ROOT,stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=log,text=True,bufsize=1)
  try:
   for tick in range(1200):
    t=tick*.005
    if tick%interval==0:
     p,v=feedback_words(state['alpha'],state['alpha_velocity']);source=tick
    proc.stdin.write(' '.join(map(str,[source]+p+v))+'\n');proc.stdin.flush()
    raw=proc.stdout.readline().split();assert len(raw)==10,raw
    values=np.array(list(map(float,raw[:9])));sent=values[[1,3]]
    rows.append([t,*state['q'],*state['dq'],values[0],values[2],*sent,*values[4:8],(tick-source)*5,state['max_closure_m'],state['warnings'],values[8]])
    if values[8]:failure=raw[9];break
    ext=np.zeros(2)
    if .4<=t<1:ext[axis]=.3
    state=plant.step(sent[::-1],steps=10,external_torque=ext)
    if not np.isfinite(state['q']).all() or not np.isfinite(state['dq']).all():failure='nonfinite';break
    if abs(state['q'][0])>np.pi/4 or not 0<=state['q'][1]<=np.pi/4:failure='plant_domain_exit';break
    if state['warnings'] or state['max_closure_m']>=1e-4:failure='plant_warning_or_closure';break
  finally:
   proc.stdin.close();code=proc.wait(timeout=15);assert code==0
 cols=['t','psi','beta','dpsi','dbeta','desired104','desired105','sent104','sent105','estimated_psi','estimated_beta','estimated_dpsi','estimated_dbeta','age_ms','closure','warnings','fault']
 with (out/(ident+'.csv')).open('w',newline='') as f:
  w=csv.writer(f);w.writerow(cols);w.writerows(rows)
 a=np.array(rows);e=np.rad2deg(a[:,1:3]-REF);v=np.rad2deg(a[:,3:5]);tail=a[:,0]>=2.5
 complete=len(a)==1200 and failure is None
 passed=bool(complete and np.all(abs(e[tail])<=.5) and np.all(abs(v[tail])<=1))
 return dict(id=ident,feedback_ms=interval*5,B=b,axis=axis,samples=len(a),complete=complete,passed=passed,failure=failure,
  tail_error_deg=float(abs(e[tail]).max()) if tail.any() else None,tail_speed_deg_s=float(abs(v[tail]).max()) if tail.any() else None,
  peak_torque_nm=abs(a[:,7:9]).max(axis=0).tolist(),peak_speed_deg_s=float(abs(v).max()))
def main():
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
 out=a.output.resolve();out.mkdir(parents=True,exist_ok=False)
 model=REF01Dynamics();terms=model.dynamics_terms(REF);M=terms['M'];eig=np.linalg.eigvalsh(M);A=terms['A']
 linear_rows=[]
 for T in (.005,.01,.02):
  for B in (0,.1,.2,.4,.6,1):
   for i,m in enumerate(eig):
    F=linear(m,T,10,B);rho=float(max(abs(np.linalg.eigvals(F))))
    jury=10*T/2 < B < 2*m/T
    if abs(rho-1)>1e-9:assert (rho<1)==jury
    linear_rows.append(dict(T_ms=T*1000,B=B,mode=i,mass=float(m),rho=rho,stable=bool(rho<1-1e-9)))
 # Compare each discrete matrix with integration under a held command.
 checks=[]
 for m in eig:
  for T in (.005,.01,.02):
   x=np.array([.01,.02]);u=-10*x[0]-.2*x[1]
   z=x.copy();dt=T/100
   for _ in range(100):z+=np.array([z[1]*dt+.5*u/m*dt*dt,u/m*dt])
   error=float(max(abs(z-linear(m,T,10,.2)@x)));assert error<1e-12;checks.append(error)
 summary=dict(reference_deg=np.rad2deg(REF).tolist(),M=M.tolist(),M_eigen=eig.tolist(),A=A.tolist(),
  task_B1_motor_damping_matrix=(np.linalg.inv(A).T@np.linalg.inv(A)).tolist(),linear_cases=linear_rows,
  linear_integration_max_error=max(checks),plant_cases=[],boundary='Model-based timing ablation; feedback rates are hypothetical, not observed hardware capability.')
 (out/'summary.json').write_text(json.dumps(summary,indent=2))
 compile_probe(out)
 for interval in (1,2,4):
  for b in (.2,1.):
   for axis in (0,1):
    r=run_case(out,interval,b,axis);summary['plant_cases'].append(r)
    (out/'summary.json').write_text(json.dumps(summary,indent=2));print(json.dumps(r),flush=True)
 # Independent CSV recomputation of every reported pass and tail metric.
 count=0
 for r in summary['plant_cases']:
  d=np.genfromtxt(out/(r['id']+'.csv'),delimiter=',',names=True);count+=len(d)
  e=np.rad2deg(np.column_stack([d['psi'],d['beta']])-REF);v=np.rad2deg(np.column_stack([d['dpsi'],d['dbeta']]))
  tail=d['t']>=2.5
  ok=bool(r['complete'] and np.all(abs(e[tail])<=.5) and np.all(abs(v[tail])<=1))
  assert ok==r['passed']
  if tail.any():assert abs(float(abs(v[tail]).max())-r['tail_speed_deg_s'])<1e-10
 summary['independent_validation']=dict(csv_rows=count,cases=12,passed=True)
 (out/'summary.json').write_text(json.dumps(summary,indent=2))
 files=[Path(__file__),ROOT/'tools/project/ref01_robustness_probe.cpp',ROOT/'tools/project/verify_ref01_robustness.py',ROOT/'ros2_ws/src/mech_controllers/include/mech_controllers/ref01_impedance_core.hpp',ROOT/'mujoco/ref01/config/mujoco/ref01_dynamics_cad_v1.xml',ROOT/'mujoco/ref01/cpp/ref01_model.hpp',ROOT/'mujoco/ref01/ref01_dynamics_api.py']
 (out/'source_hashes.json').write_text(json.dumps({str(f):hashlib.sha256(f.read_bytes()).hexdigest() for f in files},indent=2))
 os.environ['MPLCONFIGDIR']=str(out/'mplconfig')
 import matplotlib
 matplotlib.use('Agg')
 import matplotlib.pyplot as plt
 fig,axs=plt.subplots(2,2,figsize=(11,8),sharex=True)
 for ms in (5,10,20):
  for axis in (0,1):
   d=np.genfromtxt(out/f'T{ms}_B1_axis{axis}.csv',delimiter=',',names=True)
   axs[axis,0].plot(d['t'],np.rad2deg(d['psi' if axis==0 else 'beta'])-np.rad2deg(REF[axis]),label=f'{ms} ms')
   axs[axis,1].plot(d['t'],d['sent104' if axis==0 else 'sent105'],label=f'{ms} ms')
 for axis in (0,1):
  axs[axis,0].set_ylabel(('psi' if axis==0 else 'beta')+' error [deg]')
  axs[axis,1].set_ylabel(('104' if axis==0 else '105')+' command [Nm]')
  for col in (0,1):axs[axis,col].grid(alpha=.3);axs[axis,col].legend()
 axs[1,0].set_xlabel('Model time [s]');axs[1,1].set_xlabel('Model time [s]')
 fig.suptitle('PH-EXO: same K=10, B=1; change feedback period only\nNominal model + MIT quantization; early curve termination = failure')
 fig.tight_layout();fig.savefig(out/'same_gains_different_feedback.png',dpi=160)
 print('Independent CSV and linear matrix checks passed',count,flush=True)
if __name__=='__main__':main()
