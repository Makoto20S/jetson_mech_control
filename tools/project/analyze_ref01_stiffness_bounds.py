"""Offline PH-EXO torque-capacity sweep. Does not change gains or access devices."""
import argparse, csv, hashlib, itertools, json, math, os, platform, sys, time
from datetime import datetime
from pathlib import Path
import numpy as np
ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'tools/project'))
from sync_orin import safe_path
sys.path.insert(0, str(ROOT / 'mujoco/ref01'))
from ref01_dynamics_api import REF01Dynamics
from ref01_kinematics import inverse_kinematics

def digest(p):
    return hashlib.sha256(p.read_bytes()).hexdigest()

def dump(p, obj):
    p.write_text(json.dumps(obj, ensure_ascii=False, indent=2, allow_nan=False), encoding='utf-8')

def csv_write(p, rows):
    with p.open('w', newline='', encoding='utf-8-sig') as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        w.writeheader(); w.writerows(rows)

def bound(H, tg, error, gains, damping, speed):
    return np.abs(tg) + np.abs(H) @ (gains*error + damping*speed)

def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--output', required=True, help='New directory relative to repository')
    ap.add_argument('--historical-dir', help='Explicit read-only historical grid directory')
    ap.add_argument('--psi-error-deg', type=float, default=24.)
    ap.add_argument('--beta-error-deg', type=float, default=10.)
    args = ap.parse_args()
    errors = np.radians([args.psi_error_deg, args.beta_error_deg])
    if not np.isfinite(errors).all() or np.any(errors<=0):
        ap.error('Errors must be finite and positive')
    out=safe_path(ROOT,args.output)
    if out.exists(): raise RuntimeError('Use a new output directory; preserve previous results')
    out.mkdir(parents=True)
    os.environ['MPLCONFIGDIR']=str(out/'mplconfig')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    plt.rcParams.update({'font.family':'Microsoft YaHei','axes.unicode_minus':False,'font.size':10})
    start=time.perf_counter()
    files=[Path(__file__),ROOT/'mujoco/ref01/ref01_dynamics_api.py',
           ROOT/'mujoco/ref01/ref01_kinematics.py',ROOT/'mujoco/ref01/tools/ref01_v1.py',
           ROOT/'mujoco/ref01/fixed_pose_impedance.py',
           ROOT/'ros2_ws/src/mech_controllers/include/mech_controllers/ref01_impedance_core.hpp',
           ROOT/'mujoco/ref01/config/mujoco/ref01_dynamics_cad_v1.xml']
    files += sorted((ROOT/'mujoco/ref01/config/mujoco/assets/ref01_cad_v1').glob('*.stl'))
    source_hashes={p.relative_to(ROOT).as_posix():digest(p) for p in files}
    historical={}
    provenance=[]
    if args.historical_dir:
        source=Path(args.historical_dir)
        (out/'sources').mkdir()
        for name in ['pose_metrics.csv','summary.json','报告.md']:
            p=source/name
            before=digest(p); data=p.read_bytes(); after=digest(p)
            assert before==after==hashlib.sha256(data).hexdigest()
            target=safe_path(ROOT,(out/'sources'/name).relative_to(ROOT).as_posix())
            target.write_bytes(data)
            provenance.append(dict(source_project='PH-EXO Mujoco-Win historical model analysis',
              original_path=str(p),source_modified_at=datetime.fromtimestamp(p.stat().st_mtime).astimezone().isoformat(),
              read_at=datetime.now().astimezone().isoformat(),sha256=before,
              local_copy=target.relative_to(ROOT).as_posix(),
              purpose='Explicit comparison with 1026 historical ideal torque-transmission poses',
              evidence_scope='Ideal geometry; not measured motor torque, efficiency or closed-loop stability',
              derived_changes='None in copied inputs; recompute using current canonical model'))
        with (out/'sources/pose_metrics.csv').open(encoding='utf-8-sig') as f:
            historical={(float(r['psi_deg']),float(r['beta_deg'])):r for r in csv.DictReader(f)}
    api=REF01Dynamics()
    psi_grid=np.arange(-45.,45.001,1.)
    beta_grid=np.arange(-8.,45.001,.25)
    pose=[]; Hs=[]; gs=[]; ks=[]
    historical_error=0.; historical_count=0; fd_error=0.; pattern_error=0.
    for psi in psi_grid:
        for beta in beta_grid:
            q=np.radians([psi,beta]); terms=api.dynamics_terms(q); A=terms['A']
            H=np.linalg.inv(A.T)[::-1]; g=H@terms['bias']; k=-A[0,1]
            pattern_error=max(pattern_error,float(np.max(np.abs(A-[[-1,-k],[1,-k]]))))
            old=historical.get((psi,beta))
            if old:
                compared=[(k,float(old['k'])),(g[0],float(old['gravity_motor2_Nm'])),
                          (g[1],float(old['gravity_motor1_Nm']))]
                historical_error=max(historical_error,max(abs(a-b) for a,b in compared))
                historical_count+=1
                if psi in [-40,0,40] and beta in [-7,0,18,30,44]:
                    eps=1e-6
                    fd=np.column_stack([(np.array(inverse_kinematics(*(q+np.eye(2)[j]*eps),unit='rad'))-
                        inverse_kinematics(*(q-np.eye(2)[j]*eps),unit='rad'))/(2*eps) for j in range(2)])
                    fd_error=max(fd_error,float(np.max(abs(A-fd))))
            coeff=np.abs(H)@errors
            maxk=float(np.min((10-np.abs(g))/coeff))
            pose.append(dict(psi_deg=float(psi),beta_deg=float(beta),k=float(k),
                gravity104_nm=float(g[0]),gravity105_nm=float(g[1]),
                h104_psi=float(H[0,0]),h104_beta=float(H[0,1]),
                h105_psi=float(H[1,0]),h105_beta=float(H[1,1]),
                pure_psi_gross_nm=20.,pure_beta_gross_nm=float(20*k),
                equal_K_10Nm_B0=maxk))
            Hs.append(H);gs.append(g);ks.append(k)
    H=np.asarray(Hs);g=np.asarray(gs);k=np.asarray(ks);absh=np.abs(H)
    coefficient=absh*errors
    joint_capacity=np.sum(coefficient,axis=2)
    def worst(K, B=(0.,0.), V=(0.,0.)):
        demands=np.abs(g)+np.einsum('nij,j->ni',absh,np.asarray(K)*errors+np.asarray(B)*np.asarray(V))
        idx=np.unravel_index(int(demands.argmax()),demands.shape)
        return float(demands[idx]),dict(psi_deg=pose[idx[0]]['psi_deg'],beta_deg=pose[idx[0]]['beta_deg'],
                                      motor=[104,105][idx[1]])
    def limit(cap,B=(0.,0.),V=(0.,0.),reserve=0.):
        budgets=cap-np.abs(g)-np.einsum('nij,j->ni',absh,np.asarray(B)*np.asarray(V))-reserve
        limits=budgets/joint_capacity
        idx=np.unravel_index(int(limits.argmin()),limits.shape)
        return float(limits[idx]),dict(psi_deg=pose[idx[0]]['psi_deg'],beta_deg=pose[idx[0]]['beta_deg'],
                                     motor=[104,105][idx[1]])
    # Independent sign-corner enumeration checks the robust box formula.
    rng=np.random.default_rng(20261114); corner_error=0.; virtual_power_error=0.
    for idx in rng.choice(len(pose),size=200,replace=False):
        K=rng.uniform(1,45,2);B=rng.uniform(0,.6,2);V=np.radians([60,60])
        expected=bound(H[idx],g[idx],errors,K,B,V)
        actual=np.max([np.abs(g[idx]+H[idx]@(K*errors*np.array(es)-B*V*np.array(vs)))
                    for es in itertools.product([-1,1],repeat=2)
                    for vs in itertools.product([-1,1],repeat=2)],axis=0)
        corner_error=max(corner_error,float(np.max(abs(expected-actual))))
        v=rng.normal(size=2);tau=rng.normal(size=2)
        A=np.linalg.inv(H[idx][::-1]).T
        virtual_power_error=max(virtual_power_error,abs(float(tau@(A@v)-(A.T@tau)@v)))
    assert pattern_error<1e-10 and corner_error<1e-10 and virtual_power_error<1e-10
    if historical:
        assert historical_count==len(historical)==1026 and historical_error<1e-8 and fd_error<1e-7
    tiers=[]
    # Tiers describe static torque capacity, not a stability-approved gain setting.
    for cap in [2.,4.,6.,8.,10.]:
        value,where=limit(cap)
        tiers.append(dict(motor_cap_nm=cap,equal_K_static_boundary=value,limiting_pose=where))
    sensitivity=[]
    for B in [0.,.1,.2,.4,.6,1.]:
        for speed in [0.,30.,60.,120.]:
            value,where=limit(10,[B,B],np.radians([speed,speed]))
            sensitivity.append(dict(B_each=B,speed_bound_each_deg_s=speed,equal_K_boundary=value,**where))
    envelope=[]
    for E in [(5,5),(10,10),(20,20),(30,30),(args.psi_error_deg,args.beta_error_deg)]:
        coeff=np.einsum('nij,j->ni',absh,np.radians(E))
        envelope.append(dict(psi_error_deg=E[0],beta_error_deg=E[1],
           equal_K_static_boundary=float(np.min((10-np.abs(g))/coeff))))
    candidate=[]
    for pair in [(10,10),(15,15),(20,20),(25,25),(30,30),(35,35),(40,40),(25,50),(30,50),(40,20)]:
        peak,where=worst(pair)
        candidate.append(dict(Kpsi=pair[0],Kbeta=pair[1],peak_motor_nm=peak,margin_to_10Nm=10-peak,**where))
    # Conservative separated maxima for a readable workspace-wide sampled inequality.
    c=np.max(coefficient,axis=(0,1));G=float(np.max(np.abs(g)))
    split_K=(10-G)/sum(c)
    equal,where=limit(10)
    feasible_rows=[]
    for Kpsi in np.linspace(0,50,501):
        for cap in [2.,4.,6.,8.,10.]:
            kb=float(np.min((cap-np.abs(g)-coefficient[:,:,0]*Kpsi)/coefficient[:,:,1]))
            feasible_rows.append(dict(motor_cap_nm=cap,Kpsi=float(Kpsi),Kbeta_max=kb))
    csv_write(out/'pose_metrics.csv',pose);csv_write(out/'tiers.csv',tiers)
    csv_write(out/'damping_speed_sensitivity.csv',sensitivity)
    csv_write(out/'error_envelopes.csv',envelope);csv_write(out/'candidate_gains.csv',candidate)
    csv_write(out/'K_feasible_boundaries.csv',feasible_rows)
    summary=dict(created_at=datetime.now().astimezone().isoformat(),host=platform.node(),platform=platform.platform(),
       scope='Offline static ideal-model torque capacity; no hardware, deployment, gain changes or stability certification',
       errors_deg=[args.psi_error_deg,args.beta_error_deg],motor_cap_nm=10,
       motor_order=[104,105],alpha_to_motor_order=[105,104],
       interpretation='Maximum absolute simultaneous error envelope; user gait-phase estimate, not measured gait data',
       pose_count=len(pose),psi_grid=[-45,45,1],beta_grid=[-8,45,.25],
       reference_beta_domain_deg=[0,45],model_beta_domain_deg=[-8,45],
       model=api.metadata,equal_K_static_boundary=equal,limiting_pose=where,
       equal_K_reference_domain_boundary=float(min(r['equal_K_10Nm_B0'] for r in pose if r['beta_deg']>=0)),
       separated_maximum_inequality=dict(Kpsi_coefficient=float(c[0]),Kbeta_coefficient=float(c[1]),
           max_gravity_motor_nm=G,remaining_nm=10-G,equal_K_boundary=float(split_K)),
       pure_axis_K_intercepts=[float(np.min((10-np.abs(g))/coefficient[:,:,j])) for j in range(2)],
       k_range=[float(min(k)),float(max(k))],tiers=tiers,candidate_gains=candidate,
       error_envelopes=envelope,damping_speed_sensitivity=sensitivity,
       validation=dict(historical_poses=historical_count,historical_max_difference=historical_error,
           IK_finite_difference_error=fd_error,symmetry_error=pattern_error,
           corner_checks=200*16,corner_max_error=corner_error,virtual_power_error=virtual_power_error),
       source_hashes=source_hashes,historical_sources=provenance,
       source_hashes_unchanged=all(digest(ROOT/name)==value for name,value in source_hashes.items()),
       historical_sources_unchanged=all(digest(Path(r['original_path']))==r['sha256'] for r in provenance),
       elapsed_s=time.perf_counter()-start)
    assert summary['source_hashes_unchanged'] and summary['historical_sources_unchanged']
    dump(out/'summary.json',summary)
    shape=(len(psi_grid),len(beta_grid))
    fig,axs=plt.subplots(1,2,figsize=(13,5.5),layout='constrained')
    titles=['纯 β 理想输出力矩上限（不含重力）','两轴相同 K 的静态边界（含重力，B=0）']
    arrays=[np.array([r['pure_beta_gross_nm'] for r in pose]),
            np.array([r['equal_K_10Nm_B0'] for r in pose])]
    for ax,values,title,unit in zip(axs,arrays,titles,['N·m','N·m/rad']):
        im=ax.pcolormesh(psi_grid,beta_grid,values.reshape(shape).T,shading='auto',cmap='viridis')
        ax.axhline(0,color='white',ls='--',lw=1)
        ax.set(xlabel='当前 ψ（°）',ylabel='当前 β（°）',title=title)
        fig.colorbar(im,ax=ax,label=unit)
    fig.suptitle(f'PH-EXO｜每台电机 ≤10 N·m；最大误差 ψ ±{args.psi_error_deg:g}°、β ±{args.beta_error_deg:g}°\n虚线以下为模型扩展域，当前保持参考 β 仍须 ≥0°',fontsize=12)
    fig.savefig(out/'torque_K_heatmaps.png',dpi=180);plt.close(fig)
    fig,ax=plt.subplots(figsize=(9,6),layout='constrained')
    colors=['#9aa7b3','#45a2a5','#3e7cb1','#7852a9','#ca733c']
    for cap,color in zip([2.,4.,6.,8.,10.],colors):
        rr=[r for r in feasible_rows if r['motor_cap_nm']==cap and r['Kbeta_max']>=0]
        xx=[r['Kpsi'] for r in rr];yy=[r['Kbeta_max'] for r in rr]
        ax.plot(xx,yy,color=color,lw=2,label=f'每台 {cap:g} N·m')
    ax.plot([0,45],[0,45],'k--',alpha=.6,label='Kψ = Kβ')
    for K in [10,20,30]:
        ax.scatter(K,K,color='#b63232',s=30,zorder=5)
        ax.annotate(f'({K:g}, {K:g})',(K,K),xytext=(7,4),textcoords='offset points',fontsize=9)
    ax.scatter(equal,equal,marker='x',color='black',s=50,zorder=5,label='10 N·m 静态边界')
    ax.set(xlim=(0,50),ylim=(0,165),xlabel='Kψ（N·m/rad）',ylabel='Kβ（N·m/rad）',
           title=f'PH-EXO｜两轴 K 必须共同分配电机力矩\n各曲线下方可行；包含模型重力，B=0，误差 ψ ±{args.psi_error_deg:g}° / β ±{args.beta_error_deg:g}°')
    ax.grid(alpha=.2);ax.legend(loc='upper right');fig.savefig(out/'K_feasible_region.png',dpi=180);plt.close(fig)
    artifacts={p.relative_to(out).as_posix():digest(p) for p in out.rglob('*') if p.is_file() and 'mplconfig' not in p.parts}
    dump(out/'artifacts_sha256.json',artifacts)
    print(json.dumps({k:summary[k] for k in ['pose_count','equal_K_static_boundary','limiting_pose',
          'separated_maximum_inequality','pure_axis_K_intercepts','tiers','validation','elapsed_s']},ensure_ascii=False))
if __name__=='__main__':main()
