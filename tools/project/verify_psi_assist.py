"""Windows software regression for the measured psi assist boundary, no device I/O."""
import argparse
import csv
import json
import math
from pathlib import Path
import subprocess
import sys

import numpy as np

from ref01_dynamic_mit import current_scale
from ref01_mit_fixture import ROOT, feedback_words, LSB
from sync_orin import safe_path
from test_ref01_dynamic_mit import build

sys.path.insert(0, str(ROOT / 'mujoco/ref01'))
from ref01_dynamics_api import REF01Dynamics
from ref01_kinematics import inverse_kinematics
from ref01_mujoco_kinematics import KinematicsError, MuJoCoForwardKinematics
from fixed_pose_impedance import FixedPoseImpedance

DT = .005
K = np.array([20., 30.])
B = np.array([.3, 1.])
CAP = 10.
MODEL = ROOT / 'mujoco/ref01/config/mujoco/ref01_dynamics_cad_v1.xml'


def run(out, exe, sign, case, scale, ramp):
    dyn, fk = REF01Dynamics(), MuJoCoForwardKinematics()
    initial = np.radians([sign*44., 20.])
    plant = dyn.reset(initial)
    # Exercise the same defaults as ROS and the Python controller for the adopted policy.
    override = {} if ramp == .2 else dict(psi_ramp_s=ramp)
    mirror = FixedPoseImpedance(dyn, initial, K, B, [CAP, CAP], **override)
    name = ('positive_' if sign == 1 else 'negative_') + case + f'_{round(1000*ramp)}ms'
    dynamic = case.startswith('disturbance')
    args = [str(exe), str(MODEL), str(scale), *map(str,K), str(CAP), *map(str,B)]
    if override:
        args.append(str(ramp))
    samples, transitions = [], []
    returned = None
    with subprocess.Popen(args, stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True, bufsize=1) as proc:
        try:
            for tick in range(1001 if dynamic else 251):
                t = tick * DT
                reference = initial.copy()
                if case == 'offset':
                    # Move the valid reference to centre while the fixture is outside.
                    reference[0] *= 1. - np.clip((tick-84)/32., 0., 1.)
                reference_speed = [(-initial[0]/(32*DT) if case=='offset' and 84<tick<116 else 0.), 0.]
                if case == 'disturbance_offset':
                    reference[0] *= 1.-np.clip((t-1.1)/.35,0.,1.)
                    if 1.1<t<1.45: reference_speed[0]=-initial[0]/.35
                if dynamic:
                    pos, vel = feedback_words(plant['alpha'], plant['alpha_velocity'], scale)
                else:
                    # Controlled encoder crossing exactly at 45 and 60 degrees.
                    offset = min(tick,80,max(0,200-tick))*.2
                    psi = sign * (44.+offset)
                    center = round(math.degrees(inverse_kinematics(0.,math.radians(20.),unit='rad')[0])+45.,1)
                    common = center + .3*min(tick/10.,1.)
                    pos = [round((common+psi)*10),round((common-psi)*10)]
                    speed = sign*math.radians(40 if 0<tick<=80 else -40 if 120<tick<=200 else 0)
                    vel = [round(speed/scale/10),round(-speed/scale/10)]
                msg = [t,*pos,*vel,1,tick+1,1.005+t,*reference,*reference_speed]
                proc.stdin.write(' '.join(map(str,msg))+'\n');proc.stdin.flush()
                z = np.fromstring(proc.stdout.readline(),sep=' ')
                assert z.shape==(15,) and np.isfinite(z).all() and list(z[:3])==[2,0,1], (name,tick,z)
                alpha = np.radians(np.array(pos[::-1])*.1-45.)
                q = np.asarray(fk.solve(alpha).q)
                dq,_ = fk.velocity(alpha,np.array(vel[::-1])*10.*scale)
                assert np.allclose(np.r_[q,dq],z[4:8],atol=1e-8,rtol=0)
                terms = dyn.dynamics_terms(q)
                fb = K*(reference-q)+B*(reference_speed-dq)
                assist = abs(q[0]) <= math.pi/4+1e-8
                if not assist:
                    returned=None
                    weight=0.
                else:
                    if samples and not samples[-1]['assist']:
                        returned=t
                    u=1. if returned is None or not ramp else min((t-returned)/ramp,1.)
                    weight=u**3*(10+u*(-15+6*u))
                fb[0]*=weight
                req = np.linalg.solve(terms['A'].T, terms['bias']+fb)[::-1]
                factor=1/max(1.,max(abs(req))/CAP)
                assert np.allclose(req*factor,z[[8,11]],atol=2e-7,rtol=0), (name,tick,q,req,z)
                applied=z[[9,12]]
                # The production MIT encoder truncates to the lower of 4096 levels.
                quant_error = z[[8,11]] - applied
                assert np.all(quant_error >= -1e-10) and np.all(quant_error < LSB+1e-10)
                if 'offset' not in case:
                    assert np.allclose(mirror.command(q,dq,now=t).motor104_105_nm,z[[8,11]],atol=2e-7)
                row=dict(t=t,psi_deg=math.degrees(q[0]),beta_deg=math.degrees(q[1]),
                         ref_psi_deg=math.degrees(reference[0]),assist=assist,weight=weight,fb_psi=fb[0],fb_beta=fb[1],
                         gravity_psi=terms['bias'][0],gravity_beta=terms['bias'][1],
                         req104=z[8],req105=z[11],mit104=z[9],mit105=z[12],scale=factor)
                if samples and assist!=samples[-1]['assist']:
                    prev=samples[-1]
                    transitions.append(dict(event='return' if assist else 'exit',time=t,
                        previous_psi_deg=prev['psi_deg'],psi_deg=row['psi_deg'],
                        before=[prev['mit104'],prev['mit105']],after=list(applied),
                        step=(applied-[prev['mit104'],prev['mit105']]).tolist(),
                        req_step=[z[8]-prev['req104'],z[11]-prev['req105']]))
                samples.append(row)
                if dynamic:
                    # An external spring-damper fixture moves the free MuJoCo plant.
                    # Controller torques are actually applied through both actuators.
                    u=min(max((t-.3)/.7,0.),1.)-min(max((t-1.7)/.7,0.),1.)
                    desired=math.radians(sign*(44.+3.5*u))
                    external=45.*(desired-plant['q'][0])-.45*plant['dq'][0]
                    if t>=2.4: external=0.
                    plant=dyn.step(applied[::-1],steps=10,external_torque=[external,0.])
                    assert plant['warnings']==0 and plant['max_closure_m']<1e-4
        finally:
            proc.stdin.close()
        assert proc.wait(timeout=10)==0
    assert any(v['event']=='exit' for v in transitions) and transitions[-1]['event']=='return', (name,transitions)
    outside=[r for r in samples if not r['assist']]
    assert all(r['fb_psi']==0. for r in outside)
    assert max(abs(r['fb_beta']) for r in outside)>1e-5
    assert max(max(abs(r['gravity_beta']),abs(r['gravity_psi'])) for r in outside)>1e-6
    with (out/(name+'.csv')).open('w',newline='') as f:
        w=csv.DictWriter(f,fieldnames=samples[0]);w.writeheader();w.writerows(samples)
    first=next(e for e in transitions if e['event']=='return')
    idx=round(first['time']/DT)
    torque=np.array([[r['mit104'],r['mit105']] for r in samples])
    end=idx+round(ramp/DT)+1
    steps=np.abs(np.diff(torque[idx-1:end],axis=0))
    settle=None
    if dynamic:
        # Measured psi enters the 0.5 degree position band and stays there to end of run.
        err=np.abs([r['psi_deg']-r['ref_psi_deg'] for r in samples[idx:]])
        good=np.flatnonzero(np.maximum.accumulate(err[::-1])[::-1]<=.5)
        if good.size: settle=float(good[0])*DT
    return dict(case=name,ramp_ms=round(ramp*1000),first_step=first['step'],
                max_ramp_step=steps.max(axis=0).tolist(),settle_after_return_s=settle,
                max_200ms_step=np.abs(np.diff(torque[idx-1:idx+41],axis=0)).max(axis=0).tolist(),frames=len(samples),psi_span=[min(r['psi_deg'] for r in samples),max(r['psi_deg'] for r in samples)],
                saturated=int(sum(r['scale']<1. for r in samples)),transitions=transitions)


def limits(exe,scale):
    for sign in (-1,1):
        fk=MuJoCoForwardKinematics()
        center=round(math.degrees(inverse_kinematics(0.,math.radians(20.),unit='rad')[0])+45.,1)
        lines=[]
        for i,psi in enumerate((44.,45.,60.,60.1)):
            pos=np.rint((center+sign*np.array([psi,-psi]))*10).astype(int)
            alpha=np.radians(pos[::-1]*.1-45.)
            if psi<=60:
                assert abs(math.degrees(fk.solve(alpha).q[0])-sign*psi)<1e-7
            else:
                try:fk.solve(alpha)
                except KinematicsError:pass
                else:raise AssertionError('invalid FK accepted')
            t=i*DT
            lines.append(' '.join(map(str,[t,*pos,0,0,1,i+1,1.005+t,math.radians(sign*44.),math.radians(20.),0.,0.])))
        cmd=[str(exe),str(MODEL),str(scale),'20','30','10','.3','1']
        z=np.array([np.fromstring(l,sep=' ') for l in subprocess.check_output(cmd,input='\n'.join(lines)+'\n',text=True).splitlines()])
        assert np.all(z[:3,0]==2) and z[-1,0]==3 and z[-1,2]==0


def beta_limits(exe, scale):
    """Measured endpoints and references have separate domains."""
    fk = MuJoCoForwardKinematics()
    dynamics = REF01Dynamics()
    for psi in (-45., 0., 45.):
        for beta in (-8., -10.5, -11.):
            alpha = inverse_kinematics(psi, beta, observed_pose=True)
            q = fk.solve(np.radians(alpha)).q
            np.testing.assert_allclose(np.degrees(q), [psi, beta], atol=1e-6, rtol=0)
            assert all(np.isfinite(v).all() for v in dynamics.dynamics_terms(q).values())
    for beta, observed in [(-8.01, False), (-11.01, True)]:
        with np.testing.assert_raises(ValueError):
            inverse_kinematics(0., beta, observed_pose=observed)

    # Start from a legal reference, then move both measured encoders
    # across the lower endpoint and back without starting a new session.
    home = round((inverse_kinematics(0., -7.)[0] + 45.) * 10)
    positions = [home, 1262, 1263, 1262]
    rows = []
    for i, p in enumerate(positions):
        t = i * DT
        rows.append([t, p, p, 0, 0, 1, i+1, 1.005+t, 0., math.radians(-7.), 0., 0.])
    cmd = [str(exe), str(MODEL), str(scale), '20', '30', '10', '.3', '1']
    payload = ''.join(' '.join(map(str,row))+'\n' for row in rows)
    result = subprocess.check_output(cmd, input=payload, text=True)
    z = np.array([np.fromstring(line, sep=' ') for line in result.splitlines()])
    assert np.all(z[:2, 0] == 2) and np.all(z[:2, 2] == 1)
    np.testing.assert_allclose(math.degrees(z[1, 5]), -10.9952744, atol=1e-6)
    assert np.all(z[2:, 0] == 3) and np.all(z[2:, 2] == 0)
    assert z[2, 1] == 11  # Model fault
    # Jump directly to the folded branch, without observing the intervening invalid band.
    rows[2][1:3] = [1350, 1350]
    payload = ''.join(' '.join(map(str,row))+'\n' for row in rows)
    folded = subprocess.check_output(cmd, input=payload, text=True).splitlines()
    assert np.fromstring(folded[2], sep=' ')[:3].tolist() == [3., 11., 0.]
    assert abs(math.degrees(fk.solve(np.radians([81.2, 81.2])).q[1]) + 10.9952744) < 1e-6
    with np.testing.assert_raises(KinematicsError):
        fk.solve(np.radians([81.3, 81.3]))
    fk.solve(np.radians([81.2, 81.2]))


def upper_beta_assist(exe, scale):
    """Compare ordinary ROS-core and Python allocations across the upper assist boundary."""
    fk = MuJoCoForwardKinematics()
    model = REF01Dynamics()
    checks = []
    # Positive/inward position errors and damping-only commands on both sides.
    sequence = [(87., 0., 0), (88., 3., 0), (89., 3., 0), (90., 3., 0),
                (91., 3., 0), (91., -3., 0), (90., 0., 20), (90., 0., -20),
                (89., 3., 0), (88., 3., 0)]
    for psi in (-12., 12.):
        rows, references = [], []
        for i, (phi, error, speed) in enumerate(sequence):
            enc = np.rint(10 * (135. - phi + np.array([psi, -psi]))).astype(int)
            q = fk.solve(np.radians(enc[::-1] * .1 - 45.)).q
            ref = [q[0] + math.radians(1. if i else 0.), q[1] + math.radians(error)]
            t = i * DT
            rows.append([t, *enc, speed, speed, 1, i+1, 1.005+t, *ref, 0., 0.])
            references.append(ref)
        cmd = [str(exe), str(MODEL), str(scale), *map(str,K), str(CAP), *map(str,B)]
        payload = ''.join(' '.join(map(str,row))+'\n' for row in rows)
        lines = subprocess.check_output(cmd, input=payload, text=True).splitlines()
        assert len(lines) == len(rows)
        for (phi, _, _), ref, line in zip(sequence, references, lines):
            z = np.fromstring(line, sep=' ')
            assert list(z[:3]) == [2, 0, 1]
            q, dq = z[4:6], z[6:8]
            terms = model.dynamics_terms(q)
            raw = K*(ref-q) - B*dq
            weight = {87.:1., 88.:1., 89.:.5, 90.:0., 91.:0.}[phi]
            expected = raw.copy()
            if raw[1] > 0.:
                expected[1] *= weight
            actual = terms['A'].T @ z[[11,8]] - terms['bias']
            assert z[-1] == 1.
            np.testing.assert_allclose(actual, expected, atol=2e-7, rtol=0)
            mirror = FixedPoseImpedance(model, ref, K, B, [CAP, CAP])
            np.testing.assert_allclose(mirror.command(q,dq).motor104_105_nm, z[[8,11]], atol=2e-7, rtol=0)
            checks.append((phi, raw[1], actual[1]))
    assert any(phi == 90 and raw > 0. and abs(out) < 2e-7 for phi, raw, out in checks)
    assert any(phi == 90 and raw < 0. and abs(out-raw) < 2e-7 for phi, raw, out in checks)
    return checks


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output',required=True,type=Path)
    args=p.parse_args()
    out=safe_path(ROOT,args.output)
    if not out.is_relative_to(ROOT/'.local/validation'):
        p.error('use a new directory under .local/validation')
    out.mkdir(parents=True,exist_ok=False)
    exe=build(out);scale,_=current_scale()
    limits(exe,scale)
    beta_limits(exe,scale)
    upper_beta_assist(exe,scale)
    results=[]
    for sign in (1,-1):
        for case in ('near','offset','disturbance','disturbance_offset'):
            for ramp in (.1,.2):
                r=run(out,exe,sign,case,scale,ramp)
                results.append(r)
                print(r['case'],r['max_ramp_step'],r['settle_after_return_s'],flush=True)
    (out/'summary.json').write_text(json.dumps(dict(dt=DT,k=K.tolist(),b=B.tolist(),cap=CAP,
        valid_deg=60,assist_deg=45,reference_beta_deg=20,hardware=False,results=results),indent=2))
if __name__=='__main__':
    main()
