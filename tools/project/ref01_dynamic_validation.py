"""Shared dynamic-reference trajectory and software-only acceptance (no device I/O)."""
import math
import numpy as np

HOME = np.deg2rad([0.,20.])
K = [2.4,2.]
B = [.18,.15]
CAP = [.2,.2]
DURATION = 12.

def reference(t, axis):
    q=HOME.copy()
    dq=np.zeros(2)
    # Start/end at rest. Three degree excursion, six second round trip.
    if 1. < t < 7.:
        w=2.*math.pi/6.
        u=w*(t-1.)
        q[axis]+=math.radians(3.)*.5*(1.-math.cos(u))
        dq[axis]=math.radians(3.)*.5*w*math.sin(u)
    return q,dq

def assess_trace(rows):
    a=np.asarray(rows,dtype=float)  # t,q(2),dq(2),ref(2),vref(2),motor(2),closure
    err=np.rad2deg(a[:,1:3]-a[:,5:7])
    moving=(a[:,0]>=1.) & (a[:,0]<=7.)
    tail=a[:,0]>=9.6
    checks=dict(finite=bool(np.isfinite(a).all()),
        trajectory_exercised=bool(np.ptp(a[:,5:7],axis=0).max()>math.radians(2.9)),
        tracking_rmse=bool(moving.any() and np.sqrt(np.mean(err[moving]**2))<.15),
        restores_both_axes=bool(tail.any() and np.abs(err[tail]).max()<=.05 and
                               np.abs(np.rad2deg(a[tail,3:5])).max()<=.25),
        motor_cap=bool(np.abs(a[:,9:11]).max()<=.2+1e-10),
        closure=bool(np.abs(a[:,11]).max()<1e-4))
    return dict(passed=all(checks.values()),checks=checks,
        tracking_rmse_deg=float(np.sqrt(np.mean(err[moving]**2))) if moving.any() else None,
        max_error_deg=float(np.abs(err).max()),
        tail_error_deg=float(np.abs(err[tail]).max()) if tail.any() else None,
        peak_motor_nm=np.max(np.abs(a[:,9:11]),axis=0).tolist(),
        max_step_nm=np.max(np.abs(np.diff(a[:,9:11],axis=0)),axis=0).tolist())

class DynamicRun:
    """Used only by the isolated official-manager software runner."""
    def __init__(self,case):
        if case not in ('psi','beta','drop','replay','nan','gain','session','stop'):
            raise ValueError('unknown dynamic case')
        self.case=case
        self.started=None
        self.last=None
        self.events=[]
        self.fault_at=None

    def message(self,now,sequence,statuses):
        from ph_exo_interfaces.msg import ImpedanceRef
        if self.started is None and any(s['mode']==2 for s in statuses):
            self.started=now
        t=0. if self.started is None else now-self.started
        q,dq=reference(t,1 if self.case=='beta' else 0)
        msg=ImpedanceRef(id=1,seq=sequence,t=now,on=True,
            q=q.tolist(),dq=dq.tolist(),k=K,b=B,tau=CAP)
        if self.case not in ('psi','beta') and t>=4.:
            if self.fault_at is None:self.fault_at=now
            # Resume valid stream after 0.6s: the fault must remain latched.
            if t<4.6:
                if self.case=='drop':return None
                if self.case=='replay':return self.last
                if self.case=='nan':msg.dq[0]=float('nan')
                if self.case=='gain':msg.k[0]=2.
                if self.case=='session':msg.id=2
                if self.case=='stop':msg.on=False
        else:self.last=msg
        self.events.append(dict(seq=sequence,t=now,q=q.tolist(),dq=dq.tolist()))
        return msg

    def summarize(self,folder,returncode,evidence,statuses,typed):
        import json
        from ref01_manager_validation import load_rows
        raw=load_rows(folder/'trace.csv')
        meta=json.loads((folder/'metadata.json').read_text())
        end=raw[-1]['finish_ns']/1e9
        status=[s for s in statuses if s['observed_at_monotonic_s']<=end]
        good=[s for s in typed if s['mode']==2 and s['observed_at_monotonic_s']<=end]
        checks=dict(evidence['checks'],process_completed=returncode==0,
            simulation_only=meta['simulation_only'],
            completed=meta['runtime_error']=='none' and meta['samples']==len(raw),
            following=bool(good),dynamic_status=bool(good) and all(s['dynamic'] for s in good),
            session_id=bool(good) and all(s['session_id']==1 for s in good),
            motor_cap=max(abs(r[k]) for r in raw for k in ('effort104','effort105'))<=.2+1e-10,
            warnings=meta['warnings']==0,closure=meta['max_closure_m']<1e-4)
        metrics={}
        if self.case in ('psi','beta'):
            checks['no_fault']=not any(s['mode']==3 for s in status)
            # Status contains the latest accepted reference, and paired measured state.
            rows=[[s['observed_at_monotonic_s']-self.started,*s['position_rad'],
                   *s['velocity_rad_s'],*s['reference_rad'],*s['dq_ref'],
                   *s['motor_target_nm'],0.] for s in good]
            metrics=assess_trace(rows)
            checks.update(metrics['checks'])
            expected={e['seq']:e for e in self.events}
            checks['reference_echo']=all(s['accepted_sequence'] in expected and
                np.allclose(s['reference_rad'],expected[s['accepted_sequence']]['q'],atol=1e-12,rtol=0) and
                np.allclose(s['dq_ref'],expected[s['accepted_sequence']]['dq'],atol=1e-12,rtol=0) for s in good)
        else:
            expected='plan_changed' if self.case=='gain' else 'disabled' if self.case=='stop' else 'plan_expired'
            faults=[s for s in status if s['mode']==3]
            checks['expected_fault']=bool(faults) and all(s['reason_name']==expected for s in faults)
            tail=[r for r in raw if r['cycle_start_ns']/1e9>self.fault_at+.5]
            checks['fault_latched_after_resume']=bool(faults) and all(s['mode']==3 for s in status
                if s['observed_at_monotonic_s']>self.fault_at+.6)
            checks['generation_stopped']=bool(tail) and all(len({r[k] for r in tail})==1
                for k in ('command_generation104','command_generation105'))
            checks['receiver_zero']=bool(tail) and all(r['actual_force104']==0. and r['actual_force105']==0. for r in tail)
            metrics=dict(expected_reason=expected,first_fault_reason=faults[0]['reason_name'] if faults else None,
                         fault_delay_ms=(faults[0]['observed_at_monotonic_s']-self.fault_at)*1000 if faults else None)
        gaps=np.diff([r['cycle_start_ns'] for r in raw])*1e-6
        return dict(case=self.case,passed=all(checks.values()),checks=checks,metrics=metrics,
                    samples=len(raw),cycle_gap_ms=dict(p99=float(np.quantile(gaps,.99)),max=float(gaps.max())),
                    evidence_kind='ROS_MuJoCo_software',hardware_verified=False)
