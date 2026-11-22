"""Dynamic MIT software fixture/acceptance; reuses ordinary controller and real codec."""
import bisect
import math
import numpy as np
from ref01_dynamic_validation import DynamicRun, reference, K, B, CAP
from ref01_mit_fixture import MitFixture,ERPM_SCALE,LSB

def current_scale():
    from ref01_bench_hold import feedback_mapping
    mapping=feedback_mapping()
    scales=[mapping['axes'][str(mid)]['rad_s_per_erpm'] for mid in (104,105)]
    if scales[0]!=scales[1]:raise ValueError('this synthetic fixture requires equal positive scales')
    return scales[0],mapping

def fixture(k=None):
    scale,_=current_scale()
    return DynamicMitFixture(feedback_period_s=.005,erpm_scale=scale,pulse_nm=0.)

def tracking(rows,cap=.2):
    # t,q(2),dq(2),ref(2),vref(2),applied torque(2),closure
    a=np.asarray(rows,dtype=float)
    err=np.rad2deg(a[:,1:3]-a[:,5:7]);speed=np.rad2deg(a[:,3:5])
    moving=(a[:,0]>=1.)&(a[:,0]<=7.)
    tail=a[:,0]>=9.6
    checks=dict(finite=bool(np.isfinite(a).all()),
        trajectory_exercised=bool(np.ptp(a[:,5:7],axis=0).max()>math.radians(2.9)),
        tracking_rmse=bool(moving.any() and np.sqrt(np.mean(err[moving]**2))<.15),
        # Preserve the existing MIT fixture .5deg/1deg/s bands.
        recovered_both_axes=bool(tail.any() and abs(err[tail]).max()<=.5 and abs(speed[tail]).max()<=1.),
        quantized_cap=bool(abs(a[:,9:11]).max()<=cap+LSB),
        closure=bool(abs(a[:,11]).max()<1e-4))
    return dict(passed=all(checks.values()),checks=checks,
        criteria=dict(tracking_rmse_deg=.15,tail_error_deg=.5,tail_speed_deg_s=1.),
        tracking_rmse_deg=float(np.sqrt(np.mean(err[moving]**2))) if moving.any() else None,
        max_error_deg=float(abs(err).max()),tail_error_deg=float(abs(err[tail]).max()) if tail.any() else None,
        tail_speed_deg_s=float(abs(speed[tail]).max()) if tail.any() else None,
        peak_applied_nm=abs(a[:,9:11]).max(axis=0).tolist(),
        max_step_nm=abs(np.diff(a[:,9:11],axis=0)).max(axis=0).tolist(),
        ideal_bands_passed=bool(tail.any() and abs(err[tail]).max()<=.05 and abs(speed[tail]).max()<=.25))

class DynamicMitFixture(MitFixture):
    def configuration(self,*args):
        xml,params=super().configuration(*args)
        # Only the controller opts into already-supported K<=10; the hardware
        # remains simulation_only on two private PTYs with the original .2 Nm cap.
        for name,value in params.items():
            if name.endswith("/ref01_impedance"):
                value["ros__parameters"].update(simulation_only=False,bench_profile="ref01_supported_v1")
        return xml,params

class MitReferenceRun(DynamicRun):
    def __init__(self,case,gains=None):
        super().__init__(case)
        self.k=list(K if gains is None else gains)
        if len(self.k)!=2 or any(not math.isfinite(k) or not 0<k<=10 for k in self.k):
            raise ValueError("K must be two values in (0,10]")
    def message(self,*args):
        msg=super().message(*args)
        if msg is not None:msg.k=self.k
        return msg

    def summarize_mit(self,folder,returncode,evidence,statuses,typed,plant):
        from ref01_manager_validation import REQUIRED_MANAGER_CHECKS
        claims=evidence.get('checks',{})
        checks={k:claims.get(k) is True for k in REQUIRED_MANAGER_CHECKS-
                {'explicit_controller_deactivation','explicit_hardware_deactivation'}}
        checks['cleanup_confirmed']=(claims.get('explicit_controller_deactivation') is True and
            claims.get('explicit_hardware_deactivation') is True) or evidence.get('fault_shutdown_verified') is True
        end=plant.rows[-1]['ns']/1e9 if plant.rows else 0.
        observed=[s for s in statuses if s['observed_at_monotonic_s']<=end]
        good=[s for s in typed if s['mode']==2 and s['observed_at_monotonic_s']<=end]
        checks.update(manager_exit=returncode==0,emulator_clean=not plant.errors,
            complete_samples=len(plant.rows)==round(plant.duration/.005)+1,
            two_drives={r['drive'] for r in plant.tx}=={104,105},
            feedback_present={r['drive'] for r in plant.rx}=={104,105},
            typed_dynamic=bool(good) and all(s['dynamic'] and s['session_id']==1 for s in good),
            no_solver_warnings=all(r['warnings']==0 for r in plant.rows))
        tx={mid:[r for r in plant.tx if r['drive']==mid] for mid in (104,105)}
        result=dict(case=self.case,checks=checks,errors=plant.errors,
                    counts=dict(tx=len(plant.tx),rx=len(plant.rx),plant=len(plant.rows)))
        if self.case in ('psi','beta'):
            checks['no_controller_fault']=not any(s['mode']==3 for s in observed)
            checks['commands_reach_end']=bool(plant.rows) and all(v and v[-1]['ns']>=plant.rows[-1]['ns']-15000000 for v in tx.values())
            expected={e['seq']:e for e in self.events}
            checks['reference_echo']=all(s['accepted_sequence'] in expected and
                np.allclose(s['reference_rad'],expected[s['accepted_sequence']]['q'],rtol=0,atol=1e-12) and
                np.allclose(s['dq_ref'],expected[s['accepted_sequence']]['dq'],rtol=0,atol=1e-12) for s in good)
            times=[e['t'] for e in self.events];rows=[]
            for r in plant.rows:
                n=bisect.bisect_right(times,r['ns']/1e9)-1
                if n<0:continue
                ref=self.events[n]
                rows.append([r['ns']/1e9-self.started,r['psi_rad'],r['beta_rad'],
                    r['dpsi_rad_s'],r['dbeta_rad_s'],*ref['q'],*ref['dq'],
                    r['tau104_nm'],r['tau105_nm'],r['closure_m']])
            result['tracking']=tracking(rows)
            result['tracking_passed']=result['tracking']['passed']
        else:
            cut=self.fault_at or 0.
            faults=[s for s in observed if s['mode']==3]
            checks['correct_fault']=bool(faults) and all(s['reason_name']=='plan_expired' for s in faults)
            checks['healthy_before_cut']=cut>0 and all(any(cut-.03<=v['ns']/1e9<=cut for v in values) for values in tx.values())
            checks['stop_within_120ms']=cut>0 and all(v and v[-1]['ns']/1e9<=cut+.12 for v in tx.values())
            checks['no_restart']=cut>0 and not any(v['ns']/1e9>cut+.45 for v in plant.tx)
            checks['feedback_continued']=cut>0 and all(any(v['drive']==mid and v['ns']/1e9>cut+.6 for v in plant.rx) for mid in (104,105))
            tail=[r for r in plant.rows if r['ns']/1e9>cut+.3]
            checks['watchdog_zero']=bool(tail) and all(r['tau104_nm']==0. and r['tau105_nm']==0. for r in tail)
            result['last_tx_after_cut_ms']={str(mid):(v[-1]['ns']/1e9-cut)*1000 if v and cut else None for mid,v in tx.items()}
        scales=[s['scale'] for s in observed if s['mode']==2 and 'scale' in s]
        result['saturation']=dict(observed=len(scales),saturated=sum(x<1. for x in scales),
                                  min_scale=min(scales) if scales else None,
                                  basis='non-RT software status samples, not full cycle count')
        result['transport_passed']=all(checks.values())
        result['passed']=result['transport_passed'] and result.get('tracking_passed',True)
        result['wire_period_ms']={}
        result['feedback_period_ms']={}
        for mid in (104,105):
            for source,key in ((tx[mid],'wire_period_ms'),([r for r in plant.rx if r['drive']==mid],'feedback_period_ms')):
                intervals=np.diff([r['ns'] for r in source])*1e-6
                result[key][str(mid)]=dict(p50=float(np.median(intervals)),p99=float(np.quantile(intervals,.99)),max=float(intervals.max())) if len(intervals) else None
        result['boundary']='Private PTY, production MIT/CDC runtime, native controller and MuJoCo. Synthetic watchdog zero is not physical disable.'
        return result
