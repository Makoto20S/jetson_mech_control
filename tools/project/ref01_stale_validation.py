"""Post-run analysis only; never changes controller thresholds or accesses devices."""
import json
from pathlib import Path

def analyze_stale_state(traces, statuses):
    faults=[s for s in statuses if s.get("mode")==3]
    first=faults[0] if faults else {}
    timing=first.get("first_fault_timing",{})
    ages=timing.get("feedback_age_s",[])
    checks={
        "feedback_expired_is_first_fault": bool(faults) and all(s.get("reason_name")=="feedback_expired" for s in faults),
        "first_fault_timing_present": timing.get("at_s",0)>0 and len(ages)==2,
        "no_period_fault_at_expiry": 0<timing.get("manager_period_s",0)<.010 and 0<timing.get("update_gap_s",0)<.010,
        "original_source_expired": len(ages)==2 and max(ages)>=.060,
        "two_motor_traces": set(traces)=={104,105},
    }
    motors={}
    for mid in (104,105):
        rows=traces.get(mid,[])
        by=lambda stage:[r for r in rows if r.get("stage")==stage]
        frozen=[r for r in by("kinematic_read_end") if r.get("flags")==1]
        lease=[r for r in by("kinematic_lease") if r.get("flags")==1]
        begin=by("hardware_error_begin");revoked=by("hardware_error_revoked");end=by("hardware_error_end")
        complete=bool(frozen and lease and begin and revoked and end)
        prefix=str(mid)+"_"
        checks[prefix+"events_present"]=complete
        checks[prefix+"trace_complete"]=bool(rows) and rows[0].get("dropped")==0
        checks[prefix+"state_frozen"]=bool(frozen) and len({(r["position"],r["generation"]) for r in frozen})==1
        checks[prefix+"raw_feedback_advances"]=any(r["feedback"]>r["position"] for r in frozen)
        checks[prefix+"cause_order"]=complete and frozen[0]["ns"]<=lease[0]["ns"]<=begin[0]["ns"]<=revoked[0]["ns"]<=end[0]["ns"]<=timing.get("at_s",0)*1e9
        if complete:
            motors[str(mid)]=dict(
                freeze_ns=frozen[0]["ns"],frozen_source_s=frozen[0]["position"],
                lease_cancel_ns=lease[0]["ns"],error_begin_ns=begin[0]["ns"],
                error_revoked_ns=revoked[0]["ns"],error_end_ns=end[0]["ns"],
                error_callback_ms=(end[0]["ns"]-begin[0]["ns"])/1e6,
                post_revoke_ms=(end[0]["ns"]-revoked[0]["ns"])/1e6)
    return dict(passed=all(checks.values()),checks=checks,first_fault=first, motors=motors)

def load_stale_state(folder,statuses):
    traces={}
    for mid in (104,105):
        path=Path(folder)/("bus"+str(2 if mid==104 else 1)+"_motor"+str(mid)+".jsonl")
        traces[mid]=[json.loads(line) for line in path.read_text().splitlines()] if path.is_file() else []
    return analyze_stale_state(traces,statuses)
