
"""Private-PTY scale verification. Call explicitly on Orin; never physical ports."""
import json,math,struct,sys
from pathlib import Path
from unittest.mock import patch
import ref01_bench_hold as bench
import benchmark_mit_pty

def main():
    output=bench.new_output_path()
    print("ERPM_OUTPUT "+str(output),flush=True)
    packet=benchmark_mit_pty.packet
    active=[False]
    def observe(row):
        # Leave passive/initial-reference checks at rest; inject only after following.
        if row["mode"]==2:active[0]=True
    def inject(data):
        # Only the rehearsal's 0x29 feedback envelope; no command injection.
        if active[0] and len(data)==14 and struct.unpack("<I",data[:4])[0] in (0x2900+104,0x2900+105):
            mid=struct.unpack("<I",data[:4])[0]-0x2900
            data=data[:8]+struct.pack(">h",112 if mid==104 else -224)+data[10:]
        return packet(data)
    with patch.object(benchmark_mit_pty,"packet",inject):
        bench.run(output,True,2.,"normal",False,(10.,10.,0.,0.),observer=observe)
    result=json.loads((output/"result.json").read_text())
    assert result["rehearsal"] and result["disable_ack"]==[104,105] and not result.get("error")
    expected={104:1120.,105:-2240.}
    counts={}
    for mid,erpm in expected.items():
        samples=[r for r in result["samples"] if r["id"]==mid and r.get("mit_valid")==1 and r["mit_erpm"]==erpm]
        assert len(samples)>10
        for s in samples:
            assert s["mit_erpm"]==erpm
            assert abs(s["velocity"]-erpm*2*math.pi/(60*112))<1e-12
        shown=[r["feedback"][str(mid)] for r in result["panel_samples"]
               if r.get("feedback",{}).get(str(mid),{}).get("rpm") is not None
               and r["feedback"][str(mid)]["mit_erpm"]==erpm]
        assert shown
        assert all(abs(s["rpm"]-erpm/112)<1e-10 for s in shown)
        counts[str(mid)]=dict(samples=len(samples),display=len(shown),rpm=erpm/112)
    report=dict(passed=True,software_only=True,output=str(output),axes=counts,
                feedback_mapping=result["feedback_mapping"])
    (output/"scale_check.json").write_text(json.dumps(report,indent=2))
    print("ERPM_CHECK "+json.dumps(report),flush=True)

if __name__=="__main__":main()
