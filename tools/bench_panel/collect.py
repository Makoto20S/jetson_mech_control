"""Explicit recovery of a named Orin run; never opens hardware or scans history."""
import argparse
import json
from pathlib import Path
import re
import subprocess
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/"project"))
from ph_exo_bench_panel import ROOT,ssh_command
from ph_exo_bench_worker import PREFIX
from bench_archive import ArchiveReceiver,checked_path

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument("run");a=p.parse_args()
    if not re.fullmatch(r"ref01_bench_[0-9_]+(?:[a-f0-9]{8})?",a.run):raise ValueError("Invalid run name")
    destination=checked_path(ROOT,"tools/bench_panel/data/recovered/"+a.run)
    receiver=ArchiveReceiver(destination)
    config=json.loads((ROOT/"config/parallel_exo/orin_sync.json").read_text())
    command=ssh_command(config);command[-1]=command[-1].replace("--execute-panel","--collect-run "+a.run)
    proc=subprocess.Popen(command,stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=subprocess.PIPE)
    success=False
    try:
        for raw in proc.stdout:
            line=raw.decode()
            if not line.startswith(PREFIX):continue
            event=json.loads(line[len(PREFIX):])
            if event["type"].startswith("archive_"):
                ack=receiver.accept(event)
                if ack:proc.stdin.write(("ARCHIVE_OK "+ack+"\n").encode());proc.stdin.flush()
            elif event["type"]=="collected":
                success=event["removed"];print(json.dumps(dict(event,local_directory=str(destination))))
        if proc.wait(timeout=20)!=0 or not success:
            raise RuntimeError("Remote archive pending: "+proc.stderr.read().decode(errors="replace"))
    finally:
        proc.stdin.close()
    return 0

if __name__=="__main__":sys.exit(main())
