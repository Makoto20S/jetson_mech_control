#!/usr/bin/env python3
"""Offline wall-clock MIT benchmark: creates its own PTYs, never physical ports."""
import gc
import argparse
import json
import math
import os
from pathlib import Path
import signal
import struct
import threading
import time
import xml.etree.ElementTree as ET
import mit_communication as mit

def crc(data, seed, polynomial):
    for value in data:
        seed ^= value
        for _ in range(8):
            seed = (seed >> 1) ^ (polynomial if seed & 1 else 0)
    return seed

def packet(body):
    header = b'\x12' + struct.pack('<H', len(body))
    return b'\xf7'+header+bytes([crc(header,255,0x8c)])+struct.pack('<H',crc(body,65535,0x8408))+body

def feedback(drive, status=0):
    return packet(struct.pack('<IBB',0x2900+drive,4,8)+struct.pack('>hhhBB',450,0,0,40,status))

class Gateway:
    def __init__(self, drive):
        import pty, tty
        self.master,self.slave=pty.openpty()
        tty.setraw(self.slave)
        self.path=os.ttyname(self.slave)
        if not self.path.startswith('/dev/pts/'): raise RuntimeError('Not a private PTY')
        self.drive=drive
        self.rows=[];self.errors=[]
        self.done=threading.Event()
        self.thread=threading.Thread(target=self.run,daemon=True)
        self.thread.start()

    def validate_command(self, body):
        if body[4:] != bytes.fromhex('0c08' + mit.ZERO_WIRE):
            raise RuntimeError('Nonzero or malformed MIT command')

    def send_feedback(self, acknowledge=False):
        frame = feedback(self.drive, 0x77) if acknowledge else feedback(self.drive)
        os.write(self.master, frame)

    def run(self):
        import select
        pending=b''; next_rx=time.monotonic()
        try:
            while not self.done.is_set():
                if select.select([self.master],[],[],.001)[0]:
                    pending+=os.read(self.master,4096)
                while len(pending)>=7:
                    size=int.from_bytes(pending[2:4],'little')+7
                    if size>1024: raise RuntimeError('Invalid packet length')
                    if len(pending)<size: break
                    raw,pending=pending[:size],pending[size:]
                    body=raw[7:]
                    if raw!=packet(body): raise RuntimeError('Invalid CRC/header')
                    if body==bytes(6): continue
                    ident=int.from_bytes(body[:4],'little')
                    self.rows.append(dict(ns=time.monotonic_ns(),id=ident,hex=body[6:].hex()))
                    if len(self.rows)>80000: raise RuntimeError('Capture capacity exceeded')
                    if ident==0x800+self.drive:
                        self.validate_command(body)
                    elif ident==0xf00+self.drive:
                        self.send_feedback(acknowledge=True)
                    else: raise RuntimeError('Unexpected command '+hex(ident))
                now=time.monotonic()
                if now>=next_rx:
                    self.send_feedback()
                    next_rx+=.02
                    if next_rx<now: next_rx=now+.02
        except BaseException as exc:
            self.errors.append(str(exc))

    def close(self):
        self.done.set(); self.thread.join(3)
        os.close(self.master);os.close(self.slave)

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--duration',type=float,required=True)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--detail',action='store_true')
    p.add_argument('--disable-gc',action='store_true',help='Bounded test-process comparison only')
    a=p.parse_args()
    if not math.isfinite(a.duration) or not 0<a.duration<=120 or (a.detail and a.duration>30):
        p.error('Duration <=120 seconds; detailed capture <=30 seconds')
    out=a.output.resolve()
    if not out.is_relative_to((mit.ROOT/'.local/validation').resolve()) or out.exists():
        p.error('Use a new directory below .local/validation')
    gateways=[]
    gc_was_enabled=gc.isenabled()
    gc_events=[]
    def gc_event(phase,info): gc_events.append(dict(ns=time.monotonic_ns(),phase=phase,generation=info['generation']))
    gc.callbacks.append(gc_event)
    try:
        config=mit.selected_config(json.loads((mit.ROOT/'config/parallel_exo/servo_pair.ref01.json').read_text()),[105,104])
        for m in config['motors']:
            g=Gateway(m['id']);gateways.append(g)
            next(b for b in config['buses'] if b['logical_bus']==m['logical_bus'])['device_path']=g.path
        paths={m['id']:g.path for m,g in zip(config['motors'],gateways)}
        original_artifacts=mit.artifacts
        def artifacts(c,o):
            xml,params=original_artifacts(c,o)
            root=ET.fromstring(xml)
            if not a.detail:
                for hw in root.findall('ros2_control/hardware'):
                    hw.remove(hw.find("param[@name='command_trace_path']"))
                    # Native error/feedback telemetry remains enabled in both cases.
            xml=ET.tostring(root,encoding='unicode')
            params[mit.NS+'/controller_manager']['ros__parameters']['robot_description']=xml
            return xml,params
        original_shutdown=mit.shutdown_urdf
        def shutdown(c):
            root=ET.fromstring(original_shutdown(c))
            for control in root.findall('ros2_control'):
                mid=int(control.find("joint/param[@name='drive_id']").text)
                control.find("hardware/param[@name='device_path']").text=paths[mid]
            for param in root.findall("ros2_control/hardware/param[@name='device_path']"):
                if param.text not in paths.values(): raise RuntimeError('Physical path forbidden')
            return ET.tostring(root,encoding='unicode')
        mit.artifacts=artifacts;mit.shutdown_urdf=shutdown
        def interrupted(signum,frame): raise KeyboardInterrupt(str(signum))
        signal.signal(signal.SIGTERM,interrupted);signal.signal(signal.SIGHUP,interrupted)
        if a.disable_gc: gc.disable()
        mit.execute(config,out,a.duration)
    finally:
        for g in gateways:g.close()
        gc.callbacks.remove(gc_event)
        if gc_was_enabled: gc.enable()
    run=json.loads((out/'summary.json').read_text())
    results=[]
    for g in gateways:
        (out/f'gateway_{g.drive}.json').write_text(json.dumps(g.rows))
        ns=[r['ns'] for r in g.rows if r['id']==0x800+g.drive]
        gaps=[b-a for a,b in zip(ns,ns[1:])]
        results.append(dict(drive=g.drive,errors=g.errors,pty_delivery=mit.interval_stats(ns),
            span_s=(ns[-1]-ns[0])/1e9 if len(ns)>1 else 0,
            gaps_over_2ms=sum(x>2000000 for x in gaps),
            gaps_over_4ms=sum(x>4000000 for x in gaps),
            gaps_over_6ms=sum(x>6000000 for x in gaps),
            disable_commands=sum(r['id']==0xf00+g.drive for r in g.rows)))
    complete=(not run['error'] and run.get('disable_returncode')==0 and
              all(not r['errors'] and r['span_s']>=a.duration*.99 and r['disable_commands']==1 for r in results))
    summary=dict(complete=complete,duration_s=a.duration,detailed=a.detail,motors=results,
                 runtime_error=run['error'],physical_ports_opened=0,gc_disabled=a.disable_gc,gc_events=gc_events,
                 boundary='Actual ROS MIT chain with private PTYs; PTY delivery includes emulator scheduling, not CAN/device latency. Minimal mode intentionally has no native trace.')
    (out/'benchmark.json').write_text(json.dumps(summary,indent=2))
    print(json.dumps({k:v for k,v in summary.items() if k != 'gc_events'},indent=2),flush=True)
    return 0 if complete else 2

if __name__=='__main__':
    raise SystemExit(main())
