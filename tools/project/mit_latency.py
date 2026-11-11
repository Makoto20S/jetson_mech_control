#!/usr/bin/env python3
"""Explicit test-only, zero-command host latency measurement."""
import argparse
import csv
import json
import math
from pathlib import Path
import secrets
import signal
import sys
import xml.etree.ElementTree as ET
import mit_communication as mit

def install_timing(nonce):
    original=mit.artifacts
    def artifacts(config,out):
        xml,params=original(config,out)
        robot=ET.fromstring(xml)
        for control in robot.findall('ros2_control'):
            hw=control.find('hardware')
            hw.find('plugin').text='mech_bringup/MitTimingSystem'
            mid=int(hw.find("param[@name='drive_id']").text)
            for name in ('timing_rx_sequence','timing_rx_ns','timing_read_ns'):
                ET.SubElement(control.find('joint'),'state_interface',name=name)
            controller=f'zero_effort_{mid}'
            params[mit.NS+'/controller_manager']['ros__parameters'][controller]['type']='mech_bringup/MitZeroTimingController'
            params[mit.NS+'/'+controller]['ros__parameters'].update(
                timing_nonce=nonce,timing_path=str(out/f'controller_{mid}.csv'))
        xml=ET.tostring(robot,encoding='unicode')
        params[mit.NS+'/controller_manager']['ros__parameters']['robot_description']=xml
        return xml,params
    mit.artifacts=artifacts
    execute=mit.execute
    mit.execute=lambda config,out,duration: execute(config,out,duration,timing_nonce=nonce)

def statistics(values):
    values=sorted(values)
    if not values:return dict(count=0)
    def p(q):return values[math.ceil(len(values)*q)-1]
    return dict(count=len(values),p50_ms=p(.5),p95_ms=p(.95),p99_ms=p(.99),max_ms=values[-1])

def correlate(publications, controls, records, mid):
    published={r['seq']:r for r in publications if r['motor']==mid}
    if len(published)!=sum(r['motor']==mid for r in publications):
        raise ValueError('Duplicate publisher sequence')
    sends={}
    for r in records:
        if r['stage']=='transport_result' and r['result']==0:
            if r['id'] != 0x800+mid or r['hex']!=mit.ZERO_WIRE:
                raise ValueError('Unexpected physical command')
            sends.setdefault(r['generation'],[]).append(r)
    rows=[];seen=set()
    for r in controls:
        seq=r['seq']
        if seq in seen:raise ValueError('Duplicate applied target')
        seen.add(seq)
        if seq not in published or r['publish_ns']!=published[seq]['publish_ns']:
            raise ValueError('Publisher/controller identity mismatch')
        candidates=sends.get(r['command_generation'],[])
        if not candidates:continue
        tx=candidates[0]['ns']
        times=[r['publish_ns'],r['receive_ns'],r['update_ns'],tx]
        if times!=sorted(times):raise ValueError('Clock/order violation')
        rows.append(dict(seq=seq,publish_to_callback_ms=(times[1]-times[0])/1e6,
            callback_to_update_ms=(times[2]-times[1])/1e6,
            update_to_usb_ms=(times[3]-times[2])/1e6,
            publish_to_usb_ms=(times[3]-times[0])/1e6))
    return dict(motor=mid,published=len(published),applied=len(controls),matched=len(rows),
                unmatched_published=sorted(set(published)-{r['seq'] for r in rows}),
                metrics={key:statistics([r[key] for r in rows]) for key in
                 ('publish_to_callback_ms','callback_to_update_ms','update_to_usb_ms','publish_to_usb_ms')},
                samples=rows)

def window_records(rows, field, start, end):
    return [r for r in rows if start <= r[field] <= end]

def analyze(out):
    run=json.loads((out/'summary.json').read_text())
    pubs=json.loads((out/'timing_publications.json').read_text())
    events={e['event']:e['monotonic_ns'] for e in run['events']}
    start,end=events['zero_effort_window_start'],events['zero_effort_window_end']
    if end<=start:raise ValueError('Invalid test window')
    all_pubs=pubs
    pubs=window_records(pubs,'publish_ns',start,end)
    result=dict(window_start_ns=start,window_end_ns=end,window_duration_s=(end-start)/1e9,
                host_only=True,device_execution_latency_available=False,run_status=run['status'],motors=[])
    for motor in run['selected']['motors']:
        mid,bus=motor['id'],motor['logical_bus']
        with (out/f'controller_{mid}.csv').open() as f:
            header=f.readline()
            controls=[{k:int(v) for k,v in row.items()} for row in csv.DictReader(f)]
        count=int(header.split('total=')[1].split()[0])
        if count!=len(controls):raise ValueError('Controller trace missing/overflow')
        raw=[json.loads(s) for s in (out/f'bus{bus}_motor{mid}.jsonl').read_text().splitlines()]
        if raw[0]['dropped'] or raw[0]['total']!=len(raw)-1:raise ValueError('Hardware trace incomplete')
        total_controls=len(controls)
        sequences={r['seq'] for r in pubs if r['motor']==mid}
        controls=[r for r in controls if r['seq'] in sequences]
        matched=correlate(pubs,controls,raw[1:],mid)
        matched.update(total_published_including_shutdown=sum(r['motor']==mid for r in all_pubs),
                       total_applied_including_shutdown=total_controls)
        result['motors'].append(matched)
    feedback=json.loads((out/'timing_feedback.json').read_text())
    result['feedback']=[]
    for motor in run['selected']['motors']:
        rows=[r for r in feedback if r['joint']==f'mit_comm_motor{motor["id"]}']
        if any(not 0<r['rx_ns']<=r['read_ns']<=r['callback_ns'] for r in rows):
            raise ValueError('Feedback clock/order violation')
        total_rows=len(rows)
        rows=window_records(rows,'rx_ns',start,end)
        result['feedback'].append(dict(motor=motor['id'],unique_samples=len(rows),
            total_samples_including_startup_shutdown=total_rows,
            rx_to_ros_subscriber=statistics([(r['callback_ns']-r['rx_ns'])/1e6 for r in rows]),
            rx_to_read_complete=statistics([(r['read_ns']-r['rx_ns'])/1e6 for r in rows]),
            read_to_ros_subscriber=statistics([(r['callback_ns']-r['read_ns'])/1e6 for r in rows])))
    (out/'latency.json').write_text(json.dumps(result,indent=2))
    print(json.dumps({**result,'motors':[{k:v for k,v in m.items() if k!='samples'} for m in result['motors']]},indent=2))
    return result

def main():
    p=argparse.ArgumentParser(description=__doc__)
    mode=p.add_mutually_exclusive_group(required=True)
    mode.add_argument('--execute',action='store_true')
    mode.add_argument('--simulate',action='store_true')
    mode.add_argument('--analyze',action='store_true')
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--duration',type=float,default=5.)
    p.add_argument('--motors',default='105,104')
    a=p.parse_args()
    if a.analyze:analyze(a.output);return 0
    out=a.output.resolve()
    if not 0<a.duration<=20 or not out.is_relative_to((mit.ROOT/'.local/validation').resolve()) or out.exists():
        p.error('New .local/validation directory and duration in (0,20] required')
    install_timing(secrets.randbelow(2**30)+1)
    if a.simulate:
        import benchmark_mit_pty
        sys.argv=[sys.argv[0],'--duration',str(a.duration),'--detail','--output',str(out)]
        return benchmark_mit_pty.main()
    config=mit.selected_config(json.loads((mit.ROOT/'config/parallel_exo/servo_pair.ref01.json').read_text()),
                               [int(x) for x in a.motors.split(',')])
    def interrupted(signum,frame):raise KeyboardInterrupt(str(signum))
    signal.signal(signal.SIGTERM,interrupted);signal.signal(signal.SIGHUP,interrupted)
    return mit.execute(config,out,a.duration)

if __name__=='__main__':
    raise SystemExit(main())
