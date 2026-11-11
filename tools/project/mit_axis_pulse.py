"""One bounded PH-EXO bench pulse. Uses the ROS pair controller, never a raw CAN writer."""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import statistics
import struct
import subprocess
import sys
import time
from uuid import uuid4
import xml.etree.ElementTree as ET

import mit_communication as mit

IDS = (104, 105)
JOINTS = [f'mit_comm_motor{i}' for i in IDS]
EFFORT = 0.10
SLEW = 0.50
BASE_S, STEP_S, RELEASE_S = 1.0, 0.34, 1.2


def require(ok, reason):
    if not ok:
        raise RuntimeError(reason)


def setpoints(axis, sign, active):
    require(axis in IDS and sign in (-1, 1), 'Unknown axis or sign')
    return [sign * EFFORT if active and i == axis else 0.0 for i in IDS]


def make_artifacts(cfg, paths, out, ns):
    from copy import deepcopy
    sys.path.insert(0, str(mit.ROOT / 'tools/servo'))
    import servo_operator

    c = deepcopy(cfg)
    for m in c['motors']:
        bus = next(b for b in c['buses'] if b['logical_bus'] == m['logical_bus'])
        bus['device_path'] = paths[m['id']]
    mit.NS = ns
    xml, params = mit.artifacts(c, out)
    disable = servo_operator.generate_urdf(c, observe=True)
    for desc in (xml, disable):
        found = {}
        for hw in ET.fromstring(desc).findall('ros2_control'):
            ident = hw.find(".//param[@name='drive_id']")
            path = hw.find("hardware/param[@name='device_path']")
            found[int(ident.text)] = path.text
        require(found == paths, 'Device identity mismatch')
    mgr = params[ns+'/controller_manager']['ros__parameters']
    for mid in IDS:
        del mgr[f'zero_effort_{mid}']
        del params[f'{ns}/zero_effort_{mid}']
    mgr['pulse'] = {'type': 'mech_bringup/MitPairEffortController'}
    params[ns+'/pulse'] = {'ros__parameters': dict(max_effort_nm=EFFORT, max_slew_nm_s=SLEW,
        max_abs_erpm=100., max_abs_iq_a=.5, max_temperature_c=60., target_ttl_ns=60000000)}
    return params, disable


def passive(binary, desc, out):
    with (out/'pre.jsonl').open('w') as dest:
        subprocess.run([str(binary/'servo_feedback_reader'),'--urdf',str(desc),'--duration','2'],
            stdout=dest, stderr=subprocess.STDOUT, check=True, timeout=6)
    rows = [json.loads(l) for l in (out/'pre.jsonl').read_text().splitlines() if l.startswith('{')]
    require(len(rows) >= 30 and all(r['motor_command_frames']==0 for r in rows)
            and all(r['valid'] for r in rows[-25:]), 'Passive preflight incomplete')
    ref = {}
    for i in IDS:
        samples = [next(m for m in r['motors'] if m['id']==i) for r in rows[-25:]]
        pos = [s['feedback_position_deg'] for s in samples]
        require(all(s['availability']=='fresh' and s['raw_status']==0 and
                    abs(s['electrical_speed_erpm'])<=20 and abs(s['current_iq_a'])<=.20 and
                    0<s['temperature_c']<60 for s in samples), f'{i} passive status or motion')
        require(max(pos)-min(pos)<=.3 and min(pos)>2 and max(pos)<(218 if i==104 else 208),
                f'{i} unstable or close to travel boundary')
        ref[i] = statistics.median(pos)
    return ref


def status(node, client):
    import rclpy
    from std_srvs.srv import Trigger
    f = client.call_async(Trigger.Request())
    rclpy.spin_until_future_complete(node, f, timeout_sec=.4)
    require(f.done() and f.result() and f.result().success and f.result().message == 'following',
            'Pair controller not following')


def drive_window(node, manager, ns, axis, sign, ref, events):
    import rclpy
    from control_msgs.msg import DynamicJointState
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
    from std_srvs.srv import Trigger

    latest = {}
    samples = {i: [] for i in IDS}
    def receive(msg):
        for name, vals in zip(msg.joint_names, msg.interface_values):
            if name not in JOINTS:
                continue
            mid = IDS[JOINTS.index(name)]
            d = dict(zip(vals.interface_names,vals.values))
            seq = d['mit_rx_sequence']
            if mid not in latest or seq != latest[mid]['mit_rx_sequence']:
                latest[mid] = d
                samples[mid].append((int(d['mit_rx_us'])*1000, d['mit_iq_a'], d['mit_position_deg'], d['mit_erpm']))

    node.create_subscription(DynamicJointState, ns+'/dynamic_joint_states', receive, 1)
    pub = node.create_publisher(JointTrajectory, ns+'/pulse/target', 1)
    client = node.create_client(Trigger,ns+'/pulse/status')
    stop_client = node.create_client(Trigger,ns+'/pulse/stop')
    deadline = time.monotonic()+3
    while time.monotonic()<deadline and (set(latest)!=set(IDS) or pub.get_subscription_count()!=1):
        rclpy.spin_once(node, timeout_sec=.01)
    require(pub.get_subscription_count()==1 and set(latest)==set(IDS) and
            client.wait_for_service(timeout_sec=1.) and stop_client.wait_for_service(timeout_sec=1.),
            'Missing pair control or raw state')
    def check():
        require(manager.poll() is None,'Manager exited')
        for i,d in latest.items():
            require(d['mit_drive_id']==i and d['mit_valid']==1 and d['mit_fault_code']==0 and
                    0 <= time.monotonic_ns()/1000-d['mit_rx_us'] <= 60000 and
                    abs(d['mit_erpm'])<=100 and abs(d['mit_iq_a'])<=.5 and
                    0<d['mit_temperature_c']<60 and abs(d['mit_position_deg']-ref[i])<=1.,
                    f'{i} feedback limit or stale: {d}')
    def send(active):
        msg = JointTrajectory(joint_names=JOINTS)
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.points = [JointTrajectoryPoint(effort=setpoints(axis,sign,active))]
        pub.publish(msg)
    try:
        for phase, duration in [('baseline',BASE_S),('pulse',STEP_S),('release',RELEASE_S)]:
            start = time.monotonic_ns()
            events[phase] = start
            print(phase, axis, sign if phase=='pulse' else 0,flush=True)
            next_send = start
            while time.monotonic_ns()-start < duration*1e9:
                rclpy.spin_once(node,timeout_sec=.002)
                check()
                now = time.monotonic_ns()
                if now>=next_send:
                    require(now-next_send<45_000_000, 'Publisher deadline missed')
                    send(phase=='pulse')
                    next_send=now+10_000_000
            if phase=='baseline':
                status(node,client)
                for i in IDS:
                    tail=[s for s in samples[i] if s[0]>start+300_000_000]
                    require(len(tail)>=15 and all(abs(s[1])<=.2 and abs(s[3])<=20 for s in tail),
                            'Unstable nominal-zero baseline')
        status(node,client)
        events['end']=time.monotonic_ns()
    except BaseException:
        # Stop renewal. The RT stop latches both axes and emits one zero even at full effort.
        # Keep the manager live for the transport to consume it, with a bounded TTL fallback.
        events['supervisor_stop'] = time.monotonic_ns()
        if rclpy.ok() and manager.poll() is None:
            future = stop_client.call_async(Trigger.Request())
            rclpy.spin_until_future_complete(node, future, timeout_sec=.025)
            until = events['supervisor_stop'] + 85_000_000
            while time.monotonic_ns() < until and manager.poll() is None:
                time.sleep(.002)
        raise


def analyze(out, axis, sign, events):
    result={}
    for mid,bus in [(104,2),(105,1)]:
        with (out/f'bus{bus}_motor{mid}.jsonl').open() as f:
            header=json.loads(next(f))
            rows=[json.loads(l) for l in f]
        require(header['dropped']==0 and header['total']==len(rows),'Trace incomplete')
        tx=[r for r in rows if r['stage']=='transport_result']
        rx=[mit.decode_rx(r) for r in rows if r['stage']=='transport_receive' and r['id']==0x2900+mid]
        expected=int((15+sign*EFFORT)*4095/30) if mid==axis else 2047
        codes=[]
        for r in tx:
            require(r['result']==0 and r['flags']==4 and r['deadline']==bus and r['id']==0x800+mid and
                    len(r['hex'])==16 and r['hex'][:13]==mit.ZERO_WIRE[:13],'Unexpected MIT frame')
            c=int(r['hex'][-3:],16)
            require(min(2047,expected)<=c<=max(2047,expected),'Out-of-envelope MIT torque')
            codes.append(c)
        require(codes and expected in codes and codes[-1]==2047,'Pulse or final nominal zero missing')
        start=events['pulse']; release=events['release']; end=events['end']
        def window(lo, hi):
            a=[x for x in rx if lo<=x['ns']<=hi]
            require(len(a)>=4 and all(x['status']==0 for x in a),'Feedback window incomplete')
            return a
        base=window(start-450_000_000,start)
        hold=window(start+225_000_000,release)
        tail=window(end-450_000_000,end)
        b=statistics.median(x['iq_a'] for x in base)
        h=statistics.median(x['iq_a'] for x in hold)
        t=statistics.median(x['iq_a'] for x in tail)
        mad=statistics.median(abs(x['iq_a']-b) for x in base)
        band=max(.05,3*mad)
        result[mid]=dict(baseline_iq=b,hold_iq=h,tail_iq=t,delta_iq=h-b,deadband_a=band,
            position_change_deg=round(hold[-1]['position_deg']-base[-1]['position_deg'],3),
            tx_codes=[min(codes),max(codes)],max_abs_erpm=max(abs(v['erpm']) for v in rx),
            iq_sign_visible=sign*(h-b)>band if mid==axis else None,
            released=(abs(t-b)<=band and max(abs(x['erpm']) for x in tail)<=20 and
                      max(x['position_deg'] for x in tail)-min(x['position_deg'] for x in tail)<=.2))
    return result


def run(out, axis, sign, simulated, inject_trip=False):
    import fcntl
    import rclpy
    from ament_index_python.packages import get_package_prefix
    from contextlib import ExitStack
    cfg=mit.selected_config(json.loads((mit.ROOT/'config/parallel_exo/servo_pair.ref01.json').read_text()),[104,105])
    paths={m['id']:next(b['device_path'] for b in cfg['buses'] if b['logical_bus']==m['logical_bus']) for m in cfg['motors']}
    if not simulated:
        require(paths=={104:'/dev/serial/by-id/usb-Livelybot_Livelybot_Communication_Board_2024051701-if02',
                         105:'/dev/serial/by-id/usb-Livelybot_Livelybot_Communication_Board_2024051701-if00'},
                'Unexpected physical bus assignment')
    required_root=(mit.ROOT/'.local/validation').resolve()
    require(required_root in out.parents and not out.exists(),'New validation directory required')
    out.mkdir(parents=True)
    os.environ.update(ROS_DOMAIN_ID='163',ROS_LOCALHOST_ONLY='1', ROS_LOG_DIR=str(out/'ros_logs'))
    bring=Path(get_package_prefix('mech_bringup')).resolve()
    require(bring.parent==(mit.ROOT/'.local/ros/servo_imu/install').resolve(),'Wrong development overlay')
    binary=bring/'lib/mech_bringup'
    mbin=Path(get_package_prefix('controller_manager'))/'lib/controller_manager/ros2_control_node'
    ns='/ph_pulse_'+uuid4().hex
    with ExitStack() as stack:
        for lockname in ('mit_communication.lock','sync.lock'):
            file=stack.enter_context((mit.ROOT/'.local'/lockname).open('a+'))
            fcntl.flock(file,fcntl.LOCK_EX|fcntl.LOCK_NB)
        require(not (mit.ROOT/'.local/sync/incomplete.json').exists(),'Incomplete source sync')
        gates=[]
        if simulated:
            from test_mit_pair_pty import PairGateway
            class TravelTripGateway(PairGateway):
                def send_feedback(self, acknowledge=False):
                    moved = any(r['id']==0x800+self.drive and
                                int(r['hex'][-3:],16)==int((15+sign*EFFORT)*4095/30)
                                for r in reversed(self.rows[-90:]))
                    position = 715 if moved else 700
                    body = struct.pack('<IBB', 0x2900+self.drive, 4, 8)
                    sample = struct.pack('>hhhBB',position,0,0,41,119 if acknowledge else 0)
                    from benchmark_mit_pty import packet
                    os.write(self.master,packet(body+sample))

            for i in IDS:
                g=TravelTripGateway(i) if inject_trip and i==axis else PairGateway(i)
                gates.append(g)
                stack.callback(g.close)
                paths[i]=g.path
        else:
            for path in paths.values():
                require(Path(path).resolve(strict=True).is_char_device(),'Physical port missing')
                check=subprocess.run(['fuser',path],capture_output=True)
                require(check.returncode==1 and not check.stdout and not check.stderr, 'Port is busy or cannot check owner')
        params, disable=make_artifacts(cfg,paths,out,ns)
        desc=out/'disable.urdf';desc.write_text(disable)
        pf=out/'controls.json';pf.write_text(json.dumps(params))
        subprocess.run([str(binary/'servo_disable'),'--urdf',str(desc),'--check'],check=True,timeout=4)
        ref=passive(binary,desc,out)
        print('Preflight angles:',ref,flush=True)
        rclpy.init()
        node=rclpy.create_node('pulse_observer',namespace=ns)
        stack.callback(lambda:rclpy.try_shutdown())
        stack.callback(node.destroy_node)
        events={};data=dict(axis=axis,sign=sign,simulated=simulated,events=events)
        manager=None
        log=stack.enter_context((out/'manager.log').open('w'))
        try:
            manager=subprocess.Popen([str(mbin),'--ros-args','-r','__ns:='+ns,'--params-file',str(pf)],
                stdout=log,stderr=log,start_new_session=True)
            for ctrl in ('joint_state_broadcaster','pulse'):
                subprocess.run(['ros2','run','controller_manager','spawner',ctrl,'-c',ns+'/controller_manager',
                    '--controller-manager-timeout','12','--param-file',str(pf)],check=True,timeout=20,stdout=log,stderr=log)
            drive_window(node,manager,ns,axis,sign,ref,events)
        except BaseException as exc:
            data['error']=repr(exc)
        finally:
            try:
                mit.stop_child(manager)
                require(manager is not None and manager.poll() is not None,'Manager/port release uncertain')
                ack=subprocess.run([str(binary/'servo_disable'),'--urdf',str(desc),'--trace',str(out/'disable.jsonl'),
                                    '--timeout','1'],capture_output=True,text=True,timeout=7)
                (out/'disable.log').write_text(ack.stdout+ack.stderr)
                lines=[json.loads(l) for l in ack.stdout.splitlines() if l.startswith('{')]
                ids=sorted(x['id'] for x in lines if x.get('event')=='disable_ack')
                data['disable_ack']=ids
                require(ack.returncode==0 and ids==list(IDS) and any(x.get('event')=='disable_complete' and
                        x.get('confirmed') and not x.get('missing') for x in lines),'Disable unconfirmed; cut power')
            except BaseException as exc:
                data['shutdown_error']=repr(exc)
            (out/'result.json').write_text(json.dumps(data,indent=2))
        if 'supervisor_stop' in events:
            for mid,bus in ((104,2),(105,1)):
                path=out/f'bus{bus}_motor{mid}.jsonl'
                with path.open() as stream:
                    header=json.loads(next(stream))
                    rows=[json.loads(l) for l in stream]
                tx=[r for r in rows if r['stage']=='transport_result' and r['id']==0x800+mid]
                final=tx[-1] if tx else {}
                checked=(header['dropped']==0 and header['total']==len(rows) and
                         final.get('hex')==mit.ZERO_WIRE and final.get('result')==0 and
                         final.get('ns',0)>=events['supervisor_stop'])
                data.setdefault('abort_final_zero',{})[str(mid)]=checked
                data.setdefault('abort_final_delay_ms',{})[str(mid)]=(round(
                    (final['ns']-events['supervisor_stop'])/1e6,3) if final else None)
            (out/'result.json').write_text(json.dumps(data,indent=2))
        if inject_trip:
            require(simulated and 'feedback limit' in data.get('error','') and not data.get('shutdown_error')
                    and all(data.get('abort_final_zero',{}).get(str(i)) for i in IDS)
                    and all(not g.errors for g in gates),str(data))
            print('PTY position trip: both final zero writes and disable acknowledgements verified.', data, flush=True)
            return
        require('shutdown_error' not in data and 'error' not in data,str(data))
        data['axes']=analyze(out,axis,sign,events)
        (out/'result.json').write_text(json.dumps(data,indent=2))
        print(json.dumps(data),flush=True)
        require(all(v['released'] for v in data['axes'].values()),'Release not established')
        require(all(not g.errors for g in gates),'Private gateway error')


def main():
    p=argparse.ArgumentParser(description=__doc__)
    mode=p.add_mutually_exclusive_group(required=True)
    for name in ('check','rehearse','rehearse-trip','execute'):
        mode.add_argument('--'+name,action='store_true')
    p.add_argument('--axis',type=int,choices=IDS)
    p.add_argument('--sign',type=int,choices=[-1,1])
    p.add_argument('--output',type=Path)
    a=p.parse_args()
    if a.check:
        cfg=json.loads((mit.ROOT/'config/parallel_exo/servo_pair.ref01.json').read_text())
        paths={104:'/dev/pts/104',105:'/dev/pts/105'}
        params,_=make_artifacts(cfg,paths,Path('.'),'/check')
        assert params['/check/pulse']['ros__parameters']['max_effort_nm']==.1
        for i in IDS:
            for sign in (-1,1):
                v=setpoints(i,sign,True)
                assert v[IDS.index(i)]==sign*.1 and v[1-IDS.index(i)]==0
                assert setpoints(i,sign,False)==[0.,0.]
        print('Offline check passed. No device I/O.')
        return
    if a.axis is None or a.sign is None or not a.output:
        p.error('Supply --axis, --sign and --output')
    def interrupted(*args):
        raise KeyboardInterrupt('Interrupted')
    for sig in (signal.SIGTERM,signal.SIGHUP):
        signal.signal(sig,interrupted)
    run(a.output.resolve(),a.axis,a.sign,a.rehearse or a.rehearse_trip,a.rehearse_trip)


if __name__=='__main__':
    main()
