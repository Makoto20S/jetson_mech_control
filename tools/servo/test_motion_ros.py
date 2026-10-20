"""Offline controller acceptance with loopback and production-plugin synthetic PTYs."""
import importlib.util
import gzip
import math
import os
import pty
import select
import struct
import json
from pathlib import Path
import subprocess
import tempfile
import time
import threading
import tty
import unittest
import uuid
from pty_clock_fixture import audit_trace, ClockPeer
from test_artifacts import retain

ROOT=Path(__file__).resolve().parent
spec=importlib.util.spec_from_file_location('servo_motion',ROOT/'motion.py')
motion=importlib.util.module_from_spec(spec); spec.loader.exec_module(motion)


def descendant_pids(pid, proc_root=Path('/proc')):
    pending=[pid];seen=set();parents=None
    while pending:
        current=pending.pop()
        if current in seen:continue
        seen.add(current)
        # Linux records a child against the task which created it. ROS launch
        # may spawn the manager from a worker thread rather than its main task.
        children_paths=list((proc_root/str(current)/'task').glob('*/children'))
        readable=False
        for children in children_paths:
            try:
                pending.extend(map(int,children.read_text().split()));readable=True
            except OSError:continue
        if not readable:
            # Some Jetson kernels omit the task children interface entirely.
            # Parse PPid after the final comm delimiter; comm may contain spaces
            # and parentheses. Keep the actual loaded-library assertion strict.
            if parents is None:
                parents={}
                for stat in proc_root.glob('[0-9]*/stat'):
                    try:
                        fields=stat.read_text().rsplit(') ',1)[1].split()
                        parents.setdefault(int(fields[1]),[]).append(int(stat.parent.name))
                    except (OSError,ValueError,IndexError):continue
            pending.extend(parents.get(current,[]))
    return seen


class ProcessTreeTests(unittest.TestCase):
    def test_children_created_by_non_main_thread_are_included_recursively(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory)
            for process,thread,children in ((10,10,''),(10,11,'20'),(20,20,'30'),(30,30,'')):
                path=root/str(process)/'task'/str(thread)/'children'
                path.parent.mkdir(parents=True,exist_ok=True);path.write_text(children)
            self.assertEqual(set(descendant_pids(10,root)),{10,20,30})

    def test_stat_parent_fallback_without_children_handles_spaces_and_parentheses(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory)
            for pid,parent,comm in ((10,1,'operator worker'),(20,10,'launch ) worker'),(30,20,'manager (node)'),(40,1,'unrelated')):
                path=root/str(pid)/'stat';path.parent.mkdir()
                path.write_text(f'{pid} ({comm}) S {parent} 0 0 0')
            self.assertEqual(set(descendant_pids(10,root)),{10,20,30})


class RosMotionTests(unittest.TestCase):
    def test_terminal_workflow_and_config_preservation(self):
        from ament_index_python.packages import get_package_prefix
        helper=Path(get_package_prefix('mech_bringup'))/'lib/mech_bringup/servo_disable'
        serial_master,serial_slave=pty.openpty()
        serial_packets=[]
        serial_done=threading.Event()
        # Hand-checked mode15 ACK fixtures (normal-feedback frame, status 0x77).
        replies={104:bytes.fromhex('f7120e000bdffd68290000040802b9000000002877'),
                 105:bytes.fromhex('f7120e000b358369290000040802b9000000002877')}
        def serial_gateway():
            pending=b''
            while not serial_done.is_set():
                if not select.select([serial_master],[],[],.05)[0]:
                    continue
                pending+=os.read(serial_master,4096)
                while len(pending)>=7:
                    length=int.from_bytes(pending[2:4],'little')+7
                    if len(pending)<length:
                        break
                    packet,pending=pending[:length],pending[length:]
                    serial_packets.append(packet)
                    body=packet[7:]
                    if len(body)==6 and body!=bytes(6):
                        drive=int.from_bytes(body[:4],'little')&255
                        if drive in replies:
                            os.write(serial_master,replies[drive])
        gateway=threading.Thread(target=serial_gateway,daemon=True)
        gateway.start()
        config=motion.load_config(ROOT/'config.example.json',require_calibrated=False)
        config['calibrated']=True
        config['device_path']=os.ttyname(serial_slave)
        for motor in config['motors']:
            motor.update(position_min_rad=-2.,position_max_rad=2.)
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'config.json';path.write_text(json.dumps(config))
            before=path.read_bytes()
            driver=Path(directory)/'loopback_cli.py'
            # Test-only substitution: production entry has no simulation bypass flag.
            driver.write_text("import importlib.util\n"
                +"import struct\n"
                +"s=importlib.util.spec_from_file_location('motion',"+repr(str(ROOT/'motion.py'))+")\n"
                +"m=importlib.util.module_from_spec(s);s.loader.exec_module(m)\n"
                +"original=m.config_tools.generate_urdf\n"
                +"m.config_tools.generate_urdf=lambda c: original(c).replace('mech_bringup/Ak30ServoSystem','mech_hardware_ros2_control/CompositeSystem')\n"
                +"original_disable=m.disable_motors\n"
                +"def disable(helper,urdf,run,record,ids):\n"
                +"    device_urdf=run/'disable.urdf'\n"
                +"    device_urdf.write_text(urdf.read_text().replace('mech_hardware_ros2_control/CompositeSystem','mech_bringup/Ak30ServoSystem'))\n"
                +"    original_disable(helper,device_urdf,run,record,ids)\n"
                +"m.disable_motors=disable\n"
                +"original_finalize=m.storage.finalize\n"
                +"def finalize(snapshot,run,session_root,*args,**kwargs):\n"
                +"    # CompositeSystem intentionally has no servo recorder. Test-only closed empty snapshots.\n"
                +"    for name,kind,stride in [('chain.snapshot',1,216),('serial.snapshot',2,1064)]:\n"
                +"        header=bytearray(128);header[:8]=b'MCHTRC01'\n"
                +"        struct.pack_into('<4I6Q',header,8,1,kind,stride,128,1,0,0,0,0,1)\n"
                +"        (snapshot/name).write_bytes(header+bytes(stride))\n"
                +"    return original_finalize(snapshot,run,session_root,*args,**kwargs)\n"
                +"m.storage.finalize=finalize\n"
                +"raise SystemExit(m.main())\n")
            master,slave=pty.openpty()
            proc=subprocess.Popen(['/usr/bin/python3',str(driver),'--config',str(path),
                '--run-dir',str(Path(directory)/'runs'),'--disable-helper',str(helper)],stdin=slave,stdout=slave,stderr=slave,
                start_new_session=True)
            os.close(slave)
            received=bytearray()
            def expect(text,timeout=15.):
                deadline=time.monotonic()+timeout
                while time.monotonic()<deadline:
                    end=received.find(text.encode())
                    if end>=0:
                        del received[:end+len(text.encode())]
                        return
                    if select.select([master],[],[],.1)[0]:
                        try:received.extend(os.read(master,65536))
                        except OSError:break
                self.fail('Missing '+text+'; output='+received.decode(errors='replace'))
            try:
                expect('控制器已就绪',timeout=25.)
                os.write(master,b'enable\n');expect('已使能',timeout=25.)
                os.write(master,b'move 5 -3 0.6\n');expect('轨迹到达容差内')
                received.clear()
                os.write(master,b'step 2 -1 0.6\n');expect('运动中');expect('轨迹到达容差内')
                os.write(master,b'disable\n');expect('电机已失能；会话保留')
                self.assertIsNone(proc.poll())
                run=next((Path(directory)/'runs').glob('move-*'))
                self.assertFalse((run/'backend-002').exists())
                expect('上次测量')
                os.write(master,b'move 1 1 0.6\n');expect('拒绝：请先enable')
                os.write(master,b'step 1 1 0.6\n');expect('拒绝：请先enable')
                self.assertFalse((run/'backend-002').exists())
                os.write(master,b'enable\n');expect('已使能',timeout=25.)
                os.write(master,b'step 2 -1 0.6\n');expect('运动中');expect('轨迹到达容差内')
                os.write(master,b'disable\n');expect('电机已失能；会话保留')
                self.assertIsNone(proc.poll())
                os.write(master,b'quit\n')
                self.assertEqual(proc.wait(timeout=15),0,received.decode(errors='replace'))
                self.assertEqual(path.read_bytes(),before)
                records=[json.loads(line) for line in next((Path(directory)/'runs').glob('*/events.jsonl')).read_text().splitlines()]
                relative=[r for r in records if r['event']=='relative_goal']
                self.assertEqual(len(relative),2)
                self.assertEqual(relative[0]['delta_device_deg'],[2.,-1.])
                index=records.index(relative[0])
                stop=next(i for i,r in enumerate(records[index+1:],index+1) if r['event']=='stop_requested')
                self.assertTrue(any(r['event']=='result' and r['status']==4 and r['error_code']==0
                                    for r in records[index+1:stop]))
                for actual,expected in zip(relative[0]['positions_rad'],[math.radians(7),math.radians(-4)]):
                    self.assertAlmostEqual(actual,expected,delta=math.radians(.5))
                self.assertEqual([r['state'] for r in records if r['event']=='controller'],
                                 ['active','inactive','active','inactive'])
                disables=[r for r in records if r['event']=='device_disable_result']
                self.assertEqual([r['returncode'] for r in disables],[0,0])
                self.assertEqual([p[7:].hex() for p in serial_packets],
                    ['000000000000','680f00000c00','690f00000c00']*2)
                # Loopback resets to zero on a new manager: stale 7/-4 must not
                # become the base of the second relative goal or takeover hold.
                for value in relative[1]['base_positions_rad']:
                    self.assertAlmostEqual(value,0.,delta=math.radians(.5))
                inputs=[r for r in records if r['event']=='operator_input']
                self.assertEqual(inputs[-1]['words'],['quit'])
                second=[json.loads(line) for line in (run/'backend-002'/'events.jsonl').read_text().splitlines()]
                self.assertTrue(any(r['event']=='relative_goal' for r in second))
                enabling=next(r for r in second if r['event']=='operator_input' and r['words']==['enable'])
                self.assertEqual(enabling,next(r for r in inputs if r['words']==['enable'] and r.get('backend')==2))
                self.assertLess(enabling['monotonic_s'],next(r['monotonic_s'] for r in second
                                                          if r['event']=='switch_request' and r['enabled']))
                for name in ('config.json','control.urdf','framework.log','disable.log','disable-trace.jsonl'):
                    self.assertTrue((run/name).exists())
                    self.assertTrue((run/'backend-002'/name).exists())
                first_disable=next(i for i,r in enumerate(records) if r['event']=='device_disable_result')
                second_enable=next(i for i,r in enumerate(records[first_disable+1:],first_disable+1)
                                   if r['event']=='operator_input' and r['words']==['enable'])
                self.assertFalse(any(r['event'] in ('feedback','switch_request','backend_started')
                                     for r in records[first_disable+1:second_enable]))
            finally:
                if proc.poll() is None:
                    proc.terminate()
                    proc.wait(timeout=15)
                os.close(master)
                serial_done.set();gateway.join(timeout=1.)
                os.close(serial_master);os.close(serial_slave)
                artifact=os.environ.get('SERVO_TEST_ARTIFACT_DIR')
                if artifact:
                    destination=Path(artifact)/('loopback-lifecycle-'+uuid.uuid4().hex[:8])
                    retain(directory,destination)

    def single_105_workflow(self):
        """Production Ak30ServoSystem and helper share a synthetic USB gateway."""
        from ament_index_python.packages import get_package_prefix
        prefix=Path(get_package_prefix('mech_bringup'))
        helper=prefix/'lib/mech_bringup/servo_disable'
        audit_spec=importlib.util.spec_from_file_location('chain_audit',ROOT/'chain_audit.py')
        audit=importlib.util.module_from_spec(audit_spec);audit_spec.loader.exec_module(audit)
        serial_master,serial_slave=pty.openpty()
        tty.setraw(serial_slave)
        serial_packets=[];packet_times=[];gateway_errors=[]
        serial_done=threading.Event()
        clock_peer=None
        def feedback(angle,status=0):
            body=struct.pack('<IBB',0x2969,4,8)+struct.pack('>hhhBB',round(angle*10),0,0,40,status)
            head=b'\x12'+struct.pack('<H',len(body))
            return b'\xf7'+head+bytes([audit.crc(head,255,0x8c)])+struct.pack('<H',audit.crc(body,65535,0x8408))+body
        def serial_gateway():
            pending=b'';angle=10.;next_feedback=0.;initializations=0
            try:
                while not serial_done.is_set():
                    if select.select([serial_master],[],[],.002)[0]:
                        pending+=os.read(serial_master,4096)
                    while len(pending)>=7:
                        length=int.from_bytes(pending[2:4],'little')+7
                        if len(pending)<length:break
                        packet,pending=pending[:length],pending[length:]
                        serial_packets.append(packet);packet_times.append(time.monotonic())
                        body=packet[7:]
                        if body==bytes(6):
                            initializations+=1
                            # Every other init belongs to a newly started framework.
                            if initializations%2:angle=10.+5.*(initializations//2)
                        else:
                            ident=int.from_bytes(body[:4],'little')
                            if ident==0x669:
                                angle=struct.unpack('>i',body[6:10])[0]/10000.
                            elif ident==0xf69:
                                os.write(serial_master,feedback(angle,0x77))
                            else:
                                raise AssertionError('Unexpected outgoing ID '+hex(ident))
                    now=time.monotonic()
                    if now>=next_feedback:
                        # Never synthesize 104 feedback: the production graph must
                        # become ready and move with the selected 105 alone.
                        os.write(serial_master,feedback(angle));next_feedback=now+.005
                    if clock_peer is not None:
                        clock_peer.exchange(lambda: os.write(serial_master,feedback(angle)))
            except Exception as exc:gateway_errors.append(str(exc))
        gateway=threading.Thread(target=serial_gateway,daemon=True);gateway.start()
        config=motion.load_config(ROOT/'config.example.json',require_calibrated=False)
        config['calibrated']=True;config['device_path']=os.ttyname(serial_slave)
        for motor in config['motors']:motor.update(position_min_rad=-2.,position_max_rad=2.)
        with tempfile.TemporaryDirectory() as directory:
            clock_peer=ClockPeer(Path(directory)/'runs/clocks/single.sock')
            path=Path(directory)/'config.json';path.write_text(json.dumps(config));before=path.read_bytes()
            master,slave=pty.openpty()
            proc=subprocess.Popen(['/usr/bin/python3',str(ROOT/'pty_clock_fixture.py'),'--config',str(path),
                '--run-dir',str(Path(directory)/'runs'),'--disable-helper',str(helper),'--motor-id','105'],
                stdin=slave,stdout=slave,stderr=slave,start_new_session=True)
            os.close(slave);received=bytearray();transcript=bytearray();loaded=set()
            def unexpected_exit():
                details={'returncode':proc.poll(), 'terminal':transcript.decode(errors='replace')[-6000:],
                         'gateway_errors':gateway_errors}
                for name in ('framework.log','events.jsonl','disable.log'):
                    files=sorted(Path(directory).glob('runs/move-*/'+name))
                    details[name]=[p.read_text(errors='replace')[-6000:] for p in files]
                self.fail('Operator exited during lifecycle test: '+json.dumps(details,ensure_ascii=False))
            def expect(text,timeout=15.):
                deadline=time.monotonic()+timeout
                while time.monotonic()<deadline:
                    end=received.find(text.encode())
                    if end>=0:
                        del received[:end+len(text.encode())];return
                    if select.select([master],[],[],.05)[0]:
                        try:data=os.read(master,65536)
                        except OSError:break
                        received.extend(data);transcript.extend(data)
                unexpected_exit()
            def capture_loaded_library():
                for pid in descendant_pids(proc.pid):
                    entry=Path('/proc')/str(pid)
                    try:
                        for line in (entry/'maps').read_text().splitlines():
                            if 'libmech_bringup.so' in line:loaded.add(line.split()[-1])
                    except (OSError,ProcessLookupError):continue
                self.assertIn(str((prefix/'lib/libmech_bringup.so').resolve()),loaded)
            try:
                expect('控制器已就绪');capture_loaded_library()
                initial_count=len(serial_packets)
                self.assertTrue(initial_count>=1)
                self.assertTrue(all(p[7:]==bytes(6) for p in serial_packets))
                os.write(master,b'move 12 0.6\n');expect('拒绝：请先enable')
                self.assertEqual(len(serial_packets),initial_count)
                os.write(master,b'enable\n');expect('已使能')
                os.write(master,b'move 12 0.6\n');expect('运动中');expect('轨迹到达容差内')
                # Drain the terminal for >30 s while the real controller holds.
                # There is no elapsed-enable-time shutdown in this operator.
                hold_until=time.monotonic()+31.
                while time.monotonic()<hold_until:
                    if proc.poll() is not None:unexpected_exit()
                    if select.select([master],[],[],.05)[0]:
                        try:data=os.read(master,65536)
                        except OSError:unexpected_exit()
                        if not data:unexpected_exit()
                        received.extend(data);transcript.extend(data)
                self.assertFalse(any(p[7:]!=bytes(6) and int.from_bytes(p[7:11],'little')==0xf69
                                     for p in serial_packets))
                received.clear()
                os.write(master,b'step 1 0.6\n');expect('运动中');expect('轨迹到达容差内')
                os.write(master,b'disable\n');expect('电机已失能；会话保留')
                self.assertIsNone(proc.poll())
                stopped_count=len(serial_packets)
                os.write(master,b'move 12 0.6\n');expect('拒绝：请先enable')
                self.assertEqual(len(serial_packets),stopped_count)
                os.write(master,b'enable\n');expect('已使能');capture_loaded_library()
                os.write(master,b'step 1 0.6\n');expect('运动中');expect('轨迹到达容差内')
                os.write(master,b'disable\n');expect('电机已失能；会话保留')
                stopped_count=len(serial_packets)
                os.write(master,b'quit\n')
                self.assertEqual(proc.wait(timeout=15.),0,received.decode(errors='replace'))
                self.assertEqual(len(serial_packets),stopped_count)
                self.assertEqual(path.read_bytes(),before)
                run=next((Path(directory)/'runs').glob('move-*'))
                records=[json.loads(line) for line in (run/'events.jsonl').read_text().splitlines()]
                self.assertFalse(any(r['event'] in ('enable_limit_started','auto_disable_requested') for r in records))
                relative=[r for r in records if r['event']=='relative_goal']
                self.assertEqual([r['delta_device_deg'] for r in relative],[[1.],[1.]])
                self.assertAlmostEqual(relative[0]['base_positions_rad'][0],math.radians(12),delta=math.radians(.2))
                self.assertAlmostEqual(relative[1]['base_positions_rad'][0],math.radians(15),delta=math.radians(.2))
                self.assertEqual(len([p for p in serial_packets if p[7:]!=bytes(6) and int.from_bytes(p[7:11],'little')==0xf69]),2)
                self.assertEqual({int.from_bytes(p[7:11],'little') for p in serial_packets if p[7:]!=bytes(6)},{0x669,0xf69})
                traces=list(run.glob('**/command-chain.jsonl.gz'))
                self.assertEqual(len(traces),2)
                for trace in traces:
                    for filename,loss_key in (('serial-trace.jsonl.gz','overwritten'),('disable-trace.jsonl','dropped')):
                        trace_path=trace.with_name(filename)
                        self.assertTrue(trace_path.exists(),str(trace_path))
                        opener=gzip.open if trace_path.suffix == '.gz' else open
                        with opener(trace_path,'rt') as stream:metadata=json.loads(next(stream))
                        self.assertEqual(metadata[loss_key],0,metadata)
                    report=audit_trace(trace,Path(directory)/'runs/clocks')
                    events=[json.loads(line) for line in trace.with_name('events.jsonl').read_text().splitlines()]
                    report['issues']+=audit.audit_operator(events,report['acquisitions'])
                    if not report['complete'] or report['issues']:
                        with gzip.open(trace,'rt') as stream: chain_header=json.loads(next(stream))
                        manifest=json.loads(trace.with_name('capture-manifest.json').read_text())
                        with trace.with_name('framework.log').open('rb') as stream:
                            stream.seek(max(0,trace.with_name('framework.log').stat().st_size-4000))
                            tail=stream.read().decode(errors='replace')
                        self.fail(json.dumps(dict(issues=report['issues'],chain_header=chain_header,
                            snapshots=manifest.get('snapshots'),diagnostics=manifest.get('diagnostics'),
                            framework_tail=tail),ensure_ascii=False,indent=2))
                    self.assertEqual(set(report['drives']),{105})
                    trace.with_name('chain-audit.json').write_text(json.dumps(report,indent=2))
                self.assertFalse(gateway_errors,gateway_errors)
            finally:
                if proc.poll() is None:proc.terminate();proc.wait(timeout=15.)
                os.close(master);serial_done.set();gateway.join(timeout=1.)
                clock_peer.close()
                os.close(serial_master);os.close(serial_slave)
                artifact=os.environ.get('SERVO_TEST_ARTIFACT_DIR')
                if artifact:
                    destination=Path(artifact)/('single-lifecycle-'+uuid.uuid4().hex[:8])
                    (Path(directory)/'terminal.txt').write_bytes(transcript)
                    (Path(directory)/'loaded-libraries.json').write_text(json.dumps(sorted(loaded),indent=2))
                    (Path(directory)/'gateway-tx.json').write_text(json.dumps([dict(monotonic_s=t,hex=p.hex()) for t,p in zip(packet_times,serial_packets)],indent=2))
                    retain(directory,destination)

    def test_single_105_real_framework_pty_lifecycle_and_chain(self):
        self.single_105_workflow()

    def test_pair_enable_move_stop_disable_and_stale_feedback(self):
        import rclpy
        from rclpy.signals import SignalHandlerOptions
        os.environ.setdefault('ROS_LOG_DIR','/tmp/servo-motion-ros-test')
        config=motion.load_config(ROOT/'config.example.json',require_calibrated=False)
        config['calibrated']=True
        for motor in config['motors']:
            motor.update(position_min_rad=-2.,position_max_rad=2.)
        policy=motion.MotionPolicy(config)
        # Exercise the exact launch/YAML/action client without opening hardware.
        urdf=motion.config_tools.generate_urdf(config).replace(
            'mech_bringup/Ak30ServoSystem','mech_hardware_ros2_control/CompositeSystem')
        events=[]
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'loopback.urdf';path.write_text(urdf)
            ready=Path(directory)/'controllers.ready'
            with (Path(directory)/'framework.log').open('w+') as log:
                namespace='servo_test_'+uuid.uuid4().hex[:8]
                proc=subprocess.Popen(['ros2','launch',str(ROOT/'servo_pair.launch.py'),
                    'urdf:='+str(path),'namespace:='+namespace,'ready_file:='+str(ready)],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
                rclpy.init(args=[],signal_handler_options=SignalHandlerOptions.NO)
                client=motion.RosControl(policy,namespace,lambda event,**data:events.append((event,data)))
                try:
                    deadline=time.monotonic()+15
                    while time.monotonic()<deadline:
                        client.spin(.02)
                        # Do not poll services while the graph is still starting.
                        if ready.exists() and client.feedback.fresh(time.monotonic()):
                            break
                    self.assertEqual(client.state(),'inactive')
                    self.assertTrue(client.feedback.fresh(time.monotonic()))
                    self.assertFalse(client.active)
                    with self.assertRaises(ValueError):client.send([.1,.1],1.)
                    client.enable()
                    target,seconds=policy.move(['5','-3','0.6'])
                    client.send(target,seconds)
                    deadline=time.monotonic()+4
                    while client.result is not None and time.monotonic()<deadline:client.poll()
                    self.assertIsNone(client.result)
                    for actual,expected in zip(client.feedback.positions,target):
                        self.assertAlmostEqual(actual,expected,delta=math.radians(.5))
                    client.move_relative(['2','-1','0.6'])
                    deadline=time.monotonic()+4
                    while client.result is not None and time.monotonic()<deadline:client.poll()
                    self.assertIsNone(client.result)
                    for actual,expected in zip(client.feedback.positions,[math.radians(7),math.radians(-4)]):
                        self.assertAlmostEqual(actual,expected,delta=math.radians(.5))
                    client.send([.5,-.5],2.)
                    deadline=time.monotonic()+.15
                    while time.monotonic()<deadline:client.poll()
                    client.stop()
                    deadline=time.monotonic()+3
                    while client.result is not None and time.monotonic()<deadline:client.poll()
                    self.assertIsNone(client.result)
                    self.assertLess(client.feedback.positions[0],.4)
                    client.disable()
                    self.assertEqual(client.state(),'inactive')
                    self.assertFalse(client.active)
                    self.assertTrue(any(e=='stop_requested' for e,_ in events))
                    # Reproduce the incident's misleading combination on actual manager:
                    # controller remains ACTIVE while the hardware is made unavailable.
                    client.enable()
                    from controller_manager_msgs.srv import SetHardwareComponentState
                    from lifecycle_msgs.msg import State
                    service=client.node.create_client(SetHardwareComponentState,
                        'controller_manager/set_hardware_component_state')
                    self.assertTrue(service.wait_for_service(timeout_sec=2.))
                    request=SetHardwareComponentState.Request()
                    request.name='servo_pair';request.target_state=State(id=1,label='unconfigured')
                    response=client.await_future(service.call_async(request))
                    self.assertTrue(response.ok)
                    self.assertEqual(client.state(),'active')
                    before=len([e for e,_ in events if e=='goal'])
                    with self.assertRaises(RuntimeError):client.check_hardware()
                    self.assertIsNotNone(client.hardware_error)
                    with self.assertRaises(RuntimeError):client.send([0.,0.],1.)
                    with self.assertRaises(RuntimeError):client.disable()
                    self.assertEqual(before,len([e for e,_ in events if e=='goal']))
                    self.assertTrue(any(e=='hardware' and not data['healthy'] for e,data in events))
                    client.feedback.received=-math.inf
                    with self.assertRaises(RuntimeError):client.ready_feedback()
                except Exception:
                    log.flush();log.seek(0);print(log.read())
                    raise
                finally:
                    try:client.disable()
                    except Exception:pass
                    motion.terminate_launch(proc)
                    client.node.destroy_node();rclpy.shutdown()


if __name__=='__main__':unittest.main()
