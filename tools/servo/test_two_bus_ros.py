"""Two independent synthetic USB gateways; never opens a physical device."""
import fcntl
import gzip
import json
import os
from pathlib import Path
import pty
import select
import struct
import subprocess
import tempfile
import threading
import time
import tty
import unittest
import xml.etree.ElementTree as ET
from ament_index_python.packages import get_package_prefix
from test_two_bus_config import ROOT, config_for, op, load

audit = load('chain_audit')
BIN = Path(get_package_prefix('mech_bringup'))/'lib/mech_bringup'


class Gateway:
    def __init__(self, drive, angle):
        self.master, self.slave = pty.openpty()
        tty.setraw(self.slave)
        self.path = os.ttyname(self.slave)
        self.drive, self.angle = drive, angle
        self.commands, self.errors = [], []
        self.receive = True
        self.ack = True
        self.done = threading.Event()
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.thread.start()

    def feedback(self, status=0):
        body = struct.pack('<IBB',0x2900+self.drive,4,8)+struct.pack('>hhhBB',round(self.angle*10),0,0,40,status)
        head = b'\x12'+struct.pack('<H',len(body))
        return b'\xf7'+head+bytes([audit.crc(head,255,0x8c)])+struct.pack('<H',audit.crc(body,65535,0x8408))+body

    def run(self):
        pending = b''
        next_feedback = 0.
        try:
            while not self.done.is_set():
                if select.select([self.master],[],[],.002)[0]: pending += os.read(self.master,4096)
                while len(pending)>=7:
                    size = int.from_bytes(pending[2:4],'little')+7
                    if len(pending)<size: break
                    packet, pending = pending[:size], pending[size:]
                    body = packet[7:]
                    if body==bytes(6): continue
                    if packet[4]!=audit.crc(packet[1:4],255,0x8c) or int.from_bytes(packet[5:7],'little')!=audit.crc(body,65535,0x8408):
                        raise AssertionError('bad outgoing CRC')
                    ident = int.from_bytes(body[:4],'little')
                    self.commands.append(ident)
                    if ident==0x600+self.drive: self.angle=struct.unpack('>i',body[6:10])[0]/10000.
                    elif ident==0xf00+self.drive:
                        if self.ack: os.write(self.master,self.feedback(0x77))
                    else: raise AssertionError('wrong motor routed to port: '+hex(ident))
                now = time.monotonic()
                if self.receive and now>=next_feedback:
                    os.write(self.master,self.feedback()); next_feedback=now+.005
        except Exception as exc: self.errors.append(str(exc))

    def close(self):
        self.done.set(); self.thread.join(2)
        os.close(self.master); os.close(self.slave)


class TwoBusRosTests(unittest.TestCase):
    def setUp(self):
        self.gateways = [Gateway(105,10.), Gateway(104,20.)]
        self.temp = tempfile.TemporaryDirectory()
        self.root = Path(self.temp.name)
        self.config = config_for([g.path for g in self.gateways])
        self.urdf = self.root/'control.urdf'
        self.urdf.write_text(op.generate_urdf(self.config))

    def tearDown(self):
        for gateway in self.gateways: gateway.close()
        self.temp.cleanup()
        self.assertFalse([e for g in self.gateways for e in g.errors])

    def disable(self):
        result = subprocess.run([str(BIN/'servo_disable'),'--urdf',str(self.urdf),
                                 '--trace',str(self.root/'disable.jsonl'),'--timeout','.2'],
                                capture_output=True,text=True,timeout=5)
        return result, [json.loads(row) for row in result.stdout.splitlines()]

    def test_duplicate_ports_are_rejected_before_any_io(self):
        root=ET.fromstring(self.urdf.read_text())
        ports=root.findall("./ros2_control/hardware/param[@name='device_path']")
        ports[1].text=ports[0].text
        self.urdf.write_text(ET.tostring(root,encoding='unicode'))
        for tool in ('servo_disable','servo_feedback_reader'):
            result=subprocess.run([str(BIN/tool),'--urdf',str(self.urdf),'--check'],
                                  capture_output=True,text=True,timeout=5)
            self.assertNotEqual(result.returncode,0)
        self.assertEqual([g.commands for g in self.gateways],[[],[]])

    def test_disable_attempts_second_bus_when_first_port_is_occupied(self):
        fcntl.flock(self.gateways[0].slave,fcntl.LOCK_EX|fcntl.LOCK_NB)
        result, rows = self.disable()
        self.assertNotEqual(result.returncode,0)
        self.assertEqual(rows[-1]['missing'],[105])
        self.assertEqual(self.gateways[0].commands,[])
        self.assertEqual(self.gateways[1].commands,[0xf68])
        self.assertTrue(any(row.get('event')=='disable_ack' and row['id']==104 for row in rows))

    def test_disable_missing_ack_still_attempts_other_bus_without_resend(self):
        self.gateways[0].ack=False
        result, rows = self.disable()
        self.assertNotEqual(result.returncode,0)
        self.assertEqual(rows[-1]['missing'],[105])
        self.assertEqual([g.commands for g in self.gateways],[[0xf69],[0xf68]])

    def test_reader_merges_fresh_feedback_and_fails_on_missing_bus(self):
        for missing in (False,True):
            self.gateways[1].receive=not missing
            result = subprocess.run([str(BIN/'servo_feedback_reader'),'--urdf',str(self.urdf),
                                     '--duration','.4'],capture_output=True,text=True,timeout=5)
            rows=[json.loads(row) for row in result.stdout.splitlines()]
            self.assertTrue(rows,result.stderr)
            self.assertEqual(result.returncode==0,not missing,result.stderr)
            self.assertTrue(all(row['motor_command_frames']==0 for row in rows))
            self.assertEqual({m['id'] for m in rows[-1]['motors']},{104,105})
        self.assertEqual([g.commands for g in self.gateways],[[],[]])

    def test_feedback_loss_on_one_bus_stops_framework_and_disables_both(self):
        self.test_real_framework_routing_restart_disable_and_separate_traces(fault=True)

    def test_real_framework_routing_restart_disable_and_separate_traces(self, fault=False):
        config_path=self.root/'config.json'; config_path.write_text(json.dumps(self.config))
        before=config_path.read_bytes()
        master,slave=pty.openpty()
        proc=subprocess.Popen(['/usr/bin/python3',str(ROOT/'motion.py'),'--config',str(config_path),
                               '--run-dir',str(self.root/'runs'),'--disable-helper',str(BIN/'servo_disable')],
                              stdin=slave,stdout=slave,stderr=slave,start_new_session=True)
        os.close(slave); pending=bytearray()
        def expect(text,timeout=25.):
            end=time.monotonic()+timeout
            while time.monotonic()<end:
                index=pending.find(text.encode())
                if index>=0:
                    del pending[:index+len(text.encode())];return
                if select.select([master],[],[],.05)[0]:
                    try: pending.extend(os.read(master,65536))
                    except OSError: break
            logs={p.name:p.read_text(errors='replace')[-6000:]
                  for p in self.root.glob('runs/move-*/framework.log')}
            self.fail('Missing '+text+': '+pending.decode(errors='replace')+'; framework='+json.dumps(logs))
        def command(text,expected):
            os.write(master,(text+'\n').encode());expect(expected)
        try:
            expect('控制器已就绪')
            self.assertEqual([g.commands for g in self.gateways],[[],[]])
            command('enable','已使能')
            if fault:
                self.gateways[0].receive=False
                expect('结束本次操作')
                expect('失能步骤已执行')
                self.assertNotEqual(proc.wait(timeout=25),0)
                for gateway in self.gateways:
                    self.assertEqual(gateway.commands.count(0xf00+gateway.drive),1)
                return
            command('move 23 12 0.8','运动中');expect('轨迹到达容差内')
            command('step -1 1 0.8','运动中');expect('轨迹到达容差内')
            command('disable','电机已失能；会话保留')
            command('enable','已使能')
            command('quit','失能步骤已执行')
            self.assertEqual(proc.wait(timeout=25),0)
            self.assertEqual(config_path.read_bytes(),before)
            run=next((self.root/'runs').glob('move-*'))
            for directory in (run,run/'backend-002'):
                manifest=json.loads((directory/'capture-manifest.json').read_text())
                self.assertEqual(manifest['diagnostic_status'],'complete',manifest)
                self.assertEqual(len(manifest['snapshots']),4)
                events=[json.loads(row) for row in (directory/'events.jsonl').read_text().splitlines()]
                for bus,drive in ((1,105),(2,104)):
                    with gzip.open(directory/f'command-chain-bus-{bus}.jsonl.gz','rt') as stream:
                        report=audit.audit_records(json.loads(row) for row in stream)
                    self.assertTrue(report['complete'],report)
                    self.assertEqual(set(report['drives']),{drive})
                    self.assertEqual(audit.audit_operator(events,report['acquisitions']),[])
            for gateway in self.gateways:
                self.assertEqual(gateway.commands.count(0xf00+gateway.drive),2)
                self.assertIn(0x600+gateway.drive,gateway.commands)
        finally:
            if proc.poll() is None:
                proc.terminate(); proc.wait(timeout=25)
            os.close(master)


if __name__=='__main__': unittest.main()
