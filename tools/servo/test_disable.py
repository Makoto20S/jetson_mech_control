"""Exercise production disable codec/transport/serial on a PTY; no hardware."""
import fcntl
import importlib.util
import json
import os
from pathlib import Path
import select
import struct
import subprocess
import sys
import tempfile
import time
import unittest

BINARY = sys.argv.pop(1) if len(sys.argv)>1 else 'servo_disable'
ROOT=Path(__file__).resolve().parent
spec=importlib.util.spec_from_file_location('servo_config',ROOT/'servo_operator.py')
config_tools=importlib.util.module_from_spec(spec);spec.loader.exec_module(config_tools)

def crc(data,poly,initial):
    for b in data:
        initial^=b
        for _ in range(8):initial=(initial>>1)^(poly if initial&1 else 0)
    return initial

def ack(drive,status=0x77):
    body=struct.pack('<IBB',0x2900+drive,4,8)+struct.pack('>hhhBB',697,0,0,40,status)
    head=bytes([0x12])+struct.pack('<H',len(body))
    return b'\xf7'+head+bytes([crc(head,0x8c,255)])+struct.pack('<H',crc(body,0x8408,65535))+body

class DisableTests(unittest.TestCase):
    def run_case(self, replies=(104,105), occupied=False, check=False, wrong=False, bad_trace=False):
        master,slave=os.openpty()
        try:
            with tempfile.TemporaryDirectory() as directory:
                path=Path(directory)
                config=json.loads((ROOT/'config.example.json').read_text())
                config['device_path']=os.ttyname(slave)
                urdf=path/'control.urdf';urdf.write_text(config_tools.generate_urdf(config,observe=True))
                if occupied:fcntl.flock(slave,fcntl.LOCK_EX|fcntl.LOCK_NB)
                command=[BINARY,'--urdf',str(urdf),'--trace',str(path/'missing'/'trace.jsonl' if bad_trace else path/'trace.jsonl'),'--timeout','0.3']
                if check:command+=['--check']
                proc=subprocess.Popen(command,stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True)
                buf=b'';packets=[];deadline=time.monotonic()+5
                while time.monotonic()<deadline:
                    if select.select([master],[],[],.01)[0]:buf+=os.read(master,4096)
                    while len(buf)>=7:
                        size=int.from_bytes(buf[2:4],'little')+7
                        if len(buf)<size:break
                        packet,buf=buf[:size],buf[size:];packets.append(packet)
                        self.assertEqual(packet[0],247)
                        self.assertEqual(packet[4],crc(packet[1:4],0x8c,255))
                        self.assertEqual(int.from_bytes(packet[5:7],'little'),crc(packet[7:],0x8408,65535))
                        body=packet[7:]
                        if body!=bytes(6):
                            drive=int.from_bytes(body[:4],'little')&255
                            if drive in replies:
                                if wrong:
                                    os.write(master,ack(drive,0))
                                    damaged=bytearray(ack(drive));damaged[-1]^=1;os.write(master,damaged)
                                    os.write(master,ack(106))
                                else:os.write(master,ack(drive))
                    if proc.poll() is not None:break
                out,err=proc.communicate(timeout=2)
                rows=[json.loads(l) for l in out.splitlines()]
                trace=[json.loads(l) for l in (path/'trace.jsonl').read_text().splitlines()] if (path/'trace.jsonl').exists() else []
                return proc.returncode,packets,rows,trace,err
        finally:os.close(master);os.close(slave)

    def test_both_confirmed_only_two_mode15_frames(self):
        code,packets,rows,trace,err=self.run_case()
        self.assertEqual(code,0,err)
        self.assertEqual([p[7:].hex() for p in packets],['000000000000','680f00000c00','690f00000c00'])
        self.assertEqual({r['id'] for r in rows if r['event']=='disable_ack'},{104,105})
        self.assertTrue(rows[-1]['confirmed'])
        writes=[r for r in trace[1:] if r['stage']=='syscall_write']
        self.assertEqual([bytes.fromhex(r['hex']) for r in writes],packets)
        self.assertTrue(all(r['result']==r['requested'] for r in writes))

    def test_unwritable_trace_does_not_suppress_disable(self):
        code,packets,rows,_,err=self.run_case(bad_trace=True)
        self.assertEqual(code,0,err);self.assertEqual(len(packets),3)
        self.assertTrue(rows[-1]['confirmed']);self.assertIn('trace',err)

    def test_missing_one_ack_fails_and_no_resend(self):
        code,packets,rows,_,_=self.run_case(replies=(104,))
        self.assertNotEqual(code,0);self.assertEqual(len(packets),3)
        self.assertFalse(rows[-1]['confirmed']);self.assertEqual(rows[-1]['missing'],[105])

    def test_normal_bad_crc_and_wrong_id_are_not_ack(self):
        code,_,rows,_,_=self.run_case(wrong=True)
        self.assertNotEqual(code,0);self.assertFalse(rows[-1]['confirmed'])

    def test_occupied_port_sends_nothing(self):
        code,packets,_,_,_=self.run_case(occupied=True)
        self.assertNotEqual(code,0);self.assertEqual(packets,[])

    def test_check_never_opens_occupied_port(self):
        code,packets,_,_,err=self.run_case(occupied=True,check=True)
        self.assertEqual(code,0,err);self.assertEqual(packets,[])

if __name__=='__main__':unittest.main()
