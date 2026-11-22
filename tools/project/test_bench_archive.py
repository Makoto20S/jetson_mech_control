"""No devices: archive integrity, cleanup acknowledgement, deployment separation."""
import copy
import io
import json
from pathlib import Path
import tempfile
import threading
import unittest
from unittest.mock import patch
import bench_archive as archive
import sync_orin
import ph_exo_bench_panel as panel
import ph_exo_bench_worker as worker

ROOT=Path(__file__).resolve().parents[2]
TMP=ROOT/'.local/validation/archive_unit'
TMP.mkdir(parents=True,exist_ok=True)

class Archive(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory(dir=TMP);self.addCleanup(self.temp.cleanup)
        self.base=Path(self.temp.name);self.run=self.base/'ref01_bench_123'
        self.run.mkdir();(self.run/'result.json').write_bytes(b'{"a":1}')
        (self.run/'ros_logs').mkdir();(self.run/'ros_logs'/'node.log').write_bytes(b'abc'*40000)
        self.events=[];self.sha,self.files=archive.send_archive(self.run,self.events.append)
    def receive(self,events=None):
        receiver=archive.ArchiveReceiver(self.base/'received')
        ack=None
        for event in events or self.events:ack=receiver.accept(event)
        return ack
    def test_complete_transfer_and_only_acknowledged_cleanup(self):
        self.assertEqual(self.receive(),self.sha)
        self.assertFalse(archive.cleanup_acknowledged(self.run,self.base,self.files,False))
        self.assertTrue(self.run.exists())
        self.assertTrue(archive.cleanup_acknowledged(self.run,self.base,self.files,True))
        self.assertFalse(self.run.exists())
        self.assertEqual((self.base/'received/ros_logs/node.log').read_bytes(),b'abc'*40000)
    def test_corruption_retains_source(self):
        events=copy.deepcopy(self.events);events[-1]['sha256']='0'*64
        with self.assertRaises(ValueError):self.receive(events)
        self.assertTrue(self.run.exists())
    def test_missing_chunk_rejected(self):
        with self.assertRaises(ValueError):self.receive([self.events[0],self.events[-1]])
    def test_path_traversal_rejected(self):
        e=copy.deepcopy(self.events[0]);e['files']['../escape']={'size':1,'sha256':'0'*64}
        with self.assertRaises(ValueError):archive.ArchiveReceiver(self.base/'received').accept(e)
        self.assertFalse((self.base/'escape').exists())
    def test_modified_remote_not_deleted(self):
        self.receive();(self.run/'new').write_text('late write')
        with self.assertRaises(ValueError):archive.cleanup_acknowledged(self.run,self.base,self.files,True)
    def test_stop_does_not_swallow_archive_ack(self):
        stopped=threading.Event();ack=threading.Event()
        worker.watch_stop(io.StringIO('STOP\nARCHIVE_OK '+self.sha+'\n'),stopped,ack,[self.sha])
        self.assertTrue(stopped.is_set());self.assertTrue(ack.is_set())
    def test_wrong_ack_not_accepted(self):
        ack=threading.Event()
        worker.watch_stop(io.StringIO('ARCHIVE_OK wrong\n'),threading.Event(),ack,[self.sha])
        self.assertFalse(ack.is_set())
    def test_no_implicit_history_read(self):
        with patch.object(Path,'read_text',side_effect=AssertionError("No history access")):
            p=panel.Panel();self.assertIsNone(p.result)
            self.assertEqual(p.snapshot()['events'],[])

    def test_panel_archive_ack_and_result(self):
        from types import SimpleNamespace
        done={"type":"finished","result":{"disable_ack":[104,105]},"archive":{"status":"local_verified_remote_removed"}}
        wire="".join(worker.PREFIX+json.dumps(e)+"\n" for e in [*self.events,done])
        proc=SimpleNamespace(stdout=io.BytesIO(wire.encode()),stdin=io.BytesIO(),wait=lambda:0)
        p=panel.Panel();p.directory=self.base/"panel";p.directory.mkdir();p.phase="running"
        p.read_worker(proc)
        self.assertEqual(proc.stdin.getvalue(),("ARCHIVE_OK "+self.sha+"\n").encode())
        self.assertEqual(p.phase,"finished")
        self.assertTrue((p.directory/"remote_raw/archive_verification.json").exists())


class Deployment(unittest.TestCase):
    def test_allowlist_separates_desktop_and_runtime(self):
        policy=sync_orin.runtime_policy(ROOT)
        for name in ('tools/bench_panel/ph_exo_bench_panel.py','tools/bench_panel/data/runs/old/result.json',
                     'tools/project/ph_exo_bench_panel.py','tools/project/analyze_ref01_ordinary_damping.py'):
            self.assertFalse(sync_orin.runtime_selected(name,policy),name)
        for name in ('tools/project/ph_exo_bench_worker.py','tools/project/bench_archive.py',
                     'ros2_ws/src/mech_controllers/src/ref01_impedance_controller.cpp'):
            self.assertTrue(sync_orin.runtime_selected(name,policy),name)
    def test_receiver_still_detects_removed_desktop_changes(self):
        p='tools/project/ph_exo_bench_panel.html';r={'sha256':'a','size':1,'mode':420}
        self.assertEqual(sync_orin.sync_plan({p:r},{p:r},{},True)['delete'],[p])
        with self.assertRaises(RuntimeError):sync_orin.sync_plan({p:r},{p:dict(r,sha256='b')},{},True)

if __name__=='__main__':unittest.main()
