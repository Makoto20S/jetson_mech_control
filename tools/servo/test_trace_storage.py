"""Offline bounded storage/schema tests; no ROS import and no device I/O."""
import sys
if __name__ == '__main__':
    sys.path.pop(0)
import gzip
import importlib.util
import io
import json
from pathlib import Path
import struct
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parent
spec = importlib.util.spec_from_file_location('trace_storage', ROOT/'trace_storage.py')
storage = importlib.util.module_from_spec(spec); spec.loader.exec_module(storage)
spec = importlib.util.spec_from_file_location('test_chain_audit', ROOT/'test_chain_audit.py')
fixtures = importlib.util.module_from_spec(spec); spec.loader.exec_module(fixtures)


def snapshot(path, rows, kind=1, capacity=None, total=None, closed=1):
    capacity = capacity or max(1,len(rows))
    total = len(rows) if total is None else total
    stride = 216 if kind == 1 else 1064
    header = bytearray(128)
    header[:8] = b'MCHTRC01'
    struct.pack_into('<4I6Q',header,8,1,kind,stride,128,capacity,total,min(total,capacity),
                     max(total-capacity,0) if kind == 1 else 0,
                     max(total-capacity,0) if kind == 2 else 0,closed)
    with Path(path).open('wb') as stream:
        stream.write(header)
        for index in range(capacity):
            data=bytearray(stride)
            if index < len(rows):
                row=rows[index]
                if kind == 1:
                    stage=row['stage'].encode('ascii'); data[:len(stage)]=stage
                    struct.pack_into('<4q7Q2diiI',data,32,
                        row['ns'],row['begin_ns'],row['deadline'],row['result'],
                        row['cycle'],row['generation'],row['io'],row['joint'],row['requested'],
                        row['length'],row['captured'],row['position'],row['feedback'],
                        row['flags'],row['errno'],row['id'])
                    raw=bytes.fromhex(row['hex']); data[148:148+len(raw)]=raw
                else:
                    struct.pack_into('<qqQQBB',data,0,index,index+1,1,1,0,1)
                    data[34]=index
            stream.write(data)


class StorageTests(unittest.TestCase):
    def test_capacity_settings_are_bounded_before_any_allocation(self):
        self.assertLess(storage.capture_bytes(storage.CHAIN_CAPACITY,storage.RAW_CAPACITY),storage.SNAPSHOT_LIMIT)
        for capacities in ((0,1),(1,-1),(2**32,1)):
            with self.assertRaises(ValueError):storage.capture_bytes(*capacities)
    @unittest.skipUnless(hasattr(storage.os,'getuid') and Path('/dev/shm').is_dir(), 'Linux tmpfs required')
    def test_orphan_blocks_next_capture_in_same_run_root_without_deletion(self):
        with tempfile.TemporaryDirectory() as directory:
            session=Path(directory)/'move-test'; session.mkdir()
            source=storage.make_snapshots(session,1,1,session)
            try:
                self.assertTrue((source/'owner.json').exists())
                with self.assertRaisesRegex(ValueError,'未恢复'):
                    storage.make_snapshots(session/'backend-002',1,1,session)
                self.assertTrue((source/'owner.json').exists())
            finally:
                storage.abandon_prelaunch(source,session)

    def test_recovery_rejects_live_writer_group(self):
        with patch.object(storage.os,'killpg',create=True):
            with self.assertRaisesRegex(ValueError,'still present'):
                storage.require_writer_exited(dict(writer_pid=123))
        with patch.object(storage.os,'killpg',side_effect=ProcessLookupError(),create=True):
            storage.require_writer_exited(dict(writer_pid=123))

    def test_schema_export_matches_independent_audit_fixture(self):
        rows=fixtures.ChainAuditTest().fixture()
        with tempfile.TemporaryDirectory() as directory:
            base=Path(directory); snapshot(base/'chain.snapshot',rows[1:])
            metadata=storage.export_snapshot(base/'chain.snapshot',base/'chain.jsonl.gz',1024*1024,1)
            with gzip.open(base/'chain.jsonl.gz','rt') as stream:
                decoded=[json.loads(line) for line in stream]
            self.assertEqual(decoded[1:],rows[1:])
            self.assertTrue(metadata['closed'])
            audit=fixtures.ChainAuditTest().load()
            self.assertTrue(audit.audit_records(decoded)['complete'])

    def test_gzip_cli_parity_and_truncation_rejection(self):
        rows=fixtures.ChainAuditTest().fixture()
        with tempfile.TemporaryDirectory() as directory:
            base=Path(directory); plain=base/'chain.jsonl'; zipped=base/'chain.jsonl.gz'
            plain.write_text(''.join(json.dumps(row)+'\n' for row in rows))
            with gzip.open(zipped,'wt') as stream: stream.write(plain.read_text())
            reports=[]
            for path in (plain,zipped):
                result=subprocess.run([sys.executable,str(ROOT/'chain_audit.py'),str(path),'--hardware-only'],
                                      capture_output=True,text=True)
                self.assertEqual(result.returncode,0,result.stderr)
                reports.append(json.loads(result.stdout))
            self.assertEqual(*reports)
            zipped.write_bytes(zipped.read_bytes()[:-8])
            result=subprocess.run([sys.executable,str(ROOT/'chain_audit.py'),str(zipped),'--hardware-only'],
                                  capture_output=True,text=True)
            self.assertEqual(result.returncode,1)
            self.assertFalse(json.loads(result.stdout)['complete'])

    def test_unclosed_and_overflow_trace_cannot_pass(self):
        rows=fixtures.ChainAuditTest().fixture(); audit=fixtures.ChainAuditTest().load()
        rows[0]['closed']=False
        self.assertFalse(audit.audit_records(rows)['complete'])

    def test_raw_unclosed_full_ring_omits_possibly_torn_oldest_slot(self):
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'serial.snapshot'; snapshot(path,[{},{}],2,total=3,closed=0)
            meta=storage.snapshot_metadata(path,2)
            rows=list(storage.snapshot_rows(path,meta))
            self.assertEqual([r['sequence'] for r in rows],[2])

    def test_compressed_cap_is_hard_and_prior_files_preserved(self):
        rows=fixtures.ChainAuditTest().fixture()
        with tempfile.TemporaryDirectory() as directory:
            base=Path(directory); path=base/'chain.snapshot'; snapshot(path,rows[1:])
            prior=base/'old.log'; prior.write_bytes(b'evidence')
            with self.assertRaises(OSError):storage.export_snapshot(path,base/'limited.gz',32,1)
            self.assertLessEqual((base/'limited.gz').stat().st_size,32)
            self.assertEqual(prior.read_bytes(),b'evidence')
            with self.assertRaises(FileExistsError):storage.export_snapshot(path,prior,1000,1)
            self.assertEqual(prior.read_bytes(),b'evidence')

    def test_shared_session_and_run_root_admission(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory); run=root/'move-test'; child=run/'backend-002'; child.mkdir(parents=True)
            (child/'prior.log').write_bytes(b'12345678')
            (root/'old.log').write_bytes(b'12345678')
            with patch.object(storage,'SESSION_LIMIT',32), patch.object(storage,'ROOT_LIMIT',64), \
                    patch.object(storage,'MIB',1), patch.object(storage,'FREE_RESERVE',10), \
                    patch.object(storage.shutil,'disk_usage',return_value=type('Usage',(),{'free':100})()):
                self.assertEqual(storage.admit(run,20),20)
                with patch.object(storage,'ROOT_LIMIT',20):
                    with self.assertRaisesRegex(ValueError,'2 GiB'):storage.admit(run,20)
                with patch.object(storage.shutil,'disk_usage',return_value=type('Usage',(),{'free':29})()):
                    with self.assertRaisesRegex(ValueError,'1 GiB'):storage.admit(run,20)
            self.assertEqual((root/'old.log').read_bytes(),b'12345678')

    def test_events_are_capped_and_loss_is_explicit(self):
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'events.jsonl'; writer=storage.BoundedEvents(path,8)
            writer.write('1234\n'); writer.write('5678\n'); writer.close()
            self.assertEqual(path.read_bytes(),b'1234\n'); self.assertEqual(writer.dropped,1)

    def test_framework_keeps_draining_after_cap_and_open_failure(self):
        class Pipe(io.BytesIO):
            def close(self): self.was_closed=True
        with tempfile.TemporaryDirectory() as directory:
            pipe=Pipe(b'x'*200000); writer=storage.FrameworkDrain(pipe,Path(directory)/'framework.log',32)
            writer.close(); self.assertEqual(pipe.tell(),200000)
            self.assertEqual(writer.size,32); self.assertEqual(writer.discarded,199968)
            pipe=Pipe(b'x'*200000)
            writer=storage.FrameworkDrain(pipe,Path(directory)/'missing'/'framework.log',32)
            writer.close(); self.assertEqual(pipe.tell(),200000)
            self.assertEqual(writer.discarded,200000); self.assertTrue(writer.error)

    def test_export_failure_retains_owned_snapshot_and_records_outcome(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory); run=root/'run'; run.mkdir(); source=root/'snapshot'; source.mkdir()
            (source/'owner.json').write_text(json.dumps(dict(uid=1000,run=str(run.resolve()))))
            with patch.object(storage.os,'getuid',return_value=1000,create=True):
                result=storage.finalize(source,run,run,confirmed_exit=True)
            self.assertTrue(result['snapshot_retained']); self.assertTrue(source.exists())
            self.assertEqual(json.loads((run/'capture-manifest.json').read_text())['diagnostic_status'],'incomplete')

    def test_successful_export_removes_only_verified_owned_snapshot(self):
        rows=fixtures.ChainAuditTest().fixture()
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory); run=root/'run'; run.mkdir(); source=root/'snapshot'; source.mkdir()
            prior=root/'old.log'; prior.write_bytes(b'evidence')
            (source/'owner.json').write_text(json.dumps(dict(uid=1000,run=str(run.resolve()))))
            snapshot(source/'chain.snapshot',rows[1:]); snapshot(source/'serial.snapshot',[{}],2)
            with patch.object(storage.os,'getuid',return_value=1000,create=True), \
                    patch.object(storage.shutil,'disk_usage',return_value=type('Usage',(),{'free':2**40})()):
                result=storage.finalize(source,run,run,confirmed_exit=True)
            self.assertFalse(result['snapshot_retained']); self.assertFalse(source.exists())
            self.assertEqual(result['diagnostic_status'],'complete'); self.assertEqual(prior.read_bytes(),b'evidence')

    def test_recovery_keeps_old_partial_output_and_uses_successor(self):
        rows=fixtures.ChainAuditTest().fixture()
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory); run=root/'run'; run.mkdir(); source=root/'snapshot'; source.mkdir()
            (source/'owner.json').write_text(json.dumps(dict(uid=1000,run=str(run.resolve()),writer_pid=123)))
            partial=run/'command-chain.jsonl.gz'; partial.write_bytes(b'failed evidence')
            snapshot(source/'chain.snapshot',rows[1:]); snapshot(source/'serial.snapshot',[{}],2)
            with patch.object(storage.os,'getuid',return_value=1000,create=True), \
                    patch.object(storage.os,'killpg',side_effect=ProcessLookupError(),create=True), \
                    patch.object(storage.shutil,'disk_usage',return_value=type('Usage',(),{'free':2**40})()):
                result=storage.finalize(source,run,run)
            self.assertEqual(partial.read_bytes(),b'failed evidence')
            name=result['snapshots']['chain.snapshot']['output_file']
            self.assertIn('.recovered-',name); self.assertTrue((run/name).exists())

    def test_bad_snapshot_size_and_header_are_rejected(self):
        rows=fixtures.ChainAuditTest().fixture()
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'snapshot'; snapshot(path,rows[1:])
            path.write_bytes(path.read_bytes()[:-1])
            with self.assertRaises(ValueError):storage.snapshot_metadata(path,1)


if __name__ == '__main__':
    unittest.main()
