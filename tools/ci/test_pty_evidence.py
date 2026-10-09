"""Exercise clock-domain and retention failures without ROS or hardware."""
import gzip
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest
import socket
import sys


def load(name):
    path = Path(__file__).resolve().parents[1] / 'servo' / (name + '.py')
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class ClockEvidenceTests(unittest.TestCase):
    @unittest.skipUnless(sys.platform == 'linux', 'Linux PTY gateway handshake')
    def test_handshake_acknowledges_cycle_even_when_feedback_is_deliberately_absent(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'peer.sock'
            peer = load('pty_clock_fixture').ClockPeer(path)
            frames = []
            try:
                for publish in (True, False, True):
                    with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as client:
                        client.settimeout(1)
                        client.connect(str(path))
                        client.sendall(b'R')
                        peer.exchange(lambda: frames.append('feedback') if publish else None)
                        self.assertEqual(client.recv(1), b'A')
                    peer.exchange(lambda: self.fail('disconnection is not a cycle'))
                self.assertEqual(frames, ['feedback', 'feedback'])
            finally:
                peer.close()

    def exercise(self, expired=False, pauses=True):
        rows = load('test_chain_audit').ChainAuditTest().fixture()
        for record in rows[1:]:
            record['ns'] += 10**12 + 200000000
            record['deadline'] = 10**9 + (0 if expired else 3000000)
        original_claim = next(r['ns'] for r in rows[1:] if r['stage'] == 'claim_after')
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            trace = root / 'command-chain.jsonl.gz'
            with gzip.open(trace, 'wt') as stream:
                stream.write('\n'.join(json.dumps(row) for row in rows))
            clock = f'host_ns,hardware_ns,pause_ms\n{10**12},1000000000,0\n'
            if pauses:
                elapsed = 0
                for ms in (20, 50, 100):
                    elapsed += ms * 1000000
                    clock += f'{10**12+elapsed},1000000000,{ms}\n'
            (root / 'single-123.csv').write_text(clock)
            report = load('pty_clock_fixture').audit_trace(trace, root)
            self.assertEqual(report['acquisitions'][0]['ns'], original_claim)
            # The audit must not rewrite forensic raw captures.
            with gzip.open(trace, 'rt') as stream:
                self.assertEqual([json.loads(line) for line in stream], rows)
            return report

    def test_host_pause_does_not_expire_logical_command_or_break_operator_time(self):
        self.assertTrue(self.exercise()['complete'])

    def test_expired_logical_command_still_fails(self):
        report = self.exercise(expired=True)
        self.assertFalse(report['complete'])
        self.assertTrue(any('expired' in issue for issue in report['issues']))

    def test_missing_pause_evidence_fails_closed(self):
        with self.assertRaisesRegex(AssertionError, 'host pauses missing'):
            self.exercise(pauses=False)


class ArtifactTests(unittest.TestCase):
    def test_failure_snapshots_compressed_unknown_files_excluded_and_limits_reported(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            source = root / 'source'; source.mkdir()
            (source / 'capture.snapshot').write_bytes(b'captured' * 10000)
            (source / 'secret.env').write_text('must not be copied')
            (source / 'framework.log').write_bytes(b'x' * 300000)
            retain = load('test_artifacts').retain
            report = retain(source, root / 'full')
            self.assertTrue(report['complete'])
            self.assertFalse((root / 'full/secret.env').exists())
            with gzip.open(root / 'full/capture.snapshot.gz', 'rb') as stream:
                self.assertEqual(stream.read(), (source / 'capture.snapshot').read_bytes())
            report = retain(source, root / 'bounded', limit=131072)
            self.assertFalse(report['complete'])
            self.assertLessEqual(sum(row['retained_bytes'] for row in report['files']), 131072)


if __name__ == '__main__':
    unittest.main()
