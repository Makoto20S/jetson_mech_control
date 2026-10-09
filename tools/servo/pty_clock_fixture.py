"""Process-test driver and dual-clock audit; never used by the operator CLI.

Raw command traces retain host timestamps, while the test plugin uses logical
hardware deadlines. Keep both files intact and convert a copy for the ordinary
hardware-chain auditor. Operator/service correlation still uses host time.
"""
import bisect
import collections
import csv
import gzip
import importlib.util
import json
import os
import select
import socket
from pathlib import Path
import sys
import xml.etree.ElementTree as ET


class ClockPeer:
    """Gateway-thread handshake: queue feedback before allowing hardware read.

    The reply acknowledges a simulation cycle, not motor success. A loss test
    still replies but deliberately queues no feedback, so real expiry runs.
    """
    def __init__(self, path):
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        self.server = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        self.server.bind(str(path))
        self.server.listen(1)
        self.connection = None

    def exchange(self, feedback):
        if self.connection is None:
            if not select.select([self.server], [], [], 0)[0]:
                return
            self.connection, _ = self.server.accept()
        if select.select([self.connection], [], [], 0)[0]:
            request = self.connection.recv(1)
            if not request:
                self.connection.close(); self.connection = None
                return
            assert request == b'R', 'invalid clock handshake'
            feedback()
            self.connection.sendall(b'A')

    def close(self):
        if self.connection is not None:
            self.connection.close()
        self.server.close()


def load(name):
    spec = importlib.util.spec_from_file_location(name, Path(__file__).with_name(name + '.py'))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def audit_trace(trace, clocks, require_pauses=True):
    trace, clocks = Path(trace), Path(clocks)
    with gzip.open(trace, 'rt') as stream:
        records = [json.loads(line) for line in stream]
    assert len(records) > 1, 'missing command trace records'
    bus = trace.name.removeprefix('command-chain').removesuffix('.jsonl.gz').lstrip('-') or 'single'
    candidates = []
    for path in clocks.glob(bus + '-*.csv'):
        with path.open() as stream:
            rows = [{k: int(v) for k, v in row.items()} for row in csv.DictReader(stream)]
        assert rows, f'empty clock evidence: {path}'
        if rows[0]['host_ns'] <= records[1]['ns']:
            candidates.append((rows[0]['host_ns'], rows))
    assert candidates, f'no matching {bus} test clock evidence'
    rows = max(candidates, key=lambda item: item[0])[1]
    host = [r['host_ns'] for r in rows]
    hardware = [r['hardware_ns'] for r in rows]
    assert all(b > a for a, b in zip(host, host[1:])), 'host clock ordering'
    assert all(0 <= b - a <= 2000000 for a, b in zip(hardware, hardware[1:])), 'hardware clock ordering'
    pauses = [r['pause_ms'] for r in rows if r['pause_ms']]
    assert all(row['host_ns'] - before['host_ns'] >= row['pause_ms'] * 1000000
               for before, row in zip(rows, rows[1:])), 'host pause duration evidence'
    if require_pauses:
        assert pauses == [20, 50, 100], f'host pauses missing: {pauses}'
    claims = collections.defaultdict(collections.deque)
    assert all(b['ns'] >= a['ns'] for a, b in zip(records[1:], records[2:])), 'raw host trace ordering'
    for record in records[1:]:
        original = record['ns']
        index = bisect.bisect_right(host, original) - 1
        assert index >= 0, 'trace predates clock evidence'
        record['ns'] = hardware[index]
        if record['stage'] == 'claim_after' and record['flags'] & 1:
            claims[(record['joint'], record['ns'])].append(original)
    records[0]['clock'] = 'test_logical_hardware_ns'
    report = load('chain_audit').audit_records(records)
    for claim in report['acquisitions']:
        claim['ns'] = claims[(claim['joint'], claim['ns'])].popleft()
    report['clock_domains'] = 'logical hardware deadlines; host operator timestamps'
    report['injected_host_pauses_ms'] = pauses
    return report


def main():
    motion = load('motion')
    original = motion.config_tools.generate_urdf
    clock_dir = Path(sys.argv[sys.argv.index('--run-dir') + 1]) / 'clocks'
    clock_dir.mkdir(parents=True, exist_ok=True)
    os.environ['MECH_TEST_CLOCK_DIR'] = str(clock_dir)

    def generate(*args, **kwargs):
        root = ET.fromstring(original(*args, **kwargs))
        for hw in root.findall('./ros2_control/hardware'):
            path = hw.find("param[@name='device_path']").text
            assert path.startswith('/dev/pts/') and path[9:].isdigit(), 'synthetic PTYs only'
            plugin = hw.find('plugin')
            assert plugin.text == 'mech_bringup/Ak30ServoSystem'
            plugin.text = 'mech_bringup/ServoPtyTestSystem'
        return ET.tostring(root, encoding='unicode')

    motion.config_tools.generate_urdf = generate
    original_disable = motion.disable_motors

    def disable(helper, urdf, run, record, ids):
        # The separate production helper deliberately accepts only production
        # hardware descriptions. Restore that name in its own diagnostic copy.
        device_urdf = run / 'disable.urdf'
        device_urdf.write_text(urdf.read_text().replace(
            'mech_bringup/ServoPtyTestSystem', 'mech_bringup/Ak30ServoSystem'))
        return original_disable(helper, device_urdf, run, record, ids)

    motion.disable_motors = disable
    return motion.main()


if __name__ == '__main__':
    raise SystemExit(main())
