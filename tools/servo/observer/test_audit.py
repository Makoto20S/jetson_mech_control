#!/usr/bin/env python3
"""Synthetic evidence tests; all files local, no device access."""
from collections import Counter
import copy
import hashlib
import json
from pathlib import Path
import struct
import tempfile
import unittest

import audit

BOOT = '12345678-1234-1234-1234-123456789abc'
FOUR_PACKETS = [bytes.fromhex(value) for value in (
    'f7120e000bf8e2680600000c08000cd528000a000a',
    'f7120e000b0405690600000c08000ddec8000a000a',
    'f7120e000b2085682900000c080349000000002800',
    'f7120e000b7fcc692900000c08038d000000002900')]
SEQUENCE_GOLDEN = bytes.fromhex('f7120e000b55b40101ff1f0c0812345678000000a4')


def envelope(ident, payload, flags=12):
    data = struct.pack('<IBB', ident, flags, len(payload)) + payload
    head = b'\xf7\x12' + len(data).to_bytes(2, 'little')
    return head + bytes([audit.crc(head[1:], 0x8c, 0xff)]) + audit.crc(data, 0x8408, 0xffff).to_bytes(2, 'little') + data


def marker(ident, seq, nonce=0x78563412):
    return envelope(ident, struct.pack('<I', nonce) + seq.to_bytes(3, 'little') + bytes([0xa4 + 0x11 * audit.SEQUENCE_IDS.index(ident)]))


def record(kind, data, tick, result=0, count_only=False):
    requested = len(data) if kind == 1 else 1008
    returned = (len(data) if result == 0 else 0xffffffff) if kind == 1 else len(data)
    retained = b'' if count_only and kind == 2 else data
    meta = audit.META.pack(tick, requested, returned) + retained
    return audit.OUTER.pack(kind, 0, result, 0, len(meta), tick + 1) + meta


def regenerate_manifest(directory):
    files = {name: {'size': (directory / name).stat().st_size,
                    'sha256': hashlib.sha256((directory / name).read_bytes()).hexdigest()}
             for name in audit.ORIGINALS}
    (directory / 'evidence-manifest.json').write_text(json.dumps({'files': files, 'complete_file_set': True, 'boot_id': BOOT}))


def bundle(directory, role, packets, profile='sequence', count=2, count_only=False,
           tx_fault=False, fragments=False, seconds=1, hz=10):
    directory.mkdir()
    ids = audit.SEQUENCE_IDS[:] if profile == 'sequence' else list(audit.FOUR)
    # Single-ID source when only one ID occurs; an empty observer still gets
    # its source's fixture profile independently through the paired plan.
    if profile == 'sequence' and packets and all(int.from_bytes(x[7:11], 'little') == ids[0] for x in packets):
        ids = ids[:1]
    plan = {'schema': 2, 'role': role, 'seconds': seconds, 'drain_ms': 0,
            'capture_limit_bytes': 16 * 1024 * 1024, 'count_only': count_only,
            'raw_rx_retained': not count_only, 'timestamp_domain': 'absolute_host_steady_clock_ns',
            'clock': 'steady_clock_monotonic', 'read_capacity': 1008, 'init_hex': audit.INIT.hex(),
            'run_requested': True, 'motors_disconnected_declared': role == 'fixture',
            'can_wire_timestamp_available': False, 'can_timestamps': False,
            'boot_id': BOOT, 'admission_monotonic_ns': 5,
            'port_by_path': '/dev/serial/by-path/' + role,
            'actual_identity': {'vendor': 'caf1', 'product': 'ffff', 'by_path': '/dev/serial/by-path/' + role,
                                'usb_parent': '/sys/usb/' + role}}
    if role == 'fixture':
        plan.update(profile=profile, ids=ids, frames_per_id=count, nonce=0x78563412,
                    hz_per_id=hz, sequence_observable=profile == 'sequence', simulated_feedback=profile == 'four-id',
                    feedback_provenance='synthetic_fixture' if profile == 'four-id' else 'none')
    raw_parts = [record(1, audit.INIT, 20)]
    tick, rx_counts, rx_bytes, rx_frames, rx_first, tx_frames, read_calls = 40, Counter(), 0, 0, 0, 0, 0
    if role == 'fixture':
        for packet in packets:
            raw_parts.append(record(1, packet, tick, 5 if tx_fault else 0))
            tx_frames += not tx_fault
            tick += 3
    else:
        for packet in packets:
            for frame in audit.decode(packet)[0]:
                rx_counts[frame['id'], frame['extended']] += 1
                rx_frames += 1
        stream = b''.join(packets)
        chunks = [stream[x:x + (1 if fragments else 1008)] for x in range(0, len(stream), 1 if fragments else 1008)]
        for chunk in chunks:
            raw_parts.append(record(2, chunk, tick, count_only=count_only))
            tick += 3
            rx_bytes += len(chunk)
            read_calls += 1
        # Packet emission across chunks: tests avoid >1008-byte arbitrary packets.
        if packets:
            rx_first = 40 + (len(packets[0]) - 1) * 3 + 1 if fragments else 41
    raw = b''.join(raw_parts)
    summary = {'schema': 2, 'role': role, 'complete': not tx_fault,
               'raw_complete': not count_only and not tx_fault, 'count_only': count_only,
               'error': 'write_incomplete_no_retry' if tx_fault else '', 'closed': True,
               'capture_bytes': len(raw), 'start_ns': 10, 'ready_ns': 30, 'end_ns': tick + 100,
               'first_frame_ns': rx_first, 'last_frame_ns': tick - 2 if packets and role == 'observe' else 0,
               'write_calls': 1 + (len(packets) if role == 'fixture' else 0), 'read_calls': read_calls,
               'rx_bytes': rx_bytes, 'rx_frames': rx_frames, 'tx_frames': tx_frames,
               'max_lateness_ns': 0, 'min_send_interval_ns': 0, 'can_delivery_verified': False,
               'ids': [{'id': ident, 'extended': extended, 'frames': n} for (ident, extended), n in rx_counts.items()]}
    # Observer test window covers fixtures with many independent write records.
    if role == 'observe':
        summary['end_ns'] = max(summary['end_ns'], len(packets) * 3 + 200)
    documents = {'plan.json': plan, 'admission.json': plan, 'summary.json': summary,
                 'ready.json': {'ready': True, 'role': role, 'boot_id': BOOT, 'monotonic_ns': 32,
                                'worker_pid': 100 if role == 'fixture' else 99, 'count_only': count_only,
                                'actual_identity': plan['actual_identity']},
                 'exit.json': {'exit_code': 1 if tx_fault else 0, 'reaped': True, 'forced_by_coordinator': False,
                               'started_monotonic_ns': 8, 'exited_monotonic_ns': summary['end_ns'] + 1},
                 'supervisor.json': {'complete': not tx_fault, 'worker_reaped': True, 'ports_close_verified': True,
                                     'forced_timeout': False, 'export_valid': True, 'capture_exported': True,
                                     'worker_pid': 100 if role == 'fixture' else 99}}
    for name, value in documents.items():
        (directory / name).write_text(json.dumps(value))
    (directory / 'capture.bin').write_bytes(raw)
    (directory / 'stdout.txt').write_text(json.dumps(summary))
    (directory / 'stderr.txt').write_text('')
    regenerate_manifest(directory)
    return directory


class AuditTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix='observer-audit-')
        self.root = Path(self.temp.name)

    def tearDown(self):
        self.temp.cleanup()

    def pair(self, tx, rx=None, **kwargs):
        source = bundle(self.root / 'source', 'fixture', tx, **kwargs)
        observe = bundle(self.root / 'observer', 'observe', tx if rx is None else rx,
                         profile=kwargs.get('profile', 'sequence'))
        return audit.audit_directory(observe), audit.audit_directory(source)

    def test_literal_goldens(self):
        self.assertEqual(envelope(0x1fff0101, bytes.fromhex('12345678000000a4')), SEQUENCE_GOLDEN)
        for packet, (ident, payload) in zip(FOUR_PACKETS, audit.FOUR.items()):
            self.assertEqual(envelope(ident, payload), packet)
            self.assertEqual(audit.decode(packet)[0][0]['payload'], payload)
        self.assertEqual(audit.crc(audit.INIT[1:4], 0x8c, 0xff), audit.INIT[4])
        self.assertEqual(audit.crc(audit.INIT[7:], 0x8408, 0xffff), int.from_bytes(audit.INIT[5:7], 'little'))

    def test_fragmentation_prefix_unknown_and_no_tx_after_init(self):
        prefixed = bytes.fromhex('f7121100ff0eb0682900682900000c0803440000fffa2c00')
        directory = bundle(self.root / 'rx', 'observe', [prefixed, envelope(0x123, b'abc', 0)], fragments=True)
        report = audit.audit_directory(directory)
        self.assertTrue(report['valid'], report['errors'])
        self.assertEqual(report['raw_rx_frames'], 2)
        self.assertEqual(report['observed_three_byte_prefix_packets'], 1)
        raw = (directory / 'capture.bin').read_bytes() + record(1, SEQUENCE_GOLDEN, 10000)
        plan, summary = report['_plan'], copy.deepcopy(report['_summary'])
        summary.update(capture_bytes=len(raw), write_calls=2, end_ns=11000)
        self.assertIn('observer_can_write_forbidden', [x['category'] for x in audit.audit_records(raw, plan, summary)['errors']])

    def test_sequence_arbitration_and_equal_duplicate_loss(self):
        tx = [marker(ident, seq) for seq in range(2) for ident in audit.SEQUENCE_IDS]
        rx = [tx[1], tx[0], tx[2], tx[3]]
        observation, source = self.pair(tx, rx)
        self.assertTrue(audit.audit_pair(observation, source)['valid'])
        observation['_frames']['RX'][2] = observation['_frames']['RX'][1]
        result = audit.audit_pair(observation, source)
        self.assertFalse(result['valid'])
        self.assertIn('rx_duplicate_sequence', [x['category'] for x in result['errors']])

    def test_historical_highest_and_crc_valid_wrong_payload(self):
        tx = [marker(audit.SEQUENCE_IDS[0], seq) for seq in range(6)]
        observation, source = self.pair(tx, [tx[x] for x in (0, 1, 2, 5, 3, 4)], count=6)
        result = audit.audit_pair(observation, source)
        out_of_order = [x for x in result['errors'] if x['category'] == 'rx_per_id_out_of_order']
        self.assertEqual(len(out_of_order), 2)
        bad = bytearray(observation['_frames']['RX'][0]['payload']); bad[0] ^= 1
        observation['_frames']['RX'][0]['payload'] = bytes(bad)
        self.assertIn('rx_diagnostic_id_payload', [x['category'] for x in audit.audit_pair(observation, source)['errors']])

    def test_four_id_payload_and_fixed_limit(self):
        observation, source = self.pair(FOUR_PACKETS * 4, profile='four-id', count=4)
        report = audit.audit_pair(observation, source)
        self.assertTrue(report['valid'], report['errors'])
        self.assertFalse(report['sequence_observable'])
        self.assertIn('synthetic', report['limitation'])
        observation['_frames']['RX'][0]['payload'] = audit.FOUR[0x669]
        self.assertFalse(audit.audit_pair(observation, source)['valid'])

    def test_count_only_limited_to_reported_counts(self):
        tx = [marker(ident, seq) for seq in range(2) for ident in audit.SEQUENCE_IDS]
        source = audit.audit_directory(bundle(self.root / 'source', 'fixture', tx))
        observation = audit.audit_directory(bundle(self.root / 'observe', 'observe', tx, count_only=True))
        report = audit.audit_pair(observation, source)
        self.assertTrue(report['valid'], report['errors'])
        self.assertFalse(report['sequence_observable'])
        self.assertIsNone(report['per_id']['0x1fff0101']['raw_observer_received_frames'])
        self.assertEqual(observation['initialization_writes'], 1)

    def test_negative_host_write_success_or_failure_not_delivery(self):
        for fault in (False, True):
            root = self.root / str(fault); root.mkdir()
            tx = [SEQUENCE_GOLDEN]
            source = audit.audit_directory(bundle(root / 'fixture', 'fixture', tx, count=1, tx_fault=fault))
            observation = audit.audit_directory(bundle(root / 'observe', 'observe', []))
            result = audit.audit_pair(observation, source, expected='negative')
            self.assertTrue(result['valid'], result['errors'])
            self.assertFalse(result['can_ack_verified'])
            self.assertFalse(audit.audit_pair(observation, source, expected='negative', prior_nonces=[0x78563412])['valid'])

    def test_crc_shape_and_manifest_exit_failures(self):
        for byte, category in ((4, 'cdc_header_crc'), (5, 'cdc_payload_crc')):
            packet = bytearray(SEQUENCE_GOLDEN); packet[byte] ^= 1
            with self.assertRaises(audit.PacketError) as caught:
                audit.decode(packet)
            self.assertEqual(caught.exception.category, category)
        directory = bundle(self.root / 'good', 'observe', [SEQUENCE_GOLDEN])
        manifest = audit.read_json(directory / 'evidence-manifest.json')
        del manifest['files']['ready.json']
        (directory / 'evidence-manifest.json').write_text(json.dumps(manifest))
        self.assertFalse(audit.audit_directory(directory)['valid'])
        regenerate_manifest(directory)
        exit_record = audit.read_json(directory / 'exit.json'); exit_record['forced_by_coordinator'] = True
        (directory / 'exit.json').write_text(json.dumps(exit_record)); regenerate_manifest(directory)
        self.assertFalse(audit.audit_directory(directory)['valid'])

    def test_shared_boot_readiness_and_unknown_preservation(self):
        tx = [marker(ident, seq) for seq in range(2) for ident in audit.SEQUENCE_IDS]
        observation, source = self.pair(tx, tx + [envelope(0x123, b'abc', 0)])
        result = audit.audit_pair(observation, source)
        self.assertTrue(result['valid'], result['errors'])
        self.assertEqual(result['unrelated_raw_ids'][0]['id'], 0x123)
        observation['boot_id'] = 'different'
        self.assertFalse(audit.audit_pair(observation, source)['valid'])
        observation['boot_id'] = source['boot_id']; observation['_ready']['monotonic_ns'] = 100000
        self.assertFalse(audit.audit_pair(observation, source)['valid'])

    def test_long_capture_linear_scale_and_host_timing(self):
        tx = [marker(ident, seq) for seq in range(30000) for ident in audit.SEQUENCE_IDS]
        observation, source = self.pair(tx, count=30000, hz=500, seconds=60)
        result = audit.audit_pair(observation, source)
        self.assertTrue(result['valid'], result['errors'])
        self.assertEqual(source['successful_can_write_frames'], 60000)
        self.assertEqual(observation['raw_rx_frames'], 60000)
        timing = source['fixture_host_io_timing']['per_id']['0x1fff0101']
        self.assertEqual(timing['begin_interval']['samples'], 29999)
        self.assertEqual(timing['write_call_duration']['max_ns'], 1)


if __name__ == '__main__':
    unittest.main()
