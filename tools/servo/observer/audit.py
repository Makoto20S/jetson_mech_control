#!/usr/bin/env python3
"""Independent file-only schema-2 audit. Never opens a TTY or imports a codec.

Manifest hashes must originate at the exporting host; hashes alone are not
authentication. Host write completion is not a CAN ACK or a wire timestamp.
"""
import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path
import re
import struct
import math

OUTER = struct.Struct('<BBBBIQ')
META = struct.Struct('<QII')
INIT = bytes.fromhex('f71206007d7008000000000000')
ORIGINALS = {'admission.json', 'capture.bin', 'exit.json', 'plan.json',
             'ready.json', 'stderr.txt', 'stdout.txt', 'summary.json', 'supervisor.json'}
FOUR = {0x668: bytes.fromhex('000cd528000a000a'),
        0x669: bytes.fromhex('000ddec8000a000a'),
        0x2968: bytes.fromhex('0349000000002800'),
        0x2969: bytes.fromhex('038d000000002900')}
SEQUENCE_IDS = [0x1fff0101, 0x1fff0102]
FIXED_LIMITATION = ('Four-ID feedback is synthetic fixture data. Identical payloads '
                    'cannot identify individual frames, reordering, or equal loss '
                    'and duplication that cancel in the counts.')


def describe(values):
    ordered = sorted(values)
    return {'samples': len(ordered), 'min_ns': ordered[0] if ordered else None,
            'max_ns': ordered[-1] if ordered else None,
            'p99_nearest_rank_ns': ordered[math.ceil(len(ordered) * .99) - 1] if ordered else None}


def fixture_timing(frames):
    """Describe recorded host calls, not the earlier scheduler checkpoints."""
    by_id = {}
    for ident in sorted({x['id'] for x in frames}):
        writes = [x for x in frames if x['id'] == ident and x['result'] == 0]
        intervals = [b['begin_ns'] - a['begin_ns'] for a, b in zip(writes, writes[1:])]
        by_id[f'{ident:#x}'] = {'begin_interval': describe(intervals),
                               'write_call_duration': describe([x['end_ns'] - x['begin_ns'] for x in writes])}
    return {'timestamp_domain': 'absolute_host_steady_clock_ns', 'per_id': by_id,
            'limitation': 'Host IO call timings only; scheduler checkpoints and physical CAN timings are different observations.'}


def crc(data, polynomial, seed):
    """Bit-at-a-time reflected CRC, independent of vendor lookup tables."""
    value = seed
    for byte in data:
        value ^= byte
        for _ in range(8):
            value = (value >> 1) ^ (polynomial if value & 1 else 0)
    return value


class PacketError(ValueError):
    def __init__(self, category, byte, detail=''):
        super().__init__(category)
        self.category, self.byte, self.detail = category, byte, detail


def decode(packet):
    if packet[:2] != b'\xf7\x12':
        raise PacketError('cdc_envelope', 0)
    if len(packet) < 7:
        raise PacketError('cdc_truncated', len(packet))
    length = int.from_bytes(packet[2:4], 'little')
    if not 6 <= length <= 512 or len(packet) != length + 7:
        raise PacketError('cdc_length', 2, str(length))
    if crc(packet[1:4], 0x8c, 0xff) != packet[4]:
        raise PacketError('cdc_header_crc', 4)
    if crc(packet[7:], 0x8408, 0xffff) != int.from_bytes(packet[5:7], 'little'):
        raise PacketError('cdc_payload_crc', 5)
    cursor, prefixed = 7, False
    if length >= 9:
        low = int.from_bytes(packet[7:10], 'little')
        ident = int.from_bytes(packet[10:14], 'little')
        flags, size = packet[14:16]
        if (low == ident & 0xffffff and flags and size and
                size <= (64 if flags & 2 else 8) and 9 + size <= length):
            cursor, prefixed = 10, True
    frames = []
    while cursor < len(packet):
        if len(packet) - cursor < 6 or len(frames) >= 64:
            raise PacketError('cdc_record_shape', cursor)
        ident = int.from_bytes(packet[cursor:cursor + 4], 'little')
        flags, size = packet[cursor + 4:cursor + 6]
        sizes = (*range(9), 12, 16, 20, 24, 32, 48, 64) if flags & 2 else range(9)
        if (flags & 0xf0 or (flags & 1 and not flags & 2) or
                ident > (0x1fffffff if flags & 4 else 0x7ff) or
                size not in sizes or cursor + 6 + size > len(packet)):
            raise PacketError('cdc_record_shape', cursor, f'id={ident:#x},flags={flags:#x},dlc={size}')
        frames.append({'id': ident, 'extended': bool(flags & 4), 'flags': flags,
                       'payload': packet[cursor + 6:cursor + 6 + size], 'byte': cursor})
        cursor += 6 + size
    return frames, prefixed


class Findings:
    def __init__(self):
        self.errors = []

    def check(self, condition, category, record=None, byte=0, detail=''):
        if condition:
            return True
        finding = {'category': category, 'detail': detail}
        if record is not None:
            finding.update(record_index=record['index'], direction=record['direction'],
                           begin_ns=record['begin'], end_ns=record['end'],
                           capture_byte_offset=record['offset'] + 32 + byte)
        # Bound incident output without hiding whether further errors occurred.
        if len(self.errors) < 20:
            self.errors.append(finding)
        return False


def audit_records(raw, plan, summary):
    f = Findings()
    frames = {'TX': [], 'RX': []}
    calls, outcomes = Counter(), Counter()
    io, init, rx_bytes, cursor, previous, index, prefixes = [], [], 0, 0, 0, 0, 0
    pending = bytearray()
    origins = []  # One location per retained RX byte; bounded by one CDC packet.
    poisoned = False
    count_only = plan['count_only']
    while cursor < len(raw):
        if not f.check(len(raw) - cursor >= 32, 'capture_header_truncated', detail=str(cursor)):
            break
        kind, port, result, reserved, size, end = OUTER.unpack_from(raw, cursor)
        begin, requested, returned = META.unpack_from(raw, cursor + 16)
        record = {'index': index, 'direction': 'TX' if kind == 1 else 'RX',
                  'offset': cursor, 'begin': begin, 'end': end,
                  'result': result, 'requested': requested, 'returned': returned}
        if not f.check(kind in (1, 2) and port == 0 and reserved == 0 and
                       result <= 5 and 16 <= size <= 1024 and previous <= begin <= end,
                       'capture_header_or_time', record):
            break
        if not f.check(cursor + 16 + size <= len(raw), 'capture_payload_truncated', record):
            break
        data = raw[cursor + 32:cursor + 16 + size]
        calls[record['direction']] += 1
        outcomes[f"{record['direction']}_{result}"] += 1
        io.append(record)
        if kind == 1:
            f.check(requested == len(data) and 0 < requested <= 1008 and
                    returned == (requested if result == 0 else 0xffffffff), 'tx_io_size', record)
            if index == 0:
                f.check(data == INIT, 'first_write_not_initialization', record)
            if data == INIT:
                init.append(record)
                f.check(index == 0, 'initialization_not_first', record)
            elif plan['role'] == 'observe':
                f.check(False, 'observer_can_write_forbidden', record)
            else:
                try:
                    decoded, prefix = decode(data)
                    f.check(not prefix and len(decoded) == 1, 'fixture_not_separate', record)
                    for frame in decoded:
                        f.check(frame['flags'] == 0x0c, 'fixture_flags', record, frame['byte'])
                        frame.update(begin_ns=begin, end_ns=end, result=result,
                                     capture_byte_offset=cursor + 32 + frame['byte'])
                        frames['TX'].append(frame)
                except PacketError as error:
                    f.check(False, error.category, record, error.byte, error.detail)
        else:
            f.check(requested == 1008 and returned <= requested, 'rx_io_size', record)
            f.check(result != 1 or returned == 0, 'wouldblock_with_bytes', record)
            f.check(len(data) == (0 if count_only else returned), 'rx_retained_size', record)
            rx_bytes += returned
            if data and result == 0 and not poisoned:
                pending.extend(data)
                origins.extend((record, byte) for byte in range(len(data)))
                while len(pending) >= 7:
                    length = int.from_bytes(pending[2:4], 'little')
                    if not 6 <= length <= 512 or pending[:2] != b'\xf7\x12' or crc(pending[1:4], 0x8c, 0xff) != pending[4]:
                        error_byte = 0 if pending[:2] != b'\xf7\x12' else (2 if not 6 <= length <= 512 else 4)
                        origin, within = origins[error_byte]
                        f.check(False, 'rx_stream_header', origin, within)
                        poisoned = True
                        break
                    if len(pending) < length + 7:
                        break
                    try:
                        decoded, prefix = decode(bytes(pending[:length + 7]))
                        prefixes += int(prefix)
                        for frame in decoded:
                            origin, within = origins[frame['byte']]
                            frame.update(begin_ns=origin['begin'], end_ns=end, result=result,
                                         capture_byte_offset=origin['offset'] + 32 + within)
                            frames['RX'].append(frame)
                    except PacketError as error:
                        origin, within = origins[error.byte]
                        f.check(False, error.category, origin, within, error.detail)
                        poisoned = True
                        break
                    del pending[:length + 7]
                    del origins[:length + 7]
        previous, cursor, index = end, cursor + 16 + size, index + 1
    if pending and not poisoned:
        record, within = origins[0]
        f.check(False, 'rx_stream_truncated', record, within, str(len(pending)))
    f.check(len(init) == 1, 'initialization_count', detail=str(len(init)))
    f.check(not init or init[0]['result'] == 0 or len(io) == 1, 'io_after_failed_initialization')
    # No later IO may occur after an unsuccessful write/read (WouldBlock is normal).
    for position, record in enumerate(io):
        if record['result'] != 0 and not (record['direction'] == 'RX' and record['result'] == 1):
            f.check(position == len(io) - 1, 'io_after_transport_failure', record)
    f.check(summary['capture_bytes'] == len(raw) <= plan['capture_limit_bytes'], 'capture_size_summary')
    f.check(summary['write_calls'] == calls['TX'] and summary['read_calls'] == calls['RX'], 'io_calls_summary')
    f.check(summary['rx_bytes'] == rx_bytes, 'rx_bytes_summary')
    successful_tx = [x for x in frames['TX'] if x['result'] == 0]
    f.check(summary['tx_frames'] == len(successful_tx), 'tx_frames_summary')
    summary_ids = {(x['id'], x['extended']): x['frames'] for x in summary['ids']}
    f.check(len(summary_ids) == len(summary['ids']) and all(0 < n for n in summary_ids.values()) and
            sum(summary_ids.values()) == summary['rx_frames'], 'id_count_summary')
    if not count_only:
        actual_ids = Counter((x['id'], x['extended']) for x in frames['RX'])
        f.check(summary_ids == actual_ids and summary['rx_frames'] == len(frames['RX']), 'raw_rx_summary')
        first = frames['RX'][0]['end_ns'] if frames['RX'] else 0
        last = frames['RX'][-1]['end_ns'] if frames['RX'] else 0
        f.check((summary['first_frame_ns'], summary['last_frame_ns']) == (first, last), 'rx_frame_time_summary')
    f.check(summary['start_ns'] <= summary['ready_ns'] <= summary['end_ns'], 'summary_time_order')
    if io:
        f.check(summary['start_ns'] <= io[0]['begin'] and io[-1]['end'] <= summary['end_ns'], 'io_summary_time_window')
        if init and init[0]['result'] == 0:
            f.check(init[0]['end'] <= summary['ready_ns'], 'ready_before_init_complete')
    if plan['role'] == 'fixture':
        f.check(not count_only and summary['rx_frames'] == 0, 'fixture_rx_not_empty')
        expected_ids = SEQUENCE_IDS[:len(plan['ids'])] if plan['profile'] == 'sequence' else list(FOUR)
        f.check(plan['ids'] == expected_ids and len(plan['ids']) in (1, 2, 4), 'fixture_plan_ids')
        per_id = Counter()
        for frame in frames['TX']:
            ident, payload = frame['id'], frame['payload']
            lane = expected_ids.index(ident) if ident in expected_ids else None
            expected = FOUR.get(ident) if plan['profile'] == 'four-id' else (
                plan['nonce'].to_bytes(4, 'little') + per_id[ident].to_bytes(3, 'little') +
                bytes([0xa4 + 0x11 * lane]) if lane is not None else None)
            f.check(frame['extended'] and payload == expected, 'fixture_id_payload', detail=f'{ident:#x}@{frame["capture_byte_offset"]}')
            if frame['result'] == 0:
                per_id[ident] += 1
        if summary['complete']:
            f.check(all(per_id[x] == plan['frames_per_id'] for x in expected_ids) and
                    sum(per_id.values()) == plan['frames_per_id'] * len(expected_ids), 'fixture_planned_count_not_completed')
        if plan['profile'] == 'four-id':
            f.check(plan['simulated_feedback'] is True and plan['feedback_provenance'] == 'synthetic_fixture' and
                    plan['sequence_observable'] is False, 'four_id_provenance')
        else:
            f.check(plan['sequence_observable'] is True and plan['simulated_feedback'] is False, 'sequence_provenance')
    return {'valid': not f.errors, 'errors': f.errors, 'first_failure': f.errors[0] if f.errors else None,
            'record_count': index, 'io_outcomes': dict(outcomes), 'initialization_writes': len(init),
            'successful_can_write_frames': len(successful_tx), 'raw_rx_frames': None if count_only else len(frames['RX']),
            'rx_io_returned_bytes': rx_bytes, 'observed_three_byte_prefix_packets': prefixes,
            'raw_rx_available': not count_only, 'can_delivery_verified': False,
            'fixture_host_io_timing': fixture_timing(frames['TX']) if plan['role'] == 'fixture' else None,
            '_frames': frames, '_io': io}


def read_json(path):
    def unique(pairs):
        result = {}
        for key, value in pairs:
            if key in result:
                raise ValueError(f'duplicate JSON key: {key}')
            result[key] = value
        return result
    return json.loads(path.read_text(encoding='utf-8'), object_pairs_hook=unique)


def audit_directory(directory, manifest=None):
    directory = Path(directory)
    documents = {name: read_json(directory / name) for name in ORIGINALS if name.endswith('.json')}
    plan, summary = documents['plan.json'], documents['summary.json']
    ready, supervisor = documents['ready.json'], documents['supervisor.json']
    f = Findings()
    hashes = {}
    for name in sorted(ORIGINALS):
        path = directory / name
        f.check(path.is_file() and not path.is_symlink(), 'evidence_not_regular', detail=name)
        content = path.read_bytes()
        hashes[name] = {'sha256': hashlib.sha256(content).hexdigest(), 'size': len(content)}
    manifest_path = Path(manifest) if manifest else directory / 'evidence-manifest.json'
    manifest_document = read_json(manifest_path)
    remote = manifest_document['files']
    f.check(set(remote) == ORIGINALS, 'manifest_exact_file_set')
    f.check(manifest_document['complete_file_set'] is True and manifest_document['boot_id'] == plan['boot_id'], 'manifest_complete_or_boot')
    for name in ORIGINALS:
        f.check(remote.get(name) == hashes[name], 'remote_hash_or_size_mismatch', detail=name)
    f.check(documents['admission.json'] == plan, 'admission_plan_mismatch')
    f.check(plan['schema'] == 2 and summary['schema'] == 2 and plan['role'] in ('observe', 'fixture') and
            plan['role'] == summary['role'] == ready['role'], 'schema_role')
    f.check(all(type(summary[x]) is int and summary[x] >= 0 for x in (
        'capture_bytes', 'start_ns', 'ready_ns', 'end_ns', 'first_frame_ns', 'last_frame_ns',
        'read_calls', 'write_calls', 'rx_bytes', 'rx_frames', 'tx_frames', 'max_lateness_ns', 'min_send_interval_ns')) and
        all(type(summary[x]) is bool for x in ('complete', 'raw_complete', 'closed', 'count_only')) and
        all(type(plan[x]) is bool for x in ('count_only', 'raw_rx_retained', 'run_requested')) and
        all(type(x['id']) is int and type(x['extended']) is bool and type(x['frames']) is int for x in summary['ids']),
        'summary_numeric_or_boolean_types')
    f.check(type(plan['seconds']) is int and 1 <= plan['seconds'] <= 180 and
            type(plan['drain_ms']) is int and 0 <= plan['drain_ms'] <= 5000 and
            type(plan['capture_limit_bytes']) is int and plan['capture_limit_bytes'] % 1048576 == 0 and
            1048576 <= plan['capture_limit_bytes'] <= 64 * 1048576, 'plan_limits')
    if plan['role'] == 'fixture':
        f.check(plan['profile'] in ('sequence', 'four-id') and not plan['count_only'] and
                type(plan['hz_per_id']) is int and 1 <= plan['hz_per_id'] <= 500 and
                type(plan['frames_per_id']) is int and 1 <= plan['frames_per_id'] <= plan['hz_per_id'] * plan['seconds'] and
                type(plan['nonce']) is int and 0 <= plan['nonce'] <= 0xffffffff and
                plan['motors_disconnected_declared'] is True, 'fixture_plan_limits_or_isolation')
        if plan['profile'] == 'four-id':
            f.check(plan['hz_per_id'] <= 10 and 4 <= plan['frames_per_id'] <= 10, 'four_id_plan_limits')
    else:
        f.check(plan['drain_ms'] == 0, 'observe_drain_not_zero')
    f.check(plan['count_only'] == summary['count_only'] == ready['count_only'] and
            plan['raw_rx_retained'] == (not plan['count_only']) and
            summary['raw_complete'] == (summary['complete'] and not plan['count_only']), 'raw_mode_claim')
    f.check(plan['init_hex'] == INIT.hex() and plan['read_capacity'] == 1008 and
            plan['timestamp_domain'] == 'absolute_host_steady_clock_ns' and
            plan['clock'] == 'steady_clock_monotonic' and plan['run_requested'] is True,
            'plan_capture_contract')
    boot = plan['boot_id']
    f.check(isinstance(boot, str) and re.fullmatch(r'[0-9a-f]{8}(?:-[0-9a-f]{4}){3}-[0-9a-f]{12}', boot) is not None and
            ready['boot_id'] == boot, 'boot_id')
    f.check(ready['ready'] is True and ready['worker_pid'] == supervisor['worker_pid'] and
            ready['actual_identity'] == plan['actual_identity'], 'readiness_identity_or_pid')
    identity = plan['actual_identity']
    f.check(identity['vendor'] == 'caf1' and identity['product'] == 'ffff' and
            identity['by_path'] == plan['port_by_path'] and identity['usb_parent'], 'identity_admission')
    f.check(plan['admission_monotonic_ns'] <= summary['start_ns'] <= summary['ready_ns'] <=
            ready['monotonic_ns'] <= summary['end_ns'], 'admission_ready_time_order')
    f.check(all(supervisor[x] is True for x in ('worker_reaped', 'ports_close_verified', 'export_valid', 'capture_exported')) and
            supervisor['forced_timeout'] is False and summary['closed'] is True, 'close_export_or_timeout')
    exit_record = documents['exit.json']
    code = exit_record['exit_code']
    f.check(exit_record['reaped'] is True and exit_record['forced_by_coordinator'] is False and
            exit_record['started_monotonic_ns'] <= summary['start_ns'] <= summary['end_ns'] <= exit_record['exited_monotonic_ns'], 'coordinator_exit_or_time')
    f.check(code in (0, 1) and supervisor['complete'] == summary['complete'] == (code == 0) and
            bool(summary['error']) == (code != 0), 'exit_completion_consistency')
    stdout = (directory / 'stdout.txt').read_text(encoding='utf-8').strip()
    f.check(json.loads(stdout) == summary, 'stdout_summary_mismatch')
    f.check(summary['can_delivery_verified'] is False and plan['can_wire_timestamp_available'] is False and
            plan['can_timestamps'] is False, 'unsupported_can_delivery_claim')
    raw = audit_records((directory / 'capture.bin').read_bytes(), plan, summary)
    f.errors.extend(raw['errors'][:max(0, 20 - len(f.errors))])
    if raw['_io'] and len(raw['_io']) > 1:
        f.check(ready['monotonic_ns'] <= raw['_io'][1]['begin'], 'io_before_persisted_ready')
    raw.update(valid=not f.errors, errors=f.errors, first_failure=f.errors[0] if f.errors else None,
               complete=summary['complete'], role=plan['role'], evidence_hashes=hashes,
               manifest_path=str(manifest_path), boot_id=boot, count_only=plan['count_only'],
               reported_rx_frames=summary['rx_frames'], max_lateness_ns=summary['max_lateness_ns'],
               min_send_interval_ns=summary['min_send_interval_ns'], _plan=plan, _summary=summary, _ready=ready)
    raw['transport_failure_calls'] = [dict(direction=x['direction'], result=x['result'],
        begin_ns=x['begin'], end_ns=x['end'], capture_byte_offset=x['offset'],
        returned_bytes_known=x['returned'] != 0xffffffff) for x in raw['_io']
        if x['result'] != 0 and not (x['direction'] == 'RX' and x['result'] == 1)]
    if plan['count_only']:
        raw['limitation'] = 'RX raw bytes omitted: CRC, payload, source nonce, sequence, loss and duplication cannot be independently reconstructed; counts are worker-reported.'
    return raw


def audit_pair(observer, fixture, expected='positive', prior_nonces=()):
    f = Findings()
    op, fp = observer['_plan'], fixture['_plan']
    f.check(observer['valid'] and fixture['valid'], 'individual_evidence_invalid')
    f.check(op['role'] == 'observe' and fp['role'] == 'fixture', 'pair_roles')
    f.check(observer['boot_id'] == fixture['boot_id'], 'pair_boot_mismatch')
    f.check(op['actual_identity']['usb_parent'] != fp['actual_identity']['usb_parent'] and
            op['port_by_path'] != fp['port_by_path'], 'pair_same_board')
    f.check(fp['nonce'] not in prior_nonces, 'reused_nonce')
    writes = [x for x in fixture['_io'] if x['direction'] == 'TX' and x['index'] != 0]
    f.check(observer['complete'], 'observer_incomplete')
    if writes:
        f.check(observer['_ready']['monotonic_ns'] <= writes[0]['begin'] and
                writes[-1]['end'] <= observer['_summary']['end_ns'], 'observer_not_covering_fixture_writes')
    frames = observer['_frames']['RX']
    unknown = Counter((x['id'], x['extended']) for x in frames if x['id'] not in fp['ids'])
    counts, sequences, seen, highest = Counter(), {x: [] for x in fp['ids']}, {x: set() for x in fp['ids']}, {x: -1 for x in fp['ids']}
    for frame in frames:
        ident, payload = frame['id'], frame['payload']
        if ident not in counts and ident not in fp['ids']:
            continue  # Retained and reported as unrelated traffic, never discarded from capture.
        if fp['profile'] == 'sequence':
            lane = fp['ids'].index(ident)
            seq = int.from_bytes(payload[4:7], 'little') if len(payload) == 8 else -1
            wanted = fp['nonce'].to_bytes(4, 'little') + max(0, seq).to_bytes(3, 'little') + bytes([0xa4 + 0x11 * lane])
            valid = frame['extended'] and frame['flags'] in (4, 12) and payload == wanted and 0 <= seq < fp['frames_per_id']
            f.check(valid, 'rx_diagnostic_id_payload', detail=f'{ident:#x}@{frame["capture_byte_offset"]}')
            if valid:
                f.check(seq not in seen[ident], 'rx_duplicate_sequence', detail=f'{ident:#x}:{seq}')
                f.check(seq > highest[ident], 'rx_per_id_out_of_order', detail=f'{ident:#x}:{seq}')
                highest[ident] = max(highest[ident], seq)
                seen[ident].add(seq)
                sequences[ident].append(seq)
        else:
            f.check(frame['extended'] and frame['flags'] in (4, 12) and payload == FOUR[ident],
                    'rx_four_id_golden_mismatch', detail=f'{ident:#x}@{frame["capture_byte_offset"]}')
        counts[ident] += 1
    successful = Counter(x['id'] for x in fixture['_frames']['TX'] if x['result'] == 0)
    if expected == 'positive':
        f.check(fixture['complete'], 'fixture_incomplete')
        if observer['count_only']:
            reported = {(x['id'], x['extended']): x['frames'] for x in observer['_summary']['ids']}
            for ident in fp['ids']:
                f.check(reported.get((ident, True), 0) == successful[ident] == fp['frames_per_id'], 'count_only_reported_count_mismatch')
        else:
            for ident in fp['ids']:
                f.check(counts[ident] == successful[ident] == fp['frames_per_id'], 'actual_sent_received_count_mismatch', detail=f'{ident:#x}')
                if fp['profile'] == 'sequence':
                    f.check(sequences[ident] == list(range(fp['frames_per_id'])), 'rx_sequence_incomplete', detail=f'{ident:#x}')
    elif expected == 'negative':
        f.check(fp['profile'] == 'sequence' and fp['frames_per_id'] == 1 and len(fp['ids']) == 1 and not observer['count_only'], 'negative_not_single_full_sequence')
        f.check(len(writes) == 1 and len(fixture['_frames']['TX']) == 1, 'negative_missing_single_can_attempt')
        f.check(observer['_summary']['rx_frames'] == 0 and not frames, 'negative_observer_received_any_frame')
        f.check(fixture['complete'] or fixture['_summary']['error'] == 'write_incomplete_no_retry', 'negative_fixture_unexpected_failure')
    else:
        raise ValueError('expected must be positive or negative')
    per_id = {f'{ident:#x}': {'successful_host_write_frames': successful[ident],
                             'raw_observer_received_frames': None if observer['count_only'] else counts[ident],
                             'sequence_complete': sequences[ident] == list(range(fp['frames_per_id'])) if fp['profile'] == 'sequence' and not observer['count_only'] else None,
                             'received_sequence_sha256': hashlib.sha256(b''.join(struct.pack('<I', x) for x in sequences[ident])).hexdigest() if fp['profile'] == 'sequence' and not observer['count_only'] else None}
              for ident in fp['ids']}
    return {'valid': not f.errors, 'expected': expected, 'errors': f.errors,
            'first_failure': f.errors[0] if f.errors else None, 'profile': fp['profile'], 'nonce': fp['nonce'],
            'per_id': per_id, 'unrelated_raw_ids': [{'id': x[0], 'extended': x[1], 'frames': n} for x, n in unknown.items()],
            'unrelated_reported_ids': [x for x in observer['_summary']['ids'] if x['id'] not in fp['ids']],
            'sequence_observable': fp['profile'] == 'sequence' and not observer['count_only'],
            'can_ack_verified': False, 'can_wire_timestamps_available': False,
            'limitation': observer.get('limitation') or (FIXED_LIMITATION if fp['profile'] == 'four-id' else
                'Host write completion does not prove physical CAN transmission. Host read time is not wire arrival time.'),
            'comparison_scope': 'Only this fixture and this collector; no conclusion about production ROS control impact.'}


def public(value):
    return {key: item for key, item in value.items() if not key.startswith('_')}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory', type=Path)
    parser.add_argument('--manifest', type=Path)
    parser.add_argument('--fixture', type=Path)
    parser.add_argument('--fixture-manifest', type=Path)
    parser.add_argument('--expected', choices=('positive', 'negative'), default='positive')
    parser.add_argument('--prior-nonce', type=int, action='append', default=[])
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    try:
        observation = audit_directory(args.directory, args.manifest)
        result = {'observer': public(observation), 'valid': observation['valid']}
        if args.fixture:
            fixture = audit_directory(args.fixture, args.fixture_manifest)
            pair = audit_pair(observation, fixture, args.expected, args.prior_nonce)
            result.update(fixture=public(fixture), pair=pair, valid=pair['valid'])
    except (OSError, ValueError, KeyError, TypeError, struct.error, OverflowError) as error:
        result = {'valid': False, 'first_failure': {'category': 'evidence_schema_or_file', 'detail': str(error)}}
    # Never overwrite a previous audit or any source evidence.
    with args.output.open('x', encoding='utf-8') as out:
        json.dump(result, out, indent=2, sort_keys=True)
        out.write('\n')
    print(json.dumps({'valid': result['valid'], 'report': str(args.output)}))
    return 0 if result['valid'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
