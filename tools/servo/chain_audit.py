#!/usr/bin/env python3
"""Offline audit of captured boundaries; never imports the production codec or opens I/O.

A complete result proves captured host-chain consistency, not board CAN delivery,
motor acceptance, or correctness of the firmware's mode-6 lifecycle semantics.
"""
import sys
if __name__ == '__main__':
    sys.path.pop(0)
import argparse
import collections
import gzip
import json
import math
from pathlib import Path
import struct


def crc(data, initial, polynomial):
    value = initial
    for byte in data:
        value ^= byte
        for _ in range(8):
            value = (value >> 1) ^ (polynomial if value & 1 else 0)
    return value


def decode_packet(packet):
    if len(packet) < 7 or packet[:2] != b'\xf7\x12':
        raise ValueError('USB header/command')
    length = int.from_bytes(packet[2:4], 'little')
    if len(packet) != length + 7:
        raise ValueError('USB length')
    if packet[4] != crc(packet[1:4], 255, 0x8c):
        raise ValueError('USB CRC8')
    if int.from_bytes(packet[5:7], 'little') != crc(packet[7:], 65535, 0x8408):
        raise ValueError('USB CRC16')
    if packet == bytes.fromhex('f71206007d7008000000000000'):
        return dict(init=True)
    if length != 14 or packet[11:13] != b'\x0c\x08':
        raise ValueError('not classic extended data DLC8/send flag')
    ident = int.from_bytes(packet[7:11], 'little')
    if ident >> 8 != 6:
        raise ValueError('unexpected command mode')
    position, speed, accel = struct.unpack('>ihh', packet[13:])
    return dict(init=False, id=ident, drive=ident & 255, raw_position=position,
                target_deg=position / 10000, speed_erpm=speed * 10,
                acceleration_raw=accel * 10, payload=packet[13:].hex())


def audit_records(records):
    records = iter(records)
    meta = next(records, {})
    issues = []
    counts = collections.Counter()
    configs, limits, interfaces, dispatches, stored, prepared, selected = {}, {}, {}, {}, {}, {}, {}
    ios = {}
    claim_before, authorized = {}, {}
    acquisitions = []
    submitted = {}
    transport_io = None
    transport = None
    frames = []
    previous_ns = 0
    total = 0
    def issue(message):
        if len(issues) < 100:
            issues.append(message)
    if meta.get('schema_version') != 1:
        issue('unsupported/missing schema')
    if meta.get('dropped', 0):
        issue('dropped trace records: evidence incomplete')
    if meta.get('closed', True) is not True:
        issue('snapshot was not closed: evidence incomplete')
    for r in records:
        total += 1
        stage = r['stage']; counts[stage] += 1
        key = (r['joint'], r['generation'])
        cycle_key = (r['joint'], r['cycle'])
        if r['seq'] != total - 1 or r['ns'] < previous_ns:
            issue(f'record ordering at {total - 1}')
        previous_ns = r['ns']
        if r['captured'] != r['length']:
            issue(f'truncated bytes at seq {r["seq"]}')
        if stage == 'config_target':
            configs[r['joint']] = r
        elif stage == 'config_limits':
            limits[r['joint']] = r
        elif stage == 'claim_before':
            claim_before[r['joint']] = r
        elif stage == 'claim_after':
            before = claim_before.pop(r['joint'], None)
            if not before:
                issue(f'missing claim_before for joint {r["joint"]}')
            elif not before['flags'] & 1 and r['flags'] & 1:
                if r['position'] != r['feedback']:
                    issue(f'claim seed differs from measured position for joint {r["joint"]}')
                acquisitions.append(dict(joint=r['joint'], ns=r['ns']))
            authorized[r['joint']] = bool(r['flags'] & 1)
        elif stage == 'runtime_start':
            claim_before.clear(); authorized.clear()
            stored.clear(); prepared.clear(); selected.clear(); submitted.clear()
        elif stage in ('runtime_stop', 'fault'):
            authorized.clear()
            stored.clear(); prepared.clear(); selected.clear(); submitted.clear()
        elif stage == 'cancel':
            for ledger in (stored, prepared, selected, submitted):
                for old in [k for k in ledger if k[0] == r['joint']]:
                    del ledger[old]
        elif stage == 'interface':
            if r['flags'] & 1 and not authorized.get(r['joint']):
                issue(f'interface lacks claim evidence for joint {r["joint"]}')
            interfaces[cycle_key] = r
        elif stage == 'dispatch':
            dispatches[cycle_key] = r
            upstream = interfaces.get(cycle_key)
            if r['flags'] & 1 and (not upstream or not upstream['flags'] & 1 or
                                    not authorized.get(r['joint']) or upstream['position'] != r['position']):
                issue(f'interface/dispatch mismatch at {cycle_key}')
        elif stage == 'stored':
            for ledger in (stored, prepared, selected, submitted):
                for old in [k for k in ledger if k[0] == r['joint']]:
                    del ledger[old]
            stored[key] = r
            upstream = dispatches.get(cycle_key)
            if not upstream or upstream['flags'] != 3 or upstream['position'] != r['position']:
                issue(f'dispatch/stored mismatch at {key}')
        elif stage == 'prepared':
            prepared[key] = r
            command, cfg, lim = stored.get(key), configs.get(r['joint']), limits.get(r['joint'])
            if not command or not cfg or not lim:
                issue(f'preparation provenance missing at {key}')
            else:
                expected_position = math.trunc((command['position'] * cfg['position'] + cfg['feedback']) * 10000)
                expected = struct.pack('>ihh', expected_position,
                                       math.trunc(lim['position'] / 10), math.trunc(lim['feedback'] / 10)).hex()
                if r['hex'] != expected or r['id'] != (6 << 8 | cfg['flags']) or r['flags'] != 4:
                    issue(f'canonical/CAN mismatch at {key}')
        elif stage == 'submitted':
            command = stored.get(key)
            if (not command or key not in prepared or r['result'] != 0 or
                    r['position'] != command['position'] or r['deadline'] != command['deadline']):
                issue(f'invalid bus submission at {key}')
            submitted[key] = r
        elif stage == 'selected':
            if key not in submitted:
                issue(f'selected command lacks successful submission at {key}')
            selected[key] = r
            before = prepared.get(key)
            if not before or (r['hex'], r['id'], r['flags']) != (before['hex'], before['id'], before['flags']):
                issue(f'prepared/selected mismatch at {key}')
        elif stage == 'transport_send':
            if transport is not None:
                issue('nested transport attempt')
            transport = r
            transport_io = None
            before = selected.get(key)
            if not before or (r['hex'], r['id'], r['flags']) != (before['hex'], before['id'], before['flags']):
                issue(f'selected/transport mismatch at {key}')
        elif stage == 'serial_request':
            packet = bytes.fromhex(r['hex'])
            try:
                decoded = decode_packet(packet)
            except ValueError as error:
                issue(f'packet io {r["io"]}: {error}'); decoded = None
            if r['io'] in ios:
                issue('duplicate serial request')
            ios[r['io']] = dict(request=packet, actual=bytearray(), result=None, syscalls=0)
            if decoded and not decoded['init']:
                if transport_io is not None:
                    issue('multiple serial requests in one transport attempt')
                transport_io = r['io']
                if transport and key != (transport['joint'], transport['generation']):
                    issue('serial request provenance differs from transport')
                if not transport or (decoded['id'], decoded['payload']) != (transport['id'], transport['hex']):
                    issue(f'transport/USB mismatch io {r["io"]}')
                command = stored.get(key)
                if not command or r['ns'] >= command['deadline']:
                    issue(f'missing/expired command io {r["io"]}')
                frames.append(dict(ns=r['ns'], cycle=r['cycle'], generation=r['generation'], **decoded))
        elif stage == 'syscall_write':
            entry = ios.get(r['io'])
            if entry is None:
                issue('syscall without serial request'); continue
            entry['syscalls'] += 1
            actual = bytes.fromhex(r['hex'])
            if r['result'] > 0:
                offset = len(entry['actual'])
                if len(actual) != r['result'] or r['requested'] != len(entry['request']) - offset:
                    issue(f'write count mismatch io {r["io"]}')
                if actual != entry['request'][offset:offset + len(actual)]:
                    issue(f'USB/kernel byte mismatch io {r["io"]}')
                entry['actual'].extend(actual)
            elif r['result'] < 0 and r['errno'] == 4:
                pass  # EINTR: no accepted bytes, next attempt is checked
            else:
                issue(f'kernel write incomplete/error io {r["io"]}: result={r["result"]} errno={r["errno"]}')
        elif stage == 'serial_result':
            entry = ios.get(r['io'])
            if entry is None:
                issue('result without serial request'); continue
            entry['result'] = r['result']
            if r['result'] != 0 or entry['actual'] != entry['request'] or not entry['syscalls']:
                issue(f'serial/kernel completion not proven io {r["io"]}')
        elif stage == 'transport_result':
            fields = ('joint', 'generation', 'cycle', 'id', 'hex', 'flags')
            entry = ios.get(transport_io)
            if (not transport or r['result'] != 0 or
                    any(r[f] != transport[f] for f in fields) or
                    not entry or entry['result'] != 0 or not entry['syscalls'] or
                    entry['actual'] != entry['request']):
                issue(f'transport completion lacks matching USB/kernel evidence at {key}')
            transport = None
            transport_io = None
    if total != meta.get('total', 0) - meta.get('dropped', 0):
        issue('trace record count mismatch')
    if transport is not None or any(x['result'] is None for x in ios.values()):
        issue('unfinished I/O evidence')
    if not frames:
        issue('no motor command evidence')
    for stage in ('claim_after', 'interface', 'dispatch', 'stored', 'prepared', 'selected',
                  'transport_send', 'serial_request', 'syscall_write', 'serial_result', 'runtime_stop'):
        if not counts[stage]:
            issue(f'missing stage {stage}')
    per_drive = {}
    for drive in sorted({f['drive'] for f in frames}):
        data = [f for f in frames if f['drive'] == drive]
        intervals = [(b['ns'] - a['ns']) / 1e6 for a, b in zip(data, data[1:])]
        per_drive[drive] = dict(count=len(data), unique_targets_deg=sorted({f['target_deg'] for f in data}),
                               max_interval_ms=max(intervals, default=0),
                               first_ns=data[0]['ns'], last_ns=data[-1]['ns'])
    return dict(complete=not issues, issues=issues, stages=dict(counts), drives=per_drive, acquisitions=acquisitions,
                scope='hardware command interface through kernel write; operator events checked separately; not USB delivery or motor acceptance')


def audit_operator(events, acquisitions):
    issues = []
    if not acquisitions:
        return ['no claim acquisition to correlate with operator']
    input_requests = {}
    for claim in acquisitions:
        t = claim['ns'] / 1e9
        requests = [e for e in events if e.get('event') == 'switch_request' and e.get('enabled')
                    and e['monotonic_s'] <= t]
        if not requests:
            issues.append('claim without preceding enable service request'); continue
        request = requests[-1]
        next_requests = [e['monotonic_s'] for e in events if e.get('event') == 'switch_request'
                         and e['monotonic_s'] > request['monotonic_s']]
        end = min(next_requests, default=math.inf)
        if not any(e.get('event') == 'controller' and e.get('state') == 'active' and
                   t <= e['monotonic_s'] < end for e in events):
            issues.append('claim lacks matching active service confirmation')
        inputs = [e for e in events if e.get('event') == 'operator_input'
                  and e['monotonic_s'] <= request['monotonic_s']]
        latest = inputs[-1] if inputs else None
        words = latest.get('words', []) if latest else []
        if not latest or [str(w).lower() for w in words] != ['enable']:
            issues.append('enable request lacks matching operator input')
        else:
            input_time = latest['monotonic_s']
            previous_request = input_requests.setdefault(input_time, request['monotonic_s'])
            if previous_request != request['monotonic_s']:
                issues.append('enable service reused an earlier operator input')
    return issues


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('trace', type=Path)
    parser.add_argument('--hardware-only', action='store_true', help='explicitly omit operator/service correlation (offline component tests)')
    args = parser.parse_args()
    try:
        opener = gzip.open if args.trace.suffix == '.gz' else open
        with opener(args.trace, 'rt', encoding='utf-8') as stream:
            report = audit_records(json.loads(line) for line in stream)
        manifest_path = args.trace.with_name('capture-manifest.json')
        if manifest_path.exists():
            manifest = json.loads(manifest_path.read_text())
            if manifest.get('diagnostic_status') != 'complete':
                report['issues'].append('capture manifest reports incomplete diagnostics')
                report['complete'] = False
        if not args.hardware_only:
            events = [json.loads(line) for line in args.trace.with_name('events.jsonl').open()]
            operator_issues = audit_operator(events, report['acquisitions'])
            report['issues'].extend(operator_issues)
            report['complete'] = report['complete'] and not operator_issues
            report['operator_events_verified'] = not operator_issues
    except (ValueError, KeyError, TypeError, OSError, EOFError, OverflowError, struct.error) as error:
        report = dict(complete=False, issues=[f'invalid/incomplete trace: {error}'])
    print(json.dumps(report, ensure_ascii=False, indent=2))
    return 0 if report['complete'] else 1

if __name__ == '__main__':
    raise SystemExit(main())
