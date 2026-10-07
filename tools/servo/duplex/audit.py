#!/usr/bin/env python3
"""Independent file-only audit of dual-port fixed-payload duplex evidence.

CRC implementation and frame decoding are independent of the production code.
No device access, imports of production codecs, or automatic retries.
"""
import argparse
from bisect import bisect_left, bisect_right
from collections import Counter
import hashlib
import json
import math
from pathlib import Path
import re
import struct

OUTER = struct.Struct('<BBBBIQ')
META = struct.Struct('<QII')
INIT = bytes.fromhex('f71206007d7008000000000000')
COMMANDS = {0x668: bytes.fromhex('000cd528000a000a'),
            0x669: bytes.fromhex('000ea218000a000a')}
FEEDBACK = {0x2968: bytes.fromhex('03490000ffff2a00'),
            0x2969: bytes.fromhex('03be000000252a00')}
COMMAND_PACKETS_HEX = ['f7120e000bf8e2680600000c08000cd528000a000a',
                       'f7120e000ba8a6690600000c08000ea218000a000a']
FEEDBACK_PACKETS_HEX = ['f7120e000bb1b5682900000c0803490000ffff2a00',
                       'f7120e000bb5a2692900000c0803be000000252a00']
FIXED_LIMITATION = ('Fixed identical payloads support content and count comparison; '
                   'they cannot identify individual frames, ordering, or equal '
                   'loss and duplication that cancel in the counts. Feedback '
                   'is synthetic fixture data, never measured motor feedback.')


def crc(data, polynomial, seed):
    value = seed
    for byte in data:
        value ^= byte
        for _ in range(8):
            value = (value >> 1) ^ (polynomial if value & 1 else 0)
    return value


class Invalid(ValueError):
    def __init__(self, category, byte=0, detail=''):
        super().__init__(category)
        self.category, self.byte, self.detail = category, byte, detail


def decode(packet):
    if len(packet) < 7 or packet[:2] != b'\xf7\x12':
        raise Invalid('cdc_header', 0)
    length = int.from_bytes(packet[2:4], 'little')
    if not 6 <= length <= 512 or len(packet) != length + 7:
        raise Invalid('cdc_length', 2)
    if crc(packet[1:4], 0x8c, 0xff) != packet[4]:
        raise Invalid('cdc_header_crc', 4)
    if crc(packet[7:], 0x8408, 0xffff) != int.from_bytes(packet[5:7], 'little'):
        raise Invalid('cdc_payload_crc', 5)
    point = 7
    prefixed = False
    if length >= 9:
        low = int.from_bytes(packet[7:10], 'little')
        ident = int.from_bytes(packet[10:14], 'little')
        flags, size = packet[14:16]
        if low == ident & 0xffffff and flags and size and size <= (64 if flags & 2 else 8) and size + 9 <= length:
            point, prefixed = 10, True
    frames = []
    while point < len(packet):
        if len(packet) - point < 6 or len(frames) >= 64:
            raise Invalid('cdc_record_shape', point)
        ident, flags, size = struct.unpack_from('<IBB', packet, point)
        valid_sizes = (*range(9), 12, 16, 20, 24, 32, 48, 64) if flags & 2 else range(9)
        if (flags & 0xf0 or flags & 1 and not flags & 2 or size not in valid_sizes or
                ident > (0x1fffffff if flags & 4 else 0x7ff) or point + 6 + size > len(packet)):
            raise Invalid('cdc_record_shape', point, f'id={ident:#x},flags={flags:#x},size={size}')
        frames.append({'id': ident, 'extended': bool(flags & 4), 'flags': flags,
                       'payload': packet[point + 6:point + 6 + size], 'byte': point})
        point += 6 + size
    return frames, prefixed


class Findings:
    def __init__(self):
        self.errors = []
        self.error_count = 0

    def check(self, condition, category, record=None, byte=0, detail=''):
        if condition:
            return True
        self.error_count += 1
        finding = {'category': category, 'detail': detail}
        if record is not None:
            finding.update(port=record['port'], direction=record['direction'],
                           record_index=record['index'], begin_ns=record['begin_ns'], end_ns=record['end_ns'],
                           capture_byte_offset=record['offset'] + 32 + byte)
        if len(self.errors) < 20:
            self.errors.append(finding)
        return False


def scan_capture(raw, port):
    """Decode each full observer-schema2 stream and retain all valid CAN IDs."""
    findings = Findings()
    frames, io, init = {'TX': [], 'RX': []}, [], []
    pending, locations = bytearray(), []
    offset, previous, prefixes, rx_bytes, poisoned = 0, 0, 0, 0, False
    while offset < len(raw):
        if not findings.check(len(raw) - offset >= 32, 'capture_truncated_header', detail=str(offset)):
            break
        kind, recorded_port, result, reserved, size, end = OUTER.unpack_from(raw, offset)
        begin, requested, returned = META.unpack_from(raw, offset + 16)
        record = {'port': port, 'index': len(io), 'direction': 'TX' if kind == 1 else 'RX',
                  'offset': offset, 'begin_ns': begin, 'end_ns': end,
                  'result': result, 'requested': requested, 'returned': returned}
        if not findings.check(kind in (1, 2) and recorded_port == 0 and reserved == 0 and result <= 5 and
                              16 <= size <= 1024 and previous <= begin <= end, 'capture_header', record):
            break
        if not findings.check(offset + 16 + size <= len(raw), 'capture_truncated_payload', record):
            break
        data = raw[offset + 32:offset + 16 + size]
        io.append(record)
        if kind == 1:
            findings.check(requested == len(data) and 0 < requested <= 1008 and
                           returned == (requested if result == 0 else 0xffffffff), 'tx_io_metadata', record)
            if len(io) == 1:
                findings.check(data == INIT, 'initial_write_not_init', record)
            if data == INIT:
                init.append(record)
                findings.check(len(io) == 1, 'late_initialization', record)
            else:
                try:
                    decoded, prefix = decode(data)
                    findings.check(not prefix and len(decoded) == 1 and len(data) == 21,
                                   'tx_not_separate_21_byte_frame', record)
                    for frame in decoded:
                        findings.check(frame['flags'] == 12 and len(frame['payload']) == 8, 'tx_frame_shape', record, frame['byte'])
                        frame.update(port=port, direction='TX', begin_ns=begin, end_ns=end, result=result,
                                     capture_byte_offset=offset + 32 + frame['byte'])
                        frames['TX'].append(frame)
                except Invalid as error:
                    findings.check(False, error.category, record, error.byte, error.detail)
        else:
            findings.check(requested == 1008 and returned <= requested and len(data) == returned,
                           'rx_io_metadata', record)
            findings.check(result != 1 or returned == 0, 'wouldblock_with_bytes', record)
            rx_bytes += returned
            if result == 0 and data and not poisoned:
                pending.extend(data)
                locations.extend((record, point) for point in range(len(data)))
                while len(pending) >= 7:
                    length = int.from_bytes(pending[2:4], 'little')
                    header = pending[:2] == b'\xf7\x12' and 6 <= length <= 512 and crc(pending[1:4], 0x8c, 0xff) == pending[4]
                    if not header:
                        source, within = locations[0]
                        findings.check(False, 'rx_stream_header', source, within)
                        poisoned = True
                        break
                    if len(pending) < length + 7:
                        break
                    try:
                        decoded, prefix = decode(bytes(pending[:length + 7]))
                        prefixes += prefix
                        for frame in decoded:
                            source, within = locations[frame['byte']]
                            frame.update(port=port, direction='RX', begin_ns=source['begin_ns'], end_ns=end,
                                         result=0, capture_byte_offset=source['offset'] + 32 + within)
                            frames['RX'].append(frame)
                    except Invalid as error:
                        source, within = locations[error.byte]
                        findings.check(False, error.category, source, within, error.detail)
                        poisoned = True
                        break
                    del pending[:length + 7]
                    del locations[:length + 7]
        previous, offset = end, offset + 16 + size
    if pending and not poisoned:
        source, within = locations[0]
        findings.check(False, 'rx_truncated_stream', source, within, str(len(pending)))
    findings.check(len(init) == 1, 'initialization_count', detail=str(len(init)))
    return {'valid': not findings.errors, 'errors': findings.errors, 'error_count': findings.error_count,
            'first_failure': findings.errors[0] if findings.errors else None,
            'capture_bytes': len(raw), 'port': port, 'io_calls': dict(Counter(x['direction'] for x in io)),
            'io_outcomes': dict(Counter(f"{x['direction']}_{x['result']}" for x in io)),
            'initialization_writes': len(init), 'rx_bytes': rx_bytes,
            'successful_can_write_frames': sum(frame['result'] == 0 for frame in frames['TX']),
            'decoded_rx_frames': len(frames['RX']), 'prefix3_packets': prefixes,
            '_frames': frames, '_io': io, '_init': init}


def compare_direction(sender, receiver, expected_payloads):
    findings = Findings()
    attempted, sent, received = Counter(), Counter(), Counter()
    mismatches, unknown = [], []
    for frame in sender['_frames']['TX']:
        attempted[frame['id']] += 1
        valid = frame['id'] in expected_payloads and frame['payload'] == expected_payloads.get(frame['id']) and frame['flags'] == 12
        findings.check(valid, 'tx_fixed_payload_mismatch', detail=f"{frame['id']:#x}@{frame['capture_byte_offset']}")
        if frame['result'] == 0:
            sent[frame['id']] += 1
    for frame in receiver['_frames']['RX']:
        received[frame['id']] += 1
        item = {key: frame[key] for key in ('id', 'flags', 'begin_ns', 'end_ns', 'capture_byte_offset')}
        item['payload_hex'] = frame['payload'].hex()
        if frame['id'] not in expected_payloads:
            unknown.append(item)
            findings.check(False, 'rx_unknown_or_wrong_direction_id', detail=f"{frame['id']:#x}@{frame['capture_byte_offset']}")
        elif frame['flags'] not in (4, 12) or frame['payload'] != expected_payloads[frame['id']]:
            mismatches.append(item)
            findings.check(False, 'rx_fixed_payload_mismatch', detail=f"{frame['id']:#x}@{frame['capture_byte_offset']}")
    per_id = {}
    for ident in expected_payloads:
        difference = received[ident] - sent[ident]
        findings.check(difference == 0, 'received_minus_successful_hostwrite_count_deviation', detail=f'{ident:#x}:{difference}')
        per_id[f'{ident:#x}'] = {'attempted_hostwrite_frames': attempted[ident],
                               'successful_hostwrite_frames': sent[ident], 'received_frames': received[ident],
                               'received_minus_successful_hostwrite_count': difference,
                               'expected_payload_hex': expected_payloads[ident].hex()}
    return {'content_match': not findings.errors, 'first_failure': findings.errors[0] if findings.errors else None,
            'errors': findings.errors, 'error_count': findings.error_count, 'per_id': per_id,
            'payload_mismatch_frames': mismatches[:20], 'payload_mismatch_count': len(mismatches),
            'unknown_or_wrong_direction_frames': unknown[:20], 'unknown_or_wrong_direction_count': len(unknown),
            'sequence_observable': False, 'balanced_loss_duplicate_and_order_observable': False,
            'limitation': FIXED_LIMITATION, 'can_delivery_ack_verified': False}


def read_json(path):
    def unique(pairs):
        result = {}
        for key,value in pairs:
            if key in result:
                raise ValueError(f'duplicate JSON key:{key}')
            result[key] = value
        return result
    return json.loads(path.read_text(encoding='utf-8'), object_pairs_hook=unique)


def public(result):
    return {key:value for key,value in result.items() if not key.startswith('_')}


ORIGINALS = {'admission.json', 'capture-a.bin', 'capture-b.bin', 'exit.json', 'plan.json',
             'ready.json', 'stderr.txt', 'stdout.txt', 'summary.json', 'supervisor.json'}


def describe(values):
    ordered = sorted(values)
    return {'samples': len(ordered), 'min_ns': ordered[0] if ordered else None,
            'max_ns': ordered[-1] if ordered else None,
            'p99_nearest_rank_ns': ordered[math.ceil(len(ordered)*.99)-1] if ordered else None}


def timing(capture):
    report = {'per_id_host_write': {}}
    frames = [x for x in capture['_frames']['TX'] if x['result'] == 0]
    for ident in sorted({x['id'] for x in frames}):
        writes = [x for x in frames if x['id'] == ident]
        report['per_id_host_write'][f'{ident:#x}'] = {
            'begin_interval': describe([b['begin_ns']-a['begin_ns'] for a,b in zip(writes,writes[1:])]),
            'call_duration': describe([x['end_ns']-x['begin_ns'] for x in writes])}
    reads = [x for x in capture['_io'] if x['direction'] == 'RX']
    report['host_read_begin_interval'] = describe([b['begin_ns']-a['begin_ns'] for a,b in zip(reads,reads[1:])])
    report['host_read_call_duration'] = describe([x['end_ns']-x['begin_ns'] for x in reads])
    report['limitation'] = 'Descriptive host IO timings only; no physical CAN timestamps, hard real-time guarantee, or additional pass threshold.'
    return report


def verify_board(scan, reported, plan, findings):
    port = scan['port']
    findings.check(reported['schema'] == 2 and reported['role'] == port and reported['count_only'] is False,
                   'board_schema_or_role', detail=port)
    findings.check(all(type(reported[key]) is int and reported[key] >= 0 for key in (
        'capture_bytes','start_ns','ready_ns','end_ns','first_frame_ns','last_frame_ns','read_calls',
        'write_calls','rx_bytes','rx_frames','tx_frames','max_lateness_ns','min_send_interval_ns')), 'board_numeric_types', detail=port)
    findings.check(reported['capture_bytes'] == scan['capture_bytes'] <= plan['capture_limit_bytes_per_port'], 'board_capture_size', detail=port)
    findings.check(reported['read_calls'] == scan['io_calls'].get('RX',0) <= 1000000 and
                   reported['write_calls'] == scan['io_calls'].get('TX',0), 'board_io_count', detail=port)
    findings.check(reported['rx_bytes'] == scan['rx_bytes'] and reported['rx_frames'] == scan['decoded_rx_frames'] <= 1000000 and
                   reported['tx_frames'] == scan['successful_can_write_frames'], 'board_frame_or_byte_count', detail=port)
    counts = Counter((x['id'],x['extended']) for x in scan['_frames']['RX'])
    summary_counts = {(x['id'],x['extended']):x['frames'] for x in reported['ids']}
    findings.check(len(summary_counts) == len(reported['ids']) <= 256 and summary_counts == counts, 'board_id_counts', detail=port)
    received = scan['_frames']['RX']
    findings.check((reported['first_frame_ns'],reported['last_frame_ns']) ==
                   ((received[0]['end_ns'],received[-1]['end_ns']) if received else (0,0)), 'board_frame_times', detail=port)
    if scan['_io']:
        findings.check(reported['start_ns'] <= scan['_io'][0]['begin_ns'] and scan['_io'][-1]['end_ns'] <= reported['end_ns'],
                       'board_time_window', detail=port)
    findings.check(reported['can_delivery_verified'] is False, 'unsupported_delivery_claim', detail=port)


def content_counts(capture, expected):
    frames = capture['_frames']['RX']
    return {'received_per_id':[sum(x['id'] == ident for x in frames) for ident in expected],
            'unexpected_ids':sum(x['id'] not in expected for x in frames),
            'payload_mismatches':sum(x['id'] in expected and x['payload'] != expected[x['id']] for x in frames),
            'format_mismatches':sum(x['id'] in expected and x['flags'] not in (4,12) for x in frames)}


def feedback_payload(ident, ordinal):
    if ident not in FEEDBACK or not 0<=ordinal<250: raise ValueError('sequence range')
    b=bytearray(FEEDBACK[ident]);b[2:4]=(0x4000|((ident-0x2968)<<8)|(ordinal+1)).to_bytes(2,'big');return bytes(b)


def feedback_ordinal(frame):
    if frame['id'] not in FEEDBACK or len(frame['payload'])!=8:return None
    tag=int.from_bytes(frame['payload'][2:4],'big');n=(tag&255)-1
    return n if 0<=n<250 and tag==(0x4000|((frame['id']-0x2968)<<8)|(n+1)) else None


def normalize_feedback(scan):
    def norm(f):
        if feedback_ordinal(f) is None:return f
        b=bytearray(f['payload']);b[2:4]=b'\x00\x00';return {**f,'payload':bytes(b)}
    return {**scan,'_frames':{direction:[norm(f) for f in frames] for direction,frames in scan['_frames'].items()}}


def sequence_report(a,b):
    sent=Counter();received=Counter();invalid=[]
    for direction,scan,counts in [('TX',a,sent),('RX',b,received)]:
        for f in scan['_frames'][direction]:
            if f['id'] not in FEEDBACK:continue
            n=feedback_ordinal(f)
            if n is None:invalid.append({'direction':direction,'id':f['id'],'payload_hex':f['payload'].hex()})
            else:counts[(f['id'],n)]+=1
    keys=sorted(set(sent)|set(received))
    deviations=[{'id':hex(i),'ordinal':n,'sent':sent[(i,n)],'received':received[(i,n)]} for i,n in keys if sent[(i,n)]!=1 or received[(i,n)]!=1]
    return {'content_match':not invalid and not deviations,'invalid_tags':invalid,'deviations':deviations,
            'sent_tags':sum(sent.values()),'received_tags':sum(received.values()),'unique_sent_tags':len(sent),'unique_received_tags':len(received),
            'limitation':'tags identify synthetic feedback within this run, not CAN delivery or unique command frames'}


def replay_contract(recipe):
    """External reviewed JSON recipe, not metadata from the captured program."""
    return {'profile':recipe['profile'],'duration_ns':recipe['duration_ns'],
            'max_lateness_ns':250000,'read_guard_ns':200000,
            'timestamp_basis':'host_schedule_not_CAN_wire_time',
            'events':[[e['offset_ns'],int(e['port']=='B'),int(e['id'],16)&1] for e in recipe['events']]}


def verify_replay(plan, summary, a, b, expected, findings):
    r=plan['replay']
    mixed=plan.get('mixed_receive',False)
    findings.check(expected is not None and r == expected, 'replay_external_recipe_mismatch')
    findings.check(r['profile'] in ('fixed_forward__paired','observed_order__paired',
                   'fixed_forward__observed_separation','observed_order__observed_separation') and
                   (plan['seconds'],plan['command_hz'],plan['feedback_hz'],plan['feedback_order'],
                    plan['feedback_phase_ms'],plan['rx_gate_ms'])==(5,500,50,'forward',0,0) and
                   r['duration_ns']==5000000000 and r['max_lateness_ns']==250000 and
                   r['read_guard_ns']==200000, 'replay_contract_invalid')
    events=r['events']
    findings.check(len(events)==5500 and all(len(e)==3 and all(type(v) is int for v in e) and
                   0<=e[0]<5000000000 and e[1] in (0,1) and e[2] in (0,1) for e in events), 'replay_events_invalid')
    findings.check(events==sorted(events,key=lambda e:(e[0],e[1])), 'replay_event_order')
    report={}
    actual=[]
    ordinals=[0,0]
    for port,scan in enumerate((a,b)):
        selected=events if mixed and port==0 else ([] if mixed else [e for e in events if e[1]==port])
        writes=scan['_frames']['TX']
        findings.check(len(writes)==len(selected) if summary['collection_complete'] else len(writes)<=len(selected),
                       'replay_tx_count',detail=str(port))
        lates=[]
        for i,(write,e) in enumerate(zip(writes,selected)):
            ident=(0x2968 if e[1] else 0x668)+e[2]
            payload=(FEEDBACK if e[1] else COMMANDS)[ident]
            if plan.get('feedback_sequence') and e[1]:
                payload=feedback_payload(ident,ordinals[e[2]]);ordinals[e[2]]+=1
            deadline=summary['epoch_ns']+e[0]
            late=write['begin_ns']-deadline
            findings.check(write['id']==ident and write['payload']==payload and write['flags']==12 and
                           write['result']==0, 'replay_actual_tx_differs', detail=f'{port}:{i}')
            findings.check(0<=late<=250000,'replay_submission_lateness',detail=f'{port}:{i}:{late}')
            actual.append((write['begin_ns'],e));lates.append(late)
        intervals=[y['begin_ns']-x['begin_ns'] for x,y in zip(writes,writes[1:])]
        board=summary['b' if port else 'a']
        findings.check(board['max_lateness_ns']==max(lates,default=0) and
                       board['min_send_interval_ns']==min(intervals,default=0),'replay_timing_summary',detail=str(port))
        first=[e for e in events if e[1]==port][:2]
        golden=FEEDBACK_PACKETS_HEX if port else COMMAND_PACKETS_HEX
        findings.check(plan['b_first_writes_hex' if port else 'a_first_writes_hex']==[golden[e[2]] for e in first],
                       'replay_first_write_plan')
        report['B' if port else 'A']={'events':len(writes),'lateness_ns':describe(lates),'actual_submission_interval_ns':describe(intervals)}
    actual.sort(key=lambda e:e[0])
    findings.check([e for _,e in actual]==events[:len(actual)], 'replay_global_submission_order')
    return report


def audit_directory(directory, manifest=None, expected_replay=None, expected_mixed=False, expected_sequence=False):
    directory = Path(directory)
    findings = Findings()
    hashes = {}
    for name in sorted(ORIGINALS):
        path = directory / name
        findings.check(path.is_file() and not path.is_symlink(), 'original_not_regular', detail=name)
        if path.is_file():
            data = path.read_bytes()
            hashes[name] = {'sha256':hashlib.sha256(data).hexdigest(),'size':len(data)}
    remote_path = Path(manifest) if manifest else directory / 'evidence-manifest.json'
    remote = read_json(remote_path)
    findings.check(set(remote['files']) == ORIGINALS and remote['complete_file_set'] is True, 'manifest_original_set')
    for name in ORIGINALS:
        findings.check(name in hashes and remote['files'].get(name) == hashes.get(name), 'original_remote_hash_or_size', detail=name)
    a = scan_capture((directory / 'capture-a.bin').read_bytes(), 'A')
    b = scan_capture((directory / 'capture-b.bin').read_bytes(), 'B')
    a_to_b, b_to_a = compare_direction(a,b,COMMANDS), compare_direction(b,a,FEEDBACK)
    scans = {'A':a,'B':b}
    documents = {}
    for name in ('admission.json','plan.json','ready.json','summary.json','supervisor.json','exit.json'):
        try:
            documents[name] = read_json(directory / name)
        except (OSError,ValueError) as error:
            findings.check(False,'metadata_missing_or_invalid',detail=f'{name}:{error}')
    content_match = a_to_b['content_match'] and b_to_a['content_match']
    if len(documents) != 6:
        return {'valid':False,'metadata_valid':False,'acquisition_complete':False,'content_match':content_match,
                'errors':findings.errors,'evidence_hashes':hashes,'A':public(a),'B':public(b),
                'A_to_B':a_to_b,'B_to_A':b_to_a,'limitation':FIXED_LIMITATION}
    plan,summary = documents['plan.json'],documents['summary.json']
    sequence=plan.get('feedback_sequence',False)
    findings.check(type(sequence) is bool and sequence==expected_sequence,'sequence_external_expectation')
    sequence_check={}
    norm_a,norm_b=a,b
    if sequence:
        findings.check(plan.get('mixed_receive') is True and summary.get('feedback_sequence') is True and
                       plan.get('feedback_tag_scheme')=='u16be_bytes2_3_0x4000_lane8_ordinal1to250','sequence_contract')
        sequence_check=sequence_report(a,b);norm_a,norm_b=normalize_feedback(a),normalize_feedback(b)
        findings.check(summary.get('feedback_sequence_match')==(sequence_check['content_match'] if summary['collection_complete'] else False),'sequence_summary_match')
    mixed=plan.get('mixed_receive',False)
    findings.check(type(mixed) is bool and mixed==expected_mixed,'mixed_route_external_expectation')
    if mixed:
        findings.check(plan.get('replay_port_semantics')=='frame_family_0_command_1_feedback_all_from_A' and
                       summary.get('mixed_receive') is True and plan.get('replay') is not None,'mixed_contract')
        a_to_b,b_to_a=compare_direction(norm_a,norm_b,{**COMMANDS,**FEEDBACK}),compare_direction(norm_b,norm_a,{})
        content_match=a_to_b['content_match'] and b_to_a['content_match'] and (not sequence or sequence_check['content_match'])
    replay = plan.get('replay')
    ready,supervisor,exit_record = documents['ready.json'],documents['supervisor.json'],documents['exit.json']
    findings.check(documents['admission.json'] == plan, 'admission_plan_mismatch')
    findings.check(plan['schema'] == summary['schema'] == 2 and plan['role'] == summary['role'] == ready['role'] == 'duplex', 'schema_or_role')
    findings.check(plan['count_only'] is False and plan['nonce_on_wire'] is False and
                   plan['timestamp_domain'] == 'absolute_host_steady_clock_ns' and plan['clock'] == 'steady_clock_monotonic' and
                   plan['packing'] == 'separate' and plan['read_capacity'] == 1008 and plan['read_budget_per_port_per_pass'] == 1 and
                   plan['init_hex'] == INIT.hex(), 'capture_contract')
    findings.check(type(plan['seconds']) is int and 1 <= plan['seconds'] <= 60 and
                   type(plan['command_hz']) is int and 1 <= plan['command_hz'] <= 500 and
                   type(plan['feedback_hz']) is int and 0 <= plan['feedback_hz'] <= 50 and
                   plan['quiet_ms'] == 2000 and 1000 <= plan['drain_ms'] <= 5000 and
                   plan['command_ticks'] == plan['command_hz']*plan['seconds'] and
                   plan['feedback_ticks'] == plan['feedback_hz']*plan['seconds'] and
                   1048576 <= plan['capture_limit_bytes_per_port'] <= 64*1048576, 'plan_limits')
    findings.check(plan['a_tx_ids'] == (list(COMMANDS)+list(FEEDBACK) if mixed else list(COMMANDS)) and plan['b_tx_ids'] == ([] if mixed else list(FEEDBACK)) and
                   plan['command_payloads_hex'] == [x.hex() for x in COMMANDS.values()] and
                   plan['feedback_payloads_hex'] == [x.hex() for x in FEEDBACK.values()] and
                   plan['synthetic_feedback'] is True and plan['feedback_provenance'] == 'synthetic_fixture', 'plan_golden_fixture')
    feedback_order = plan.get('feedback_order','forward')
    findings.check(feedback_order in ('forward','reverse'), 'feedback_order_unknown')
    feedback_phase_ms = plan.get('feedback_phase_ms',0)
    findings.check(type(feedback_phase_ms) is int and feedback_phase_ms in (0,5) and
                   (plan['feedback_hz'] != 0 or feedback_phase_ms == 0), 'feedback_phase_invalid')
    rx_gate_ms = plan.get('rx_gate_ms',0)
    gate_valid = type(rx_gate_ms) is int and rx_gate_ms in (0,6) and (rx_gate_ms == 0 or
        (plan['seconds'],plan['command_hz'],plan['feedback_hz'],feedback_order) == (2,10,10,'forward'))
    findings.check(gate_valid, 'rx_gate_invalid')
    if 'rx_gate_ms' in plan:
        findings.check(plan.get('rx_gate_anchor') == 'A_nominal_tick' and
                       plan.get('rx_gate_scope') == 'transmit_only_both_ports', 'rx_gate_contract')
    ordered_ids = list(FEEDBACK)[::-1] if feedback_order == 'reverse' else list(FEEDBACK)
    expected_b_packets = FEEDBACK_PACKETS_HEX[::-1] if feedback_order == 'reverse' else FEEDBACK_PACKETS_HEX
    replay_report = {}
    if replay is not None:
        replay_report = verify_replay(plan,summary,a,b,expected_replay,findings)
    else:
        findings.check(expected_replay is None,'expected_replay_missing_from_capture')
        findings.check(plan['a_first_writes_hex'] == COMMAND_PACKETS_HEX and
                       plan['b_first_writes_hex'] == expected_b_packets, 'first_writes_plan_order')
        findings.check(all(frame['id'] == ordered_ids[index % 2] for index,frame in enumerate(b['_frames']['TX'])),
                       'feedback_write_order')
    findings.check(plan['run_requested'] is True and plan['motors_disconnected_declared'] is True, 'run_isolation_admission')
    boot = plan['boot_id']
    findings.check(isinstance(boot,str) and re.fullmatch(r'[0-9a-f]{8}(?:-[0-9a-f]{4}){3}-[0-9a-f]{12}',boot) is not None and
                   ready['boot_id'] == remote['boot_id'] == boot, 'boot_identity')
    findings.check(ready['ready'] is True and ready['worker_pid'] == supervisor['worker_pid'] and
                   ready['actual_identity_a'] == plan['actual_identity_a'] and ready['actual_identity_b'] == plan['actual_identity_b'], 'ready_identity_pid')
    ida,idb = plan['actual_identity_a'],plan['actual_identity_b']
    findings.check(ida['usb_parent'] != idb['usb_parent'] and ida['tty'] != idb['tty'] and
                   ida['by_path'] == plan['port_a_by_path'] != plan['port_b_by_path'] == idb['by_path'] and
                   ida['vendor'] == idb['vendor'] == 'caf1' and ida['product'] == idb['product'] == 'ffff', 'distinct_admitted_boards')
    findings.check(all(supervisor[key] is True for key in ('worker_reaped','ports_close_verified','export_valid','capture_exported')) and
                   supervisor['forced_timeout'] is False and summary['closed'] is True and
                   summary['a']['closed'] is summary['b']['closed'] is True and
                   exit_record['reaped'] is True and exit_record['forced_by_coordinator'] is False, 'close_reap_export_or_timeout')
    collection = summary['collection_complete']
    findings.check(type(collection) is bool and summary['complete'] == summary['raw_complete'] ==
                   summary['a']['complete'] == summary['b']['complete'] == summary['a']['raw_complete'] == summary['b']['raw_complete'] == collection and
                   bool(summary['error']) == (not collection), 'collection_claim_consistency')
    findings.check(summary['content_match'] == content_match if collection else summary['content_match'] is False, 'content_claim_consistency')
    expected_exit = 0 if collection and content_match else (4 if collection else 1)
    findings.check(supervisor['worker_exit_code'] == exit_record['exit_code'] == expected_exit and
                   supervisor['collection_complete'] == collection and supervisor['complete'] == (expected_exit == 0), 'exit_categories')
    findings.check(json.loads((directory / 'stdout.txt').read_text()) == summary, 'stdout_summary')
    findings.check(plan['can_wire_timestamp_available'] is False and plan['can_timestamps'] is False and
                   summary['can_delivery_verified'] is False and summary['motor_feedback_verified'] is False and
                   summary['fixed_payload_sequence_observable'] is False, 'unsupported_observability_claim')
    for scan,reported in ((a,summary['a']),(b,summary['b'])):
        verify_board(scan,reported,plan,findings)
    if mixed:
        def without_feedback(scan):
            return {**scan,'_frames':{**scan['_frames'],'RX':[x for x in scan['_frames']['RX'] if x['id'] not in FEEDBACK]}}
        feedback_only={**norm_b,'_frames':{**norm_b['_frames'],'RX':[x for x in norm_b['_frames']['RX'] if x['id'] in FEEDBACK]}}
        findings.check(summary['content_a']==content_counts(a,FEEDBACK) and
                       summary['content_b']==content_counts(without_feedback(b),COMMANDS) and
                       summary['content_b_feedback']==content_counts(feedback_only,FEEDBACK),'mixed_content_counters')
    else:
        findings.check(summary['content_a'] == content_counts(a,FEEDBACK) and summary['content_b'] == content_counts(b,COMMANDS), 'content_counter_summary')
    findings.check(summary['sent_a'] == [a_to_b['per_id'][f'{ident:#x}']['successful_hostwrite_frames'] for ident in COMMANDS] and
                   summary['sent_b'] == [(a_to_b if mixed else b_to_a)['per_id'][f'{ident:#x}']['successful_hostwrite_frames'] for ident in FEEDBACK], 'successful_send_summary')
    quiet_begin,quiet_end,epoch,transmit_end = (summary[key] for key in ('quiet_begin_ns','quiet_end_ns','epoch_ns','transmit_end_ns'))
    findings.check(all(frame['begin_ns'] >= epoch+feedback_phase_ms*1000000 for frame in b['_frames']['TX']),
                   'feedback_write_before_phase')
    read_gate_report = {}
    if replay is None and gate_valid and type(plan['command_hz']) is int and 1 <= plan['command_hz'] <= 500:
        gate_ns, period_ns = rx_gate_ms*1000000, 1000000000//plan['command_hz']
        for label,scan in scans.items():
            active_reads = [r for r in scan['_io'] if r['direction'] == 'RX' and epoch <= r['begin_ns'] < transmit_end]
            read_begins = [r['begin_ns'] for r in active_reads]
            read_ends = [r['end_ns'] for r in active_reads]
            reads_by_tick = {}
            for read in active_reads:
                reads_by_tick.setdefault((read['begin_ns']-epoch)//period_ns,[]).append(read)
            crossing = []
            ticks = []
            for tick in range(plan['command_ticks']):
                anchor = epoch+tick*period_ns
                reads = reads_by_tick.get(tick,[])
                first = reads[0] if reads else None
                row = {'tick':tick,'A_nominal_tick_ns':anchor,'gate_end_ns':anchor+gate_ns,
                              'reads_started_in_tick':len(reads),
                              'first_read_begin_ns':first['begin_ns'] if first else None,
                              'first_read_begin_minus_A_nominal_tick_ns':first['begin_ns']-anchor if first else None,
                              'first_read_begin_minus_B_phase_deadline_ns':first['begin_ns']-anchor-feedback_phase_ms*1000000 if first else None}
                # Descriptive confounder check, independent of the begin gate:
                # include a read started earlier that continues into the span.
                a_group,b_group = a['_frames']['TX'][2*tick:2*tick+2],b['_frames']['TX'][2*tick:2*tick+2]
                if plan['command_hz'] == plan['feedback_hz'] and len(a_group) == len(b_group) == 2:
                    span_start,span_end = a_group[0]['begin_ns'],b_group[1]['end_ns']
                    overlapping = active_reads[bisect_left(read_ends,span_start):bisect_right(read_begins,span_end)]
                    row['AB_submission_span_ns'] = {'A_first_write_begin':span_start,'B_second_write_end':span_end}
                    row['read_intervals_intersecting_AB_submission_span'] = [
                        {'record_index':r['index'],'begin_ns':r['begin_ns'],'end_ns':r['end_ns'],
                         'started_before_A_first_write':r['begin_ns']<span_start} for r in overlapping]
                else:
                    row['AB_submission_span_ns'] = None
                    row['read_intervals_intersecting_AB_submission_span'] = None
                ticks.append(row)
            for read in active_reads:
                tick = (read['begin_ns']-epoch)//period_ns
                anchor = epoch+tick*period_ns
                findings.check(read['begin_ns'] >= anchor+gate_ns, 'rx_read_started_in_gate', read,
                               detail=f'{label}:tick={tick},gate_ms={rx_gate_ms}')
                # Gate is a begin-only rule. Report, without a new rejection
                # threshold, a compliant call continuing into a later gate.
                next_anchor = anchor+period_ns
                if gate_ns and next_anchor < transmit_end and read['end_ns'] > next_anchor:
                    crossing.append({'record_index':read['index'],'begin_ns':read['begin_ns'],
                                     'end_ns':read['end_ns'],'next_gate_start_ns':next_anchor})
            read_gate_report[label] = {'active_read_calls':len(active_reads),'ticks':ticks,
                'compliant_read_calls_crossing_later_gate':crossing,
                'rule':'RX begin only, both ports during transmit; quiet/drain excluded. Host API times, not CAN wire times.'}
    if collection:
        findings.check(plan['admission_monotonic_ns'] <= summary['a']['start_ns'] == summary['b']['start_ns'] <= quiet_begin and
                       quiet_begin+2000000000 <= quiet_end <= ready['monotonic_ns'] <= epoch and
                       summary['a']['ready_ns'] == summary['b']['ready_ns'] == quiet_end and
                       transmit_end == epoch+plan['seconds']*1000000000 and
                       transmit_end+plan['drain_ms']*1000000 <= summary['a']['end_ns'] == summary['b']['end_ns'], 'quiet_ready_epoch_drain_window')
        findings.check(exit_record['started_monotonic_ns'] <= summary['a']['start_ns'] and
                       summary['a']['end_ns'] <= exit_record['exited_monotonic_ns'], 'wrapper_time_window')
        for label,scan,expected_ticks in (('A',a,plan['command_ticks']+(plan['feedback_ticks'] if mixed else 0)),('B',b,0 if mixed else plan['feedback_ticks'])):
            findings.check(scan['valid'] and len(scan['_init']) == 1 and scan['_init'][0]['result'] == 0 and
                           scan['_init'][0]['end_ns'] <= quiet_begin, 'initialization_before_quiet', detail=label)
            quiet_reads = [x for x in scan['_io'] if x['direction'] == 'RX' and x['begin_ns'] < epoch]
            findings.check(all(x['returned'] == 0 and quiet_begin <= x['begin_ns'] for x in quiet_reads), 'quiet_baseline_not_empty_or_missing', detail=label)
            findings.check(bool(quiet_reads) and quiet_reads[-1]['end_ns'] <= quiet_end, 'quiet_port_not_serviced', detail=label)
            findings.check(not [x for x in scan['_frames']['RX'] if x['begin_ns'] < epoch], 'rx_spills_from_quiet', detail=label)
            writes = scan['_frames']['TX']
            findings.check(all(epoch <= x['begin_ns'] <= x['end_ns'] < transmit_end for x in writes), 'write_outside_active_window', detail=label)
            findings.check(len(writes) == expected_ticks*2 and all(x['result'] == 0 for x in writes), 'planned_writes_not_completed', detail=label)
            hz = plan['command_hz'] if label == 'A' else plan['feedback_hz']
            if replay is None and type(hz) is int and hz > 0:
                phase_ns = feedback_phase_ms*1000000 if label == 'B' else 0
                for index,write in enumerate(writes):
                    deadline = epoch+phase_ns+(index//2)*(1000000000//hz)
                    findings.check(write['begin_ns'] >= deadline, 'write_before_nominal_tick',
                                   detail=f'{label}:tick={index//2},begin={write["begin_ns"]},deadline={deadline}')
            findings.check(all(x['result'] == 0 or x['direction'] == 'RX' and x['result'] == 1 for x in scan['_io']), 'transport_fault_in_complete_capture', detail=label)
        findings.check(summary['quiet_rx_a'] == summary['quiet_rx_b'] == 0, 'quiet_rx_summary')
    metadata_valid = not findings.errors
    acquisitions = collection and metadata_valid and a['valid'] and b['valid']
    return {'valid':acquisitions and content_match,'metadata_valid':metadata_valid,
            'acquisition_complete':acquisitions,'content_match':content_match,
            'collection_reported_complete':collection,'first_failure':findings.errors[0] if findings.errors else
                a_to_b['first_failure'] or b_to_a['first_failure'] or a['first_failure'] or b['first_failure'],
            'metadata_errors':findings.errors,'evidence_hashes':hashes,'boot_id':boot,'nonce':plan['nonce'],
            'feedback_order':feedback_order,
            'feedback_phase_ms':feedback_phase_ms,
            'replay_timing':replay_report,'feedback_sequence_audit':sequence_check,
            'rx_gate_ms':rx_gate_ms,'independent_read_gate':read_gate_report,
            'A':public(a),'B':public(b),'A_to_B':a_to_b,'B_to_A':b_to_a,
            'A_host_io_timing':timing(a),'B_host_io_timing':timing(b),
            'scheduler_checkpoints_self_reported':{label:{key:summary[label.lower()][key] for key in ('max_lateness_ns','min_send_interval_ns')} for label in ('A','B')},
            'window_ns':{key:summary[key] for key in ('quiet_begin_ns','quiet_end_ns','epoch_ns','transmit_end_ns')},
            'limitation':FIXED_LIMITATION,'can_wire_crc_available':False,'can_delivery_ack_verified':False}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory',type=Path)
    parser.add_argument('--manifest',type=Path)
    parser.add_argument('--replay-recipe',type=Path)
    parser.add_argument('--mixed-receive',action='store_true')
    parser.add_argument('--feedback-sequence',action='store_true')
    parser.add_argument('--output',type=Path,required=True)
    args = parser.parse_args()
    try:
        expected = replay_contract(read_json(args.replay_recipe)) if args.replay_recipe else None
        result = audit_directory(args.directory,args.manifest,expected,args.mixed_receive,args.feedback_sequence)
    except (OSError,ValueError,TypeError,KeyError,struct.error,OverflowError) as error:
        result = {'valid':False,'acquisition_complete':False,'content_match':False,
                  'first_failure':{'category':'evidence_schema_or_file','detail':str(error)}}
    with args.output.open('x',encoding='utf-8') as out:
        json.dump(result,out,indent=2,sort_keys=True);out.write('\n')
    print(json.dumps({'valid':result['valid'],'acquisition_complete':result['acquisition_complete'],
                      'content_match':result['content_match'],'report':str(args.output)}))
    return 0 if result['valid'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
