#!/usr/bin/env python3
"""Independent offline duplex audit fixtures. No device access."""
import copy
import hashlib
import json
from pathlib import Path
import struct
import tempfile
import unittest

import audit

GOLDENS = {
    0x668: bytes.fromhex('f7120e000bf8e2680600000c08000cd528000a000a'),
    0x669: bytes.fromhex('f7120e000ba8a6690600000c08000ea218000a000a'),
    0x2968: bytes.fromhex('f7120e000bb1b5682900000c0803490000ffff2a00'),
    0x2969: bytes.fromhex('f7120e000bb5a2692900000c0803be000000252a00')}


def envelope(data):
    head = b'\xf7\x12' + len(data).to_bytes(2, 'little')
    return head + bytes([audit.crc(head[1:], 0x8c, 255)]) + audit.crc(data, 0x8408, 65535).to_bytes(2, 'little') + data


def packet(ident, payload, flags=12):
    return envelope(struct.pack('<IBB', ident, flags, len(payload)) + payload)


def record(kind, data, time, result=0):
    requested = len(data) if kind == 1 else 1008
    returned = (len(data) if result == 0 else 0xffffffff) if kind == 1 else len(data)
    body = audit.META.pack(time, requested, returned) + data
    return audit.OUTER.pack(kind, 0, result, 0, len(body), time+1) + body


def capture(port, transmitted, received, fragments=False, tx_fault=False):
    pieces = [record(1, audit.INIT, 10)]
    time = 30
    for item in transmitted:
        pieces.append(record(1, item, time, 5 if tx_fault else 0));time += 3
    stream = b''.join(received)
    chunk = 1 if fragments else 1008
    for point in range(0, len(stream), chunk):
        pieces.append(record(2, stream[point:point+chunk], time));time += 3
    return audit.scan_capture(b''.join(pieces), port)


class CodecAndRecords(unittest.TestCase):
    def test_literal_crc_and_payload_goldens(self):
        for ident, payload in {**audit.COMMANDS, **audit.FEEDBACK}.items():
            golden = GOLDENS[ident]
            self.assertEqual(packet(ident, payload), golden)
            frame = audit.decode(golden)[0][0]
            self.assertEqual((frame['id'], frame['payload']), (ident, payload))
        self.assertEqual(audit.crc(audit.INIT[1:4], 0x8c, 255), audit.INIT[4])

    def test_all_ids_fragmentation_joined_rx_and_observed_prefix(self):
        prefixed = bytes.fromhex('f7121100ff0eb0682900682900000c0803440000fffa2c00')
        joined = envelope(GOLDENS[0x668][7:] + GOLDENS[0x669][7:] + struct.pack('<IBB', 0x123, 0, 3) + b'xyz')
        report = capture('B', [], [prefixed, joined], fragments=True)
        self.assertTrue(report['valid'], report['errors'])
        self.assertEqual([x['id'] for x in report['_frames']['RX']], [0x2968, 0x668, 0x669, 0x123])
        self.assertEqual(report['prefix3_packets'], 1)

    def test_header_crc_payload_crc_length_and_partial_stream(self):
        for byte, category in ((4, 'cdc_header_crc'), (5, 'cdc_payload_crc')):
            mutated = bytearray(GOLDENS[0x668]);mutated[byte] ^= 1
            with self.assertRaises(audit.Invalid) as caught:
                audit.decode(mutated)
            self.assertEqual(caught.exception.category, category)
        self.assertFalse(capture('B', [], [GOLDENS[0x668][:-1]])['valid'])
        self.assertFalse(capture('B', [], [b'\xf7\x12\xff\xff\x00\x00\x00'])['valid'])
        self.assertFalse(audit.scan_capture(record(1, audit.INIT, 10)[:-1], 'A')['valid'])

    def test_tx_separate_only_and_metadata_capacity(self):
        self.assertFalse(capture('A', [GOLDENS[0x668]+GOLDENS[0x669]], [])['valid'])
        too_large = record(1, audit.INIT, 10) + record(2, b'a'*1009, 30)
        self.assertFalse(audit.scan_capture(too_large, 'A')['valid'])
        nonzero_wb = record(1, audit.INIT, 10) + record(2, b'x', 30, 1)
        self.assertFalse(audit.scan_capture(nonzero_wb, 'A')['valid'])

    def test_per_record_prefix_shape_rejected(self):
        frame = GOLDENS[0x668][7:]
        prefixed_twice = envelope(frame[:3]+frame+frame[:3]+frame)
        with self.assertRaises(audit.Invalid):
            audit.decode(prefixed_twice)

    def test_duplex_golden_both_directions_and_arbitration(self):
        commands = [GOLDENS[x] for _ in range(4) for x in audit.COMMANDS]
        feedback = [GOLDENS[x] for _ in range(2) for x in audit.FEEDBACK]
        a = capture('A', commands, feedback, fragments=True)
        b = capture('B', feedback, [GOLDENS[0x669]]*4+[GOLDENS[0x668]]*4)
        self.assertTrue(a['valid'] and b['valid'])
        self.assertTrue(audit.compare_direction(a,b,audit.COMMANDS)['content_match'])
        self.assertTrue(audit.compare_direction(b,a,audit.FEEDBACK)['content_match'])

    def test_fixed_payload_order_and_balanced_loss_duplicate_not_observable(self):
        sender = capture('A', [GOLDENS[0x668]]*4, [])
        receiver = capture('B', [], [GOLDENS[0x668]]*4)
        report = audit.compare_direction(sender, receiver, audit.COMMANDS)
        self.assertTrue(report['content_match'])
        self.assertFalse(report['sequence_observable'])
        self.assertFalse(report['balanced_loss_duplicate_and_order_observable'])
        self.assertIn('synthetic', report['limitation'])

    def test_crc_valid_id_or_payload_mismatch_and_unknown_kept(self):
        sender = capture('A', [GOLDENS[0x668],GOLDENS[0x669]], [])
        for altered in ([GOLDENS[0x669]]*2,
                        [packet(0x668,audit.COMMANDS[0x669]),GOLDENS[0x669]],
                        [packet(0x123,b'abc',0),GOLDENS[0x669]]):
            receiver = capture('B', [], altered)
            self.assertTrue(receiver['valid'])
            self.assertFalse(audit.compare_direction(sender,receiver,audit.COMMANDS)['content_match'])
        self.assertEqual(receiver['_frames']['RX'][0]['id'],0x123)

    def test_counts_use_actual_successful_write_not_planned_or_attempted(self):
        failed_sender = capture('A',[GOLDENS[0x668]],[],tx_fault=True)
        empty_receiver = capture('B',[],[])
        report = audit.compare_direction(failed_sender,empty_receiver,audit.COMMANDS)
        self.assertTrue(report['content_match'])
        self.assertEqual(report['per_id']['0x668']['attempted_hostwrite_frames'],1)
        self.assertEqual(report['per_id']['0x668']['successful_hostwrite_frames'],0)
        self.assertFalse(audit.compare_direction(capture('A',[GOLDENS[0x668]],[]),empty_receiver,audit.COMMANDS)['content_match'])


BOOT = '12345678-1234-1234-1234-123456789abc'


def refresh_manifest(directory):
    files = {name:{'sha256':hashlib.sha256((directory/name).read_bytes()).hexdigest(),
                   'size':(directory/name).stat().st_size} for name in audit.ORIGINALS}
    collection = audit.read_json(directory/'summary.json')['collection_complete']
    # This independent fixture tests the auditor's manifest schema. It does
    # not exercise the field coordinator's factory or depend on ignored tmp/.
    manifest = {'files':files,'collection_complete':collection,
                'complete_file_set':set(files)==audit.ORIGINALS,'boot_id':BOOT}
    (directory/'evidence-manifest.json').write_text(json.dumps(manifest))


def bundle(directory, feedback_on=True, mismatch=False, feedback_order=None, feedback_phase_ms=None, rx_gate_ms=None):
    directory.mkdir()
    quiet_begin,quiet_end,epoch = 1001000000,3001000000,3003000000
    seconds,hz = (2,10) if rx_gate_ms is not None else (1,1)
    ticks,period = seconds*hz,1000000000//hz
    transmit_end,end = epoch+seconds*1000000000,epoch+(seconds+1)*1000000000+1000000
    commands = [GOLDENS[ident] for ident in audit.COMMANDS]
    feedback = [GOLDENS[ident] for ident in audit.FEEDBACK] if feedback_on else []
    if feedback_order == 'reverse':
        feedback.reverse()
    received_commands = [packet(0x668,audit.COMMANDS[0x669]),commands[1]] if mismatch else commands
    captures = {}
    for port,tx,rx in [('A',commands,feedback),('B',feedback,received_commands)]:
        pieces = [record(1,audit.INIT,1000000000),record(2,b'',quiet_begin+1000,1),
                  record(2,b'',quiet_end-1000,1)]
        phase = (feedback_phase_ms or 0)*1000000
        for tick in range(ticks):
            anchor = epoch+tick*period
            for lane,data in enumerate(tx):pieces.append(record(1,data,anchor+(phase if port=='B' else 0)+10000+lane*20000))
            rx_offset = max((phase if port=='A' else 0)+1000000,(rx_gate_ms or 0)*1000000+1000000)
            pieces.append(record(2,b''.join(rx),anchor+rx_offset,0 if rx else 1))
        pieces.append(record(2,b'',transmit_end+999000000,1))
        # The phase5 fixture observes A's command before B's first send.
        # Serialize each port's IO in actual chronological order.
        pieces.sort(key=lambda value:struct.unpack_from('<Q',value,16)[0])
        captures[port] = b''.join(pieces)
        (directory/f'capture-{port.lower()}.bin').write_bytes(captures[port])
    plan = {'schema':2,'role':'duplex','seconds':seconds,'command_hz':hz,'feedback_hz':hz if feedback_on else 0,
            'quiet_ms':2000,'drain_ms':1000,'capture_limit_bytes_per_port':1048576,'count_only':False,
            'nonce':42,'nonce_on_wire':False,'timestamp_domain':'absolute_host_steady_clock_ns',
            'can_timestamps':False,'poll_sleep_ms':1,'read_budget_per_port_per_pass':1,'read_capacity':1008,
            'packing':'separate','command_ticks':ticks,'feedback_ticks':ticks if feedback_on else 0,
            'a_tx_ids':list(audit.COMMANDS),'b_tx_ids':list(audit.FEEDBACK),
            'command_payloads_hex':[x.hex() for x in audit.COMMANDS.values()],
            'feedback_payloads_hex':[x.hex() for x in audit.FEEDBACK.values()],
            'synthetic_feedback':True,'feedback_provenance':'synthetic_fixture','init_hex':audit.INIT.hex(),
            'a_first_writes_hex':audit.COMMAND_PACKETS_HEX,
            'b_first_writes_hex':audit.FEEDBACK_PACKETS_HEX[::-1] if feedback_order == 'reverse' else audit.FEEDBACK_PACKETS_HEX,
            'run_requested':True,'motors_disconnected_declared':True,'clock':'steady_clock_monotonic',
            'admission_monotonic_ns':900000000,'boot_id':BOOT,'can_wire_timestamp_available':False}
    if feedback_order is not None:
        plan['feedback_order'] = feedback_order
    if feedback_phase_ms is not None:
        plan['feedback_phase_ms'] = feedback_phase_ms
    if rx_gate_ms is not None:
        plan.update(rx_gate_ms=rx_gate_ms,rx_gate_anchor='A_nominal_tick',rx_gate_scope='transmit_only_both_ports')
    for port in ('a','b'):
        identity = {'by_path':f'/dev/serial/by-path/{port}','tty':f'/dev/ttyACM{port}',
                    'usb_parent':f'/sys/usb/{port}','vendor':'caf1','product':'ffff'}
        plan[f'actual_identity_{port}']=identity;plan[f'port_{port}_by_path']=identity['by_path']
    scans = {port:audit.scan_capture(raw,port) for port,raw in captures.items()}
    summary = {'schema':2,'role':'duplex','complete':True,'collection_complete':True,'raw_complete':True,
               'content_match':not mismatch,'error':'','closed':True,'quiet_begin_ns':quiet_begin,'quiet_end_ns':quiet_end,
               'epoch_ns':epoch,'transmit_end_ns':transmit_end,'quiet_rx_a':0,'quiet_rx_b':0,
               'sent_a':[ticks,ticks],'sent_b':[ticks,ticks] if feedback_on else [0,0],
               'fixed_payload_sequence_observable':False,'can_delivery_verified':False,'motor_feedback_verified':False}
    for port in ('A','B'):
        scan=scans[port];received=scan['_frames']['RX'];ids={}
        for frame in received:ids[frame['id']]=ids.get(frame['id'],0)+1
        summary[port.lower()]={'schema':2,'role':port,'complete':True,'raw_complete':True,'count_only':False,
            'error':'','closed':True,'capture_bytes':len(captures[port]),'start_ns':990000000,
            'ready_ns':quiet_end,'end_ns':end,'first_frame_ns':received[0]['end_ns'] if received else 0,
            'last_frame_ns':received[-1]['end_ns'] if received else 0,
            'read_calls':scan['io_calls']['RX'],'write_calls':scan['io_calls']['TX'],
            'rx_bytes':scan['rx_bytes'],'rx_frames':len(received),'tx_frames':scan['successful_can_write_frames'],
            'max_lateness_ns':0,'min_send_interval_ns':0,'can_delivery_verified':False,
            'ids':[{'id':ident,'extended':True,'frames':n} for ident,n in ids.items()]}
    summary['content_a']=audit.content_counts(scans['A'],audit.FEEDBACK)
    summary['content_b']=audit.content_counts(scans['B'],audit.COMMANDS)
    supervisor={'complete':not mismatch,'collection_complete':True,'worker_exit_code':4 if mismatch else 0,
                'worker_reaped':True,'ports_close_verified':True,'forced_timeout':False,'export_valid':True,
                'capture_exported':True,'worker_pid':99}
    ready={'ready':True,'role':'duplex','boot_id':BOOT,'monotonic_ns':quiet_end+1000000,'worker_pid':99,
           'actual_identity_a':plan['actual_identity_a'],'actual_identity_b':plan['actual_identity_b']}
    exit_record={'exit_code':4 if mismatch else 0,'reaped':True,'forced_by_coordinator':False,
                 'started_monotonic_ns':980000000,'exited_monotonic_ns':end+1000000}
    for name,value in {'admission.json':plan,'plan.json':plan,'summary.json':summary,'supervisor.json':supervisor,
                       'ready.json':ready,'exit.json':exit_record}.items():(directory/name).write_text(json.dumps(value))
    (directory/'stdout.txt').write_text(json.dumps(summary));(directory/'stderr.txt').write_text('')
    refresh_manifest(directory)
    return directory


class EvidenceContract(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory(prefix='duplex-audit-');self.root=Path(self.temp.name)
    def tearDown(self):self.temp.cleanup()

    def test_complete_uni_and_duplex(self):
        for enabled in (False,True):
            result=audit.audit_directory(bundle(self.root/str(enabled),feedback_on=enabled))
            self.assertTrue(result['valid'],result['metadata_errors'])
            self.assertTrue(result['acquisition_complete'] and result['content_match'])
            self.assertEqual(result['A']['decoded_rx_frames'],2 if enabled else 0)

    def test_content_diff_exit4_preserves_complete_acquisition(self):
        result=audit.audit_directory(bundle(self.root/'bad',mismatch=True))
        self.assertFalse(result['valid'])
        self.assertTrue(result['metadata_valid'] and result['acquisition_complete'])
        self.assertFalse(result['content_match'])
        self.assertEqual(result['A_to_B']['payload_mismatch_count'],1)

    def test_explicit_forward_reverse_and_wrong_write_order(self):
        for order in ('forward','reverse'):
            result=audit.audit_directory(bundle(self.root/order,feedback_order=order))
            self.assertTrue(result['valid'],result['metadata_errors'])
            self.assertEqual(result['feedback_order'],order)
            self.assertEqual(result['B_to_A']['per_id']['0x2968']['successful_hostwrite_frames'],1)
            self.assertEqual(result['B_to_A']['per_id']['0x2969']['successful_hostwrite_frames'],1)
        path=bundle(self.root/'wrong-order',feedback_order='reverse')
        plan=audit.read_json(path/'plan.json');plan['feedback_order']='forward';plan['b_first_writes_hex']=audit.FEEDBACK_PACKETS_HEX
        for name in ('plan.json','admission.json'):(path/name).write_text(json.dumps(plan))
        refresh_manifest(path)
        result=audit.audit_directory(path)
        self.assertFalse(result['valid'])
        self.assertIn('feedback_write_order',[x['category'] for x in result['metadata_errors']])

    def test_phase5_and_early_B_write_rejected(self):
        result=audit.audit_directory(bundle(self.root/'phase5',feedback_phase_ms=5))
        self.assertTrue(result['valid'],result['metadata_errors'])
        self.assertEqual(result['feedback_phase_ms'],5)
        self.assertEqual(len(result['independent_read_gate']['B']['ticks'][0]['read_intervals_intersecting_AB_submission_span']),1)
        self.assertEqual(result['B_to_A']['per_id']['0x2968']['successful_hostwrite_frames'],1)
        path=bundle(self.root/'early-phase',feedback_phase_ms=0)
        plan=audit.read_json(path/'plan.json');plan['feedback_phase_ms']=5
        for name in ('plan.json','admission.json'):(path/name).write_text(json.dumps(plan))
        refresh_manifest(path)
        result=audit.audit_directory(path)
        self.assertFalse(result['valid'])
        self.assertIn('feedback_write_before_phase',[x['category'] for x in result['metadata_errors']])

    def test_rx_gate6_both_ports_and_phase5(self):
        for phase in (0,5):
            result=audit.audit_directory(bundle(self.root/f'gate6-phase{phase}',feedback_phase_ms=phase,rx_gate_ms=6))
            self.assertTrue(result['valid'],result['metadata_errors'])
            self.assertEqual(result['rx_gate_ms'],6)
            for port in ('A','B'):
                ticks=result['independent_read_gate'][port]['ticks']
                self.assertEqual(len(ticks),20)
                self.assertTrue(all(tick['first_read_begin_minus_A_nominal_tick_ns']==7000000 for tick in ticks))
                self.assertTrue(all(tick['read_intervals_intersecting_AB_submission_span']==[] for tick in ticks))
                self.assertEqual(result['independent_read_gate'][port]['active_read_calls'],20)
        legacy=audit.audit_directory(bundle(self.root/'legacy-gate0'))
        self.assertTrue(legacy['valid'])
        self.assertEqual(legacy['rx_gate_ms'],0)

    def test_forged_gate6_read_and_later_tick_early_write_rejected(self):
        for port in ('A','B'):
            path=bundle(self.root/f'early-read-{port}',feedback_phase_ms=5,rx_gate_ms=6)
            plan=audit.read_json(path/'plan.json');summary=audit.read_json(path/'summary.json')
            epoch=summary['epoch_ns'];capture=path/f'capture-{port.lower()}.bin'
            raw=capture.read_bytes();offset=0;pieces=[]
            while offset<len(raw):
                kind,_,result,_,size,end=audit.OUTER.unpack_from(raw,offset)
                begin,requested,returned=audit.META.unpack_from(raw,offset+16)
                data=raw[offset+32:offset+16+size]
                # Only a later tick violates the6ms begin rule. Correct phase5
                # writes and the remaining19 read windows remain untouched.
                if kind==2 and begin==epoch+10*100000000+7000000:
                    begin=epoch+10*100000000+5500000
                pieces.append(record(kind,data,begin,result));offset+=16+size
            pieces.sort(key=lambda value:struct.unpack_from('<Q',value,16)[0])
            capture.write_bytes(b''.join(pieces));refresh_manifest(path)
            result=audit.audit_directory(path)
            self.assertFalse(result['valid'])
            self.assertIn('rx_read_started_in_gate',[x['category'] for x in result['metadata_errors']])
        path=bundle(self.root/'later-early-B-write',feedback_phase_ms=5,rx_gate_ms=6)
        summary=audit.read_json(path/'summary.json');epoch=summary['epoch_ns']
        capture=path/'capture-b.bin';raw=capture.read_bytes();offset=0;pieces=[]
        while offset<len(raw):
            kind,_,result,_,size,end=audit.OUTER.unpack_from(raw,offset)
            begin,requested,returned=audit.META.unpack_from(raw,offset+16)
            data=raw[offset+32:offset+16+size]
            if kind==1 and epoch+10*100000000+5000000<=begin<epoch+10*100000000+6000000:
                begin-=100000
            pieces.append(record(kind,data,begin,result));offset+=16+size
        pieces.sort(key=lambda value:struct.unpack_from('<Q',value,16)[0])
        capture.write_bytes(b''.join(pieces));refresh_manifest(path)
        result=audit.audit_directory(path)
        self.assertFalse(result['valid'])
        self.assertIn('write_before_nominal_tick',[x['category'] for x in result['metadata_errors']])

    def test_compliant_begin_crossing_next_gate_is_reported_without_new_rejection(self):
        path=bundle(self.root/'crossing-call',feedback_phase_ms=0,rx_gate_ms=6)
        summary=audit.read_json(path/'summary.json');epoch=summary['epoch_ns']
        capture=path/'capture-b.bin';raw=capture.read_bytes();offset=0;pieces=[]
        while offset<len(raw):
            kind,_,result,_,size,end=audit.OUTER.unpack_from(raw,offset)
            begin,requested,returned=audit.META.unpack_from(raw,offset+16)
            data=raw[offset+32:offset+16+size]
            if kind==2 and begin==epoch+7000000:
                begin,end=epoch+99000000,epoch+101000000
            elif kind==1 and epoch+100000000<=begin<epoch+101000000:
                begin+=1020000;end=begin+1
            body=audit.META.pack(begin,requested,returned)+data
            pieces.append(audit.OUTER.pack(kind,0,result,0,len(body),end)+body);offset+=16+size
        pieces.sort(key=lambda value:struct.unpack_from('<Q',value,16)[0])
        capture.write_bytes(b''.join(pieces))
        summary['b']['first_frame_ns']=epoch+101000000
        for name in ('summary.json','stdout.txt'):(path/name).write_text(json.dumps(summary))
        refresh_manifest(path)
        result=audit.audit_directory(path)
        self.assertTrue(result['valid'],result['metadata_errors'])
        self.assertEqual(len(result['independent_read_gate']['B']['compliant_read_calls_crossing_later_gate']),1)
        overlap=result['independent_read_gate']['B']['ticks'][1]['read_intervals_intersecting_AB_submission_span']
        self.assertEqual(len(overlap),1)
        self.assertTrue(overlap[0]['started_before_A_first_write'])

    def test_hash_set_boot_close_and_summary_counts(self):
        for field,category in [('boot','boot_identity'),('count','board_frame_or_byte_count'),('close','close_reap_export_or_timeout')]:
            path=bundle(self.root/field)
            if field=='boot':
                document=audit.read_json(path/'ready.json');document['boot_id']='different';(path/'ready.json').write_text(json.dumps(document))
            elif field=='count':
                document=audit.read_json(path/'summary.json');document['a']['rx_bytes']+=1
                (path/'summary.json').write_text(json.dumps(document));(path/'stdout.txt').write_text(json.dumps(document))
            else:
                document=audit.read_json(path/'supervisor.json');document['ports_close_verified']=False;(path/'supervisor.json').write_text(json.dumps(document))
            refresh_manifest(path)
            result=audit.audit_directory(path)
            self.assertFalse(result['valid'])
            self.assertIn(category,[x['category'] for x in result['metadata_errors']])
        path=bundle(self.root/'manifest');document=audit.read_json(path/'evidence-manifest.json');document['files']={};(path/'evidence-manifest.json').write_text(json.dumps(document))
        self.assertFalse(audit.audit_directory(path)['valid'])

    def test_quiet_and_time_boundaries_are_not_discarded(self):
        path=bundle(self.root/'quiet')
        document=audit.read_json(path/'summary.json');document['quiet_begin_ns']+=1_000_000
        (path/'summary.json').write_text(json.dumps(document));(path/'stdout.txt').write_text(json.dumps(document));refresh_manifest(path)
        result=audit.audit_directory(path)
        self.assertFalse(result['valid'])
        self.assertIn('quiet_baseline_not_empty_or_missing',[x['category'] for x in result['metadata_errors']])


if __name__ == '__main__':
    unittest.main()
