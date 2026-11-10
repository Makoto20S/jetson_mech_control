"""Offline, stdlib-only analysis of bounded IMU/state timing captures.

Every interval is CLOCK_MONOTONIC nanoseconds on the same host. ROS stamps
are correlation keys only. Quantiles describe an observed sample, not WCET.
"""
from array import array
from collections import Counter, defaultdict
import argparse
import bisect
import csv
import hashlib
import heapq
import json
import math
from pathlib import Path

NS_MS = 1_000_000
EXPECTED_PACKET_MS = 10
OBSERVER_HEADER = ('event', 'index', 'source_stamp_ns', 'entry_ns', 'end_ns',
    'state_stamp_ns', 'imu1_stamp_ns', 'imu2_stamp_ns', 'imu1_valid', 'imu2_valid',
    'imu1_receive_age_ms', 'imu2_receive_age_ms')
PACKET_HEADER = ('packet_seq', 'device_ms', 'ros_stamp_ns',
    'frame_host_arrival_ns', 'last_serial_read_end_ns', 'decode_begin_ns',
    'decode_end_ns', 'imu1_publish_begin_ns', 'imu1_publish_end_ns',
    'imu2_publish_begin_ns', 'imu2_publish_end_ns', 'packet_done_ns')
STATE_HEADER = ('event', 'index', 'source_stamp_ns', 'entry_ns', 'expected_ns',
    'start_ns', 'snapshot_ns', 'end_ns', 'state_stamp_ns', 'imu1_stamp_ns',
    'imu2_stamp_ns', 'joints_valid', 'imu1_valid', 'imu2_valid',
    'imu1_receive_age_ns', 'imu2_receive_age_ns', 'imu1_source_age_ns',
    'imu2_source_age_ns', 'skipped_periods', 'slot_index', 'publish_begin_ns')
POLL_HEADER = ('poll_seq', 'begin_ns', 'end_ns', 'previous_begin_ns',
               'frames', 'packets', 'result')


class Audit:
    def __init__(self):
        self.errors = []
        self.warnings = []
        self.counts = Counter()

    def error(self, code, detail=''):
        self.counts[code] += 1
        if len(self.errors) < 80:
            self.errors.append({'code': code, 'detail': str(detail)})

    def warning(self, code, detail=''):
        self.counts[code] += 1
        if len(self.warnings) < 80:
            self.warnings.append({'code': code, 'detail': str(detail)})

    def output(self):
        return {'complete': not self.errors, 'errors': self.errors,
                'warnings': self.warnings, 'counts': dict(self.counts)}


def read_json(path, audit, required=True):
    try:
        value = json.loads(path.read_text(encoding='utf-8'))
        if not isinstance(value, dict):
            raise ValueError('JSON object required')
        return value
    except (OSError, ValueError) as exc:
        (audit.error if required else audit.warning)('json_unreadable', f'{path}: {exc}')
        return {}


def read_csv(path, required, audit):
    result = []
    try:
        with path.open(encoding='utf-8', newline='') as stream:
            reader = csv.DictReader(stream)
            absent = set(required) - set(reader.fieldnames or ())
            if absent:
                audit.error('csv_missing_columns', f'{path.name}: {sorted(absent)}')
                return result
            for number, values in enumerate(reader, 2):
                row = {'_row': number}
                bad = False
                for key, value in values.items():
                    if key is None:
                        audit.error('csv_extra_values', f'{path.name}:{number}')
                        bad = True
                        break
                    if key == 'event':
                        row[key] = value
                    elif value is None or value.strip() == '':
                        row[key] = None
                    else:
                        try:
                            if key.endswith('_ms') and key != 'device_ms':
                                converted = float(value)
                                row[key] = converted if math.isfinite(converted) else None
                            else:
                                row[key] = int(value)
                        except ValueError:
                            audit.error('csv_invalid_number', f'{path.name}:{number}:{key}={value}')
                            bad = True
                            break
                if not bad:
                    result.append(row)
    except OSError as exc:
        audit.error('csv_unreadable', f'{path}: {exc}')
    return result


def percentile(sorted_values, fraction):
    if not sorted_values:
        return None
    position = (len(sorted_values) - 1) * fraction
    lo = math.floor(position)
    hi = math.ceil(position)
    return sorted_values[lo] + (sorted_values[hi] - sorted_values[lo]) * (position - lo)


class Series:
    def __init__(self):
        self.values = array('q')
        self.maximum = None

    def add(self, value_ns, anchor_ns, context=None):
        value_ns = int(value_ns)
        self.values.append(value_ns)
        if self.maximum is None or value_ns > self.maximum['value_ns']:
            self.maximum = {'value_ns': value_ns, 'anchor_ns': anchor_ns,
                            'context': context or {}}

    def output(self):
        values = sorted(self.values)
        maximum = dict(self.maximum) if self.maximum else None
        if maximum:
            maximum['value_ms'] = maximum['value_ns'] / NS_MS
        return {'count': len(values),
                'min_ms': values[0] / NS_MS if values else None,
                'p50_ms': percentile(values, .50) / NS_MS if values else None,
                'p95_ms': percentile(values, .95) / NS_MS if values else None,
                'p99_ms': percentile(values, .99) / NS_MS if values else None,
                'p99_9_ms': percentile(values, .999) / NS_MS if values else None,
                'max_ms': values[-1] / NS_MS if values else None,
                'negative_count': sum(v < 0 for v in values),
                'over_10ms_count': sum(v > 10 * NS_MS for v in values),
                'over_20ms_count': sum(v > 20 * NS_MS for v in values),
                'over_40ms_count': sum(v > 40 * NS_MS for v in values),
                'max_event': maximum}


def key_index(rows, keys, label, audit):
    indexed = {}
    ambiguous = set()
    for row in rows:
        key = tuple(row.get(field) for field in keys)
        if any(value is None or (field.endswith('stamp_ns') and value <= 0)
               for field, value in zip(keys, key)):
            audit.error('invalid_join_key', f'{label}:{row.get("_row")}: {key}')
            continue
        if key in indexed or key in ambiguous:
            indexed.pop(key, None)
            ambiguous.add(key)
            audit.error('duplicate_join_key', f'{label}: {key}')
        else:
            indexed[key] = row
    return indexed


def validate_event_rows(rows, label, audit):
    valid=[]
    for row in rows:
        event=row.get('event')
        fields=['entry_ns','end_ns']
        if event=='imu':
            fields += ['index','source_stamp_ns']
            if row.get('index') not in (1,2):
                audit.error('invalid_imu_index',f'{label}:{row.get("_row")}')
                continue
        elif event=='state':
            fields += ['state_stamp_ns','imu1_valid','imu2_valid']
            if label=='state':
                fields += ['start_ns','snapshot_ns','publish_begin_ns','expected_ns','skipped_periods','slot_index']
            for index in (1,2):
                if row.get(f'imu{index}_valid')==1:
                    fields += [f'imu{index}_stamp_ns']
        elif event=='gc':
            fields += ['index']
        else:
            audit.error('unknown_event',f'{label}:{row.get("_row")}:{event}')
            continue
        absent=[field for field in fields if row.get(field) is None]
        if absent:
            audit.error('missing_event_fields',f'{label}:{row.get("_row")}:{absent}')
            continue
        if row['end_ns']<row['entry_ns']:
            audit.error('event_clock_regression',f'{label}:{row.get("_row")}')
            continue
        valid.append(row)
    return valid


class Analysis:
    def __init__(self, metadata, audit):
        self.meta = metadata
        self.audit = audit
        self.start = metadata.get('start_mono_ns', 0)
        self.lo = metadata.get('warmup_end_ns', 0)
        self.hi = metadata.get('measurement_end_ns', 0)
        self.stop = metadata.get('stop_requested_ns', self.hi)
        self.end = metadata.get('end_mono_ns', self.stop)
        if not all(isinstance(x, int) for x in (self.start, self.lo, self.hi, self.stop, self.end)):
            raise ValueError('run timestamps must be integer monotonic ns')
        if not self.start <= self.lo < self.hi <= self.stop <= self.end:
            raise ValueError('invalid warmup/measurement/stop/end window ordering')
        self.series = defaultdict(Series)
        self.gap_candidates = []
        self.gap_count = Counter()
        self.gc = {}
        self.join_counts = Counter()
        self.delivery = {}
        self.max_events = []

    def inside(self, ns):
        return isinstance(ns, int) and self.lo <= ns < self.hi

    def add(self, name, value, anchor, context=None, allow_negative=False):
        if value is None or anchor is None:
            self.audit.error('metric_missing_time', name)
            return
        if value < 0 and not allow_negative:
            self.audit.error('negative_interval', f'{name}:{value}')
            return
        self.series[name].add(value, anchor, context)

    def interval(self, name, row, begin, end, anchor=None):
        start = row.get(begin)
        finish = row.get(end)
        if start is None or finish is None:
            self.audit.error('metric_missing_time', f'{name}:row {row.get("_row")}')
            return
        self.add(name, finish-start, anchor if anchor is not None else start,
                 {'row': row.get('_row'), 'begin_ns': start, 'end_ns': finish})

    def phase_counts(self, rows, field):
        counts = Counter()
        for row in rows:
            stamp = row.get(field)
            if stamp is None:
                counts['missing_time'] += 1
            elif stamp < self.start:
                counts['before_run_start'] += 1
            elif stamp < self.lo:
                counts['warmup'] += 1
            elif stamp < self.hi:
                counts['measurement'] += 1
            elif stamp <= self.end:
                counts['tail'] += 1
            else:
                counts['after_run_end'] += 1
        return dict(counts)

    def gaps(self, label, rows, time_field, source_field=None):
        selected = [r for r in rows if self.inside(r.get(time_field))]
        for before, after in zip(selected, selected[1:]):
            left, right = before[time_field], after[time_field]
            gap = right-left
            context = {'previous_row': before.get('_row'), 'row': after.get('_row'),
                       'begin_ns': left, 'end_ns': right}
            if source_field:
                first, second = before.get(source_field), after.get(source_field)
                if first is not None and second is not None:
                    context['source_stamp_gap_ms'] = (second-first)/NS_MS
            self.add(label+'.gap', gap, right, context)
            if gap > 20*NS_MS:
                self.gap_count[label+'.over20ms'] += 1
            if gap > 40*NS_MS:
                self.gap_count[label+'.over40ms'] += 1
            if label.startswith('observer.') and gap > 20*NS_MS:
                item = {'stream': label, 'gap_ns': gap, **context,
                        'before': before, 'after': after}
                self.gap_candidates.append(item)
        return selected

    def gc_events(self, name, rows):
        events = []
        for row in rows:
            if row.get('event') != 'gc':
                continue
            begin, end = row.get('entry_ns'), row.get('end_ns')
            if begin is None or end is None or end < begin:
                self.audit.error('invalid_gc_interval', f'{name}: {row.get("_row")}')
                continue
            events.append({'begin_ns': begin, 'end_ns': end,
                           'generation': row.get('index'), 'row': row.get('_row')})
            if self.inside(begin):
                self.add(name+'.gc.duration', end-begin, begin, events[-1])
        self.gc[name] = sorted(events, key=lambda e: e['begin_ns'])

    def gc_overlap(self, name, begin, end):
        events = self.gc.get(name, ())
        starts = [event['begin_ns'] for event in events]
        # GC intervals are non-overlapping within one process callback stream.
        pos = max(0, bisect.bisect_left(starts, begin)-1)
        matches = []
        overlap = 0
        for event in events[pos:]:
            if event['begin_ns'] >= end:
                break
            shared = max(0, min(end,event['end_ns'])-max(begin,event['begin_ns']))
            if shared:
                overlap += shared
                matches.append({**event, 'overlap_ms': shared/NS_MS})
        return {'count': len(matches), 'overlap_ms': overlap/NS_MS,
                'fraction_of_gap': overlap/(end-begin) if end>begin else None,
                'events': matches[:10]}

    def delivery_progression(self,label,rows,lookup,sequence_field):
        sequences=[]
        missing_matches=0
        for row in rows:
            if not self.inside(row.get('entry_ns')):continue
            source=lookup(row)
            if source is None:
                missing_matches+=1
            else:sequences.append(source[sequence_field])
        deltas=[right-left for left,right in zip(sequences,sequences[1:])]
        self.delivery[label]={'matched_callbacks':len(sequences),
            'unmatched_callbacks':missing_matches,
            'sequence_delta_counts':dict(Counter(deltas)),
            'duplicate_source_events':sum(delta==0 for delta in deltas),
            'reordered_source_events':sum(delta<0 for delta in deltas),
            'unobserved_source_events_between_adjacent_callbacks':sum(max(0,delta-1) for delta in deltas),
            'boundary_policy':'Only source events between matched callbacks inside measurement; startup/end absence is not counted.',
            'interpretation':'Host publication/callback continuity, not physical CAN packet-loss rate.'}


def check_meta_counts(audit, metadata, observed_key, stored_key, overflow_key, actual, label):
    for key in (observed_key, stored_key, overflow_key):
        if not isinstance(metadata.get(key), int):
            audit.error('missing_or_invalid_count', f'{label}.{key}')
    stored, overflow = metadata.get(stored_key), metadata.get(overflow_key)
    if isinstance(stored,int) and stored != actual:
        audit.error('row_count_mismatch', f'{label}: stored={stored}, CSV={actual}')
    if isinstance(overflow,int) and overflow:
        audit.error('trace_overflow', f'{label}: {overflow}')
    observed = metadata.get(observed_key)
    if all(isinstance(v,int) for v in (observed,stored,overflow)) and observed != stored+overflow:
        audit.error('observed_stored_mismatch', f'{label}: {observed}!={stored}+{overflow}')


def timer_reference_metadata(metadata):
    """Keep nominal grid diagnostics separate from timer-API anchor evidence."""
    available=metadata.get('deadline_reference_available') is True
    origin=metadata.get('phase_origin')
    uncertainty=metadata.get('anchor_uncertainty_ns')
    error=metadata.get('anchor_error')
    before=metadata.get('anchor_mono_before_ns')
    after=metadata.get('anchor_mono_after_ns')
    until=metadata.get('anchor_until_ns')
    phase=metadata.get('first_phase_ns')
    bracket_consistent=(all(type(value) is int for value in (before,after,until,phase,uncertainty))
        and after>=before and uncertainty>=after-before
        and phase==(before+after)//2+until)
    verified_shape=(available and isinstance(origin,str)
        and origin.startswith('ros_timer_time_until_next_call_bracket')
        and bracket_consistent and not error)
    if verified_shape:
        definition=('Nominal end_ns > expected_ns + period_ns on the ROS timer API '
            'bracket-anchored grid. Anchor uncertainty must be retained when interpreting '
            'near-boundary results; this is not a kernel scheduling trace or a hard-real-time guarantee.')
        scope='ros_timer_api_grid_with_reported_uncertainty'
    else:
        definition=('Diagnostic only: end_ns > expected_ns + period_ns on an unverified '
            'or first-callback-estimated phase grid. Zero exceedances do not establish '
            'deadline acceptance; no valid timer API reference is available.')
        scope='diagnostic_only_unverified_phase_grid'
    return {'phase_origin':origin,'deadline_reference_available':available,
        'deadline_reference_valid':bool(verified_shape),
        'anchor_uncertainty_ns':uncertainty,'anchor_error':error,
        'anchor_mono_before_ns':metadata.get('anchor_mono_before_ns'),
        'anchor_mono_after_ns':metadata.get('anchor_mono_after_ns'),
        'anchor_until_ns':metadata.get('anchor_until_ns'),
        'deadline_evidence_scope':scope,'deadline_definition':definition}


def analyze_run(directory):
    directory = Path(directory)
    audit = Audit()
    metadata = read_json(directory/'run.json', audit)
    analysis = Analysis(metadata, audit)
    if metadata.get('completed') is not True:
        audit.error('run_incomplete_or_unknown',metadata.get('completed'))
    observer = read_csv(directory/'observer.csv', OBSERVER_HEADER, audit)
    if metadata.get('observer_rows') != len(observer):
        audit.error('observer_row_count_mismatch', f'{metadata.get("observer_rows")} != {len(observer)}')
    observer=validate_event_rows(observer,'observer',audit)
    analysis.gc_events('observer', observer)
    if metadata.get('observer_dropped', None) != 0:
        audit.error('observer_rows_dropped_or_unknown', metadata.get('observer_dropped'))
    observer_streams = {}
    observer_counts = {}
    for event,index in (('imu',1),('imu',2),('state',0)):
        label = 'observer.'+(f'imu{index}' if event=='imu' else 'state')
        values = [r for r in observer if r.get('event')==event and (event=='state' or r.get('index')==index)]
        observer_streams[label] = values
        selected = analysis.gaps(label, values, 'entry_ns', 'source_stamp_ns' if event=='imu' else 'state_stamp_ns')
        observer_counts[label] = analysis.phase_counts(values,'entry_ns')
        if not selected:
            audit.error('empty_measurement_stream', label)
        for row in selected:
            analysis.interval(label+'.callback_duration',row,'entry_ns','end_ns')
            if event=='state':
                for imu in (1,2):
                    age = row.get(f'imu{imu}_receive_age_ms')
                    if age is not None and row.get(f'imu{imu}_valid'):
                        analysis.add(f'observer.state.imu{imu}.reported_receive_age',round(age*NS_MS),row['entry_ns'])
    trace = metadata.get('trace_enabled')
    if not isinstance(trace,bool):
        audit.error('trace_enabled_missing_or_invalid')
    phases = {'observer': observer_counts}
    packets=[]; states=[]; polls=[]; callbacks={}; packet_map={}; state_map={}
    timer = None
    device = None
    poll_summary = None
    bridge_meta={}; state_meta={}
    if trace:
        bridge_dir = directory/'bridge'
        bridge_meta=read_json(bridge_dir/'bridge_meta.json',audit)
        packets=read_csv(bridge_dir/'bridge_packets.csv',PACKET_HEADER,audit)
        polls=read_csv(bridge_dir/'bridge_polls.csv',POLL_HEADER,audit)
        states=read_csv(directory/'state.csv',STATE_HEADER,audit)
        state_meta=read_json(directory/'state.csv.meta.json',audit)
        if state_meta.get('completed') is not True:
            audit.error('state_flush_incomplete_or_unknown',state_meta.get('completed'))
        check_meta_counts(audit,bridge_meta,'packet_observed','packet_stored','packet_overflow',len(packets),'bridge.packets')
        # Anomaly-only polling intentionally does not store ordinary callbacks.
        poll_observed_key='poll_anomalies' if bridge_meta.get('poll_mode')=='anomaly' else 'poll_observed'
        check_meta_counts(audit,bridge_meta,poll_observed_key,'poll_stored','poll_overflow',len(polls),'bridge.polls')
        if state_meta.get('rows_written') != len(states):
            audit.error('state_row_count_mismatch',f'{state_meta.get("rows_written")} != {len(states)}')
        if state_meta.get('rows_dropped') != 0 or state_meta.get('overflow') not in (0,False):
            audit.error('state_trace_overflow_or_unknown',state_meta)
        if bridge_meta.get('clock_errors') != 0:
            audit.error('bridge_clock_errors_or_unknown',bridge_meta.get('clock_errors'))
        if bridge_meta.get('serial_read_clock_errors') != 0:
            audit.error('serial_read_clock_errors_or_unknown',bridge_meta.get('serial_read_clock_errors'))
        if bridge_meta.get('completed') is not True:
            audit.error('bridge_flush_incomplete_or_unknown',bridge_meta.get('completed'))
        if state_meta.get('clock_regressions') != 0:
            audit.error('state_clock_regressions_or_unknown',state_meta.get('clock_regressions'))
        if bridge_meta.get('clock') != 'CLOCK_MONOTONIC':
            audit.error('unexpected_bridge_clock',bridge_meta.get('clock'))
        states=validate_event_rows(states,'state',audit)
        complete_packets=[]
        for row in packets:
            absent=[field for field in PACKET_HEADER if row.get(field) is None]
            if absent:
                audit.error('missing_packet_fields',f'row {row.get("_row")}: {absent}')
            else:complete_packets.append(row)
        packets=complete_packets
        analysis.gc_events('state',states)
        packet_map=key_index(packets,('ros_stamp_ns',),'bridge',audit)
        callback_rows=[r for r in states if r.get('event')=='imu']
        callbacks=key_index(callback_rows,('index','source_stamp_ns'),'state.imu',audit)
        publications=[r for r in states if r.get('event')=='state']
        for seq,row in enumerate(publications,1):row['_publication_seq']=seq
        state_map=key_index(publications,('state_stamp_ns',),'state.publication',audit)
        selected_packets=analysis.gaps('bridge.packet',packets,'last_serial_read_end_ns','ros_stamp_ns')
        phases['bridge_packets']=analysis.phase_counts(packets,'last_serial_read_end_ns')
        seq_gaps=Counter(); device_deltas=Counter(); bad_device=0
        for before,after in zip(selected_packets,selected_packets[1:]):
            if before.get('packet_seq') is None or after.get('packet_seq') is None:
                audit.error('missing_packet_sequence')
            else:
                seq_gaps[after['packet_seq']-before['packet_seq']]+=1
            if before.get('device_ms') is None or after.get('device_ms') is None:
                audit.error('missing_device_timestamp')
            else:
                delta=(after['device_ms']-before['device_ms']) & 0xffffffff
                device_deltas[delta]+=1
                bad_device += delta != EXPECTED_PACKET_MS
        if any(value != 1 for value in seq_gaps):
            audit.error('packet_sequence_discontinuity',dict(seq_gaps))
        device={'expected_increment_ms':EXPECTED_PACKET_MS,
                'increment_histogram_ms':dict(device_deltas),
                'non_10ms_count':bad_device,'packet_sequence_delta_counts':dict(seq_gaps),
                'interpretation':'Device timestamp continuity; not exact wire loss or independent IMU sample freshness.'}
        for row in selected_packets:
            anchor=row['last_serial_read_end_ns']
            for name,begin,end in (
                ('serial_read_to_frame_decode','last_serial_read_end_ns','frame_host_arrival_ns'),
                ('application_decode','decode_begin_ns','decode_end_ns'),
                ('serial_read_to_packet_done','last_serial_read_end_ns','packet_done_ns')):
                analysis.interval('bridge.'+name,row,begin,end,anchor)
            for imu in (1,2):
                analysis.interval(f'bridge.imu{imu}.publish_call',row,f'imu{imu}_publish_begin_ns',f'imu{imu}_publish_end_ns',anchor)
                analysis.interval(f'bridge.imu{imu}.serial_read_to_publish_begin',row,'last_serial_read_end_ns',f'imu{imu}_publish_begin_ns',anchor)
        for imu in (1,2):
            rows=[r for r in callback_rows if r.get('index')==imu]
            analysis.delivery_progression(f'bridge_to_state_imu{imu}',rows,
                lambda row:packet_map.get((row.get('source_stamp_ns'),)),'packet_seq')
            selected=analysis.gaps(f'state.imu{imu}',rows,'entry_ns','source_stamp_ns')
            phases[f'state_imu{imu}']=analysis.phase_counts(rows,'entry_ns')
            for row in selected:
                analysis.interval(f'state.imu{imu}.callback_duration',row,'entry_ns','end_ns')
                packet=packet_map.get((row.get('source_stamp_ns'),))
                if packet is None:
                    analysis.join_counts[f'state_imu{imu}_to_bridge_unmatched']+=1
                    continue
                analysis.join_counts[f'state_imu{imu}_to_bridge_matched']+=1
                analysis.add(f'bridge_to_state.imu{imu}.publish_begin_to_callback',
                    row['entry_ns']-packet[f'imu{imu}_publish_begin_ns'],row['entry_ns'],
                    {'state_row':row['_row'],'packet_seq':packet['packet_seq']})
        selected_states=analysis.gaps('state.timer',publications,'start_ns','state_stamp_ns')
        phases['state_publication']=analysis.phase_counts(publications,'start_ns')
        period=state_meta.get('period_ns')
        if not isinstance(period,int) or period<=0:
            audit.error('missing_state_period',period)
            period=10*NS_MS
        reference=timer_reference_metadata(state_meta)
        uncertainty=reference['anchor_uncertainty_ns']
        bound_available=type(uncertainty) is int and uncertainty>=0
        phase_samples=0; negative_finish=0
        early_start=0 if bound_available else None
        early_finish=0 if bound_available else None
        missed=0; skipped=0; consecutive=0; max_consecutive=0; valid_count=Counter(); reused=Counter(); prev_stamp={}
        for pos,row in enumerate(selected_states):
            start=row.get('start_ns'); expected=row.get('expected_ns'); end=row.get('end_ns')
            if any(v is None for v in (start,expected,end,row.get('skipped_periods'),row.get('slot_index'))):
                audit.error('missing_timer_time',row.get('_row'))
                continue
            phase_samples+=1
            negative_finish+=end<expected
            if bound_available:
                early_start+=start<expected-uncertainty
                early_finish+=end<expected-uncertainty
            if pos:
                prev=selected_states[pos-1]
                if prev.get('slot_index') is not None and row['slot_index']-prev['slot_index'] != row['skipped_periods']+1:
                    audit.error('slot_skip_inconsistent',row.get('_row'))
                if prev.get('expected_ns') is not None and row['expected_ns']-prev['expected_ns'] != (row['skipped_periods']+1)*period:
                    audit.error('phase_grid_inconsistent',row.get('_row'))
            first_skipped_expected=expected-row['skipped_periods']*period
            lower=max(0,math.ceil((analysis.lo-first_skipped_expected)/period))
            upper=min(row['skipped_periods'],math.ceil((analysis.hi-first_skipped_expected)/period))
            row_skips=max(0,upper-lower)
            # Count only skipped slots whose estimated expected time is in this window.
            skipped += row_skips
            consecutive += row_skips
            max_consecutive=max(max_consecutive,consecutive)
            is_miss=end>expected+period
            missed+=is_miss
            consecutive=consecutive+1 if is_miss else 0
            max_consecutive=max(max_consecutive,consecutive)
            analysis.add('state.timer.phase_start_error',start-expected,start,allow_negative=True)
            analysis.add('state.timer.phase_start_lateness',max(0,start-expected),start)
            analysis.add('state.timer.phase_release_to_completion',end-expected,start,allow_negative=True)
            analysis.interval('state.timer.callback_duration',row,'start_ns','end_ns',start)
            analysis.interval('state.publish_call',row,'publish_begin_ns','end_ns',start)
            for imu in (1,2):
                stamp=row.get(f'imu{imu}_stamp_ns')
                valid=row.get(f'imu{imu}_valid')==1
                valid_count[f'imu{imu}_valid' if valid else f'imu{imu}_invalid']+=1
                if stamp and stamp==prev_stamp.get(imu): reused[f'imu{imu}']+=1
                prev_stamp[imu]=stamp
                if valid:
                    for age in ('receive','source'):
                        value=row.get(f'imu{imu}_{age}_age_ns')
                        if value is not None:
                            analysis.add(f'state.imu{imu}.{age}_age_at_snapshot',value,start,allow_negative=(age=='source'))
                    callback=callbacks.get((imu,stamp))
                    packet=packet_map.get((stamp,))
                    if callback is None or packet is None:
                        analysis.join_counts[f'state_publish_imu{imu}_unmatched']+=1
                        continue
                    analysis.join_counts[f'state_publish_imu{imu}_matched']+=1
                    analysis.add(f'state.imu{imu}.callback_end_to_snapshot',row['snapshot_ns']-callback['end_ns'],start)
                    analysis.add(f'state.imu{imu}.host_read_age_at_snapshot',row['snapshot_ns']-packet['last_serial_read_end_ns'],start)
        runtime_reference_valid=bool(reference['deadline_reference_valid'] and phase_samples
                                     and early_start==0 and early_finish==0)
        if reference['deadline_reference_valid'] and (early_start or early_finish):
            reference['deadline_evidence_scope']='diagnostic_only_runtime_phase_conflict'
            reference['deadline_definition']=(
                'Diagnostic only: nominal end_ns > expected_ns + period_ns is retained, '
                'but observed callbacks start or finish earlier than their assigned expected '
                'release beyond the anchor uncertainty. The runtime phase grid is not a '
                'validated timer release reference; these counts cannot establish actual '
                'deadline failure or acceptance. This is not a trace-completeness failure.')
        timer={'period_ns':period,**reference,
               'runtime_phase_reference_valid':runtime_reference_valid,
               'deadline_acceptance_available':runtime_reference_valid,
               'runtime_phase_samples_evaluated':phase_samples,
               'starts_before_expected_beyond_anchor_uncertainty':early_start,
               'negative_release_to_completion_count':negative_finish,
               'finishes_before_expected_beyond_anchor_uncertainty':early_finish,
               'runtime_reference_definition':'Requires a consistent startup API anchor, at least one measured callback, and no observed early-start or early-finish contradiction beyond its uncertainty. Deadline acceptance availability additionally requires complete trace integrity; it is not an acceptance result or a hard-real-time guarantee.',
               'executed_callbacks':len(selected_states),'executed_deadline_exceedances':missed,
               'executed_deadline_exceedance_fraction':missed/len(selected_states) if selected_states else None,
               'skipped_phase_grid_slots':skipped,
               'max_consecutive_bad_phase_grid_slots':max_consecutive,
               'validity_counts':dict(valid_count),'reused_input_stamp_counts':dict(reused),
               'boundary_policy':'Callbacks start inside measurement; skipped slots counted only if their estimated expected time is also inside measurement.'}
        poll_summary={'mode':bridge_meta.get('poll_mode'),
                      'observed_all_phases':bridge_meta.get('poll_observed'),
                      'stored_anomalies_in_measurement':sum(analysis.inside(r.get('begin_ns')) for r in polls),
                      'max_gap_all_phases_ms':bridge_meta.get('max_poll_gap_ns',0)/NS_MS,
                      'max_duration_all_phases_ms':bridge_meta.get('max_poll_duration_ns',0)/NS_MS,
                      'quantiles_available':bridge_meta.get('poll_mode')=='all'}
        if bridge_meta.get('poll_mode')=='all':
            analysis.gaps('bridge.poll',polls,'begin_ns')
        for row in polls:
            if analysis.inside(row.get('begin_ns')):
                analysis.interval('bridge.poll.'+('duration' if bridge_meta.get('poll_mode')=='all' else 'recorded_anomaly_duration'),row,'begin_ns','end_ns')
        for label,values in observer_streams.items():
            analysis.delivery_progression(label,values,
                (lambda row:packet_map.get((row.get('source_stamp_ns'),))) if label!='observer.state'
                else (lambda row:state_map.get((row.get('state_stamp_ns'),))),
                'packet_seq' if label!='observer.state' else '_publication_seq')
            for row in values:
                if not analysis.inside(row.get('entry_ns')):continue
                if row['event']=='imu':
                    imu=row['index']; packet=packet_map.get((row['source_stamp_ns'],))
                    if packet is None:
                        analysis.join_counts[label+'_bridge_unmatched']+=1;continue
                    analysis.join_counts[label+'_bridge_matched']+=1
                    analysis.add(f'bridge_to_observer.imu{imu}.publish_begin_to_callback',row['entry_ns']-packet[f'imu{imu}_publish_begin_ns'],row['entry_ns'])
                    analysis.add(f'bridge_to_observer.imu{imu}.host_read_to_callback',row['entry_ns']-packet['last_serial_read_end_ns'],row['entry_ns'])
                else:
                    publication=state_map.get((row['state_stamp_ns'],))
                    if publication is None:
                        analysis.join_counts['observer_state_publication_unmatched']+=1;continue
                    analysis.join_counts['observer_state_publication_matched']+=1
                    analysis.add('state_to_observer.publish_begin_to_callback',row['entry_ns']-publication['publish_begin_ns'],row['entry_ns'])
                    for imu in (1,2):
                        if row.get(f'imu{imu}_valid')!=1:continue
                        packet=packet_map.get((row.get(f'imu{imu}_stamp_ns'),))
                        if packet is None:
                            analysis.join_counts[f'observer_state_imu{imu}_packet_unmatched']+=1;continue
                        analysis.join_counts[f'observer_state_imu{imu}_packet_matched']+=1
                        analysis.add(f'pipeline.imu{imu}.host_read_to_state_observer',row['entry_ns']-packet['last_serial_read_end_ns'],row['entry_ns'])
        for name,count in analysis.join_counts.items():
            if name.endswith('_unmatched') and count:
                audit.error('unmatched_trace_events',f'{name}: {count}')
    return finish_analysis(directory,analysis,observer,packets,states,polls,packet_map,
                           callbacks,state_map,phases,timer,device,poll_summary,bridge_meta,state_meta)


def finish_analysis(directory,a,observer,packets,states,polls,packet_map,callbacks,state_map,
                    phases,timer,device,poll_summary,bridge_meta,state_meta):
    ordered_packets=sorted(packets,key=lambda row:row['packet_seq'])
    packet_sequence=[row['packet_seq'] for row in ordered_packets]
    ordered_states=sorted(state_map.values(),key=lambda row:row['_publication_seq'])
    state_sequence=[row['_publication_seq'] for row in ordered_states]
    callback_sequence={}
    for imu in (1,2):
        items=[]
        for (index,stamp),row in callbacks.items():
            packet=packet_map.get((stamp,))
            if index==imu and packet:
                items.append((packet['packet_seq'],row))
        items.sort(key=lambda pair:pair[0])
        callback_sequence[imu]=([pair[0] for pair in items],[pair[1] for pair in items])

    def production_window(sequences,rows,first,last,field):
        """Inspect intermediate source events, not only observed endpoints.

        An observer may omit messages while source publication stays regular;
        an endpoint time span alone cannot localize that gap upstream.
        """
        lo=bisect.bisect_left(sequences,first)
        hi=bisect.bisect_right(sequences,last)
        events=rows[lo:hi]
        times=[row[field] for row in events]
        gaps=[right-left for left,right in zip(times,times[1:])]
        return {'events':len(events),
                'endpoint_span_ms':(times[-1]-times[0])/NS_MS if len(times)>1 else None,
                'max_adjacent_gap_ms':max(gaps)/NS_MS if gaps else None}

    gap_details=[]; classifications=Counter(); gc_overlap_counts=Counter()
    for item in a.gap_candidates:
        before,after=item['before'],item['after']
        detail={key:value for key,value in item.items() if key not in ('before','after','gap_ns')}
        detail['gap_ms']=item['gap_ns']/NS_MS
        detail['gc']={name:a.gc_overlap(name,item['begin_ns'],item['end_ns']) for name in a.gc}
        for name,value in detail['gc'].items():
            if value['count']:gc_overlap_counts[name+'.observer_gaps_overlapping_gc']+=1
        evidence={}
        classification='observer_gap_only_no_internal_trace'
        if a.meta.get('trace_enabled'):
            if after['event']=='imu':
                imu=after['index']
                p0=packet_map.get((before['source_stamp_ns'],));p1=packet_map.get((after['source_stamp_ns'],))
                c0=callbacks.get((imu,before['source_stamp_ns']));c1=callbacks.get((imu,after['source_stamp_ns']))
                if p0 and p1:
                    evidence['bridge_host_read_window']=production_window(packet_sequence,ordered_packets,p0['packet_seq'],p1['packet_seq'],'last_serial_read_end_ns')
                    evidence['bridge_publish_window']=production_window(packet_sequence,ordered_packets,p0['packet_seq'],p1['packet_seq'],f'imu{imu}_publish_begin_ns')
                    callback_gap=0
                    if c0 and c1:
                        sequences,rows=callback_sequence[imu]
                        evidence['state_callback_window']=production_window(sequences,rows,p0['packet_seq'],p1['packet_seq'],'entry_ns')
                        callback_gap=evidence['state_callback_window']['max_adjacent_gap_ms'] or 0
                    bridge_gap=evidence['bridge_publish_window']['max_adjacent_gap_ms'] or 0
                    classification=('upstream_or_bridge_gap_observed' if bridge_gap>20
                        else 'shared_or_state_and_observer_delay' if callback_gap>20
                        else 'observer_path_delay_supported')
                else:classification='unmatched_internal_trace'
            else:
                s0=state_map.get((before['state_stamp_ns'],));s1=state_map.get((after['state_stamp_ns'],))
                if s0 and s1:
                    evidence['state_callback_window']=production_window(state_sequence,ordered_states,s0['_publication_seq'],s1['_publication_seq'],'start_ns')
                    evidence['state_publish_window']=production_window(state_sequence,ordered_states,s0['_publication_seq'],s1['_publication_seq'],'publish_begin_ns')
                    publication_gap=evidence['state_publish_window']['max_adjacent_gap_ms'] or 0
                    classification=('state_production_gap_observed' if publication_gap>20 else 'observer_path_delay_supported')
                else:classification='unmatched_internal_trace'
        detail['classification']=classification
        detail['evidence']=evidence
        detail['interpretation']='Localization evidence and GC overlap are correlations; not a unique causal diagnosis.'
        classifications[classification]+=1
        gap_details.append(detail)
    gap_details=heapq.nlargest(30,gap_details,key=lambda row:row['gap_ms'])
    series={name:value.output() for name,value in sorted(a.series.items())}
    worst=heapq.nlargest(12,((name,value.maximum) for name,value in a.series.items() if value.maximum),key=lambda pair:pair[1]['value_ns'])
    def events_near(rows,field,lo,hi,keep):
        selected=[r for r in rows if r.get(field) is not None and lo<=r[field]<=hi]
        # Bound report size but explicitly expose truncation.
        return {'total':len(selected),'truncated':len(selected)>80,
                'rows':[{k:r.get(k) for k in keep} for r in selected[:80]]}
    max_windows=[]
    for name,event in worst:
        anchor=event['anchor_ns'];lo=anchor-100*NS_MS;hi=anchor+100*NS_MS
        max_windows.append({'metric':name,'maximum':event,'window_ns':[lo,hi],
            'bridge':events_near(packets,'last_serial_read_end_ns',lo,hi,('packet_seq','device_ms','ros_stamp_ns','last_serial_read_end_ns','imu1_publish_begin_ns','imu2_publish_begin_ns')),
            'state':events_near(states,'entry_ns',lo,hi,('event','index','source_stamp_ns','entry_ns','start_ns','expected_ns','end_ns','state_stamp_ns','slot_index')),
            'observer':events_near(observer,'entry_ns',lo,hi,('event','index','source_stamp_ns','state_stamp_ns','entry_ns','end_ns'))})
    files=[directory/'run.json',directory/'observer.csv']
    if a.meta.get('trace_enabled'):
        files += [directory/'state.csv',directory/'state.csv.meta.json',directory/'bridge/bridge_packets.csv',directory/'bridge/bridge_polls.csv',directory/'bridge/bridge_meta.json']
    hashes={str(p.relative_to(directory)):hashlib.sha256(p.read_bytes()).hexdigest() for p in files if p.is_file()}
    if timer is not None:
        timer['deadline_acceptance_available']=bool(timer['runtime_phase_reference_valid'] and not a.audit.errors)
    return {'schema_version':1,'measurement_window':{'start_ns':a.lo,'end_ns':a.hi,
            'duration_s':(a.hi-a.lo)/1e9,'warmup_s':(a.lo-a.start)/1e9,
            'tail_s':(a.end-a.hi)/1e9,'policy':'target entry in [start,end); correlation indexes include all phases'},
            'trace_enabled':a.meta.get('trace_enabled'),'load_kind':a.meta.get('load_kind'),
            'integrity':a.audit.output(),'phase_counts':phases,'metrics':series,
            'timer':timer,'device_timestamp':device,'bridge_poll':poll_summary,
            'join_counts':dict(a.join_counts),'delivery_continuity':a.delivery,'observer_gap_counts':dict(a.gap_count),
            'gap_classification_counts':dict(classifications),'gc_gap_overlap_counts':dict(gc_overlap_counts),
            'largest_observer_gaps':gap_details,'maximum_event_windows':max_windows,
            'child_exit_codes':a.meta.get('child_exit_codes'),'bridge_meta':bridge_meta,'state_meta':state_meta,
            'source_sha256':hashes,
            'interpretation':[
                'Quantiles use linear interpolation (type 7); high percentiles with few observations are not worst-case bounds.',
                '20 ms and 40 ms gap counts are diagnostic thresholds, not certified control safety limits.',
                'All internal latency segments use same-host CLOCK_MONOTONIC; ROS timestamps are join keys.',
                'Host read completion is not first-byte arrival, CAN wire time, or physical IMU acquisition time.',
                'Subscribe callbacks may precede publish return; publish-begin markers avoid treating this overlap as negative latency.',
                'Trace-off runs report observer-only metrics and do not infer internal timings.',
                'No input trace is discarded silently: malformed rows, overflow and unmatchable events appear in integrity.',
                'This analysis is measurement evidence, not hard-real-time certification or a unique causal diagnosis.']}


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run',type=Path)
    parser.add_argument('--output',type=Path)
    args=parser.parse_args(argv)
    result=analyze_run(args.run)
    encoded=json.dumps(result,indent=2,ensure_ascii=False,allow_nan=False)+'\n'
    if args.output:
        args.output.write_text(encoded,encoding='utf-8')
        print(json.dumps({'output':str(args.output),'complete':result['integrity']['complete'],
                          'metrics':len(result['metrics']),'errors':result['integrity']['errors']},ensure_ascii=False))
    else:print(encoded)
    return 0 if result['integrity']['complete'] else 2


if __name__=='__main__':
    raise SystemExit(main())
