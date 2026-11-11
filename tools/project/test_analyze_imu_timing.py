import csv
import json
from pathlib import Path
import tempfile
import unittest

from analyze_imu_timing import (analyze_run, OBSERVER_HEADER, PACKET_HEADER, POLL_HEADER,
                         STATE_HEADER, percentile, Series)

MS=1_000_000
BASE=1_000_000_000


def write_csv(path,header,rows):
    with path.open('w',encoding='utf-8',newline='') as stream:
        writer=csv.DictWriter(stream,header,extrasaction='ignore')
        writer.writeheader()
        writer.writerows(rows)


def write_json(path,value):
    path.write_text(json.dumps(value),encoding='utf-8')


def fixture(root,kind='normal',trace=True):
    """Synthetic same-host traces with no real ROS, clock or hardware calls."""
    root=Path(root)
    (root/'bridge').mkdir()
    packets=[];states=[];observer=[]
    for i in range(24):
        upstream=(50*MS if kind=='bridge_block' and i>=8 else 0)
        state_delay=(50*MS if kind=='state_block' and i>=8 else 0)
        observer_delay=(50*MS if kind=='observer_block' and i>=8 else 0)
        rx=BASE+i*10*MS+upstream
        source=10_000_000_000+i*10*MS
        state_stamp=20_000_000_000+i*10*MS
        packet={'packet_seq':i+1,'device_ms':i*10,'ros_stamp_ns':source,
            'last_serial_read_end_ns':rx,'frame_host_arrival_ns':rx+20_000,
            'decode_begin_ns':rx+30_000,'decode_end_ns':rx+50_000,
            'imu1_publish_begin_ns':rx+60_000,'imu1_publish_end_ns':rx+80_000,
            'imu2_publish_begin_ns':rx+90_000,'imu2_publish_end_ns':rx+110_000,
            'packet_done_ns':rx+120_000}
        packets.append(packet)
        for imu in (1,2):
            callback=rx+200_000+imu*10_000+state_delay
            states.append({'event':'imu','index':imu,'source_stamp_ns':source,
                           'entry_ns':callback,'end_ns':callback+10_000})
            callback=rx+400_000+imu*10_000+observer_delay
            observer.append({'event':'imu','index':imu,'source_stamp_ns':source,
                'entry_ns':callback,'end_ns':callback+10_000})
        start=rx+MS+state_delay
        shift=(upstream+state_delay)//(10*MS)
        slot=i+shift
        skipped=5 if i==8 and kind in ('state_block','bridge_block') else 0
        state={'event':'state','index':0,'entry_ns':start,'start_ns':start,
            'expected_ns':BASE+MS+slot*10*MS,'slot_index':slot,
            'snapshot_ns':start+50_000,'publish_begin_ns':start+100_000,
            'end_ns':start+200_000,'state_stamp_ns':state_stamp,
            'imu1_stamp_ns':source,'imu2_stamp_ns':source,
            'joints_valid':0,'imu1_valid':1,'imu2_valid':1,
            'imu1_receive_age_ns':840_000,'imu2_receive_age_ns':830_000,
            'imu1_source_age_ns':1_050_000,'imu2_source_age_ns':1_050_000,
            'skipped_periods':skipped}
        states.append(state)
        callback=start+400_000+observer_delay
        observer.append({'event':'state','index':0,'source_stamp_ns':0,
            'entry_ns':callback,'end_ns':callback+10_000,'state_stamp_ns':state_stamp,
            'imu1_stamp_ns':source,'imu2_stamp_ns':source,'imu1_valid':1,'imu2_valid':1,
            'imu1_receive_age_ms':.84,'imu2_receive_age_ms':.83})
    if kind=='observer_block':
        observer.append({'event':'gc','index':2,'entry_ns':BASE+76*MS,'end_ns':BASE+126*MS})
    if kind=='state_block':
        states.append({'event':'gc','index':2,'entry_ns':BASE+76*MS,'end_ns':BASE+126*MS})
    observer.sort(key=lambda r:r['entry_ns'])
    states.sort(key=lambda r:r['entry_ns'])
    metadata={'schema_version':1,'completed':True,'start_mono_ns':BASE,'warmup_end_ns':BASE+30*MS,
        'measurement_end_ns':BASE+300*MS,'stop_requested_ns':BASE+310*MS,
        'end_mono_ns':BASE+350*MS,'duration_s':.3,'warmup_s':.03,
        'trace_enabled':trace,'load_kind':kind,'child_exit_codes':[0,0],
        'observer_rows':len(observer),'observer_dropped':0,'environment':{'fixture':True}}
    write_json(root/'run.json',metadata)
    write_csv(root/'observer.csv',OBSERVER_HEADER,observer)
    if trace:
        write_csv(root/'bridge/bridge_packets.csv',PACKET_HEADER,packets)
        write_csv(root/'bridge/bridge_polls.csv',POLL_HEADER,[])
        write_json(root/'bridge/bridge_meta.json',{'schema_version':1,'clock':'CLOCK_MONOTONIC',
            'packet_observed':len(packets),'packet_stored':len(packets),'packet_overflow':0,
            'poll_observed':1000,'poll_anomalies':0,'poll_stored':0,'poll_overflow':0,
            'poll_mode':'anomaly','clock_errors':0,'serial_read_clock_errors':0,'completed':True,
            'max_poll_gap_ns':MS,'max_poll_duration_ns':100_000})
        write_csv(root/'state.csv',STATE_HEADER,states)
        write_json(root/'state.csv.meta.json',{'period_ns':10*MS,'completed':True,
            'phase_origin':'first_observed_callback','first_phase_ns':BASE+MS,
            'rows_written':len(states),'rows_dropped':0,'overflow':False,
            'skipped_periods_total':5 if kind in ('state_block','bridge_block') else 0,
            'capacity_rows':10000,'buffer_bytes':100000,'clock_regressions':0})
    return metadata,packets,states,observer


class TimingAnalysisTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory(prefix='imu-timing-test-')
        self.root=Path(self.temp.name)

    def tearDown(self):
        self.temp.cleanup()

    def test_normal_complete_chain(self):
        fixture(self.root)
        result=analyze_run(self.root)
        self.assertTrue(result['integrity']['complete'],result['integrity'])
        self.assertEqual(result['timer']['executed_deadline_exceedances'],0)
        self.assertEqual(result['timer']['skipped_phase_grid_slots'],0)
        self.assertAlmostEqual(result['metrics']['observer.imu1.gap']['max_ms'],10)
        self.assertAlmostEqual(result['metrics']['pipeline.imu1.host_read_to_state_observer']['p99_ms'],1.4)
        self.assertEqual(result['device_timestamp']['non_10ms_count'],0)
        self.assertEqual(result['delivery_continuity']['observer.imu1']['unobserved_source_events_between_adjacent_callbacks'],0)
        self.assertFalse(result['bridge_poll']['quantiles_available'])
        self.assertNotIn('bridge.poll.gap',result['metrics'])
        self.assertEqual(result['phase_counts']['observer']['observer.imu1']['warmup'],3)

    def test_observer_block_localized_and_gc_overlap(self):
        fixture(self.root,'observer_block')
        result=analyze_run(self.root)
        self.assertTrue(result['integrity']['complete'],result['integrity'])
        self.assertAlmostEqual(result['metrics']['observer.imu1.gap']['max_ms'],60)
        self.assertAlmostEqual(result['metrics']['bridge.packet.gap']['max_ms'],10)
        self.assertAlmostEqual(result['metrics']['state.imu1.gap']['max_ms'],10)
        self.assertGreater(result['gap_classification_counts']['observer_path_delay_supported'],0)
        self.assertGreater(result['gc_gap_overlap_counts']['observer.observer_gaps_overlapping_gc'],0)
        self.assertAlmostEqual(result['metrics']['observer.gc.duration']['max_ms'],50)

    def test_state_block_phase_skips_and_gc_overlap(self):
        fixture(self.root,'state_block')
        result=analyze_run(self.root)
        self.assertTrue(result['integrity']['complete'],result['integrity'])
        self.assertEqual(result['timer']['skipped_phase_grid_slots'],5)
        self.assertEqual(result['timer']['max_consecutive_bad_phase_grid_slots'],5)
        self.assertEqual(result['timer']['executed_deadline_exceedances'],0)
        self.assertGreater(result['gap_classification_counts']['state_production_gap_observed'],0)
        self.assertGreater(result['gc_gap_overlap_counts']['state.observer_gaps_overlapping_gc'],0)
        self.assertAlmostEqual(result['metrics']['observer.imu1.gap']['max_ms'],10)

    def test_bridge_block_is_upstream_evidence_not_cause(self):
        fixture(self.root,'bridge_block')
        result=analyze_run(self.root)
        self.assertTrue(result['integrity']['complete'],result['integrity'])
        self.assertAlmostEqual(result['metrics']['bridge.packet.gap']['max_ms'],60)
        self.assertGreater(result['gap_classification_counts']['upstream_or_bridge_gap_observed'],0)
        self.assertEqual(result['device_timestamp']['non_10ms_count'],0)

    def test_trace_off_cannot_invent_internal_latency(self):
        fixture(self.root,'observer_block',trace=False)
        result=analyze_run(self.root)
        self.assertTrue(result['integrity']['complete'],result['integrity'])
        self.assertIsNone(result['timer'])
        self.assertIsNone(result['device_timestamp'])
        self.assertTrue(all(name.startswith('observer.') for name in result['metrics']))
        self.assertNotIn('state_to_observer.publish_begin_to_callback',result['metrics'])
        self.assertIn('observer_gap_only_no_internal_trace',result['gap_classification_counts'])

    def test_legacy_timer_reference_remains_diagnostic_only(self):
        fixture(self.root)
        timer=analyze_run(self.root)['timer']
        self.assertFalse(timer['deadline_reference_available'])
        self.assertFalse(timer['deadline_reference_valid'])
        self.assertFalse(timer['runtime_phase_reference_valid'])
        self.assertFalse(timer['deadline_acceptance_available'])
        self.assertIsNone(timer['anchor_uncertainty_ns'])
        self.assertEqual(timer['phase_origin'],'first_observed_callback')
        self.assertEqual(timer['deadline_evidence_scope'],'diagnostic_only_unverified_phase_grid')
        self.assertIn('Zero exceedances do not establish deadline acceptance',timer['deadline_definition'])
        self.assertEqual(timer['executed_deadline_exceedances'],0)

    def test_api_timer_reference_reports_origin_and_uncertainty(self):
        fixture(self.root)
        path=self.root/'state.csv.meta.json'
        meta=json.loads(path.read_text())
        origin='ros_timer_time_until_next_call_bracket; midpoint_plus_until'
        meta.update(phase_origin=origin,deadline_reference_available=True,
            anchor_uncertainty_ns=4000,anchor_error=None,
            anchor_mono_before_ns=BASE-2000,anchor_mono_after_ns=BASE+2000,
            anchor_until_ns=MS)
        write_json(path,meta)
        timer=analyze_run(self.root)['timer']
        self.assertTrue(timer['deadline_reference_available'])
        self.assertTrue(timer['deadline_reference_valid'])
        self.assertTrue(timer['runtime_phase_reference_valid'])
        self.assertTrue(timer['deadline_acceptance_available'])
        self.assertEqual(timer['starts_before_expected_beyond_anchor_uncertainty'],0)
        self.assertEqual(timer['negative_release_to_completion_count'],0)
        self.assertEqual(timer['phase_origin'],origin)
        self.assertEqual(timer['anchor_uncertainty_ns'],4000)
        self.assertEqual(timer['anchor_mono_after_ns'],BASE+2000)
        self.assertIn('ROS timer API bracket-anchored grid',timer['deadline_definition'])
        self.assertEqual(timer['executed_deadline_exceedances'],0)

    def test_runtime_phase_conflict_keeps_data_complete_but_not_deadline_eligible(self):
        _,_,states,_=fixture(self.root)
        path=self.root/'state.csv.meta.json'
        meta=json.loads(path.read_text())
        meta.update(phase_origin='ros_timer_time_until_next_call_bracket; midpoint_plus_until',
            deadline_reference_available=True,anchor_uncertainty_ns=4000,anchor_error=None,
            anchor_mono_before_ns=BASE-2000,anchor_mono_after_ns=BASE+2000,
            anchor_until_ns=MS)
        write_json(path,meta)
        # A one-slot offset remains arithmetically continuous but puts each
        # observed callback before its assigned release, as after a grid bias.
        for row in states:
            if row['event']=='state':
                row['slot_index']+=1
                row['expected_ns']+=10*MS
        write_csv(self.root/'state.csv',STATE_HEADER,states)
        result=analyze_run(self.root)
        timer=result['timer']
        self.assertTrue(result['integrity']['complete'],result['integrity'])
        self.assertTrue(timer['deadline_reference_valid'])
        self.assertFalse(timer['runtime_phase_reference_valid'])
        self.assertFalse(timer['deadline_acceptance_available'])
        self.assertEqual(timer['starts_before_expected_beyond_anchor_uncertainty'],21)
        self.assertEqual(timer['negative_release_to_completion_count'],21)
        self.assertEqual(timer['finishes_before_expected_beyond_anchor_uncertainty'],21)
        self.assertEqual(timer['deadline_evidence_scope'],'diagnostic_only_runtime_phase_conflict')
        self.assertIn('cannot establish actual deadline failure or acceptance',timer['deadline_definition'])
        self.assertEqual(timer['executed_deadline_exceedances'],0)
        self.assertEqual(timer['skipped_phase_grid_slots'],0)

    def test_early_start_inside_anchor_bracket_is_not_runtime_conflict(self):
        _,_,states,_=fixture(self.root)
        path=self.root/'state.csv.meta.json'
        meta=json.loads(path.read_text())
        meta.update(phase_origin='ros_timer_time_until_next_call_bracket; midpoint_plus_until',
            deadline_reference_available=True,anchor_uncertainty_ns=4000,anchor_error=None,
            anchor_mono_before_ns=BASE-2000,anchor_mono_after_ns=BASE+2000,
            anchor_until_ns=MS)
        write_json(path,meta)
        for row in states:
            if row['event']=='state':
                for key in ('entry_ns','start_ns','snapshot_ns','publish_begin_ns','end_ns'):
                    row[key]-=2000
        write_csv(self.root/'state.csv',STATE_HEADER,states)
        result=analyze_run(self.root)
        timer=result['timer']
        self.assertTrue(result['integrity']['complete'],result['integrity'])
        self.assertTrue(timer['runtime_phase_reference_valid'])
        self.assertTrue(timer['deadline_acceptance_available'])
        self.assertEqual(timer['starts_before_expected_beyond_anchor_uncertainty'],0)
        self.assertEqual(timer['negative_release_to_completion_count'],0)

    def test_failed_or_incomplete_anchor_cannot_imply_acceptance(self):
        fixture(self.root)
        path=self.root/'state.csv.meta.json'
        meta=json.loads(path.read_text())
        meta.update(phase_origin='ros_timer_time_until_next_call_bracket; midpoint_plus_until',
            deadline_reference_available=True,anchor_uncertainty_ns=None,
            anchor_error='timer API unavailable')
        write_json(path,meta)
        timer=analyze_run(self.root)['timer']
        self.assertTrue(timer['deadline_reference_available'])
        self.assertFalse(timer['deadline_reference_valid'])
        self.assertEqual(timer['anchor_error'],'timer API unavailable')
        self.assertEqual(timer['deadline_evidence_scope'],'diagnostic_only_unverified_phase_grid')

    def test_overflow_is_not_ignored(self):
        fixture(self.root)
        path=self.root/'bridge/bridge_meta.json'
        meta=json.loads(path.read_text())
        meta.update(packet_observed=25,packet_overflow=1)
        write_json(path,meta)
        result=analyze_run(self.root)
        self.assertFalse(result['integrity']['complete'])
        self.assertGreater(result['integrity']['counts']['trace_overflow'],0)

    def test_unmatched_stamp_is_not_ignored(self):
        meta,packets,states,observer=fixture(self.root)
        row=next(r for r in observer if r['event']=='imu' and r['entry_ns']>BASE+40*MS)
        row['source_stamp_ns']+=1
        write_csv(self.root/'observer.csv',OBSERVER_HEADER,observer)
        result=analyze_run(self.root)
        self.assertFalse(result['integrity']['complete'])
        self.assertGreater(result['integrity']['counts']['unmatched_trace_events'],0)

    def test_delivery_gap_is_explicit_without_calling_wire_loss(self):
        meta,packets,states,observer=fixture(self.root)
        chosen=next(r for r in observer if r['event']=='imu' and r['index']==1 and r['entry_ns']>BASE+40*MS)
        observer.remove(chosen)
        meta['observer_rows']=len(observer)
        write_json(self.root/'run.json',meta)
        write_csv(self.root/'observer.csv',OBSERVER_HEADER,observer)
        result=analyze_run(self.root)
        self.assertTrue(result['integrity']['complete'],result['integrity'])
        progression=result['delivery_continuity']['observer.imu1']
        self.assertEqual(progression['unobserved_source_events_between_adjacent_callbacks'],1)
        self.assertIn('not physical',progression['interpretation'])

    def test_duplicate_join_key_fails_closed(self):
        meta,packets,states,observer=fixture(self.root)
        packets[8]['ros_stamp_ns']=packets[7]['ros_stamp_ns']
        write_csv(self.root/'bridge/bridge_packets.csv',PACKET_HEADER,packets)
        result=analyze_run(self.root)
        self.assertFalse(result['integrity']['complete'])
        self.assertIn('duplicate_join_key',result['integrity']['counts'])

    def test_observer_missing_imu_messages_does_not_fake_bridge_gap(self):
        meta,packets,states,observer=fixture(self.root)
        observer=[r for r in observer if not (r['event']=='imu' and r['index']==1
                    and BASE+80*MS<r['entry_ns']<BASE+130*MS)]
        meta['observer_rows']=len(observer)
        write_json(self.root/'run.json',meta)
        write_csv(self.root/'observer.csv',OBSERVER_HEADER,observer)
        result=analyze_run(self.root)
        self.assertTrue(result['integrity']['complete'],result['integrity'])
        gap=result['largest_observer_gaps'][0]
        self.assertEqual(gap['classification'],'observer_path_delay_supported')
        self.assertAlmostEqual(gap['evidence']['bridge_publish_window']['endpoint_span_ms'],60)
        self.assertAlmostEqual(gap['evidence']['bridge_publish_window']['max_adjacent_gap_ms'],10)
        self.assertEqual(result['delivery_continuity']['observer.imu1']['unobserved_source_events_between_adjacent_callbacks'],5)

    def test_observer_missing_state_messages_does_not_fake_state_gap(self):
        meta,packets,states,observer=fixture(self.root)
        observer=[r for r in observer if not (r['event']=='state'
                    and BASE+80*MS<r['entry_ns']<BASE+130*MS)]
        meta['observer_rows']=len(observer)
        write_json(self.root/'run.json',meta)
        write_csv(self.root/'observer.csv',OBSERVER_HEADER,observer)
        result=analyze_run(self.root)
        self.assertTrue(result['integrity']['complete'],result['integrity'])
        gap=result['largest_observer_gaps'][0]
        self.assertEqual(gap['classification'],'observer_path_delay_supported')
        self.assertAlmostEqual(gap['evidence']['state_publish_window']['endpoint_span_ms'],60)
        self.assertAlmostEqual(gap['evidence']['state_publish_window']['max_adjacent_gap_ms'],10)

    def test_device_wrap_is_continuity_not_backward(self):
        meta,packets,states,observer=fixture(self.root)
        for i,row in enumerate(packets):row['device_ms']=((2**32)-80+i*10)&0xffffffff
        write_csv(self.root/'bridge/bridge_packets.csv',PACKET_HEADER,packets)
        result=analyze_run(self.root)
        self.assertEqual(result['device_timestamp']['non_10ms_count'],0)

    def test_publish_overlap_is_valid(self):
        meta,packets,states,observer=fixture(self.root)
        for row in packets:row['imu1_publish_end_ns']=row['imu1_publish_begin_ns']+MS
        write_csv(self.root/'bridge/bridge_packets.csv',PACKET_HEADER,packets)
        result=analyze_run(self.root)
        self.assertTrue(result['integrity']['complete'],result['integrity'])
        self.assertGreater(result['metrics']['bridge_to_state.imu1.publish_begin_to_callback']['min_ms'],0)

    def test_real_deadline_overrun_separate_from_skip(self):
        meta,packets,states,observer=fixture(self.root)
        target=[r for r in states if r['event']=='state' and r['start_ns']>BASE+40*MS][0]
        target['end_ns']=target['expected_ns']+11*MS
        write_csv(self.root/'state.csv',STATE_HEADER,states)
        result=analyze_run(self.root)
        self.assertEqual(result['timer']['executed_deadline_exceedances'],1)
        self.assertEqual(result['timer']['max_consecutive_bad_phase_grid_slots'],1)

    def test_missing_flush_metadata_rejected(self):
        fixture(self.root)
        (self.root/'bridge/bridge_meta.json').unlink()
        result=analyze_run(self.root)
        self.assertFalse(result['integrity']['complete'])
        self.assertIn('bridge_flush_incomplete_or_unknown',result['integrity']['counts'])

    def test_malformed_timing_field_reported_not_crash(self):
        meta,packets,states,observer=fixture(self.root)
        packets[8]['packet_done_ns']=''
        write_csv(self.root/'bridge/bridge_packets.csv',PACKET_HEADER,packets)
        result=analyze_run(self.root)
        self.assertFalse(result['integrity']['complete'])
        self.assertIn('missing_packet_fields',result['integrity']['counts'])

    def test_invalid_window_rejected(self):
        meta,*_=fixture(self.root)
        meta['warmup_end_ns']=meta['measurement_end_ns']
        write_json(self.root/'run.json',meta)
        with self.assertRaises(ValueError):analyze_run(self.root)

    def test_quantile_small_sample_is_explicit(self):
        self.assertAlmostEqual(percentile([1,2,3,4],.5),2.5)
        self.assertAlmostEqual(percentile([1,2,3,4],.999),3.997)
        series=Series();series.add(500000,1)
        result=series.output()
        self.assertEqual(result['count'],1)
        self.assertEqual(result['p99_9_ms'],.5)


if __name__=='__main__':
    unittest.main()
