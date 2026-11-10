"""Replay archived MIT receive frames through StateCore on the trace monotonic timeline."""
import hashlib
import json
import math
import struct
from types import SimpleNamespace

from mit_communication import decode_rx
from mit_feedback import FIELDS
from state_core import StateCore


def replay(folder):
    manifest = json.loads((folder.parent / 'remote_manifest.json').read_text())['files']
    digests = {f['path']: f['sha256'] for f in manifest}
    run = json.loads((folder / 'summary.json').read_text())
    events = {e['event']: e['monotonic_ns'] for e in run['events']}
    core = StateCore(motor_source='mit')
    stream = []
    for index, (bus, mid) in enumerate(((2, 104), (1, 105))):
        name = f'bus{bus}_motor{mid}.jsonl'
        data = (folder / name).read_bytes()
        assert hashlib.sha256(data).hexdigest() == digests[name], name
        lines = data.splitlines()
        header = json.loads(lines[0])
        assert header['total'] == len(lines)-1 and header['dropped'] == 0
        seq = 0
        for line in lines[1:]:
            row = json.loads(line)
            if row['stage'] != 'transport_receive' or row['id'] != 0x2900 + mid:
                continue
            assert (row['deadline'], row['flags'], row['result'], row['length'], row['captured']) == (bus, 4, 0, 8, 8)
            seq += 1
            stream.append((row['ns'], index, seq, row))
        assert seq > 0
    counts = [0, 0]
    in_window = [0, 0]
    bounds = [{}, {}]
    pairs = 0
    for mono, i, seq, row in sorted(stream):
        raw = decode_rx(row)
        values = [104+i, seq, mono//1000, float(raw['status'] == 0), raw['position_deg'],
                  raw['erpm'], raw['iq_a'], raw['temperature_c'], raw['status']]
        msg = SimpleNamespace(joint_names=[core.mit.names[i]], interface_values=[
            SimpleNamespace(interface_names=list(FIELDS), values=values)])
        core.mit_feedback(msg, mono, mono)
        state = core.snapshot(mono, mono)
        axis = state.mit_motors[i]
        assert axis.drive_id == i+104 and axis.has_sample and axis.rx_sequence == seq
        assert axis.host_rx_us == mono//1000 and axis.valid == (raw['status'] == 0)
        # Check all eight original bytes from the published fields, including sign and scale.
        encoded = struct.pack('>hhhbB', round(axis.position_deg*10), round(axis.erpm/10),
                              round(axis.iq_a*100), round(axis.temperature_c), axis.fault_code)
        assert encoded.hex() == row['hex']
        assert math.isclose(axis.output_velocity_rad_s, raw['erpm'] * math.tau / (60*14*8))
        assert not axis.output_velocity_valid
        counts[i] += 1
        if events['zero_effort_window_start'] <= mono <= events['zero_effort_window_end']:
            in_window[i] += 1
        for key in ('position_deg','erpm','iq_a','temperature_c','fault_code'):
            v = getattr(axis, key)
            low, high = bounds[i].get(key, (v, v))
            bounds[i][key] = (min(low, v), max(high, v))
        if state.sources[0].valid:
            pairs += 1
            assert list(state.joints.position) == [math.radians(a.position_deg) for a in state.mit_motors]
            assert not state.joints.velocity and not state.joints.effort
        assert not state.q_valid
    assert pairs and all(in_window)
    assert not any(a.valid for a in core.snapshot(mono+61_000_000, mono+61_000_000).mit_motors)
    print(json.dumps(dict(order=[104,105], byte_exact_rx_frames=counts, zero_window=in_window,
                          valid_pair_snapshots=pairs, raw_ranges=bounds), ensure_ascii=True))
