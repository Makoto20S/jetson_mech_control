"""Auditor must reject missing evidence and a mismatched downstream packet."""
import importlib.util
import unittest
from pathlib import Path

class ChainAuditTest(unittest.TestCase):
    def load(self):
        path = Path(__file__).with_name('chain_audit.py')
        self.assertTrue(path.exists(), 'independent chain auditor is missing')
        spec = importlib.util.spec_from_file_location('chain_audit', path)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        return module

    def test_decodes_literal_mode6_packet_and_crc(self):
        audit = self.load()
        # Use the saved real trace as evidence, not the production encoder.
        packet = bytes.fromhex('f7120e00a4')
        with self.assertRaises(ValueError):
            audit.decode_packet(packet)
        init = bytes.fromhex('f71206007d7008000000000000')
        self.assertEqual(audit.decode_packet(init)['init'], True)
        with self.assertRaises(ValueError):
            audit.decode_packet(init[:-1] + b'\x01')

    def test_empty_trace_is_not_a_pass(self):
        audit = self.load()
        report = audit.audit_records([dict(schema_version=1, total=0, dropped=0)])
        self.assertFalse(report['complete'])

    def test_overflow_is_not_a_pass(self):
        audit = self.load()
        report = audit.audit_records([dict(schema_version=1, total=1, dropped=1)])
        self.assertFalse(report['complete'])
        self.assertTrue(any('dropped' in x for x in report['issues']))

    def fixture(self):
        packet = 'f7120e000b5675680600000c08001a7188000a000a'
        rows = []
        def add(stage, **data):
            row = dict(seq=len(rows), stage=stage, ns=100+len(rows), begin_ns=0,
                       joint=0, cycle=1, generation=1, io=1, position=173.3,
                       feedback=173.3, flags=0, deadline=10000, result=0,
                       errno=0, requested=0, length=0, captured=0, hex='', id=0)
            row.update(data); rows.append(row)
        add('config_target', position=1., feedback=0., flags=104)
        add('config_limits', position=100., feedback=100.)
        add('claim_before', flags=0)
        add('claim_after', flags=1)
        add('interface', flags=1)
        add('dispatch', flags=3)
        add('stored')
        for stage in ('prepared', 'submitted', 'selected', 'transport_send'):
            add(stage, id=0x668, flags=4, hex='001a7188000a000a', length=8, captured=8)
        add('serial_request', hex=packet, requested=21, length=21, captured=21)
        add('syscall_write', hex=packet, requested=21, length=21, captured=21, result=21)
        add('serial_result', requested=21)
        add('transport_result', id=0x668, flags=4, hex='001a7188000a000a', length=8, captured=8)
        add('runtime_stop')
        return [dict(schema_version=1, total=len(rows), dropped=0)] + rows

    def test_complete_chain_and_short_write_reassembly(self):
        audit = self.load(); rows = self.fixture()
        self.assertTrue(audit.audit_records(rows)['complete'])
        write = next(r for r in rows[1:] if r['stage'] == 'syscall_write')
        index = rows.index(write)
        second = dict(write, hex=write['hex'][14:], requested=14, length=14, captured=14, result=14)
        write.update(hex=write['hex'][:14], length=7, captured=7, result=7)
        rows.insert(index+1, second)
        rows[0]['total'] += 1
        for i, r in enumerate(rows[1:]): r.update(seq=i, ns=100+i)
        self.assertTrue(audit.audit_records(rows)['complete'])

    def test_rejects_each_uncorrelated_boundary(self):
        audit = self.load()
        for stage, field, value in [('claim_after', 'position', 999),
                                    ('claim_after', 'flags', 0),
                                    ('transport_result', 'generation', 999),
                                    ('transport_result', 'id', 999),
                                    ('syscall_write', 'hex', '00'*21)]:
            with self.subTest(stage=stage, field=field):
                rows = self.fixture()
                next(r for r in rows[1:] if r['stage'] == stage)[field] = value
                self.assertFalse(audit.audit_records(rows)['complete'])

    def test_second_transport_attempt_cannot_borrow_first_io_evidence(self):
        audit = self.load(); rows = self.fixture()
        extra = [dict(next(r for r in rows[1:] if r['stage'] == stage))
                 for stage in ('selected', 'transport_send', 'transport_result')]
        rows[-1:-1] = extra
        rows[0]['total'] += len(extra)
        for i, r in enumerate(rows[1:]): r.update(seq=i, ns=100+i)
        self.assertFalse(audit.audit_records(rows)['complete'])

    def test_dispatch_cannot_authorize_itself(self):
        audit = self.load(); rows = self.fixture()
        for r in rows[1:]:
            if r['stage'] in ('claim_after', 'interface'): r['flags'] = 0
        self.assertFalse(audit.audit_records(rows)['complete'])

    def test_operator_request_cannot_reuse_old_enable(self):
        audit = self.load()
        events = [dict(event='operator_input', words=['enable'], monotonic_s=1.),
                  dict(event='switch_request', enabled=True, monotonic_s=2.),
                  dict(event='controller', state='active', monotonic_s=4.),
                  dict(event='switch_request', enabled=True, monotonic_s=5.),
                  dict(event='controller', state='active', monotonic_s=7.)]
        self.assertEqual(audit.audit_operator(events, [dict(ns=3000000000, joint=0)]), [])
        self.assertTrue(audit.audit_operator(events, [dict(ns=3000000000, joint=0), dict(ns=6000000000, joint=0)]))

    def test_cancelled_generation_cannot_reach_transport(self):
        audit = self.load(); rows = self.fixture()
        index = next(i for i, r in enumerate(rows) if r.get('stage') == 'selected')
        rows.insert(index, dict(rows[index], stage='cancel'))
        rows[0]['total'] += 1
        for i, r in enumerate(rows[1:]): r.update(seq=i, ns=100+i)
        self.assertFalse(audit.audit_records(rows)['complete'])

    def test_missing_submission_cannot_pass(self):
        audit = self.load(); rows = self.fixture()
        rows = [r for r in rows if r.get('stage') != 'submitted']
        rows[0]['total'] -= 1
        for i, r in enumerate(rows[1:]): r.update(seq=i, ns=100+i)
        self.assertFalse(audit.audit_records(rows)['complete'])

if __name__ == '__main__':
    unittest.main()
