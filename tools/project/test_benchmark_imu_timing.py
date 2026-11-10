import csv
import io
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from benchmark_imu_timing import Buffer, FIELDS, age_ns, stamp_ns, stop_child


class BufferTests(unittest.TestCase):
    def test_exact_monotonic_and_stamp_precision(self):
        b=Buffer(2)
        exact=2173630123456789
        b.add((0,1,1793948693611708445,exact,exact+17,0,0,0,0,0,0,0))
        self.assertEqual(b.data[2],1793948693611708445)
        self.assertEqual(b.data[3],exact)
        self.assertEqual(b.data[4]-b.data[3],17)

    def test_overflow_preserves_initial_records(self):
        b=Buffer(1); b.add([0]*12); b.add([1]*12)
        self.assertEqual((b.size,b.dropped),(1,1))
        self.assertEqual(list(b.data),[0]*12)

    def test_csv_roundtrip_units_and_events(self):
        b=Buffer(2)
        b.add((1,0,0,100,200,300,400,500,1,0,1234567,-1000000))
        b.add((2,2,0,600,700,0,0,0,0,0,0,0))
        with tempfile.TemporaryDirectory() as d:
            path=Path(d)/'observer.csv'; b.dump(path)
            rows=list(csv.DictReader(io.StringIO(path.read_text())))
        self.assertEqual(list(rows[0]),FIELDS)
        self.assertEqual(rows[0]['event'],'state')
        self.assertEqual(rows[0]['imu1_receive_age_ms'],'1.234567')
        self.assertEqual(rows[1]['event'],'gc')

    def test_invalid_capacity_and_width(self):
        for cap in (0,-1,True,2000001,1.5):
            with self.assertRaises(ValueError): Buffer(cap)
        with self.assertRaises(ValueError): Buffer(1).add([0])

    def test_timestamp_and_unavailable_age(self):
        self.assertEqual(stamp_ns(SimpleNamespace(sec=123,nanosec=456)),123000000456)
        self.assertEqual(age_ns(float('inf')),-1)
        self.assertEqual(age_ns(0.5),500000)

    def test_cleanup_reaps_child_that_exits_before_signal(self):
        child=Mock(pid=12345,returncode=0)
        child.poll.return_value=None
        with patch('benchmark_imu_timing.os.killpg',create=True,side_effect=ProcessLookupError):
            self.assertEqual(stop_child(child),0)
        child.wait.assert_called_once_with(timeout=15)

    def test_cleanup_does_not_signal_already_reaped_child(self):
        child=Mock(pid=12345,returncode=0)
        child.poll.return_value=0
        with patch('benchmark_imu_timing.os.killpg',create=True) as send:
            self.assertEqual(stop_child(child),0)
        send.assert_not_called()


if __name__=='__main__':
    unittest.main()
