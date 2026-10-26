#!/usr/bin/env python3
"""Offline endpoint acceptance checks; no hardware access."""
import copy
import importlib.util
import math
from pathlib import Path
import unittest

SPEC = importlib.util.spec_from_file_location(
    'dm4310_range', Path(__file__).parents[1] / 'bench/dm4310_range.py')
RANGE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(RANGE)


def capture(positions=(0, 1)):
    rows = []
    for n in range(20):
        for motor in (1, 2):
            qcode = int((positions[motor-1]+12.5)*65535/25)
            raw = bytes([motor, qcode >> 8, qcode & 255, 0x7f, 0xf7, 0xff, 25, 25])
            rows.append({'t_s': n*.1+motor*.05,
                         'motor_id': motor, 'feedback_can_id': motor+3,
                         'status': 0, 'raw': raw.hex(),
                         'position_rad': qcode*25/65535-12.5,
                         'velocity_rad_s': 2047*60/4095-30})
    return rows


class EndpointTests(unittest.TestCase):
    def test_reverse_endpoint_order_and_independent_origins(self):
        result = RANGE.candidate(capture((1, -2)), capture((0, -1)), 5)
        self.assertFalse(result['motion_authorized'])
        self.assertFalse(result['deployment_verified'])
        for motor, axis in enumerate(result['axes'], 1):
            self.assertEqual(axis['feedback_can_id'], motor+3)
            self.assertLess(axis['position_min_rad'], axis['position_max_rad'])
        self.assertGreater(result['axes'][0]['position_min_rad'], .08)
        self.assertLess(result['axes'][1]['position_max_rad'], -1.08)

    def test_bad_margin_and_collapsed_interval(self):
        for margin in (0, -1, 1, 30, math.nan, math.inf):
            with self.subTest(margin=margin), self.assertRaises(ValueError):
                RANGE.candidate(capture(), capture((.2, 1.2)), margin)

    def test_missing_duplicate_or_wrong_route(self):
        for kind in ('missing', 'duplicate', 'route'):
            data = capture()
            if kind == 'missing':
                data.pop()
            elif kind == 'duplicate':
                data[3] = copy.deepcopy(data[1])
            else:
                data[0]['feedback_can_id'] = 0
            with self.subTest(kind=kind), self.assertRaises(ValueError):
                RANGE.endpoint(data)

    def test_enabled_fault_nonfinite_moving_or_wrong_scaling(self):
        for key, value in (('status', 1), ('status', 8),
                           ('position_rad', math.nan), ('velocity_rad_s', .1),
                           ('position_rad', .01), ('raw', '0180007FF7FF1919')):
            data = capture()
            data[0][key] = value
            with self.subTest(key=key, value=value), self.assertRaises(ValueError):
                RANGE.endpoint(data)

    def test_unstable_endpoint(self):
        data = capture()
        other = capture((.01, 1))
        data[-2] = other[-2]
        with self.assertRaisesRegex(ValueError, '不稳定'):
            RANGE.endpoint(data)

    def test_gap_and_short_window(self):
        for scale in (.1, 3):
            data = capture()
            for row in data:
                row['t_s'] *= scale
            with self.subTest(scale=scale), self.assertRaises(ValueError):
                RANGE.endpoint(data)

    def test_vmax50_capture_requires_matching_decode_range(self):
        data = capture()
        for row in data:
            row['velocity_rad_s'] = 2047*100/4095-50
        with self.assertRaises(ValueError):
            RANGE.endpoint(data)
        RANGE.endpoint(data,50)
        second = capture((1,2))
        for row in second:
            row['velocity_rad_s'] = 2047*100/4095-50
        profile = RANGE.candidate(data,second,5,50)
        self.assertEqual(profile['encoding_ranges_assumed'],[12.5,50.,10.])


if __name__ == '__main__':
    unittest.main()
