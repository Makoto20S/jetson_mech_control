import json
import math
import random
import unittest
from ao_core import AOConfig, AdaptiveOscillator


def sine(ao, start, duration, frequency=1.0, amplitude=1.0, offset=0.0):
    result = []
    for i in range(round(duration * 100)):
        t = start + i * .01
        result.append(ao.update(offset + amplitude * math.sin(2 * math.pi * frequency * t), t))
    return result


class TestAO(unittest.TestCase):
    def test_config_rejects_invalid(self):
        for kwargs in [dict(input_scale_rad_s=0), dict(min_frequency_hz=2), dict(n_harmonics=0),
                       dict(timeout_s=.001), dict(input_offset_rad_s=float('nan'))]:
            with self.assertRaises(ValueError): AOConfig(**kwargs)

    def test_zero_and_constant_stop(self):
        for value in [0.0, .3]:
            ao = AdaptiveOscillator()
            records = [ao.update(value, i * .01) for i in range(300)]
            self.assertEqual(records[-1]['state'], 'STOPPED')
            self.assertFalse(any(r['valid'] for r in records))
            json.dumps(records, allow_nan=False)

    def test_sine_tracks(self):
        records = sine(AdaptiveOscillator(), 0, 25)
        tail = records[-300:]
        self.assertGreater(sum(r['valid'] for r in tail) / len(tail), .9)
        self.assertLess(abs(sum(r['frequency_hz'] for r in tail) / len(tail) - 1), .05)
        self.assertLess(math.sqrt(sum(r['residual'] ** 2 for r in tail) / len(tail)), .15)

    def test_stop_restart(self):
        ao = AdaptiveOscillator()
        self.assertTrue(sine(ao, 0, 20)[-1]['valid'])
        stopped = [ao.update(0., 20 + i*.01) for i in range(300)]
        self.assertEqual(stopped[-1]['state'], 'STOPPED')
        self.assertFalse(any(r['valid'] for r in stopped[100:]))
        resumed = sine(ao, 23, 25)
        self.assertFalse(resumed[0]['valid'])
        self.assertTrue(resumed[-1]['valid'])

    def test_invalid_inputs_revoke_validity(self):
        for value in [float('nan'), float('inf'), 21.0]:
            ao = AdaptiveOscillator()
            sine(ao, 0, 15)
            record = ao.update(value, 15)
            self.assertFalse(record['valid']); self.assertEqual(record['state'], 'INVALID')
            json.dumps(record, allow_nan=False)

    def test_timestamps_and_recovery(self):
        ao = AdaptiveOscillator(); ao.update(0.1, 1.)
        self.assertEqual(ao.update(.2, 1.)['reason'], 'nonmonotonic_timestamp')
        self.assertEqual(ao.update(.2, .9)['reason'], 'nonmonotonic_timestamp')
        self.assertEqual(ao.update(.2, 2.)['reason'], 'sample_gap')
        self.assertFalse(ao.update(.2, 2.01)['valid'])
        self.assertEqual(ao.update(.2, float('nan'))['reason'], 'nonfinite_timestamp')

    def test_timeout_no_extrapolated_valid_phase(self):
        ao=AdaptiveOscillator(); self.assertTrue(sine(ao, 0, 20)[-1]['valid'])
        self.assertTrue(ao.check_timeout(20.0)['valid'])
        out=ao.check_timeout(20.2)
        self.assertEqual(out['reason'], 'input_timeout'); self.assertFalse(out['valid'])
        self.assertGreater(out['input_age_s'], .15)
        self.assertFalse(ao.update(.3, 20.21)['valid'])

    def test_denominator_guard_before_division(self):
        ao=AdaptiveOscillator(); ao.update(0.,0.)
        ao._alphas=[0.,0.,0.]
        record=ao.update(.1,.01)
        json.dumps(record,allow_nan=False)
        self.assertFalse(record['valid'])

    def test_explicit_unit_scaling(self):
        a=AdaptiveOscillator(); b=AdaptiveOscillator(AOConfig(input_scale_rad_s=2, input_offset_rad_s=.3))
        for i in range(300):
            t=i*.01; v=math.sin(2*math.pi*t)
            ra=a.update(v,t); rb=b.update(2*v+.3,t)
            self.assertAlmostEqual(ra['phase_rad'],rb['phase_rad'],places=10)
            self.assertAlmostEqual(ra['frequency_hz'],rb['frequency_hz'],places=10)

    def test_output_bounded_and_json_safe_with_jitter(self):
        ao=AdaptiveOscillator(); t=0.
        for i in range(1000):
            t+=.009 if i%2 else .011
            out=ao.update(math.sin(2*math.pi*t),t)
            self.assertGreaterEqual(out['phase_rad'],0.)
            self.assertLess(out['phase_rad'],2*math.pi)
            self.assertGreaterEqual(out['frequency_hz'],.2)
            self.assertLessEqual(out['frequency_hz'],3.)
            json.dumps(out,allow_nan=False)

    def test_frequency_step_and_noise(self):
        ao=AdaptiveOscillator(); rng=random.Random(533)
        records=[]; phase=0.
        for i in range(4000):
            t=i*.01
            frequency=1.0 if t<20 else 1.4
            phase+=2*math.pi*frequency*.01
            records.append(ao.update(math.sin(phase)+rng.gauss(0,.03),t))
        for tail,target in [(records[1700:2000],1.0),(records[-300:],1.4)]:
            self.assertGreater(sum(r['valid'] for r in tail)/len(tail),.8)
            self.assertLess(abs(sum(r['frequency_hz'] for r in tail)/len(tail)-target),.08)
        json.dumps(records,allow_nan=False)

    def test_clock_backward_and_tiny_scale(self):
        ao=AdaptiveOscillator(); ao.update(.1,1.)
        self.assertEqual(ao.check_timeout(.9)['reason'],'clock_moved_backward')
        ao=AdaptiveOscillator(AOConfig(input_scale_rad_s=1e-300))
        ao.update(0.,0.)
        self.assertEqual(ao.update(1.,.01)['reason'],'normalization_out_of_range')


if __name__=='__main__': unittest.main(verbosity=2)
