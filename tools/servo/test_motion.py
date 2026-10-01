"""Offline policy checks for the real ROS trajectory operator."""
import importlib.util
import math
from pathlib import Path
import unittest
import subprocess
import tempfile
import json
from unittest.mock import Mock, patch
from types import SimpleNamespace as NS

ROOT = Path(__file__).resolve().parent
spec = importlib.util.spec_from_file_location('servo_motion', ROOT / 'motion.py')
motion = importlib.util.module_from_spec(spec)
spec.loader.exec_module(motion)


class MotionTests(unittest.TestCase):
    def setUp(self):
        self.config = motion.load_config(ROOT / 'config.example.json', require_calibrated=False)
        self.config['calibrated'] = True
        for m in self.config['motors']:
            m.update(position_min_rad=-2., position_max_rad=2.)
        self.policy = motion.MotionPolicy(self.config)

    def test_terminal_failure_cannot_skip_controller_or_process_cleanup(self):
        client=Mock()
        keyboard=Mock()
        keyboard.close.side_effect=OSError('PTY gone')
        display=Mock()
        display.clear.side_effect=OSError('stdout gone')
        with patch.object(motion,'terminate_launch') as terminate:
            errors=motion.cleanup(client,object(),keyboard,display)
        client.disable.assert_called_once()
        terminate.assert_called_once()
        self.assertTrue(errors)

    def test_disable_failure_still_terminates_process(self):
        client=Mock()
        client.disable.side_effect=RuntimeError('unconfirmed')
        with patch.object(motion,'terminate_launch') as terminate:
            errors=motion.cleanup(client,object(),None,Mock())
        terminate.assert_called_once()
        self.assertIn('unconfirmed',' '.join(errors))

    def test_check_works_without_ros_and_preserves_config(self):
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'config.json'
            path.write_text(json.dumps(self.config))
            before=path.read_bytes()
            result=subprocess.run(['/usr/bin/python3',str(ROOT/'motion.py'),
                '--config',str(path),'--run-dir',str(Path(directory)/'runs'),'--check'],
                capture_output=True,text=True)
            self.assertEqual(result.returncode,0,result.stderr)
            self.assertEqual(path.read_bytes(),before)
            self.assertFalse((Path(directory)/'runs').exists())

    def test_enable_requires_feedback_before_any_service_call(self):
        client=object.__new__(motion.RosControl)
        client.uncertain=False
        client.ready_feedback=Mock(side_effect=ValueError('no feedback'))
        client.state=Mock(return_value='inactive')
        client.switch=Mock()
        with self.assertRaises(ValueError):client.enable()
        client.state.assert_not_called()
        client.switch.assert_not_called()

    def test_never_enabled_cleanup_does_not_query_starting_manager(self):
        client=object.__new__(motion.RosControl)
        client.active=False;client.uncertain=False
        client.state=Mock()
        client.disable()
        client.state.assert_not_called()

    def test_relative_target_uses_measured_position_each_time(self):
        current=[math.radians(20),math.radians(-10)]
        target,seconds=self.policy.relative_move(['5','-3','2'],current)
        self.assertAlmostEqual(target[0],math.radians(25))
        self.assertAlmostEqual(target[1],math.radians(-13))
        again,_=self.policy.relative_move(['5','-3','2'],[math.radians(24),math.radians(-12)])
        self.assertAlmostEqual(again[0],math.radians(29))
        self.assertAlmostEqual(again[1],math.radians(-15))
        self.assertEqual(seconds,2.)

    def test_relative_mapping_and_limits(self):
        m=self.config['motors'][0]
        m.update(feedback_scale=-.02,feedback_offset=1.,target_scale=-50.,target_offset=50.)
        policy=motion.MotionPolicy(self.config)
        target,_=policy.relative_move(['5','0','2'],[0.,.2])
        self.assertAlmostEqual(target[0],-.1)
        self.assertAlmostEqual(target[1],.2)
        for words in (['200','0','2'],['nan','0','2'],['0','0','inf'],['0']):
            with self.subTest(words=words),self.assertRaises(ValueError):
                self.policy.relative_move(words,[0.,0.])
        with self.assertRaises(ValueError):self.policy.relative_move(['0','0','2'],[3.,0.])

    def test_relative_rejects_stale_feedback_and_ongoing_motion(self):
        client=object.__new__(motion.RosControl)
        client.goal=None
        client.ready_feedback=Mock(side_effect=ValueError('stale'))
        client.send=Mock()
        with self.assertRaises(ValueError):client.move_relative(['1','1','2'])
        client.send.assert_not_called()
        client.goal=object();client.ready_feedback.reset_mock()
        with self.assertRaises(ValueError):client.move_relative(['1','1','2'])
        client.ready_feedback.assert_not_called()

    def test_hardware_health_rejects_cached_state_with_active_controller(self):
        def response(state=3, available=True, claimed=True):
            return NS(component=[NS(name='servo_pair',state=NS(id=state,label='active'),
                command_interfaces=[NS(name=n+'/position',is_available=available,is_claimed=claimed)
                                    for n in self.policy.names])])
        self.assertTrue(motion.hardware_health(response(),self.policy.names,True)['healthy'])
        for reply in (response(state=1),response(available=False),response(claimed=False),NS(component=[])):
            with self.subTest(reply=reply):
                self.assertFalse(motion.hardware_health(reply,self.policy.names,True)['healthy'])
        self.assertTrue(motion.hardware_health(response(claimed=False),self.policy.names,False)['healthy'])

    def test_hardware_fault_invalidates_feedback_and_suppresses_hold(self):
        client=object.__new__(motion.RosControl)
        client.policy=self.policy;client.record=Mock();client.active=True
        client.feedback=motion.Feedback(self.policy.names)
        client.feedback.ingest(self.policy.names,[0.,0.],1.,1.)
        client.hardware_error=None;client.hardware_status=None
        with self.assertRaises(RuntimeError):client.accept_hardware(NS(component=[]))
        self.assertFalse(client.feedback.fresh(1.))
        client.on_feedback(NS())  # Cached broadcaster traffic must not restore freshness.
        self.assertFalse(client.feedback.fresh(1.))
        client.stop=Mock();client.state=Mock();client.uncertain=False
        with self.assertRaises(RuntimeError):client.disable()
        client.stop.assert_not_called();client.state.assert_not_called()

    def test_requires_saved_calibration(self):
        self.config['calibrated'] = False
        with self.assertRaises(ValueError):
            motion.MotionPolicy(self.config)

    def test_degree_input_uses_target_mapping_and_joint_order(self):
        self.config['motors'].reverse()
        m = self.config['motors'][1]
        m.update(target_scale=-50., target_offset=10.)
        policy = motion.MotionPolicy(self.config)
        target, seconds = policy.move(['-40', '30', '2'])
        self.assertEqual(policy.names, ['motor104_joint', 'motor105_joint'])
        self.assertAlmostEqual(target[0], 1.)
        self.assertAlmostEqual(target[1], math.radians(30))
        self.assertEqual(seconds, 2.)

    def test_rejects_nonfinite_outside_bounds_and_bad_duration(self):
        for args in (['nan','0','2'], ['0','inf','2'], ['200','0','2'],
                     ['0','0','nan'], ['0','0','0'], ['0','0','61'], ['0']):
            with self.subTest(args=args), self.assertRaises(ValueError):
                self.policy.move(args)

    def test_feedback_mapping_is_distinct_from_target_mapping(self):
        self.config['motors'][0].update(feedback_scale=-.02, feedback_offset=1.)
        self.assertAlmostEqual(motion.MotionPolicy(self.config).device_angles([0.,0.])[0],50.)

    def test_requires_both_finite_fresh_monotonic_joint_samples(self):
        feedback = motion.Feedback(self.policy.names)
        self.assertFalse(feedback.fresh(1.))
        self.assertFalse(feedback.ingest(['motor104_joint'], [0.], 1., 1.))
        self.assertTrue(feedback.ingest(self.policy.names[::-1], [.2,.1], 1., 1.))
        self.assertEqual(feedback.positions, [.1,.2])
        self.assertFalse(feedback.fresh(1.3))
        self.assertFalse(feedback.ingest(self.policy.names, [0.,0.], 1., 1.3))
        self.assertFalse(feedback.fresh(1.3))
        self.assertFalse(feedback.ingest(self.policy.names, [float('nan'),0.],2.,2.))

    def test_estimated_velocity_and_invalid_sample_invalidation(self):
        feedback = motion.Feedback(self.policy.names)
        feedback.ingest(self.policy.names,[0.,0.],1.,1.)
        feedback.ingest(self.policy.names,[.1,-.2],1.2,1.2)
        for actual,expected in zip(feedback.velocity,[.5,-1.]):
            self.assertAlmostEqual(actual,expected)
        feedback.ingest(self.policy.names,[0.,float('inf')],1.3,1.3)
        self.assertFalse(feedback.fresh(1.3))

    def test_current_position_must_be_inside_saved_bounds(self):
        with self.assertRaises(ValueError):
            self.policy.check_positions([0.,2.1])
        self.policy.check_positions([-2.,2.])


if __name__ == '__main__':
    unittest.main()
