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

    def test_single_105_policy_uses_one_angle_and_validates_full_pair_first(self):
        policy=motion.MotionPolicy(self.config,motor_id=105)
        self.assertEqual(policy.names,['motor105_joint'])
        target,seconds=policy.move(['12','2'])
        self.assertEqual(len(target),1)
        self.assertAlmostEqual(target[0],math.radians(12))
        relative,_=policy.relative_move(['3','1'],[math.radians(12)])
        self.assertAlmostEqual(relative[0],math.radians(15))
        for words in (['1','2','3'],['nan','1'],['1','.1']):
            with self.assertRaises(ValueError):policy.move(words)
        with self.assertRaises(ValueError):policy.check_positions([0.,0.])
        self.config['motors'][0]['target_mapping_verified']=False
        with self.assertRaises(ValueError):motion.MotionPolicy(self.config,motor_id=105)

    def test_single_check_is_device_free_and_preserves_pair_config(self):
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'config.json';path.write_text(json.dumps(self.config))
            before=path.read_bytes()
            result=subprocess.run(['/usr/bin/python3',str(ROOT/'motion.py'),
                '--config',str(path),'--run-dir',str(Path(directory)/'runs'),
                '--motor-id','105','--check'],capture_output=True,text=True)
            self.assertEqual(result.returncode,0,result.stderr)
            self.assertIn('105',result.stdout)
            self.assertEqual(path.read_bytes(),before)
            self.assertFalse((Path(directory)/'runs').exists())

    def test_single_session_has_no_elapsed_time_shutdown(self):
        policy=motion.MotionPolicy(self.config,motor_id=105)
        backend=motion.BackendSession(self.config,policy,'urdf','helper',Path('/unused'),Mock())
        backend.client=Mock();backend.proc=Mock();backend.proc.poll.return_value=None
        backend.ready=True;backend.stop=Mock()
        with patch.object(motion.time,'monotonic',return_value=10.):backend.enable()
        with patch.object(motion.time,'monotonic',return_value=41.):backend.poll()
        backend.stop.assert_not_called()
        backend.client.poll.assert_called_once()

    def test_single_feedback_requires_105_and_never_fills_missing_joint(self):
        feedback=motion.Feedback(['motor105_joint'])
        self.assertFalse(feedback.ingest(['motor104_joint'],[.1],1.,1.))
        self.assertIsNone(feedback.positions)
        self.assertTrue(feedback.ingest(['motor105_joint'],[.2],2.,2.))
        self.assertEqual(feedback.positions,[.2])
        self.assertTrue(feedback.fresh(2.))
        self.assertFalse(feedback.ingest([],[],3.,3.))
        self.assertFalse(feedback.fresh(3.))

    def backend_fixture(self):
        backend=motion.BackendSession(self.config,self.policy,'urdf','helper',Path('/unused'),Mock())
        backend.proc=Mock(pid=123)
        backend.client=Mock()
        backend.client.feedback.positions=[.1,.2]
        backend.disable_pending=True
        return backend

    def test_terminal_failure_cannot_skip_controller_or_process_cleanup(self):
        backend=self.backend_fixture()
        client=backend.client
        keyboard=Mock()
        keyboard.close.side_effect=OSError('PTY gone')
        display=Mock()
        display.clear.side_effect=OSError('stdout gone')
        selector=Mock()
        with patch.object(motion,'terminate_launch') as terminate:
            with patch.object(motion,'disable_motors') as disabler:
                errors=motion.cleanup(backend,keyboard,display,selector)
                disabler.assert_called_once()
        client.disable.assert_called_once()
        terminate.assert_called_once()
        selector.close.assert_called_once()
        self.assertTrue(errors)

    def test_disable_failure_still_terminates_process(self):
        backend=self.backend_fixture()
        backend.client.disable.side_effect=RuntimeError('unconfirmed')
        with patch.object(motion,'terminate_launch') as terminate:
            with patch.object(motion,'disable_motors'):
                errors=motion.cleanup(backend,None,Mock())
        terminate.assert_called_once()
        self.assertIn('unconfirmed',' '.join(errors))

    def test_device_disable_runs_after_process_exit_even_if_controller_fails(self):
        order=[]
        backend=self.backend_fixture()
        def deactivate():
            order.append('controller')
            raise RuntimeError('controller fault')
        backend.client.disable.side_effect=deactivate
        with patch.object(motion,'terminate_launch',side_effect=lambda p:order.append('exit')):
            with patch.object(motion,'disable_motors',side_effect=lambda *a:order.append('mode15')):
                errors=motion.cleanup(backend,None,Mock())
        self.assertEqual(order,['controller','exit','mode15'])
        self.assertTrue(errors)
        with self.assertRaises(RuntimeError):backend.start()

    def test_no_device_access_when_process_exit_unconfirmed(self):
        backend=self.backend_fixture()
        with patch.object(motion,'terminate_launch',side_effect=RuntimeError('still running')):
            with patch.object(motion,'disable_motors') as disabler:
                errors=motion.cleanup(backend,None,Mock())
                disabler.assert_not_called()
        self.assertTrue(errors)

    def test_snapshot_export_only_after_exit_and_formal_disable_attempt(self):
        for disable_fails in (False,True):
            backend=self.backend_fixture(); backend.snapshot=Path('/owned-snapshot')
            order=[]
            def disable(*args):
                order.append('mode15')
                if disable_fails: raise RuntimeError('no acknowledgement')
            with patch.object(motion,'terminate_launch',side_effect=lambda p:order.append('exit')), \
                    patch.object(motion,'disable_motors',side_effect=disable), \
                    patch.object(motion.storage,'finalize',side_effect=lambda *a,**k:
                        (order.append('export') or dict(snapshot_retained=False,diagnostic_status='complete'))):
                if disable_fails:
                    with self.assertRaises(RuntimeError):backend.stop()
                else: backend.stop()
            self.assertEqual(order,['exit','mode15','export'])

    def test_group_survivor_prevents_export_and_device_access(self):
        backend=self.backend_fixture(); backend.snapshot=Path('/owned-snapshot')
        with patch.object(motion,'terminate_launch',side_effect=RuntimeError('group present')), \
                patch.object(motion,'disable_motors') as disable, \
                patch.object(motion.storage,'finalize') as export:
            with self.assertRaises(RuntimeError):backend.stop()
            disable.assert_not_called(); export.assert_not_called()

    def test_export_failure_latches_reenable_after_disable(self):
        backend=self.backend_fixture(); backend.snapshot=Path('/owned-snapshot')
        with patch.object(motion,'terminate_launch'), patch.object(motion,'disable_motors') as disable, \
                patch.object(motion.storage,'finalize',side_effect=OSError('disk full')):
            with self.assertRaisesRegex(RuntimeError,'disk full'):backend.stop()
            disable.assert_called_once()
        with self.assertRaises(RuntimeError):backend.start()

    def test_log_admission_rejects_backend_before_device_process_launch(self):
        backend=self.backend_fixture(); backend.proc=None; backend.client=None; backend.disable_pending=False
        with patch.object(motion.storage,'admit',side_effect=ValueError('quota exhausted')), \
                patch.object(motion.subprocess,'Popen') as launch:
            with self.assertRaisesRegex(ValueError,'quota'):backend.start()
            launch.assert_not_called()

    def test_launch_failure_removes_only_empty_prelaunch_snapshot(self):
        with tempfile.TemporaryDirectory() as directory:
            backend=motion.BackendSession(self.config,self.policy,'urdf','helper',Path(directory),Mock())
            with patch.object(motion.storage,'admit'), \
                    patch.object(motion.storage,'make_snapshots',return_value=Path('/owned-empty')), \
                    patch.object(motion.storage,'abandon_prelaunch') as abandon, \
                    patch.object(motion.subprocess,'Popen',side_effect=OSError('cannot launch')):
                with self.assertRaisesRegex(OSError,'cannot launch'):backend.start()
                abandon.assert_called_once_with(Path('/owned-empty'),Path(directory))
            self.assertIsNone(backend.snapshot); self.assertFalse(backend.disable_pending)

    def test_disable_ack_failure_is_not_success(self):
        backend=self.backend_fixture()
        with patch.object(motion,'terminate_launch'):
            with patch.object(motion,'disable_motors',side_effect=RuntimeError('105 missing ack')):
                errors=motion.cleanup(backend,None,Mock())
        self.assertIn('105 missing ack',' '.join(errors))

    def test_backend_disable_latches_failure_and_preserves_writer_ownership(self):
        with tempfile.TemporaryDirectory() as directory:
            backend=motion.BackendSession(self.config,self.policy,'urdf','helper',Path(directory),Mock())
            backend.proc=Mock(pid=123)
            backend.client=Mock()
            backend.client.feedback.positions=[.1,.2]
            backend.disable_pending=True
            with patch.object(motion,'terminate_launch',side_effect=RuntimeError('writer still alive')):
                with patch.object(motion,'disable_motors') as disabler:
                    with self.assertRaises(RuntimeError):backend.stop()
                    disabler.assert_not_called()
            self.assertIsNotNone(backend.proc)
            with self.assertRaises(RuntimeError):backend.start()

    def test_confirmed_backend_disable_is_idempotent_and_caches_only_last_position(self):
        with tempfile.TemporaryDirectory() as directory:
            backend=motion.BackendSession(self.config,self.policy,'urdf','helper',Path(directory),Mock())
            backend.proc=Mock(pid=123)
            backend.client=Mock()
            backend.client.feedback.positions=[.1,.2]
            backend.disable_pending=True
            with patch.object(motion,'terminate_launch'):
                with patch.object(motion,'disable_motors') as disabler:
                    backend.stop();backend.stop()
                    disabler.assert_called_once()
            self.assertIsNone(backend.proc)
            self.assertIsNone(backend.client)
            self.assertIn('上次测量',backend.status())
            self.assertNotIn('ROS消息更新',backend.status())

    def test_backend_missing_disable_ack_latches_reenable(self):
        with tempfile.TemporaryDirectory() as directory:
            backend=motion.BackendSession(self.config,self.policy,'urdf','helper',Path(directory),Mock())
            backend.proc=Mock(pid=123)
            backend.client=Mock()
            backend.client.feedback.positions=[.1,.2]
            backend.disable_pending=True
            with patch.object(motion,'terminate_launch'):
                with patch.object(motion,'disable_motors',side_effect=RuntimeError('105 missing ack')):
                    with self.assertRaisesRegex(RuntimeError,'105 missing ack'):backend.stop()
            self.assertTrue(backend.disable_pending)
            with self.assertRaises(RuntimeError):backend.start()
            with patch.object(motion,'disable_motors') as disabler:
                with self.assertRaisesRegex(RuntimeError,'105 missing ack'):backend.stop()
                disabler.assert_not_called()  # Final cleanup preserves the failed attempt.

    def test_helper_exit_zero_alone_is_not_disable_confirmation(self):
        with tempfile.TemporaryDirectory() as directory:
            run=Path(directory)
            with self.assertRaisesRegex(RuntimeError,'失能未获全部确认'):
                motion.disable_motors('/bin/true',run/'unused',run,Mock(),[104,105])

    def test_disable_helper_requires_both_ack_records_and_final_confirmation(self):
        for ids,confirmed,valid in (([104],True,False),([104,105],False,False),([104,105],True,True)):
            with tempfile.TemporaryDirectory() as directory:
                run=Path(directory)
                def fake_run(command,**kwargs):
                    rows=[dict(event='disable_ack',id=drive,status=119) for drive in ids]
                    rows.append(dict(event='disable_complete',confirmed=confirmed,missing=[]))
                    return NS(returncode=0,stdout='\n'.join(json.dumps(r) for r in rows),stderr='')
                with patch.object(motion.subprocess,'run',side_effect=fake_run):
                    if valid:
                        motion.disable_motors('helper',run/'urdf',run,Mock(),[104,105])
                    else:
                        with self.assertRaises(RuntimeError):
                            motion.disable_motors('helper',run/'urdf',run,Mock(),[104,105])

    def test_log_open_failure_still_attempts_device_disable(self):
        with tempfile.TemporaryDirectory() as directory:
            run=Path(directory)
            with patch.object(Path,'open',side_effect=OSError('disk unavailable')):
                with patch.object(motion.subprocess,'run',return_value=NS(returncode=2,stdout='',stderr='')) as invoke:
                    with self.assertRaises(RuntimeError):
                        motion.disable_motors('helper',run/'urdf',run,Mock(),[104,105])
                invoke.assert_called_once()

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
