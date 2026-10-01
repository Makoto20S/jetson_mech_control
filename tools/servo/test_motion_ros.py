"""Real controller_manager/JTC offline acceptance; only the loopback plugin is used."""
import importlib.util
import math
import os
import pty
import select
import json
from pathlib import Path
import subprocess
import tempfile
import time
import unittest
import uuid

ROOT=Path(__file__).resolve().parent
spec=importlib.util.spec_from_file_location('servo_motion',ROOT/'motion.py')
motion=importlib.util.module_from_spec(spec); spec.loader.exec_module(motion)


class RosMotionTests(unittest.TestCase):
    def test_terminal_workflow_and_config_preservation(self):
        config=motion.load_config(ROOT/'config.example.json',require_calibrated=False)
        config['calibrated']=True
        for motor in config['motors']:
            motor.update(position_min_rad=-2.,position_max_rad=2.)
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'config.json';path.write_text(json.dumps(config))
            before=path.read_bytes()
            driver=Path(directory)/'loopback_cli.py'
            # Test-only substitution: production entry has no simulation bypass flag.
            driver.write_text("import importlib.util\n"
                +"s=importlib.util.spec_from_file_location('motion',"+repr(str(ROOT/'motion.py'))+")\n"
                +"m=importlib.util.module_from_spec(s);s.loader.exec_module(m)\n"
                +"original=m.config_tools.generate_urdf\n"
                +"m.config_tools.generate_urdf=lambda c: original(c).replace('mech_bringup/Ak30ServoSystem','mech_hardware_ros2_control/CompositeSystem')\n"
                +"raise SystemExit(m.main())\n")
            master,slave=pty.openpty()
            proc=subprocess.Popen(['/usr/bin/python3',str(driver),'--config',str(path),
                '--run-dir',str(Path(directory)/'runs')],stdin=slave,stdout=slave,stderr=slave,
                start_new_session=True)
            os.close(slave)
            received=bytearray()
            def expect(text,timeout=15.):
                deadline=time.monotonic()+timeout
                while time.monotonic()<deadline:
                    if text.encode() in received:return
                    if select.select([master],[],[],.1)[0]:
                        try:received.extend(os.read(master,65536))
                        except OSError:break
                self.fail('Missing '+text+'; output='+received.decode(errors='replace'))
            try:
                expect('控制器已就绪')
                os.write(master,b'enable\n');expect('已使能')
                os.write(master,b'move 5 -3 0.6\n');expect('轨迹到达容差内')
                os.write(master,b'quit\n')
                self.assertEqual(proc.wait(timeout=15),0,received.decode(errors='replace'))
                self.assertEqual(path.read_bytes(),before)
                records=[json.loads(line) for line in next((Path(directory)/'runs').glob('*/events.jsonl')).read_text().splitlines()]
                self.assertTrue(any(r['event']=='goal' for r in records))
                self.assertEqual([r['state'] for r in records if r['event']=='controller'],['active','inactive'])
            finally:
                if proc.poll() is None:
                    proc.terminate()
                    proc.wait(timeout=15)
                os.close(master)

    def test_pair_enable_move_stop_disable_and_stale_feedback(self):
        import rclpy
        from rclpy.signals import SignalHandlerOptions
        os.environ.setdefault('ROS_LOG_DIR','/tmp/servo-motion-ros-test')
        config=motion.load_config(ROOT/'config.example.json',require_calibrated=False)
        config['calibrated']=True
        for motor in config['motors']:
            motor.update(position_min_rad=-2.,position_max_rad=2.)
        policy=motion.MotionPolicy(config)
        # Exercise the exact launch/YAML/action client without opening hardware.
        urdf=motion.config_tools.generate_urdf(config).replace(
            'mech_bringup/Ak30ServoSystem','mech_hardware_ros2_control/CompositeSystem')
        events=[]
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'loopback.urdf';path.write_text(urdf)
            ready=Path(directory)/'controllers.ready'
            with (Path(directory)/'framework.log').open('w+') as log:
                namespace='servo_test_'+uuid.uuid4().hex[:8]
                proc=subprocess.Popen(['ros2','launch',str(ROOT/'servo_pair.launch.py'),
                    'urdf:='+str(path),'namespace:='+namespace,'ready_file:='+str(ready)],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
                rclpy.init(args=[],signal_handler_options=SignalHandlerOptions.NO)
                client=motion.RosControl(policy,namespace,lambda event,**data:events.append((event,data)))
                try:
                    deadline=time.monotonic()+15
                    while time.monotonic()<deadline:
                        client.spin(.02)
                        # Do not poll services while the graph is still starting.
                        if ready.exists() and client.feedback.fresh(time.monotonic()):
                            break
                    self.assertEqual(client.state(),'inactive')
                    self.assertTrue(client.feedback.fresh(time.monotonic()))
                    self.assertFalse(client.active)
                    with self.assertRaises(ValueError):client.send([.1,.1],1.)
                    client.enable()
                    target,seconds=policy.move(['5','-3','0.6'])
                    client.send(target,seconds)
                    deadline=time.monotonic()+4
                    while client.result is not None and time.monotonic()<deadline:client.poll()
                    self.assertIsNone(client.result)
                    for actual,expected in zip(client.feedback.positions,target):
                        self.assertAlmostEqual(actual,expected,delta=math.radians(.5))
                    client.send([.5,-.5],2.)
                    deadline=time.monotonic()+.15
                    while time.monotonic()<deadline:client.poll()
                    client.stop()
                    deadline=time.monotonic()+3
                    while client.result is not None and time.monotonic()<deadline:client.poll()
                    self.assertIsNone(client.result)
                    self.assertLess(client.feedback.positions[0],.4)
                    client.disable()
                    self.assertEqual(client.state(),'inactive')
                    self.assertFalse(client.active)
                    self.assertTrue(any(e=='stop_requested' for e,_ in events))
                    client.feedback.received=-math.inf
                    with self.assertRaises(ValueError):client.ready_feedback()
                except Exception:
                    log.flush();log.seek(0);print(log.read())
                    raise
                finally:
                    try:client.disable()
                    except Exception:pass
                    motion.terminate_launch(proc)
                    client.node.destroy_node();rclpy.shutdown()


if __name__=='__main__':unittest.main()
