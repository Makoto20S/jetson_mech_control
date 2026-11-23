"""Device-free contracts for visual bench tuning. No ROS or serial imports."""
import io
import json
from pathlib import Path
import subprocess
import tempfile
import threading
import time
import unittest
import urllib.request
import urllib.error
from types import SimpleNamespace
from unittest.mock import Mock,patch
import ph_exo_bench_panel as panel
import ph_exo_bench_worker as worker
from ref01_bench_hold import panel_observation,validate_gains
TEST_TMP=panel.ROOT/".local/validation/panel_unit_tmp"
def setUpModule():TEST_TMP.mkdir(parents=True,exist_ok=True)

class Config(unittest.TestCase):
    def test_default_matches_roundtrip_candidate(self):
        self.assertEqual(worker.validate_config(worker.DEFAULT),dict(k_psi=20,k_beta=20,b_psi=.3,b_beta=.3,torque_nm=10,current_a=6,travel_deg=90))
    def test_duration_field_is_not_accepted(self):
        with self.assertRaises(ValueError):worker.validate_config(dict(worker.DEFAULT,duration_s=30))
    def test_console_retention_is_bounded(self):
        sink=worker.TailConsole()
        for _ in range(1000):sink.write("x"*3000)
        self.assertLessEqual(len(sink.getvalue()),128*2048)
    def test_independent_gains(self):
        c=dict(worker.DEFAULT,k_psi=3,k_beta=7,b_psi=.1,b_beta=.3)
        self.assertEqual(worker.validate_config(c),c)
        self.assertEqual(validate_gains((3,7,.1,.3)),(3.,7.,.1,.3))
    def test_invalid_values(self):
        for key,values in {"k_psi":[0,-1,30.1,float("nan"),True,"5"],
                           "b_beta":[-1,1.01,float("inf")],"torque_nm":[0,10.01],"current_a":[0,6.01],"travel_deg":[0,90.01]}.items():
            for value in values:
                with self.subTest(key=key,value=value),self.assertRaises(ValueError):
                    worker.validate_config(dict(worker.DEFAULT,**{key:value}))
        with self.assertRaises(ValueError):worker.validate_config(dict(worker.DEFAULT,extra=1))
    def test_motor_order_and_snapshot_copy(self):
        msg=SimpleNamespace(observed_at_monotonic_s=9.,position_rad=[1,2],velocity_rad_s=[3,4],
            reference_rad=[5,6],mode=2,reason="none",state_valid=True,motor_target_nm=[.7,-.4])
        source={104:{"mit_iq_a":2}}
        r=panel_observation(msg,source);source[104]["mit_iq_a"]=3
        self.assertEqual(r["motor_target_nm"],{"104":.7,"105":-.4})
        self.assertEqual(r["feedback"]["104"]["mit_iq_a"],2)
    def test_check_does_not_require_ros(self):
        out=subprocess.check_output([__import__("sys").executable,"-B",str(Path(worker.__file__)),"--check"])
        self.assertEqual(json.loads(out)["mode"],"check")
    def test_ssh_fixed_target_and_quoted_root(self):
        cmd=panel.ssh_command({"ssh_host":"orin-nx","remote_root":"/home/nvidia/project ' test"})
        self.assertIn("-T",cmd);self.assertIn("StrictHostKeyChecking=yes",cmd)
        self.assertIn("set -e; cd ",cmd[-1]);self.assertIn("--execute-panel",cmd[-1])
        with self.assertRaises(ValueError):panel.ssh_command({"ssh_host":"-bad","remote_root":"/tmp"})

class Stops(unittest.TestCase):
    def test_eof_and_stop_request(self):
        for text in ("","STOP\n","anything\n"):
            event=threading.Event();worker.watch_stop(io.StringIO(text),event);self.assertTrue(event.is_set())
    def test_constructor_and_preview_never_spawn(self):
        spawn=Mock();p=panel.Panel(preview=True,popen=spawn)
        p.snapshot()
        with self.assertRaises(ValueError):p.start(worker.DEFAULT)
        spawn.assert_not_called()
    def test_double_start_rejected(self):
        p=panel.Panel(popen=Mock());p.phase="running"
        with self.assertRaises(ValueError):p.start(worker.DEFAULT)
        p.popen.assert_not_called()
    def test_uncertain_exit_blocks_retry(self):
        p=panel.Panel(popen=Mock());p.phase="unconfirmed"
        with self.assertRaises(ValueError):p.start(worker.DEFAULT)
        p.popen.assert_not_called()
    def test_stop_is_request_not_confirmation(self):
        p=panel.Panel();p.phase="running";p.proc=SimpleNamespace(stdin=io.BytesIO())
        p.stop()
        self.assertEqual(p.phase,"stopping");self.assertEqual(p.proc.stdin.getvalue(),b"STOP\n")
    def test_browser_timeout(self):
        p=panel.Panel();p.phase="running";p.proc=SimpleNamespace(stdin=io.BytesIO())
        p.last_poll=time.monotonic()-5;p.watchdog()
        self.assertEqual(p.phase,"stopping")
    def test_finished_requires_both_disable_acks(self):
        for ack,phase in (([104,105],"finished"),([104],"unconfirmed")):
            with tempfile.TemporaryDirectory(dir=TEST_TMP) as d:
                p=panel.Panel();p.directory=Path(d);p.phase="running"
                e={"type":"finished","result":{"disable_ack":ack},"error":None}
                proc=SimpleNamespace(stdout=io.BytesIO((worker.PREFIX+json.dumps(e)+"\n").encode()),wait=lambda:0)
                p.read_worker(proc);self.assertEqual(p.phase,phase)
                self.assertTrue((Path(d)/"result.json").exists())
    def test_connection_loss_cannot_claim_disabled(self):
        with tempfile.TemporaryDirectory(dir=TEST_TMP) as d:
            p=panel.Panel();p.directory=Path(d);p.phase="running"
            p.read_worker(SimpleNamespace(stdout=io.BytesIO(b"SSH lost\n"),wait=lambda:255))
            self.assertEqual(p.phase,"unconfirmed")
    def test_start_to_archive_with_fake_transport(self):
        with tempfile.TemporaryDirectory(dir=TEST_TMP) as d,patch.object(panel,"ROOT",Path(d)):
            root=Path(d);(root/"config/parallel_exo").mkdir(parents=True)
            (root/"config/parallel_exo/orin_sync.json").write_text(json.dumps({"ssh_host":"orin-nx","remote_root":"/safe"}))
            done={"type":"finished","result":{"disable_ack":[104,105],"samples":[],"statuses":[]},"error":None}
            fake=SimpleNamespace(stdin=io.BytesIO(),stdout=io.BytesIO((worker.PREFIX+json.dumps(done)+"\n").encode()),
                                 wait=lambda:0,poll=lambda:0)
            spawn=Mock(return_value=fake);p=panel.Panel(popen=spawn);p.start(worker.DEFAULT)
            until=time.monotonic()+2
            while not p.finished_seen and time.monotonic()<until:time.sleep(.01)
            self.assertTrue(p.finished_seen)
            sent=json.loads(fake.stdin.getvalue())
            self.assertEqual(sent["config"],worker.DEFAULT)
            self.assertEqual(sent["source_signature"],worker.source_signature())
            self.assertEqual(p.phase,"finished");spawn.assert_called_once()


    def test_cancel_gate_is_explicit(self):
        event=threading.Event()
        worker.confirm_start(event)
        event.set()
        with self.assertRaises(worker.CancelledBeforeStart):worker.confirm_start(event)

    def test_cancelled_only_before_start(self):
        for phase,flag,result,expected in (
            ("preparing",True,None,"cancelled"),
            ("stopping",True,None,"cancelled"),
            ("preparing",False,None,"unconfirmed"),
            ("running",True,None,"unconfirmed"),
            ("stopping",True,{"disable_ack":[]},"unconfirmed")):
            with self.subTest(phase=phase,flag=flag,result=result),tempfile.TemporaryDirectory(dir=TEST_TMP) as d:
                p=panel.Panel();p.directory=Path(d);p.phase=phase
                e={"type":"finished","result":result,"cancelled_before_start":flag}
                proc=SimpleNamespace(stdout=io.BytesIO((worker.PREFIX+json.dumps(e)+"\n").encode()),wait=lambda:0,poll=lambda:0)
                p.proc=proc;p.read_worker(proc)
                self.assertEqual(p.phase,expected)
                if expected=="cancelled":
                    p.reset();self.assertEqual(p.phase,"idle")
                else:
                    with self.assertRaises(ValueError):p.reset()

    def test_reset_cannot_release_live_worker(self):
        p=panel.Panel();p.phase="cancelled";p.proc=SimpleNamespace(poll=lambda:None)
        with self.assertRaises(ValueError):p.reset()


    def test_estop_protocol_latches_both_events(self):
        stop=threading.Event();urgent=threading.Event()
        worker.watch_stop(io.StringIO("ESTOP\n"),stop,emergency=urgent)
        self.assertTrue(stop.is_set());self.assertTrue(urgent.is_set())

    def test_estop_sends_distinct_command(self):
        p=panel.Panel();p.phase="running"
        p.proc=SimpleNamespace(stdin=io.BytesIO(),poll=lambda:None)
        p.emergency_stop()
        self.assertEqual(p.proc.stdin.getvalue(),b"ESTOP\n")
        self.assertEqual(p.phase,"stopping")
        with self.assertRaises(ValueError):p.reset()

    def test_estop_broken_link_stays_unconfirmed(self):
        p=panel.Panel();p.phase="running";p.proc=SimpleNamespace(poll=lambda:255)
        p.emergency_stop();self.assertEqual(p.phase,"unconfirmed")
        with self.assertRaises(ValueError):p.reset()

    def test_urgent_monitor_interrupts_manager(self):
        from ref01_bench_hold import emergency_manager_watch
        import signal
        event=threading.Event();event.set();manager=Mock();manager.poll.return_value=None
        record={}
        emergency_manager_watch(manager,event,threading.Event(),record)
        manager.send_signal.assert_called_once_with(signal.SIGINT)
        self.assertIn("emergency_received_s",record)

class Http(unittest.TestCase):
    def setUp(self):
        self.p=panel.Panel(preview=True,popen=Mock())
        self.s=panel.make_server(self.p);self.url="http://127.0.0.1:"+str(self.s.server_port)
        self.t=threading.Thread(target=self.s.serve_forever,daemon=True);self.t.start()
    def tearDown(self):
        self.s.shutdown();self.s.server_close();self.t.join()
    def request(self,path,body=None,**headers):
        req=urllib.request.Request(self.url+path,data=None if body is None else json.dumps(body).encode(),headers=headers)
        return urllib.request.urlopen(req,timeout=2)
    def test_page_opens_without_worker(self):
        with self.request("/") as r:
            html=r.read().decode()
            self.assertIn("开始本轮",html);self.assertNotIn("__TOKEN__",html)
        self.p.popen.assert_not_called()
    def test_api_requires_token_and_same_origin(self):
        with self.assertRaises(urllib.error.HTTPError) as err:self.request("/state")
        self.assertEqual(err.exception.code,403)
        with self.assertRaises(urllib.error.HTTPError) as err:
            self.request("/start",worker.DEFAULT,**{"X-Panel-Token":self.s.panel_token,"Origin":"https://other.example"})
        self.assertEqual(err.exception.code,403);self.p.popen.assert_not_called()
    def test_preview_start_rejected_over_http(self):
        with self.assertRaises(urllib.error.HTTPError) as err:
            self.request("/start",worker.DEFAULT,**{"X-Panel-Token":self.s.panel_token})
        self.assertEqual(err.exception.code,400);self.p.popen.assert_not_called()
    def test_read_state(self):
        with self.request("/state?after=0",**{"X-Panel-Token":self.s.panel_token}) as r:
            data=json.load(r);self.assertEqual(data["phase"],"idle");self.assertTrue(data["preview"])

if __name__=="__main__":unittest.main()
