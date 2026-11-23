import unittest,math
from ref01_mit_fixture import feedback_words,decode_command,ERPM_SCALE,LSB
import mit_communication as mit

class WireContract(unittest.TestCase):
    def test_plant_warmup_restores_initial_state(self):
        from ref01_mit_fixture import Plant
        x=Plant()
        self.assertAlmostEqual(float(x.state["q"][0]),0.,places=10)
        self.assertAlmostEqual(float(x.state["q"][1]),math.pi/9,places=10)
        self.assertLess(max(abs(float(v)) for v in x.state["dq"]),1e-10)

    def test_gateway_disable_has_zero_dlc(self):
        import struct
        from ref01_mit_fixture import can_id
        self.assertEqual(can_id(struct.pack("<IBB",0xf68,0x0c,0)),0xf68)
        self.assertEqual(can_id(struct.pack("<IBB",0x868,0x0c,8)+bytes.fromhex(mit.ZERO_WIRE)),0x868)
        for bad in (b"",struct.pack("<IBB",0xf68,0x0c,1),
                    struct.pack("<IBB",0xf68,4,0),struct.pack("<IBB",0xf68,0x0c,0)+bytes(8)):
            with self.assertRaises(ValueError):can_id(bad)

    def test_nominal_zero_is_half_lsb_negative(self):
        torque,code=decode_command(bytes.fromhex(mit.ZERO_WIRE))
        self.assertEqual(code,2047);self.assertAlmostEqual(torque,-LSB/2)
    def test_nonzero_gain_rejected(self):
        b=bytearray.fromhex(mit.ZERO_WIRE);b[5]=1
        with self.assertRaises(ValueError):decode_command(b)
    def test_out_of_envelope_rejected(self):
        b=bytearray.fromhex(mit.ZERO_WIRE);b[6]|=15;b[7]=255
        with self.assertRaises(ValueError):decode_command(b)
    def test_feedback_units_and_axis_order(self):
        p,v=feedback_words([0.,math.pi/180],[ERPM_SCALE*10,-ERPM_SCALE*20])
        self.assertEqual(p,[460,450]);self.assertEqual(v,[-2,1])
    def test_nominal_feedback_word_resolution(self):
        p,v=feedback_words([math.radians(.049),0.],[ERPM_SCALE*4.9,0])
        self.assertEqual(p,[450,450]);self.assertEqual(v,[0,0])
    def test_independent_positive_negative_torque_words(self):
        for command in (-.1,.1,.2,-.2):
            code=int((command+15)/30*4095)
            b=bytearray.fromhex(mit.ZERO_WIRE)
            b[6]=(b[6]&240)|(code>>8);b[7]=code&255
            actual,decoded=decode_command(b)
            self.assertEqual(decoded,code)
            self.assertLessEqual(actual,command+1e-12)
            self.assertLess(command-actual,LSB+1e-12)

class FixtureScopeTests(unittest.TestCase):
    def test_two_private_endpoints(self):
        import tempfile,xml.etree.ElementTree as ET
        from pathlib import Path
        from ref01_mit_fixture import MitFixture,PLUGIN,JOINTS
        x=MitFixture();x.ports=[dict(mid=104,path="/dev/pts/901"),dict(mid=105,path="/dev/pts/902")]
        xml,params=x.configuration(Path.cwd(), "psi",3.,"/ph_exo_manager_unit")
        r=ET.fromstring(xml)
        self.assertEqual({c.text for c in r.findall("./ros2_control/hardware/plugin")},{PLUGIN})
        self.assertEqual({c.get("name") for c in r.findall("./ros2_control/joint")},set(JOINTS))
        self.assertEqual(params["/ph_exo_manager_unit/controller_manager"]["ros__parameters"]["update_rate"],200)
        self.assertEqual({p.text for p in r.findall("./ros2_control/hardware/param[@name='command_hard_ttl_ns']")},{"6000000"})
        self.assertEqual({p.text for p in r.findall("./ros2_control/hardware/param[@name='feedback_ttl_ns']")},{"60000000"})
        x.ports[0]["path"]="/dev/ttyACM0"
        with self.assertRaises(ValueError):x.configuration(Path.cwd(),"psi",3.,"/ph_exo_manager_unit")


class FaultAcceptanceTests(unittest.TestCase):
    def fixture(self):
        from ref01_mit_fixture import MitFixture
        from ref01_manager_validation import REQUIRED_MANAGER_CHECKS
        x=MitFixture();x.case="loss";x.duration=3.;x.cut_ns=2000000000
        x.rows=[dict(time_s=i*.005,tau104_nm=0.,tau105_nm=0.) for i in range(601)]
        x.tx=[dict(ns=t,drive=mid) for mid in (104,105) for t in (1990000000,2030000000)]
        x.rx=[dict(ns=2600000000,drive=mid) for mid in (104,105)]
        return x,dict(checks={k:True for k in REQUIRED_MANAGER_CHECKS}),[dict(mode=2,reason_name="none")]
    def test_bounded_stop(self):
        x,e,s=self.fixture();self.assertTrue(x.summarize(0,e,s)["passed"])
    def test_unrelated_early_stop_rejected(self):
        x,e,s=self.fixture()
        for r in x.tx:r["ns"]-=1000000000
        self.assertFalse(x.summarize(0,e,s)["passed"])
    def test_restart_rejected(self):
        x,e,s=self.fixture();x.tx.append(dict(ns=2700000000,drive=104))
        self.assertFalse(x.summarize(0,e,s)["passed"])
    def test_no_returned_feedback_rejected(self):
        x,e,s=self.fixture();x.rx.clear()
        self.assertFalse(x.summarize(0,e,s)["passed"])
    def test_nonzero_watchdog_tail_rejected(self):
        x,e,s=self.fixture();x.rows[-1]["tau104_nm"]=.001
        self.assertFalse(x.summarize(0,e,s)["passed"])
    def test_missing_manager_evidence_rejected(self):
        x,e,s=self.fixture();self.assertFalse(x.summarize(0,{},s)["passed"])

if __name__=="__main__":unittest.main()
