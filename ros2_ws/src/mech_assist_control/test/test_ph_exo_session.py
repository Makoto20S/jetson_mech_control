import math
import unittest
from mech_assist_control.robot.ph_exo_session import FixedPoseSession

class SessionTests(unittest.TestCase):
    def new(self):
        s=FixedPoseSession();s.start([0.,.2],[10.,10.,0.,0.],[10.,10.],1.)
        return s
    def test_requires_explicit_start_and_reset(self):
        s=FixedPoseSession();self.assertIsNone(s.plan(1.))
        s.start([0.,.2],[10.,10.,0.,0.],[10.,10.],1.)
        with self.assertRaises(ValueError):s.start([0.,.2],[10.,10.,0.,0.],[10.,10.],1.)
        s.stop("急停");self.assertIsNone(s.plan(2.))
        with self.assertRaises(ValueError):s.reset()
        s.completed(True,[True,True]);s.reset()
        self.assertEqual(s.phase,"debug");self.assertIsNone(s.plan(3.))
    def test_missing_ack_blocks_reset(self):
        for ack in ([True,False],[False,True],[False,False]):
            s=self.new();s.stop("stop");s.completed(True,ack)
            self.assertEqual(s.phase,"unconfirmed")
            with self.assertRaises(ValueError):s.reset()
    def test_simulated_stop_is_not_physical_ack(self):
        s=self.new();s.stop("stop");s.completed(True,[False,False],True)
        self.assertTrue(s.stop_confirmed);self.assertEqual(s.disable_confirmed,[False,False])
        s.reset()
    def test_immutable_plan_and_new_session(self):
        s=self.new();p=s.plan(1.1);q=s.plan(1.2)
        self.assertEqual(p["reference_rad"],q["reference_rad"])
        self.assertGreater(q["sequence"],p["sequence"])
        old=s.session_id;s.stop("stop");s.completed(True,[True,True]);s.reset()
        s.start([.1,.3],[2.,2.,.1,.1],[.2,.2],2.)
        self.assertGreater(s.session_id,old);self.assertEqual(s.sequence,0)
    def test_invalid_ranges_and_dimensions(self):
        for ref,gains,caps in (([0.,-.15],[10,10,0,0],[10,10]),
                               ([0.,.2],[31,10,0,0],[10,10]),
                               ([0.,.2],[10,10,-1,0],[10,10]),
                               ([0.,.2],[10,10,0,0],[11,10]),
                               ([0.,.2],[10,10,0,0],[math.nan,10]),
                               ([0.],[10,10,0,0],[10,10])):
            with self.assertRaises(ValueError):FixedPoseSession().start(ref,gains,caps,1.)
if __name__=="__main__":unittest.main()
