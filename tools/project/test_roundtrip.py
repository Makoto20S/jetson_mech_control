import math
import unittest
from ph_exo_bench_worker import DEFAULT,RoundTrip,TargetPose,make_trajectory,validate_config,TargetPoseRejected

class RoundTripTests(unittest.TestCase):
    def config(self,**kw):return dict(DEFAULT,amp_deg=[3.,-2.],period_s=[6.,4.],**kw)
    def test_mode_selection_and_defaults(self):
        self.assertIsNone(make_trajectory(DEFAULT))
        self.assertIsInstance(make_trajectory(dict(DEFAULT,pose_deg=[0,20],move_s=3)),TargetPose)
        self.assertIsInstance(make_trajectory(self.config()),RoundTrip)
        self.assertEqual([DEFAULT[k] for k in ('k_psi','k_beta','b_psi','b_beta')],[20,20,.3,.3])
    def test_reject_bad_input(self):
        for extra in ({'amp_deg':[1,2]}, {'period_s':[2,3]}, {'amp_deg':[0,0],'period_s':[6,6]},
                      {'amp_deg':[True,3],'period_s':[6,6]}, {'amp_deg':[3,float('nan')],'period_s':[6,6]},
                      {'amp_deg':[46,3],'period_s':[6,6]}, {'amp_deg':[3,3],'period_s':[.99,6]},
                      {'amp_deg':[3,3],'period_s':[6,float('inf')]}, {'amp_deg':None,'period_s':None}):
            with self.subTest(extra=extra),self.assertRaises(ValueError):validate_config(dict(DEFAULT,**extra))
    def test_base_extrema_period_and_derivative(self):
        tr=RoundTrip(self.config());home=[0.,math.radians(20)]
        self.assertEqual(tr.sample(1,home),(home,[0,0]))
        self.assertAlmostEqual(tr.sample(2.5,home)[0][0],math.radians(3))
        self.assertAlmostEqual(tr.sample(3,home)[0][1],math.radians(18))
        for t in (1,13,25,12001):
            q,v=tr.sample(t,home)
            for i in (0,1):self.assertAlmostEqual(q[i],home[i],places=11);self.assertAlmostEqual(v[i],0,places=11)
        for tick in range(1201):
            t=1+tick*.01;q,v=tr.sample(t,home);h=1e-5
            lo=tr.sample(max(0,t-h),home)[0];hi=tr.sample(t+h,home)[0]
            for i in (0,1):self.assertAlmostEqual((hi[i]-lo[i])/(2*h),v[i],places=7)
            self.assertTrue(-math.radians(3)-1e-12<=q[0]<=math.radians(3)+1e-12)
            self.assertTrue(math.radians(18)-1e-12<=q[1]<=home[1]+1e-12)
    def test_full_domain_one_second(self):
        home=[0., math.radians(-8)]
        config=dict(DEFAULT, amp_deg=[45.,53.], period_s=[1.,1.])
        trajectory=RoundTrip(config)
        for t,expected in [(1,(0,-8)),(1.25,(45,18.5)),(1.5,(0,45)),(1.75,(-45,18.5)),(2,(0,-8))]:
            q,_=trajectory.sample(t,home)
            for actual,target in zip(q,expected):self.assertAlmostEqual(math.degrees(actual),target)
        with self.assertRaises(ValueError):trajectory.validate([-.01,home[1]])
        with self.assertRaises(ValueError):trajectory.validate([0.,0.])

    def test_approach_then_cycle(self):
        tr=RoundTrip(self.config(pose_deg=[1,21],move_s=3));home=[0,math.radians(20)]
        self.assertEqual(tr.startup_s,4)
        q,v=tr.sample(4,home);self.assertEqual(q,tr.approach.goal);self.assertEqual(v,[0,0])
        self.assertAlmostEqual(tr.sample(5.5,home)[0][0],math.radians(4))
        for t in (1,4,6,7,8,10):
            a=tr.sample(t-1e-6,home);b=tr.sample(t+1e-6,home)
            self.assertLess(max(abs(a[0][i]-b[0][i]) for i in (0,1)),1e-6)
            self.assertLess(max(abs(a[1][i]-b[1][i]) for i in (0,1)),1e-6)
    def test_single_axis_and_domain(self):
        cfg=self.config();cfg['amp_deg']=[0,3];tr=RoundTrip(cfg)
        for t in (0,1,2,3,12):self.assertEqual(tr.sample(t,[.1,.2])[0][0],.1)
        with self.assertRaises(TargetPoseRejected):tr.validate([0,math.radians(44)])
        with self.assertRaises(ValueError):validate_config(dict(cfg,pose_deg=[0,44],move_s=3))
    def test_entire_encoder_envelope(self):
        tr=RoundTrip(self.config());home=[0,math.radians(20)];tr.validate(home)
        import sys
        from pathlib import Path
        sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'mujoco/ref01'))
        from ref01_kinematics import inverse_kinematics
        a1,a2=inverse_kinematics(*home,unit='rad');enc={104:math.degrees(a2)+45,105:math.degrees(a1)+45}
        tr.validate_encoders(enc)
        small=self.config();small['travel_deg']=1;limited=RoundTrip(small);limited.validate(home)
        with self.assertRaises(TargetPoseRejected):limited.validate_encoders(enc)
    def test_private_fixture_enforces_configured_cap_and_zero_gains(self):
        from ref01_bench_hold import validate_rehearsal_command
        import struct
        def body(nm):
            code=int((nm+15)/30*4095)
            return struct.pack('<IBB',0x800+104,0x0c,8)+bytes.fromhex(__import__('mit_communication').ZERO_WIRE)[:6]+bytes([(bytes.fromhex(__import__('mit_communication').ZERO_WIRE)[6]&0xf0)|(code>>8),code&255])
        for torque in (-10,-.3,0,.3,10):validate_rehearsal_command(body(torque),10.)
        for torque in (-.3,.3):
            with self.assertRaises(ValueError):validate_rehearsal_command(body(torque),.2)
        with self.assertRaises(ValueError):validate_rehearsal_command(body(10.1),10.)
        bad=bytearray(body(0));bad[11]=1
        with self.assertRaises(ValueError):validate_rehearsal_command(bad,10.)
    def test_no_auto_expiration_or_gain_changes(self):
        tr=RoundTrip(self.config());self.assertFalse(tr.expects_reference_loss)
        tr.sample(1e6,[0,.2]);self.assertEqual(tr.config['k_psi'],20);self.assertEqual(tr.config['b_beta'],.3)

if __name__=='__main__':unittest.main()


class RosArrayReferenceTests(unittest.TestCase):
    def test_ros_fixed_array_and_numpy_scalars(self):
        import numpy as np
        from mech_assist_control.robot.ph_exo_reference import RoundTrip as Shared,pose
        tr=Shared([.05,-.03],[6.,4.]);q=np.array([0.,.2],dtype=np.float64)
        tr.validate(q)
        self.assertEqual(tr.sample(1.,q),([0.,.2],[0.,0.]))
        self.assertAlmostEqual(tr.sample(2.5,tuple(q))[0][0],.05)
        for bad in ([True,.2],[0.,np.nan],np.array([0.,-.15]),[0.],None):
            with self.assertRaises(ValueError):pose(bad)


class RosParameterReferenceTests(unittest.TestCase):
    def test_parameter_service_arrays_are_normalized(self):
        from array import array
        import numpy as np
        from mech_assist_control.robot.ph_exo_reference import DEFAULT,validate
        for kind in (lambda v:array('d',v),np.array,list,tuple):
            c=dict(DEFAULT,ref='cycle',q=kind([0,.2]),amp=kind([.05,-.03]),period=kind([6,4]))
            result=validate(c)
            self.assertEqual(result['q'],[0.,.2]);self.assertEqual(result['amp'],[.05,-.03])
            self.assertTrue(all(type(x) is float for x in result['period']))
        for v in ([True,1],['0','1'],[float('nan'),1],None):
            with self.assertRaises(ValueError):validate(dict(DEFAULT,amp=v))
