import math,unittest
from ph_exo_bench_worker import DEFAULT,validate_config,TargetPose,TargetPoseRejected
class Target(unittest.TestCase):
 def test_old_config_unchanged(self):self.assertEqual(validate_config(DEFAULT),DEFAULT)
 def test_invalid(self):
  for extra in [dict(pose_deg=[0,20]),dict(move_s=3),dict(pose_deg=None,move_s=None),dict(pose_deg=[0,-9],move_s=3),dict(pose_deg=[46,20],move_s=3),dict(pose_deg=[0,20],move_s=0),dict(pose_deg=[float('nan'),20],move_s=3)]:
   with self.assertRaises(ValueError):validate_config(dict(DEFAULT,**extra))
 def test_smooth_path(self):
  home=[-.1,.2];tr=TargetPose(dict(DEFAULT,pose_deg=[3,23],move_s=3))
  for t in (0,.5,1):self.assertEqual(tr.sample(t,home),(home,[0,0]))
  for t in (4,10,10000):
   q,dq=tr.sample(t,home);self.assertEqual(q,tr.goal);self.assertEqual(dq,[0,0])
  for n in range(1,300):
   t=1+n*.01;q,v=tr.sample(t,home);h=1e-5
   lo=tr.sample(t-h,home)[0];hi=tr.sample(t+h,home)[0]
   for i in (0,1):
    self.assertAlmostEqual((hi[i]-lo[i])/(2*h),v[i],places=8)
    self.assertTrue(min(home[i],tr.goal[i])<=q[i]<=max(home[i],tr.goal[i]))
  self.assertFalse(tr.expects_reference_loss)
 def test_final_encoder_envelope(self):
  tr=TargetPose(dict(DEFAULT,pose_deg=[0,20],move_s=3))
  self.assertEqual(tr.startup_s,4.)
  tr.validate_encoders({104:80.,105:80.})
  with self.assertRaises(ValueError):tr.validate_encoders({104:200.,105:200.})
 def test_preflight_rejection_type(self):
  tr=TargetPose(dict(DEFAULT,pose_deg=[30,15],move_s=2,travel_deg=60))
  with self.assertRaisesRegex(TargetPoseRejected,"104/105"):tr.validate_encoders({104:36.,105:91.})
 def test_default_90_accepts_previous_goal(self):
  tr=TargetPose(dict(DEFAULT,pose_deg=[30,15],move_s=2))
  tr.validate_encoders({104:36.,105:91.})
  self.assertEqual(DEFAULT["travel_deg"],90.)
 def test_start_range(self):
  tr=TargetPose(dict(DEFAULT,pose_deg=[0,20],move_s=3))
  for q in ([0,-.15],[1,.2],[float('nan'),.2]):
   with self.assertRaises(ValueError):tr.validate(q)
if __name__=='__main__':unittest.main()
