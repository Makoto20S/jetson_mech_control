import math
import unittest
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from ref01_dynamic_mit import fixture,current_scale,tracking
from ref01_dynamic_validation import reference
from ref01_mit_fixture import MitFixture,ERPM_SCALE,feedback_words

class Contract(unittest.TestCase):
    def test_explicit_current_mapping_and_legacy_preserved(self):
        scale,mapping=current_scale()
        self.assertEqual(scale,mapping['axes']['104']['rad_s_per_erpm'])
        f=fixture()
        self.assertEqual(f.feedback_period_s,.005)
        self.assertEqual(f.erpm_scale,scale)
        self.assertEqual(f.pulse_nm,0.)
        self.assertEqual(MitFixture().erpm_scale,ERPM_SCALE)
        self.assertEqual(MitFixture().feedback_period_s,.02)
        p,v=feedback_words([0.,0.],[scale*20,-scale*30],scale)
        self.assertEqual(v,[-3,2])

    def test_only_private_endpoints_receive_scale(self):
        f=fixture();f.ports=[dict(mid=104,path='/dev/pts/901'),dict(mid=105,path='/dev/pts/902')]
        xml,_=f.configuration(Path.cwd(),'psi',12.,'/ph_exo_manager_contract')
        r=ET.fromstring(xml)
        self.assertEqual({float(p.text) for p in r.findall("./ros2_control/hardware/param[@name='kinematic_erpm_to_rad_s']")},{current_scale()[0]})
        f.ports[0]['path']='/dev/ttyACM0'
        with self.assertRaises(ValueError):f.configuration(Path.cwd(),'psi',12.,'/ph_exo_manager_contract')
    def rows(self):
        rows=[]
        for t in np.arange(0.,12.005,.005):
            q,v=reference(t,0)
            rows.append([t,*q,*v,*q,*v,.01,-.01,0.])
        return np.array(rows)
    def test_exact_tracking_and_zero_tail_passes(self):
        self.assertTrue(tracking(self.rows())['passed'])
    def test_finite_offset_cannot_pass(self):
        a=self.rows();a[:,1]+=.01
        self.assertFalse(tracking(a)['checks']['tracking_rmse'])
    def test_speed_violation_cannot_hide_in_position_band(self):
        a=self.rows();a[a[:,0]>=9.6,3]=math.radians(1.01)
        self.assertFalse(tracking(a)['checks']['recovered_both_axes'])
    def test_missing_motion_cannot_pass(self):
        a=self.rows();a[:,1]=0.;a[:,5]=0.
        self.assertFalse(tracking(a)['checks']['trajectory_exercised'])
if __name__=='__main__':unittest.main()
