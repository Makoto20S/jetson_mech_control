"""Portable velocity, state gating and reconstructed-state impedance regression."""
import json
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT/'mujoco/ref01'))
sys.path.insert(0, str(ROOT/'mujoco/ref01/tools'))
from ref01_dynamics_api import REF01Dynamics
from ref01_mujoco_kinematics import MuJoCoForwardKinematics, KinematicsError
from fixed_pose_impedance import FixedPoseImpedance
from state_impedance import StateImpedance
from verify_fixed_pose import REFERENCE, STIFFNESS, DAMPING, LIMIT, experiment


class VelocityTests(unittest.TestCase):
    def test_velocity_against_native_model_and_finite_difference(self):
        model, fk = REF01Dynamics(), MuJoCoForwardKinematics()
        maximum = 0.
        for psi in (-30., 0., 30.):
            for beta in (5., 20., 40.):
                q = np.deg2rad([psi,beta])
                for dq in ([0.,0.],[.3,-.2],[-.2,.4]):
                    state = model.reset(q,dq)
                    speed,condition = fk.velocity(state['alpha'],state['alpha_velocity'])
                    np.testing.assert_allclose(speed,state['dq'],atol=1e-9,rtol=0)
                    h = 1e-4  # Above the position projection tolerance; central difference.
                    plus = np.array(fk.solve(state['alpha']+h*state['alpha_velocity']).q)
                    minus = np.array(fk.solve(state['alpha']-h*state['alpha_velocity']).q)
                    np.testing.assert_allclose(speed,(plus-minus)/(2*h),atol=1e-7,rtol=0)
                    maximum = max(maximum,float(np.max(np.abs(speed-state['dq']))))
                    self.assertLess(condition,1e8)
        print('27 native velocity poses, max rad/s error',maximum)

    def test_invalid_and_ill_conditioned_velocity(self):
        fk = MuJoCoForwardKinematics()
        alpha = REF01Dynamics().reset(REFERENCE)['alpha']
        with self.assertRaises(KinematicsError):
            fk.velocity(alpha,[np.nan,0.])
        from unittest.mock import patch
        with patch('ref01_mujoco_kinematics.np.linalg.cond',return_value=1e10):
            with self.assertRaises(KinematicsError):
                fk.velocity(alpha,[0.,0.])

    def test_reconstructed_state_restores_both_axes(self):
        import verify_fixed_pose as verify
        class Reconstructed(FixedPoseImpedance):
            def __init__(self,*args):
                super().__init__(*args)
                self.fk = MuJoCoForwardKinematics()
            def command(self,q,dq):
                # Independent native simulator motor coordinates and velocities.
                state = self.model.simulation_state()
                pose = self.fk.solve(state['alpha']).q
                velocity,_ = self.fk.velocity(state['alpha'],state['alpha_velocity'])
                return super().command(pose,velocity)
        model = REF01Dynamics()
        controller = Reconstructed(model,REFERENCE,STIFFNESS,DAMPING,LIMIT)
        for axis in (0,1):
            rows,state = verify.run_case(model,controller,axis)
            result = verify.assess(rows,state,axis)
            self.assertTrue(result['passed'],result)
            print('Reconstructed axis',axis,json.dumps(result))


class GuardTests(unittest.TestCase):
    def setUp(self):
        self.controller = FixedPoseImpedance(REF01Dynamics(),REFERENCE,STIFFNESS,DAMPING,LIMIT)
        self.gate = StateImpedance(self.controller)
        self.now = 10_000_000_000
    def state(self,stamp=None):
        value = self.now if stamp is None else stamp
        source = SimpleNamespace(valid=True, source_stamp=SimpleNamespace(sec=value//10**9,nanosec=value%10**9))
        return SimpleNamespace(sources=[source],mapping_confirmed=True,q_valid=True,
                               q_velocity_valid=True,q_position=REFERENCE,q_velocity=[0.,0.])
    def test_duplicate_cannot_refresh_and_recovery_stays_stopped(self):
        s = self.state()
        self.gate.update(s,self.now,self.now)
        self.assertIsNotNone(self.gate.command(self.now,self.now))
        for age in (10,30,50):
            self.gate.update(s,self.now+age*10**6,self.now+age*10**6)
            self.assertIsNone(self.gate.command(self.now+age*10**6,self.now+age*10**6))
        self.assertIsNone(self.gate.command(self.now+61*10**6,self.now+61*10**6))
        self.assertTrue(self.gate.tripped)
        new = self.now+70*10**6
        self.gate.update(self.state(new),new,new)
        self.assertIsNone(self.gate.command(new,new))
    def test_invalid_velocity_stops_active_controller(self):
        self.gate.update(self.state(),self.now,self.now)
        self.gate.command(self.now,self.now)
        s = self.state(self.now+1)
        s.q_velocity_valid = False
        self.gate.update(s,self.now+1,self.now+1)
        self.assertIsNone(self.gate.command(self.now+1,self.now+1))
        self.assertTrue(self.gate.tripped)


if __name__ == '__main__':
    unittest.main()
