"""Actual MuJoCo FK versus frozen MATLAB outputs; no ROS or devices."""
import json
import math
from pathlib import Path
import unittest
import numpy as np
from ref01_mujoco_kinematics import MuJoCoForwardKinematics, KinematicsError


class ForwardKinematicsTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        root = Path(__file__).resolve().parents[4]
        cls.baseline = json.loads((root / 'mujoco/ref01/fixtures/ref01_kinematics_validation_20261103/matlab_baseline.json').read_text())

    def test_frozen_grid(self):
        fk = MuJoCoForwardKinematics()
        self.assertEqual(len(self.baseline['alpha_rad']),16736)
        worst = residual = 0.
        for alpha, expected in zip(self.baseline['alpha_rad'],self.baseline['fk_rad']):
            actual = fk.solve(alpha)
            worst = max(worst,*(abs(x-y) for x,y in zip(actual.q,expected)))
            residual = max(residual,actual.residual_m)
            np.testing.assert_array_equal(fk.data.qpos[fk.active],alpha)
        self.assertLess(worst,1e-8)
        self.assertLessEqual(residual,1e-11)
        self.assertEqual(fk.data.time,0.)
        print(f'MuJoCo FK: 16736 frozen rows; max error={worst:.3g} rad; closure={residual:.3g} m')

    def test_cold_start_and_discontinuous_inputs(self):
        fk = MuJoCoForwardKinematics()
        rows = list(zip(self.baseline['alpha_rad'],self.baseline['fk_rad']))
        # Corners are part of the frozen grid; random order defeats warm-path assumptions.
        indices = {int(np.argmin(np.asarray(self.baseline['fk_rad'])[:,j])) for j in (0,1)}
        indices |= {int(np.argmax(np.asarray(self.baseline['fk_rad'])[:,j])) for j in (0,1)}
        indices = list(indices) + list(np.random.default_rng(47).choice(len(rows),200,replace=False))
        for i in indices:
            alpha,expected = rows[i]
            for cold in (False,True):
                if cold:
                    fk.reset()
                actual = fk.solve(alpha)
                self.assertLess(max(abs(x-y) for x,y in zip(actual.q,expected)),1e-8)

    def test_invalid_inputs_and_failed_projection_recovery(self):
        fk = MuJoCoForwardKinematics()
        alpha,expected = self.baseline['alpha_rad'][0],self.baseline['fk_rad'][0]
        good = fk.solve(alpha)
        bad = [None,[],[0.], [0.,0.,0.]]
        bad += [[x,0.] for x in (True,'1',None,complex(1,2),math.nan,math.inf,-math.inf,1e308)]
        bad += [np.deg2rad(x) for x in [(-45.,-45.),(90.,90.),(-90.,90.),(360.,360.)]]
        for value in bad:
            with self.subTest(value=value), self.assertRaises(KinematicsError):
                fk.solve(value)
            # Failure must leave last successful seed unchanged; subsequent good
            # input is explicitly recomputed/returned as that input, never as fallback.
            np.testing.assert_array_equal(fk.data.qpos[fk.active],alpha)
            self.assertEqual(fk.solve(alpha),good)
        fk.reset()
        self.assertLess(max(abs(x-y) for x,y in zip(fk.solve(alpha).q,expected)),1e-8)

    def test_output_responds_to_mjcf_geometry(self):
        # Demonstrates that runtime actually uses MuJoCo geometry, not a formula
        # or a lookup table. Mutate only this test's private compiled model.
        fk = MuJoCoForwardKinematics()
        import copy
        import mujoco
        fk.model = copy.copy(fk.model)
        fk.model.body_pos[fk.model.body('L6').id,1] += .001
        fk.data = mujoco.MjData(fk.model)
        fk.reset()
        q = fk.solve(fk.model.qpos0[fk.active]).q
        self.assertGreater(abs(q[1]-fk.model.qpos0[fk.output[1]]),1e-4)


if __name__ == '__main__':
    unittest.main()
