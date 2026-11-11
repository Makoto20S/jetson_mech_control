"""Motor-angle FK using the REF01 MJCF equality constraints, without simulation.

Only MuJoCo supplies geometry, constraint residuals and Jacobians. A bounded
Newton projection pins alpha1/alpha2 and solves the six dependent hinge angles.
No analytic REF01 IK/FK, mj_step, actuator commands, or IMU inputs are used.
One solver instance per thread. All public angles are radians.
"""
from dataclasses import dataclass
from functools import lru_cache
from pathlib import Path
import math
import numbers
import mujoco
import numpy as np

DEFAULT_MODEL = Path(__file__).resolve().parent / 'config/mujoco/ref01_dynamics_cad_v1.xml'


class KinematicsError(ValueError):
    """Input, branch or closure failure; never reuse the previous output as valid."""


@dataclass(frozen=True)
class ForwardResult:
    q: tuple
    residual_m: float
    iterations: int


@lru_cache(maxsize=4)
def _model(path):
    # Python file I/O also supports non-ASCII Windows checkout paths.
    path = Path(path)
    assets = {p.name: p.read_bytes() for p in (path.parent / 'assets/ref01_cad_v1').glob('*.stl')}
    model = mujoco.MjModel.from_xml_string(path.read_text(encoding='utf-8'), assets=assets)
    if model.nq != 8 or model.nv != 8 or model.neq != 2:
        raise ValueError('Expected the REF01 eight-hinge, two-connect model')
    if np.any(model.jnt_type != int(mujoco.mjtJoint.mjJNT_HINGE)):
        raise ValueError('REF01 projection requires hinge coordinates')
    if np.any(model.eq_type != int(mujoco.mjtEq.mjEQ_CONNECT)):
        raise ValueError('REF01 projection requires connect equalities')
    model.opt.jacobian = mujoco.mjtJacobian.mjJAC_DENSE
    return model


class MuJoCoForwardKinematics:
    tolerance_m = 1e-11
    max_iterations = 20
    max_alpha_step_rad = .15
    branch_tolerance_rad = 1e-8

    def __init__(self, model_path=None):
        self.model = _model(str(Path(model_path or DEFAULT_MODEL).resolve()))
        self.data = mujoco.MjData(self.model)
        self.active = np.array([self.model.joint(n).qposadr[0] for n in ('alpha1', 'alpha2')])
        self.output = np.array([self.model.joint(n).qposadr[0] for n in ('psi', 'beta')])
        self.dependent = np.array([i for i in range(self.model.nq) if i not in self.active])
        self.reset()

    def reset(self):
        self._seed = self.model.qpos0.copy()
        self._last_alpha = None
        self._last_result = None
        self.data.qpos[:] = self._seed

    def _constraints(self):
        mujoco.mj_fwdPosition(self.model, self.data)
        if self.data.nefc != 6 or not np.all(np.isfinite(self.data.efc_pos)):
            raise KinematicsError('unexpected_constraints')
        return self.data.efc_pos.copy(), self.data.efc_J.reshape(6, self.model.nv)[:, self.dependent].copy()

    def _project(self, alpha):
        self.data.qpos[self.active] = alpha
        for iteration in range(self.max_iterations + 1):
            residual, jac = self._constraints()
            norm = float(np.max(np.abs(residual)))
            if norm <= self.tolerance_m:
                return norm, iteration
            if iteration == self.max_iterations:
                break
            try:
                step = np.linalg.solve(jac, -residual)
            except np.linalg.LinAlgError as exc:
                raise KinematicsError('singular_constraint_jacobian') from exc
            if not np.all(np.isfinite(step)):
                raise KinematicsError('nonfinite_projection')
            step *= min(1., .25 / max(float(np.max(np.abs(step))), 1e-15))
            previous = self.data.qpos.copy()
            # Bounded backtracking; never adjust measured active coordinates.
            for power in range(8):
                self.data.qpos[self.dependent] = previous[self.dependent] + step * (0.5 ** power)
                trial, _ = self._constraints()
                if np.max(np.abs(trial)) < norm:
                    break
            else:
                raise KinematicsError('closure_not_converged')
        raise KinematicsError('iteration_limit')

    def solve(self, alpha):
        try:
            valid = (len(alpha) == 2 and all(
                isinstance(x, numbers.Real) and not isinstance(x, (bool, np.bool_))
                and math.isfinite(x) and abs(x) <= math.pi for x in alpha))
        except TypeError:
            valid = False
        if not valid:
            raise KinematicsError('expected_two_finite_principal_angles')
        target = np.asarray(alpha, dtype=float)
        if self._last_alpha is not None and np.array_equal(target, self._last_alpha):
            return self._last_result
        self.data.qpos[:] = self._seed
        start = self._seed[self.active].copy()
        count = max(1, math.ceil(float(np.max(np.abs(target - start))) / self.max_alpha_step_rad))
        iterations = 0
        try:
            for index in range(1, count + 1):
                residual, used = self._project(target if index == count else start + (target - start) * (index / count))
                iterations += used
            q = self.data.qpos[self.output].copy()
            tol = self.branch_tolerance_rad
            if not (-math.pi/4-tol <= q[0] <= math.pi/4+tol and -tol <= q[1] <= math.pi/4+tol):
                raise KinematicsError('outside_ref01_branch')
            # Return computed values without angle wrapping or boundary clamping.
            result = ForwardResult(tuple(float(x) for x in q), residual, iterations)
        except KinematicsError:
            self.data.qpos[:] = self._seed
            raise
        self._seed = self.data.qpos.copy()
        self._last_alpha, self._last_result = target.copy(), result
        return result
