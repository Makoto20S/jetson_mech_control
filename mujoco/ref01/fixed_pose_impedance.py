"""Fixed-pose PH-EXO joint-space impedance for REF-01, in radians and N m."""
from dataclasses import dataclass
import time

import numpy as np

from ref01_kinematics import inverse_kinematics


def pair(value, name):
    result = np.asarray(value, dtype=float)
    if result.shape != (2,) or not np.isfinite(result).all():
        raise ValueError(f'{name} requires two finite values')
    return result.copy()


@dataclass(frozen=True)
class TorqueCommand:
    # alpha1 drives motor 105; alpha2 drives motor 104 (nominal +1 mapping).
    alpha_nm: np.ndarray
    gravity_q_nm: np.ndarray
    feedback_q_nm: np.ndarray
    scale: float

    @property
    def motor104_105_nm(self):
        return self.alpha_nm[::-1].copy()


class FixedPoseImpedance:
    """Reference and diagonal gains in [psi, beta], caps in [alpha1, alpha2].

    Requires a valid model-domain pose and velocity. This simulation controller
    returns a command; hardware authorization and transport live elsewhere.
    """
    def __init__(self, model, reference_rad, stiffness, damping, limit_alpha_nm, *, psi_ramp_s=.2):
        self.model = model
        self.psi_ramp_s = psi_ramp_s
        self._outside = False
        self._returned = None
        self.reference = pair(reference_rad, 'reference')
        if abs(self.reference[0]) > np.pi/4 + 1e-12:
            raise ValueError('psi reference outside ±45°')
        if not np.deg2rad(-8) - 1e-12 <= self.reference[1] <= np.pi/4 + 1e-12:
            raise ValueError('beta reference outside [-8,45] degrees')
        self.stiffness = pair(stiffness, 'stiffness')
        self.damping = pair(damping, 'damping')
        self.limits = pair(limit_alpha_nm, 'torque limits')
        if np.any(self.stiffness <= 0) or np.any(self.damping < 0) or np.any(self.limits <= 0):
            raise ValueError('Require positive stiffness and limits, nonnegative damping')

    def command(self, q_rad, dq_rad_s, *, now=None):
        q = pair(q_rad, 'q')
        dq = pair(dq_rad_s, 'dq')
        # At zero velocity, bias is the gravity torque in output coordinates.
        terms = self.model.dynamics_terms(q, (0., 0.))
        gravity = terms['bias']
        feedback = self.stiffness * (self.reference - q) - self.damping * dq
        now = time.monotonic() if now is None else now
        if abs(q[0]) > np.pi/4 + 1e-8:
            self._outside = True
            self._returned = None
            feedback[0] = 0.
        else:
            if self._outside:
                self._outside = False
                self._returned = now
            if self._returned is not None and self.psi_ramp_s > 0.:
                u = np.clip((now-self._returned)/self.psi_ramp_s, 0., 1.)
                feedback[0] *= u**3*(10.+u*(-15.+6.*u))
                if u == 1.:
                    self._returned = None
        if feedback[1] > 0.:
            alpha = inverse_kinematics(*q, unit='rad', observed_pose=True)
            phi = np.pi/2 - (alpha[0] + alpha[1])/2
            u = np.clip((np.pi/2 - phi) / np.deg2rad(2.), 0., 1.)
            feedback[1] *= u*u*(3. - 2.*u)
        demand = np.linalg.solve(terms['A'].T, gravity + feedback)
        if not np.isfinite(demand).all():
            raise ValueError('Nonfinite actuator demand')
        # A common factor preserves the direction of the total generalized torque.
        scale = 1. / max(1., float(np.max(np.abs(demand) / self.limits)))
        return TorqueCommand(demand * scale, gravity, feedback, scale)
