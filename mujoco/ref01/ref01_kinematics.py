"""Public, dependency-free REF01 inverse kinematics (Python 3.10+).

Input: fixed MATLAB frame output angles psi, beta, with R6=Rz(psi)Rx(beta).
Output: mechanism input angles alpha1, alpha2, in the same requested unit.
Selected branch: psi in [-45,45] degrees, beta in [0,45] degrees.
No simulation, files, hardware access or motor commands are performed on import/call.
"""
import math
from numbers import Real

__all__ = ['inverse_kinematics', 'inverse_kinematics_many', 'KinematicsError']


class KinematicsError(ValueError):
    """Invalid input or pose outside the supported REF01 assembly branch."""


def inverse_kinematics(psi: float, beta: float, *, unit: str = 'deg') -> tuple[float, float]:
    """Return (alpha1, alpha2); input and output both use ``unit`` (deg or rad).

    alpha1 is positive from -x towards +y; alpha2 from +x towards +y.
    These are mechanism input angles, not motor encoder readings or motor torque.
    Raises KinematicsError for nonfinite, nonnumeric or out-of-domain input.
    """
    if unit not in ('deg', 'rad'):
        raise KinematicsError("unit must be 'deg' or 'rad'")
    for name, value in (('psi', psi), ('beta', beta)):
        if isinstance(value, bool) or not isinstance(value, Real):
            raise KinematicsError(f'{name} must be a real number')
        if not math.isfinite(value):
            raise KinematicsError(f'{name} must be finite')
    psi, beta = float(psi), float(beta)
    if unit == 'deg':
        psi, beta = math.radians(psi), math.radians(beta)
    limit = math.pi/4
    tol = 1e-12  # radians, floating-point boundary tolerance only
    if not -limit-tol <= psi <= limit+tol or not -tol <= beta <= limit+tol:
        raise KinematicsError('Supported branch: psi in [-45,45] deg; beta in [0,45] deg')
    psi, beta = max(-limit,min(limit,psi)), max(0.,min(limit,beta))
    # Frozen REF01 geometry, SI metres; same parameters as the MATLAB baseline.
    r, L, l, a, h, d = .060, .236, .200, .020, .060, .0913
    Y = h+l*math.cos(beta)
    Z = l*math.sin(beta)-d
    rho = math.hypot(Y,a)
    cosine = (r*r+rho*rho+Z*Z-L*L)/(2*r*rho)
    if not -1-tol <= cosine <= 1+tol:
        raise KinematicsError('Requested pose has no real solution for REF01 geometry')
    phi = math.atan2(a,Y)+math.acos(max(-1.,min(1.,cosine)))
    alpha = (math.pi/2-phi-psi, math.pi/2-phi+psi)
    return tuple(math.degrees(x) for x in alpha) if unit == 'deg' else alpha


def inverse_kinematics_many(poses, *, unit: str = 'deg') -> list[tuple[float, float]]:
    """Convert iterable of (psi,beta) to a list of (alpha1,alpha2), preserving order.

    Fails on the first invalid row; no partial result is returned.
    """
    if unit not in ('deg', 'rad'):
        raise KinematicsError("unit must be 'deg' or 'rad'")
    results = []
    for index, pose in enumerate(poses):
        try:
            psi, beta = pose
            results.append(inverse_kinematics(psi, beta, unit=unit))
        except (ValueError, TypeError) as exc:
            raise KinematicsError(f'pose[{index}]: {exc}') from exc
    return results


if __name__ == '__main__':
    import argparse
    import json
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--psi', type=float, required=True)
    parser.add_argument('--beta', type=float, required=True)
    parser.add_argument('--unit', choices=('deg','rad'), default='deg')
    args = parser.parse_args()
    try:
        alpha1, alpha2 = inverse_kinematics(args.psi,args.beta,unit=args.unit)
    except KinematicsError as exc:
        parser.error(str(exc))
    print(json.dumps(dict(psi=args.psi,beta=args.beta,alpha1=alpha1,alpha2=alpha2,unit=args.unit),allow_nan=False))
