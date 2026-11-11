"""Callable REF01 closed-chain dynamics backed by the CAD MuJoCo model.

q=[psi,beta] (rad), dq (rad/s), ddq (rad/s²).
tau=[tau_alpha1,tau_alpha2] is mechanism input-shaft torque (N m).
Algebraic forward/inverse methods use ideal closed-chain constraint projection.
reset/step run the native MuJoCo soft-constraint simulation on separate state.
"""
from pathlib import Path
import hashlib
import numpy as np
import mujoco
from ref01_kinematics import inverse_kinematics
from tools.ref01_v1 import universal_angles

ROOT = Path(__file__).resolve().parent


def _pair(value, name):
    try: a = np.asarray(value, dtype=float)
    except (ValueError, TypeError) as exc: raise ValueError(f'{name} must be two finite numbers') from exc
    if a.shape != (2,) or not np.isfinite(a).all():
        raise ValueError(f'{name} must have shape (2,) and contain finite values')
    return a.copy()


class REF01Dynamics:
    """Construct once, reuse per call; use one instance per thread.

    Loads the existing CAD candidate XML without changing files or starting a viewer.
    Ideal dynamics ignore actuator saturation; native step uses XML actuator limits.
    external_torque is conjugate to [psi,beta], not Cartesian XYZ torque.
    """
    def __init__(self):
        path = ROOT/'config/mujoco/ref01_dynamics_cad_v1.xml'
        xml = path.read_text(encoding='utf-8')
        assets = {p.name:p.read_bytes() for p in (ROOT/'config/mujoco/assets/ref01_cad_v1').glob('*.stl')}
        self.model = mujoco.MjModel.from_xml_string(xml, assets=assets)
        self._data = mujoco.MjData(self.model)
        self._sim = mujoco.MjData(self.model)
        self._initialized = False
        self._out = np.array([self.model.joint(n).qposadr[0] for n in ('psi','beta')])
        self._active = np.array([self.model.joint(n).qposadr[0] for n in ('alpha1','alpha2')])
        self._dependent = np.array([i for i in range(self.model.nv) if i not in self._out])
        self._actuators = np.array([self.model.actuator(f'alpha{i}_torque').id for i in (1,2)])
        self._sites = [(self.model.site(f'rod_end{i}').id, self.model.site(f'Q{i}').id) for i in (1,2)]
        if self.model.nq != 8 or self.model.nv != 8 or self.model.neq != 2:
            raise ValueError('This API requires the verified REF01 eight-hinge, two-connect model')
        if np.any(self.model.dof_frictionloss):
            raise ValueError('Ideal projection currently supports viscous damping, not dry-friction constraints')
        self.metadata = dict(model_path=str(path), model_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
                             parameter_status='CAD candidate with documented component-grouping approximations',
                             coordinates=['psi','beta'], inputs=['tau_alpha1','tau_alpha2'],
                             units='rad, s, N m', gravity_m_s2=self.model.opt.gravity.tolist())

    def _jac(self, data):
        J = []
        for rod, target in self._sites:
            a = np.zeros((3,self.model.nv)); b = a.copy()
            mujoco.mj_jacSite(self.model,data,a,None,rod)
            mujoco.mj_jacSite(self.model,data,b,None,target)
            J.append(a-b)
        return np.vstack(J)

    def _position(self, q):
        alpha = np.array(inverse_kinematics(*q,unit='rad'))
        u = universal_angles(q,alpha)
        values = dict(psi=q[0], beta=q[1], alpha1=alpha[0], alpha2=alpha[1],
                      u1_z=u[0,0],u1_x=u[0,1],u2_z=u[1,0],u2_x=u[1,1])
        x = np.zeros(self.model.nq)
        for name,value in values.items(): x[self.model.joint(name).qposadr[0]] = value
        return x

    def _terms(self, q, dq):
        q, dq = _pair(q,'q'), _pair(dq,'dq')
        d = self._data
        mujoco.mj_resetData(self.model,d)
        x = self._position(q); d.qpos[:] = x
        mujoco.mj_forward(self.model,d)
        J = self._jac(d)
        N = np.zeros((self.model.nv,2)); N[self._out] = np.eye(2)
        N[self._dependent] = np.linalg.solve(J[:,self._dependent],-J[:,self._out])
        velocity = N@dq
        # Jdot*v via central difference in the ambient hinge coordinates.
        # This derivative remains defined at the selected workspace boundaries.
        jdotv = np.zeros(6)
        if np.any(dq):
            eps = 1e-6/max(1.,float(np.linalg.norm(velocity)))
            derivatives = []
            for sign in (1,-1):
                d.qpos[:] = x+sign*eps*velocity
                mujoco.mj_kinematics(self.model,d); mujoco.mj_comPos(self.model,d)
                derivatives.append(self._jac(d))
            jdotv = ((derivatives[0]-derivatives[1])/(2*eps))@velocity
        gamma = np.zeros(self.model.nv)
        gamma[self._dependent] = np.linalg.solve(J[:,self._dependent],-jdotv)
        d.qpos[:] = x; d.qvel[:] = velocity
        mujoco.mj_forward(self.model,d)
        M = np.zeros((self.model.nv,self.model.nv)); mujoco.mj_fullM(self.model,d,M)
        b = d.qfrc_bias-d.qfrc_passive
        return dict(M=N.T@M@N, bias=N.T@(M@gamma+b), A=N[self._active].copy(),
                    N=N, gamma=gamma, J=J, jdotv=jdotv, full_M=M, full_bias=b.copy(),
                    full_q=x, full_dq=velocity)

    def dynamics_terms(self, q, dq=(0.,0.)):
        """Return M,bias,A satisfying M*ddq+bias=A.T*tau+external_torque."""
        t = self._terms(q,dq)
        return {k:t[k].copy() for k in ('M','bias','A')}

    def inverse_dynamics(self, q, dq, ddq, *, external_torque=(0.,0.)):
        """Desired output acceleration -> required input-shaft torques (2,)."""
        ddq = _pair(ddq,'ddq'); ext = _pair(external_torque,'external_torque')
        t = self._terms(q,dq)
        return np.linalg.solve(t['A'].T,t['M']@ddq+t['bias']-ext)

    def forward_dynamics(self, q, dq, tau, *, external_torque=(0.,0.)):
        """Input-shaft torques -> output acceleration (2,), ideal closed chain."""
        tau = _pair(tau,'tau'); ext = _pair(external_torque,'external_torque')
        t = self._terms(q,dq)
        return np.linalg.solve(t['M'],t['A'].T@tau+ext-t['bias'])

    def gravity_compensation(self, q):
        """Input-shaft torques that balance gravity at zero velocity."""
        return self.inverse_dynamics(q,(0.,0.),(0.,0.))

    def reset(self, q, dq=(0.,0.)):
        """Initialize the separate native simulator on the constraint manifold."""
        t = self._terms(q,dq)
        mujoco.mj_resetData(self.model,self._sim)
        self._sim.qpos[:] = t['full_q']; self._sim.qvel[:] = t['full_dq']
        mujoco.mj_forward(self.model,self._sim)
        self._initialized = True
        return self.simulation_state()

    def step(self, tau, *, steps=1, external_torque=(0.,0.)):
        """Native mj_step with constant torque for steps; default dt=0.5ms.

        This retains all eight joint states and soft closure deflections between calls.
        Returns output state, actual actuator torque and closure residual.
        """
        if not self._initialized: raise RuntimeError('Call reset(q,dq) before step()')
        if isinstance(steps,bool) or not isinstance(steps,(int,np.integer)) or steps<1:
            raise ValueError('steps must be a positive integer')
        tau = _pair(tau,'tau'); ext = _pair(external_torque,'external_torque')
        self._sim.ctrl[self._actuators] = tau
        self._sim.qfrc_applied[:] = 0
        self._sim.qfrc_applied[self._out] = ext
        for _ in range(steps): mujoco.mj_step(self.model,self._sim)
        mujoco.mj_forward(self.model,self._sim)
        return self.simulation_state()

    def simulation_state(self):
        if not self._initialized: raise RuntimeError('Call reset(q,dq) first')
        d = self._sim
        closure = max(np.linalg.norm(d.site_xpos[a]-d.site_xpos[b]) for a,b in self._sites)
        return dict(time_s=float(d.time),q=d.qpos[self._out].copy(),dq=d.qvel[self._out].copy(),
                    ddq=d.qacc[self._out].copy(),tau_applied=d.actuator_force[self._actuators].copy(),
                    max_closure_m=float(closure),warnings=int(sum(w.number for w in d.warning)))
