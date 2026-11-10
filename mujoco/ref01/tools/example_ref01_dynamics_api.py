"""Run this file with the project MuJoCo Python environment."""
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import numpy as np
from ref01_dynamics_api import REF01Dynamics

robot=REF01Dynamics()  # Load CAD MuJoCo model once; reuse this instance.
q=np.deg2rad([10,20])  # [psi,beta]
dq=np.array([0.,0.])
ddq=np.array([0.,0.])

tau=robot.inverse_dynamics(q,dq,ddq)
print('Required input-shaft torque [N m]:',tau)
print('Predicted acceleration [rad/s^2]:',robot.forward_dynamics(q,dq,tau))
print('Mass matrix in [psi,beta] coordinates:',robot.dynamics_terms(q,dq)['M'])

robot.reset(q,dq)
state=robot.step(tau,steps=20)  # 20*0.5ms=10ms native simulation
print('Native simulation time [s]:',state['time_s'])
print('Native output angles [deg]:',np.rad2deg(state['q']))
