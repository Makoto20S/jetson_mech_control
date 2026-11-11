"""REF01 geometric prototype. SI units, alpha sign conventions from the frozen MATLAB baseline."""
from pathlib import Path
import json
import math
import os
import xml.etree.ElementTree as ET
import numpy as np
import mujoco

ROOT = Path(__file__).resolve().parents[1]
MODEL = ROOT / 'config/mujoco/ref01_v1.xml'
PARAMS = dict(r=.060, L=.236, l=.200, a=.020, h=.060, d=.0913)
HOME = np.deg2rad([0., 20.])


def ik(q, p=PARAMS):
    psi, beta = q
    if abs(psi) > np.pi/4+1e-10 or not -1e-10 <= beta <= np.pi/4+1e-10:
        raise ValueError('Outside REF01 selected branch')
    Y = p['h']+p['l']*np.cos(beta)
    Z = p['l']*np.sin(beta)-p['d']
    rho = np.hypot(Y,p['a'])
    k = (p['r']**2+rho**2+Z**2-p['L']**2)/(2*p['r']*rho)
    phi = np.arctan2(p['a'],Y)+np.arccos(np.clip(k,-1,1))
    return np.array([np.pi/2-phi-psi,np.pi/2-phi+psi]), phi


def fk(alpha, p=PARAMS):
    psi = (alpha[1]-alpha[0])/2
    phi = np.pi/2-sum(alpha)/2
    lo, hi = ik([0,0],p)[1], ik([0,np.pi/4],p)[1]
    if abs(psi)>np.pi/4+1e-7 or not lo-1e-7<=phi<=hi+1e-7:
        raise ValueError('Outside coupled alpha domain')
    u = p['h']-p['r']*np.cos(phi)
    R = np.hypot(u,p['d'])
    s2 = p['L']**2-(p['a']-p['r']*np.sin(phi))**2
    beta = np.arccos(np.clip((s2-p['l']**2-R**2)/(2*p['l']*R),-1,1))-np.arctan2(p['d'],u)
    return np.array([psi,beta])


def rz(x):
    c,s=np.cos(x),np.sin(x)
    return np.array([[c,-s,0],[s,c,0],[0,0,1.]])


def geometry(q, alpha, p=PARAMS):
    psi,beta=q
    R=rz(psi); et,er=R[:,0],R[:,1]
    H=p['h']*er
    C=H+p['l']*np.cos(beta)*er+np.array([0,0,p['l']*np.sin(beta)])
    Q=np.array([C-p['a']*et,C+p['a']*et])
    P=np.array([[-p['r']*np.cos(alpha[0]),p['r']*np.sin(alpha[0]),p['d']],
                [p['r']*np.cos(alpha[1]),p['r']*np.sin(alpha[1]),p['d']]])
    return P,Q,H,C


def reference_jac(q,p=PARAMS):
    psi,beta=q; phi=ik(q,p)[1]
    Y=p['h']+p['l']*np.cos(beta)
    k=p['l']*((p['h']-p['r']*np.cos(phi))*np.sin(beta)+p['d']*np.cos(beta))/(p['r']*(Y*np.sin(phi)-p['a']*np.cos(phi)))
    J=np.array([[-.5,.5],[-1/(2*k),-1/(2*k)]])
    et,er=rz(psi)[:,0],rz(psi)[:,1]; ez=np.array([0,0,1])
    JC=np.column_stack((-Y*et,p['l']*(-np.sin(beta)*er+np.cos(beta)*ez)))
    JW=np.column_stack((ez,et))
    return J,JC@J,JW@J,k


def universal_angles(q,alpha):
    P,Q,_,_=geometry(q,alpha)
    result=[]
    for i,sign in enumerate([-1,1]):
        v=rz(sign*alpha[i]).T@(Q[i]-P[i])/PARAMS['L']
        result.append([math.atan2(-v[0],v[1]),math.atan2(v[2],math.hypot(v[0],v[1]))])
    return np.array(result)


def numbers(values):
    return ' '.join(format(float(v),'.17g') for v in np.atleast_1d(values))


def build():
    """Nested two-hinge ideal U joints. Their axes are a declared modeling assumption, not CAD measurements."""
    p=PARAMS; alpha,_=ik(HOME); univ=universal_angles(HOME,alpha)
    root=ET.Element('mujoco',model='REF01_v1_kinematics')
    root.append(ET.Comment('Geometric prototype: synthetic inertia, gravity/contact off; no hardware commands.'))
    ET.SubElement(root,'compiler',angle='radian',autolimits='true')
    ET.SubElement(root,'option',timestep='0.0005',gravity='0 0 0',integrator='implicitfast',solver='Newton',iterations='100',tolerance='1e-12',jacobian='dense')
    ET.SubElement(root,'statistic',center='0 0.13 0.05',extent='0.38')
    visual=ET.SubElement(root,'visual'); ET.SubElement(visual,'global',offwidth='960',offheight='720',azimuth='135',elevation='-25')
    defaults=ET.SubElement(root,'default')
    ET.SubElement(defaults,'joint',type='hinge',damping='0.01',limited='false')
    ET.SubElement(defaults,'geom',type='capsule',size='0.006',contype='0',conaffinity='0',density='1000')
    ET.SubElement(defaults,'site',size='0.009',rgba='0.95 0.95 0.95 1')
    world=ET.SubElement(root,'worldbody')
    ET.SubElement(world,'light',pos='0 -1 1',dir='0 1 -1')
    ET.SubElement(world,'light',pos='1 1 .6',dir='-1 -1 -.5',diffuse='.6 .6 .6')
    base=ET.SubElement(world,'body',name='L0')
    ET.SubElement(base,'geom',name='base',type='box',size='.045 .025 .012',pos='0 -.025 0',rgba='.4 .42 .45 1')
    # OD is a reference axis, drawn thinner than physical rods.
    ET.SubElement(base,'geom',name='OD_reference',fromto=numbers([0,0,0,0,0,p['d']]),size='.002',rgba='.6 .6 .6 .6')
    ET.SubElement(base,'site',name='O',pos='0 0 0'); ET.SubElement(base,'site',name='D',pos=numbers([0,0,p['d']]))
    def body(parent,name,pos,angle=0,axis='z'):
        quat=[math.cos(angle/2),0,0,0]; quat[{'x':1,'z':3}[axis]]=math.sin(angle/2)
        return ET.SubElement(parent,'body',name=name,pos=numbers(pos),quat=numbers(quat))
    def joint(parent,name,axis,ref):
        ET.SubElement(parent,'joint',name=name,axis=axis,ref=numbers(ref))
    def rod(parent,name,end,color,mass):
        ET.SubElement(parent,'geom',name=name,fromto=numbers([0,0,0,*end]),rgba=color,mass=str(mass))
    green='.15 .65 .4 1'
    b5=body(base,'L5',[0,0,0],HOME[0]); joint(b5,'psi','0 0 1',HOME[0]); rod(b5,'OH',[0,p['h'],0],green,.08)
    b6=body(b5,'L6',[0,p['h'],0],HOME[1],'x'); joint(b6,'beta','1 0 0',HOME[1])
    rod(b6,'HC',[0,p['l'],0],green,.15)
    ET.SubElement(b6,'geom',name='Q1_C_Q2',fromto=numbers([-p['a'],p['l'],0,p['a'],p['l'],0]),rgba=green,mass='.02')
    ET.SubElement(b6,'site',name='H',pos='0 0 0'); ET.SubElement(b6,'site',name='C',pos=numbers([0,p['l'],0]))
    for i,sign,color in [(1,-1,'.95 .38 .08 1'),(2,1,'.08 .4 .85 1')]:
        a=alpha[i-1]; uz,ux=univ[i-1]
        ET.SubElement(b6,'site',name=f'Q{i}',pos=numbers([sign*p['a'],p['l'],0]),rgba=color)
        crank=body(base,'L1' if i==1 else 'L3',[0,0,p['d']],sign*a)
        joint(crank,f'alpha{i}',f'0 0 {sign}',a)
        rod(crank,f'DP{i}',[sign*p['r'],0,0],color,.08)
        ET.SubElement(crank,'site',name=f'P{i}',pos=numbers([sign*p['r'],0,0]),rgba=color)
        yoke=body(crank,f'U{i}_ideal_yoke',[sign*p['r'],0,0],uz)
        joint(yoke,f'u{i}_z','0 0 1',uz)
        ET.SubElement(yoke,'inertial',pos='0 0 0',mass='.001',diaginertia='1e-7 1e-7 1e-7')
        link=body(yoke,'L2' if i==1 else 'L4',[0,0,0],ux,'x')
        joint(link,f'u{i}_x','1 0 0',ux)
        rod(link,f'P{i}Q{i}',[0,p['L'],0],color,.10)
        ET.SubElement(link,'site',name=f'rod_end{i}',pos=numbers([0,p['L'],0]),rgba=color,size='.005')
    eq=ET.SubElement(root,'equality')
    for i in (1,2):
        ET.SubElement(eq,'connect',name=f'S{i}_closure',site1=f'rod_end{i}',site2=f'Q{i}',solref='.002 1',solimp='.99 .999 .0001')
    actuators=ET.SubElement(root,'actuator')
    for i in (1,2):
        ET.SubElement(actuators,'position',name=f'alpha{i}_demo_servo',joint=f'alpha{i}',kp='50',kv='2',ctrllimited='true',ctrlrange='-1.25 1.60')
    # Compiler qpos0 is closed already. Home key also initializes the position servo targets.
    keys=ET.SubElement(root,'keyframe')
    qhome=[*HOME,alpha[0],*univ[0],alpha[1],*univ[1]]
    ET.SubElement(keys,'key',name='home',qpos=numbers(qhome),ctrl=numbers(alpha))
    MODEL.parent.mkdir(parents=True,exist_ok=True)
    ET.indent(root)
    ET.ElementTree(root).write(MODEL,encoding='utf-8',xml_declaration=True)
    return MODEL


class Mechanism:
    def __init__(self):
        # Read text with Python so Windows native filename handling never sees the Chinese root.
        self.m=mujoco.MjModel.from_xml_string(MODEL.read_text(encoding='utf-8'))
        self.d=mujoco.MjData(self.m)
        self.active=np.array([self.m.joint(f'alpha{i}').qposadr[0] for i in (1,2)])
        self.output=np.array([self.m.joint(n).qposadr[0] for n in ('psi','beta')])
        self.passive=np.array([i for i in range(self.m.nv) if i not in self.active])
        self.sites={n:self.m.site(n).id for n in ('P1','P2','Q1','Q2','H','C','rod_end1','rod_end2')}
        self.reset()

    def reset(self):
        mujoco.mj_resetDataKeyframe(self.m,self.d,0)
        mujoco.mj_forward(self.m,self.d)

    def update(self):
        mujoco.mj_kinematics(self.m,self.d); mujoco.mj_comPos(self.m,self.d)

    def site_jac(self,name):
        jp=np.zeros((3,self.m.nv)); jr=jp.copy()
        mujoco.mj_jacSite(self.m,self.d,jp,jr,self.sites[name])
        return jp,jr

    def constraints(self):
        self.update()
        residual=np.concatenate([self.d.site_xpos[self.sites[f'rod_end{i}']]-self.d.site_xpos[self.sites[f'Q{i}']] for i in (1,2)])
        jac=np.vstack([self.site_jac(f'rod_end{i}')[0]-self.site_jac(f'Q{i}')[0] for i in (1,2)])
        return residual,jac

    def solve(self,alpha):
        """Fix only two input angles; solve six closure coordinates with MuJoCo Jacobians.
        No FK/IK or desired output q is used here. Continuation retains the home assembly branch.
        """
        self.d.qpos[self.active]=alpha
        for iteration in range(40):
            residual,jac=self.constraints()
            if np.max(np.abs(residual))<1e-12:
                return iteration
            step=np.linalg.solve(jac[:,self.passive],-residual)
            scale=min(1., .3/max(np.max(np.abs(step)),1e-15))
            old=self.d.qpos.copy(); norm=np.linalg.norm(residual)
            for _ in range(14):
                self.d.qpos[:]=old
                self.d.qpos[self.passive]+=scale*step
                if np.linalg.norm(self.constraints()[0])<norm:
                    break
                scale*=.5
            else:
                raise RuntimeError('Closure line search failed')
        raise RuntimeError('Closure solve did not converge')

    def reduced_jacobians(self):
        residual,jac=self.constraints()
        T=np.zeros((self.m.nv,2)); T[self.active]=np.eye(2)
        T[self.passive]=-np.linalg.solve(jac[:,self.passive],jac[:,self.active])
        jp,jr=self.site_jac('C')
        return T[self.output],jp@T,jr@T,np.linalg.matrix_rank(jac),residual


if __name__=='__main__':
    print(build())
    robot=Mechanism()
    print('nv=',robot.m.nv,'neq=',robot.m.neq,'home closure=',robot.constraints()[0])
