"""Shared SI reference generation. No ROS, files, devices or control law.

q/amp are [psi,beta] radians, period/move seconds. A cycle is symmetric about base for psi and base -> base+amp -> base for beta. Start and turns have continuous position/velocity/acceleration.
"""
import math
from numbers import Real

DEFAULT = dict(ref="fixed", target=False, q=[0., math.pi/9], amp=[0.,0.],
               period=[6.,6.], move=3.)

class TargetPoseRejected(ValueError):
    """Reference outside the current model contract."""

def pose(q):
    try:q=list(q)
    except TypeError:raise TargetPoseRejected("位姿需要两轴数值") from None
    if (len(q)!=2 or
        any(isinstance(x,bool) or not isinstance(x,Real) or not math.isfinite(x) for x in q) or
        not -math.pi/4<=q[0]<=math.pi/4 or not math.radians(-8)-1e-14<=q[1]<=math.pi/4+1e-14):
        raise TargetPoseRejected("位姿超出当前模型范围：ψ −45～45°，β −8～45°")
    return [float(q[0]), min(math.pi/4, max(math.radians(-8), float(q[1])))]

def validate_extrema(base, amp):
    pose([base[0] - amp[0], base[1]])
    pose([base[0] + amp[0], base[1] + amp[1]])

def validate(c):
    if not isinstance(c,dict) or set(c)!=set(DEFAULT):raise ValueError("参考参数字段不完整")
    if c['ref'] not in ('fixed','target','cycle') or type(c['target']) is not bool:raise ValueError("参考模式无效")
    c=dict(c)
    for key in ('q','amp','period'):
        try:pair=list(c[key])
        except TypeError:raise ValueError(key+"需要两轴有限数值") from None
        if len(pair)!=2 or any(isinstance(x,bool) or not isinstance(x,Real) or not math.isfinite(x) for x in pair):raise ValueError(key+"需要两轴有限数值")
        c[key]=list(map(float,pair))
    pose(c['q'])
    if any(abs(x)>math.radians(limit) for x,limit in zip(c['amp'],(45,53))):raise ValueError("幅度上限：ψ 45°、β 53°")
    if c['ref']=='cycle' and not any(c['amp']):raise ValueError("至少一轴幅度非零")
    if any(not 1<=x<=60 for x in c['period']):raise ValueError("完整周期1～60秒")
    if type(c['move']) not in (int,float) or not math.isfinite(c['move']) or not 1<=c['move']<=60:raise ValueError("过渡1～60秒")
    if c['ref']=='cycle' and c['target']:validate_extrema(c['q'], c['amp'])
    return {k:list(map(float,v)) if isinstance(v,(list,tuple)) else float(v) if k=='move' else v for k,v in c.items()}

def from_degrees(c):
    """Desktop/legacy optional fields; all installed ROS parameters use SI."""
    has_pose='pose_deg' in c;has_cycle='amp_deg' in c
    return validate(dict(DEFAULT,ref='cycle' if has_cycle else 'target' if has_pose else 'fixed',
        target=has_pose,q=[math.radians(x) for x in c['pose_deg']] if has_pose else DEFAULT['q'],
        move=c.get('move_s',3.),amp=[math.radians(x) for x in c['amp_deg']] if has_cycle else [0.,0.],
        period=c.get('period_s',[6.,6.])))

def blend(u):return min(1., max(0., u*u*u*(10.+u*(-15.+6.*u)))),30.*u*u*(1.-u)*(1.-u)

class TargetPose:
    expects_reference_loss=False
    def __init__(self,goal,duration):
        self.goal=pose(goal);self.duration=duration;self.startup_s=1.+duration
        validate(dict(DEFAULT,ref='target',q=self.goal,move=duration))
    def validate(self,q):pose(q)
    def sample(self,t,q):
        q=pose(q);self.validate(q)
        if not math.isfinite(t) or t<0:raise ValueError("目标时间无效")
        u=min(1.,max(0.,(t-1.)/self.duration));h,dh=blend(u)
        if u==0.:return list(q),[0.,0.]
        if u==1.:return list(self.goal),[0.,0.]
        return ([q[i]+(self.goal[i]-q[i])*h for i in range(2)],
                [(self.goal[i]-q[i])*dh/self.duration for i in range(2)])

class RoundTrip:
    expects_reference_loss=False
    def __init__(self,amp,period,goal=None,move=3.):
        c=validate(dict(DEFAULT,ref='cycle',amp=amp,period=period,
                        target=goal is not None,q=goal if goal is not None else DEFAULT['q'],move=move))
        self.amp=c['amp'];self.period=c['period'];self.home=None
        self.approach=TargetPose(c['q'],move) if goal is not None else None
        self.startup_s=self.approach.startup_s if self.approach else 1.
    def validate(self,q):
        pose(q);base=self.approach.goal if self.approach else q
        validate_extrema(base, self.amp);self.home=list(q)
    def sample(self,t,q):
        q=pose(q);self.validate(q)
        if not math.isfinite(t) or t<0:raise ValueError("目标时间无效")
        if t<=self.startup_s:return self.approach.sample(t,q) if self.approach else (list(q),[0.,0.])
        base=self.approach.goal if self.approach else q
        pos=list(base);vel=[0.,0.]
        for i in range(2):
            phase=((t-self.startup_s)%self.period[i])/self.period[i]
            if i == 0:
                # Each half-cycle visits one side of the base.
                direction = 1. if phase < .5 else -1.
                phase = (2. * phase) % 1.
                rate = 4. / self.period[i]
            else:
                direction = 1.
                rate = 2. / self.period[i]
            u=2.*phase if phase<=.5 else 2.*(1.-phase)
            h,dh=blend(u)
            pos[i]+=direction*self.amp[i]*h
            vel[i]=direction*self.amp[i]*dh*rate*(1. if phase<=.5 else -1.)
        return pose(pos),vel

def create(c):
    c=validate(c)
    if c['ref']=='fixed':return None
    if c['ref']=='target':return TargetPose(c['q'],c['move'])
    return RoundTrip(c['amp'],c['period'],c['q'] if c['target'] else None,c['move'])
