"""Small fixed-pose session contract. No ROS, devices or control-law duplication."""
import math
import time

class FixedPoseSession:
    def __init__(self):
        self.phase="debug";self.reason="";self.session_id=0;self.sequence=0
        self.reference=None;self.gains=None;self.caps=None
        self.stop_confirmed=False;self.disable_confirmed=[False,False]
    def start(self, reference, gains, caps, now):
        if self.phase!="debug":raise ValueError("先停止并返回调试")
        if len(reference)!=2 or len(gains)!=4 or len(caps)!=2:raise ValueError("参数维数错误")
        if not all(math.isfinite(x) for x in (*reference,*gains,*caps,now)):raise ValueError("非有限参数")
        if not (-math.pi/4<=reference[0]<=math.pi/4 and math.radians(-8)<=reference[1]<=math.pi/4):
            raise ValueError("参考姿态超出当前模型合同")
        if not all(0<x<=30 for x in gains[:2]) or not all(0<=x<=1 for x in gains[2:]):
            raise ValueError("K/B超出当前合同")
        if not all(0<x<=10 for x in caps):raise ValueError("力矩上限超出合同")
        self.session_id=max(self.session_id+1,time.monotonic_ns())
        self.sequence=0;self.reference=tuple(reference);self.gains=tuple(gains);self.caps=tuple(caps)
        self.phase="starting";self.reason="";self.stop_confirmed=False;self.disable_confirmed=[False,False]
    def plan(self, now):
        if self.phase not in ("starting","running"):return None
        self.sequence+=1
        return dict(schema_version=1,session_id=self.session_id,sequence=self.sequence,
                    issued_at_monotonic_s=now,enabled=True,reference_rad=list(self.reference),
                    stiffness_nm_per_rad=list(self.gains[:2]),damping_nm_s_per_rad=list(self.gains[2:]),
                    motor_torque_cap_nm=list(self.caps))
    def stop(self,reason):
        self.phase="stopping";self.reason=reason
    def completed(self, stopped, ack, simulation=False):
        self.disable_confirmed=list(ack)
        self.stop_confirmed=bool(stopped and (simulation or ack==[True,True]))
        self.phase="stopped" if self.stop_confirmed else "unconfirmed"
    def reset(self):
        if self.phase not in ("debug","stopped") or (self.phase=="stopped" and not self.stop_confirmed):
            raise ValueError("停止未确认，不能返回调试")
        self.phase="debug";self.reason="";self.reference=None
