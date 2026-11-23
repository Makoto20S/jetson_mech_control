"""Explicit terminal-gated dynamic bench; retains original stop/disable implementation."""
import argparse,math,signal,sys,time
from ref01_bench_hold import run,new_output_path,require

GAINS=(10.,10.,.18,.15)
LIMITS=dict(torque_nm=10.,current_a=6.,travel_deg=90.)
DURATION=26.

class Trajectory:
    """Same 3 degree / 6 second round trip, each axis, then reference loss."""
    def validate(self,q):
        require(len(q)==2 and all(math.isfinite(x) for x in q),"Invalid starting pose")
        require(-math.pi/4<=q[0]<=math.pi/4-math.radians(3.) and
                0<=q[1]<=math.pi/4-math.radians(3.),"Startup pose lacks room for +3 degree trajectories")
    def sample(self,t,q):
        self.validate(q)
        if t>=22.:return None
        target=list(q);velocity=[0.,0.]
        for axis,offset in ((0,0.),(1,12.)):
            elapsed=t-offset
            if 1.<elapsed<7.:
                w=2*math.pi/6.;phase=w*(elapsed-1.)
                target[axis]+=math.radians(3.)*.5*(1.-math.cos(phase))
                velocity[axis]=math.radians(3.)*.5*w*math.sin(phase)
        return target,velocity

def confirm(duration,ref,q,predictive,gains):
    require(sys.stdin.isatty(),"必须在可见终端输入START")
    Trajectory().validate(q)
    print("\n准备实机动态参考测试，尚未发送运动目标。",flush=True)
    print("K=[10,10]；B=[0.18,0.15]；每台力矩上限10 N·m；电流6 A；电机位移±90°。",flush=True)
    print("以当前姿态为中心：ψ先+3°往返，β再+3°往返；第22秒中断参考，观察停发并失能。",flush=True)
    print("每轴运动段6秒；总时长不超过26秒；驱动Kp=Kd=0。",flush=True)
    print("启动编码器角度：",ref,"；启动ψ/β(°)：",[math.degrees(x) for x in q],flush=True)
    print("本次沿用既定台架限幅10 N·m / 6 A / ±90°；参数效果由实机记录评估。",flush=True)
    print("本轮是自动小轨迹，请不要手推；Ctrl+C停止；需要时使用现场急停。",flush=True)
    require(input("输入 START 并回车才启动；其他输入取消：").strip()=="START","已取消，未启动")
    return time.monotonic()

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--execute',action='store_true',required=True)
    a=p.parse_args()
    require(sys.platform=='linux',"请在Orin ROS环境运行")
    def interrupted(*args):raise KeyboardInterrupt("操作者或连接中断")
    for sig in (signal.SIGHUP,signal.SIGTERM):signal.signal(sig,interrupted)
    out=new_output_path()
    print("结果目录：",out,flush=True)
    run(out,False,DURATION,'normal',False,GAINS,confirm=confirm,limits=LIMITS,trajectory=Trajectory())
if __name__=='__main__':main()
