"""Remote adapter for the local panel. No device access unless --execute-panel."""
from bench_archive import send_archive, cleanup_acknowledged
import argparse
import contextlib
from collections import deque
import io
import hashlib
import json
import math
import queue
import signal
import sys
import threading
import time
from pathlib import Path

PREFIX="PH_EXO_PANEL "
DEFAULT={"k_psi":20.0,"k_beta":20.0,"b_psi":0.3,"b_beta":0.3,"torque_nm":10.0,"current_a":6.0,"travel_deg":90.0}
def validate_config(value):
    if isinstance(value, dict) and "wave" in value:
        remaining = dict(value)
        wave = remaining.pop("wave")
        checked = validate_config(remaining)
        if wave != "sine" or "amp_deg" not in checked or checked["amp_deg"][1] != 0:
            raise ValueError("正弦往返需要 ψ 幅度，β 幅度须为 0")
        return dict(checked, wave="sine")
    if not isinstance(value,dict) or set(value) not in tuple(set(DEFAULT)|p|r for p in (set(),{"pose_deg","move_s"}) for r in (set(),{"amp_deg","period_s"})):
        raise ValueError("参数必须包含两轴K、B和三个限幅；本模式不设置时长")
    pose=value.get("pose_deg"); move=value.get("move_s"); has_pose="pose_deg" in value
    amp=value.get("amp_deg"); period=value.get("period_s"); has_cycle="amp_deg" in value
    out={}
    for key,value in value.items():
        if key in ("pose_deg","move_s","amp_deg","period_s"):continue
        if isinstance(value,bool) or not isinstance(value,(int,float)) or not math.isfinite(value):
            raise ValueError(key+"必须为有限数值")
        out[key]=float(value)
    if not all(0<out[k]<=30 for k in ("k_psi","k_beta")):raise ValueError("K范围为(0,30] N·m/rad")
    if not all(0<=out[k]<=1 for k in ("b_psi","b_beta")):raise ValueError("B范围为[0,1] N·m·s/rad")
    for key,maximum in (("torque_nm",10.),("current_a",6.),("travel_deg",90.)):
        if not 0<out[key]<=maximum:raise ValueError(key+"必须大于0且不超过"+str(maximum))
    if pose is not None or move is not None:
        if not isinstance(pose,list) or len(pose)!=2 or any(type(x) not in (int,float) or not math.isfinite(x) for x in pose):
            raise ValueError("目标ψ/β必须同时填写有限角度")
        if not -45<=pose[0]<=45 or not -8<=pose[1]<=45:raise ValueError("当前模型目标范围：ψ -45～45°，β −8～45°")
        if type(move) not in (int,float) or not math.isfinite(move) or not 1<=move<=60:raise ValueError("过渡时间为1～60秒")
        out.update(pose_deg=[float(x) for x in pose],move_s=float(move))
    elif has_pose:raise ValueError("目标参数不能为null")
    if has_cycle:
        for name,pair in (("幅度",amp),("周期",period)):
            if not isinstance(pair,list) or len(pair)!=2 or any(type(x) not in (int,float) or not math.isfinite(x) for x in pair):
                raise ValueError(name+"必须为两轴有限数值")
        if (abs(amp[0])>45 or abs(amp[1])>53) or not any(amp):raise ValueError("往返幅度上限为ψ 45°、β 53°，至少一轴非零")
        if any(not 1<=x<=60 for x in period):raise ValueError("完整往返周期为1～60秒")
        if has_pose:
            end=[pose[i]+amp[i] for i in range(2)]
            if not -45<=end[0]<=45 or not -45<=pose[0]-amp[0]<=45 or not -8<=end[1]<=45:raise ValueError("往返端点超出当前模型范围")
        out.update(amp_deg=[float(x) for x in amp],period_s=[float(x) for x in period])
    return out

# Reuse the installed package's pure trajectory; bench-only encoder checks stay here.
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/"ros2_ws/src/mech_assist_control"))
from mech_assist_control.robot.ph_exo_reference import (
    TargetPose as _TargetPose, RoundTrip as _RoundTrip, TargetPoseRejected, blend)

class TargetPose(_TargetPose):
    def __init__(self,config):
        config=validate_config(config)
        super().__init__([math.radians(x) for x in config["pose_deg"]],config["move_s"])
        self.travel_deg=config["travel_deg"]
    def validate_encoders(self,ref):
        # Existing model IK is used only to preflight the final encoder envelope.
        sys.path.insert(0,str(Path(__file__).resolve().parents[2]/"mujoco/ref01"))
        from ref01_kinematics import inverse_kinematics
        a1,a2=inverse_kinematics(*self.goal,unit="rad")
        goal={105:math.degrees(a1)+45.,104:math.degrees(a2)+45.}
        delta={mid:goal[mid]-ref[mid] for mid in (104,105)}
        if any(abs(delta[mid])>self.travel_deg for mid in (104,105)):
            raise TargetPoseRejected(
                f"目标需104/105位移 {delta[104]:.2f}° / {delta[105]:.2f}°，"
                f"超出保持±{self.travel_deg:g}°；启动±90°不扩大最终保持范围")

class RoundTrip(_RoundTrip):
    def __init__(self,config):
        self.config=validate_config(config)
        super().__init__([math.radians(x) for x in config["amp_deg"]],config["period_s"],
            [math.radians(x) for x in config["pose_deg"]] if "pose_deg" in config else None,config.get("move_s",3.))
    def sample(self, t, q):
        pos, vel = super().sample(t, q)
        if self.config.get("wave") != "sine" or t <= self.startup_s:
            return pos, vel
        elapsed = t - self.startup_s
        period = self.period[0]
        u = min(elapsed / period, 1.)
        envelope, slope = blend(u)
        omega = 2. * math.pi / period
        phase = omega * elapsed
        base = self.approach.goal if self.approach else q
        pos[0] = base[0] + self.amp[0] * envelope * math.sin(phase)
        vel[0] = self.amp[0] * (slope / period * math.sin(phase) + envelope * omega * math.cos(phase))
        return pos, vel

    def validate_encoders(self,ref):
        if self.home is None:raise TargetPoseRejected("需先核对启动位姿")
        base=self.approach.goal if self.approach else self.home
        # Each encoder is monotonic in psi and beta on the accepted branch.
        # The rectangle covers different axis periods and the initial approach.
        import itertools
        bounds=[(min(self.home[i],base[i],base[i]+self.amp[i]),
                 max(self.home[i],base[i],base[i]+self.amp[i])) for i in range(2)]
        bounds[0] = (min(bounds[0][0], base[0]-self.amp[0]), max(bounds[0][1], base[0]-self.amp[0]))
        for point in itertools.product(*bounds):
            cfg={k:self.config[k] for k in DEFAULT}
            TargetPose(dict(cfg,pose_deg=[math.degrees(x) for x in point],move_s=1.)).validate_encoders(ref)

def make_trajectory(config):
    config=validate_config(config)
    return RoundTrip(config) if "amp_deg" in config else TargetPose(config) if "pose_deg" in config else None

def source_signature():
    base=Path(__file__).resolve().parent
    digest=hashlib.sha256()
    for name in ("ph_exo_bench_worker.py","ref01_bench_hold.py","bench_archive.py"):
        digest.update(name.encode());digest.update((base/name).read_bytes())
    config=base.parent.parent/"ros2_ws/src/mech_exo_state/config/ph_exo_mit_state.yaml"
    digest.update(b"ph_exo_mit_state.yaml");digest.update(config.read_bytes())
    digest.update((base.parent.parent/"ros2_ws/src/mech_assist_control/mech_assist_control/robot/ph_exo_reference.py").read_bytes())
    return digest.hexdigest()

class CancelledBeforeStart(KeyboardInterrupt):
    """Raised only by the pre-manager confirmation gate."""

def confirm_start(stopped):
    if stopped.is_set():
        raise CancelledBeforeStart("启动已取消，未发送控制命令")

def watch_stop(stream,stopped,archive_ack=None,expected=None,emergency=None):
    for line in stream:
        line=line.strip()
        if line in {"STOP","ESTOP"}:
            if line=="ESTOP" and emergency is not None:emergency.set()
            stopped.set()
            if archive_ack is None:return
        elif archive_ack is not None and expected and line=="ARCHIVE_OK "+expected[0]:
            archive_ack.set()
    stopped.set()


class TailConsole:
    """Bound console retention for a manually ended run."""
    def __init__(self):self.parts=deque(maxlen=128)
    def write(self,text):self.parts.append(text[-2048:]);return len(text)
    def flush(self):pass
    def getvalue(self):return "".join(self.parts)

def main(argv=None):
    parser=argparse.ArgumentParser(description="PH-EXO界面适配器；默认只检查，不打开设备")
    mode=parser.add_mutually_exclusive_group()
    mode.add_argument("--collect-run",help="Explicitly archive one pending run; no devices")
    mode.add_argument("--execute-panel",action="store_true")
    mode.add_argument("--software-test",action="store_true",help="仅私有PTY软件验证，不打开实际设备")
    parser.add_argument("--check",action="store_true")
    a=parser.parse_args(argv)
    if a.collect_run:
        import re
        if not re.fullmatch(r"ref01_bench_[0-9_]+(?:[a-f0-9]{8})?",a.collect_run):
            raise ValueError("Invalid run name")
        base=Path(__file__).resolve().parents[2]/".local/validation"
        out=base/a.collect_run
        if not out.is_dir():raise ValueError("Run not found")
        # Refuse recovery while a control run or managed deployment holds this tree.
        import fcntl
        from contextlib import ExitStack
        with ExitStack() as stack:
            for name in ("mit_communication.lock","sync.lock"):
                handle=stack.enter_context((base.parent/name).open("a+"))
                fcntl.flock(handle,fcntl.LOCK_EX|fcntl.LOCK_NB)
            ack=threading.Event();expected=[""];stopped=threading.Event()
            threading.Thread(target=watch_stop,args=(sys.stdin,stopped,ack,expected),daemon=True).start()
            def emit_archive(event):
                if event["type"]=="archive_begin":expected[0]=event["sha256"]
                print(PREFIX+json.dumps(event),flush=True)
            sha,files=send_archive(out,emit_archive)
            removed=cleanup_acknowledged(out,base,files,ack.wait(15))
            print(PREFIX+json.dumps({"type":"collected","removed":removed,"sha256":sha}),flush=True)
            return 0 if removed else 1
    if a.check or not (a.execute_panel or a.software_test):
        print(json.dumps({"mode":"check","source_signature":source_signature(),"defaults":DEFAULT}))
        return 0
    if sys.platform!="linux":raise RuntimeError("实机适配器仅在Orin运行")
    request=json.loads(sys.stdin.readline())
    if request.get("source_signature")!=source_signature():raise RuntimeError("两端入口版本不一致，请先同步")
    cfg=validate_config(request["config"])
    import ref01_bench_hold as bench
    stopped=threading.Event();emergency=threading.Event()
    archive_ack=threading.Event();expected=[""]
    for sig in (signal.SIGTERM,signal.SIGHUP,signal.SIGINT):
        signal.signal(sig,lambda *_:stopped.set())
    threading.Thread(target=watch_stop,args=(sys.stdin,stopped,archive_ack,expected,emergency),daemon=True).start()
    events=queue.Queue(maxsize=16)
    write_lock=threading.Lock()
    dropped=[0]
    closing=threading.Event()
    output_stream=sys.stdout
    console=TailConsole()
    def emit(value):
        if value["type"]=="archive_begin":expected[0]=value["sha256"]
        with write_lock:
            if closing.is_set() and value["type"]=="sample":return
            output_stream.write(PREFIX+json.dumps(value,ensure_ascii=True,allow_nan=False)+"\n")
            output_stream.flush()
    def pump():
        while True:
            row=events.get()
            try:emit({"type":"sample","sample":row})
            except (OSError,ValueError):stopped.set();return
            finally:events.task_done()
    threading.Thread(target=pump,daemon=True).start()
    def observe(row):
        try:events.put_nowait(row)
        except queue.Full:dropped[0]+=1  # Plot data can drop; control never waits for the browser.
    def confirm(duration,ref,qref,predictive,gains):
        confirm_start(stopped)
        emit({"type":"reference","reference_deg":ref,"reference_q_rad":qref})
        return time.monotonic()  # Explicit panel Start already authorized this run.
    out=bench.new_output_path()
    emit({"type":"preparing","output":str(out),"config":cfg,"software_test":a.software_test})
    error=None;cancelled_before_start=False
    try:
        with contextlib.redirect_stdout(console):
            bench.run(out,a.software_test,None,"normal",False,
                (cfg["k_psi"],cfg["k_beta"],cfg["b_psi"],cfg["b_beta"]),
                confirm=confirm,observer=observe,stop_requested=stopped.is_set,manual_stop=True,emergency=emergency,
                limits={k:cfg[k] for k in ("torque_nm","current_a","travel_deg")},
                trajectory=make_trajectory(cfg))
    except BaseException as exc:
        error=repr(exc)
        cancelled_before_start=isinstance(exc, (CancelledBeforeStart, TargetPoseRejected, bench.PreflightRejected))
    # Existing bench writes result only after its stop + independent disable path.
    result_path=out/"result.json"
    result=json.loads(result_path.read_text()) if result_path.exists() else None
    closing.set()
    archive={"status":"pending","remote_path":str(out)}
    try:
        sha,files=send_archive(out,emit)
        acknowledged=archive_ack.wait(15)
        cleared=cleanup_acknowledged(out,bench.ROOT/".local/validation",files,acknowledged)
        archive.update(status="local_verified_remote_removed" if cleared else "pending",sha256=sha)
    except Exception as exc:
        archive["error"]=str(exc)  # Preserve evidence; never delete on transfer failure.
    emit({"type":"finished","output":str(out),"result":result,"error":error,
          "emergency_requested":emergency.is_set(),"cancelled_before_start":cancelled_before_start,"archive":archive,"console":console.getvalue(),"display_dropped":dropped[0],"stop_requested":stopped.is_set()})
    return 0 if error is None else 1

if __name__=="__main__":sys.exit(main())
