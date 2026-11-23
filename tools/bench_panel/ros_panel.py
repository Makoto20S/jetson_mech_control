"""Desktop ROS transport for the existing panel; originals remain in Panel."""
from collections import deque
import json
from pathlib import Path
import shlex
import subprocess
import threading
import time
from uuid import uuid4
from ph_exo_ros_client import DEFAULT, PREFIX, API, NODE, signature, validate_config, route
from ph_exo_bench_panel import Panel, ROOT, ssh_command

BUSY={"connecting","preparing","starting","running","stopping","resetting"}

def ros_command(config,software=False):
    command=ssh_command(config)
    # Build a new shell command with the same checked host/root and environment.
    shell=("set -e; cd "+shlex.quote(config["remote_root"])+"; "
        "unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH PYTHONPATH LD_LIBRARY_PATH; "
        "source /opt/ros/humble/setup.bash; source tools/project/mujoco_ros_env.sh; "
        "source .local/ros/ref01_native/install/local_setup.bash; "
        "export PYTHONDONTWRITEBYTECODE=1 PYTHONUNBUFFERED=1; "
        "export ROS_LOG_DIR=\"$PWD/.local/ros/ref01_native/ros_logs\"; "
        "exec python3 -B tools/project/ph_exo_ros_client.py"+(" --software" if software else ""))
    command[-1]="bash -lc "+shlex.quote(shell)
    return command

class RosPanel(Panel):
    def __init__(self,preview=False,software=False,popen=subprocess.Popen,api=API,node=NODE):
        super().__init__(preview=preview,popen=popen)
        self.config={k:list(v) for k,v in DEFAULT.items()}
        self.api=route(api);self.node=route(node);self.software=software
        self.ros={};self.ready=False;self.pending=None;self.seq=0;self.sent={}
        self.last_rx=0.;self.last_ping=0.;self.samples=deque(maxlen=2400)
        self.states=deque(maxlen=1200);self.saved=False;self.closed=False;self.owns=False;self.uncertain=False;self.await_start=False;self.control=False
        self.refresh=False;self.recover="";self.recover_at=0.;self.reset_after=0;self.state_seq=0;self.notice=""
    def snapshot(self,after=0,heartbeat=True):
        with self.lock:
            data=super().snapshot(after,heartbeat)
            data.update(mode="ros",ros=self.ros,connected=self.ready,software=self.software,
                        link_reset=self.can_reset_link(),refresh=self.refresh,recover=self.recover,
                        notice=self.notice,can_start=(not self.refresh or self.recover=="ready"),
                        need_stop=self.control and self.recover!="ready")
            if self.refresh:data.update(phase="idle",run_id=None)
            return data
    def can_reset_link(self):
        # A local disconnect cannot certify a motor stop or clear a control request.
        return (not self.control and
                self.ros.get("state") in (None,"debug"))
    def link_failed(self,message):
        self.ready=False;self.pending=None
        self.phase="connect_error" if self.can_reset_link() else "unconfirmed"
        if self.refresh:
            self.recover="failed";self.notice="已返回初始界面；"+message+"。可再次返回调试以刷新连接。"
        self.logs.append(message)
        self.add(dict(type="connection_lost",message=message))
    def send(self,cmd,**data):
        if self.proc is None or self.proc.poll() is not None:raise ValueError("ROS连接未建立")
        if cmd=="start":self.control=True
        if cmd!="ping":
            self.seq+=1;data["seq"]=self.seq;self.sent[self.seq]=cmd
        packet=dict(cmd=cmd,**data)
        self.proc.stdin.write((json.dumps(packet)+"\n").encode());self.proc.stdin.flush()
        return self.seq
    def connect(self):
        with self.lock:
            if self.preview:raise ValueError("预览模式不连接ROS")
            if self.refresh and self.recover=="ready":
                self.refresh=False;self.recover="";self.notice=""
            if self.proc is not None and self.proc.poll() is None:return
            self.ready=False;self.ros={};self.closed=False;self.sent.clear();self.phase="connecting"
            self.proc=None
            try:
                sync=json.loads((ROOT/"config/parallel_exo/orin_sync.json").read_text(encoding="utf-8"))
                self.proc=self.popen(ros_command(sync,self.software),stdin=subprocess.PIPE,stdout=subprocess.PIPE,
                    stderr=subprocess.STDOUT,bufsize=0,creationflags=getattr(subprocess,"CREATE_NO_WINDOW",0))
                hello=dict(sig=signature(),api=self.api,node=self.node)
                self.proc.stdin.write((json.dumps(hello)+"\n").encode());self.proc.stdin.flush()
                self.last_rx=time.monotonic()
                threading.Thread(target=self.read_worker,args=(self.proc,),daemon=True).start()
            except Exception:
                self.link_failed("ROS连接失败，可返回调试后重试")
                if self.proc and self.proc.stdin:
                    try:self.proc.stdin.close()
                    except (OSError,ValueError):pass
                raise
    def start(self,config):
        config=validate_config(config)
        with self.lock:
            if self.preview:raise ValueError("预览模式禁止启动")
            if self.refresh and self.recover!="ready":
                raise ValueError("界面已复位；ROS停止/复位尚未确认，请先连接并返回调试")
            if self.phase not in {"idle","debug"} or self.pending is not None:
                raise ValueError("先停止并通过ROS返回调试")
            from sync_orin import safe_path
            self.refresh=False;self.recover="";self.notice=""
            self.run_id=time.strftime("%Y%m%d_%H%M%S")+"_ros_"+uuid4().hex[:8]
            self.directory=safe_path(ROOT,"tools/bench_panel/data/runs/"+self.run_id)
            self.directory.mkdir(parents=True,exist_ok=False)
            self.config=config;self.result=None;self.events.clear();self.logs.clear()
            self.samples.clear();self.states.clear();self.saved=False
            self.owns=True;self.phase="connecting";self.pending=config;self.last_poll=time.monotonic()
            (self.directory/"parameters.json").write_text(json.dumps(config,indent=2),encoding="utf-8")
            try:self.connect();self.maybe_start()
            except Exception:
                self.link_failed("启动前连接失败；未确认的控制请求仍保留")
                if self.proc and self.proc.stdin:self.proc.stdin.close()
                raise
    def maybe_start(self):
        if self.refresh or self.pending is None or not self.ready or not self.ros:return
        config=self.pending;self.pending=None
        if self.ros.get("state")!="debug":
            self.phase=self.ros.get("state","unconfirmed")
            self.logs.append("ROS未处于debug，未发送启动请求");return
        self.await_start=True;self.phase="preparing";self.send("start",config=config)
    def detach(self):
        old=self.proc
        if old is not None and old.poll() is None:
            try:self.send("close")
            except (OSError,ValueError):pass
            try:old.stdin.close()
            except (OSError,ValueError):pass
        self.proc=None;self.ready=False;self.closed=False;self.sent.clear()
    def local_reset(self):
        self.detach();self.ros={};self.control=False;self.uncertain=False
        self.owns=False;self.phase="idle";self.recover="ready"
        self.run_id=None;self.directory=None;self.result=None
        self.samples.clear();self.states.clear()
        self.notice="已返回初始状态，可修改参数或重新连接ROS。"
    def reset(self):
        with self.lock:
            # UI refresh is unconditional. Motor stop/reset remains an observed ROS operation.
            self.pending=None;self.await_start=False;self.refresh=True
            self.add(dict(type="ros_reset",local=True))
            if self.can_reset_link():
                self.local_reset();return
            if self.recover in {"connect","stop","wait","reset"}:
                return  # Repeat clicks refresh the page; do not duplicate a pending operation.
            self.recover_at=time.monotonic();self.notice="已返回初始界面；正在停止并复位ROS。"
            try:
                if self.proc is None or self.proc.poll() is not None or self.closed or not self.ready:
                    self.detach();self.recover="connect";self.connect()
                else:
                    self.recover="connect"
                self.advance_reset()
            except (OSError,ValueError):
                self.link_failed("ROS连接不可用，实际停止状态未确认")
    def advance_reset(self):
        if not self.refresh:return
        fresh=self.ready and time.monotonic()-self.last_rx<1 and self.ros
        if self.recover=="connect" and fresh:
            # Cancel the remote client's pending start before waiting on stop evidence.
            self.recover="stop";self.phase="stopping";self.send("stop")
        elif self.recover=="wait" and fresh and self.state_seq>self.reset_after:
            actual=self.ros.get("state")
            if actual=="stopped" and self.ros.get("stop_ok"):
                self.recover="reset";self.phase="resetting";self.send("reset")
            elif actual=="unconfirmed":
                self.recover="failed";self.notice="已返回初始界面；ROS停止未确认，可再次返回调试重试。"
    def stop(self,reason="界面停止"):
        with self.lock:
            self.pending=None;self.await_start=False
            if self.proc is None or self.proc.poll() is not None:return
            self.logs.append(reason);self.phase="stopping"
            try:self.send("stop")
            except (OSError,ValueError):self.phase="unconfirmed"
    def emergency_stop(self):
        with self.lock:
            self.pending=None;self.await_start=False
            try:self.phase="stopping";self.send("estop")
            except (OSError,ValueError):
                self.phase="unconfirmed";self.logs.append("ROS急停连接不可用；未确认停止")
    def watchdog(self):
        with self.lock:
            now=time.monotonic()
            if self.refresh and self.recover in {"connect","stop","wait","reset"} and now-self.recover_at>16:
                self.recover="failed";self.notice="已返回初始界面；ROS恢复超时，可再次返回调试重试。"
            if self.proc is None or self.proc.poll() is not None:return
            if now-self.last_poll<4 and now-self.last_ping>.5:
                try:self.send("ping");self.last_ping=now
                except (OSError,ValueError):self.phase="unconfirmed"
            if self.owns and self.phase in BUSY and now-self.last_poll>4:
                self.stop("页面超过4秒无响应，已请求ROS停止")
                self.shutdown()
            if self.ready and now-self.last_rx>2:
                self.link_failed("ROS状态已过期，不把旧状态作为停止确认")
    def finish(self):
        key=(self.ros.get("id"),self.ros.get("state"),self.ros.get("stop_ok"),
             tuple(self.ros.get("disable_ok",[])),self.ros.get("reason"))
        if self.directory is None or not self.owns or self.saved==key:return
        self.saved=key
        r=dict(backend="ros",rehearsal=bool(self.ros.get("sim",True)),
            gains=self.config["k"]+self.config["b"],caps=self.config["tau"],
            reference_q_rad=self.ros.get("q_ref"),ros=self.ros,
            disable_ack=[mid for mid,ok in zip((104,105),self.ros.get("disable_ok",[])) if ok],
            stop_ok=bool(self.ros.get("stop_ok")),statuses=list(self.states),samples=[],
            panel_samples=list(self.samples),error=self.ros.get("reason",""))
        self.result={"result":r}
        (self.directory/"result.json").write_text(json.dumps(r,indent=2,allow_nan=False),encoding="utf-8")
        (self.directory/"completion.json").write_text(json.dumps(self.ros,indent=2),encoding="utf-8")
        self.add(dict(type="ros_done",status=self.ros,local_directory=str(self.directory)))
    def handle(self,e):
        kind=e.get("type")
        if kind=="ready":
            self.ready=True;self.maybe_start();self.advance_reset()
        elif kind=="state":
            self.last_rx=time.monotonic();self.state_seq+=1;self.ros=e["status"]
            actual=self.ros["state"]
            if actual!="debug":
                self.await_start=False;self.control=True
            waiting=set(self.sent.values())
            if self.uncertain and actual not in {"stopped","unconfirmed"}:
                self.phase="unconfirmed"
            elif actual=="debug" and (self.await_start or "start" in waiting or self.pending is not None):
                pass
            elif actual in {"debug","stopped"} and "reset" in waiting:
                pass
            elif actual in {"preparing","starting","running","debug"} and waiting & {"stop","estop"}:
                pass
            else:self.phase=actual
            self.states.append(self.ros)
            self.add(dict(type="ros_state",status=self.ros))
            if actual in {"stopped","unconfirmed"} and self.pending is None:self.finish()
            self.maybe_start();self.advance_reset()
        elif kind=="reply":
            self.sent.pop(e["seq"],None)
            self.logs.append(e.get("cmd","")+"："+e.get("message",""))
            if not e.get("ok"):
                self.await_start=False;self.pending=None
                if self.refresh and e["cmd"] in {"stop","reset"}:
                    self.recover="failed";self.notice="已返回初始界面；"+e.get("message","ROS恢复未确认")
                # An unknown service result cannot be treated as a safe retry.
                if "未确认" in e.get("message",""):
                    self.phase="unconfirmed";self.uncertain=True
                    if e["cmd"]=="start":self.control=True
                elif self.ros:self.phase=self.ros["state"]
                else:self.phase="error"
                self.add(dict(type="error",message=e.get("message","ROS请求失败")))
            elif e["cmd"]=="stop" and self.refresh and self.recover=="stop":
                self.recover="wait"
                # Require a subsequent status, not a cached stopped flag.
                self.reset_after=self.state_seq
            elif e["cmd"]=="reset":
                self.recover="ready";self.notice="已返回初始状态；ROS已恢复调试。"
                self.phase="debug";self.owns=False;self.control=False;self.uncertain=False;self.await_start=False;self.ros=dict(self.ros,state="debug",q_ref=None)
                self.add(dict(type="ros_reset"))
        elif kind=="sample" and self.ros and self.ready:
            self.samples.append(e["sample"]);self.add(e)
        elif kind=="closed":
            self.closed=True;self.ready=False
            if self.refresh and self.recover in {"connect","stop","wait","reset"}:
                self.recover="failed";self.notice="已返回初始界面；ROS连接退出，恢复尚未确认。"
            state=e.get("status") or {}
            if state.get("state"):
                self.ros=state
                if state["state"]!="debug":self.control=True
            if state.get("state")=="stopped" and state.get("stop_ok"):
                self.ros=state;self.phase="stopped";self.finish()
            elif state.get("state")=="debug" and self.can_reset_link():
                self.phase="idle"
            else:self.link_failed("ROS连接未建立，可返回调试" if self.can_reset_link() else "连接退出，停止状态未确认")
        elif kind=="error":
            self.logs.append(e["message"]);self.add(e)
    def read_worker(self,proc):
        try:
            for raw in iter(proc.stdout.readline,b""):
                with self.lock:
                    if proc is not self.proc:continue
                    if self.directory:
                        with (self.directory/"worker.log").open("ab") as f:f.write(raw)
                    line=raw.decode("utf-8",errors="replace").rstrip()
                    if line.startswith(PREFIX):self.handle(json.loads(line[len(PREFIX):]))
                    else:self.logs.append(line[:2000])
            code=proc.wait()
            with self.lock:
                if proc is not self.proc:return
                self.ready=False;self.logs.append("ROS连接退出："+str(code))
                if not self.closed:
                    self.link_failed("连接中断，可返回调试" if self.can_reset_link() else "连接中断，停止状态未确认；不自动重连或启动")
        except Exception as exc:
            with self.lock:
                if proc is self.proc:self.link_failed(str(exc))
            try:proc.stdin.close()
            except (OSError,ValueError):pass
    def shutdown(self):
        with self.lock:
            self.pending=None
            try:
                if self.proc and self.proc.poll() is None:
                    self.send("close");self.proc.stdin.close()
            except (OSError,ValueError):pass
