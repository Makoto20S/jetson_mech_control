"""Local-only visual bench adapter. Opening the page never starts SSH or motors."""
import argparse
import json
import os
from pathlib import Path
import secrets
import shlex
import subprocess
import threading
import time
import webbrowser
from collections import deque
from http.server import BaseHTTPRequestHandler,ThreadingHTTPServer
from urllib.parse import urlsplit
from uuid import uuid4
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/"project"))
from ph_exo_bench_worker import DEFAULT,PREFIX,source_signature,validate_config
from bench_archive import ArchiveReceiver
sys.path.insert(0,str(Path(__file__).resolve().parent))
from panel_kinematics import encoder_reference, preview_plan

ROOT=Path(__file__).resolve().parents[2]
ACTIVE={"preparing","running","stopping"}
def ssh_command(config,software=False):
    host=config["ssh_host"];root=config["remote_root"]
    if not isinstance(host,str) or host.startswith("-") or any(c.isspace() for c in host):
        raise ValueError("非法SSH主机别名")
    if not isinstance(root,str) or not root.startswith("/") or "\n" in root:
        raise ValueError("非法远端路径")
    command=( "cd "+shlex.quote(root)+"; "
        "unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH PYTHONPATH LD_LIBRARY_PATH; "
        )
    command="set -e; "+command+(
        "source /opt/ros/humble/setup.bash; source tools/project/mujoco_ros_env.sh; "
        "source .local/ros/ref01_native/install/local_setup.bash; "
        "export PYTHONDONTWRITEBYTECODE=1 PYTHONUNBUFFERED=1; "
        "exec python3 -B tools/project/ph_exo_bench_worker.py "+("--software-test" if software else "--execute-panel"))
    # No PTY: stdin is exclusively the bounded adapter protocol.
    return ["ssh","-T","-o","BatchMode=yes","-o","StrictHostKeyChecking=yes",
            "-o","ConnectTimeout=10","-o","ServerAliveInterval=2","-o","ServerAliveCountMax=2",
            host,"bash -lc "+shlex.quote(command)]

class Panel:
    def __init__(self,preview=False,output=None,popen=subprocess.Popen,software=False):
        self.software=software;self.preview=preview;self.output=output or ROOT/"tools/bench_panel/data/runs"
        self.popen=popen;self.lock=threading.RLock();self.proc=None;self.phase="idle"
        self.config=dict(DEFAULT);self.events=deque(maxlen=1200);self.logs=deque(maxlen=60)
        self.serial=0;self.last_poll=time.monotonic();self.run_id=None;self.result=None
        self.directory=None;self.finished_seen=False
    def add(self,event):
        self.serial+=1;self.events.append(dict(index=self.serial,**event))
    def snapshot(self,after=0,heartbeat=True):
        with self.lock:
            if heartbeat:self.last_poll=time.monotonic()
            return dict(phase=self.phase,preview=self.preview,config=self.config,run_id=self.run_id,
                        events=[e for e in self.events if e["index"]>after],logs=list(self.logs))
    def start(self,config):
        config=validate_config(config)
        with self.lock:
            if self.preview:raise ValueError("预览模式不连接Orin，不能启动实机")
            if self.phase in ACTIVE or self.phase=="unconfirmed" or (self.proc is not None and self.proc.poll() is None):
                raise ValueError("上一轮尚未结束，不能重复启动")
            # Local output must stay in the repository, without following a reparse point.
            from sync_orin import safe_path
            expected=safe_path(ROOT,"tools/bench_panel/data/runs")
            if self.output.resolve()!=expected.resolve():raise ValueError("不允许自定义实机输出根目录")
            self.run_id=time.strftime("%Y%m%d_%H%M%S")+"_"+uuid4().hex[:8]
            self.directory=safe_path(ROOT,"tools/bench_panel/data/runs/"+self.run_id)
            self.directory.mkdir(parents=True,exist_ok=False)
            self.config=config;self.events.clear();self.logs.clear();self.result=None
            self.finished_seen=False;self.last_poll=time.monotonic();self.phase="preparing"
            (self.directory/"parameters.json").write_text(json.dumps(config,indent=2),encoding="utf-8")
            try:
                sync=json.loads((ROOT/"config/parallel_exo/orin_sync.json").read_text(encoding="utf-8"))
                self.proc=self.popen(ssh_command(sync,self.software),stdin=subprocess.PIPE,stdout=subprocess.PIPE,
                    stderr=subprocess.STDOUT,bufsize=0,creationflags=getattr(subprocess,"CREATE_NO_WINDOW",0))
                payload={"config":config,"source_signature":source_signature()}
                self.proc.stdin.write((json.dumps(payload)+"\n").encode());self.proc.stdin.flush()
                threading.Thread(target=self.read_worker,args=(self.proc,),daemon=True).start()
            except Exception:
                # If anything failed after spawn, EOF requests stop; never retry.
                if self.proc is not None and self.proc.stdin:self.proc.stdin.close()
                self.phase="unconfirmed" if self.proc is not None else "error";raise
    def reset(self):
        with self.lock:
            if self.phase not in {"idle","cancelled","finished","error"}:
                raise ValueError("本轮仍在运行或失能未确认，不能返回调试")
            if self.proc is not None and self.proc.poll() is None:
                raise ValueError("连接进程尚未退出，请稍候")
            self.phase="idle";self.proc=None;self.run_id=None
            self.result=None;self.events.clear();self.logs.clear()
    def stop(self,reason="界面停止"):
        with self.lock:
            if self.phase not in ACTIVE:return
            self.phase="stopping";self.logs.append(reason)
            try:
                self.proc.stdin.write(b"STOP\n");self.proc.stdin.flush()
            except (OSError,ValueError,AttributeError):self.logs.append("停止通道不可用，等待远端退出记录")
    def emergency_stop(self):
        with self.lock:
            if self.phase not in ACTIVE and self.phase!="unconfirmed":return
            self.phase="stopping";self.logs.append("软件急停已请求：停止控制并独立失能")
            try:
                if self.proc is None or self.proc.poll() is not None:raise OSError("连接已退出")
                self.proc.stdin.write(b"ESTOP\n");self.proc.stdin.flush()
            except (OSError,ValueError,AttributeError):
                self.phase="unconfirmed";self.logs.append("急停通道不可用，请使用现场硬件急停；未确认失能")
    def watchdog(self):
        with self.lock:
            if self.phase in {"preparing","running"} and time.monotonic()-self.last_poll>4:
                self.stop("页面超过4秒无响应，已请求停止")
    def read_worker(self,proc):
        try:
            receiver=ArchiveReceiver(self.directory/"remote_raw")
            with (self.directory/"worker.log").open("wb") as logfile:
                for raw in iter(proc.stdout.readline,b""):
                    line=raw.decode("utf-8",errors="replace").rstrip()
                    if not (line.startswith(PREFIX) and '"type": "archive_' in line):
                        logfile.write(raw);logfile.flush()
                    with self.lock:
                        if line.startswith(PREFIX):
                            e=json.loads(line[len(PREFIX):])
                            if e["type"].startswith("archive_"):
                                ack=receiver.accept(e)
                                if ack:
                                    proc.stdin.write(("ARCHIVE_OK "+ack+"\n").encode());proc.stdin.flush()
                                    self.logs.append("本轮原始记录已保存到电脑并通过SHA256校验。")
                                continue
                            if e["type"]=="sample":
                                if e["sample"].get("mode")==2 and self.phase!="stopping":self.phase="running"
                            elif e["type"]=="finished":
                                self.finished_seen=True;self.result=e
                                archive=e.get("archive",{})
                                if archive:
                                    self.logs.append("远端临时记录已清理" if archive.get("status")=="local_verified_remote_removed"
                                                     else "远端记录待回收："+str(archive.get("remote_path","")))
                                # Never call a stop request proof of actual drive disable.
                                r=e.get("result") or {}
                                confirmed=r.get("disable_ack")==[104,105] and not r.get("shutdown_error")
                                cancelled=(e.get("cancelled_before_start") is True and e.get("result") is None
                                           and self.phase in {"preparing","stopping"})
                                self.phase="cancelled" if cancelled else ("finished" if confirmed else "unconfirmed")
                                (self.directory/"result.json").write_text(json.dumps(r,indent=2),encoding="utf-8")
                                (self.directory/"completion.json").write_text(json.dumps(e,indent=2),encoding="utf-8")
                                e=dict(e,result=None,disable_confirmed=confirmed,cancelled_confirmed=cancelled,
                                       local_directory=str(self.directory))
                            self.add(e)
                        else:self.logs.append(line[:2000])
            code=proc.wait()
            with self.lock:
                self.logs.append("连接进程退出："+str(code))
                if not self.finished_seen:
                    self.phase="unconfirmed";self.add({"type":"connection_lost",
                        "message":"未收到完整结束记录，不能确认电机失能；请检查现场与急停。"})
        except Exception as exc:
            with self.lock:
                self.logs.append(repr(exc));self.phase="unconfirmed"
            try:proc.stdin.close()
            except (OSError,ValueError):pass
    def result_download(self):
        with self.lock:
            if self.result is None:raise ValueError("尚无完整结果")
            return self.result["result"]


class BenchPanel(Panel):
    """Independent bench entry; same worker/control core, no upper-node client."""
    def __init__(self,**kwargs):
        super().__init__(**kwargs);self.refresh=False
    def can_start(self):
        return (self.phase in {"idle","cancelled","finished","error"} and
                (self.proc is None or self.proc.poll() is not None))
    def snapshot(self,after=0,heartbeat=True):
        with self.lock:
            ready=self.can_start()
            if self.refresh and ready:
                self.phase = "idle"
                self.run_id = None
                self.proc = None
                self.logs.clear()
                self.events.clear()
            data=super().snapshot(after,heartbeat)
            data.update(mode="bench",software=self.software,refresh=self.refresh,
                        can_start=ready,need_stop=self.phase in ACTIVE|{"unconfirmed"},
                        actual=self.phase)
            if self.refresh:
                recovery="ready" if ready else ("failed" if self.phase=="unconfirmed" else "wait")
                data.update(phase="idle",run_id=None,recover=recovery,
                    notice="已返回初始状态，可设置下一轮参数。" if ready else
                           "已返回初始界面；独立调参实际状态："+self.phase+"。")
            return data
    def reset(self):
        with self.lock:
            self.refresh=True
            self.add(dict(type="panel_reset"))
            if self.phase in {"preparing","running"}:self.stop("返回调试：停止本轮独立控制")
            elif self.phase=="unconfirmed" and self.proc is not None and self.proc.poll() is None:
                self.emergency_stop()
            # Keep records/unknown stop evidence; the page still resets immediately.
    def start(self,config):
        with self.lock:
            if not self.can_start():raise ValueError("独立调参尚未停止确认，不能开始下一轮")
            self.refresh=False
            super().start(config)

def resolve_mode(mode=None,legacy=False):
    if mode not in (None,"ros","bench"):raise ValueError("mode必须为ros或bench")
    if legacy and mode=="ros":raise ValueError("--legacy与--mode ros冲突")
    return "bench" if legacy else (mode or "ros")

def create_panel(mode,preview=False,software=False,api="/exoskeleton/impedance",node="/exoskeleton_control"):
    if mode=="bench":return BenchPanel(preview=preview,software=software)
    if mode!="ros":raise ValueError("未知调参入口")
    from ros_panel import RosPanel
    return RosPanel(preview=preview,software=software,api=api,node=node)

def make_server(panel,port=0):
    token=secrets.token_urlsafe(32)
    class Handler(BaseHTTPRequestHandler):
        def log_message(self,*_):pass
        def reply(self,data,status=200,kind="application/json; charset=utf-8"):
            content=data if isinstance(data,bytes) else json.dumps(data,ensure_ascii=False,allow_nan=False).encode()
            self.send_response(status);self.send_header("Content-Type",kind)
            self.send_header("Content-Length",str(len(content)));self.send_header("Cache-Control","no-store")
            self.send_header("X-Content-Type-Options","nosniff")
            self.send_header("Content-Security-Policy","default-src 'self'; script-src 'self' 'unsafe-inline'; style-src 'self' 'unsafe-inline'; connect-src 'self'; object-src 'none'; frame-ancestors 'none'")
            self.end_headers();self.wfile.write(content)
        def allowed(self,api=False):
            host="127.0.0.1:"+str(self.server.server_port)
            if self.headers.get("Host")!=host:return False
            origin=self.headers.get("Origin")
            if origin is not None and origin!="http://"+host:return False
            return not api or self.headers.get("X-Panel-Token")==token
        def do_GET(self):
            if not self.allowed():return self.reply({"error":"来源不匹配"},403)
            u=urlsplit(self.path)
            if u.path=="/":
                html=Path(__file__).with_name("ph_exo_bench_panel.html").read_text(encoding="utf-8")
                return self.reply(html.replace("__TOKEN__",token).replace("__ROS__","true" if hasattr(panel,"ros") else "false").encode(),kind="text/html; charset=utf-8")
            if not self.allowed(True):return self.reply({"error":"接口凭据不匹配"},403)
            try:
                if u.path=="/state":
                    from urllib.parse import parse_qs
                    after=int(parse_qs(u.query).get("after",["0"])[0])
                    state=panel.snapshot(after)
                    state["events"]=[dict(e,sample=dict(e["sample"],encoder_ref_deg=encoder_reference(e["sample"].get("reference"))))
                                     if e.get("type")=="sample" else e for e in state["events"]]
                    return self.reply(state)
                if u.path=="/result":return self.reply(panel.result_download())
                return self.reply({"error":"未找到"},404)
            except Exception as exc:self.reply({"error":str(exc)},400)
        def do_POST(self):
            if not self.allowed(True):return self.reply({"error":"来源或接口凭据不匹配"},403)
            try:
                size=int(self.headers.get("Content-Length","0"))
                limit=32768 if self.path=="/kinematics/ik" else 4096
                if not 0<size<=limit:raise ValueError("请求长度不合适")
                value=json.loads(self.rfile.read(size))
                if self.path=="/kinematics/preview":
                    return self.reply(preview_plan(value,ros=hasattr(panel,"ros")))
                if self.path=="/kinematics/ik":
                    if not isinstance(value,list) or len(value)>256:
                        raise ValueError("每次最多输入256个位姿")
                    return self.reply([encoder_reference(q) for q in value])
                if self.path=="/connect":panel.connect()
                elif self.path=="/start":panel.start(value)
                elif self.path=="/stop":panel.stop()
                elif self.path=="/reset":panel.reset()
                elif self.path=="/estop":panel.emergency_stop()
                else:return self.reply({"error":"未找到"},404)
                self.reply({"ok":True})
            except Exception as exc:self.reply({"error":str(exc)},400)
    server=ThreadingHTTPServer(("127.0.0.1",port),Handler)
    server.panel_token=token
    return server

def main():
    p=argparse.ArgumentParser(description="PH-EXO中文阻抗调参界面，打开后不自动连接或启动电机")
    p.add_argument("--preview",action="store_true",help="只看界面/导入结果，禁止实机启动")
    p.add_argument("--port",type=int,default=0)
    p.add_argument("--no-browser",action="store_true")
    p.add_argument("--result",type=Path,help="启动时只读打开一个本地result.json")
    p.add_argument("--mode",choices=("ros","bench"),help="ros连接上层；bench独立调参")
    p.add_argument("--legacy",action="store_true",help="兼容参数：等价于--mode bench")
    p.add_argument("--software",action="store_true",help="ROS入口拒绝实机；独立入口使用私有模拟串口")
    p.add_argument("--api",default="/exoskeleton/impedance")
    p.add_argument("--node",default="/exoskeleton_control")
    args=p.parse_args()
    try:mode=resolve_mode(args.mode,args.legacy)
    except ValueError as exc:p.error(str(exc))
    panel=create_panel(mode,preview=args.preview,software=args.software,api=args.api,node=args.node)
    if args.result:
        r=json.loads(args.result.read_text(encoding="utf-8"))
        if not isinstance(r,dict) or not isinstance(r.get("statuses"),list) or not isinstance(r.get("samples"),list):
            raise ValueError("请选择台架result.json")
        panel.result={"result":r}
        panel.add({"type":"loaded","name":args.result.name})
    server=make_server(panel,args.port)
    closed=threading.Event()
    def watchdog():
        while not closed.wait(.5):panel.watchdog()
    threading.Thread(target=watchdog,daemon=True).start()
    url="http://127.0.0.1:"+str(server.server_port)
    print("PH-EXO "+("ROS系统调参" if mode=="ros" else "独立台架调参")+"："+url,flush=True)
    print("打开界面不会启机；"+("先连接上层ROS读取状态。" if mode=="ros" else "不要求上层ROS预先启动，点击开始才创建本轮底层控制。")+"Ctrl+C关闭界面服务。",flush=True)
    if not args.no_browser:webbrowser.open(url)
    try:server.serve_forever(poll_interval=.2)
    except KeyboardInterrupt:pass
    finally:
        if hasattr(panel,"shutdown"):panel.shutdown()
        else:panel.stop("界面服务关闭")
        closed.set();server.server_close()
        if panel.proc is not None:
            try:panel.proc.wait(timeout=35)
            except subprocess.TimeoutExpired:print("未确认远端退出，请检查现场。",flush=True)

if __name__=="__main__":main()
