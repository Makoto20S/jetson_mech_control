"""SSH stdio client of the existing ROS upper node; no device or manager ownership."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import queue
import re
import signal
import sys
import threading
import time

sys.path.insert(0,str(Path(__file__).resolve().parents[2]/"ros2_ws/src/mech_assist_control"))
from mech_assist_control.robot import ph_exo_reference as reference

PREFIX = "PH_EXO_ROS "
DEFAULT = {"k": [20., 20.], "b": [.3, .3], "tau": [10., 10.]}
ACTIVE = {"preparing", "starting", "running", "stopping"}
API = "/exoskeleton/impedance"
NODE = "/exoskeleton_control"

def signature():
    return hashlib.sha256(Path(__file__).read_bytes()+Path(reference.__file__).read_bytes()).hexdigest()

def validate_config(value):
    if not isinstance(value, dict) or set(value) not in tuple({"k","b","tau"}|p|r for p in (set(),{"pose_deg","move_s"}) for r in (set(),{"amp_deg","period_s"})):
        raise ValueError("需要k/b/tau及可选目标、往返参数")
    out = {}
    for key, cap, zero in (("k", 30., False), ("b", 1., True), ("tau", 10., False)):
        values = value[key]
        if not isinstance(values, list) or len(values) != 2:
            raise ValueError(key + " 必须包含两个数值")
        if any(isinstance(v, bool) or not isinstance(v, (float, int)) or
               not math.isfinite(v) or v > cap or (v < 0 if zero else v <= 0) for v in values):
            raise ValueError(key + " 超出当前 ROS 接口范围")
        out[key] = list(map(float, values))
    extra={k:v for k,v in value.items() if k not in out}
    for key in ('pose_deg','amp_deg','period_s'):
        if key in extra and (not isinstance(extra[key],list) or len(extra[key])!=2 or any(type(v) not in (float,int) or not math.isfinite(v) for v in extra[key])):
            raise ValueError(key+"需要两轴有限数值")
    reference.from_degrees(extra)
    return dict(out,**extra)

def parameters(config):
    c=validate_config(config)
    return dict(reference.from_degrees(c),**{k:c[k] for k in ('k','b','tau')})

def route(value):
    if not isinstance(value, str) or not re.fullmatch(r"/[A-Za-z_][A-Za-z_0-9]*(/[A-Za-z_][A-Za-z_0-9]*)*", value):
        raise ValueError("需要绝对 ROS 名称")
    return value

def clean(value):
    if isinstance(value, float) and not math.isfinite(value): return None
    if isinstance(value, dict): return {k: clean(v) for k, v in value.items()}
    if isinstance(value, (tuple, list)): return [clean(v) for v in value]
    return value

def sample(ctrl, robot, now):
    """Display only; never feed an interpolated or cached value into control."""
    row = dict(observed_s=now, q=[], dq=[], reference=[], mode=0, reason="",
               state_valid=False, motor_target_nm={}, feedback={})
    if ctrl is not None and 0 <= now - ctrl.observed_at_monotonic_s < .25:
        row.update(q=list(ctrl.position_rad), dq=list(ctrl.velocity_rad_s),
                   reference=list(ctrl.reference_rad), mode=ctrl.mode, reason=ctrl.reason,
                   state_valid=ctrl.state_valid,
                   motor_target_nm={str(mid): ctrl.motor_target_nm[i] for i, mid in enumerate((104,105))})
    if robot is not None and robot.sources[0].valid and 0 <= robot.sources[0].source_age_ms < 100:
        for m in robot.mit_motors:
            if m.valid and 0 <= m.age_ms < 100:
                row["feedback"][str(m.drive_id)] = dict(mit_position_deg=m.position_deg,
                    rpm=m.output_velocity_rad_s*30/math.pi if m.output_velocity_valid else None,
                    mit_erpm=m.erpm, mit_iq_a=m.iq_a)
        if not row["feedback"]:
            for i, name in enumerate(robot.joints.name):
                if name not in ("motor104_joint","motor105_joint"): continue
                mid = name[5:8]
                row["feedback"][mid] = dict(
                    mit_position_deg=math.degrees(robot.joints.position[i]) if i<len(robot.joints.position) else None,
                    rpm=robot.joints.velocity[i]*30/math.pi if i<len(robot.joints.velocity) else None)
    return clean(row)

def main(argv=None):
    parser=argparse.ArgumentParser(description="ROS界面接口；只调用现有ROS服务，不启动硬件进程")
    parser.add_argument("--check",action="store_true")
    parser.add_argument("--software",action="store_true")
    args=parser.parse_args(argv)
    if args.check:
        print(json.dumps(dict(signature=signature(),defaults=DEFAULT)));return 0
    request=json.loads(sys.stdin.readline())
    if request.get("sig")!=signature(): raise ValueError("两端ROS接口版本不一致")
    api=route(request.get("api",API)); node_name=route(request.get("node",NODE))
    if sys.platform!="linux": raise RuntimeError("ROS客户端只在Linux运行")
    import rclpy
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.signals import SignalHandlerOptions
    from rclpy.qos import qos_profile_sensor_data
    from std_msgs.msg import String
    from std_srvs.srv import Trigger
    from rcl_interfaces.srv import SetParametersAtomically,GetParameters
    from ph_exo_interfaces.msg import ImpedanceStatus
    from mech_exo_state.msg import RobotState
    incoming=queue.Queue(maxsize=32); eof=threading.Event()
    def read():
        try:
            for line in sys.stdin:
                if len(line)>4096: raise ValueError("请求过长")
                incoming.put_nowait(json.loads(line))
        except Exception: pass
        finally: eof.set()
    threading.Thread(target=read,daemon=True).start()
    for sig in (signal.SIGINT,signal.SIGTERM,signal.SIGHUP):
        signal.signal(sig,lambda *_:eof.set())
    rclpy.init(args=[],signal_handler_options=SignalHandlerOptions.NO)
    node=Node("ph_exo_panel_client")
    def emit(kind,**data):
        try: print(PREFIX+json.dumps(clean(dict(type=kind,**data)),allow_nan=False),flush=True)
        except (BrokenPipeError,OSError): eof.set()
    clients={name:node.create_client(Trigger,api+"/"+name) for name in
             ("start","stop","emergency_stop","reset","status")}
    params=node.create_client(SetParametersAtomically,node_name+"/set_parameters_atomically")
    get_params=node.create_client(GetParameters,node_name+"/get_parameters")
    state={}; last_state=0.; robot=None; robot_at=0.; ctrl=None
    pending={}; generation=0; starting=False; owned=False; closing=None
    last_ping=time.monotonic(); last_seq=0; ready=False
    def on_state(msg):
        nonlocal state,last_state
        try: data=json.loads(msg.data)
        except (ValueError,TypeError): return
        if not isinstance(data,dict) or data.get("state") not in ACTIVE|{"debug","stopped","unconfirmed"}:return
        state=data;last_state=time.monotonic()
        emit("state",status=state)
    node.create_subscription(String,api+"/state",on_state,1)
    def on_ctrl(msg):
        nonlocal ctrl
        ctrl=msg
    node.create_subscription(ImpedanceStatus,api+"/controller_state",on_ctrl,1)
    last_robot_stamp=0
    def on_robot(msg):
        nonlocal robot,robot_at,last_robot_stamp
        source=msg.sources[0]
        stamp=source.source_stamp.sec*10**9+source.source_stamp.nanosec
        age=node.get_clock().now().nanoseconds-stamp
        if stamp>last_robot_stamp and source.valid and 0<=age<100_000_000:
            last_robot_stamp=stamp;robot=msg;robot_at=time.monotonic()
    def reply(cmd,seq,ok,message):
        emit("reply",cmd=cmd,seq=seq,ok=bool(ok),message=message)
    def async_call(client,req,done):
        if not client.service_is_ready():
            done(None,"ROS服务不可用："+client.srv_name);return
        future=client.call_async(req);pending[future]=(time.monotonic()+4,done)
    def trigger(name,cmd,seq):
        def done(res,error):
            nonlocal owned
            if cmd=="reset" and res is not None and res.success:owned=False
            reply(cmd,seq,res is not None and res.success,error or res.message)
        async_call(clients[name],Trigger.Request(),done)
    def command(data):
        nonlocal generation,starting,owned,last_ping,last_seq
        cmd=data.get("cmd")
        if cmd=="ping":last_ping=time.monotonic();return
        seq=data.get("seq")
        if not isinstance(seq,int) or isinstance(seq,bool) or seq<=last_seq:raise ValueError("命令序号必须递增")
        last_seq=seq
        if cmd=="close":eof.set();return
        if cmd in ("stop","estop"):
            generation+=1;starting=False
            trigger("emergency_stop" if cmd=="estop" else "stop",cmd,seq);return
        if cmd=="reset":
            generation+=1;starting=False
            trigger("reset",cmd,seq);return
        if cmd!="start":raise ValueError("未知命令")
        config=validate_config(data.get("config"))
        if starting or state.get("state")!="debug" or time.monotonic()-last_state>1:
            raise ValueError("ROS尚未处于新鲜的调试状态")
        if args.software and state.get("backend") not in ("simulation","mit_pty"):
            raise ValueError("软件测试拒绝实机后端")
        generation+=1; current=generation;starting=True
        req=SetParametersAtomically.Request()
        req.parameters=[Parameter("hold."+key,value=value).to_parameter_msg() for key,value in parameters(config).items()]
        def configured(res,error):
            nonlocal starting,owned
            if current!=generation:
                reply("start",seq,False,"启动已取消");return
            if error or not res.result.successful:
                starting=False;reply("start",seq,False,error or res.result.reason);return
            # Stop/EOF cancels an in-flight parameter request before it can start.
            if eof.is_set():starting=False;reply("start",seq,False,"连接已取消");return
            owned=True
            def started(res,error):
                nonlocal starting,owned
                starting=False
                if res is not None and not res.success:owned=False
                if error:eof.set()
                reply("start",seq,res is not None and res.success,error or res.message)
            async_call(clients["start"],Trigger.Request(),started)
        async_call(params,req,configured)
    started_at=time.monotonic();last_sample=0.
    # Resolve the actual input topic; no fixed namespace assumption.
    def configured_topic(res,error):
        nonlocal ready
        if error or not res.values or not res.values[0].string_value:
            emit("error",message=error or "未配置ROS状态输入");eof.set();return
        node.create_subscription(RobotState,route(res.values[0].string_value),on_robot,qos_profile_sensor_data)
        ready=True;emit("ready",software=args.software)
    requested_topic=False
    try:
        while rclpy.ok():
            rclpy.spin_once(node,timeout_sec=.01);now=time.monotonic()
            for f,(deadline,done) in list(pending.items()):
                if f.done() or now>=deadline:
                    del pending[f]
                    if f.done():
                        try:result=f.result()
                        except Exception as exc:done(None,str(exc))
                        else:done(result,"")
                    else:
                        # Remove pending request so a late response cannot trigger start.
                        client_error="ROS服务结果未确认"
                        done(None,client_error)
            if not requested_topic and get_params.service_is_ready():
                req=GetParameters.Request();req.names=["hold.state"]
                async_call(get_params,req,configured_topic);requested_topic=True
            if not ready and now-started_at>8:emit("error",message="未找到ROS上层接口");eof.set()
            for _ in range(8):
                try:data=incoming.get_nowait()
                except queue.Empty:break
                try:command(data)
                except Exception as exc:reply(data.get("cmd",""),data.get("seq",0),False,str(exc))
            if now-last_sample>=.05:
                emit("sample",sample=sample(ctrl,robot if now-robot_at<.1 else None,now))
                last_sample=now
            if now-last_ping>3:eof.set()
            if owned and now-last_state>1 and closing is None:
                emit("error",message="ROS状态中断，已请求停止");eof.set()
            if eof.is_set() and closing is None:
                closing=now;generation+=1;starting=False
                if owned:trigger("stop","close",0)
            if closing is not None:
                if not owned or state.get("state") in ("stopped","unconfirmed") or now-closing>12:break
    finally:
        if owned and closing is None and state.get("state") in ACTIVE:
            trigger("stop","close",0)
            end=time.monotonic()+12
            while rclpy.ok() and time.monotonic()<end and state.get("state") in ACTIVE:
                rclpy.spin_once(node,timeout_sec=.02)
        emit("closed",status=state,stop_ok=state.get("stop_ok",False))
        node.destroy_node();rclpy.shutdown()
    return 0 if not owned or state.get("stop_ok") else 2

if __name__=="__main__":raise SystemExit(main())
