"""Explicit short supported-bench hold using the native REF01 controller and MIT adapter.
--rehearse owns private PTYs. --execute opens only the configured 104/105 stable ports.
No automatic restart. Ctrl+C stops renewal and runs the independent disable path.
"""
import argparse, gc, hashlib, json, math, os, signal, struct, subprocess, sys, time, threading
from contextlib import ExitStack, contextmanager
from collections import deque
from copy import deepcopy
from pathlib import Path
from uuid import uuid4
import xml.etree.ElementTree as ET
import mit_communication as mit
from mit_axis_pulse import passive, require
ROOT=mit.ROOT
JOINTS=("motor104_joint","motor105_joint")
PATHS={104:"/dev/serial/by-id/usb-Livelybot_Livelybot_Communication_Board_2024051701-if02",
       105:"/dev/serial/by-id/usb-Livelybot_Livelybot_Communication_Board_2024051701-if00"}
# The live bench and state node share this robot baseline. Historical synthetic
# MIT fixtures retain their explicitly documented mapping.
SPEED_CONFIG=ROOT/"ros2_ws/src/mech_exo_state/config/ph_exo_mit_state.yaml"

def feedback_mapping(path=SPEED_CONFIG):
    import yaml
    raw=path.read_bytes()
    p=yaml.safe_load(raw)["ph_exo_state"]["ros__parameters"]
    poles=p["mit_pole_pairs"];ratios=p["mit_gear_ratios"]
    signs=p["mit_speed_directions"];confirmed=p["mit_speed_confirmed"]
    if (not all(len(x)==2 for x in (poles,ratios,signs,confirmed)) or
        not all(type(x) is int and x>0 for x in poles) or
        not all(type(x) in (int,float) and math.isfinite(x) and x>0 for x in ratios) or
        not all(type(x) is int and x==1 for x in signs) or
        not all(type(x) is bool for x in confirmed)):
        raise ValueError("Invalid live speed mapping; kinematic adapter requires positive encoder directions")
    axes={str(mid):dict(pole_pairs=poles[i],gear_ratio=ratios[i],direction=signs[i],
        rad_s_per_erpm=2*math.pi/(60*poles[i]*ratios[i]),confirmed=confirmed[i])
        for i,mid in enumerate((104,105))}
    return dict(source="ros2_ws/src/mech_exo_state/config/ph_exo_mit_state.yaml",
                sha256=hashlib.sha256(raw).hexdigest(),axes=axes)

PROFILE="ref01_supported_v1"
# Output-space [K_psi, K_beta, B_psi, B_beta]; not drive MIT Kp/Kd.
GAINS=(10.,10.,1.,1.)
TORQUE_CAP_NM=10.
CURRENT_LIMIT_A=6.
TRAVEL_LIMIT_DEG=90.


DEFAULT_LIMITS=dict(torque_nm=10.,current_a=6.,travel_deg=90.)

def validate_limits(limits):
    require(isinstance(limits,dict) and set(limits)==set(DEFAULT_LIMITS),"需要力矩、电流和电机角度三个限幅")
    require(all(isinstance(v,(int,float)) and not isinstance(v,bool) and math.isfinite(v) and
                0<v<=DEFAULT_LIMITS[k] for k,v in limits.items()),"限幅必须为正有限数，且不超过10 N·m / 6 A / 90°")
    return {k:float(v) for k,v in limits.items()}

def check_feedback(mid, d, reference, now, limits=None):
    limits=DEFAULT_LIMITS if limits is None else limits
    age=now-d["mit_rx_us"]*1e-6
    travel=d["mit_position_deg"]-reference
    require(d["mit_drive_id"]==mid and d["mit_valid"]==1 and d["mit_fault_code"]==0,
            f"{mid} invalid_feedback: drive={d['mit_drive_id']} valid={d['mit_valid']} fault={d['mit_fault_code']}")
    require(0<=age<=.060,f"{mid} feedback_stale: age_ms={age*1000:.3f}, limit_ms=60")
    require(math.isfinite(d["mit_erpm"]),f"{mid} nonfinite_speed")
    require(abs(d["mit_iq_a"])<=limits["current_a"],f"{mid} current_limit: iq_a={d['mit_iq_a']:.3f}, limit_a={limits['current_a']:g}")
    require(0<d["mit_temperature_c"]<60,f"{mid} temperature_limit: temperature_c={d['mit_temperature_c']}")
    require(abs(travel)<=limits["travel_deg"],
            f"{mid} travel_limit: position_deg={d['mit_position_deg']:.3f}, reference_deg={reference:.3f}, delta_deg={travel:.3f}, limit_deg={limits['travel_deg']:g}")

def new_output_path():
    # Naming only: does not open ports or create the directory.
    return ROOT/".local/validation"/("ref01_bench_"+time.strftime("%Y%m%d_%H%M%S")+"_"+uuid4().hex[:8])

def configuration(cfg, paths, ref, out, ns, rehearsal, predictive=False, *, manual_stop=False, limits=None, dynamic=False, startup_s=0.):
    require(not (dynamic and predictive),"Dynamic references require ordinary impedance")
    require(math.isfinite(startup_s) and 0<=startup_s<=61 and (startup_s==0 or dynamic),"Invalid startup interval")
    limits=validate_limits(DEFAULT_LIMITS if limits is None else limits)
    require(set(paths)=={104,105} and set(ref)=={104,105},"Both axes required")
    require((all(p.startswith("/dev/pts/") and p[9:].isascii() and p[9:].isdigit() for p in paths.values())
             if rehearsal else paths==PATHS),"Unexpected endpoints")
    cfg=deepcopy(cfg)
    for m in cfg["motors"]:
        next(b for b in cfg["buses"] if b["logical_bus"]==m["logical_bus"])["device_path"]=paths[m["id"]]
    previous=mit.NS;mit.NS=ns
    try:xml,params=mit.artifacts(cfg,out)
    finally:mit.NS=previous
    robot=ET.fromstring(xml)
    speed=feedback_mapping()["axes"]
    for control in robot.findall("ros2_control"):
        hw=control.find("hardware");mid=int(hw.find("param[@name='drive_id']").text)
        hw.find("plugin").text="mech_bringup/Ak30KinematicSystem"
        # The unchanged shared runtime requires a positive ERPM threshold.
        # Cover every signed int16*10 feedback value for this bench only:
        # speed is recorded, with no in-range speed cutoff. Other profiles retain theirs.
        values=dict(simulation_only="false",bench_profile=PROFILE,bench_rehearsal=str(rehearsal).lower(),
                    bench_reference_deg=str(ref[mid]),kinematic_erpm_to_rad_s=str(speed[str(mid)]["rad_s_per_erpm"]),
                    torque_max_abs_erpm="327680",control_period_ns="5000000",command_ttl_ns="5500000",
                    command_hard_ttl_ns="6000000",feedback_telemetry_log="false")
        if manual_stop or limits!=DEFAULT_LIMITS:
            values.update(bench_manual_stop=str(manual_stop).lower(),
                bench_torque_limit_nm=str(limits["torque_nm"]),
                bench_current_limit_a=str(limits["current_a"]),
                bench_travel_limit_deg=str(limits["travel_deg"]))
        if startup_s>0:values["bench_startup_s"]=str(startup_s)
        if manual_stop:
            # Fixed-capacity full-rate trace is a bounded experiment recorder,
            # not an endless logger. GUI observations are retained in a rolling buffer.
            trace_param=hw.find("param[@name='command_trace_path']")
            if trace_param is not None:hw.remove(trace_param)
        for key,value in values.items():
            item=hw.find("param[@name='"+key+"']")
            if item is None:item=ET.SubElement(hw,"param",name=key)
            item.text=value
        joint=control.find("joint");old=joint.get("name");name=JOINTS[mid-104]
        joint.set("name",name);robot.find("joint[@name='"+old+"']").set("name",name)
        for field in ("sample_time_s","sequence")+(("accepted_effort","accepted_time_s","accepted_sequence") if predictive else ()):
            ET.SubElement(joint,"state_interface",name=field)
    xml=ET.tostring(robot,encoding="unicode")
    mgr=params[ns+"/controller_manager"]["ros__parameters"]
    mgr.update(robot_description=xml,update_rate=200,use_sim_time=False,lock_memory=False)
    for mid in (104,105):
        del mgr["zero_effort_"+str(mid)];del params[ns+"/zero_effort_"+str(mid)]
    mgr["ref01_impedance"]={"type":"mech_controllers/Ref01ImpedanceController"}
    params[ns+"/ref01_impedance"]={"ros__parameters":dict(simulation_only=False,bench_profile=PROFILE,
        model_path=str(ROOT/"mujoco/ref01/config/mujoco/ref01_dynamics_cad_v1.xml"),joints=list(JOINTS),
        predictive_impedance=predictive,receive_time_uncertainty_s=.007 if predictive else 0.)}
    if dynamic:params[ns+"/ref01_impedance"]["ros__parameters"]["command_source"]="dynamic_ref_v1"
    sys.path.insert(0,str(ROOT/"tools/servo"));import servo_operator
    return params,servo_operator.generate_urdf(cfg,observe=True)

def panel_observation(msg,latest):
    # Core TorqueResult.motor_nm is already in [104,105] joint order.
    return dict(observed_s=msg.observed_at_monotonic_s,
        q=list(msg.position_rad),dq=list(msg.velocity_rad_s),
        reference=list(msg.reference_rad),mode=msg.mode,reason=msg.reason,
        state_valid=msg.state_valid,
        motor_target_nm={"104":msg.motor_target_nm[0],"105":msg.motor_target_nm[1]},
        feedback={str(mid):dict(d,rpm=(
            d["velocity"]*30/math.pi if
            type(d.get("velocity")) in (int,float) and math.isfinite(d["velocity"]) and
            d.get("mit_valid")==1 and d.get("mit_fault_code")==0 and
            type(d.get("mit_rx_us")) in (int,float) and
            0<=msg.observed_at_monotonic_s-d["mit_rx_us"]*1e-6<=.060 else None))
            for mid,d in latest.items()})

def emergency_manager_watch(manager, emergency, closed, record):
    while not closed.wait(.005):
        if emergency.is_set():
            record["emergency_received_s"]=time.monotonic()
            if manager.poll() is None:
                try:manager.send_signal(signal.SIGINT)
                except ProcessLookupError:pass
            return

def wait_spawner(command, log, emergency=None):
    child=subprocess.Popen(command,stdout=log,stderr=log)
    deadline=time.monotonic()+20
    try:
        while child.poll() is None:
            if emergency is not None and emergency.is_set():
                raise KeyboardInterrupt("软件急停")
            if time.monotonic()>=deadline:raise TimeoutError("Spawner timeout")
            time.sleep(.005)
        if child.returncode:raise subprocess.CalledProcessError(child.returncode,command)
    finally:
        if child.poll() is None:
            child.terminate()
            try:child.wait(timeout=.2)
            except subprocess.TimeoutExpired:child.kill();child.wait(timeout=1)

def stop_emergency_manager(manager):
    if manager is None or manager.poll() is not None:return
    for sig,timeout in ((signal.SIGINT,.15),(signal.SIGTERM,.15),(signal.SIGKILL,.5)):
        if manager.poll() is not None:return
        try:manager.send_signal(sig)
        except ProcessLookupError:return
        try:manager.wait(timeout=timeout);return
        except subprocess.TimeoutExpired:pass
    raise RuntimeError("急停后控制进程仍未退出")

def window(node,manager,ns,ref,qref,duration,scenario,record,gains=GAINS,observer=None,stop_requested=None,limits=None,trajectory=None):
    limits=DEFAULT_LIMITS if limits is None else limits
    import rclpy
    from control_msgs.msg import DynamicJointState
    from std_msgs.msg import Float64MultiArray
    from std_srvs.srv import Trigger
    latest={};samples=deque(maxlen=12000) if duration is None else []
    statuses=deque(maxlen=1200) if duration is None else []
    record.update(samples=samples,statuses=statuses,events={})
    def receive(msg):
        for name,vals in zip(msg.joint_names,msg.interface_values):
            if name not in JOINTS:continue
            mid=104+JOINTS.index(name);d=dict(zip(vals.interface_names,vals.values))
            if mid not in latest or d["mit_rx_sequence"]!=latest[mid]["mit_rx_sequence"]:
                latest[mid]=d;samples.append(dict(observed_s=time.monotonic(),id=mid,**d))
    sub=node.create_subscription(DynamicJointState,ns+"/dynamic_joint_states",receive,1)
    from ph_exo_interfaces.msg import ImpedanceRef
    pub=node.create_publisher(ImpedanceRef if trajectory else Float64MultiArray,
        ns+"/ref01_impedance/"+("ref" if trajectory else "plan"),1)
    if trajectory:record["references"]=deque(maxlen=6000) if duration is None else []
    status=node.create_client(Trigger,ns+"/ref01_impedance/software_status")
    stop=node.create_client(Trigger,ns+"/ref01_impedance/stop")
    typed_sub=None
    if observer is not None:
        from ph_exo_interfaces.msg import ImpedanceStatus
        record["panel_samples"]=deque(maxlen=2400) if duration is None else []
        def receive_status(msg):
            row=panel_observation(msg,latest)
            record["panel_samples"].append(row)
            observer(row)  # Caller must only enqueue, never perform network/file I/O here.
        typed_sub=node.create_subscription(ImpedanceStatus,ns+"/ref01_impedance/status",receive_status,1)
    begin=None;future=None;seq=0;following=False
    try:
        if stop_requested is not None and stop_requested():raise KeyboardInterrupt("界面已请求停止")
        deadline=time.monotonic()+3
        while time.monotonic()<deadline and (len(latest)!=2 or pub.get_subscription_count()!=1):
            if stop_requested is not None and stop_requested():raise KeyboardInterrupt("界面已请求停止")
            rclpy.spin_once(node,timeout_sec=.005)
        require(len(latest)==2 and pub.get_subscription_count()==1 and
                status.wait_for_service(timeout_sec=1.) and stop.wait_for_service(timeout_sec=1.),"Missing native interfaces")
        begin=time.monotonic();record["events"]["first_plan_s"]=begin
        next_plan=begin;next_status=begin;next_print=begin;status_deadline=0
        while duration is None or time.monotonic()-begin<duration-.025:
            if stop_requested is not None and stop_requested():
                record["operator_stopped"]=True
                break
            rclpy.spin_once(node,timeout_sec=.001);now=time.monotonic()
            require(manager.poll() is None,"Manager exited")
            for mid,d in latest.items():
                active_limits=dict(limits,travel_deg=90.) if trajectory and now-begin<getattr(trajectory,"startup_s",0.) else limits
                check_feedback(mid,d,ref[mid],now,active_limits)
            if now>=next_plan and not (scenario=="plan_loss" and now-begin>=1.):
                require(now-next_plan<.045,"Plan publisher late")
                target=trajectory.sample(now-begin,qref) if trajectory else (qref,[0.,0.])
                if target is not None:
                    seq+=1
                    if trajectory:
                        q,dq=target
                        pub.publish(ImpedanceRef(id=1,seq=seq,t=now,on=True,q=q,dq=dq,
                            k=list(gains[:2]),b=list(gains[2:]),tau=[limits["torque_nm"]]*2))
                        record["references"].append(dict(t=now,seq=seq,q=q,dq=dq))
                    else:pub.publish(Float64MultiArray(data=[1.,float(seq),now,1.,*qref,*gains,limits["torque_nm"],limits["torque_nm"]]))
                elif "reference_cut_s" not in record["events"]:
                    record["events"]["reference_cut_s"]=now
                    print("已暂停参考发布，正在观察过期停发。",flush=True)
                next_plan=now+.020
            if future is not None:
                require(now<status_deadline,"Native status timeout")
                if future.done():
                    reply=future.result();require(reply and reply.success,"Native status unavailable")
                    state=json.loads(reply.message);state["received_s"]=now;statuses.append(state);future=None
                    if (trajectory and state["mode"]==3 and state["reason_name"]=="plan_expired" and
                            "reference_cut_s" in record["events"]):
                        record["expected_reference_fault"]=True
                        print("已观察到参考过期：停止测试并独立失能。",flush=True)
                        break
                    require(state["mode"]!=3,"Native controller fault: "+state["reason_name"])
                    if state["mode"]==2 and not following:
                        following=True;record["events"]["following_s"]=now
                        print("HOLD ACTIVE：阻抗保持已生效；"+("手动停止" if duration is None else f"最长{duration:g}秒")+
                              f"；每台力矩{limits['torque_nm']:g} N·m，电流{limits['current_a']:g} A，相对位移±{limits['travel_deg']:g}°。",flush=True)
            if future is None and now>=next_status:
                future=status.call_async(Trigger.Request());status_deadline=now+.4;next_status=now+.1
            require(following or now-begin<1.,"Native controller did not follow")
            if now>=next_print:
                print("保持时间",round(now-begin,2),"秒；电机编号 / 角度(°) / 电流(A)：",[(i,latest[i]["mit_position_deg"],latest[i]["mit_iq_a"]) for i in (104,105)],flush=True)
                next_print=now+1
        require(following or record.get("operator_stopped"),"Never followed")
    finally:
        record["events"]["stop_requested_s"]=time.monotonic()
        if manager.poll() is None and rclpy.ok() and stop.service_is_ready():
            future_stop=stop.call_async(Trigger.Request())
            rclpy.spin_until_future_complete(node,future_stop,timeout_sec=.025)
            record["stop_service_replied"]=future_stop.done() and future_stop.result() is not None
            time.sleep(.025)
        node.destroy_subscription(sub)
        if typed_sub is not None:node.destroy_subscription(typed_sub)

def confirm_terminal(duration, ref, qref, predictive=False, gains=GAINS):
    require(sys.stdin.isatty(),"实机启动必须在可见的交互终端确认")
    print("\n准备完成 / 等待操作员确认",flush=True)
    print("控制模式：", "预测阻抗（主机时间候选版本）" if predictive else "普通阻抗（使用测量状态，预测关闭）",flush=True)
    print("持续时间：%.3g 秒；每台目标力矩上限：10 N·m；反馈电流：6 A；相对启动位置：±90°。" % duration,flush=True)
    print("驱动Kp=Kd=0；直接跟踪目标；速度仅记录；启动电机角度(°)：",ref,flush=True)
    print("输出轴 [ψ,β] 刚度(N·m/rad)：",gains[:2],"阻尼(N·m·s/rad)：",gains[2:],flush=True)
    print("MuJoCo启动姿态 [ψ,β](°)：",[math.degrees(x) for x in qref],flush=True)
    require(input("输入 START 并回车才会启动；其他输入取消：").strip()=="START",
            "操作员已取消，电机未启动")
    return time.monotonic()

def validate_rehearsal_command(body,cap):
    """Check a private PTY command against this bench's configured cap."""
    base=bytes.fromhex(mit.ZERO_WIRE)
    if (len(body)!=14 or body[4:6]!=bytes.fromhex("0c08") or
            body[6:12]!=base[:6] or body[12]>>4!=base[6]>>4):
        raise ValueError("Unexpected MIT mode, gain, position or velocity")
    if not math.isfinite(cap) or not 0<cap<=10:raise ValueError("Invalid software cap")
    code=int.from_bytes(body[-2:],"big")&4095
    if abs(code*30/4095-15)>cap+30/4095:
        raise ValueError("Effort exceeds configured software fixture envelope")

class PreflightRejected(RuntimeError):
    """A read-only startup check failed before the controller was launched."""


@contextmanager
def read_only_preflight():
    try:
        yield
    except Exception as exc:
        raise PreflightRejected(f"启动预检失败，本轮未使能：{exc}") from exc


def validate_start_reference(qref):
    require(abs(qref[0]) <= math.pi/4 + 1e-12, "启动参考 ψ 超出 ±45°")
    require(math.radians(-8) - 1e-12 <= qref[1] <= math.pi/4 + 1e-12,
            "当前 β 可计算，但启动参考必须在 −8°～+45° 内")


def run(out,rehearsal,duration,scenario,predictive=False,gains=GAINS,*,confirm=None,observer=None,stop_requested=None,manual_stop=False,limits=None,emergency=None,trajectory=None):
    validate_gains(gains)
    require(not (trajectory and predictive),"Dynamic references require ordinary mode")
    limits=validate_limits(DEFAULT_LIMITS if limits is None else limits)
    if manual_stop:
        require(duration is None and confirm is not None and stop_requested is not None and observer is not None,
                "手动停止模式必须使用完整界面适配入口")
    else:
        require(duration is not None and 2<=duration<=30,"计时模式需2～30秒")
    import fcntl,rclpy
    from ament_index_python.packages import get_package_prefix
    require(rehearsal or scenario=="normal","Physical mode only supports normal hold")
    require(out.is_relative_to(ROOT/".local/validation") and not out.exists() and
            not any(p.is_symlink() for p in [out,*out.parents]),"Output must be a new nonlinked directory below .local/validation; omit --output to generate a fresh name.")
    out.mkdir(parents=True)
    os.environ.update(ROS_DOMAIN_ID="181",ROS_LOCALHOST_ONLY="1",ROS_LOG_DIR=str(out/"ros_logs"))
    install=ROOT/".local/ros/ref01_native/install"
    for package in ("mech_bringup","mech_controllers"):
        require(Path(get_package_prefix(package)).resolve()==install/package,"Wrong native overlay")
    binary=install/"mech_bringup/lib/mech_bringup"
    manager_bin=Path(get_package_prefix("controller_manager"))/"lib/controller_manager/ros2_control_node"
    require(str(manager_bin)=="/opt/ros/humble/lib/controller_manager/ros2_control_node","Official manager required")
    cfg=mit.selected_config(json.loads((ROOT/"config/parallel_exo/servo_pair.ref01.json").read_text()),[104,105])
    paths={m["id"]:next(b["device_path"] for b in cfg["buses"] if b["logical_bus"]==m["logical_bus"]) for m in cfg["motors"]}
    require(paths==PATHS,"Current config differs from physical mapping")
    ns="/ph_exo_bench_"+uuid4().hex[:10]
    record=dict(rehearsal=rehearsal,scenario=scenario,requested_duration_s=duration,target_cap_nm=limits["torque_nm"],gains=list(gains),effort_policy="direct",
                limits=dict(iq_a=limits["current_a"],travel_deg=limits["travel_deg"],erpm=None,slew_nm_s=None,absolute_time_s=None if manual_stop else 30.),
                manual_stop=manual_stop,recording="rolling_ros_state" if manual_stop else "full_bounded_trace",
                control_mode="predictive_host_time" if predictive else "ordinary_measured_state",
                predictive_impedance=predictive,receive_time_uncertainty_s=.007 if predictive else 0.,
                feedback_mapping=feedback_mapping())
    gateways=[];manager=None
    with ExitStack() as stack:
        for name in ("mit_communication.lock","sync.lock"):
            lock=stack.enter_context((ROOT/".local"/name).open("a+"));fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        require(not (ROOT/".local/sync/incomplete.json").exists(),"Incomplete sync")
        was_gc=gc.isenabled();gc.disable()
        if was_gc:stack.callback(gc.enable)
        if rehearsal:
            from test_mit_pair_pty import PairGateway
            from benchmark_mit_pty import packet
            class BenchGateway(PairGateway):
                def validate_command(self,body):
                    validate_rehearsal_command(body,limits["torque_nm"])
                def send_feedback(self,acknowledge=False):
                    commands=[r for r in self.rows if r["id"]==0x800+self.drive]
                    elapsed=(time.monotonic_ns()-commands[0]["ns"])*1e-9 if commands else 0.
                    if scenario=="stale" and 1.<elapsed<1.5 and not acknowledge:return
                    pos=(806 if self.drive==104 else 813)
                    if scenario=="travel" and self.drive==104 and elapsed>1.:pos+=15
                    body=struct.pack("<IBB",0x2900+self.drive,4,8)
                    os.write(self.master,packet(body+struct.pack(">hhhBB",pos,0,0,41,119 if acknowledge else 0)))
            for mid in (104,105):
                g=BenchGateway(mid);gateways.append(g);stack.callback(g.close);paths[mid]=g.path
        else:
            for p in paths.values():
                require(Path(p).resolve(strict=True).is_char_device(),"Missing physical port")
                owner=subprocess.run(["fuser",p],capture_output=True)
                require(owner.returncode==1 and not owner.stdout and not owner.stderr,"Port occupied")
        params,disable=configuration(cfg,paths,{104:80.,105:80.},out,ns,rehearsal,predictive,manual_stop=manual_stop,limits=limits,dynamic=trajectory is not None)
        desc=out/"disable.urdf";desc.write_text(disable)
        subprocess.run([str(binary/"servo_disable"),"--urdf",str(desc),"--check"],check=True,timeout=4)
        with read_only_preflight():
            try:
                ref = passive(binary, desc, out)
            except (subprocess.CalledProcessError, subprocess.TimeoutExpired) as exc:
                raise RuntimeError('启动前反馈读取失败，请检查电机供电与接线后重试') from exc
            record["reference_deg"] = ref
            sys.path.insert(0,str(ROOT/"mujoco/ref01"))
            from ref01_mujoco_kinematics import MuJoCoForwardKinematics
            qref=list(MuJoCoForwardKinematics().solve([math.radians(ref[105])-math.pi/4,
                                                    math.radians(ref[104])-math.pi/4]).q)
            validate_start_reference(qref)
            record["reference_q_rad"]=qref
            if trajectory:trajectory.validate(qref)
            if not rehearsal:
                gate=confirm_terminal if confirm is None else confirm
                key="terminal_confirmed_s" if confirm is None else "interface_confirmed_s"
                record[key]=gate(duration,ref,qref,predictive,gains)
                confirmed=out/"confirmed_pre";confirmed.mkdir()
                fresh=passive(binary,desc,confirmed)
                require(all(abs(fresh[i]-ref[i])<=.3 for i in (104,105)),
                        "Pose changed while waiting; no motor start")
                ref=fresh
                qref=list(MuJoCoForwardKinematics().solve([math.radians(ref[105])-math.pi/4,
                                                        math.radians(ref[104])-math.pi/4]).q)
                validate_start_reference(qref)
                record.update(reference_deg=ref,reference_q_rad=qref)
                if trajectory:trajectory.validate(qref)
            if trajectory and hasattr(trajectory,"validate_encoders"):trajectory.validate_encoders(ref)
        startup_s=getattr(trajectory,"startup_s",0.)
        record["startup"]=dict(seconds=startup_s,travel_deg=90. if startup_s else limits["travel_deg"],current_a=limits["current_a"])
        params,_=configuration(cfg,paths,ref,out,ns,rehearsal,predictive,manual_stop=manual_stop,limits=limits,dynamic=trajectory is not None,startup_s=startup_s)
        controls=out/"controls.json";controls.write_text(json.dumps(params))
        rclpy.init();node=rclpy.create_node("bench_supervisor",namespace=ns)
        stack.callback(rclpy.try_shutdown);stack.callback(node.destroy_node)
        log=stack.enter_context((out/"manager.log").open("w"))
        watch_closed=threading.Event();watch_thread=None
        try:
            if stop_requested is not None and stop_requested():raise KeyboardInterrupt("启动已取消")
            manager=subprocess.Popen([str(manager_bin),"--ros-args","-r","__ns:="+ns,"--params-file",str(controls)],
                stdout=log,stderr=log,start_new_session=True)
            if emergency is not None:
                watch_thread=threading.Thread(target=emergency_manager_watch,args=(manager,emergency,watch_closed,record),daemon=True)
                watch_thread.start()
            for name in ("joint_state_broadcaster","ref01_impedance"):
                wait_spawner(["ros2","run","controller_manager","spawner",name,"-c",ns+"/controller_manager",
                    "--controller-manager-timeout","12","--param-file",str(controls)],
                    log,emergency)
            window(node,manager,ns,ref,qref,duration,scenario,record,gains,observer,stop_requested,limits,trajectory=trajectory)
        except BaseException as exc:record["error"]=repr(exc)
        finally:
            print("正在停止：已停止续发目标，正在核验独立失能。",flush=True)
            try:
                if emergency is not None and emergency.is_set():
                    record["emergency_requested"]=True
                    stop_emergency_manager(manager)
                else:mit.stop_child(manager)
                require(manager is not None and manager.poll() is not None,"Manager exit uncertain")
                for p in paths.values():
                    owner=subprocess.run(["fuser",p],capture_output=True)
                    # Rehearsal owns PTY slave; physical ports must be released.
                    if not rehearsal:require(owner.returncode==1 and not owner.stdout and not owner.stderr,"Port release uncertain")
                ack=subprocess.run([str(binary/"servo_disable"),"--urdf",str(desc),"--trace",str(out/"disable.jsonl"),"--timeout","1"],
                    capture_output=True,text=True,timeout=7)
                (out/"disable.log").write_text(ack.stdout+ack.stderr)
                lines=[json.loads(l) for l in ack.stdout.splitlines() if l.startswith("{")]
                record["disable_ack"]=sorted(x["id"] for x in lines if x.get("event")=="disable_ack")
                require(ack.returncode==0 and record["disable_ack"]==[104,105] and any(
                    x.get("event")=="disable_complete" and x.get("confirmed") and not x.get("missing") for x in lines),
                    "Disable unconfirmed: cut power; do not retry")
                print("已确认失能：104、105",flush=True)
            except BaseException as exc:
                record["shutdown_error"]=repr(exc);print("失能状态不明：请立即断电",repr(exc),flush=True)
            watch_closed.set()
            if watch_thread is not None:watch_thread.join(timeout=.1)
            for mid,bus in ((104,2),(105,1)):
                p=out/f"bus{bus}_motor{mid}.jsonl"
                if p.exists():
                    rows=[json.loads(l) for l in p.read_text().splitlines()]
                    tx=[r for r in rows[1:] if r["stage"]=="transport_result" and r["id"]==0x800+mid]
                    record.setdefault("trace",{})[str(mid)]=dict(count=len(tx),dropped=rows[0]["dropped"],
                        final_zero=bool(tx and tx[-1]["hex"]==mit.ZERO_WIRE),
                        span_s=(tx[-1]["ns"]-tx[0]["ns"])*1e-9 if len(tx)>1 else 0.,
                        max_abs_encoded_nm=max([abs((int(t["hex"][-3:],16)*30/4095)-15) for t in tx],default=0.))
            record["gateway_errors"]=[e for g in gateways for e in g.errors]
            for key in ("samples","statuses","panel_samples","references"):
                if isinstance(record.get(key),deque):record[key]=list(record[key])
            (out/"result.json").write_text(json.dumps(record,indent=2))
            print(json.dumps({k:v for k,v in record.items() if k not in ("samples","statuses","panel_samples","references")}),flush=True)
        require(not record.get("shutdown_error") and not record["gateway_errors"],"Shutdown or emulator error")
        if scenario=="normal":
            require("error" not in record,str(record.get("error")))
            if trajectory and getattr(trajectory,"expects_reference_loss",True):require(record.get("expected_reference_fault"),"Reference interruption was not verified")
            require(manual_stop or (record.get("trace") and all(v["final_zero"] and v["dropped"]==0 for v in record["trace"].values())),
                    "Final zero or trace incomplete")
        else:require("error" in record and any(s["mode"]==2 for s in record.get("statuses",[])),"Fault not observed after following")

def validate_gains(gains):
    require(len(gains)==4 and all(isinstance(x,(int,float)) and not isinstance(x,bool) and math.isfinite(x) for x in gains),
            "K/B必须是四个有限数值")
    require(all(0<x<=30 for x in gains[:2]) and all(0<=x<=1 for x in gains[2:]),
            "普通台架范围：K为(0,30]，B为[0,1]；不改变控制器边界")
    return tuple(float(x) for x in gains)


def parse_args(argv=None):
    p=argparse.ArgumentParser(description="PH-EXO 双电机固定姿态阻抗测试；Ctrl+C停止，不自动重启。");m=p.add_mutually_exclusive_group(required=True)
    m.add_argument("--execute",action="store_true",help="实机模式，必须在终端确认START");m.add_argument("--rehearse",action="store_true",help="软件模拟串口模式")
    p.add_argument("--duration",type=float,default=30.);p.add_argument("--output",type=Path,help="结果目录（必须为新目录）；省略时自动生成")
    p.add_argument("--scenario",choices=("normal","travel","stale","plan_loss"),default="normal")
    mode=p.add_mutually_exclusive_group()
    mode.add_argument("--predictive",dest="predictive",action="store_true",help="预测阻抗候选版本")
    mode.add_argument("--ordinary",dest="predictive",action="store_false",help="普通阻抗：使用测量状态，关闭预测和自适应")
    p.set_defaults(predictive=False)
    p.add_argument("--damping",type=float,default=1.,help="普通模式双轴阻尼B，单位N·m·s/rad，范围0～1；刚度K保持10")
    a=p.parse_args(argv)
    if not math.isfinite(a.damping) or not 0<=a.damping<=1:
        p.error("--damping must be finite and in [0,1]")
    if a.predictive and a.damping!=1:
        p.error("Variable damping is only available with ordinary mode")
    a.gains=(10.,10.,a.damping,a.damping)
    return a

def main():
    a=parse_args()
    require(sys.platform=="linux","请在Orin上运行ROS环境")
    def interrupted(*args):raise KeyboardInterrupt("Operator or connection stopped")
    for sig in (signal.SIGTERM,signal.SIGHUP):signal.signal(sig,interrupted)
    out=a.output.resolve() if a.output is not None else new_output_path()
    print("结果目录：",out,flush=True)
    run(out,a.rehearse,a.duration,a.scenario,a.predictive,a.gains)
if __name__=="__main__":main()
