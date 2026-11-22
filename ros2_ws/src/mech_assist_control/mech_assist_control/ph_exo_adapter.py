"""Optional PH-EXO branch of the existing exoskeleton_control_node.

ROS callbacks never wait for controller_manager. One worker serializes lifecycle
operations; stop revokes plan renewal immediately. No manager process is killed.
"""
import json
import math
import threading
import time
import subprocess
from pathlib import Path
import xml.etree.ElementTree as ET

from std_msgs.msg import String
from std_srvs.srv import Trigger
from ph_exo_interfaces.msg import FixedPosePlan, ImpedanceRef, ImpedanceStatus
from mech_exo_state.msg import RobotState
from controller_manager_msgs.srv import SwitchController, SetHardwareComponentState, ListHardwareComponents, ListControllers
from rcl_interfaces.srv import GetParameters
from rcl_interfaces.msg import SetParametersResult
from rclpy.qos import qos_profile_sensor_data
from ament_index_python.packages import get_package_prefix
from .robot.ph_exo_session import FixedPoseSession
from .robot import ph_exo_reference as reference

class PhExoAdapter:
    def __init__(self,node):
        self.node=node;self.lock=threading.RLock();self.operation_lock=threading.Lock();self.session=FixedPoseSession()
        self.cancel=threading.Event();self.worker=None;self.state=None;self.controller_state=None
        self.renew=False;self.started_at=0.;self.last_status_at=0.;self.last_state_stamp=0
        defaults={"hold.cm":"/ph_exo_manager_launch/controller_manager",
                  "hold.ctrl":"/ph_exo_manager_launch/ref01_impedance",
                  "hold.hw":["ref01_mujoco"],"hold.backend":"simulation",
                  "hold.state":"/ph_exo/state",
                  "hold.k":[2.4,2.0],"hold.b":[.18,.15],
                  "hold.tau":[.2,.2],"hold.disable":"",
                  "hold.log":""}
        defaults.update({"hold."+k:v for k,v in reference.DEFAULT.items()})
        defaults.update({"hold.source":"fixed_pose_v1","hold.profile":""})
        for k,v in defaults.items():node.declare_parameter(k,v)
        get=lambda k:node.get_parameter(k).value
        self.manager=get("hold.cm").rstrip("/");self.controller=get("hold.ctrl").rstrip("/")
        self.controller_name=self.controller.rsplit("/",1)[-1]
        self.hardware=list(get("hold.hw"));self.backend=get("hold.backend")
        if self.backend not in ("simulation","mit_pty","mit"):raise ValueError("非法后端")
        if not self.manager.startswith("/") or not self.controller.startswith("/") or not self.hardware or len(set(self.hardware))!=len(self.hardware):
            raise ValueError("必须明确管理器、控制器和唯一硬件组件")
        if self.backend=="mit" and not (get("allow_motor_output") and get("joint_mapping_verified")):
            raise ValueError("真实电机需要显式许可及已核对映射")
        self.disable_urdf=get("hold.disable");self.output=get("hold.log")
        self.source=get("hold.source");self.profile=get("hold.profile")
        if self.source not in ("fixed_pose_v1","dynamic_ref_v1"):raise ValueError("参考接口无效")
        if self.profile not in ("","ref01_supported_v1"):raise ValueError("参数范围配置无效")
        self.plan_type=ImpedanceRef if self.source=="dynamic_ref_v1" else FixedPosePlan
        self.plan_topic=self.controller+("/ref" if self.source=="dynamic_ref_v1" else "/fixed_pose_plan")
        self.plan_pub=node.create_publisher(self.plan_type,self.plan_topic,1)
        self.trajectory=None;self.ref_cfg=dict(reference.DEFAULT);self.q_ref=None;self.dq_ref=None
        self.status_pub=node.create_publisher(String,"exoskeleton/impedance/state",1)
        self.raw_pub=node.create_publisher(ImpedanceStatus,"exoskeleton/impedance/controller_state",1)
        node.create_subscription(RobotState,get("hold.state"),self.on_state,qos_profile_sensor_data)
        node.create_subscription(ImpedanceStatus,self.controller+"/status",self.on_status,1)
        self.switch=node.create_client(SwitchController,self.manager+"/switch_controller")
        self.hw=node.create_client(SetHardwareComponentState,self.manager+"/set_hardware_component_state")
        self.hw_list=node.create_client(ListHardwareComponents,self.manager+"/list_hardware_components")
        self.ctrl_list=node.create_client(ListControllers,self.manager+"/list_controllers")
        self.parameters=node.create_client(GetParameters,self.manager+"/get_parameters")
        self.ctrl_stop=node.create_client(Trigger,self.controller+"/stop")
        for name,handler in (("start",self.start),("stop",self.stop),("emergency_stop",self.estop),
                             ("reset",self.reset),("status",self.status)):
            node.create_service(Trigger,"exoskeleton/impedance/"+name,handler)
        node.create_timer(0.01, self.tick)
        node.add_on_set_parameters_callback(self.change_parameters)
    def change_parameters(self,values):
        allowed={"hold.k","hold.b","hold.tau"}|{"hold."+k for k in reference.DEFAULT}
        with self.lock:
            if self.session.phase!="debug" or any(p.name not in allowed for p in values):
                return SetParametersResult(successful=False,reason="仅调试状态允许修改增益、力矩和参考参数")
            current={k:self.node.get_parameter(k).value for k in allowed}
            current.update({p.name:p.value for p in values})
            try:
                self.validate_gains(current);self.validate_reference(current)
            except (ValueError,TypeError) as e:return SetParametersResult(successful=False,reason=str(e))
        return SetParametersResult(successful=True)
    def validate_gains(self,c=None):
        c=c or {k:self.node.get_parameter(k).value for k in ("hold.k","hold.b","hold.tau")}
        k=c["hold.k"];b=c["hold.b"];cap=c["hold.tau"]
        if any(len(v)!=2 for v in (k,b,cap)):raise ValueError("每项必须两轴")
        FixedPoseSession().start([0.,.2],list(k)+list(b),cap,time.monotonic())
        if self.backend!="mit" and self.profile!="ref01_supported_v1" and (k[0]>2.4 or k[1]>2 or b[0]>.18 or b[1]>.15 or max(cap)>.2):
            raise ValueError("软件后端沿用低力矩验证范围")
        return list(k)+list(b),list(cap)
    def validate_reference(self,c=None):
        c=c or {"hold."+k:self.node.get_parameter("hold."+k).value for k in reference.DEFAULT}
        result=reference.validate({k:c["hold."+k] for k in reference.DEFAULT})
        if self.source=="fixed_pose_v1" and result['ref']!='fixed':
            raise ValueError("目标/往返模式需要上层和控制器均配置dynamic_ref_v1")
        return result
    def on_state(self,m):
        stamp=m.sources[0].source_stamp.sec*10**9+m.sources[0].source_stamp.nanosec
        now_ros=self.node.get_clock().now().nanoseconds
        good=(m.q_valid and m.q_velocity_valid and m.mapping_confirmed and m.sources[0].valid
              and 0<=m.sources[0].source_age_ms<100 and 0<=m.sources[0].receive_age_ms<100
              and 0<=now_ros-stamp<100_000_000
              and all(math.isfinite(x) for x in (*m.q_position,*m.q_velocity)))
        if self.backend!="simulation":
            good=good and all(x.valid and x.output_velocity_valid and x.host_rx_us>0 for x in m.mit_motors)
        with self.lock:
            if stamp>self.last_state_stamp:
                self.last_state_stamp=stamp
                self.state=(list(m.q_position),time.monotonic(),good)
            elif not good:self.state=None
    def on_status(self,m):
        with self.lock:
            # Retain source observation time; replayed messages never refresh it.
            if m.observed_at_monotonic_s<=self.last_status_at:return
            self.last_status_at=m.observed_at_monotonic_s;self.controller_state=m
            if m.session_id==self.session.session_id and m.mode==2 and m.state_valid:
                if self.session.phase=="starting":self.session.phase="running"
        self.raw_pub.publish(m)
    def call(self,client,req,timeout=3.):
        if not client.wait_for_service(timeout_sec=timeout):raise RuntimeError("服务不可用："+client.srv_name)
        f=client.call_async(req);end=time.monotonic()+timeout
        while not f.done() and time.monotonic()<end:time.sleep(.005)
        if not f.done():raise RuntimeError("服务结果未确认："+client.srv_name)
        result=f.result()
        if result is None:raise RuntimeError("服务无回复")
        return result
    def switch_controller(self,enable):
        req=SwitchController.Request()
        req.activate_controllers=[self.controller_name] if enable else []
        req.deactivate_controllers=[] if enable else [self.controller_name]
        req.strictness=SwitchController.Request.STRICT;req.activate_asap=True;req.timeout.sec=2
        if not self.call(self.switch,req).ok:raise RuntimeError("控制器切换失败")
    def hardware_state(self,name,state):
        req=SetHardwareComponentState.Request();req.name=name;req.target_state.id=state
        result=self.call(self.hw,req)
        if not result.ok or result.state.id!=state:raise RuntimeError("硬件状态未确认："+name)
    def validate_controller(self, controllers=None):
        if self.controller.rsplit("/",1)[0]!=self.manager.rsplit("/",1)[0]:
            raise RuntimeError("控制器与管理器命名空间不一致")
        if controllers is None:
            controllers=self.call(self.ctrl_list,ListControllers.Request()).controller
        target=next((c for c in controllers if c.name==self.controller_name),None)
        if target is None or target.type!="mech_controllers/Ref01ImpedanceController":
            raise RuntimeError("目标控制器缺失或类型不匹配")
        return target
    def validate_targets(self):
        components={c.name:c for c in self.call(self.hw_list,ListHardwareComponents.Request()).component}
        expected="mech_simulation/Ref01MujocoSystem" if self.backend=="simulation" else "mech_bringup/Ak30KinematicSystem"
        for name in self.hardware:
            if name not in components or components[name].class_type!=expected:raise RuntimeError("目标硬件类型不匹配")
        expected_joints={"motor104_joint","motor105_joint"}
        actual={i.name.split("/")[0] for n in self.hardware for i in components[n].command_interfaces}
        if actual!=expected_joints:raise RuntimeError("硬件组件必须恰好对应104/105")
        controllers=self.call(self.ctrl_list,ListControllers.Request()).controller
        self.validate_controller(controllers)
        for c in controllers:
            if c.name!=self.controller_name and c.state=="active" and any(x.split("/")[0] in expected_joints for x in c.claimed_interfaces):
                raise RuntimeError("其他控制器正在占用目标关节")
        if self.backend!="simulation":self.validate_disable_scope()
        return components
    def validate_disable_scope(self):
        desc=Path(self.disable_urdf)
        if not desc.is_absolute() or not desc.is_file():raise RuntimeError("缺少独立失能URDF")
        req=GetParameters.Request();req.names=["robot_description"]
        values=self.call(self.parameters,req).values
        manager_xml=ET.fromstring(values[0].string_value)
        own=ET.parse(desc).getroot()
        def targets(tree,names=None):
            result={}
            for c in tree.findall("ros2_control"):
                if names is not None and c.get("name") not in names:continue
                params={p.get("name"):p.text for p in c.findall("hardware/param")}
                if names is None:
                    if c.findtext("hardware/plugin")!="mech_bringup/Ak30ServoSystem":
                        raise ValueError("失能文件必须使用已有伺服失能合同")
                    ids=[int(j.findtext("param[@name='drive_id']")) for j in c.findall("joint")]
                else:
                    ids=[int(params["drive_id"])]
                for mid in ids:
                    if mid in result:raise ValueError("重复电机ID")
                    result[mid]=(params["device_path"],int(params["logical_bus"]))
            return result
        a=targets(manager_xml,self.hardware);b=targets(own)
        if a!=b or set(a)!={104,105}:raise RuntimeError("失能URDF与目标硬件不一致")
        if self.backend=="mit_pty" and any(not Path(p).resolve().as_posix().startswith("/dev/pts/") for p,bus in a.values()):
            raise RuntimeError("软件模式拒绝真实端口")
        if not Path(self.output).is_absolute():raise RuntimeError("需要绝对输出目录")
        check=subprocess.run([str(Path(get_package_prefix("mech_bringup"))/"lib/mech_bringup/servo_disable"),
                              "--urdf",self.disable_urdf,"--check"],
                             capture_output=True,text=True,timeout=4)
        if check.returncode:raise RuntimeError("独立失能配置检查失败")
    def check_plan_route(self):
        """Require the selected typed controller and one plan producer."""
        end=time.monotonic()+3.
        while self.plan_pub.get_subscription_count()!=1:
            if self.cancel.is_set() or time.monotonic()>=end:
                raise RuntimeError("所选阻抗参考接口缺失或不唯一")
            time.sleep(.01)
        if self.node.count_publishers(self.plan_topic)!=1:
            raise RuntimeError("已有其他阻抗计划发布者")

    def start(self,req,res):
        with self.lock:
            if self.session.phase!="debug" or (self.worker and self.worker.is_alive()):
                res.success=False;res.message="先停止并返回调试";return res
            try:
                self.validate_gains();self.validate_reference()
            except (ValueError,TypeError) as e:res.success=False;res.message=str(e);return res
            self.session.phase="preparing";self.cancel.clear();self.renew=False
            self.worker=threading.Thread(target=self.start_work,daemon=True);self.worker.start()
        res.success=True;res.message="启动请求已接收；以状态running为准";return res
    def start_work(self):
        with self.operation_lock:self._start_work()
    def _start_work(self):
        try:
            components=self.validate_targets()
            self.check_plan_route()
            for name in self.hardware:
                if self.cancel.is_set():raise RuntimeError("启动被取消")
                if components[name].state.id!=3:self.hardware_state(name,3)
            began=time.monotonic()
            while True:
                with self.lock:
                    ready=self.state and self.state[2] and self.state[1]>began
                    if ready:reference_pose=list(self.state[0]);break
                if self.cancel.is_set() or time.monotonic()-began>3:raise RuntimeError("启动姿态不可用或已取消")
                time.sleep(.005)
            if self.cancel.is_set():raise RuntimeError("启动已取消")
            self.ref_cfg=self.validate_reference()
            self.trajectory=reference.create(self.ref_cfg)
            if self.trajectory:self.trajectory.validate(reference_pose)
            self.switch_controller(True)
            with self.lock:
                if self.cancel.is_set():raise RuntimeError("启动已取消")
                gains,caps=self.validate_gains();self.session.phase="debug"
                self.session.start(reference_pose,gains,caps,time.monotonic())
                self.q_ref=list(reference_pose);self.dq_ref=[0.,0.]
                self.started_at=time.monotonic();self.renew=True
        except Exception as e:
            with self.lock:self.session.stop(str(e));self.renew=False;self.cancel.set()
            self._stop_work()
    def request_stop(self,reason):
        with self.lock:
            pending=self.session.phase=="stopping"
            self.cancel.set();self.renew=False;self.session.stop(reason)
            if not pending:
                self.worker=threading.Thread(target=self.stop_work,daemon=True);self.worker.start()
    def stop(self,req,res):
        self.request_stop("停止请求");res.success=True;res.message="已撤销续发；等待停止确认";return res
    def estop(self,req,res):
        self.request_stop("软件急停");res.success=True;res.message="急停已锁存；等待目标硬件失能确认";return res
    def stop_work(self):
        with self.operation_lock:
            if self.session.phase in ("stopped","unconfirmed"):return
            self._stop_work()
    def deactivate_after_fault(self):
        """Restore unavailable interfaces in INACTIVE, never activate drives.

        Humble rejects deactivation of an active controller after its hardware
        has gone UNCONFIGURED. Reconfigure only those exact components, with
        the controller stop latched, then deactivate and verify its state.
        The caller always cleans up communication and obtains independent ACKs.
        """
        if self.backend=="simulation":raise RuntimeError("控制器切换失败")
        components=self.validate_targets()
        if not any(components[n].state.id==1 for n in self.hardware):
            raise RuntimeError("控制器切换失败；不符合故障恢复条件")
        if any(components[n].state.id not in (1,2) for n in self.hardware):
            raise RuntimeError("硬件尚未停用，拒绝恢复接口")
        reply=self.call(self.ctrl_stop,Trigger.Request())
        if not reply.success:raise RuntimeError("控制器停发锁存未确认")
        for name in self.hardware:
            if components[name].state.id==1:self.hardware_state(name,2)
        self.switch_controller(False)
        if self.validate_controller().state!="inactive":
            raise RuntimeError("控制器停用状态未确认")

    def _stop_work(self):
        ack=[False,False]
        errors=[]
        try:
            # Deactivate the selected controller before latching the driver lease.
            # Calling controller ~/stop first makes Humble remove its command
            # interfaces on driver ERROR, which can reject a subsequent switch.
            target=self.validate_controller()
            if target.state=="active":
                try:self.switch_controller(False)
                except Exception as error:
                    try:self.deactivate_after_fault()
                    except Exception as recovery_error:
                        errors.append(str(recovery_error))
                        if self.ctrl_stop.service_is_ready():self.ctrl_stop.call_async(Trigger.Request())
            components=self.validate_targets()
            for name in self.hardware:
                try:
                    if components[name].state.id==3:self.hardware_state(name,2)
                except Exception as error:errors.append(str(error))
                if self.backend!="simulation":
                    try:self.hardware_state(name,1)
                    except Exception as error:errors.append(str(error))
            if self.backend!="simulation":
                folder=Path(self.output)
                if not folder.is_absolute():raise RuntimeError("需要绝对输出目录")
                folder.mkdir(parents=True,exist_ok=True)
                prefix=Path(get_package_prefix("mech_bringup"))
                tag=str(time.monotonic_ns())
                command=[str(prefix/"lib/mech_bringup/servo_disable"),"--urdf",self.disable_urdf,
                         "--trace",str(folder/("disable_trace_"+tag+".jsonl")),"--timeout","1"]
                result=subprocess.run(command,capture_output=True,text=True,timeout=7)
                (folder/("disable_"+tag+".jsonl")).write_text(result.stdout+result.stderr)
                rows=[json.loads(l) for l in result.stdout.splitlines() if l.startswith("{")]
                ids={x["id"] for x in rows if x.get("event")=="disable_ack"}
                ack=[104 in ids,105 in ids]
                confirmed=result.returncode==0 and ids=={104,105} and any(
                    x.get("event")=="disable_complete" and x.get("confirmed") and not x.get("missing") for x in rows)
                if not confirmed:errors.append("双电机失能未确认")
            with self.lock:
                if errors:self.session.reason="; ".join(errors)
                self.session.completed(not errors,ack,self.backend=="simulation")
        except Exception as error:
            with self.lock:
                self.session.reason=str(error)
                self.session.completed(False,ack)
    def reset(self,req,res):
        with self.lock:
            try:
                if self.worker and self.worker.is_alive():raise ValueError("停止处理尚未完成")
                self.session.reset();self.state=None;self.controller_state=None;self.renew=False
                self.trajectory=None;self.q_ref=None;self.dq_ref=None
                res.success=True;res.message="已返回调试；重新开始需新鲜姿态和显式start"
            except ValueError as e:res.success=False;res.message=str(e)
        return res
    def snapshot(self):
        return dict(state=self.session.phase,reason=self.session.reason,id=self.session.session_id,
                    seq=self.session.sequence,q_ref=self.q_ref,dq_ref=self.dq_ref,ref=self.ref_cfg["ref"],source=self.source,
                    stop_ok=self.session.stop_confirmed,disable_ok=self.session.disable_confirmed,
                    backend=self.backend,sim=self.backend!="mit",
                    ctrl=self.controller,hw=self.hardware)
    def status(self,req,res):
        with self.lock:res.message=json.dumps(self.snapshot(),allow_nan=False)
        res.success=True;return res
    def tick(self):
        with self.lock:
            now=time.monotonic()
            if self.renew:
                m=self.controller_state
                fault=(not self.state or not self.state[2] or now-self.state[1]>=.1)
                if now-self.started_at>.5:
                    fault=fault or m is None or not 0<=now-self.last_status_at<.25 or m.mode==3
                if now-self.started_at>1.:
                    fault=fault or m is None or m.mode!=2 or m.session_id!=self.session.session_id
                fault=fault or self.node.count_publishers(self.plan_topic)!=1
                if fault:self.request_stop("状态失效/控制器故障")
                else:
                    plan=self.session.plan(now)
                    if plan:
                        try:
                            if self.source=="dynamic_ref_v1":
                                q,dq=(self.trajectory.sample(now-self.started_at,self.session.reference)
                                      if self.trajectory else (list(self.session.reference),[0.,0.]))
                                packet=ImpedanceRef(id=self.session.session_id,seq=self.session.sequence,
                                    t=now,on=True,q=q,dq=dq,k=list(self.session.gains[:2]),
                                    b=list(self.session.gains[2:]),tau=list(self.session.caps))
                            else:q,dq=list(self.session.reference),[0.,0.];packet=FixedPosePlan(**plan)
                            self.plan_pub.publish(packet);self.q_ref=q;self.dq_ref=dq
                        except (ValueError,TypeError) as error:self.request_stop("参考无效："+str(error))
            payload=self.snapshot()
        self.status_pub.publish(String(data=json.dumps(payload,allow_nan=False)))
    def close(self):
        if self.session.phase in ("preparing","starting","running"):self.request_stop("管理节点退出")
