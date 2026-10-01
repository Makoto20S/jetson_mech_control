#!/usr/bin/env python3
"""Paired position operator: standard ROS services/actions, never device I/O."""
import argparse
from collections import deque
import importlib.util
import json
import math
import os
from pathlib import Path
import selectors
import signal
import subprocess
import sys
import termios
import time
import uuid

_spec = importlib.util.spec_from_file_location('servo_config', Path(__file__).with_name('operator.py'))
config_tools = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(config_tools)
CONTROLLER = 'servo_trajectory_controller'
FEEDBACK_TIMEOUT = .25


def load_config(path, require_calibrated=True):
    config = json.loads(Path(path).read_text())
    config_tools.validate_config(config, observe=not require_calibrated)
    if require_calibrated and config.get('calibrated') is not True:
        raise ValueError('请先完成范围标定并保存')
    return config


class MotionPolicy:
    def __init__(self, config):
        config_tools.validate_config(config)
        if config.get('calibrated') is not True:
            raise ValueError('请先完成范围标定并保存')
        self.motors = sorted(config['motors'], key=lambda m: m['id'])
        self.names = [m['joint_name'] for m in self.motors]

    def check_positions(self, positions):
        if len(positions) != 2:
            raise ValueError('必须有两个位置')
        for m, value in zip(self.motors, positions):
            if not config_tools.finite(value) or not m['position_min_rad'] <= value <= m['position_max_rad']:
                raise ValueError(f"电机{m['id']}位置超出已标定范围")

    def move(self, words):
        if len(words) != 3:
            raise ValueError('格式：move 104目标角度 105目标角度 移动秒数')
        a, b, seconds = map(float, words)
        if not all(math.isfinite(v) for v in (a, b, seconds)) or not .5 <= seconds <= 60.:
            raise ValueError('请输入有限角度，移动时间须为0.5至60秒')
        positions = [(angle-m['target_offset'])/m['target_scale']
                     for m, angle in zip(self.motors, (a,b))]
        self.check_positions(positions)
        return positions, seconds

    def relative_move(self, words, current):
        if len(words) != 3:
            raise ValueError('格式：step 104增量角度 105增量角度 移动秒数')
        self.check_positions(current)
        delta104,delta105,seconds = map(float,words)
        angles = self.device_angles(current)
        return self.move([angles[0]+delta104,angles[1]+delta105,seconds])

    def device_angles(self, positions):
        return [(p-m['feedback_offset'])/m['feedback_scale']
                for m,p in zip(self.motors,positions)]


class Feedback:
    def __init__(self, names):
        self.names = names
        self.positions = None
        self.received = -math.inf
        self.stamp = -math.inf
        self.history = deque()
        self.velocity = None

    def ingest(self, names, positions, stamp, now):
        if (len(names) != len(set(names)) or len(positions) != len(names)
                or not all(n in names for n in self.names)
                or not math.isfinite(stamp) or stamp <= self.stamp):
            self.received = -math.inf
            return False
        selected = [positions[names.index(n)] for n in self.names]
        if not all(math.isfinite(p) for p in selected):
            self.received = -math.inf
            return False
        if now-self.received > FEEDBACK_TIMEOUT:
            self.history.clear()
            self.velocity = None
        self.positions, self.stamp, self.received = selected, stamp, now
        self.history.append((stamp,selected))
        while len(self.history)>2 and stamp-self.history[1][0] >= .2:
            self.history.popleft()
        elapsed = stamp-self.history[0][0]
        if elapsed >= .1:
            self.velocity = [(p-q)/elapsed for p,q in zip(selected,self.history[0][1])]
        return True

    def fresh(self, now):
        return self.positions is not None and 0 <= now-self.received <= FEEDBACK_TIMEOUT


class CommandInput:
    """Keep typed text visible while the status area refreshes; retain Ctrl-C."""
    def __init__(self, stream):
        self.fd = stream.fileno()
        self.saved = termios.tcgetattr(self.fd)
        settings = termios.tcgetattr(self.fd)
        settings[3] &= ~(termios.ICANON | termios.ECHO)
        settings[6][termios.VMIN] = 1
        settings[6][termios.VTIME] = 0
        termios.tcsetattr(self.fd, termios.TCSANOW, settings)
        self.buffer = ''
        self.escape = False

    def read(self):
        data = os.read(self.fd, 1)
        if not data or data == b'\x04':
            return 'quit'
        char = data.decode('ascii', errors='ignore')
        if char == '\x1b':
            self.escape = True
        elif self.escape:
            if char.isalpha() or char == '~':
                self.escape = False
        elif char in ('\r', '\n'):
            line, self.buffer = self.buffer, ''
            return line
        elif char in ('\x7f', '\b'):
            self.buffer = self.buffer[:-1]
        elif char and char.isprintable() and len(self.buffer) < 160:
            self.buffer += char
        return None

    def close(self):
        termios.tcsetattr(self.fd, termios.TCSANOW, self.saved)


def hardware_health(response, names, require_claimed):
    expected={name+'/position' for name in names}
    components=[]
    seen=set()
    healthy=True
    for component in response.component:
        selected=[i for i in component.command_interfaces if i.name in expected]
        if not selected:
            continue
        for interface in selected:
            if interface.name in seen:
                healthy=False
            seen.add(interface.name)
            healthy=healthy and interface.is_available and (interface.is_claimed or not require_claimed)
        healthy=healthy and component.state.id==3  # lifecycle ACTIVE
        components.append(dict(name=component.name,state_id=component.state.id,state=component.state.label,
            interfaces=[dict(name=i.name,available=i.is_available,claimed=i.is_claimed) for i in selected]))
    return dict(healthy=bool(healthy and seen==expected),components=components,
                missing=sorted(expected-seen),require_claimed=require_claimed)


class RosControl:
    """One-thread ROS client; server owns hardware and enforces hardware limits."""
    def __init__(self, policy, namespace, record):
        import rclpy
        from rclpy.action import ActionClient
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        from control_msgs.action import FollowJointTrajectory
        from controller_manager_msgs.srv import ListControllers, SwitchController, ListHardwareComponents
        from sensor_msgs.msg import JointState
        self.ros = rclpy
        self.node = Node('servo_move_operator', namespace=namespace)
        self.policy, self.record = policy, record
        self.feedback = Feedback(policy.names)
        self.node.create_subscription(JointState, 'joint_states', self.on_feedback, qos_profile_sensor_data)
        self.list_client = self.node.create_client(ListControllers, 'controller_manager/list_controllers')
        self.hardware_client = self.node.create_client(ListHardwareComponents, 'controller_manager/list_hardware_components')
        self.hardware_error = None
        self.hardware_status = None
        self.hardware_future = None
        self.hardware_deadline = 0.
        self.next_hardware_check = 0.
        self.monitor_hardware = False
        self.switch_client = self.node.create_client(SwitchController, 'controller_manager/switch_controller')
        self.action = ActionClient(self.node, FollowJointTrajectory, CONTROLLER+'/follow_joint_trajectory')
        self.active = False
        self.uncertain = False
        self.goal = self.result = None
        self.deadline = None
        self.targets = None
        self.stopping = False
        self.message = '未使能'

    def on_feedback(self, msg):
        if self.hardware_error:
            return  # Later broadcaster messages cannot revive a failed hardware session.
        self.feedback.ingest(list(msg.name),list(msg.position),
                             msg.header.stamp.sec+msg.header.stamp.nanosec/1e9,time.monotonic())

    def spin(self, seconds=0.):
        self.ros.spin_once(self.node, timeout_sec=seconds)

    def await_future(self, future, timeout=3.):
        deadline=time.monotonic()+timeout
        while not future.done() and self.ros.ok() and time.monotonic()<deadline:
            self.spin(.02)
        if not future.done():
            raise RuntimeError('ROS请求超时，结果不确定')
        return future.result()

    def state(self):
        from controller_manager_msgs.srv import ListControllers
        if not self.list_client.wait_for_service(timeout_sec=.5):
            raise RuntimeError('控制器管理服务不可用')
        response=self.await_future(self.list_client.call_async(ListControllers.Request()))
        states={c.name:c.state for c in response.controller}
        return states.get(CONTROLLER,'未加载')

    def switch(self, enabled):
        from controller_manager_msgs.srv import SwitchController
        from builtin_interfaces.msg import Duration
        if not self.switch_client.wait_for_service(timeout_sec=.5):
            raise RuntimeError('控制器切换服务不可用')
        request=SwitchController.Request()
        request.activate_controllers=[CONTROLLER] if enabled else []
        request.deactivate_controllers=[] if enabled else [CONTROLLER]
        request.strictness=SwitchController.Request.STRICT
        request.activate_asap=True
        request.timeout=Duration(sec=2)
        # An unanswered request may complete later; never retry movement in that session.
        self.uncertain=True
        self.record('switch_request',enabled=enabled,positions_rad=list(self.feedback.positions))
        response=self.await_future(self.switch_client.call_async(request))
        actual=self.state()
        if not response.ok or actual != ('active' if enabled else 'inactive'):
            raise RuntimeError(f'切换未确认，控制器状态：{actual}')
        self.active=enabled
        self.uncertain=False
        self.record('controller',state=actual)

    def hardware_failed(self, reason):
        self.hardware_error = self.hardware_error or reason
        self.feedback.received = -math.inf
        raise RuntimeError(self.hardware_error)

    def accept_hardware(self, response):
        status=hardware_health(response,self.policy.names,self.active)
        if status != self.hardware_status:
            self.record('hardware',**status)
        self.hardware_status=status
        if not status['healthy']:
            self.hardware_failed('硬件状态或位置命令接口不可用；停止接受目标，请查看framework.log首次故障记录')

    def check_hardware(self):
        from controller_manager_msgs.srv import ListHardwareComponents
        if self.hardware_error:
            raise RuntimeError(self.hardware_error)
        try:
            if not self.hardware_client.wait_for_service(timeout_sec=.5):
                raise RuntimeError('硬件状态服务不可用')
            # Discard responses issued before a claim/lifecycle transition.
            if self.hardware_future is not None:
                self.hardware_future.cancel()
                self.hardware_future=None
            # A new request is required before each enable/goal, not a cached healthy flag.
            response=self.await_future(self.hardware_client.call_async(ListHardwareComponents.Request()),1.)
            self.accept_hardware(response)
            self.monitor_hardware=True
        except Exception as exc:
            self.hardware_failed('硬件检查失败：'+str(exc))

    def poll_hardware(self):
        from controller_manager_msgs.srv import ListHardwareComponents
        if not self.monitor_hardware:
            return
        now=time.monotonic()
        if self.hardware_future is not None:
            if self.hardware_future.done():
                future=self.hardware_future
                self.hardware_future=None
                try:
                    self.accept_hardware(future.result())
                except Exception as exc:
                    self.hardware_failed('硬件检查失败：'+str(exc))
            elif now>=self.hardware_deadline:
                self.hardware_failed('硬件状态响应超时')
        if self.hardware_future is None and now>=self.next_hardware_check:
            if not self.hardware_client.service_is_ready():
                self.hardware_failed('硬件状态服务中断')
            self.hardware_future=self.hardware_client.call_async(ListHardwareComponents.Request())
            self.hardware_deadline=now+1.
            self.next_hardware_check=now+.2

    def ready_feedback(self):
        if getattr(self,'hardware_error',None):
            raise RuntimeError(self.hardware_error)
        if not self.feedback.fresh(time.monotonic()):
            raise ValueError('等待有效的双电机ROS位置反馈')
        self.policy.check_positions(self.feedback.positions)

    def enable(self):
        if self.uncertain:
            raise RuntimeError('上次请求结果不确定，请退出并检查日志')
        self.ready_feedback()
        self.check_hardware()
        if self.state() != 'inactive':
            raise ValueError('使能前控制器必须为inactive')
        self.switch(True)
        self.check_hardware()
        self.message='已使能，保持当前位置；可输入move'

    def move_relative(self, words):
        if self.goal is not None:
            raise ValueError('当前运动尚未完成，请等待或stop')
        self.ready_feedback()
        base=list(self.feedback.positions)
        target,seconds=self.policy.relative_move(words,base)
        self.send(target,seconds)
        self.record('relative_goal',base_positions_rad=base,
                    delta_device_deg=[float(words[0]),float(words[1])],
                    positions_rad=target,duration_s=seconds)

    def send(self, positions, seconds, settling=3.):
        from builtin_interfaces.msg import Duration
        from control_msgs.action import FollowJointTrajectory
        from control_msgs.msg import JointTolerance
        from trajectory_msgs.msg import JointTrajectoryPoint
        if not self.active or self.uncertain:
            raise ValueError('请先enable；不确定状态下拒绝运动')
        self.check_hardware()
        self.ready_feedback()
        self.policy.check_positions(positions)
        if not self.action.wait_for_server(timeout_sec=.5):
            raise RuntimeError('轨迹action不可用')
        goal=FollowJointTrajectory.Goal()
        goal.trajectory.joint_names=self.policy.names
        for values,t in ((self.feedback.positions,0.),(positions,seconds)):
            point=JointTrajectoryPoint()
            point.positions=list(values)
            point.velocities=[0.,0.]
            ns=round(t*1e9)
            point.time_from_start=Duration(sec=ns//10**9,nanosec=ns%10**9)
            goal.trajectory.points.append(point)
        # Device-degree tolerance mapped into joint radians; do not accept time elapsed alone.
        goal.goal_tolerance=[JointTolerance(name=m['joint_name'],position=.5/abs(m['target_scale']))
                             for m in self.policy.motors]
        goal.goal_time_tolerance=Duration(sec=int(settling))
        self.uncertain=True
        handle=self.await_future(self.action.send_goal_async(goal))
        if not handle.accepted:
            self.uncertain=False
            raise ValueError('轨迹被控制器拒绝')
        self.goal=handle
        self.result=handle.get_result_async()
        self.targets=list(positions)
        self.deadline=time.monotonic()+seconds+settling+2.
        self.uncertain=False
        self.record('goal',positions_rad=positions,duration_s=seconds)
        self.message='运动中'
        self.stopping=False

    def cancel(self):
        if self.goal is not None and self.result is not None and not self.result.done():
            response=self.await_future(self.goal.cancel_goal_async())
            if not response.goals_canceling:
                # Completion can race cancellation; require an actual terminal result.
                self.await_future(self.result,1.)
        self.goal=self.result=None
        self.deadline=None

    def stop(self):
        if self.uncertain:
            raise RuntimeError('请求状态不确定，停止将转为停用控制器')
        if not self.active:
            return
        self.cancel()
        self.ready_feedback()
        # Cancel alone makes JTC hold its last desired point, not the measured position.
        self.send(list(self.feedback.positions),.2,settling=1.)
        self.message='已请求保持当前测量位置；请观察机构是否停稳'
        self.stopping=True
        self.record('stop_requested')

    def disable(self):
        if getattr(self,'hardware_error',None):
            raise RuntimeError('硬件已失效，不再发送保持目标；将关闭框架进程：'+self.hardware_error)
        if not self.active and not self.uncertain:
            self.message='本工具未使能控制器，关闭框架进程'
            return
        if self.active and not self.uncertain and self.feedback.fresh(time.monotonic()):
            try:
                self.stop()
                self.await_future(self.result,2.)
            except Exception as exc:
                self.record('hold_unconfirmed',error=str(exc))
        if self.hardware_error:
            raise RuntimeError('硬件失效，跳过控制器切换，关闭框架进程：'+self.hardware_error)
        # Always check the server, including after an ambiguous activation/goal request.
        if self.state() == 'active':
            self.switch(False)
        elif self.uncertain:
            raise RuntimeError('存在未确认请求；需要检查控制器日志')
        self.active=False
        self.goal=self.result=None
        self.deadline=None
        self.message='控制器已停用；不等同于驱动断能或机械急停'

    def poll(self):
        self.spin(.01)
        self.poll_hardware()
        if self.active and not self.feedback.fresh(time.monotonic()):
            raise RuntimeError('ROS位置反馈中断')
        if self.result is not None and self.result.done():
            result=self.result.result()
            self.record('result',status=result.status,error_code=result.result.error_code,
                        error_string=result.result.error_string)
            self.goal=self.result=None
            self.deadline=None
            if result.status != 4 or result.result.error_code != 0:
                raise RuntimeError('轨迹未成功：'+result.result.error_string)
            self.message=('保持当前测量位置请求完成，请观察机构状态' if self.stopping
                          else '轨迹到达容差内，保持目标位置')
        if self.deadline is not None and time.monotonic()>self.deadline:
            raise RuntimeError('轨迹结果超时')

    def status(self):
        fresh=self.feedback.fresh(time.monotonic()) and not self.hardware_error
        lines=[self.message+(' | ROS消息更新' if fresh else ' | ROS消息过期/硬件异常/等待中')]
        positions=self.feedback.positions
        for i,m in enumerate(self.policy.motors):
            actual='未知' if positions is None else f'{self.policy.device_angles(positions)[i]:.2f}'
            target='—' if self.targets is None else f"{self.targets[i]*m['target_scale']+m['target_offset']:.2f}"
            speed='未知' if not fresh or self.feedback.velocity is None else f"{self.feedback.velocity[i]/m['feedback_scale']:.2f}"
            lines.append(f"{m['id']} 目标 {target}° | 实际 {actual}° | 估算速度 {speed}°/s")
        lines.append('电流/温度/原始故障码：当前控制链路未发布')
        return '\n'.join(lines)


def terminate_launch(proc):
    if proc is None:
        return
    # Signal the group even when its launch leader has already died.
    for sig,timeout in ((signal.SIGINT,7.),(signal.SIGTERM,3.),(signal.SIGKILL,2.)):
        try:
            os.killpg(proc.pid,sig)
        except ProcessLookupError:
            proc.wait(timeout=1.)
            return
        deadline=time.monotonic()+timeout
        while time.monotonic()<deadline:
            proc.poll()  # Reap the leader, so it cannot keep a dead group visible.
            try:
                os.killpg(proc.pid,0)
            except ProcessLookupError:
                return
            time.sleep(.05)
    raise RuntimeError('控制进程组未能确认退出')


def cleanup(client, proc, keyboard, display):
    errors=[]
    # Terminal/log failures must never bypass controller and process cleanup.
    operations=[]
    if client is not None:
        operations.append(('停用未确认',client.disable))
    operations.append(('框架退出未确认',lambda:terminate_launch(proc)))
    if keyboard is not None:
        operations.append(('终端恢复失败',keyboard.close))
    operations.append(('终端显示关闭失败',display.clear))
    for label,operation in operations:
        try:
            operation()
        except Exception as exc:
            errors.append(label+': '+str(exc))
    return errors


def report_safely(callback, *args, **kwargs):
    try:
        callback(*args,**kwargs)
    except Exception:
        pass


def main():
    parser=argparse.ArgumentParser(description='双电机位置试验，使用现有ROS控制框架')
    parser.add_argument('--config',required=True)
    parser.add_argument('--run-dir',required=True)
    parser.add_argument('--check',action='store_true',help='只校验配置，不启动ROS或设备')
    args=parser.parse_args()
    config=load_config(args.config)
    policy=MotionPolicy(config)
    description=config_tools.generate_urdf(config)
    if args.check:
        print('配置有效；双电机目标将受已标定限位约束，未打开设备。')
        return 0
    if not sys.stdin.isatty():
        raise ValueError('运动入口需要交互终端，不接受管道批量运动指令')
    run=Path(args.run_dir)/('move-'+time.strftime('%Y%m%d-%H%M%S')+'-'+uuid.uuid4().hex[:8])
    run.mkdir(parents=True)
    (run/'config.json').write_text(json.dumps(config,ensure_ascii=False,indent=2)+'\n')
    urdf=run/'control.urdf'; urdf.write_text(description)
    ready_file=run/'controllers.ready'
    import rclpy
    from rclpy.signals import SignalHandlerOptions
    rclpy.init(args=[],signal_handler_options=SignalHandlerOptions.NO)
    interrupted=False
    def interrupt(signum, frame):
        nonlocal interrupted
        interrupted=True
    old={sig:signal.signal(sig,interrupt) for sig in (signal.SIGINT,signal.SIGTERM,signal.SIGHUP)}
    proc=None; client=None; keyboard=None; result=0
    display=config_tools.TerminalDisplay(sys.stdout)
    with (run/'events.jsonl').open('w',buffering=1) as events, (run/'framework.log').open('w') as output:
        def record(event,**data):
            events.write(json.dumps(dict(event=event,monotonic_s=time.monotonic(),**data),ensure_ascii=False,allow_nan=False)+'\n')
        try:
            namespace='servo_move_'+uuid.uuid4().hex[:8]
            proc=subprocess.Popen(['ros2','launch',str(Path(__file__).with_name('servo_pair.launch.py')),
                                   'urdf:='+str(urdf),'namespace:='+namespace,'ready_file:='+str(ready_file)],
                                  stdout=output,stderr=subprocess.STDOUT,start_new_session=True,
                                  env={**os.environ,'MECH_SERVO_TRACE_PATH':str(run/'serial-trace.jsonl'),
                                       'MECH_SERVO_CHAIN_PATH':str(run/'command-chain.jsonl')})
            client=RosControl(policy,namespace,record)
            display.message('输入设备绝对角度（与servo-status一致）；不会设置零点。\n'
                            'enable：使能并保持当前位置\nmove 104角度 105角度 秒数：绝对移动\n'
                            'step 104增量 105增量 秒数：从当前实际角度增量移动\n'
                            'stop：请求保持当前测量位置；disable：停用；quit：退出\n'
                            '软件停止不等同于机械急停。日志：'+str(run))
            for m in policy.motors:
                limits=sorted(p*m['target_scale']+m['target_offset'] for p in (m['position_min_rad'],m['position_max_rad']))
                display.message(f"{m['id']} 允许目标角度：{limits[0]:.2f}° 至 {limits[1]:.2f}°")
            selector=selectors.DefaultSelector(); selector.register(sys.stdin,selectors.EVENT_READ)
            keyboard=CommandInput(sys.stdin)
            next_display=0.; next_state=0.
            startup_deadline=time.monotonic()+20.
            ready=False
            while not interrupted:
                if proc.poll() is not None:
                    raise RuntimeError('控制框架已退出，请查看framework.log')
                client.poll()
                now=time.monotonic()
                if not ready:
                    ready=ready_file.exists() and client.feedback.fresh(now)
                    if ready:
                        client.check_hardware()
                        client.message='控制器已就绪；输入enable使能'
                    if not ready and now>startup_deadline:
                        raise RuntimeError('启动20秒后双电机反馈或控制器仍未就绪，请检查framework.log')
                if now>=next_display:
                    display.update(client.status(),'命令> '+keyboard.buffer)
                    if client.feedback.fresh(now):
                        record('feedback',positions_rad=client.feedback.positions,
                               estimated_velocity_rad_s=client.feedback.velocity)
                    next_display=now+.2
                if client.active and now>=next_state:
                    if client.state() != 'active':
                        raise RuntimeError('轨迹控制器不再active')
                    next_state=now+1.
                if selector.select(timeout=0):
                    line=keyboard.read()
                    next_display=0.
                    if line is None:
                        continue
                    words=line.split()
                    if not words:
                        continue
                    record('operator_input',words=words)
                    command=words[0].lower()
                    try:
                        if command=='quit' and len(words)==1:
                            break
                        elif command=='enable' and len(words)==1:
                            if not ready:
                                raise ValueError('控制器尚未就绪，请等待启动完成')
                            client.enable()
                        elif command=='move':
                            if client.goal is not None:
                                raise ValueError('当前运动尚未完成，请等待或stop')
                            target,seconds=policy.move(words[1:])
                            client.send(target,seconds)
                        elif command=='step':
                            client.move_relative(words[1:])
                        elif command=='stop' and len(words)==1:
                            client.stop()
                        elif command=='disable' and len(words)==1:
                            client.disable()
                        else:
                            raise ValueError('命令：enable / move 角度104 角度105 秒数 / step 增量104 增量105 秒数 / stop / disable / quit')
                    except ValueError as exc:
                        display.message('拒绝：'+str(exc))
            selector.close()
        except Exception as exc:
            result=1
            report_safely(display.message,'结束本次操作：'+str(exc))
            report_safely(record,'error',error=str(exc))
        finally:
            errors=cleanup(client,proc,keyboard,display)
            if errors:
                result=1
                report_safely(print,'；'.join(errors)+'；请现场确认机构状态。',file=sys.stderr)
                report_safely(record,'cleanup_unconfirmed',errors=errors)
            elif client is not None:
                report_safely(print,client.message)
            if client is not None:
                client.node.destroy_node()
            rclpy.shutdown()
            for sig,handler in old.items():
                signal.signal(sig,handler)
    return result


if __name__=='__main__':
    try:
        raise SystemExit(main())
    except (ValueError,OSError) as exc:
        print(str(exc),file=sys.stderr)
        raise SystemExit(2)
