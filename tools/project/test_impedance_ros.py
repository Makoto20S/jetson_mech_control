"""Isolated ROS/DDS -> MuJoCo effort bridge test, with no device access."""
import argparse
import csv
import gc
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
from uuid import uuid4
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'mujoco/ref01'))
sys.path.insert(0,str(ROOT/'mujoco/ref01/tools'))
from ref01_dynamics_api import REF01Dynamics
from verify_fixed_pose import REFERENCE, CONTROL_DT, PULSE_START, PULSE_END, PULSE_NM, assess


def stop(child):
    if child.poll() is None:
        child.send_signal(signal.SIGINT)
        try:
            child.wait(timeout=8)
        except subprocess.TimeoutExpired:
            child.terminate()
            child.wait(timeout=5)


def run_case(output, case, state_exe, duration=2.5, gc_policy="freeze"):
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import SingleThreadedExecutor
    import resource
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import JointState
    from trajectory_msgs.msg import JointTrajectory
    from mech_exo_state.msg import RobotState

    namespace = '/ph_exo_sim_'+uuid4().hex
    folder = output/case
    folder.mkdir()
    loss = case in ('loss','duplicate')
    axis = 1 if case == 'beta' else 0
    total = 1.85 if loss else duration
    gc_events = []
    def gc_event(phase, info):
        gc_events.append((time.monotonic_ns(), phase, info['generation']))
    class Plant(Node):
        def __init__(self):
            super().__init__('mujoco',namespace=namespace)
            self.model = REF01Dynamics()
            self.state = self.model.reset(REFERENCE)
            self.torque = np.zeros(2)
            self.received = None
            self.last_target_stamp = 0
            self.ticks = 0
            self.rows = []
            self.targets = []
            self.snapshots = []
            self.interrupted_ns = None
            self.expired_ns = None
            self.frozen = None
            self.closed = False
            self.pub = self.create_publisher(JointState,'joint_states',qos_profile_sensor_data)
            self.create_subscription(JointTrajectory,'pair_effort/target',self.target,1)
            self.create_subscription(RobotState,'state',self.snapshot,qos_profile_sensor_data)
            self.create_timer(CONTROL_DT,self.tick)
        def target(self,msg):
            stamp = msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
            age = self.get_clock().now().nanoseconds-stamp
            if (self.closed or list(msg.joint_names)!=['mit_comm_motor104','mit_comm_motor105']
                    or len(msg.points)!=1 or len(msg.points[0].effort)!=2
                    or stamp<=self.last_target_stamp or not 0<=age<60_000_000):
                return
            values = np.array(msg.points[0].effort)
            if not np.isfinite(values).all() or np.max(np.abs(values))>.2+1e-12:
                raise ValueError('Invalid simulation effort')
            self.received = time.monotonic_ns()
            self.last_target_stamp = stamp
            self.torque = values
            self.targets.append(dict(mono_ns=self.received,tau104_nm=float(values[0]),tau105_nm=float(values[1])))
        def snapshot(self,msg):
            self.snapshots.append(dict(mono_ns=time.monotonic_ns(),valid=bool(msg.q_velocity_valid),
                                       q=list(msg.q_position),dq=list(msg.q_velocity)))
        def feedback(self, elapsed):
            msg = JointState(name=['motor104_joint','motor105_joint'])
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.position = (self.state['alpha'][::-1]+np.pi/4).tolist()
            msg.velocity = self.state['alpha_velocity'][::-1].tolist()
            if loss and 1.2<=elapsed<1.65:
                if self.interrupted_ns is None:
                    self.interrupted_ns = time.monotonic_ns()
                    self.frozen = msg
                if case=='duplicate':
                    self.pub.publish(self.frozen)
            else:
                self.pub.publish(msg)
        def tick(self):
            entry = time.monotonic_ns()
            cpu_start = time.thread_time_ns()
            elapsed = self.ticks*CONTROL_DT
            self.feedback(elapsed)
            if self.received is None or elapsed>=total:
                return
            start = time.monotonic_ns()
            if start-self.received>60_000_000:
                self.torque[:] = 0.
                self.closed = True
                if self.expired_ns is None:
                    self.expired_ns = start
            q,dq = self.state['q'],self.state['dq']
            external = np.zeros(2)
            if PULSE_START<=elapsed<PULSE_END:
                external[axis] = PULSE_NM
            self.state = self.model.step(self.torque[::-1],steps=10,external_torque=external)
            step_end = time.monotonic_ns()
            self.rows.append(dict(case=case,time_s=elapsed,mono_ns=start,
                callback_entry_ns=entry,feedback_ms=(start-entry)/1e6,
                step_ms=(step_end-start)/1e6,cpu_ms=(time.thread_time_ns()-cpu_start)/1e6,psi_rad=q[0],beta_rad=q[1],
                dpsi_rad_s=dq[0],dbeta_rad_s=dq[1],tau104_nm=self.torque[0],tau105_nm=self.torque[1],
                scale=1.,closure_end_m=self.state['max_closure_m'],  # Saturation is verified from controller.json.
                applied104_end_nm=self.state['tau_applied'][1],applied105_end_nm=self.state['tau_applied'][0]))
            self.ticks+=1

    node = Plant()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    children,logs = [],[]
    gc_was_enabled = gc.isenabled()
    gc.callbacks.append(gc_event)
    try:
        commands = [
            [str(state_exe),'--ros-args','-r','__ns:='+namespace,
             '-r','ph_exo/state:=state','-p','joint_topic:='+namespace+'/joint_states',
             '-p','mapping_confirmed:=true','-p','publish_rate_hz:=200.0',
             '-p','publish_on_joint_message:=true','-p','timeout_ms:=60.0',
             '-p','freeze_gc_before_spin:='+str(gc_policy=='freeze').lower(),
             '-p','timing_gc_path:='+str(folder/'state_gc.json')],
            [sys.executable,'-B',str(ROOT/'tools/project/impedance_ros.py'),
             '--namespace',namespace,'--output',str(folder/'controller.json')]
            + (['--freeze-gc'] if gc_policy=='freeze' else [])]
        for i,args in enumerate(commands):
            log=(folder/f'node_{i}.log').open('w')
            logs.append(log)
            children.append(subprocess.Popen(args,stdout=log,stderr=log))
        if gc_policy != 'automatic':
            gc.collect()
            if gc_policy == 'pause':
                gc.disable()
            else:
                gc.freeze()
        rss_start = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss
        deadline=time.monotonic()+max(20.,total*2+10.)
        while node.ticks<round(total/CONTROL_DT) and time.monotonic()<deadline:
            if any(p.poll() is not None for p in children):
                raise RuntimeError('ROS child exited; inspect node logs')
            executor.spin_once(timeout_sec=.005)
        if node.ticks<round(total/CONTROL_DT):
            raise RuntimeError('Software graph did not finish')
    finally:
        rss_end = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss
        executor.remove_node(node)
        executor.shutdown()
        gc.callbacks.remove(gc_event)
        if gc_policy == 'freeze':
            gc.unfreeze()
        if gc_was_enabled:
            gc.enable()
        for child in reversed(children):
            stop(child)
        for log in logs:
            log.close()
        node.destroy_node()

    with (folder/'traces.csv').open('x',newline='') as stream:
        writer=csv.DictWriter(stream,fieldnames=list(node.rows[0]))
        writer.writeheader()
        writer.writerows(node.rows)
    (folder/'gc.json').write_text(json.dumps(gc_events))
    (folder/'targets.json').write_text(json.dumps(node.targets))
    (folder/'states.json').write_text(json.dumps(node.snapshots))
    controller=json.loads((folder/'controller.json').read_text())
    if loss:
        last=max(row['mono_ns'] for row in node.targets)
        cutoff=node.interrupted_ns
        final=all(np.max(np.abs([r['tau104_nm'],r['tau105_nm']]))==0 for r in node.rows[-20:])
        checks=dict(controller_tripped=controller['tripped'],receiver_expired=node.closed,
                    stopped_within_80ms=last-cutoff<80_000_000,
                    no_restart_after_feedback_returns=last<node.rows[-40]['mono_ns'],
                    zero_effort_after_expiry=bool(final))
        result=dict(passed=all(checks.values()),checks=checks,reason=controller['reason'],
                    last_target_after_cut_ms=(last-cutoff)/1e6,
                    zero_applied_after_cut_ms=(node.expired_ns-cutoff)/1e6 if node.expired_ns else None)
    else:
        result=assess(node.rows,node.state,axis)
        result['checks']['unsaturated']=all(r['scale']==1. for r in controller['commands'])
        result['passed']=all(result['checks'].values())
        result['controller_tripped']=controller['tripped']
        result['passed']=result['passed'] and not controller['tripped']
    intervals=np.diff([r['mono_ns'] for r in node.rows])/1e6
    commands=controller['commands']
    result['functional_passed'] = result['passed']
    result['timing_passed'] = None
    result['acceptance_scope'] = 'functional_only; timing statistics are diagnostic'
    result['timing']=dict(plant_interval_p99_ms=float(np.percentile(intervals,99)),
                          plant_interval_max_ms=float(max(intervals)),
                          plant_intervals_over_6ms=int(np.sum(intervals>6.)),
                          state_age_p99_ms=float(np.percentile([r['state_age_ms'] for r in commands],99)),
                          compute_p99_ms=float(np.percentile([r['compute_ms'] for r in commands],99)))
    result['timing'].update(plant_interval_p999_ms=float(np.percentile(intervals,99.9)),
                            plant_intervals_over_5_5ms=int(np.sum(intervals>5.5)),
                            plant_intervals_over_10ms=int(np.sum(intervals>10.)),
                            plant_interval_min_ms=float(min(intervals)),
                            callback_wall_max_ms=max(r['feedback_ms']+r['step_ms'] for r in node.rows),
                            callback_cpu_max_ms=max(r['cpu_ms'] for r in node.rows))
    command_intervals = np.diff([r['mono_ns'] for r in commands])/1e6
    result['timing'].update(command_interval_p99_ms=float(np.percentile(command_intervals,99)),
                            command_interval_max_ms=float(max(command_intervals)))
    result['gc_policy']=gc_policy
    result['max_rss_kib']={'start':rss_start,'end':rss_end}
    result['samples']=len(node.rows)
    result['commands']=len(commands)
    (folder/'summary.json').write_text(json.dumps(result,indent=2))
    print(case,json.dumps(result),flush=True)
    return result


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--duration',type=float,default=2.5)
    parser.add_argument('--cases',nargs='+',choices=['psi','beta','loss','duplicate'],
                        default=['psi','beta','loss','duplicate'])
    parser.add_argument('--managed-gc',action='store_true',help='Diagnostic alias for --gc-policy pause')
    parser.add_argument('--gc-policy',choices=['automatic','freeze','pause'],default='freeze')
    args=parser.parse_args()
    if not np.isfinite(args.duration) or not 2.5<=args.duration<=120:
        parser.error('duration must be within [2.5,120] seconds')
    args.output.mkdir(parents=True,exist_ok=False)
    os.environ.update(ROS_DOMAIN_ID='179',ROS_LOCALHOST_ONLY='1')
    import rclpy
    from ament_index_python.packages import get_package_prefix
    prefix=Path(get_package_prefix('mech_exo_state')).resolve()
    expected=(ROOT/'.local/ros/servo_imu/install/mech_exo_state').resolve()
    if prefix!=expected:
        raise RuntimeError('Expected the development servo_imu overlay')
    rclpy.init()
    try:
        results={case:run_case(args.output,case,prefix/'lib/mech_exo_state/state_node.py',
                               args.duration,'pause' if args.managed_gc else args.gc_policy) for case in args.cases}
    finally:
        rclpy.shutdown()
    summary=dict(passed=all(r['passed'] for r in results.values()),cases=results,
                 endpoint='MuJoCo trajectory effort bridge',model_sha256=REF01Dynamics().metadata['model_sha256'])
    (args.output/'summary.json').write_text(json.dumps(summary,indent=2))
    return 0 if summary['passed'] else 1


if __name__=='__main__':
    raise SystemExit(main())
