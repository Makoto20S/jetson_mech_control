"""Bounded ROS observer fixture. No serial, CAN or motor commands."""
import argparse
import csv
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time
import uuid


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--output',required=True,type=Path)
    parser.add_argument('--duration',type=float,default=22.)
    parser.add_argument('--load-only',action='store_true')
    args=parser.parse_args()
    if os.environ.get('ROS_LOCALHOST_ONLY')!='1': raise RuntimeError('require localhost-only ROS domain')
    args.output.mkdir(parents=True,exist_ok=False)
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile,ReliabilityPolicy
    from sensor_msgs.msg import Imu
    from mech_exo_state.msg import GaitPhase
    ns='/ao_fixture_'+uuid.uuid4().hex[:10]
    qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT)
    env={**os.environ,'ROS_LOG_DIR':str(args.output/'ros_logs'),'PYTHONDONTWRITEBYTECODE':'1'}
    log=(args.output/'observer.log').open('w')
    child=subprocess.Popen(['ros2','run','mech_exo_state','ao_node.py','--ros-args','-p','input_topic:='+ns+'/imu','-p','output_topic:='+ns+'/phase'],env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    rclpy.init(); node=Node('ao_fixture'); pub=node.create_publisher(Imu,ns+'/imu',qos)
    rows=[]; start=None
    def receive(msg):
        if start is None:return
        rows.append(dict(elapsed=time.monotonic()-start,state=msg.state,reason=msg.reason,valid=msg.valid,phase_reference_verified=msg.phase_reference_verified,phase=msg.phase_rad,frequency=msg.frequency_hz,age_ms=msg.source_age_ms,receive_age_ms=msg.receive_age_ms,compute_us=msg.compute_us,count=msg.accepted_samples))
    node.create_subscription(GaitPhase,ns+'/phase',receive,qos)
    error=None; forced=False
    try:
        deadline=time.monotonic()+10
        while pub.get_subscription_count()==0 and time.monotonic()<deadline:
            if child.poll() is not None:raise RuntimeError('observer exited during startup')
            rclpy.spin_once(node,timeout_sec=.02)
        if pub.get_subscription_count()==0:raise RuntimeError('no observer subscription')
        start=time.monotonic(); next_sample=start; saved=None
        while time.monotonic()-start<args.duration:
            now=time.monotonic(); elapsed=now-start
            if child.poll() is not None:raise RuntimeError('observer exited')
            if now>=next_sample:
                next_sample=now+.01
                if args.load_only or elapsed<18:
                    msg=Imu(); msg.header.stamp=node.get_clock().now().to_msg()
                    msg.angular_velocity.x=math.sin(2*math.pi*elapsed)
                    if not args.load_only and 7<=elapsed<9:msg.angular_velocity.x=0.
                    pub.publish(msg); saved=msg
                elif elapsed<19 and saved is not None:
                    pub.publish(saved) # duplicate source time must not refresh freshness
                elif elapsed<20:
                    pass # true dropout
                else:
                    msg=Imu(); msg.header.stamp=node.get_clock().now().to_msg()
                    msg.angular_velocity.x=float('nan');pub.publish(msg)
            rclpy.spin_once(node,timeout_sec=.001)
    except Exception as exc:error=repr(exc)
    finally:
        os.killpg(child.pid,signal.SIGINT) if child.poll() is None else None
        try:child.wait(timeout=8)
        except subprocess.TimeoutExpired:
            forced=True;os.killpg(child.pid,signal.SIGTERM);child.wait(timeout=5)
        log.close();node.destroy_node();rclpy.shutdown()
    if rows:
        with (args.output/'trace.csv').open('w',newline='') as f:
            writer=csv.DictWriter(f,fieldnames=list(rows[0]));writer.writeheader();writer.writerows(rows)
    def any_valid(lo,hi):return any(r['valid'] for r in rows if lo<=r['elapsed']<hi)
    def invalid_all(lo,hi):
        subset=[r for r in rows if lo<=r['elapsed']<hi]
        return bool(subset) and all(not r['valid'] for r in subset)
    checks=dict(no_error=error is None,normal_exit=child.returncode==0 and not forced,received=len(rows)>100,finite=all(math.isfinite(r[k]) for r in rows for k in ['phase','frequency','age_ms','compute_us']),no_anatomical_phase_claim=all(not r['phase_reference_verified'] for r in rows))
    if not args.load_only:
        checks.update(initial_tracking=any_valid(4,7),stop_revokes=invalid_all(8,9),restart_tracks=any_valid(14,18),duplicate_revokes=invalid_all(18.2,19),dropout_revokes=invalid_all(19.2,20),nan_revokes=invalid_all(20.2,22))
        counts=[r['count'] for r in rows if 18.2<=r['elapsed']<20]
        checks['duplicate_does_not_refresh_count']=bool(counts) and max(counts)==min(counts)
    unique={}
    for r in rows:
        if r['count']>0:unique.setdefault(r['count'],r['compute_us'])
    computes=sorted(unique.values())
    result=dict(passed=all(checks.values()),checks=checks,error=error,rows=len(rows),compute_us_p99=computes[int(.99*(len(computes)-1))] if computes else None,compute_us_max=max(computes,default=None),observer_returncode=child.returncode)
    (args.output/'summary.json').write_text(json.dumps(result,indent=2),encoding='utf-8');print(json.dumps(result),flush=True)
    return 0 if result['passed'] else 1

if __name__=='__main__':raise SystemExit(main())
