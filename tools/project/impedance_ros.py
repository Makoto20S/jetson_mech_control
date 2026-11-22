"""ROS adapter for fixed-pose simulation impedance; explicit namespace is required."""
import argparse
import gc
import json
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT/'mujoco/ref01'))
from fixed_pose_impedance import FixedPoseImpedance
from ref01_dynamics_api import REF01Dynamics
from state_impedance import StateImpedance
sys.path.insert(0,str(ROOT/'mujoco/ref01/tools'))
from verify_fixed_pose import REFERENCE, STIFFNESS, DAMPING, LIMIT


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--namespace', required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--freeze-gc', action='store_true')
    args = parser.parse_args()
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from mech_exo_state.msg import RobotState
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

    class ImpedanceNode(Node):
        def __init__(self):
            super().__init__('impedance',namespace=args.namespace)
            self.gate = StateImpedance(FixedPoseImpedance(
                REF01Dynamics(),REFERENCE,STIFFNESS,DAMPING,LIMIT))
            self.rows = []
            self.pub = self.create_publisher(JointTrajectory,'pair_effort/target',1)
            self.create_subscription(RobotState,'state',self.receive,
                                     QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT))
            self.create_timer(.005,self.tick)
        def receive(self,msg):
            self.gate.update(msg,self.get_clock().now().nanoseconds,time.monotonic_ns())
            self.tick()
        def tick(self):
            start = time.monotonic_ns()
            now = self.get_clock().now().nanoseconds
            torque = self.gate.command(now,start)
            if torque is None:
                return
            msg = JointTrajectory(joint_names=['mit_comm_motor104','mit_comm_motor105'])
            msg.header.stamp.sec,msg.header.stamp.nanosec = divmod(now,10**9)
            msg.points = [JointTrajectoryPoint(effort=torque.motor104_105_nm.tolist())]
            self.pub.publish(msg)
            self.rows.append(dict(mono_ns=start,source_ns=self.gate.source_ns,scale=torque.scale,
                                  state_age_ms=(now-self.gate.source_ns)/1e6,
                                  compute_ms=(time.monotonic_ns()-start)/1e6,
                                  tau104_nm=msg.points[0].effort[0],tau105_nm=msg.points[0].effort[1]))

    rclpy.init()
    node = ImpedanceNode()
    gc_events = []
    def gc_event(phase, info):
        gc_events.append((time.monotonic_ns(),phase,info['generation']))
    gc.callbacks.append(gc_event)
    if args.freeze_gc:
        gc.collect()
        gc.freeze()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        gc.callbacks.remove(gc_event)
        if args.freeze_gc:
            gc.unfreeze()
        args.output.write_text(json.dumps(dict(reason=node.gate.reason,tripped=node.gate.tripped,
                                               commands=node.rows,gc_events=gc_events),indent=2))
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
