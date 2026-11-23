#!/usr/bin/env python3
"""Receive-only ContraKin observer; no controller or command publisher."""
import math
import signal
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from rclpy.executors import SingleThreadedExecutor
from rcl_interfaces.msg import SetParametersResult
from sensor_msgs.msg import Imu
from mech_exo_state.msg import GaitPhase
from ao_core import AOConfig, AdaptiveOscillator


class AONode(Node):
    def __init__(self):
        super().__init__('ph_exo_ao')
        for key,value in dict(input_topic='/imu/imu1/data',output_topic='/ph_exo/gait_phase',
                              axis='x',input_scale_rad_s=1.0,input_offset_rad_s=0.0,
                              timeout_s=.1).items():
            self.declare_parameter(key,value)
        param=lambda k:self.get_parameter(k).value
        self.axis=param('axis')
        self.timeout=float(param('timeout_s'))
        if self.axis not in ('x','y','z') or not math.isfinite(self.timeout) or not .05<=self.timeout<=1:
            raise ValueError('axis must be x/y/z; timeout_s must be 0.05..1')
        self.core=AdaptiveOscillator(AOConfig(input_scale_rad_s=float(param('input_scale_rad_s')),
                                             input_offset_rad_s=float(param('input_offset_rad_s')),
                                             timeout_s=self.timeout))
        self.last_source=None; self.last_receive=None; self.count=0
        self.result=dict(state='INVALID',reason='no_input',valid=False,phase_rad=0.,frequency_hz=0.,amplitude=0.,residual=0.)
        self.compute_us=0.; self.rejection=None
        qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT)
        self.pub=self.create_publisher(GaitPhase,param('output_topic'),qos)
        self.sub=self.create_subscription(Imu,param('input_topic'),self.receive,qos)
        self.timer=self.create_timer(.01,self.tick)
        self.add_on_set_parameters_callback(lambda _:SetParametersResult(successful=False,reason='restart observer to change parameters'))

    def receive(self,msg):
        start=time.perf_counter_ns(); mono=time.monotonic()
        stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        now=self.get_clock().now().nanoseconds*1e-9
        value=getattr(msg.angular_velocity,self.axis)
        if stamp<=0 or stamp>now+.005 or now-stamp>self.timeout:
            self.rejection='source_time_invalid'
            self.result=self.core.update(float('nan'),stamp)
        elif self.last_source is not None and stamp<=self.last_source:
            self.rejection='source_time_not_increasing'
            self.result=self.core.update(float('nan'),stamp)
        elif msg.angular_velocity_covariance[0]<0 or not math.isfinite(value):
            self.rejection='angular_velocity_invalid'
            self.result=self.core.update(float('nan'),stamp)
        else:
            self.rejection=None
            self.result=self.core.update(value,stamp)
            if self.result['state']=='INVALID':
                self.rejection=self.result['reason']
            else:
                self.last_source=stamp; self.last_receive=mono; self.count+=1
        self.compute_us=(time.perf_counter_ns()-start)/1000.
        self.publish()

    def tick(self):
        if self.last_receive is not None:
            age=time.monotonic()-self.last_receive
            if age>self.timeout:
                self.result=self.core.check_timeout(self.last_source+age)
                self.rejection='input_timeout'
        self.publish()

    def publish(self):
        msg=GaitPhase(); msg.header.stamp=self.get_clock().now().to_msg()
        if self.last_source is not None:
            ns=round(self.last_source*1e9); msg.source_stamp.sec,msg.source_stamp.nanosec=divmod(ns,10**9)
        r=self.result
        msg.state='INVALID' if self.rejection else r['state']
        msg.reason=self.rejection or r['reason']; msg.valid=bool(r['valid'] and not self.rejection)
        msg.phase_reference_verified=False
        for key in ('phase_rad','frequency_hz','amplitude','residual'):
            setattr(msg,key,float(r[key]))
        now=self.get_clock().now().nanoseconds*1e-9
        msg.source_age_ms=(now-self.last_source)*1000 if self.last_source is not None else -1.
        if self.last_source is not None and (now-self.last_source>self.timeout or now<self.last_source-.005):
            msg.valid=False
            msg.state='INVALID'
            msg.reason='source_expired'
        msg.receive_age_ms=(time.monotonic()-self.last_receive)*1000 if self.last_receive is not None else -1.
        msg.compute_us=self.compute_us; msg.accepted_samples=self.count
        self.pub.publish(msg)


def main():
    # Background shells may inherit SIGINT=SIG_IGN. Explicit handlers only set a
    # flag; context destruction occurs after the bounded spin_once returns.
    stopped=[False]
    def stop(_signum,_frame): stopped[0]=True
    previous={s:signal.signal(s,stop) for s in (signal.SIGINT,signal.SIGTERM)}
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO); node=AONode()
    executor=SingleThreadedExecutor();executor.add_node(node)
    try:
        while rclpy.ok() and not stopped[0]:executor.spin_once(timeout_sec=.01)
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
        for s,handler in previous.items():signal.signal(s,handler)

if __name__=='__main__': main()
