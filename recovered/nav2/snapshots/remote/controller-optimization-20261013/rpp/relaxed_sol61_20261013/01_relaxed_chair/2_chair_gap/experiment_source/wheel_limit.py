#!/usr/bin/env python3
"""Gazebo-only proportional wheel-rate limiter; no per-axis speed box."""
import math,time,json
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from geometry_msgs.msg import Twist
from std_msgs.msg import String
RADIUS=.0635
LEVER=(.35+.6038)/2

def wheel_rates(v):
    x,y,z=v;k=LEVER
    return [(x-y-k*z)/RADIUS,(x+y+k*z)/RADIUS,(x+y-k*z)/RADIUS,(x-y+k*z)/RADIUS]

def limit_twist(v,cap=4.90):
    if not math.isfinite(cap) or cap<=0 or cap>5 or not all(math.isfinite(x) for x in v):
        raise ValueError('finite twist and 0 < command wheel cap <= 5 required')
    peak=max(map(abs,wheel_rates(v)))
    scale=min(1.,cap/peak) if peak else 1.
    return [x*scale for x in v],scale

class WheelLimit(Node):
    def __init__(self):
        super().__init__('wheel_limit',parameter_overrides=[Parameter('use_sim_time',value=True)])
        self.declare_parameter('command_wheel_cap_rad_s',4.90)
        self.declare_parameter('frequency_hz',20.)
        self.declare_parameter('command_timeout_s',.5)
        self.cap=float(self.get_parameter('command_wheel_cap_rad_s').value)
        limit_twist([0.,0.,0.],self.cap)
        self.command=None;self.received_sim=0.;self.received_wall=0.
        self.publisher=self.create_publisher(Twist,'/cmd_vel',10)
        self.state=self.create_publisher(String,'/wheel_limit/state',10)
        self.create_subscription(Twist,'/nav2/cmd_vel',self.receive,10)
        self.create_timer(1./float(self.get_parameter('frequency_hz').value),self.update)
    def now(self):return self.get_clock().now().nanoseconds*1e-9
    def receive(self,m):
        self.command=[m.linear.x,m.linear.y,m.angular.z];self.received_sim=self.now();self.received_wall=time.monotonic()
    def update(self):
        sim=self.now();wall=time.monotonic();requested=self.command or [0.,0.,0.];reason='coupled_wheel_limit'
        timeout=float(self.get_parameter('command_timeout_s').value)
        try:
            if self.command is None or sim-self.received_sim>timeout or wall-self.received_wall>timeout:
                output=[0.,0.,0.];scale=0.;reason='input_command_timeout'
            else:output,scale=limit_twist(requested,self.cap)
        except ValueError:
            output=[0.,0.,0.];scale=0.;reason='invalid_twist'
        m=Twist();m.linear.x,m.linear.y,m.angular.z=output;self.publisher.publish(m)
        report={'sim_s':sim,'requested':requested,'output':output,'scale':scale,'command_wheel_cap_rad_s':self.cap,'predicted_wheel_rad_s':wheel_rates(output),'reason':reason,'input_sim_age_s':sim-self.received_sim,'input_wall_age_s':wall-self.received_wall}
        self.state.publish(String(data=json.dumps(report)))

def main():
    rclpy.init();node=WheelLimit()
    try:rclpy.spin(node)
    finally:
        node.publisher.publish(Twist());node.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
