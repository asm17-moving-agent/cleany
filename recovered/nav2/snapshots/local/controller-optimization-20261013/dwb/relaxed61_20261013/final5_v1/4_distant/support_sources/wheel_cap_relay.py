#!/usr/bin/env python3
"""Gazebo-only direct delivery; safety outputs remain on shadow/cmd_vel."""
import json,math,time
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock,ClockType
from rclpy.parameter import Parameter
from geometry_msgs.msg import Twist
from std_msgs.msg import String
class Relay(Node):
 def __init__(self):
  super().__init__('wheel_cap_relay',parameter_overrides=[Parameter('use_sim_time',value=True)])
  self.request=[0.,0.,0.];self.received=0.;self.source_sim=0.
  self.declare_parameter('wheel_cap_rad_s',4.99);self.cap=self.get_parameter('wheel_cap_rad_s').value
  self.pub=self.create_publisher(Twist,'/cmd_vel',10);self.state=self.create_publisher(String,'/benchmark/relay_state',10)
  self.create_subscription(Twist,'/nav2/cmd_vel',self.receive,10)
  self.create_timer(.05,self.tick,clock=Clock(clock_type=ClockType.STEADY_TIME))
 def receive(self,m):
  self.request=[m.linear.x,m.linear.y,m.angular.z];self.received=time.monotonic();self.source_sim=self.get_clock().now().nanoseconds*1e-9
 def tick(self):
  now=time.monotonic();v=list(self.request);reason='DIRECT_WHEEL_ENVELOPE'
  if not all(map(math.isfinite,v)):v=[0.,0.,0.];reason='INVALID_COMMAND'
  elif now-self.received>5.:v=[0.,0.,0.];reason='STALE_NAV2_5S'
  x,y,z=v;k=(.35+.6038)/2;rates=[(x-y-k*z)/.0635,(x+y+k*z)/.0635,(x+y-k*z)/.0635,(x-y+k*z)/.0635];rate=max(map(abs,rates))
  scale=min(1.,self.cap/rate) if rate else 1.;v=[x*scale for x in v]
  m=Twist();m.linear.x,m.linear.y,m.angular.z=v;self.pub.publish(m)
  self.state.publish(String(data=json.dumps({'sim_s':self.get_clock().now().nanoseconds*1e-9,'wall_s':now,'source_receive_sim_s':self.source_sim,'source_wall_age_s':now-self.received,'reason':reason,'requested':self.request,'output':v,'requested_wheel_max_rad_s':rate,'scale':scale,'safety_outputs':'shadow/cmd_vel'})))
def main():
 rclpy.init();node=Relay()
 try:rclpy.spin(node)
 finally:
  node.pub.publish(Twist());node.destroy_node()
  if rclpy.ok():rclpy.shutdown()
if __name__=='__main__':main()
