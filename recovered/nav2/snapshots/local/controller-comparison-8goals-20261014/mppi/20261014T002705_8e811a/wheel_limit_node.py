#!/usr/bin/env python3
"""Gazebo-only coupled wheel projection, with an explicit command-loss stop."""
import json,math
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from geometry_msgs.msg import Twist
from std_msgs.msg import String

def project(value,maximum=5.,radius=.0635,lever=(.35+.6038)/2):
    if not all(math.isfinite(v) for v in value):return (0.,0.,0.),0.
    demand=(abs(value[0])+abs(value[1])+lever*abs(value[2]))/radius
    scale=min(1.,maximum/demand) if demand else 1.
    return tuple(v*scale for v in value),scale

class WheelLimit(Node):
    def __init__(self):
        super().__init__('mppi_wheel_limit',parameter_overrides=[Parameter('use_sim_time',value=True)])
        self.declare_parameter('timeout_s',.5)
        self.value=(0.,0.,0.);self.last=-math.inf
        self.pub=self.create_publisher(Twist,'/cmd_vel',10)
        self.diag=self.create_publisher(String,'/wheel_limit/state',10)
        self.create_subscription(Twist,'/nav2/cmd_vel',self.receive,10)
        self.create_timer(.05,self.check_timeout)
    def now(self):return self.get_clock().now().nanoseconds*1e-9
    def emit(self,output,scale,reason):
        m=Twist();m.linear.x,m.linear.y,m.angular.z=output;self.pub.publish(m)
        d=String();d.data=json.dumps({'sim_s':self.now(),'requested':self.value,'output':output,'scale':scale,'reason':reason,'source_age_s':self.now()-self.last if math.isfinite(self.last) else None});self.diag.publish(d)
    def receive(self,msg):
        self.value=(msg.linear.x,msg.linear.y,msg.angular.z);self.last=self.now()
        out,scale=project(self.value);self.emit(out,scale,'COUPLED_WHEEL_CAP' if scale<1. else 'PASS')
    def check_timeout(self):
        if self.now()-self.last>self.get_parameter('timeout_s').value:
            self.emit((0.,0.,0.),0.,'CONTROLLER_COMMAND_TIMEOUT')

def main():
    rclpy.init();node=WheelLimit()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:node.destroy_node();rclpy.try_shutdown()
if __name__=='__main__':main()
