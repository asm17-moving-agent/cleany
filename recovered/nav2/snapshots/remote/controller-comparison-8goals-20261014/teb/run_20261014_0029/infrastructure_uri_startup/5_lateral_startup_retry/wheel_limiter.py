"""Gazebo-only uniform twist scaling against the actual mecanum wheel envelope."""
from math import isfinite

RADIUS = .0635
K = (.35 + .6038)/2
CAP = 5.0

def wheel_rates(twist):
    x,y,w=twist
    return ((x-y-K*w)/RADIUS, (x+y+K*w)/RADIUS,
            (x+y-K*w)/RADIUS, (x-y+K*w)/RADIUS)

def limit(twist, cap=CAP):
    if not all(isfinite(v) for v in twist):
        return (0.,0.,0.),0.
    peak=max(abs(v) for v in wheel_rates(twist))
    scale=min(1.,cap/peak) if peak else 1.
    return tuple(v*scale for v in twist),scale

def slew(previous, target, dt, acceleration):
    """Convex interpolation preserves the coupled envelope and bounds wheel slew."""
    delta = tuple(b-a for a,b in zip(previous,target))
    peak = max(map(abs,wheel_rates(delta)))
    alpha = min(1., acceleration*max(0.,dt)/peak) if peak else 1.
    return tuple(a+alpha*d for a,d in zip(previous,delta)),alpha

def main():
    import json
    import time
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import Twist
    from std_msgs.msg import String
    from rclpy.executors import ExternalShutdownException
    class WheelLimiter(Node):
        def __init__(self):
            super().__init__('wheel_envelope_limiter')
            self.declare_parameter('wheel_cap_rad_s',4.99)
            self.declare_parameter('wheel_acceleration_rad_s2',8.0)
            self.cap=float(self.get_parameter('wheel_cap_rad_s').value)
            self.acceleration=float(self.get_parameter('wheel_acceleration_rad_s2').value)
            self.raw=(0.,0.,0.);self.output=(0.,0.,0.);self.last_sim=None
            self.pub=self.create_publisher(Twist,'/cmd_vel',10)
            self.state=self.create_publisher(String,'/wheel_limiter/state',10)
            self.last=None
            self.create_subscription(Twist,'/nav2/cmd_vel',self.receive,10)
            self.create_timer(.05,self.tick)
        def emit(self,raw,reason):
            target,scale=limit(raw,self.cap)
            now=self.get_clock().now().nanoseconds*1e-9
            dt=min(.05,max(0.,now-self.last_sim)) if self.last_sim is not None else 0.
            out,alpha=slew(self.output,target,dt,self.acceleration)
            self.output=out;self.last_sim=now
            m=Twist();m.linear.x,m.linear.y,m.angular.z=out;self.pub.publish(m)
            s=String();s.data=json.dumps({'sim_s':self.get_clock().now().nanoseconds*1e-9,
                'reason':reason,'requested':raw,'target':target,'output':out,'scale':scale,'slew_alpha':alpha,'slew_dt_s':dt,'wheel_cap_rad_s':self.cap,'wheel_acceleration_rad_s2':self.acceleration,
                'requested_wheels_rad_s':wheel_rates(raw),'output_wheels_rad_s':wheel_rates(out)})
            self.state.publish(s)
        def receive(self,m):
            raw=(m.linear.x,m.linear.y,m.angular.z)
            self.last=time.monotonic();self.raw=raw
        def tick(self):
            stale=self.last is None or time.monotonic()-self.last>2
            raw=(0.,0.,0.) if stale else self.raw
            self.emit(raw,'NO_CONTROLLER_COMMAND_2S_WALL' if stale else 'COUPLED_WHEEL_SLEW')
    rclpy.init();node=WheelLimiter()
    try:rclpy.spin(node)
    except (KeyboardInterrupt,ExternalShutdownException):pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()

if __name__=='__main__':main()
