#!/usr/bin/env python3
"""Exercise installed Nav2/BT using synthetic sensors on a non-hardware domain."""
import argparse
import json
import math
import os
from pathlib import Path
import time


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--exercise-slip',action='store_true')
    args=parser.parse_args()
    if int(os.environ.get('ROS_DOMAIN_ID','0'))==0:
        parser.error('Synthetic sensors are forbidden on hardware domain 0')
    import rclpy
    from rclpy.action import ActionClient
    from rclpy.qos import qos_profile_sensor_data
    from geometry_msgs.msg import TransformStamped,Twist
    from nav_msgs.msg import Odometry
    from nav2_msgs.action import NavigateToPose
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import String
    from tf2_ros import StaticTransformBroadcaster
    rclpy.init();node=rclpy.create_node('traction_isolated_probe')
    odom=node.create_publisher(Odometry,'odom',qos_profile_sensor_data)
    scans=[node.create_publisher(LaserScan,name,qos_profile_sensor_data) for name in ('scan_raw','scan')]
    broadcaster=StaticTransformBroadcaster(node)
    transforms=[]
    for parent,child,x,y in [('map','odom',.2075,-4.3197),('odom','base_link',0.,0.),('base_link','lidar_link',0.,0.)]:
        t=TransformStamped();t.header.frame_id=parent;t.child_frame_id=child;t.header.stamp=node.get_clock().now().to_msg()
        t.transform.rotation.w=1.;t.transform.translation.x=x;t.transform.translation.y=y;transforms.append(t)
    broadcaster.sendTransform(transforms)
    observations=[];phases=[];velocities=[]
    simulate=False;wheel_x=0.;last_sensor=time.monotonic()
    # The physical phone bridge normally forwards Nav2 intent into this topic.
    intent=node.create_publisher(Twist,'motion/cmd_vel',1)
    node.create_subscription(Twist,'nav2/cmd_vel',intent.publish,1)
    node.create_subscription(String,'navigation/slip',lambda m:observations.append(json.loads(m.data)),10)
    node.create_subscription(String,'navigation/traction_recovery',lambda m:phases.append(json.loads(m.data)),10)
    node.create_subscription(Twist,'navigation/safe_cmd_vel',lambda m:velocities.append([m.linear.x,m.linear.y,m.angular.z]),10)
    ranges=[]
    for i in range(720):
        a=-math.pi+i*math.pi/360
        ranges.append(min(3/max(abs(math.cos(a)),1e-9),2/max(abs(math.sin(a)),1e-9)))
    def publish():
        nonlocal wheel_x,last_sensor
        now=time.monotonic();dt=now-last_sensor;last_sensor=now
        speed=.12 if simulate and phases and phases[-1]['phase'] in ('normal','longitudinal_retry') else 0.
        wheel_x+=speed*dt
        stamp=node.get_clock().now()
        o=Odometry();o.header.frame_id='odom';o.child_frame_id='base_link';o.header.stamp=stamp.to_msg();o.pose.pose.orientation.w=1.
        o.pose.pose.position.x=wheel_x;o.twist.twist.linear.x=speed
        odom.publish(o)
        s=LaserScan();s.header.frame_id='lidar_link';s.header.stamp=(stamp-rclpy.duration.Duration(seconds=.13)).to_msg()
        s.angle_min=-math.pi;s.angle_increment=math.pi/360;s.scan_time=.13;s.time_increment=.13/720
        s.range_min=.1;s.range_max=10.;s.ranges=ranges
        for p in scans:p.publish(s)
    node.create_timer(.025,publish)
    client=ActionClient(node,NavigateToPose,'navigate_to_pose')
    start=time.monotonic()
    while time.monotonic()-start<15 and not (client.server_is_ready() and any(x['state']=='clear' for x in observations)):
        rclpy.spin_once(node,timeout_sec=.05)
    assert client.server_is_ready(),'Nav2 server did not activate'
    assert any(x['state']=='clear' for x in observations),'Installed monitor did not classify synthetic stationary room'
    from lifecycle_msgs.srv import GetState
    for name in ('bt_navigator','traction_retry/planner_server'):
        lifecycle=node.create_client(GetState,name+'/get_state')
        assert lifecycle.wait_for_service(timeout_sec=2.),name
        active=False
        while time.monotonic()-start<18 and not active:
            state=lifecycle.call_async(GetState.Request())
            while not state.done() and time.monotonic()-start<18:rclpy.spin_once(node,timeout_sec=.05)
            active=state.done() and state.result().current_state.id==3
            if not active:rclpy.spin_once(node,timeout_sec=.05)
        assert active,name+' did not reach active state'
    from action_msgs.srv import CancelGoal
    cancel_all=node.create_client(CancelGoal,'navigate_to_pose/_action/cancel_goal')
    assert cancel_all.wait_for_service(timeout_sec=2.)
    pending=cancel_all.call_async(CancelGoal.Request())
    while not pending.done():rclpy.spin_once(node,timeout_sec=.05)
    end=time.monotonic()+.4
    while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.05)
    goal=NavigateToPose.Goal();goal.pose.header.frame_id='map';goal.pose.header.stamp=node.get_clock().now().to_msg()
    goal.pose.pose.position.x=.7075;goal.pose.pose.position.y=-4.3197;goal.pose.pose.orientation.w=1.
    accepted=client.send_goal_async(goal)
    while not accepted.done() and time.monotonic()-start<20:rclpy.spin_once(node,timeout_sec=.05)
    assert accepted.done() and accepted.result().accepted,'Full BT goal rejected'
    handle=accepted.result();result=handle.get_result_async()
    while time.monotonic()-start<25 and not any(max(map(abs,v))>1e-5 for v in velocities):
        rclpy.spin_once(node,timeout_sec=.05)
    assert any(max(map(abs,v))>1e-5 for v in velocities),'No synthetic path tracking command observed'
    if args.exercise_slip:
        simulate=True
    else:
        cancel=handle.cancel_goal_async()
    while not result.done() and time.monotonic()-start<35:rclpy.spin_once(node,timeout_sec=.05)
    assert result.done(),'Synthetic navigation cancellation did not complete'
    expected=6 if args.exercise_slip else 5
    assert result.result().status==expected,f'Unexpected synthetic goal result: {result.result().status}'
    if args.exercise_slip:
        seen=[x['phase'] for x in phases]
        assert all(p in seen for p in ('braking','longitudinal_retry','failed')),seen
        assert any(x['phase']=='failed' and x['reason']=='slip_repeated_during_retry' for x in phases),phases[-10:]
    assert phases and any(x['phase']=='normal' for x in phases),'Full BT did not run'
    end=time.monotonic()+.5
    while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.05)
    assert all(max(map(abs,v))<1e-6 for v in velocities[-5:]),'Safe command did not settle to zero after cancellation'
    report=dict(domain=os.environ['ROS_DOMAIN_ID'],synthetic=True,physical_motion=False,
                nav_status=result.result().status,slip_states=sorted({x['state'] for x in observations}),
                recovery_phases=sorted({x['phase'] for x in phases}),safe_command_count=len(velocities),
                slip_injected=args.exercise_slip,
                failure_reasons=sorted({x['reason'] for x in phases if x['phase']=='failed'}))
    args.output.write_text(json.dumps(report,indent=2));print(json.dumps(report))
    node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
