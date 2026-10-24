import json
import time
import os
os.environ.update(ROS_DOMAIN_ID='227',ROS_LOCALHOST_ONLY='1')
import rclpy
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from ros_gz_interfaces.msg import Contacts
from rclpy.qos import qos_profile_sensor_data
rclpy.init()
n=rclpy.create_node('teb_readonly_status')
status={}
def odom(m):
    p=m.pose.pose; status['truth']=[p.position.x,p.position.y,p.orientation.z,p.orientation.w]
    status['sim_s']=m.header.stamp.sec+m.header.stamp.nanosec*1e-9
def state(m,topic):
    try: status[topic]=json.loads(m.data)
    except ValueError: status[topic]=m.data
def cmd(m,topic):
    status[topic]=[m.linear.x,m.linear.y,m.angular.z]
n.create_subscription(Odometry,'/ground_truth/odom',odom,qos_profile_sensor_data)
for t in ('/head_pan/state','/body_guard_3d/state'):
    n.create_subscription(String,t,lambda m,t=t:state(m,t),qos_profile_sensor_data)
for t in ('/nav2/cmd_vel','/pan_ready/cmd_vel','/cmd_vel'):
    n.create_subscription(Twist,t,lambda m,t=t:cmd(m,t),qos_profile_sensor_data)
def joint(m):
    values=[abs(v) for k,v in zip(m.name,m.velocity) if k.endswith('wheel_joint')]
    if values:status['sampled_wheel_max_rad_s']=max(status.get('sampled_wheel_max_rad_s',0),max(values))
def contact(m):
    if m.contacts:status['contact_names']=[[v.collision1.name,v.collision2.name] for v in m.contacts[:2]]
n.create_subscription(JointState,'/joint_states',joint,qos_profile_sensor_data)
n.create_subscription(Contacts,'/evaluation/contacts/front_left_wheel_link',contact,qos_profile_sensor_data)
until=time.monotonic()+2
while time.monotonic()<until:rclpy.spin_once(n,timeout_sec=.1)
print(json.dumps(status))
n.destroy_node();rclpy.shutdown()
