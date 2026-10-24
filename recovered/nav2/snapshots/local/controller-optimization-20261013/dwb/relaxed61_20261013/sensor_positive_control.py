#!/usr/bin/env python3
"""Separate forced-overlap sensor check; excluded from navigation comparison."""
import json,math,os,signal,subprocess,time,uuid,xml.etree.ElementTree as E
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from ros_gz_interfaces.msg import Contacts
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
root=Path(__file__).parent;source=root/'pilot02_chair_gap/2_chair_gap';out=root/'contact_positive_control';out.mkdir(exist_ok=False)
partition='dwb_contact_validation_'+uuid.uuid4().hex;os.environ.update(ROS_DOMAIN_ID='225',ROS_LOCALHOST_ONLY='1',IGN_PARTITION=partition,GZ_PARTITION=partition,IGN_IP='127.0.0.1',GZ_IP='127.0.0.1',IGN_DISCOVERY_MSG_PORT='22551',IGN_DISCOVERY_SRV_PORT='22552')
tree=E.parse(source/'world.sdf');world=tree.getroot().find('world');robot=world.find("model[@name='cleany_mecanum']");pose=list(map(float,robot.findtext('pose').split()));x,y,z=0.,.24,-.274;c,s=math.cos(pose[5]),math.sin(pose[5]);position=[pose[0]+c*x-s*y,pose[1]+s*x+c*y,pose[2]+z]
model=E.SubElement(world,'model',name='sensor_validation_fixture');E.SubElement(model,'static').text='true';E.SubElement(model,'pose').text=' '.join(map(str,position))+' 0 0 0';link=E.SubElement(model,'link',name='body');collision=E.SubElement(link,'collision',name='forced_overlap');box=E.SubElement(E.SubElement(collision,'geometry'),'box');E.SubElement(box,'size').text='.03 .03 .03';tree.write(out/'world.sdf',encoding='unicode',xml_declaration=True)
(out/'diagnostic_manifest.json').write_text(json.dumps({'excluded_from_navigation_comparison':True,'reason':'Positive control: known base_link collision0 overlaps a static 3cm cube. Validates body/environment contact telemetry. No Nav2 goal or commanded motion.','fixture_world_position':position,'domain':225,'partition':partition,'source_world':str(source/'world.sdf')},indent=2))
class Probe(Node):
 def __init__(self):
  super().__init__('contact_sensor_validation',parameter_overrides=[Parameter('use_sim_time',value=True)]);self.events=[];self.ground=0;self.gt=None
  self.create_subscription(Contacts,'/benchmark/contacts/c0',self.contact,qos_profile_sensor_data);self.create_subscription(Contacts,'/benchmark/contacts/c25',self.floor,qos_profile_sensor_data);self.create_subscription(Odometry,'/ground_truth/odom',self.truth,qos_profile_sensor_data)
 def floor(self,m):self.ground+=len(m.contacts)
 def truth(self,m):self.gt={'stamp_s':m.header.stamp.sec+m.header.stamp.nanosec*1e-9,'world_xyz':[m.pose.pose.position.x,m.pose.pose.position.y,m.pose.pose.position.z]}
 def contact(self,m):
  for c in m.contacts:
   if 'sensor_validation_fixture' in c.collision1.name+' '+c.collision2.name and len(self.events)<100:
    self.events.append({'sim_s':m.header.stamp.sec+m.header.stamp.nanosec*1e-9,'collision1':c.collision1.name,'collision2':c.collision2.name,'positions':[[v.x,v.y,v.z] for v in c.positions],'depths':list(c.depths),'latest_GT':self.gt})
def stop(p):
 if p and p.poll() is None:
  os.killpg(p.pid,signal.SIGINT)
  try:p.wait(timeout=8)
  except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL);p.wait(timeout=5)
rclpy.init();node=Probe();bridge=bag=launch=None;result={}
try:
 bridge=subprocess.Popen(['ros2','run','ros_gz_bridge','parameter_bridge','/benchmark/contacts/c0@ros_gz_interfaces/msg/Contacts[ignition.msgs.Contacts','/benchmark/contacts/c25@ros_gz_interfaces/msg/Contacts[ignition.msgs.Contacts'],stdout=(out/'bridge.log').open('w'),stderr=subprocess.STDOUT,start_new_session=True)
 bag=subprocess.Popen(['ros2','bag','record','-o',str(out/'bag'),'/clock','/ground_truth/odom','/benchmark/contacts/c0','/benchmark/contacts/c25'],stdout=(out/'bag.log').open('w'),stderr=subprocess.STDOUT,start_new_session=True)
 end=time.monotonic()+1
 while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.02)
 launch=subprocess.Popen(['ros2','launch',str(source/'backend.launch.py'),'world:='+str(out/'world.sdf'),'headless:=true','sensor_profile:=lidar_depth_nav','odometry_source:=wheel','wheel_odometry_config:='+str(source/'config/wheel_odometry.yaml'),'odometry_error_config:='+str(source/'config/odometry_error_ideal.yaml')],stdout=(out/'launch.log').open('w'),stderr=subprocess.STDOUT,start_new_session=True)
 (out/'session.json').write_text(json.dumps({'domain':225,'partition':partition,'launch_pid':launch.pid,'bridge_pid':bridge.pid,'bag_pid':bag.pid,'supervisor_pid':os.getpid()}));deadline=time.monotonic()+45
 while node.get_clock().now().nanoseconds*1e-9<3 and time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.005)
 result={'body_fixture_contact_observed':bool(node.events),'events':node.events,'wheel_ground_samples':node.ground,'final_GT':node.gt,'sim_s':node.get_clock().now().nanoseconds*1e-9,'excluded_from_navigation_comparison':True,'depth_unavailable_reason':'ROS Contacts depths arrays empty' if node.events and not node.events[0]['depths'] else None}
finally:
 stop(bag);stop(launch);stop(bridge);node.destroy_node()
 if rclpy.ok():rclpy.shutdown()
 (out/'validation.json').write_text(json.dumps(result,indent=2));print(json.dumps(result),flush=True)
