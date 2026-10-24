#!/usr/bin/env python3
"""Explicitly separated Gazebo-only relaxed stage; TEB feasibility stays enabled."""
import argparse
import copy
import difflib
import json
import math
import os
import shutil
import subprocess
import re
from pathlib import Path
import xml.etree.ElementTree as ET
import yaml
from ament_index_python.packages import get_package_share_directory
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from ros_gz_interfaces.msg import Contacts
from rclpy.qos import qos_profile_sensor_data
import conservative_runner_snapshot as C
from relaxed_probe import SafetyProbe as RelaxedSafetyProbe

ROOT=Path(__file__).resolve().parent
from materialize import prepare as prepare_common, load_locked, COMMON
REFERENCE=Path('/home/donglee/workspace/cleany/controller-optimization-20261013/teb/relaxed_20261013_01/final/0_straight')
LOCKED=None
from scene_check import verify as verify_scene

def setup(out,candidate,rtf):
    overrides={}
    goal=LOCKED['goals'][int(out.name.split('_')[0])]
    scenario=prepare_common(out,goal,LOCKED)
    for name in ('runner.py','conservative_runner_snapshot.py','relaxed_probe.py','wheel_limiter.py','materialize.py','scene_check.py'):
        (out/name).write_text((ROOT/name).read_text())
    parameters=yaml.safe_load((REFERENCE/'config/relaxed_parameters.yaml').read_text())
    serialized=yaml.safe_dump(parameters,sort_keys=False).replace(str(REFERENCE),str(out))
    parameters=yaml.safe_load(serialized)
    # Shared planner and raster contract, retaining Humble-compatible runtime,
    # padded CAD footprints, depth observation layers, and relaxed guard routing.
    planner=(COMMON/'planner_params.yaml').read_text().replace('common://',str(COMMON.resolve())+'/')
    parameters['planner_server']=yaml.safe_load(planner)['planner_server']
    global_params=parameters['global_costmap']['global_costmap']['ros__parameters']
    global_params['resolution']=LOCKED['map_resolution_m']
    global_params['inflation_layer']['inflation_radius']=LOCKED['inflation_radius_m']
    for costmap in ('global_costmap','local_costmap'):
        p=parameters[costmap][costmap]['ros__parameters']
        p['footprint']=str(LOCKED['footprint']);p['footprint_padding']=LOCKED['footprint_padding_m']
    ctrl=parameters['controller_server']['ros__parameters']
    frequency=float(os.environ.get('TEB_RELAXED_FREQUENCY','10'))
    ctrl.update(controller_frequency=frequency,failure_tolerance=-1.)
    ctrl['goal_checker'].update(xy_goal_tolerance=.30,yaw_goal_tolerance=math.radians(10))
    home=LOCKED['home_world_map']
    parameters['amcl']['ros__parameters']['initial_pose'].update(x=home[0],y=home[1],yaw=home[2])
    ctrl['progress_checker'].update(required_movement_radius=.01,required_movement_angle=.01,movement_time_allowance=1200.)
    f=ctrl['FollowPath']
    f.update(max_vel_x=.3175,max_vel_x_backwards=.3175,max_vel_y=.3175,max_vel_theta=.3175/((.35+.6038)/2),
             acc_lim_x=.5,acc_lim_y=.5,acc_lim_theta=.8,min_obstacle_dist=.05,
             enable_homotopy_class_planning=False,enable_multithreading=False,
             dt_ref=.1,dt_hysteresis=.02,no_inner_iterations=5,no_outer_iterations=4,
             benchmark_timing_file=str(out/'compute_timing.csv'))
    f.update(xy_goal_tolerance=.30,yaw_goal_tolerance=math.radians(10))
    forbidden={'footprint_model','feasibility_check_no_poses','feasibility_check_lookahead_distance','include_costmap_obstacles','min_resolution_collision_check_angular'}
    if forbidden & overrides.keys():raise ValueError('Internal safety overrides reserved for separately labeled diagnostics')
    f.update(overrides)
    # Preserve the original BT's path validation/replanning. Its retries cannot be
    # the first limiting mechanism while failure_tolerance/progress run to watchdog.
    bt=(REFERENCE/'wait_before_detour.xml').read_text().replace('number_of_retries="3"','number_of_retries="10000"')
    (out/'relaxed_bt.xml').write_text(bt)
    parameters['bt_navigator']['ros__parameters']['default_nav_to_pose_bt_xml']=str(out/'relaxed_bt.xml')
    (out/'config/relaxed_parameters.yaml').write_text(yaml.safe_dump(parameters,sort_keys=False))
    if any(uri in (out/'config/relaxed_parameters.yaml').read_text() for uri in ('common://','fixture://','repo://','/home/ehdrms/')):
        raise ValueError('Unresolved runtime parameter path')
    for extra in ('wait_before_detour.xml','bt_manifest.json'):
        if (REFERENCE/extra).exists():(out/extra).write_text((REFERENCE/extra).read_text())
    share=Path(get_package_share_directory('cleany_gazebo_sim'))
    base=yaml.safe_load((share/'config/base.yaml').read_text())
    base['gazebo_command_guard']['ros__parameters'].update(max_linear_x=.4,max_linear_y=.4,max_angular_z=1.,cmd_vel_timeout_sec=2.)
    (out/'config/relaxed_base.yaml').write_text(yaml.safe_dump(base,sort_keys=False))
    gazebo=(share/'launch/gazebo_fortress.launch.py').read_text().replace("base_config = package_share / 'config' / 'base.yaml'","base_config = Path(__file__).resolve().parent / 'config' / 'relaxed_base.yaml'")
    (out/'gazebo_relaxed.launch.py').write_text(gazebo)
    # Observation-only contact sensors; robot collision geometry and physics remain.
    original_world=(out/'world.sdf').read_text()
    (out/'world_without_contact_sensors.sdf').write_text(original_world)
    tree=ET.parse(out/'world.sdf');world=tree.getroot().find('world')
    if world.find("plugin[@name='ignition::gazebo::systems::Contact']") is None:ET.SubElement(world,'plugin',{'filename':'ignition-gazebo-contact-system','name':'ignition::gazebo::systems::Contact'})
    robot=next(m for m in world.findall('model') if m.find("plugin[@name='ignition::gazebo::systems::MecanumDrive']") is not None)
    drive=robot.find("plugin[@name='ignition::gazebo::systems::MecanumDrive']")
    for tag in ('min_acceleration','max_acceleration'):
        value=drive.find(tag)
        if value is not None:drive.remove(value)
    # Body-axis interpolation can leave the wheel envelope. The ROS wheel
    # limiter now owns acceleration in coupled wheel coordinates.
    contact_topics=[]
    coverage=[]
    for link in robot.findall('link'):
        covered=set()
        for sensor in link.findall("sensor[@type='contact']"):
            topic=sensor.findtext('contact/topic') or sensor.findtext('topic')
            if not topic:raise ValueError('Canonical contact sensor missing topic')
            contact_topics.append(topic)
            covered.update(x.text for x in sensor.findall('contact/collision'))
        required={c.get('name') for c in link.findall('collision')}
        if not required<=covered:raise ValueError('Contact coverage missing '+link.get('name')+str(required-covered))
        coverage.append({'link':link.get('name'),'collision_count':len(required),'covered':True})
    (out/'contact_coverage.json').write_text(json.dumps(coverage,indent=2))
    tree.write(out/'world.sdf',encoding='unicode',xml_declaration=True)
    (out/'contact_sensors.patch').write_text(''.join(difflib.unified_diff(original_world.splitlines(True),(out/'world.sdf').read_text().splitlines(True),fromfile='world_without_contact_sensors.sdf',tofile='world.sdf')))
    bridge=[{'ros_topic_name':t,'gz_topic_name':t,'ros_type_name':'ros_gz_interfaces/msg/Contacts','gz_type_name':'ignition.msgs.Contacts','direction':'GZ_TO_ROS'} for t in contact_topics]
    (out/'config/contact_bridge.yaml').write_text(yaml.safe_dump(bridge,sort_keys=False))
    scenario.update(relaxed=True,contact_topics=contact_topics,controller_frequency_hz=frequency,
                    soft_timeout_sim_s=240,hard_timeout_sim_s=1200,hard_timeout_wall_s=1800,
                    bypass='Nav2 raw -> uniform wheel envelope -> /cmd_vel -> Gazebo command guard; pan/2D/3D guard outputs routed to /ignored/cmd_vel; those nodes still record counterfactual decisions',
                    comparison_limits={'wheel_rad_s':5.,'xy_tolerance':.30,'yaw_tolerance':math.radians(10)},
                    internal_feasibility_retained=True,contact_sensor_model=robot.get('name'),actuator_smoothing={'wheel_cap_rad_s':4.99,'wheel_acceleration_rad_s2':8.,'output_frequency_hz':20.,'gazebo_body_acceleration_limits':'removed; coupled wheel limiter owns slew'})
    (out/'scenario.json').write_text(json.dumps(scenario,indent=2))
    launch='''from pathlib import Path
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, SetRemap
from cleany_navigation.launch_runtime import nav2_nodes
def generate_launch_description():
 p=Path(__file__).resolve().parent
 parameters=str(p/'config/relaxed_parameters.yaml')
 import yaml
 camera=yaml.safe_load((p/'config/nav2_safety.yaml').read_text())['depth_camera_tf']
 adapters=[
  Node(package='ros_gz_bridge',executable='parameter_bridge',name='head_pan_bridge',arguments=['/model/cleany_mecanum/head_pan/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double'],remappings=[('/model/cleany_mecanum/head_pan/cmd_pos','/head_pan/command')],output='screen'),
  Node(package='cleany_gazebo_sim',executable='gazebo_pan_tf_publisher',parameters=[camera,{'use_sim_time':True}],output='screen'),
  Node(package='tf2_ros',executable='static_transform_publisher',name='depth_optical_tf',parameters=[{'use_sim_time':True}],arguments=['--frame-id','head_camera_depth_frame','--child-frame-id','head_camera_depth_optical_frame','--roll','-1.5707963267948966','--pitch','0','--yaw','-1.5707963267948966'])]
 gazebo=IncludeLaunchDescription(PythonLaunchDescriptionSource(str(p/'gazebo_relaxed.launch.py')),
  launch_arguments={'world':str(p/'world.sdf'),'headless':'true','headless_rendering':'true','use_sim_time':'true','sensor_profile':'lidar_depth_nav',
   'sensor_config':str(p/'config/sensor_tf.yaml'),'odometry_source':'wheel','wheel_odometry_config':str(p/'config/wheel_odometry.yaml'),
   'odometry_error_config':str(p/'config/odometry_error_ideal.yaml')}.items())
 navigation=GroupAction([SetRemap(src='cmd_vel',dst='nav2/cmd_vel'),*nav2_nodes(parameters,str(p/'map.yaml'),True,True)])
 counterfactual=GroupAction([SetRemap(src='cmd_vel',dst='ignored/cmd_vel'),
  *[Node(package='cleany_navigation',executable=n,name=n,output='screen',parameters=[parameters,{'use_sim_time':True}])
    for n in ['depth_clearance','pan_motion_gate','obstacle_memory','voxel_guard']],
  Node(package='nav2_collision_monitor',executable='collision_monitor',name='collision_monitor',output='screen',parameters=[parameters,{'use_sim_time':True}]),
  Node(package='nav2_lifecycle_manager',executable='lifecycle_manager',name='lifecycle_manager_safety',parameters=[{'use_sim_time':True,'autostart':True,'node_names':['collision_monitor']}])])
 limiter=ExecuteProcess(cmd=['python3','WHEEL_LIMITER_PATH','--ros-args','-p','use_sim_time:=true'],output='screen')
 contacts=Node(package='ros_gz_bridge',executable='parameter_bridge',name='contact_bridge',parameters=[{'config_file':str(p/'config/contact_bridge.yaml'),'use_sim_time':True}],output='screen')
 return LaunchDescription([gazebo,*adapters,navigation,counterfactual,limiter,contacts])
'''.replace('WHEEL_LIMITER_PATH',str(out/'wheel_limiter.py'))
    (out/'run.launch.py').write_text(launch)
    return scenario

class Probe(C.Probe):
    def truth(self):
        return RelaxedSafetyProbe.truth(self)
    def __init__(self,out,scenario):
        self.contacts=[];self.contact_counts={};self.limiter_states=[];self.last_floor_save={}
        super().__init__(out,scenario)
        for t in scenario['contact_topics']:
            self.create_subscription(Contacts,t,lambda m,t=t:self.contact(m,t),qos_profile_sensor_data)
        self.create_subscription(Twist,'/gazebo_cmd_vel',lambda m:self.cmd(m,'/gazebo_cmd_vel'),qos_profile_sensor_data)
        self.create_subscription(Twist,'/ignored/cmd_vel',lambda m:self.cmd(m,'/ignored/cmd_vel'),qos_profile_sensor_data)
        self.create_subscription(String,'/wheel_limiter/state',self.limiter,qos_profile_sensor_data)
    def initialize(self):
        RelaxedSafetyProbe.initialize(self)
        scene=verify_scene(self.output,self.scenario)
        self.event('actual_scene_verified',**scene)
        planner_keys=['GridBased.'+k for k in LOCKED['planner_parameters']['GridBased']]
        actual_planner=self.parameters('/planner_server',planner_keys)
        expected_planner={'GridBased.'+k:(v.replace('common://',str(COMMON.resolve())+'/') if isinstance(v,str) else v) for k,v in LOCKED['planner_parameters']['GridBased'].items()}
        assert actual_planner==expected_planner,(actual_planner,expected_planner)
        self.event('shared_input_runtime_verified',planner=actual_planner,
                   global_resolution=self.parameters('/global_costmap/global_costmap',['resolution']))
        self.event('relaxed_runtime_verified',cmd_vel_publishers=[p.node_name for p in self.get_publishers_info_by_topic('/cmd_vel')],
                   progress_checker=self.parameters('/controller_server',['failure_tolerance','controller_frequency','progress_checker.movement_time_allowance']),
                   downstream_limits=self.parameters('/gazebo_command_guard',['max_linear_x','max_linear_y','max_angular_z','cmd_vel_timeout_sec']),
                   wheel_limiter_parameters=self.parameters('/wheel_envelope_limiter',['wheel_cap_rad_s','wheel_acceleration_rad_s2']))
    def contact(self,m,topic):
        count=self.contact_counts.setdefault(topic,{'messages':0,'nonempty':0})
        count['messages']+=1;count['nonempty']+=bool(m.contacts)
        if m.contacts:
            # Preserve full physics messages in the bag. JSON retains every
            # non-floor contact plus 1Hz ground positive-control snapshots.
            interesting=any(c.collision1.name and c.collision2.name and 'ground_plane' not in c.collision1.name and 'ground_plane' not in c.collision2.name and not (c.collision1.name.startswith('cleany_mecanum::') and c.collision2.name.startswith('cleany_mecanum::')) for c in m.contacts)
            if not interesting and self.now()-self.last_floor_save.get(topic,-100)<1:return
            self.last_floor_save[topic]=self.now()
            self.contacts.append({'sim_s':self.now(),'stamp_s':m.header.stamp.sec+m.header.stamp.nanosec*1e-9,'topic':topic,
                'pairs':[{'a':c.collision1.name,'b':c.collision2.name,'id_a':c.collision1.id,'id_b':c.collision2.id,'max_depth_m':max(c.depths,default=None)} for c in m.contacts]})
    def limiter(self,m):
        try:self.limiter_states.append({'received_sim_s':self.now(),**json.loads(m.data)})
        except ValueError:pass
    def save(self):
        super().save()
        (self.output/'contacts.json').write_text(json.dumps({'counts':self.contact_counts,'samples':self.contacts}))
        (self.output/'wheel_limiter.json').write_text(json.dumps(self.limiter_states))

C.setup=setup;C.Probe=Probe

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);p.add_argument('--goal-index',type=int);a=p.parse_args()
    LOCKED=load_locked()
    C.GOALS=[(g['id'],g['pose_map_world']) for g in sorted(LOCKED['goals'],key=lambda g:g['index'])]
    a.output.mkdir(parents=True,exist_ok=True)
    (ROOT/'locked_snapshot.json').write_text(json.dumps(LOCKED,indent=2))
    (a.output/'manifest.json').write_text(json.dumps({'stage':'18F facility relaxed TEB8','goals':LOCKED['goals'],'initial_world_map_pose':LOCKED['home_world_map'],'wheel_cap_rad_s':5.,'output_wheel_cap_rad_s':4.99,'hard_timeout_sim_s':1200,'hard_timeout_wall_s':1800,'xy_goal_tolerance_m':.30,'yaw_goal_tolerance_deg':10.,'repetitions_per_goal':1,'shared_stationary_definition':'hypot(vx,vy)<.01 AND abs(wz)<.02 rad/s'},indent=2))
    for i in ([a.goal_index] if a.goal_index is not None else range(8)):
        out=a.output/(str(i)+'_'+C.GOALS[i][0])
        r=C.run(out,'teb',i,1.,227)
        if not r.get('goal_issued') and not r['completed']:
            # A startup retry never reissues a measured navigation goal.
            C.run(a.output/(str(i)+'_'+C.GOALS[i][0]+'_startup_retry'),'teb',i,1.,227)
