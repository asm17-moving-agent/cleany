#!/usr/bin/env python3
"""Authorized Gazebo-only relaxed RPP campaign; native collision checks retained."""
import copy,argparse,hashlib,json,math,os,sys,xml.etree.ElementTree as ET
from pathlib import Path
import yaml
HERE=Path(__file__).resolve().parent
os.environ['CLEANY_COMPARISON_REPO']=str(HERE.parents[1]/'rpp-source')
sys.path.insert(0,str(HERE))
import common
ORIGINAL_SETUP=common.setup
TUNES={
    'relaxed':{},
    'short_ttc':{'max_allowed_time_to_collision_up_to_carrot':0.5},
    'lookahead25':{'min_lookahead_dist':0.25},
}

def setup(out,candidate,rtf):
    scenario=ORIGINAL_SETUP(out,candidate,rtf)
    source_snapshot=out/'experiment_source';source_snapshot.mkdir()
    for filename in ['runner.py','common.py','probe.py','wheel_limit.py']:(source_snapshot/filename).write_bytes((HERE/filename).read_bytes())
    motion_path=out/'config/comparison_motion.yaml';motion=yaml.safe_load(motion_path.read_text())
    ctrl=motion['controller_server']['ros__parameters']
    ctrl.update(controller_frequency=20.0,failure_tolerance=1200.0)
    ctrl['progress_checker'].update(required_movement_radius=0.001,required_movement_angle=0.001,movement_time_allowance=1200.0)
    ctrl['FollowPath'].update(desired_linear_vel=0.30,min_lookahead_dist=0.18,max_lookahead_dist=0.60,lookahead_time=0.8,rotate_to_heading_angular_vel=0.35,max_angular_accel=1.5,cost_scaling_dist=0.6)
    tune=os.environ['CLEANY_RPP_TUNE'];ctrl['FollowPath'].update(TUNES[tune])
    # The planner plugin and route strategies are identical; only added footprint padding changes.
    ctrl['goal_checker'].update(xy_goal_tolerance=.15,yaw_goal_tolerance=math.radians(10))
    motion_path.write_text(yaml.safe_dump(motion,sort_keys=False))
    safety_path=out/'config/nav2_safety.yaml';safety=yaml.safe_load(safety_path.read_text())
    physical=scenario['model_metadata']['navigation_geometry']['physical']
    for name in ['local_costmap','global_costmap']:
        safety[name][name]['ros__parameters'].update(footprint=physical,footprint_padding=0.0)
    safety_path.write_text(yaml.safe_dump(safety,sort_keys=False))
    # Contact instrumentation adds no collision volumes or physical parameter changes.
    world_path=out/'world.sdf';tree=ET.parse(world_path);world=tree.getroot().find('world')
    original_scene=ET.tostring(tree.getroot())
    original_world_sha=hashlib.sha256(world_path.read_bytes()).hexdigest()
    ET.SubElement(world,'plugin',filename='ignition-gazebo-contact-system',name='ignition::gazebo::systems::Contact')
    model=next(m for m in world.findall('model') if m.get('name')=='cleany_mecanum')
    sensor_count=0
    collision_count=0
    for link in model.findall('link'):
        collisions=link.findall('collision')
        if not collisions:continue
        sensor=ET.SubElement(link,'sensor',name='rpp_contact_all',type='contact')
        ET.SubElement(sensor,'always_on').text='true';ET.SubElement(sensor,'update_rate').text='50'
        contact=ET.SubElement(sensor,'contact')
        ET.SubElement(contact,'topic').text='/evaluation/physics_contacts'
        for collision in collisions:ET.SubElement(contact,'collision').text=collision.get('name');collision_count+=1
        sensor_count+=1
    verification=copy.deepcopy(tree.getroot())
    vw=verification.find('world')
    for element in list(vw.findall('plugin')):
        if element.get('name')=='ignition::gazebo::systems::Contact':vw.remove(element)
    for link in verification.iter('link'):
        for sensor in list(link.findall('sensor')):
            if sensor.get('name','').startswith('rpp_contact_'):link.remove(sensor)
    assert ET.tostring(verification)==original_scene,'Contact instrumentation modified the existing scene'
    tree.write(world_path,encoding='unicode')
    launch_path=out/'run.launch.py';launch=launch_path.read_text()
    launch=launch.replace('from launch.actions import IncludeLaunchDescription','from launch.actions import IncludeLaunchDescription, GroupAction, ExecuteProcess\nfrom launch_ros.actions import SetRemap')
    launch=launch.replace("        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(share/'launch/amcl_nav2_safety.launch.py')),", "        GroupAction([SetRemap(src='voxel_guard:cmd_vel', dst='legacy_guard/cmd_vel'),\n        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(share/'launch/amcl_nav2_safety.launch.py')),")
    launch=launch.replace("'memory_params_file':str(p/'config/obstacle_memory.yaml')", "'memory_params_file':str(p/'config/obstacle_memory.yaml')")
    needle="**({'voxel_params_file':str(p/'config/voxel_guard.yaml')} if (p/'config/voxel_guard.yaml').exists() else {})}.items()),"
    launch=launch.replace(needle,needle[:-1]+"]),")
    extra=f"""        ExecuteProcess(cmd=['python3', '{HERE}/wheel_limit.py'], output='screen'),
        Node(package='ros_gz_bridge', executable='parameter_bridge', name='rpp_contact_bridge', output='screen',
             arguments=['/evaluation/physics_contacts@ros_gz_interfaces/msg/Contacts[ignition.msgs.Contacts']),
"""
    launch=launch.replace('    ])',extra+'    ])')
    compile(launch,str(launch_path),'exec');launch_path.write_text(launch)
    scenario.update(profile='gazebo_only_relaxed',rpp_tune_name=tune,rpp_tune=TUNES[tune],effective_costmap_footprint=physical,effective_footprint_padding=0.0,
        safety_bypass={'final_authority':'wheel_limit','input':'/nav2/cmd_vel','output':'/cmd_vel','legacy_guard_output':'/legacy_guard/cmd_vel','bypassed_motion_vetoes':['pan_motion_gate','collision_monitor','voxel_guard freshness/obstacle/speed box'],'background_sensor_costmaps_retained':True,'native_rpp_collision_detection':True},
        contact_instrumentation={'physics_geometry_unchanged':True,'sensor_count':sensor_count,'collision_count':collision_count,'declared_frequency_hz':50,'actual_publication_note':'Fortress Contact publishes when contacts exist per physics step; actual intervals are recorded; declared SDF update_rate is not enforced by this system','topic':'/evaluation/physics_contacts','pre_instrumentation_world_sha256':original_world_sha},
        command_wheel_cap_rad_s=4.90,physical_wheel_limit_rad_s=5.0,removed_axis_speed_box=True,hard_sim_timeout_s=1200,hard_wall_watchdog_s=1800,soft_schedule_sim_s=240,controller_frequency_hz=20.,command_frequency_hz=20.,rpp_limit_note='Native Humble RPP, no native linear acceleration limit. Final proportional wheel limiter preserves curvature and uses a 4.90rad/s command margin under the physical 5rad/s cap. No per-axis box, controller yaw clipping or velocity smoother.')
    (out/'scenario.json').write_text(json.dumps(scenario,indent=2))
    (out/'config/relaxed_profile.yaml').write_text(yaml.safe_dump({k:scenario[k] for k in ['profile','safety_bypass','command_wheel_cap_rad_s','physical_wheel_limit_rad_s','effective_costmap_footprint','effective_footprint_padding','hard_sim_timeout_s','hard_wall_watchdog_s','controller_frequency_hz','command_frequency_hz']},sort_keys=False))
    return scenario

def main():
    p=argparse.ArgumentParser();p.add_argument('--tune',choices=sorted(TUNES),default='relaxed');p.add_argument('--output',type=Path,required=True);p.add_argument('--goal-index',type=int,choices=range(5));p.add_argument('--sim-limit',type=float,default=1200.);p.add_argument('--domain',type=int,default=228);a=p.parse_args()
    if a.domain!=228:p.error('RPP owns domain 228 only')
    if not 0<a.sim_limit<=1200:p.error('sim-limit must be 0..1200')
    os.environ.update(CLEANY_RPP_TUNE=a.tune,CLEANY_RPP_SIM_LIMIT=str(a.sim_limit),CLEANY_BENCHMARK_TIMING='1')
    common.setup=setup;a.output.mkdir(parents=True,exist_ok=False)
    common.describe(a.output)
    manifest=json.loads((a.output/'manifest.json').read_text());manifest.update(profile='gazebo_only_relaxed',sim_timeout_s=a.sim_limit,wall_watchdog_s=1800,soft_schedule_sim_s=240,limits={'wheel_rad_s':5.,'command_wheel_cap_rad_s':4.90,'xy_tolerance':.15,'yaw_tolerance':math.radians(10)},filter='GT20Hz sim time, centered5point smoothing, acceleration/jerk, .25s edge trim')
    manifest.pop('wheel_worst_case_rad_s',None);manifest['pure_straight_physical_max_m_s']=.3175
    (a.output/'manifest.json').write_text(json.dumps(manifest,indent=2))
    for index in [a.goal_index] if a.goal_index is not None else range(5):
        r=common.run(a.output/f'{index}_{common.GOALS[index][0]}','rpp',index,1.,228)
        if not r.get('completed'):raise RuntimeError('Infrastructure error; preserve result and stop campaign')
if __name__=='__main__':main()
