"""Isaac Lab USD environment clones sharing one PhysX step and ROS clock."""
from __future__ import annotations

import json
import math
import os
from pathlib import Path
import time
import numpy as np
from isaacsim import SimulationApp

ROOT = Path('/output')
SETTINGS = json.loads((ROOT/'config/clones.json').read_text())
app = SimulationApp({'headless': True, 'hide_ui': True, 'renderer': 'MinimalRendering',
                     'limit_cpu_threads': 4, 'disable_viewport_updates': True})
import isaacsim.core.experimental.utils.app as app_utils
import omni.physx
import omni.timeline
import omni.usd
from isaacsim.core.simulation_manager import SimulationManager
from pxr import Gf, Usd, UsdGeom, UsdPhysics


def main():
    for extension in ('isaacsim.asset.importer.urdf', 'isaacsim.ros2.core',
                      'isaacsim.ros2.bridge', 'isaacsim.ros2.control'):
        app_utils.enable_extension(extension)
    app.update()
    import rclpy
    import torch
    import yaml
    from isaaclab.cloner import usd_replicate
    from isaacsim.ros2.control import Ros2ControlManager
    from isaacsim.ros2.control.bindings import _isaacsim_ros2_control as backend
    from builtin_interfaces.msg import Time
    from controller_manager_msgs.srv import ConfigureController, LoadController, SwitchController
    from geometry_msgs.msg import TransformStamped, Twist
    from nav_msgs.msg import Odometry
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import Bool
    from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
    from cadence import measured_twist, publication_config
    from profiling import LoopProfiler
    from ros_scope import remappings
    from scene_assets import build_stage

    device = SETTINGS.get('physics_device', 'cpu')

    outputs = [ROOT/f'env_{i}' for i in range(SETTINGS['num_envs'])]
    configs = [json.loads((p/'config/room.json').read_text()) for p in outputs]
    assert all(c['physics_hz'] == 500 and c['physics_sync'] == 'fabric' for c in configs)
    stage = build_stage(outputs[0], configs[0])
    stage.Export(str(ROOT/'prototype.usda'))
    assert omni.usd.get_context().open_stage(str(ROOT/'prototype.usda'))
    stage = omni.usd.get_context().get_stage()
    UsdGeom.Xform.Define(stage, '/World/envs/env_0')
    for name in ('cleany', 'Room', 'Facility18'):
        editor = Usd.NamespaceEditor(stage)
        editor.MovePrimAtPath('/'+name, '/World/envs/env_0/'+name)
        assert editor.CanApplyEdits(), name
        assert editor.ApplyEdits(), name
    stage.SetDefaultPrim(stage.GetPrimAtPath('/World'))
    origins = np.array([[i*SETTINGS['spacing_m'], 0., 0.] for i in range(len(outputs))])
    # Use Isaac Lab's public cloning API, retaining CPU PhysX for the comparison.
    usd_replicate(stage, ['/World/envs/env_0'], ['/World/envs/env_{}'],
                  torch.arange(len(outputs)), positions=torch.tensor(origins))
    import_report = json.loads((outputs[0]/'evaluations/world-import.json').read_text())
    for i, output in enumerate(outputs):
        root = f'/World/envs/env_{i}'
        prims = list(Usd.PrimRange(stage.GetPrimAtPath(root)))
        colliders = sum(p.HasAPI(UsdPhysics.CollisionAPI) and '/Facility18/' in str(p.GetPath()) for p in prims)
        assert colliders == import_report['static_colliders'], (i, colliders)
        for prim in prims:
            if prim.IsA(UsdPhysics.Joint):
                for name in ('physics:body0', 'physics:body1'):
                    for target in prim.GetRelationship(name).GetTargets():
                        assert str(target).startswith(root+'/'), (i, target)
        (output/'evaluations/world-import.json').write_text(json.dumps(import_report, indent=2))
    (ROOT/'evaluations/clones.json').write_text(json.dumps({
        'api': 'isaaclab.cloner.usd_replicate', 'isaaclab_commit': SETTINGS['isaaclab_commit'],
        'num_envs': len(outputs), 'origins': origins.tolist(), 'physics_backend': 'GPU PhysX' if device.startswith('cuda') else 'CPU PhysX',
        'shared_physics_scene': True, 'per_environment_static_colliders': import_report['static_colliders'],
        'joint_targets_local_to_environment': True,
        'isolation': '100m spatial separation, local ROS coordinate transforms and ROS namespaces'}, indent=2))
    stage.Export(str(ROOT/'cloned-stage.usda'))
    SimulationManager.setup_simulation(dt=.002, device='cpu')
    if device.startswith('cuda'):
        # The packaged ros2_control backend owns host tensor buffers. Keep
        # PhysX CPU readback while enabling the GPU solver and broadphase.
        import carb.settings
        carb.settings.get_settings().set_int('/physics/cudaDevice', 0)
        for physics_scene in SimulationManager.get_physics_scenes():
            physics_scene.set_broadphase_type('GPU')
            physics_scene.set_enabled_gpu_dynamics(True)
    SimulationManager.enable_fabric(True)
    timeline = omni.timeline.get_timeline_interface()
    rclpy.init()
    clock_node = rclpy.create_node('cleany_clones_clock')
    clock_pub = clock_node.create_publisher(Clock, '/clock', 10)
    clock_count = 0
    clock_published = 0
    profiler = None

    def step():
        nonlocal clock_count, clock_published
        SimulationManager.step()
        if profiler:
            profiler.mark('physics_step')
        seconds = SimulationManager.get_simulation_time()
        if clock_count % 5 == 0:
            clock_pub.publish(Clock(clock=Time(sec=int(seconds), nanosec=int(seconds % 1*1e9))))
            clock_published += 1
        clock_count += 1
        if profiler:
            profiler.mark('clock_publish')

    def wait(node, predicate, timeout=60.):
        deadline = time.monotonic()+timeout
        while not predicate():
            if time.monotonic() > deadline:
                raise TimeoutError('Cloned environment controller setup timeout')
            step()
            rclpy.spin_once(node, timeout_sec=0)

    class Environment:
        def __init__(self, i):
            self.output, self.config, self.origin = outputs[i], configs[i], origins[i]
            self.prefix = f'env_{i}'
            self.robot = f'/World/envs/{self.prefix}/cleany'
            args = ['--ros-args']
            for source, destination in remappings(self.prefix):
                args.extend(['-r', f'{source}:={destination}'])
            self.node = rclpy.create_node('cleany_isaac_adapter', namespace=self.prefix, cli_args=args)
            self.command = {'value': np.zeros(3), 'received': -math.inf, 'blocked': False}
            def command(msg):
                value = np.array([msg.linear.x, msg.linear.y, msg.angular.z])
                self.command.update(value=value if np.isfinite(value).all() else np.zeros(3), received=time.monotonic())
            self.node.create_subscription(Twist, 'cmd_vel', command, 10)
            self.node.create_subscription(Bool, 'isaac/test/block_base', lambda m: self.command.update(blocked=m.data), 10)
            self.odom = self.node.create_publisher(Odometry, 'wheel/odom', 10)
            self.scan = self.node.create_publisher(LaserScan, 'scan', 10)
            self.tf = TransformBroadcaster(self.node)
            self.static = StaticTransformBroadcaster(self.node)
            transform = TransformStamped()
            transform.header.frame_id, transform.child_frame_id = 'base_link', 'isaac_lidar'
            transform.transform.translation.x = self.config.get('scan_x', 0.)
            transform.transform.translation.z = self.config['scan_z']-self.config['spawn'][2]
            transform.transform.rotation.w = 1.
            self.static.sendTransform(transform)
            self.view = SimulationManager.get_physics_sim_view().create_articulation_view(self.robot+'/Geometry/base_link')
            assert self.view.count == 1
            positions = self.view.get_dof_positions().copy()
            for j, name in enumerate(self.view.shared_metatype.dof_names):
                positions[0,j] = self.config['park_joints'][name]
            self.view.set_dof_positions(positions, np.array([0], dtype=np.int32))
            self.view.set_dof_position_targets(positions, np.array([0], dtype=np.int32))
            canonical = yaml.safe_load((self.output/'config/controllers.yaml').read_text())
            scoped = self.output/'config/controllers-scoped.yaml'
            scoped.write_text(yaml.safe_dump({self.prefix: canonical}))
            timeline.pause()
            app.update()
            Ros2ControlManager.setup(self.robot, str(scoped), urdf_path=str(self.output/'assets/cleany.urdf'),
                                    namespace='/'+self.prefix, publish_robot_description=False, use_sim_time=True)
            timeline.play()
            app.update()
            names = ['joint_state_broadcaster', 'left_arm_controller', 'right_arm_controller',
                     'left_gripper_controller', 'right_gripper_controller']
            for name in names:
                self.call(LoadController, 'load_controller', LoadController.Request(name=name))
                self.call(ConfigureController, 'configure_controller', ConfigureController.Request(name=name))
            self.call(SwitchController, 'switch_controller', SwitchController.Request(activate_controllers=names, strictness=2))
            self.names = names

        def call(self, kind, endpoint, request):
            client = self.node.create_client(kind, 'controller_manager/'+endpoint)
            wait(self.node, client.service_is_ready)
            future = client.call_async(request)
            wait(self.node, future.done)
            result = future.result()
            self.node.destroy_client(client)
            assert result.ok, (self.prefix, endpoint)

        def ready(self):
            self.view = SimulationManager.get_physics_sim_view().create_articulation_view(self.robot+'/Geometry/base_link')
            pose = self.view.get_root_transforms().copy()
            pose[0,:3] = np.array(self.config['spawn'])+self.origin
            heading = self.config['spawn_yaw']/2
            pose[0,3:] = [0.,0.,math.sin(heading),math.cos(heading)]
            self.view.set_root_transforms(pose, np.array([0],dtype=np.int32))
            self.old_pose = pose[0].copy()
            self.old_pose[:3] -= self.origin
            self.old_time = SimulationManager.get_simulation_time()
            self.base = (self.output/'evaluations/base.jsonl').open('w',buffering=1)
            media = self.output/'media'
            media.mkdir(exist_ok=True)
            self.motion = (media/'motion.jsonl').open('w',buffering=1)
            paths = []
            prims = list(Usd.PrimRange(stage.GetPrimAtPath(self.robot)))
            for name in self.view.shared_metatype.link_names:
                matches = [str(p.GetPath()) for p in prims if p.GetName()==name and '/Geometry/' in str(p.GetPath())]
                assert len(matches)==1, (name,matches)
                paths.append(matches[0])
            (media/'links.json').write_text(json.dumps(paths))
            (media/'coordinates.json').write_text(json.dumps({'poses':'environment-local','origin_world':self.origin.tolist()}))
            self.counts = {'odometry':0, 'scan':0}
            (self.output/'evaluations/adapter-ready.json').write_text(json.dumps({'controllers':self.names,
                'namespace':self.prefix,'actuation':'ideal root velocity','physical_cleanup':False}))

        def apply(self, started):
            rclpy.spin_once(self.node, timeout_sec=0)
            self.pose = self.view.get_root_transforms()[0].copy()
            velocity = self.view.get_root_velocities()[0].copy()
            qx,qy,qz,qw = self.pose[3:]
            self.yaw = math.atan2(2*(qw*qz+qx*qy),1-2*(qy*qy+qz*qz))
            cmd = self.command
            self.requested = cmd['value'] if started-cmd['received'] < self.config['command_timeout_s'] and not cmd['blocked'] and not (self.output/'stop').exists() else np.zeros(3)
            vx,vy,wz = np.clip(self.requested,[-.25,-.25,-.5],[.25,.25,.5])
            c,s = math.cos(self.yaw),math.sin(self.yaw)
            velocity[[0,1,3,4,5]] = [c*vx-s*vy,s*vx+c*vy,0.,0.,wz]
            self.view.set_root_velocities(velocity.reshape(1,6),np.array([0],dtype=np.int32))

        def publish(self, count, stamp_s):
            if count % 10 == 0:
                self.pose = self.view.get_root_transforms()[0].copy()
                local = self.pose.copy()
                local[:3] -= self.origin
                stamp = Time(sec=int(stamp_s),nanosec=int(stamp_s%1*1e9))
                msg = Odometry()
                msg.header.stamp,msg.header.frame_id,msg.child_frame_id = stamp,'odom','base_link'
                msg.pose.pose.position.x,msg.pose.pose.position.y,msg.pose.pose.position.z = map(float,local[:3])
                rotation = msg.pose.pose.orientation
                rotation.x,rotation.y,rotation.z,rotation.w = map(float,local[3:])
                vx,vy,wz = measured_twist(self.old_pose,local,stamp_s-self.old_time)
                msg.twist.twist.linear.x,msg.twist.twist.linear.y,msg.twist.twist.angular.z = float(vx),float(vy),float(wz)
                self.old_pose,self.old_time = local.copy(),stamp_s
                self.odom.publish(msg)
                transform = TransformStamped()
                transform.header,transform.child_frame_id = msg.header,'base_link'
                transform.transform.translation.x,transform.transform.translation.y,transform.transform.translation.z = map(float,local[:3])
                transform.transform.rotation = rotation
                self.tf.sendTransform(transform)
                self.counts['odometry'] += 1
            if count % 50 == 0:
                links = self.view.get_link_transforms()[0].copy()
                links[:,:3] -= self.origin
                self.motion.write(json.dumps({'sim_time':stamp_s,'wall_time':time.time(),'links':links.tolist()})+'\n')
                scan = LaserScan()
                scan.header.stamp,scan.header.frame_id = stamp,'isaac_lidar'
                scan.angle_min,scan.angle_max,scan.angle_increment = -math.pi,math.pi,math.pi/180
                scan.range_min,scan.range_max,scan.scan_time = .15,12.,.1
                ranges = []
                c,s = math.cos(self.yaw),math.sin(self.yaw)
                origin = (float(self.pose[0])+c*self.config.get('scan_x',0.),float(self.pose[1])+s*self.config.get('scan_x',0.),float(self.pose[2])+self.config['scan_z']-self.config['spawn'][2])
                for angle in np.linspace(-math.pi,math.pi,361):
                    hits = []
                    def hit(hit):
                        if not str(hit.rigid_body).startswith(self.robot):
                            hits.append(float(hit.distance))
                        return True
                    query.raycast_all(origin,(math.cos(self.yaw+angle),math.sin(self.yaw+angle),0.),12.,hit)
                    ranges.append(min(hits) if hits else math.inf)
                scan.ranges = ranges
                self.scan.publish(scan)
                self.counts['scan'] += 1
                self.base.write(json.dumps({'time':stamp_s,'pose':local.tolist(),'velocity':self.view.get_root_velocities()[0].tolist(),'requested':self.requested.tolist(),'blocked':self.command['blocked']})+'\n')

    timeline.play()
    app.update()
    SimulationManager.initialize_physics()
    wait(clock_node, backend.is_ready)
    environments = [Environment(i) for i in range(len(outputs))]
    sample = environments[0].view.get_root_transforms()
    physics = {'requested_device': device, 'manager_device': str(SimulationManager.get_device()),
               'tensor_device': str(getattr(sample, 'device', 'cpu')), 'tensor_pipeline': 'host_readback',
               'gpu_dynamics': SimulationManager.is_gpu_dynamics_enabled(),
               'scenes': [{'broadphase': scene.get_broadphase_type(),
                           'gpu_dynamics': scene.get_enabled_gpu_dynamics()} for scene in SimulationManager.get_physics_scenes()]}
    assert physics['gpu_dynamics'] == device.startswith('cuda'), physics
    assert isinstance(sample, np.ndarray), physics
    assert all(s['broadphase'] == ('GPU' if device.startswith('cuda') else 'MBP') for s in physics['scenes']), physics
    from pxr import PhysicsSchemaTools
    from omni.physx.bindings import _physx

    def physics_statistics():
        result = []
        for prim in stage.Traverse():
            if prim.IsA(UsdPhysics.Scene):
                stats = _physx.PhysicsSceneStats()
                entry = {'scene': str(prim.GetPath())}
                try:
                    entry['available'] = _physx.acquire_physx_statistics_interface().get_physx_scene_statistics(
                        omni.usd.get_context().get_stage_id(), PhysicsSchemaTools.sdfPathToInt(prim.GetPath()), stats)
                    if entry['available']:
                        entry['gpu_memory_bytes'] = {name: getattr(stats, name) for name in dir(stats) if name.startswith('gpu_mem_heap')}
                except Exception as error:
                    entry['error'] = repr(error)
                result.append(entry)
        return result

    physics['runtime_statistics'] = physics_statistics()
    (ROOT/'evaluations/physics-backend.json').write_text(json.dumps(physics, indent=2))
    if SETTINGS.get('gpu_trace'):
        assert device.startswith('cuda')
        with torch.profiler.profile(activities=[torch.profiler.ProfilerActivity.CPU, torch.profiler.ProfilerActivity.CUDA]) as trace:
            for _ in range(20):
                step()
            torch.cuda.synchronize()
        trace.export_chrome_trace(str(ROOT/'evaluations/gpu-physics-trace.json'))
    for env in environments:
        env.ready()
    query = omni.physx.get_physx_scene_query_interface()
    count = clock_count = clock_published = 0
    sim_start = SimulationManager.get_simulation_time()
    profiler = LoopProfiler(ROOT/'evaluations/loop-profile.jsonl',500)
    print('CLONED_ENVIRONMENTS_READY',flush=True)
    while app.is_running() and not (ROOT/'stop').exists():
        started = time.monotonic()
        profiler.begin()
        for env in environments:
            env.apply(started)
        profiler.mark('command_and_state')
        step()
        stamp_s = SimulationManager.get_simulation_time()
        for env in environments:
            env.publish(count,stamp_s)
        profiler.mark('environment_publication_and_sensors')
        count += 1
        sleep_s = max(0.,.002-(time.monotonic()-started))
        profiler.before_sleep(sleep_s)
        time.sleep(sleep_s)
        profiler.mark('pacing')
        profiler.finish()
    profiler.close()
    (ROOT/'evaluations/physics-statistics-final.json').write_text(json.dumps(physics_statistics(), indent=2))
    for env in environments:
        env.base.close()
        env.motion.close()
        rates = publication_config('split',500)
        (env.output/'evaluations/publication-counts.json').write_text(json.dumps({**rates,'physics_steps':count,
            'simulation_s':SimulationManager.get_simulation_time()-sim_start,'counts':{'clock':clock_published,**env.counts}},indent=2))
        env.node.destroy_node()
    Ros2ControlManager.teardown_all()
    clock_node.destroy_node()
    rclpy.shutdown()


try:
    main()
except BaseException:
    import traceback
    traceback.print_exc()
    raise
finally:
    app.close()
