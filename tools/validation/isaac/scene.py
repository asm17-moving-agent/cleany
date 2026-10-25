"""Isaac adapter: original articulated robot, ideal base actuator and PhysX LiDAR.

No Mission or navigation logic lives here. Root velocity commands are an explicit
ideal actuator approximation; they do not validate mecanum roller dynamics.
"""
from __future__ import annotations

import json
import math
import os
from pathlib import Path
import time

import numpy as np
from cadence import measured_twist, publication_config
from isaacsim import SimulationApp

OUTPUT = Path("/output")
CONFIG = json.loads((OUTPUT / "config/room.json").read_text())
app = SimulationApp({"headless": True, "hide_ui": True, "renderer": "RaytracedLighting" if CONFIG.get("cleanup") else "MinimalRendering",
                     "limit_cpu_threads": 4, "disable_viewport_updates": True,
                     "extra_args": ["--/app/profilerBackend=cpu", "--/app/profilerMask=0"]
                     if os.environ.get("CLEANY_NATIVE_PROFILE") == "1" else []})

import isaacsim.core.experimental.utils.app as app_utils
import omni.physx
import omni.timeline
import omni.usd
from isaacsim.core.simulation_manager import SimulationManager
from pxr import Gf, UsdGeom, UsdPhysics, UsdShade, PhysxSchema, Sdf


def main() -> None:
    for extension in ("isaacsim.asset.importer.urdf", "isaacsim.ros2.core",
                      "isaacsim.ros2.bridge", "isaacsim.ros2.control"):
        app_utils.enable_extension(extension)
    app.update()
    from isaacsim.asset.importer.urdf import URDFImporter, URDFImporterConfig
    from isaacsim.ros2.control import Ros2ControlManager
    from isaacsim.ros2.control.bindings import _isaacsim_ros2_control as backend
    import rclpy
    from builtin_interfaces.msg import Time, Duration
    from controller_manager_msgs.srv import ConfigureController, LoadController, SwitchController
    from geometry_msgs.msg import TransformStamped, Twist
    from nav_msgs.msg import Odometry
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import Bool
    from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster

    from scene_assets import build_stage
    stage = build_stage(OUTPUT, CONFIG)
    SimulationManager.setup_simulation(dt=1./CONFIG["physics_hz"], device="cpu")
    cleanup_physics = CONFIG.get("cleanup", {}).get("physics", {})
    if cleanup_physics.get("solver"):
        SimulationManager.set_solver_type(cleanup_physics["solver"])
    # Headless consumers read live PhysX state. Fabric avoids 500Hz USD writes;
    # recorded link poses drive the separate offline renderer.
    if CONFIG.get("physics_sync", "usd") == "fabric":
        SimulationManager.enable_fabric(True)
    timeline = omni.timeline.get_timeline_interface()
    rclpy.init()
    node = rclpy.create_node("cleany_isaac_adapter")
    camera_control = None
    if CONFIG.get("cleanup"):
        from cleanup_camera_control import create_camera_control
        camera_control = create_camera_control(rclpy)
    clock_pub = node.create_publisher(Clock, "/clock", 10)
    profiler = None
    native = None
    rates = publication_config(CONFIG.get("publication_profile", "legacy"), CONFIG["physics_hz"])
    clock_stride = CONFIG["physics_hz"] // rates["clock_hz"]
    odometry_stride = CONFIG["physics_hz"] // rates["odometry_hz"]
    scan_stride = CONFIG["physics_hz"] // rates["scan_hz"]
    clock_count = 0
    publication_counts = {"clock": 0, "odometry": 0, "scan": 0}

    def step():
        nonlocal clock_count
        # Manual physics stepping avoids tying ROS/sensor cadence to viewport rendering.
        if native:
            native.begin_step()
        SimulationManager.step()
        if native:
            native.end_step()
        if profiler:
            profiler.mark("physics_step")
        seconds = SimulationManager.get_simulation_time()
        if clock_count % clock_stride == 0:
            clock_pub.publish(Clock(clock=Time(sec=int(seconds), nanosec=int(seconds % 1*1e9))))
            publication_counts["clock"] += 1
        clock_count += 1
        if profiler:
            profiler.mark("clock_publish")
    command = {"value": np.zeros(3), "received": -math.inf, "blocked": False}

    def on_command(message):
        values = np.array([message.linear.x, message.linear.y, message.angular.z])
        command.update(value=values if np.isfinite(values).all() else np.zeros(3), received=time.monotonic())

    node.create_subscription(Twist, "/cmd_vel", on_command, 10)
    node.create_subscription(Bool, "/isaac/test/block_base",
                             lambda msg: command.update(blocked=msg.data), 10)
    odom_pub = node.create_publisher(Odometry, "/wheel/odom", 10)
    scan_pub = node.create_publisher(LaserScan, "/scan", 10)
    tf = TransformBroadcaster(node)
    static = StaticTransformBroadcaster(node)
    transform = TransformStamped()
    transform.header.frame_id, transform.child_frame_id = "base_link", "isaac_lidar"
    transform.transform.translation.x = CONFIG.get("scan_x", 0.)
    transform.transform.translation.z = CONFIG["scan_z"] - CONFIG["spawn"][2]
    transform.transform.rotation.w = 1.
    static.sendTransform(transform)

    def wait(predicate, timeout=180.):
        deadline = time.monotonic()+timeout
        while not predicate():
            if time.monotonic() > deadline:
                raise TimeoutError("Isaac ROS control setup timeout")
            step()
            rclpy.spin_once(node, timeout_sec=0)

    def call(kind, endpoint, request):
        print(f"CONTROLLER_REQUEST {endpoint} {request}", flush=True)
        client = node.create_client(kind, endpoint)
        wait(client.service_is_ready)
        future = client.call_async(request)
        wait(future.done)
        result = future.result()
        node.destroy_client(client)
        if not result.ok:
            raise RuntimeError(f"Controller request failed: {endpoint}")
        print(f"CONTROLLER_OK {endpoint}", flush=True)

    timeline.play()
    app.update()
    SimulationManager.initialize_physics()
    wait(backend.is_ready)
    if CONFIG.get("park_joints"):
        initial_view = SimulationManager.get_physics_sim_view().create_articulation_view("/cleany/Geometry/base_link")
        positions = initial_view.get_dof_positions().copy()
        for i, name in enumerate(initial_view.shared_metatype.dof_names):
            positions[0, i] = CONFIG["park_joints"][name]
        initial_view.set_dof_positions(positions, np.array([0], dtype=np.int32))
        initial_view.set_dof_position_targets(positions, np.array([0], dtype=np.int32))
    timeline.pause()
    app.update()
    Ros2ControlManager.setup("/cleany", str(OUTPUT / "config/controllers.yaml"),
                            publish_robot_description=False, use_sim_time=True)
    timeline.play()
    app.update()
    names = ["joint_state_broadcaster", "left_arm_controller", "right_arm_controller",
             "left_gripper_controller", "right_gripper_controller"]
    for name in names:
        call(LoadController, "/controller_manager/load_controller", LoadController.Request(name=name))
        call(ConfigureController, "/controller_manager/configure_controller", ConfigureController.Request(name=name))
    call(SwitchController, "/controller_manager/switch_controller",
         SwitchController.Request(activate_controllers=names, strictness=2,
                                  timeout=Duration(sec=30 if CONFIG.get("cleanup") else 1)))
    physics = SimulationManager.get_physics_sim_view()
    view = physics.create_articulation_view("/cleany/Geometry/base_link")
    if view.count != 1:
        raise RuntimeError(f"Expected one robot, got {view.count}")
    pose = view.get_root_transforms().copy()
    pose[0, :3] = CONFIG["spawn"]
    pose[0, 3:] = [0., 0., math.sin(CONFIG.get("spawn_yaw", 0.)/2), math.cos(CONFIG.get("spawn_yaw", 0.)/2)]
    view.set_root_transforms(pose, np.array([0], dtype=np.int32))
    camera = evidence = grasp_audit = None
    if CONFIG.get("cleanup"):
        from cleanup_camera import CleanupCamera
        camera = CleanupCamera(stage, node, OUTPUT)
        from cleanup_assets import PlacementEvidence
        evidence = PlacementEvidence(physics, node, CONFIG)
        from grasp_physics_audit import GraspPhysicsAudit
        grasp_audit = GraspPhysicsAudit(stage, physics, OUTPUT, CONFIG['physics_hz'])
    query = omni.physx.get_physx_scene_query_interface()
    motion = None
    if os.environ.get("CLEANY_RECORD_MOTION") == "1":
        media = OUTPUT / "media"
        media.mkdir(exist_ok=True)
        paths = []
        for name in view.shared_metatype.link_names:
            matches = [str(prim.GetPath()) for prim in stage.Traverse()
                       if prim.GetName() == name and str(prim.GetPath()).startswith("/cleany/Geometry/")]
            if len(matches) != 1:
                raise RuntimeError(f"Cannot identify PhysX link {name}: {matches}")
            paths.append(matches[0])
        stage.Export(str(media / "scene.usda"))
        (media / "links.json").write_text(json.dumps(paths))
        motion = (media / "motion.jsonl").open("w", buffering=1)
    readiness = {
        "controllers": names, "actuation": "ideal root velocity", "lidar": "PhysX raycast",
        "physical_cleanup_enabled": bool(CONFIG.get("cleanup")),
        "physical_cleanup_verified": False,
        "camera": "RTX RGB-D" if camera else "not yet connected"}
    ready = False
    if camera:
        import faulthandler
        faulthandler.dump_traceback_later(60., repeat=True)
        print("ISAAC_WAITING_FOR_FIRST_SENSOR_FRAME", flush=True)
    count = 0
    last_publish = -1.
    clock_count = 0
    publication_counts = {"clock": 0, "odometry": 0, "scan": 0}
    odometry_pose = view.get_root_transforms()[0].copy()
    odometry_time = loop_sim_start = SimulationManager.get_simulation_time()
    if os.environ.get("CLEANY_PROFILE") == "1":
        from profiling import LoopProfiler
        profiler = LoopProfiler(OUTPUT / "evaluations/loop-profile.jsonl", CONFIG["physics_hz"])
    if os.environ.get("CLEANY_NATIVE_PROFILE") == "1":
        from native_profile import NativeProfiler
        native = NativeProfiler(OUTPUT / "evaluations")
    with (OUTPUT / "evaluations/base.jsonl").open("w", buffering=1) as log:
        while app.is_running() and not (OUTPUT / "stop").exists():
            started = time.monotonic()
            if profiler:
                profiler.begin()
            rclpy.spin_once(node, timeout_sec=0)
            pose = view.get_root_transforms()[0].copy()
            velocity = view.get_root_velocities()[0].copy()
            x, y, z, qx, qy, qz, qw = map(float, pose)
            yaw = math.atan2(2*(qw*qz+qx*qy), 1-2*(qy*qy+qz*qz))
            requested = command["value"] if (started-command["received"] < CONFIG["command_timeout_s"]
                                             and not command["blocked"]) else np.zeros(3)
            if native:
                native.tick(requested)
            vx, vy, wz = np.clip(requested, [-.25, -.25, -.5], [.25, .25, .5])
            velocity[[0, 1, 3, 4, 5]] = [math.cos(yaw)*vx-math.sin(yaw)*vy,
                                        math.sin(yaw)*vx+math.cos(yaw)*vy, 0., 0., wz]
            view.set_root_velocities(velocity.reshape(1, 6), np.array([0], dtype=np.int32))
            if profiler:
                profiler.mark("command_and_state")
            step()
            if camera_control:
                rclpy.spin_once(camera_control, timeout_sec=0)
            camera_rate = (10 if camera_control and any(camera_control.get_parameter(p).value
                for p in ("head_depth_boost", "head_high_rate")) else 5)
            if camera and count % (CONFIG["physics_hz"] // camera_rate) == 0:
                from omni.physxfabric import get_physx_fabric_interface
                before_render_state = grasp_audit.snapshot()
                get_physx_fabric_interface().update(
                    SimulationManager.get_simulation_time(), 1./CONFIG["physics_hz"])
                before_capture = SimulationManager.get_simulation_time()
                camera.capture(before_capture, camera_control.get_parameter('active_camera').value)
                after_capture = SimulationManager.get_simulation_time()
                grasp_audit.record_render_delta(before_render_state, after_capture)
                if abs(after_capture-before_capture) > 1e-6:
                    raise RuntimeError("Camera rendering unexpectedly advanced physics")
                evidence.publish(view.get_root_transforms()[0], after_capture)
                grasp_audit.record(after_capture)
            stamp_s = SimulationManager.get_simulation_time()
            if stamp_s <= last_publish:
                continue
            last_publish = stamp_s
            if count % odometry_stride == 0:
                pose = view.get_root_transforms()[0].copy()
                stamp = Time(sec=int(stamp_s), nanosec=int((stamp_s % 1)*1e9))
                message = Odometry()
                message.header.stamp, message.header.frame_id, message.child_frame_id = stamp, "odom", "base_link"
                message.pose.pose.position.x, message.pose.pose.position.y, message.pose.pose.position.z = map(float, pose[:3])
                rotation = message.pose.pose.orientation
                rotation.x, rotation.y, rotation.z, rotation.w = map(float, pose[3:])
                vx_measured, vy_measured, wz_measured = measured_twist(odometry_pose, pose, stamp_s-odometry_time)
                message.twist.twist.linear.x = float(vx_measured)
                message.twist.twist.linear.y = float(vy_measured)
                message.twist.twist.angular.z = float(wz_measured)
                odometry_pose, odometry_time = pose.copy(), stamp_s
                odom_pub.publish(message)
                transform = TransformStamped()
                transform.header, transform.child_frame_id = message.header, "base_link"
                transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z = map(float, pose[:3])
                transform.transform.rotation = rotation
                tf.sendTransform(transform)
                publication_counts["odometry"] += 1
            if profiler:
                profiler.mark("odometry_tf")
            if count % scan_stride == 0:
                velocity = view.get_root_velocities()[0].copy()
                if motion:
                    motion.write(json.dumps({"sim_time": stamp_s, "wall_time": time.time(),
                        "links": view.get_link_transforms()[0].tolist()})+"\n")
                if profiler:
                    profiler.mark("motion_recording")
                scan = LaserScan()
                scan.header.stamp, scan.header.frame_id = stamp, "isaac_lidar"
                scan.angle_min, scan.angle_max, scan.angle_increment = -math.pi, math.pi, math.pi/180
                scan.range_min, scan.range_max, scan.scan_time = .15, 12., .1
                ranges = []
                for angle in np.linspace(-math.pi, math.pi, 361):
                    direction = (math.cos(yaw+angle), math.sin(yaw+angle), 0.)
                    hits = []
                    def hit_callback(hit):
                        if not str(hit.rigid_body).startswith("/cleany"):
                            hits.append(float(hit.distance))
                        return True
                    query.raycast_all((float(pose[0])+math.cos(yaw)*CONFIG.get("scan_x", 0.),
                                       float(pose[1])+math.sin(yaw)*CONFIG.get("scan_x", 0.),
                                       float(pose[2]) + CONFIG["scan_z"]-CONFIG["spawn"][2]),
                                      direction, scan.range_max, hit_callback)
                    ranges.append(min(hits) if hits else math.inf)
                scan.ranges = ranges
                if profiler:
                    profiler.mark("lidar_raycast")
                scan_pub.publish(scan)
                publication_counts["scan"] += 1
                log.write(json.dumps({"time": stamp_s, "pose": pose.tolist(), "velocity": velocity.tolist(),
                                      "requested": requested.tolist(), "blocked": command["blocked"]})+"\n")
                if profiler:
                    profiler.mark("scan_publish_base_recording")
            if (not ready and all(publication_counts.values())
                    and (camera is None or camera.count > 0)):
                (OUTPUT / "evaluations/adapter-ready.json").write_text(json.dumps(
                    {**readiness, "first_published_simulation_s": stamp_s}, indent=2))
                ready = True
                if camera:
                    faulthandler.cancel_dump_traceback_later()
                print("ISAAC_ADAPTER_READY", flush=True)
            count += 1
            sleep_s = max(0., 1./CONFIG["physics_hz"]-(time.monotonic()-started))
            if profiler:
                profiler.before_sleep(sleep_s)
            time.sleep(sleep_s)
            if profiler:
                profiler.mark("pacing")
                profiler.finish()
    (OUTPUT / "evaluations/publication-counts.json").write_text(json.dumps({
        **rates, "physics_steps": count, "simulation_s": SimulationManager.get_simulation_time()-loop_sim_start,
        "counts": publication_counts}, indent=2)+"\n")
    if profiler:
        profiler.close()
    if native:
        native.close()
    if motion:
        motion.close()
    node.destroy_node()
    rclpy.shutdown()


try:
    main()
except BaseException:
    import traceback
    traceback.print_exc()
    raise
finally:
    app.close()
