"""Mission/Nav2 with explicit mock or sensor-driven Isaac cleanup configuration."""
import os
import json
from pathlib import Path

import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap

from cleany_navigation.launch_runtime import nav2_nodes
from cleany_navigation.runtime_config import adapt_nav2_parameters, merge_parameters, set_sim_time, resolve_default_lattice
from cleany_mission_manager.adapters.config import load_targets


def start(context):
    output = Path(LaunchConfiguration("output").perform(context))
    source = Path(LaunchConfiguration("source").perform(context)) / "ros2_ws/src"
    scenario = LaunchConfiguration("scenario").perform(context)
    targets = load_targets(str(output / "config/targets.yaml"))
    scene = json.loads((output / "config/room.json").read_text())
    if targets.map_id != scene["map_id"]:
        raise ValueError("Isaac world map and Mission targets must match")
    config = source / "cleany_navigation/config"
    parameters = yaml.safe_load((config / "nav2_amcl.yaml").read_text())
    safety = yaml.safe_load((config / "nav2_safety.yaml").read_text())
    omni = yaml.safe_load((config / "nav2_omni_motion.yaml").read_text())
    # Match the existing navigation launch's complete profile composition.
    merge_parameters(parameters, safety)
    merge_parameters(parameters, omni)
    tuning_file = Path(__file__).parent / "profiles/mppi-lattice-r10cm-benchmark.original.yaml"
    tuning = yaml.safe_load(tuning_file.read_text())["controller_server"]["ros__parameters"]["FollowPath"]
    # Reuse recovered optimizer tuning; preserve the current controller identity,
    # speed limits and the separate Mission/goal checker/safety contracts.
    for key in ("plugin", "wrapped_plugin", "benchmark_timing_file", "vx_max", "vx_min", "vy_max", "wz_max"):
        tuning.pop(key, None)
    merge_parameters(parameters["controller_server"]["ros__parameters"]["FollowPath"], tuning)
    if scene["world"] == "facility18":
        merge_parameters(parameters, yaml.safe_load(
            (Path(__file__).parent / "profiles/facility18.yaml").read_text()))
    if scene.get('cleanup', {}).get('navigation_parameters'):
        merge_parameters(parameters, scene['cleanup']['navigation_parameters'])
    resolve_default_lattice(parameters, config / "lattice/omni-r10cm.json")
    # Navigation retains its validated LiDAR safety profile. Cleanup RGB-D
    # feeds manipulation only; depth-based navigation safety is not validated.
    parameters["collision_monitor"]["ros__parameters"]["observation_sources"] = ["scan"]
    local = parameters["local_costmap"]["local_costmap"]["ros__parameters"]
    local["plugins"].remove("depth_layer")
    del local["depth_layer"]
    for name in ("controller_server", "bt_navigator"):
        parameters[name]["ros__parameters"]["odom_topic"] = "/wheel/odom"
    parameters["amcl"]["ros__parameters"].update(set_initial_pose=True,
        initial_pose={"x": scene["spawn"][0], "y": scene["spawn"][1], "z": 0., "yaw": scene.get("spawn_yaw", 0.)})
    parameters = adapt_nav2_parameters(parameters, os.environ["ROS_DISTRO"])
    set_sim_time(parameters, True)
    nav_file = output / "config/navigation.yaml"
    nav_file.write_text(yaml.safe_dump(parameters))
    mission = source / "cleany_mission_manager"
    fixture = yaml.safe_load((mission / "config/manipulation_server_fixture.yaml").read_text())
    fixture_file = output / "config/manipulation_fixture.yaml"
    fixture_file.write_text(yaml.safe_dump(fixture))
    runtime = yaml.safe_load((mission / "config/runtime_gazebo_manipulation_mock.yaml").read_text())["mission_runtime"]["ros__parameters"]
    runtime.update(use_sim_time=True, targets_file=str(output / "config/targets.yaml"),
                   journal_path=str(output / "missions.db"), post_mission="return_home",
                   manipulation_mock_fixture=str(fixture_file))
    runtime["navigation_timeout"] = scene.get("navigation_timeout_s", 180.)
    if scenario == "navigation_failure":
        runtime["navigation_timeout"] = 12.
    if scenario == "manipulation_failure":
        # The existing external fixture encodes successful-pick snapshots only.
        # Exercise a single failed pick and terminal reobservation, without
        # submitting a retry against an incompatible external snapshot.
        runtime["max_skill_retries"] = 0
    physical = bool(scene.get("cleanup"))
    if physical:
        if scenario != "success":
            raise ValueError("Mock fault scenarios cannot be used as physical manipulation evidence")
        runtime.update(manipulation_backend="action_isaac", manipulation_initial_mock_safe=False,
            manipulation_action="/sim/manipulation/execute_skill",
            manipulation_query="/sim/manipulation/get_execution",
            manipulation_drive_readiness="/sim/manipulation/ready_to_drive",
            manipulation_destination="trash_right", cleaning_timeout=2400., operation_timeout=1200.,
            cancel_timeout=30., max_skill_retries=0)
    autostart = os.environ.get('CLEANY_CONTROLLED_BRINGUP') != '1'
    nodes = [GroupAction([SetRemap(src="cmd_vel", dst="nav2/cmd_vel"),
                          *nav2_nodes(str(nav_file), str(output / scene["map_file"]), True, autostart)]),
        Node(package="nav2_collision_monitor", executable="collision_monitor", name="collision_monitor",
             parameters=[str(nav_file)], output="screen"),
        Node(package="nav2_lifecycle_manager", executable="lifecycle_manager", name="lifecycle_manager_safety",
             parameters=[{"use_sim_time": True, "autostart": autostart, "node_names": ["collision_monitor"]}]),
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             parameters=[{"use_sim_time": True, "robot_description": (output / "assets/cleany.urdf").read_text()}]),
        *([] if physical else [Node(package="cleany_skill_executor", executable="manipulation_server", namespace="mock", output="screen",
             parameters=[{"backend": "mock", "mock_config": str(fixture_file),
                          "scenario": "grasp_failure" if scenario == "manipulation_failure" else "success",
                          "database_path": str(output / "executions.db"), "use_sim_time": False}])]),
        Node(package="cleany_mission_manager", executable="mission_runtime", name="mission_runtime",
             parameters=[runtime], output="screen")]
    if physical:
        launch = source / "cleany_skill_executor/launch/study_cafe_nearest_grasp_demo.launch.py"
        verifier = yaml.safe_load((source / "cleany_mujoco_sim/config/placement_verifier.yaml").read_text())
        labels = ['cup','paper cup','disposable paper cup'] if scene['cleanup']['payload']['name'] == 'cup' else ['crumpled tissue']
        verifier.setdefault("simulation_placement_verifier", {}).setdefault("ros__parameters", {}).update(
            labels=labels, bodies=['isaac_'+scene['cleanup']['payload']['name']]*len(labels))
        verifier_file = output / "config/cleanup_verifier.yaml"
        verifier_file.write_text(yaml.safe_dump(verifier))
        gripper_park = {scene['park_joints'][f'{arm}_gripper_joint'] for arm in ('left','right')}
        if len(gripper_park) != 1:
            raise ValueError('Sorting return configuration requires a common gripper park position')
        parameters = {
            "placement_verifier_parameters": str(verifier_file),
            "external_simulator": "isaac", "start_simulator": "false", "use_sim_time": "true",
            "manipulation_bt": "true", "sorting_mode": "true", "plan_only": "false", "sensor_scene": "true",
            "start_perception": "false", "sorting_use_wrist_camera": "true",
            "visibility_sensor_link": "head_tilt_link",
            "depth_octomap_plugin": "cleany_scene_mapping/KnownGeometryOctomapUpdater",
            "grasp_approach_tilt_options": "[0.0, 16.0, 32.0, 48.0, 64.0]",
            "scene_cloud_pixel_stride": "4",
            "cartesian_orientation_weight_m": "0.002",
            "sorting_return_gripper_position_rad": str(next(iter(gripper_park))),
            "stowed_positions_json": json.dumps(scene["park_joints"]),
            "sorting_bins_config": str(output / "config/cleanup_bins.yaml"),
            "coordinator_parameters": str(source / "cleany_manipulation_bt/config/manipulation_mujoco.yaml"),
            "bt_database_path": str(output / "executions.db"), "bt_monitor_port": "1678",
            "sorting_artifact_directory": str(output / "evaluations/manipulation"),
            "use_rviz": "false", "use_image_view": "false",
            "velocity_scaling": "0.08", "acceleration_scaling": "0.08",
        }
        if scene['cleanup'].get('launch_parameters'):
            parameters.update(scene['cleanup']['launch_parameters'])
            parameters['coordinator_parameters'] = str(output/'config/coordinator.yaml')
        nodes.extend([
            IncludeLaunchDescription(PythonLaunchDescriptionSource(str(launch)), launch_arguments=parameters.items()),
            ExecuteProcess(cmd=["python3", str(Path(__file__).parent / "local_perception.py")], output="screen"),
        ])
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("output", default_value="/output"),
        DeclareLaunchArgument("source", default_value="/main"),
        DeclareLaunchArgument("scenario", default_value="success",
                              choices=["success", "navigation_failure", "manipulation_failure", "cancel"]),
        OpaqueFunction(function=start)])
