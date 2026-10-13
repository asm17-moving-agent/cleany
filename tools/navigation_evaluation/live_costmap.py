#!/usr/bin/env python3
"""Keep a live Gazebo costmap demo open until its RViz closes."""

from __future__ import annotations

import argparse
import json
import math
import os
import re
import shutil
from pathlib import Path
import subprocess
import time
import uuid
import yaml

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile
from visualization_msgs.msg import Marker, MarkerArray

from run_safety_evaluation import prepare, stop
from safety_probe import SafetyProbe, yaw as pose_yaw
from rclpy.time import Time
from std_srvs.srv import SetBool
from tf2_ros import TransformException


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--map', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--domain-id', type=int, default=207)
    parser.add_argument('--hold-seconds', type=float, default=6.0)
    parser.add_argument('--drive', action='store_true', help='Run the configured Nav2 avoidance route once')
    parser.add_argument('--full-map-route', action='store_true', help='Follow the original study-cafe survey route in simulation, without test fixtures')
    parser.add_argument('--route-end-inset', type=float, default=0.0, help='Move the survey outer X endpoints inward by this many meters for turning clearance')
    parser.add_argument('--survey-controller', choices=('ground_truth',), default='ground_truth', help='Survey waypoint controller; ground_truth is simulation-only')
    parser.add_argument('--max-survey-position-error', type=float, default=0.35)
    parser.add_argument('--max-survey-yaw-error-deg', type=float, default=25.0)
    parser.add_argument('--no-test-obstacle', action='store_true', help='Drive without injecting the evaluation obstacle')
    parser.add_argument('--robot-model', choices=('legacy', 'cad_frame'), default='cad_frame')
    parser.add_argument('--drive-delay', type=float, default=0.0, help='Wall seconds to prepare viewers before driving')
    parser.add_argument('--idle', action='store_true', help='Keep stationary without scripted fixtures')
    parser.add_argument('--initial-pose', type=float, nargs=3, metavar=('X', 'Y', 'YAW'), help='Initial saved-map pose')
    parser.add_argument('--restore-memory', type=Path, help='Restore a compatible obstacle-memory NPZ snapshot')
    parser.add_argument('--guard-3d', action='store_true', help='Add observed-voxel 3D velocity gate after Collision Monitor')
    args = parser.parse_args()
    if not all(math.isfinite(v) and v > 0 for v in (args.max_survey_position_error, args.max_survey_yaw_error_deg)):
        parser.error('Survey localization error limits must be finite and positive')
    if not math.isfinite(args.route_end_inset) or not 0 <= args.route_end_inset < 1.0:
        parser.error('--route-end-inset must be finite and in [0, 1) meters')
    if args.full_map_route:
        args.drive, args.no_test_obstacle = True, True
        if args.initial_pose is not None:
            parser.error('--full-map-route starts at the original spawn; omit --initial-pose')
    if args.no_test_obstacle and not args.drive:
        parser.error('--no-test-obstacle requires --drive')
    if args.hold_seconds <= 0:
        parser.error('--hold-seconds must be positive')
    if args.drive_delay < 0:
        parser.error('--drive-delay must be nonnegative')
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    os.environ['ROS_DOMAIN_ID'] = str(args.domain_id)
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    os.environ['GZ_PARTITION'] = 'cleany_live_'+uuid.uuid4().hex[:12]
    scenario = prepare(output, args.map.resolve(), robot_model=args.robot_model, initial_pose=args.initial_pose,
                       obstacle_memory=True, memory_restore=args.restore_memory, guard_3d=args.guard_3d)
    scenario['injected_obstacle'] = args.drive and not args.no_test_obstacle
    scenario['survey_localization_guard'] = dict(position_error_m=args.max_survey_position_error,
        yaw_error_deg=args.max_survey_yaw_error_deg, persistence_sim_s=1.0,
        on_failure='Stop follower and freeze obstacle memory')
    route_goals = None
    if args.full_map_route:
        route_source = Path(get_package_share_directory('cleany_gazebo_sim'))/'config/study_cafe/study_cafe_route.yaml'
        shutil.copy2(route_source, output/'config/study_cafe_route.yaml')
        values = yaml.safe_load(route_source.read_text())['ground_truth_route_follower']['ros__parameters']['waypoints_xy']
        outer_x = max(abs(x) for x in values[::2])
        values = [value-math.copysign(args.route_end_inset, value) if i%2==0 and abs(value)==outer_x else value
                  for i, value in enumerate(values)]
        sx, sy, _, _, _, yaw = scenario['spawn_pose']
        c, s = math.cos(yaw), math.sin(yaw)
        points = [(c*(x-sx)+s*(y-sy), -s*(x-sx)+c*(y-sy)) for x, y in zip(values[::2], values[1::2])]
        route_goals = [[x, y, math.atan2(y-py, x-px)] for (px, py), (x, y) in zip(points, points[1:])]
        scenario['full_map_route'] = {'source': str(route_source), 'goals_map': route_goals,
                                     'controller': args.survey_controller,
                                     'end_inset_m': args.route_end_inset,
                                     'length_m': sum(math.hypot(x-px, y-py) for (px, py), (x, y) in zip(points, points[1:]))}
    (output/'scenario.json').write_text(json.dumps(scenario, indent=2))
    share = Path(get_package_share_directory('cleany_gazebo_sim'))
    launch = rviz = probe = follower = None
    (output/'session.json').write_text(json.dumps({
        'supervisor_pid': os.getpid(), 'ros_domain_id': args.domain_id,
        'gz_partition': os.environ['GZ_PARTITION'],
        'started': time.strftime('%Y-%m-%dT%H:%M:%S%z'),
        'stop': 'Close the RViz window or send SIGINT to supervisor_pid',
        'mode': 'Full-map ground-truth survey with Collision Monitor' if args.full_map_route else (('Nav2 route without test obstacle' if args.no_test_obstacle else 'Nav2 avoidance drive') if args.drive else ('Stationary viewer' if args.idle else 'Stationary sensor demo')),
    }, indent=2))
    try:
        with (output/'launch.log').open('w') as launch_log, (output/'rviz.log').open('w') as rviz_log:
            launch = subprocess.Popen(
                ['ros2', 'launch', str(output/'run.launch.py')],
                stdout=launch_log, stderr=subprocess.STDOUT, start_new_session=True)
            rclpy.init()
            probe = SafetyProbe(output, scenario)
            probe.initialize()
            labels = probe.create_publisher(MarkerArray, '/evaluation/live_labels',
                                           QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
            rviz_env = {**os.environ, 'QT_QPA_PLATFORM': 'xcb'}
            rviz = subprocess.Popen(
                ['rviz2', '-d', str(share/'config/rviz/navigation_costmap.rviz'),
                 '--ros-args', '-p', 'use_sim_time:=true'],
                stdout=rviz_log, stderr=subprocess.STDOUT, env=rviz_env, start_new_session=True)

            def show(text: str) -> None:
                title = Marker()
                title.header.frame_id = 'map'
                title.ns, title.id, title.type = 'live_demo', 0, Marker.TEXT_VIEW_FACING
                title.pose.orientation.w = 1.0
                title.pose.position = Point(x=2.0, y=2.25, z=0.1)
                title.scale.z = 0.13
                title.color.r, title.color.g, title.color.b, title.color.a = 0.05, 0.15, 0.3, 1.0
                title.text = text
                markers = [title]
                if route_goals:
                    route = Marker()
                    route.header.frame_id = 'map'
                    route.ns, route.id, route.type = 'survey_route', 0, Marker.LINE_STRIP
                    route.pose.orientation.w = 1.0
                    route.scale.x = 0.025
                    route.color.r, route.color.g, route.color.b, route.color.a = 1.0, 0.55, 0.0, 0.7
                    route.points = [Point(x=float(x), y=float(y), z=0.05) for x, y in points]
                    markers.append(route)
                labels.publish(MarkerArray(markers=markers))

            def hold() -> None:
                deadline = time.monotonic()+args.hold_seconds
                while time.monotonic() < deadline:
                    if rviz.poll() is not None or launch.poll() is not None:
                        raise InterruptedError('Viewer or simulation closed')
                    probe.tick()

            if args.drive:
                show('READY')
                deadline = time.monotonic()+args.drive_delay
                while time.monotonic() < deadline:
                    if rviz.poll() is not None or launch.poll() is not None:
                        raise InterruptedError('Viewer or simulation closed')
                    probe.tick()
                if route_goals and args.survey_controller == 'ground_truth':
                    route_parameters = yaml.safe_load(route_source.read_text())
                    route_parameters['ground_truth_route_follower']['ros__parameters'].update(
                        use_sim_time=True, waypoints_xy=[float(v) for v in values])
                    route_file = output/'config/survey_follower.yaml'
                    route_file.write_text(yaml.safe_dump(route_parameters))
                    follower_log = output/'survey_follower.log'
                    with follower_log.open('w') as stream:
                        follower = subprocess.Popen(['ros2', 'run', 'cleany_gazebo_sim', 'ground_truth_route_follower',
                            '--ros-args', '--params-file', str(route_file), '-r', 'cmd_vel:=/nav2/cmd_vel'],
                            stdout=stream, stderr=subprocess.STDOUT, start_new_session=True)
                    show('SURVEY STARTING')
                    deadline = time.monotonic()+1800
                    last_pose, last_movement = probe.truth(), probe.now()
                    previous_status = ''
                    localization_failed_since = None
                    localization_error = None
                    while True:
                        if rviz.poll() is not None or launch.poll() is not None or follower.poll() is not None:
                            raise InterruptedError('Survey process closed')
                        probe.tick()
                        log_text = follower_log.read_text()
                        matches = re.findall(r'Following waypoint (\d+)/(\d+)', log_text)
                        status = 'SURVEY '+('/'.join(matches[-1]) if matches else 'STARTING')
                        if status != previous_status:
                            show(status); probe.event('survey_waypoint', progress=matches[-1:] or [])
                            probe.save(); previous_status = status
                        current = probe.truth()
                        try:
                            pose = probe.buffer.lookup_transform('map', 'base_link', Time()).transform
                            position_error = math.hypot(current[0]-pose.translation.x, current[1]-pose.translation.y)
                            yaw_error = abs(math.degrees(math.atan2(math.sin(current[2]-pose_yaw(pose.rotation)), math.cos(current[2]-pose_yaw(pose.rotation)))))
                            localization_error = dict(position_m=position_error, yaw_deg=yaw_error)
                            bad_localization = position_error > args.max_survey_position_error or yaw_error > args.max_survey_yaw_error_deg
                        except TransformException:
                            bad_localization = True
                            localization_error = dict(reason='Missing map to base transform')
                        if bad_localization:
                            if localization_failed_since is None:
                                localization_failed_since = probe.now()
                        else:
                            localization_failed_since = None
                        localization_lost = localization_failed_since is not None and probe.now()-localization_failed_since >= 1.0
                        if math.hypot(current[0]-last_pose[0], current[1]-last_pose[1]) > 0.02 or abs(math.atan2(math.sin(current[2]-last_pose[2]),math.cos(current[2]-last_pose[2]))) > 0.03:
                            last_pose, last_movement = current, probe.now()
                        completed = 'Study-cafe evaluation route completed' in log_text
                        stalled = probe.now()-last_movement > 30 or time.monotonic()>deadline
                        if completed or stalled or localization_lost:
                            stop(follower); follower = None
                            if stalled or localization_lost:
                                probe.service(SetBool, '/obstacle_memory/set_enabled', SetBool.Request(data=False))
                            result = dict(controller='ground_truth', completed=completed, stalled=stalled,
                                          localization_lost=localization_lost, localization_error=localization_error,
                                          final_truth=current, progress=matches[-1:] or [])
                            (output/'drive_result.json').write_text(json.dumps(result, indent=2));probe.save()
                            shutil.copy2(output/'samples.csv', output/'drive_samples.csv')
                            show('SURVEY COMPLETE' if completed else 'SURVEY STOPPED')
                            break
                    while True:
                        hold(); probe.rows = probe.rows[-2000:]
                show('DRIVING')
                result = probe.run_avoid(keep_running=lambda: rviz.poll() is None and launch.poll() is None,
                                         inject_obstacle=not args.no_test_obstacle)
                (output/'drive_result.json').write_text(json.dumps(result, indent=2))
                probe.save()
                shutil.copy2(output/'samples.csv', output/'drive_samples.csv')
                show('ARRIVED' if result['checks']['nav2_succeeded'] else 'STOPPED')
                while True:
                    hold()
                    probe.rows = probe.rows[-2000:]

            if args.idle:
                show('READY')
                while True:
                    hold()
                    probe.save()
                    probe.rows = probe.rows[-2000:]
            show('CLEAR')
            hold()
            while True:
                for case in scenario['evaluation']['sensor_cases'][:2]:
                    name, center = case['fixture'], tuple(case['center'])
                    probe.move_fixture(name, *center)
                    probe.event('live_'+name)
                    show('LIDAR + DEPTH' if name=='tall' else 'DEPTH ONLY')
                    hold()
                    probe.sensor_snapshot(name+'_present', center)
                    probe.move_fixture(name, 50.0, 50.0)
                    probe.event('live_clear')
                    show('CLEARING')
                    hold()
                    probe.sensor_snapshot(name+'_removed', center)
                    probe.save()
                    # Keep the visible session bounded in memory; CSV is a rolling sample.
                    probe.rows = probe.rows[-2000:]
                    probe.events = probe.events[-100:]
    except (KeyboardInterrupt, InterruptedError):
        pass
    finally:
        stop(follower)
        if probe is not None:
            probe.save()
            if probe.active_goal is not None:
                try:
                    probe.finish()
                except Exception as exc:
                    print(f'Goal cleanup: {exc}', flush=True)
            probe.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        try:
            stop(rviz)
        finally:
            stop(launch)


if __name__ == '__main__':
    main()
