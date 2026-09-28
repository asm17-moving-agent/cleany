"""Live SCRUM-306 observer and explicitly separated simulation test stimuli."""

from __future__ import annotations

import csv
import ast
import json
import math
from pathlib import Path
import subprocess
import time
from collections.abc import Callable

import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter, parameter_value_to_python
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.time import Time
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import OccupancyGrid, Odometry
from nav2_msgs.action import NavigateToPose
from nav2_msgs.msg import CollisionMonitorState
from nav2_msgs.srv import ClearEntireCostmap
from rcl_interfaces.srv import GetParameters, SetParameters
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from std_srvs.srv import Empty
from tf2_ros import Buffer, TransformListener
import yaml

from cleany_gazebo_sim.navigation_safety_metrics import rectangle_clearance


def yaw(q) -> float:
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y*q.y + q.z*q.z))


def transform_points(points: np.ndarray, transform) -> np.ndarray:
    q = transform.rotation
    x, y, z, w = q.x, q.y, q.z, q.w
    rotation = np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
        [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)],
    ])
    t = transform.translation
    return points @ rotation.T + np.array([t.x, t.y, t.z])


class SafetyProbe(Node):
    def __init__(self, output: Path, scenario: dict):
        super().__init__('nav2_safety_evaluation', parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.output = output
        self.scenario = scenario
        self.evaluation = scenario['evaluation']
        self.thresholds = self.evaluation['thresholds']
        self.spawn = scenario['spawn_pose']
        self.latest: dict = {}
        self.counts: dict[str, int] = {}
        self.phase = 'startup'
        self.events: list[dict] = []
        self.rows: list[dict] = []
        self.points = np.empty((0, 3))
        self.point_header = None
        self.active_goal = None
        self.command = None
        self.command_velocity = 0.0
        self.last_tick = 0.0
        self.last_command = 0.0
        self.last_cloud_stamp = -1.0
        self.fixture_xy = None
        self.sampled_cloud = self.create_publisher(PointCloud2, '/evaluation/depth_points', qos_profile_sensor_data)
        for topic, kind in [('/scan', LaserScan), ('/camera/head/depth/points', PointCloud2),
                            ('/ground_truth/odom', Odometry), ('/amcl_pose', PoseWithCovarianceStamped),
                            ('/cmd_vel', Twist), ('/nav2/cmd_vel', Twist),
                            ('/collision_monitor_state', CollisionMonitorState)]:
            self.create_subscription(kind, topic, lambda msg, topic=topic: self.receive(topic, msg), qos_profile_sensor_data)
        for topic in ['/map', '/local_costmap/costmap', '/global_costmap/costmap']:
            self.create_subscription(OccupancyGrid, topic, lambda msg, topic=topic: self.receive(topic, msg),
                                     QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.initial = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.nav = ActionClient(self, NavigateToPose, '/navigate_to_pose')

    def receive(self, topic: str, msg) -> None:
        self.latest[topic] = msg
        self.counts[topic] = self.counts.get(topic, 0) + 1
        if topic != '/camera/head/depth/points':
            return
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if stamp - self.last_cloud_stamp < 0.15:
            return
        self.last_cloud_stamp = stamp
        fields = {f.name: f for f in msg.fields}
        assert all(fields[name].datatype == PointField.FLOAT32 for name in ('x', 'y', 'z'))
        endian = '>' if msg.is_bigendian else '<'
        dtype = np.dtype({'names': ['x', 'y', 'z'], 'formats': [endian+'f4']*3,
                          'offsets': [fields[n].offset for n in ('x', 'y', 'z')], 'itemsize': msg.point_step})
        array = np.ndarray((msg.height, msg.width), dtype=dtype, buffer=msg.data,
                           strides=(msg.row_step, msg.point_step))[::8, ::8]
        points = np.column_stack([array[n].ravel() for n in ('x', 'y', 'z')])
        self.points = points[np.isfinite(points).all(axis=1)]
        self.point_header = msg.header
        sampled = PointCloud2()
        sampled.header = msg.header
        sampled.height, sampled.width = 1, len(self.points)
        sampled.fields = [PointField(name=n, offset=i*4, datatype=PointField.FLOAT32, count=1)
                          for i, n in enumerate(('x', 'y', 'z'))]
        sampled.point_step = 12
        sampled.row_step = sampled.width * 12
        sampled.is_dense = True
        sampled.data = self.points.astype('<f4').tobytes()
        self.sampled_cloud.publish(sampled)

    def now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def truth(self) -> list[float]:
        p = self.latest['/ground_truth/odom'].pose.pose
        dx, dy = p.position.x-self.spawn[0], p.position.y-self.spawn[1]
        c, s = math.cos(self.spawn[5]), math.sin(self.spawn[5])
        return [c*dx+s*dy, -s*dx+c*dy, math.atan2(math.sin(yaw(p.orientation)-self.spawn[5]), math.cos(yaw(p.orientation)-self.spawn[5]))]

    def tick(self) -> None:
        rclpy.spin_once(self, timeout_sec=0.005)
        now = self.now()
        if self.command is not None and now-self.last_command >= 0.05:
            msg = Twist()
            msg.linear.x = self.command_velocity
            self.command.publish(msg)
            self.last_command = now
        if now-self.last_tick < 0.05 or '/ground_truth/odom' not in self.latest:
            return
        self.last_tick = now
        safe = self.latest.get('/cmd_vel', Twist())
        raw = self.latest.get('/nav2/cmd_vel', Twist())
        state = self.latest.get('/collision_monitor_state', CollisionMonitorState())
        gt = self.truth()
        center = self.scenario.get('robot_center', [0.0, 0.0])
        body_pose = [gt[0]+math.cos(gt[2])*center[0]-math.sin(gt[2])*center[1],
                     gt[1]+math.sin(gt[2])*center[0]+math.cos(gt[2])*center[1], gt[2]]
        tf = self.buffer.lookup_transform('map', 'base_link', Time()).transform if self.buffer.can_transform('map', 'base_link', Time()) else None
        self.rows.append(dict(sim_s=now, phase=self.phase, gt_x=gt[0], gt_y=gt[1], gt_yaw=gt[2],
                              tf_x=tf.translation.x if tf else float('nan'), tf_y=tf.translation.y if tf else float('nan'),
                              raw_vx=raw.linear.x, raw_vy=raw.linear.y, raw_wz=raw.angular.z,
                              safe_vx=safe.linear.x, safe_vy=safe.linear.y, safe_wz=safe.angular.z,
                              gt_vx=self.latest['/ground_truth/odom'].twist.twist.linear.x,
                              gt_vy=self.latest['/ground_truth/odom'].twist.twist.linear.y,
                              action=state.action_type, polygon=state.polygon_name,
                              fixture_x=self.fixture_xy[0] if self.fixture_xy else float('nan'),
                              fixture_y=self.fixture_xy[1] if self.fixture_xy else float('nan'),
                              clearance_m=rectangle_clearance(tuple(body_pose), tuple(self.scenario['robot_half_size']), self.fixture_xy, self.fixture_half_size) if self.fixture_xy else float('nan')))

    def wait(self, seconds: float) -> None:
        start = self.now()
        deadline = time.monotonic() + max(30, seconds * 8)
        while self.now()-start < seconds:
            self.tick()
            if time.monotonic() > deadline:
                raise TimeoutError('Simulation did not advance')

    def event(self, phase: str, **details) -> None:
        self.phase = phase
        event = {'phase': phase, 'sim_s': self.now(), **details}
        self.events.append(event)
        (self.output / 'events.json').write_text(json.dumps(self.events, indent=2))
        print(json.dumps(event), flush=True)

    def future(self, future, timeout: float = 30):
        deadline = time.monotonic() + timeout
        while not future.done():
            self.tick()
            if time.monotonic() > deadline:
                raise TimeoutError('ROS response timeout')
        return future.result()

    def service(self, kind, name: str, request):
        client = self.create_client(kind, name)
        try:
            if not client.wait_for_service(timeout_sec=20):
                raise TimeoutError(name)
            return self.future(client.call_async(request))
        finally:
            self.destroy_client(client)

    def parameters(self, node: str, names: list[str]) -> dict:
        response = self.service(GetParameters, node+'/get_parameters', GetParameters.Request(names=names))
        return dict(zip(names, [parameter_value_to_python(p) for p in response.values]))

    def initialize(self) -> None:
        deadline = time.monotonic() + 120
        required = ['/scan', '/camera/head/depth/points', '/ground_truth/odom', '/amcl_pose', '/local_costmap/costmap']
        while not all(k in self.latest for k in required):
            self.tick()
            if time.monotonic() > deadline:
                raise TimeoutError('Missing runtime data: ' + str(self.counts))
        initial = PoseWithCovarianceStamped()
        initial.header.frame_id = 'map'
        initial.header.stamp = self.get_clock().now().to_msg()
        x, y, initial_yaw = self.scenario.get('initial_pose', [0.0, 0.0, 0.0])
        initial.pose.pose.position.x = x
        initial.pose.pose.position.y = y
        initial.pose.pose.orientation.z = math.sin(initial_yaw/2)
        initial.pose.pose.orientation.w = math.cos(initial_yaw/2)
        initial.pose.covariance[0] = initial.pose.covariance[7] = 0.25**2
        initial.pose.covariance[35] = math.radians(15)**2
        self.initial.publish(initial)
        for _ in range(5):
            self.service(Empty, '/request_nomotion_update', Empty.Request())
            self.wait(1.0)
        names = ['map_server', 'amcl', 'controller_server', 'planner_server', 'behavior_server', 'bt_navigator',
                 'collision_monitor', 'local_costmap/local_costmap', 'global_costmap/global_costmap']
        lifecycle_deadline = time.monotonic()+30
        while True:
            states = {n: self.service(GetState, '/'+n+'/get_state', GetState.Request()).current_state.label for n in names}
            if all(s == 'active' for s in states.values()):
                break
            if time.monotonic() > lifecycle_deadline:
                raise TimeoutError(f'Lifecycle activation incomplete: {states}')
            self.wait(1.0)
        assert self.point_header.frame_id == 'head_camera_depth_frame', self.point_header
        publishers = [p.node_name for p in self.get_publishers_info_by_topic('/cmd_vel')]
        assert publishers == (['voxel_guard'] if self.scenario.get('guard_3d') else ['collision_monitor']), publishers
        if self.scenario.get('guard_3d'):
            upstream = [p.node_name for p in self.get_publishers_info_by_topic('/safety_2d/cmd_vel')]
            assert upstream == ['collision_monitor'], upstream
        params = {n: self.parameters('/'+n, ['odom_topic']) for n in ('controller_server', 'bt_navigator')}
        assert all(p['odom_topic'] == '/wheel/odom' for p in params.values()), params
        wheel_names = ['wheel_radius_m', 'wheelbase_m', 'wheel_separation_m']
        params['wheel_odometry'] = self.parameters('/wheel_odometry', wheel_names)
        expected_wheel = yaml.safe_load((self.output/'config/wheel_odometry.yaml').read_text())['wheel_odometry']['ros__parameters']
        assert params['wheel_odometry'] == {k: expected_wheel[k] for k in wheel_names}
        controller_plugin = self.parameters('/controller_server', ['FollowPath.plugin'])['FollowPath.plugin']
        controller_names = ['FollowPath.plugin']
        if controller_plugin == 'dwb_core::DWBLocalPlanner':
            controller_names += ['FollowPath.'+name for name in (
                'trajectory_generator_name', 'max_vel_x', 'max_vel_y', 'max_vel_theta',
                'axis_linear_stopped', 'axis_angular_stopped', 'axis_settle_s')]
        else:
            controller_names += ['FollowPath.'+name for name in (
                'time_steps', 'model_dt', 'vx_max', 'PathAlignCritic.cost_weight',
                'PathFollowCritic.cost_weight', 'GoalCritic.cost_weight')]
        params['controller_server'].update(self.parameters('/controller_server', controller_names))
        ideal = yaml.safe_load((self.output/'config/odometry_error_ideal.yaml').read_text())['simulated_odometry_error']['ros__parameters']
        params['simulated_odometry_error'] = self.parameters('/simulated_odometry_error', list(ideal))
        assert params['simulated_odometry_error'] == ideal, params['simulated_odometry_error']
        geometry = ['footprint', 'footprint_padding', 'inflation_layer.inflation_radius']
        height = ['obstacle_layer.min_obstacle_height', 'obstacle_layer.scan.min_obstacle_height']
        params['local_costmap'] = self.parameters('/local_costmap/local_costmap', geometry+height+['plugins', 'depth_layer.depth.topic', 'depth_layer.enabled'])
        params['global_costmap'] = self.parameters('/global_costmap/global_costmap', geometry+height)
        params['collision_monitor'] = self.parameters('/collision_monitor', ['cmd_vel_in_topic', 'cmd_vel_out_topic', 'observation_sources', 'source_timeout'])
        stop = self.parameters('/collision_monitor', ['StopZone.points'])['StopZone.points']
        model = self.scenario.get('model_metadata')
        if model and 'navigation_geometry' in model:
            expected = model['navigation_geometry']
            np.testing.assert_allclose(ast.literal_eval(stop), expected['stop'])
            for name in ('local_costmap', 'global_costmap'):
                np.testing.assert_allclose(ast.literal_eval(params[name]['footprint']), expected['stop'])
                assert params[name]['footprint_padding'] == expected['planning_padding']
        for costmap in ('local_costmap', 'global_costmap'):
            footprint = ast.literal_eval(params[costmap]['footprint'])
            for i in (0, 1):
                assert max(abs(p[i]) for p in footprint) >= max(abs(p[i]) for p in ast.literal_eval(stop))
            assert all(params[costmap][h] < -0.08 for h in height)
        cloud = self.latest['/camera/head/depth/points']
        report = dict(states=states, parameters=params, cmd_vel_publishers=publishers,
                      cloud_frame=cloud.header.frame_id, cloud_size=[cloud.width, cloud.height],
                      finite_sampled_points=len(self.points), truth=self.truth())
        (self.output/'runtime_verified.json').write_text(json.dumps(report, indent=2))
        self.event('runtime_verified', **report)

    def save(self) -> None:
        if self.rows:
            with (self.output/'samples.csv').open('w') as stream:
                writer = csv.DictWriter(stream, fieldnames=list(self.rows[0]))
                writer.writeheader()
                writer.writerows(self.rows)

    def move_fixture(self, name: str, x: float, y: float) -> None:
        c, s = math.cos(self.spawn[5]), math.sin(self.spawn[5])
        wx, wy = self.spawn[0]+c*x-s*y, self.spawn[1]+s*x+c*y
        z = self.scenario['fixtures'][name]['z']
        request = (f'name: "safety_{name}" position: {{x: {wx} y: {wy} z: {z}}} '
                   f'orientation: {{z: {math.sin(self.spawn[5]/2)} w: {math.cos(self.spawn[5]/2)}}}')
        command = ['gz', 'service', '-s', '/world/'+self.scenario['world']+'/set_pose',
                   '--reqtype', 'gz.msgs.Pose', '--reptype', 'gz.msgs.Boolean', '--timeout', '3000', '--req', request]
        process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        deadline = time.monotonic() + 5
        while process.poll() is None:
            self.tick()
            if time.monotonic() > deadline:
                process.kill()
                process.wait()
                raise TimeoutError('Gazebo fixture pose service')
        output = process.communicate()[0]
        assert process.returncode == 0 and 'data: true' in output, output
        self.fixture_xy = (x, y) if abs(x) < 20 else None
        self.fixture_half_size = tuple(s/2 for s in self.scenario['fixtures'][name]['size'][:2])
        self.events.append({'phase':self.phase, 'sim_s':self.now(), 'fixture':name, 'map_pose':[x, y, z]})
        (self.output/'events.json').write_text(json.dumps(self.events, indent=2))

    def sensor_snapshot(self, name: str, center: tuple[float, float]) -> dict:
        cloud_tf = self.buffer.lookup_transform('map', self.point_header.frame_id, Time.from_msg(self.point_header.stamp)).transform
        cloud = transform_points(self.points, cloud_tf)
        scan = self.latest['/scan']
        ranges = np.asarray(scan.ranges)
        angles = scan.angle_min + np.arange(len(ranges))*scan.angle_increment
        valid = np.isfinite(ranges) & (ranges >= scan.range_min) & (ranges <= scan.range_max)
        laser = np.column_stack((ranges[valid]*np.cos(angles[valid]), ranges[valid]*np.sin(angles[valid]), np.zeros(valid.sum())))
        scan_tf = self.buffer.lookup_transform('map', scan.header.frame_id, Time.from_msg(scan.header.stamp)).transform
        laser = transform_points(laser, scan_tf)
        costmap = self.latest['/local_costmap/costmap']
        info = costmap.info
        ys, xs = np.indices((info.height, info.width))
        cells = np.column_stack((info.origin.position.x+(xs.ravel()+0.5)*info.resolution,
                                 info.origin.position.y+(ys.ravel()+0.5)*info.resolution, np.zeros(xs.size)))
        grid_tf = self.buffer.lookup_transform('map', costmap.header.frame_id, Time()).transform
        cells = transform_points(cells, grid_tf)
        roi = np.max(np.abs(cells[:, :2]-np.array(center)), axis=1) < 0.3
        values = np.array(costmap.data)
        cloud_roi = np.max(np.abs(cloud[:, :2]-np.array(center)), axis=1) < 0.3
        laser_roi = np.max(np.abs(laser[:, :2]-np.array(center)), axis=1) < 0.3
        report = dict(sim_s=self.now(), depth_points=int(cloud_roi.sum()), scan_points=int(laser_roi.sum()),
                      lethal_costmap_cells=int(((values == 100) & roi).sum()),
                      max_cost_in_roi=int(values[roi].max()) if roi.any() else None,
                      cloud_frame=self.point_header.frame_id)
        np.savez_compressed(self.output/(name+'.npz'), cloud_map=cloud, scan_map=laser, costmap_points_map=cells,
                            costmap=values, dimensions=[info.height, info.width], center=center)
        self.event(name, **report)
        return report

    def run_sensors(self) -> dict:
        results = {}
        durations = self.evaluation['sensor_durations_s']
        lidar_center = tuple(next(case['center'] for case in self.evaluation['sensor_cases'] if case['fixture'] == 'tall'))
        # Establish LiDAR's contribution to the local costmap independently.
        def depth_enabled(enabled):
            response = self.service(SetParameters, '/local_costmap/local_costmap/set_parameters',
                                    SetParameters.Request(parameters=[Parameter('depth_layer.enabled', value=enabled).to_parameter_msg()]))
            assert all(r.successful for r in response.results), response
            self.service(ClearEntireCostmap, '/local_costmap/clear_entirely_local_costmap', ClearEntireCostmap.Request())
            assert self.parameters('/local_costmap/local_costmap', ['depth_layer.enabled'])['depth_layer.enabled'] == enabled
        depth_enabled(False)
        try:
            self.wait(durations['before'])
            before = self.sensor_snapshot('lidar_only_before', lidar_center)
            self.move_fixture('tall', *lidar_center)
            self.wait(durations['present'])
            present = self.sensor_snapshot('lidar_only_present', lidar_center)
            self.move_fixture('tall', 50.0, 50.0)
            self.wait(durations['removed'])
            removed = self.sensor_snapshot('lidar_only_removed', lidar_center)
            results['lidar_only'] = dict(before=before, present=present, removed=removed)
        finally:
            depth_enabled(True)
        for case in self.evaluation['sensor_cases']:
            fixture, center = case['fixture'], tuple(case['center'])
            self.event(fixture+'_baseline')
            self.wait(durations['before'])
            before = self.sensor_snapshot(fixture+'_before', center)
            self.move_fixture(fixture, *center)
            self.wait(durations['present'])
            during = self.sensor_snapshot(fixture+'_present', center)
            self.move_fixture(fixture, 50.0, 50.0)
            self.wait(durations['removed'])
            after = self.sensor_snapshot(fixture+'_removed', center)
            results[fixture] = {'before':before, 'present':during, 'removed':after}
        results['checks'] = {
            'lidar_only_marks_local_costmap': results['lidar_only']['present']['lethal_costmap_cells'] > results['lidar_only']['before']['lethal_costmap_cells'],
            'lidar_only_clears_removed': results['lidar_only']['removed']['lethal_costmap_cells'] == results['lidar_only']['before']['lethal_costmap_cells'],
            'lidar_marks_tall': results['tall']['present']['scan_points'] >= 3 and results['tall']['present']['lethal_costmap_cells'] > results['tall']['before']['lethal_costmap_cells'],
            'depth_marks_above_lidar': results['elevated']['present']['scan_points'] == 0 and results['elevated']['present']['depth_points'] >= 10 and results['elevated']['present']['lethal_costmap_cells'] > results['elevated']['before']['lethal_costmap_cells'],
            'depth_clears_removed': results['elevated']['removed']['lethal_costmap_cells'] == results['elevated']['before']['lethal_costmap_cells'],
            'tall_clears_removed': results['tall']['removed']['lethal_costmap_cells'] == results['tall']['before']['lethal_costmap_cells'],
        }
        return results

    def run_monitor(self) -> dict:
        config = self.evaluation['monitor']
        self.move_fixture('tall', config['fixture_initial_x'], 0.0)
        self.wait(2.0)
        self.command = self.create_publisher(Twist, '/nav2/cmd_vel', 10)
        self.command_velocity = config['raw_vx']
        self.event('monitor_approach', stimulus='Direct test command to monitor input; no Nav2 action', raw_vx=config['raw_vx'])
        start = self.now()
        next_move = start
        deadline = time.monotonic()+config['approach_timeout_s']*8
        while self.now()-start < config['approach_timeout_s']:
            self.tick()
            if time.monotonic()>deadline:
                raise TimeoutError('Monitor approach clock timeout')
            if self.now() >= next_move:
                self.move_fixture('tall', max(config['fixture_final_x'], config['fixture_initial_x']-config['fixture_speed_mps']*(self.now()-start)), 0.0)
                next_move = self.now()+config['fixture_update_s']
            state = self.latest.get('/collision_monitor_state')
            if state and state.action_type == CollisionMonitorState.STOP and state.polygon_name == 'StopZone' and self.now()-start > 1:
                break
        self.event('monitor_hold')
        self.wait(config['hold_s'])
        self.move_fixture('tall', 50.0, 50.0)
        self.event('monitor_clear')
        self.wait(config['clear_s'])
        self.command_velocity = 0.0
        self.wait(1.0)
        slow = [r for r in self.rows if r['phase']=='monitor_approach' and r['action']==2 and r['raw_vx']>0.1]
        # Use the final second of the hold to avoid command/state transition samples.
        hold = [r for r in self.rows if r['phase']=='monitor_hold']
        stopped = [r for r in hold if r['sim_s'] >= hold[-1]['sim_s']-1] if hold else []
        resumed = [r for r in self.rows if r['phase']=='monitor_clear' and r['raw_vx']>0.1]
        ratios = [r['safe_vx']/r['raw_vx'] for r in slow]
        clearance = [r['clearance_m'] for r in self.rows if r['phase'].startswith('monitor_') and math.isfinite(r['clearance_m'])]
        result = dict(slowdown_samples=len(slow), slowdown_ratio_median=float(np.median(ratios)) if ratios else None,
                      stopped_samples=len(stopped),
                      stopped_max_actual_speed_mps=max((math.hypot(r['gt_vx'], r['gt_vy']) for r in stopped), default=None),
                      min_padded_footprint_clearance_m=min(clearance) if clearance else None)
        result['checks'] = dict(slowdown=bool(ratios) and abs(float(np.median(ratios))-self.scenario['expected_slowdown_ratio'])<self.thresholds['slowdown_ratio_tolerance'],
                                stop=bool(stopped) and all(r['action']==1 and r['polygon']=='StopZone' and abs(r['safe_vx'])<1e-6 and abs(r['safe_vy'])<1e-6 for r in stopped),
                                actual_stop=bool(stopped) and result['stopped_max_actual_speed_mps']<self.thresholds['stopped_speed_mps'],
                                resume=any(r['safe_vx']>0.1 and r['action']==0 for r in resumed),
                                footprint_clearance=bool(clearance) and min(clearance)>self.thresholds['min_clearance_m'])
        self.event('monitor_complete', **result)
        return result

    def run_avoid(self, keep_running: Callable[[], bool] | None = None, inject_obstacle: bool = True) -> dict:
        config = self.evaluation['avoid']
        assert self.nav.wait_for_server(timeout_sec=20)
        request = NavigateToPose.Goal()
        request.pose.header.frame_id = 'map'
        request.pose.header.stamp = self.get_clock().now().to_msg()
        request.pose.pose.position.x, request.pose.pose.position.y = config['goal'][:2]
        request.pose.pose.orientation.z = math.sin(config['goal'][2]/2)
        request.pose.pose.orientation.w = math.cos(config['goal'][2]/2)
        feedback = []
        def on_feedback(msg):
            f = msg.feedback
            feedback.append({'sim_s':self.now(), 'remaining_m':f.distance_remaining, 'recoveries':f.number_of_recoveries})
        self.active_goal = self.future(self.nav.send_goal_async(request, feedback_callback=on_feedback))
        assert self.active_goal.accepted
        result_future = self.active_goal.get_result_async()
        self.event('nav2_avoid', goal=config['goal'], stimulus=(
            'Gazebo fixture enters during NavigateToPose' if inject_obstacle else 'Route without injected fixtures'))
        start = self.now()
        next_move = start+config['fixture_start_s']
        next_report = start+5.0
        deadline = time.monotonic()+config['timeout_wall_s']
        fixture_settled = not inject_obstacle
        excessive_recoveries = False
        while not result_future.done():
            if keep_running is not None and not keep_running():
                raise InterruptedError('Viewer or simulation closed')
            self.tick()
            if self.now() >= next_move and not fixture_settled:
                y = max(config['fixture_final_y'], config['fixture_initial_y']-config['fixture_speed_mps']*(self.now()-start-config['fixture_start_s']))
                self.move_fixture('tall', config['fixture_x'], y)
                fixture_settled = y <= config['fixture_final_y']
                next_move = self.now()+config['fixture_update_s']
            if self.now() >= next_report:
                print(json.dumps({'phase':self.phase, 'sim_s':self.now(), 'truth':self.truth(), 'feedback':feedback[-1:]}), flush=True)
                next_report = self.now()+5
            if time.monotonic()>deadline:
                raise TimeoutError('Nav2 dynamic obstacle avoidance timeout')
            if feedback and feedback[-1]['recoveries'] > config['max_recoveries']:
                excessive_recoveries = True
                self.future(self.active_goal.cancel_goal_async())
                self.future(result_future, timeout=10)
                break
        action = result_future.result()
        self.active_goal = None
        self.event('nav2_settled', status=action.status, error_code=action.result.error_code)
        self.wait(2.0)
        rows = [r for r in self.rows if r['phase']=='nav2_avoid']
        clearance = [r['clearance_m'] for r in rows if math.isfinite(r['clearance_m'])]
        result = dict(status=action.status, error_code=action.result.error_code, error_msg=action.result.error_msg,
                      canceled_for_excessive_recoveries=excessive_recoveries,
                      final_truth=self.truth(), max_abs_lateral_m=max(abs(r['gt_y']) for r in rows),
                      min_padded_footprint_clearance_m=min(clearance) if clearance else None,
                      max_recoveries=max((f['recoveries'] for f in feedback), default=0), feedback=feedback)
        result['checks'] = dict(nav2_succeeded=action.status==4 and action.result.error_code==0,
                                lateral_avoidance=result['max_abs_lateral_m']>self.thresholds['min_lateral_avoidance_m'],
                                footprint_clearance=bool(clearance) and min(clearance)>self.thresholds['min_clearance_m'],
                                real_progress=math.hypot(self.truth()[0]-config['goal'][0], self.truth()[1]-config['goal'][1])<self.thresholds['max_final_goal_error_m'])
        result['injected_obstacle'] = inject_obstacle
        if not inject_obstacle:
            del result['checks']['lateral_avoidance']
            del result['checks']['footprint_clearance']
            result['not_applicable_checks'] = ['lateral_avoidance', 'footprint_clearance_to_test_fixture']
        return result

    def finish(self) -> None:
        if self.active_goal is not None:
            self.future(self.active_goal.cancel_goal_async(), timeout=10)
        if self.command is not None:
            self.command_velocity = 0.0
            self.wait(0.5)
            self.destroy_publisher(self.command)
            self.command = None
        self.nav.destroy()
