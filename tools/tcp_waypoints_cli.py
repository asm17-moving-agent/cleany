"""SSH-friendly numbered TCP teaching, guarded MoveIt replay and per-point reports."""
from __future__ import annotations

import argparse
import csv
from datetime import datetime
import hashlib
import json
import math
import os
from pathlib import Path
import select
import signal
import socket
import stat
import subprocess
import time
import xml.etree.ElementTree as ET

import yaml

from cleany_robot_interface.tcp_waypoints import Dataset, compare_point, sequence, validate_control_contract
from cleany_robot_interface.core import JointMapping
from tcp_waypoints_session import ROOT, BoardPreview, CameraMeasurement, TeachBus, create_session, load_json, register_gripper_reference


def emit(value: dict) -> None:
    print(json.dumps(value, ensure_ascii=False, allow_nan=False), flush=True)


def control_socket_path(dataset: Dataset) -> Path:
    """Keep AF_UNIX below Linux's 108-byte limit, even for long dataset paths."""
    user_runtime = Path('/run/user') / str(os.getuid())
    base = user_runtime if user_runtime.is_dir() else Path('/tmp')
    directory = base / f'cleany-tcp-{os.getuid()}'
    directory.mkdir(mode=0o700, exist_ok=True)
    info = directory.lstat()
    if info.st_uid != os.getuid() or not stat.S_ISDIR(info.st_mode) or info.st_mode & 0o077:
        raise RuntimeError('TCP control directory must be private and owned by the current user')
    identity = hashlib.sha256(str(dataset.root).encode()).hexdigest()[:32]
    return directory / (identity + '.sock')


class ControlSocket:
    """A scoped process-owned socket, never unlinks another process's endpoint."""
    def __init__(self, dataset: Dataset):
        self.path = control_socket_path(dataset)
        self.marker = dataset.root / 'control.sock'
        self.socket = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        self.bound = False
        self.marked = False
        self.stopped = False
        try:
            self.socket.bind(str(self.path))
            self.bound = True
            self.path.chmod(0o600)
            self.socket.listen(4)
            self.marker.symlink_to(self.path)
            self.marked = True
        except BaseException:
            self.close()
            raise

    def pending(self, timeout: float = 0) -> bool:
        return bool(select.select([self.socket], [], [], timeout)[0])

    def handle(self, handler) -> None:
        if not self.pending():
            return
        connection, _ = self.socket.accept()
        with connection:
            connection.settimeout(20)
            try:
                with connection.makefile('r') as stream:
                    text = stream.readline(2048)
                request = json.loads(text)
                if request['action'] == 'stop':
                    self.stopped = True
                    result = {'ok': True, 'message': 'Stop requested; torque hold is preserved'}
                else:
                    result = {'ok': True, **handler(request)}
            except Exception as error:
                result = {'ok': False, 'error': str(error)}
            try:
                connection.sendall((json.dumps(result, allow_nan=False) + '\n').encode())
            except (BrokenPipeError, ConnectionResetError):
                pass

    def close(self) -> None:
        self.socket.close()
        if self.bound:
            self.path.unlink(missing_ok=True)
            self.bound = False
        if self.marked:
            self.marker.unlink(missing_ok=True)
            self.marked = False


def request(dataset: Dataset, action: str, number: int | None = None, *, confirmed_closed: bool = False) -> dict:
    with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as connection:
        connection.settimeout(load_json(dataset.root / 'policy.json')['measurement_timeout_s'] + 10)
        connection.connect(str(control_socket_path(dataset)))
        connection.sendall((json.dumps({'action': action, 'number': number, 'confirmed_closed': confirmed_closed}) + '\n').encode())
        with connection.makefile('r') as stream:
            response = json.loads(stream.readline())
    if not response.get('ok'):
        raise RuntimeError(response.get('error', 'Request failed'))
    if action == 'stop':
        deadline = time.monotonic() + 15
        while control_socket_path(dataset).exists():
            if time.monotonic() > deadline:
                raise RuntimeError('Stop requested, but process shutdown is not confirmed')
            time.sleep(.05)
        response['process_shutdown_confirmed'] = True
    return response


def closed_gripper(dataset: Dataset, q: dict) -> None:
    policy = load_json(dataset.root / 'policy.json')
    actual, expected = q[dataset.side + '_gripper_joint'], dataset.manifest['closed_gripper_joint_rad']
    tolerance = policy['closed_gripper_tolerance_rad']
    if not math.isfinite(actual) or abs(actual - expected) > tolerance:
        raise ValueError(f'Close the selected gripper manually before using its closed-fingertip TCP: '
                         f'actual={actual:.6f}, reference={expected:.6f}, tolerance={tolerance:.6f} rad. '
                         'If physically fully closed, register its reference before saving any points')


def teach(dataset: Dataset, preview: bool = False) -> None:
    channel = ControlSocket(dataset)
    bus = camera = viewer = None
    try:
        # A display failure must occur before acquiring the serial bus.
        if preview:
            viewer = BoardPreview(dataset.side)
        bus = TeachBus(dataset)
        camera = CameraMeasurement(dataset)
        if viewer is not None:
            camera.preview = viewer

        def handler(payload):
            action = payload['action']
            if action == 'hold':
                bus.hold()
                return {'message': 'Current-position hold ON; operating speed/torque reused'}
            if action == 'release':
                bus.release()
                return {'message': 'Selected five arm motors torque OFF; no gripper/head writes'}
            if action == 'status':
                state = bus.read()
                return {'phase': 'teaching', 'side': dataset.side, 'state': state, 'powered': bus.powered(),
                        'closed_gripper_reference_rad': dataset.manifest['closed_gripper_joint_rad'],
                        'board': getattr(camera, 'preview_status', None)}
            if action == 'reference-gripper':
                return register_gripper_reference(dataset, bus.read, confirmed_closed=payload.get('confirmed_closed', False))
            if action != 'save':
                raise ValueError('Unknown teaching command')
            number = payload['number']
            if not isinstance(number, int) or number < 1 or (dataset.root / 'points' / str(number)).exists():
                raise ValueError('Use a new positive point number')
            if not bus.powered():
                raise ValueError('Run hold before saving; teach and arrival must both be powered')
            closed_gripper(dataset, bus.read()['positions_rad'])
            measurement, image = camera.measure(bus.read)
            if not bus.powered():
                raise RuntimeError('Torque hold changed during capture')
            closed_gripper(dataset, measurement['positions_rad'])
            measurement['torque_hold_verified'] = True
            path = dataset.commit(number, measurement, image)
            emit({'saved': number, 'directory': str(path), 'base_xyz_m': [row[3] for row in measurement['base_T_tcp'][:3]]})
            return {'saved': number, 'directory': str(path)}

        emit({'phase': 'teaching', 'side': dataset.side, 'session': str(dataset.root), 'preview': preview,
              'commands': ['release', 'hold', 'reference-gripper --confirm-closed', 'save NUMBER', 'status', 'stop']})
        while not channel.stopped:
            if channel.pending(.05 if preview else .1):
                channel.handle(handler)
            if preview and not channel.stopped:
                camera.poll_preview()
    finally:
        if camera:
            camera.close()
        if bus:
            bus.close()
        if viewer:
            viewer.close()
        channel.close()


class MoveItReplay:
    def __init__(self, dataset: Dataset, channel: ControlSocket, process=None):
        import rclpy
        from rclpy.action import ActionClient
        from sensor_msgs.msg import JointState
        from std_msgs.msg import String
        from control_msgs.action import FollowJointTrajectory
        self.ros = rclpy
        self.dataset, self.channel, self.process = dataset, channel, process
        self.policy = load_json(dataset.root / 'policy.json')
        self.control = yaml.safe_load((dataset.root / 'control.yaml').read_text())
        self.node = rclpy.create_node('tcp_waypoint_replay')
        self.state = self.guard = None
        self.guard_received = 0
        self.baseline = None
        self.handle = None
        self.clients = {}
        self.subscriptions = [self.node.create_subscription(JointState, '/joint_states', self.receive, 20),
            self.node.create_subscription(String, '/real_control/status', self.receive_guard, 10)]
        self.action = ActionClient(self.node, FollowJointTrajectory,
                                  f'/{dataset.side}_arm_controller/follow_joint_trajectory')
        self.maps = [JointMapping(**row) for row in load_json(dataset.root / 'mapping.json')]
        tree = ET.parse(dataset.root / 'profile/robot_control.urdf')
        self.bounds = {j.get('name'): (float(j.find('limit').get('lower')), float(j.find('limit').get('upper')))
                       for j in tree.findall('joint') if j.get('name') in dataset.manifest['state_joints']}

    def receive(self, message):
        names, positions = message.name, message.position
        if (len(names) != len(positions) or len(names) != len(set(names))
                or set(names) != set(self.dataset.manifest['state_joints'])
                or not all(math.isfinite(v) for v in positions)):
            self.state = None
            return
        now = time.monotonic_ns()
        self.state = {'positions_rad': dict(zip(names, positions)),
            'header_stamp_ns': message.header.stamp.sec * 10**9 + message.header.stamp.nanosec,
            'host_monotonic_start_ns': now, 'host_monotonic_end_ns': now}

    def receive_guard(self, message):
        self.guard = json.loads(message.data)
        self.guard_received = time.monotonic()

    def poll(self):
        self.ros.spin_once(self.node, timeout_sec=.01)
        def command(payload):
            if payload['action'] != 'status':
                raise ValueError('During replay, use stop first; teaching/torque commands are unavailable')
            return {'phase': 'executing', 'side': self.dataset.side, 'state': self.state,
                    'guard': self.guard}
        self.channel.handle(command)
        if self.channel.stopped:
            raise RuntimeError('Operator stop requested')
        if self.process is not None and self.process.poll() is not None:
            raise RuntimeError('Owned ROS launch exited; inspect ros.log')

    def check(self) -> dict:
        self.poll()
        s = self.state
        if (s is None or not 0 <= time.monotonic_ns() - s['host_monotonic_end_ns'] <= self.control['max_state_age_s'] * 1e9
                or not 0 <= self.node.get_clock().now().nanoseconds - s['header_stamp_ns'] <= self.control['max_state_age_s'] * 1e9):
            raise RuntimeError('Fresh complete 14-joint encoder feedback required')
        if self.guard is None or self.guard['state'] != 'ready' or time.monotonic() - self.guard_received > .3:
            raise RuntimeError('Hardware feedback guard not ready')
        q = s['positions_rad']
        closed_gripper(self.dataset, q)
        self.check_bounds(q)
        if self.baseline:
            self.passive_match(self.baseline, q)
        return q.copy()

    def passive_match(self, reference, observed):
        tolerance = self.policy['passive_joint_tolerance_rad']
        moved = [j for j in reference if j not in self.dataset.command_joints and abs(reference[j] - observed[j]) > tolerance]
        if moved:
            raise RuntimeError('Uncommanded joints differ from teaching/start: ' + ', '.join(moved))

    def check_bounds(self, q):
        for name, (low, high) in self.bounds.items():
            if not math.isfinite(q[name]) or not low <= q[name] <= high:
                raise ValueError(name + ': outside materialized URDF bounds')
        for j in self.maps:
            raw = round(j.zero_tick + j.direction * (q[j.name] - j.offset_rad) * 4096 / math.tau)
            if not j.range_min <= raw <= j.range_max:
                raise ValueError(j.name + ': outside calibrated encoder bounds')

    def wait(self, future, timeout, monitor=False):
        deadline = time.monotonic() + timeout
        while not future.done():
            self.check() if monitor else self.poll()
            if time.monotonic() > deadline:
                raise RuntimeError('ROS operation timeout')
        if future.exception():
            raise future.exception()
        return future.result()

    def query(self, kind, path, payload, monitor=True):
        client = self.clients.get(path)
        if client is None:
            client = self.node.create_client(kind, path)
            self.clients[path] = client
            if not client.wait_for_service(timeout_sec=self.policy['startup_timeout_s']):
                raise RuntimeError('ROS service missing: ' + path)
        return self.wait(client.call_async(payload), self.policy['service_timeout_s'], monitor)

    def ready(self, points):
        from controller_manager_msgs.srv import ListControllers
        from rcl_interfaces.srv import GetParameters
        deadline = time.monotonic() + self.policy['startup_timeout_s']
        while True:
            try:
                current = self.check()
                break
            except RuntimeError:
                if (self.channel.stopped or (self.guard and self.guard.get('state') == 'fault')
                        or (self.process and self.process.poll() is not None) or time.monotonic() > deadline):
                    raise
        self.baseline = current
        expected = {self.dataset.side + '_arm_controller'}
        while True:
            controllers = self.query(ListControllers, '/controller_manager/list_controllers', ListControllers.Request()).controller
            active = {c.name for c in controllers if c.state == 'active' and c.name != 'joint_state_broadcaster'}
            if active - expected:
                raise ValueError('Unexpected active controllers: ' + str(active))
            if active == expected:
                break
            if time.monotonic() > deadline:
                raise RuntimeError('Selected arm controller did not activate within startup deadline')
            self.check()
        model = (self.dataset.root / 'profile/robot_control.urdf').read_text()
        for path, names in (('/controller_manager/get_parameters', ['robot_description']),
                            ('/move_group/get_parameters', ['robot_description', 'allow_trajectory_execution', 'use_sim_time'])):
            values = self.query(GetParameters, path, GetParameters.Request(names=names)).values
            if ET.tostring(ET.fromstring(values[0].string_value)) != ET.tostring(ET.fromstring(model)):
                raise ValueError('Planner/controller model differs from frozen teaching model')
            if len(values) > 1 and (values[1].bool_value or values[2].bool_value):
                raise ValueError('MoveIt must be plan-only and use real time')
        if not self.action.wait_for_server(timeout_sec=10):
            raise RuntimeError('Selected arm action server unavailable')
        for point in points:
            self.check_bounds(point['positions_rad'])
            self.passive_match(point['positions_rad'], self.check())
            self.valid({**self.check(), **{j: point['positions_rad'][j] for j in self.dataset.command_joints}})
        self.valid(self.check())

    def valid(self, q):
        from moveit_msgs.srv import GetStateValidity
        self.check_bounds(q)
        request = GetStateValidity.Request()
        request.robot_state.joint_state.name = list(q)
        request.robot_state.joint_state.position = list(q.values())
        # Empty group checks the complete robot, including the opposite arm and head.
        result = self.query(GetStateValidity, '/check_state_validity', request)
        if not result.valid:
            raise ValueError('MoveIt rejected full-robot collision state: ' + str(result.contacts[:3]))

    def plan(self, point, folder):
        from moveit_msgs.srv import GetMotionPlan
        from moveit_msgs.msg import Constraints, JointConstraint
        from cleany_skill_executor.core.joint_path import executable_joint_path
        from control_msgs.action import FollowJointTrajectory
        from control_msgs.msg import JointTolerance
        from trajectory_msgs.msg import JointTrajectoryPoint
        names = self.dataset.command_joints
        start = self.check()
        target = {j: point['positions_rad'][j] for j in names}
        self.passive_match(point['positions_rad'], start)
        self.valid(start)
        self.valid({**start, **target})
        request = GetMotionPlan.Request()
        m = request.motion_plan_request
        m.group_name = self.dataset.side + '_arm'
        m.pipeline_id = 'ompl'
        m.planner_id = 'RRTConnectkConfigDefault'
        m.allowed_planning_time = self.policy['planning_time_s']
        m.num_planning_attempts = self.policy['planning_attempts']
        m.max_velocity_scaling_factor = self.policy['velocity_scaling']
        m.max_acceleration_scaling_factor = self.policy['acceleration_scaling']
        m.start_state.joint_state.name = list(start)
        m.start_state.joint_state.position = list(start.values())
        m.goal_constraints = [Constraints(joint_constraints=[JointConstraint(joint_name=j,
            position=target[j], tolerance_above=1e-5, tolerance_below=1e-5, weight=1.) for j in names])]
        response = self.query(GetMotionPlan, '/plan_kinematic_path', request).motion_plan_response
        if response.error_code.val != 1:
            raise RuntimeError(f'MoveIt planning failed: {response.error_code.val}')
        trajectory = response.trajectory.joint_trajectory
        if set(trajectory.joint_names) != set(names) or len(trajectory.points) < 2:
            raise ValueError('Planner returned unexpected joints or empty path')
        rows = [tuple(dict(zip(trajectory.joint_names, p.positions))[j] for j in names) for p in trajectory.points]
        if max(abs(rows[0][i] - start[j]) for i, j in enumerate(names)) > 1e-4 or max(abs(rows[-1][i] - target[j]) for i, j in enumerate(names)) > 1e-4:
            raise ValueError('Planner start/goal differs from actual/taught state')
        rows[0], rows[-1] = tuple(start[j] for j in names), tuple(target[j] for j in names)
        path = executable_joint_path(rows, simplification_tolerance=0)
        d = self.control['joint_defaults']
        duration = path.minimum_duration([d['max_velocity'] * self.policy['velocity_scaling']] * 5,
            [d['max_acceleration'] * self.policy['acceleration_scaling']] * 5) * 1.05
        samples = path.samples(self.policy['collision_sample_step_rad'],
            self.policy['minimum_path_subdivisions'], self.policy['maximum_path_points'])
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(names)
        for sample in samples:
            self.valid({**start, **dict(zip(names, sample.positions))})
            pt = JointTrajectoryPoint(positions=list(sample.positions), velocities=[v / duration for v in sample.derivatives])
            ns = round(sample.progress * duration * 1e9)
            pt.time_from_start.sec, pt.time_from_start.nanosec = divmod(ns, 10**9)
            goal.trajectory.points.append(pt)
        settings = self.control['controller']
        # Preserve the established controller's tracking/goal tolerances.
        goal.goal_tolerance = [JointTolerance(name=j, position=settings['goal_tolerance']) for j in names]
        ns = round(settings['goal_time'] * 1e9)
        goal.goal_time_tolerance.sec, goal.goal_time_tolerance.nanosec = divmod(ns, 10**9)
        (folder / 'plan.json').write_text(json.dumps({'start': start, 'target': target,
            'duration_s': duration, 'collision_sample_step_rad': self.policy['collision_sample_step_rad'],
            'samples': [{'progress': p.progress, 'positions': list(p.positions), 'derivatives': list(p.derivatives)} for p in samples]}, indent=2))
        current = self.check()
        if max(abs(current[j] - start[j]) for j in current) > self.policy['start_tolerance_ticks'] * math.tau / 4096:
            raise RuntimeError('Robot moved during planning; path not executed')
        return goal, duration

    def execute(self, goal, duration, folder):
        feedback = []

        def received(message):
            f = message.feedback
            feedback.append({'monotonic_ns': time.monotonic_ns(), 'names': list(f.joint_names),
                'desired': list(f.desired.positions), 'actual': list(f.actual.positions), 'error': list(f.error.positions)})

        try:
            self.handle = self.wait(self.action.send_goal_async(goal, feedback_callback=received), 5, True)
            if not self.handle.accepted:
                raise RuntimeError('Selected arm rejected the trajectory')
            result = self.wait(self.handle.get_result_async(), duration + self.control['controller']['goal_time'] + 5, True)
            if result.status != 4 or result.result.error_code != 0:
                raise RuntimeError(f'Trajectory failed: status={result.status}, code={result.result.error_code}, {result.result.error_string}')
            self.handle = None
        finally:
            (folder / 'feedback.json').write_text(json.dumps(feedback, indent=2))

    def read_measurement_state(self):
        q = self.check()
        previous = self.state['header_stamp_ns']
        deadline = time.monotonic() + self.control['max_state_age_s']
        while self.state['header_stamp_ns'] == previous:
            q = self.check()
            if time.monotonic() > deadline:
                raise RuntimeError('Encoder stamp did not advance')
        row = {**self.state, 'positions_rad': q, 'joints': {}}
        for j in self.maps:
            row['joints'][j.name] = {'id': j.motor_id,
                'raw_tick': round(j.zero_tick + j.direction * (q[j.name] - j.offset_rad) * 4096 / math.tau)}
        # Also enforce stationary head feedback during image capture.
        head_map = load_json(self.dataset.root / 'head_mapping.json')
        for i, name in enumerate(('head_pan_joint', 'head_tilt_joint')):
            row['joints'][name] = {'id': 13+i, 'raw_tick': round(head_map['reference_ticks'][i] +
                q[name] / (head_map['encoder_directions'][i] * head_map['encoder_scale_rad_per_tick']))}
        return row

    def stop(self):
        from std_srvs.srv import Trigger
        errors = []
        try:
            if self.handle is not None and self.handle.accepted:
                self.ros.spin_until_future_complete(self.node, self.handle.cancel_goal_async(), timeout_sec=2)
            client = self.node.create_client(Trigger, '/real_control/stop')
            if client.wait_for_service(timeout_sec=2):
                self.ros.spin_until_future_complete(self.node, client.call_async(Trigger.Request()), timeout_sec=2)
            deadline = time.monotonic() + 3
            while time.monotonic() < deadline:
                self.ros.spin_once(self.node, timeout_sec=.05)
                if self.guard and self.guard.get('controllers_stopped'):
                    return {'controllers_stopped': True, 'torque_hold_preserved': True}
            errors.append('Controller deactivation not confirmed by feedback guard')
        except Exception as error:
            errors.append(str(error))
        return {'controllers_stopped': False, 'errors': errors, 'physical_stop_verified': False}


def replay(dataset: Dataset, order: tuple[int, ...], plan_only: bool = False) -> Path:
    import rclpy
    dataset.verify()
    validate_control_contract(dataset)
    points = [dataset.point(n) for n in order]
    os.environ['ROS_DOMAIN_ID'] = str(load_json(dataset.root / 'policy.json')['ros_domain_id'])
    channel = ControlSocket(dataset)
    output = dataset.root / 'runs' / str(time.time_ns())
    output.mkdir(parents=True)
    camera = process = client = None
    record = {'sequence': list(order), 'side': dataset.side, 'status': 'starting', 'points': [],
              'plan_only': plan_only, 'absolute_tcp_accuracy_verified': False}
    log = (output / 'ros.log').open('w')
    initialized = False
    try:
        # Verify camera availability before enabling the arm controller.
        camera = CameraMeasurement(dataset)
        process = subprocess.Popen(['ros2', 'launch', 'cleany_robot_interface', 'tcp_waypoints.launch.py',
            'session:=' + str(dataset.root)], stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        rclpy.init()
        initialized = True
        client = MoveItReplay(dataset, channel, process)
        client.ready(points)
        emit({'phase': 'ready', 'side': dataset.side, 'run': str(output), 'sequence': list(order)})
        for index, point in enumerate(points, 1):
            folder = output / f'{index:03d}_point_{point["number"]}'
            folder.mkdir()
            item = {'order': index, 'point': point['number'], 'status': 'planning'}
            record['points'].append(item)
            goal, duration = client.plan(point, folder)
            emit({'planning_complete': point['number'], 'duration_s': duration})
            if plan_only:
                item['status'] = 'planned_from_current_state'
                continue
            item['status'] = 'executing'
            client.execute(goal, duration, folder)
            measurement, image = camera.measure(client.read_measurement_state, lambda: channel.stopped)
            # Board measurement comes after settled FJT completion. Never hide a failed arrival.
            observed = measurement['positions_rad']
            tolerance = client.control['controller']['goal_tolerance']
            if max(abs(observed[j] - point['positions_rad'][j]) for j in dataset.command_joints) > tolerance:
                raise RuntimeError('Settled arrival exceeds the operating controller goal tolerance')
            error = compare_point(point, measurement)
            item.update({'status': 'measured', **error,
                'joint_error_rad': {j: observed[j] - point['positions_rad'][j] for j in dataset.command_joints}})
            (folder / 'rgb.png').write_bytes(image)
            (folder / 'measurement.json').write_text(json.dumps(measurement, indent=2, allow_nan=False))
            (output / 'report.json').write_text(json.dumps(record, indent=2, allow_nan=False))
            emit({'arrived': point['number'], 'encoder_tcp_mm': error['encoder_tcp']['position_mm'],
                  'board_video_tcp_mm': error['board_video_tcp']['position_mm']})
        record['status'] = 'planned' if plan_only else 'completed'
        write_report(output, record)
        emit({'status': record['status'], 'report': str(output / 'report.json'),
              'csv': str(output / 'errors.csv'), 'torque': 'Holding; stop then release when supported'})
        while not channel.stopped:
            client.poll()
    except BaseException as error:
        if record['status'] not in ('completed', 'planned'):
            record['status'] = 'stopped'
            record['error'] = str(error) or type(error).__name__
            if record['points'] and record['points'][-1]['status'] in ('planning', 'executing'):
                record['points'][-1]['status'] = 'failed'
                record['points'][-1]['error'] = record['error']
        elif not isinstance(error, KeyboardInterrupt) and not channel.stopped:
            record['hold_error'] = str(error)
        if not isinstance(error, KeyboardInterrupt) and not channel.stopped:
            raise
    finally:
        if client:
            record['shutdown'] = client.stop()
        if process and process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=8)
            except subprocess.TimeoutExpired:
                record['shutdown_forced'] = True
                os.killpg(process.pid, signal.SIGTERM)
                try:
                    process.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait(timeout=3)
        if camera:
            camera.close()
        if client:
            client.node.destroy_node()
        if initialized and rclpy.ok():
            rclpy.shutdown()
        log.close()
        write_report(output, record)
        channel.close()
    return output


def write_report(output: Path, record: dict) -> None:
    (output / 'report.json').write_text(json.dumps(record, indent=2, allow_nan=False) + '\n')
    columns = ['order', 'point', 'status', 'encoder_tcp_mm', 'board_video_tcp_mm', 'board_video_orientation_deg',
               'board_dx_mm', 'board_dy_mm', 'board_dz_mm']
    with (output / 'errors.csv').open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=columns)
        writer.writeheader()
        for item in record['points']:
            row = {key: item[key] for key in ('order', 'point', 'status')}
            if item['status'] == 'measured':
                board = item['board_video_tcp']
                row.update(encoder_tcp_mm=item['encoder_tcp']['position_mm'], board_video_tcp_mm=board['position_mm'],
                    board_video_orientation_deg=board['orientation_deg'],
                    **dict(zip(('board_dx_mm', 'board_dy_mm', 'board_dz_mm'), board['delta_xyz_mm'])))
            writer.writerow(row)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest='action', required=True)
    init = sub.add_parser('init', help='Freeze the current model, calibration and operating settings; no device writes')
    init.add_argument('side', choices=('left', 'right'))
    init.add_argument('--session', type=Path, required=True)
    init.add_argument('--config', type=Path, default=ROOT / 'configs/robot/tcp_waypoints.json')
    for action in ('teach', 'save', 'hold', 'release', 'stop', 'status', 'run', 'reference-gripper'):
        command = sub.add_parser(action)
        command.add_argument('--session', type=Path, required=True)
        if action == 'teach':
            command.add_argument('--preview', action='store_true', help='Show live wrist/board detection on DISPLAY; commands stay in SSH')
        if action == 'reference-gripper':
            command.add_argument('--confirm-closed', action='store_true', required=True,
                                 help='Operator confirms the empty gripper is physically fully closed')
        if action == 'save':
            command.add_argument('number', type=int)
        if action == 'run':
            command.add_argument('--sequence', type=sequence, required=True)
            command.add_argument('--plan-only', action='store_true', help='Plan each target from the current pose; current-position torque hold is enabled')
    args = parser.parse_args()
    if args.action == 'init':
        dataset = create_session(args.side, args.session, args.config)
        emit({'session': str(dataset.root), 'side': dataset.side,
              'operating_settings': yaml.safe_load((dataset.root / 'control.yaml').read_text())['joint_defaults']})
        return
    dataset = Dataset.load(args.session)
    if args.action == 'teach':
        teach(dataset, preview=args.preview)
    elif args.action == 'run':
        replay(dataset, args.sequence, args.plan_only)
    elif args.action == 'release' and not control_socket_path(dataset).exists():
        bus = TeachBus(dataset, release_only=True)
        try:
            bus.release()
            emit({'torque_off_confirmed': True, 'side': dataset.side})
        finally:
            bus.close()
    elif args.action == 'reference-gripper' and not control_socket_path(dataset).exists():
        bus = TeachBus(dataset)
        try:
            emit(register_gripper_reference(dataset, bus.read, confirmed_closed=args.confirm_closed))
        finally:
            bus.close()
    else:
        emit(request(dataset, args.action, getattr(args, 'number', None),
                     confirmed_closed=getattr(args, 'confirm_closed', False)))


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        emit({'status': 'stopped', 'torque': 'Current hold preserved; use release explicitly'})
    except Exception as error:
        emit({'error': str(error)})
        raise SystemExit(1)
