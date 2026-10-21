"""Actual MoveIt/JTC/C++ driver replay against 14 PTY motors and rendered USB images.

No USB devices are opened. Rendering injects a known camera displacement independent
of encoder feedback to ensure the external error column cannot be encoder FK alone.
"""
import importlib.util
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time
import xml.etree.ElementTree as ET

import cv2
import numpy as np
import pytest
import rclpy
import yaml
from moveit_msgs.srv import GetStateValidity

from cleany_robot_interface.core import JointMapping
from cleany_robot_interface.tcp_waypoints import Dataset, digest, pose_error
from cleany_handeye_calibration.headcam_core import make_board
from cleany_handeye_calibration.offline_fk import UrdfOfflineFk
import tcp_waypoints_cli as cli
from tcp_waypoints_session import ROOT, CameraMeasurement, TeachBus, create_session, load_json

spec = importlib.util.spec_from_file_location('tcp_pty_motors',
    ROOT / 'ros2_ws/src/cleany_robot_interface/runtime_test/test_controller_emulator.py')
emulator = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = emulator
spec.loader.exec_module(emulator)


def terminate(process):
    if process.poll() is None:
        os.killpg(process.pid, signal.SIGINT)
        try:
            process.wait(timeout=8)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait(timeout=3)


def freeze_again(session):
    manifest = load_json(session.root / 'session.json')
    manifest['model_sha256'] = digest(session.root / 'profile/robot_control.urdf')
    manifest['frozen_sha256'] = {name: digest(session.root / name) for name in manifest['frozen_sha256']}
    (session.root / 'session.json').write_text(json.dumps(manifest, indent=2))
    return Dataset.load(session.root)


def valid_fixture_poses(session):
    """Use the real full-robot collision service to select five nearby fixture poses."""
    output = session.root / 'fixture_planner.log'
    with output.open('w') as stream:
        process = subprocess.Popen(['ros2', 'launch', 'cleany_moveit_config', 'physical_move_group.launch.py',
            'robot_description_file:=' + str(session.root / 'profile/robot_control.urdf'),
            'joint_limits_file:=' + str(session.root / 'profile/joint_limits.yaml'),
            'include_head_camera:=true', 'allow_trajectory_execution:=false', 'use_sim_time:=false'],
            stdout=stream, stderr=subprocess.STDOUT, start_new_session=True)
    rclpy.init()
    node = rclpy.create_node('tcp_fixture_pose_query')
    try:
        client = node.create_client(GetStateValidity, '/check_state_validity')
        assert client.wait_for_service(timeout_sec=40), output.read_text()[-4000:]

        def valid(q):
            request = GetStateValidity.Request()
            request.robot_state.joint_state.name = list(q)
            request.robot_state.joint_state.position = list(q.values())
            future = client.call_async(request)
            rclpy.spin_until_future_complete(node, future, timeout_sec=5)
            assert future.done()
            return future.result().valid

        tree = ET.parse(session.root / 'profile/robot_control.urdf')
        bounds = {j.get('name'): tuple(float(j.find('limit').get(k)) for k in ('lower', 'upper'))
                  for j in tree.findall('joint') if j.get('name') in session.manifest['state_joints']}
        q = {j: 0. for j in session.manifest['state_joints']}
        for side in ('left', 'right'):
            q[side + '_gripper_joint'] = session.manifest['closed_gripper_joint_rad']
        rng = np.random.default_rng(813)
        for attempt in range(150):
            if attempt:
                # Park both arms in collision-free configurations; constrain yaw spread.
                for side in ('left', 'right'):
                    for suffix in ('shoulder_yaw', 'shoulder_pitch', 'elbow_pitch', 'wrist_pitch', 'wrist_roll'):
                        j = side + '_' + suffix + '_joint'
                        low, high = bounds[j]
                        q[j] = float(rng.uniform(max(low + .05, -.9), min(high - .05, 2.5)))
            points = []
            for offset in (0, .015, -.015, .025, -.025):
                pose = q.copy()
                pose[session.command_joints[0]] += offset
                if not all(bounds[j][0] <= pose[j] <= bounds[j][1] for j in pose) or not valid(pose):
                    break
                points.append(pose)
            if len(points) == 5:
                return points
        raise AssertionError('No collision-free fixture poses found')
    finally:
        node.destroy_node()
        rclpy.shutdown()
        terminate(process)


class RenderedWrist:
    def __init__(self, session, motors):
        self.session, self.motors = session, motors
        self.maps = [JointMapping(**row) for row in load_json(session.root / 'mapping.json')]
        self.head = load_json(session.root / 'head_mapping.json')
        self.fk = UrdfOfflineFk(session.root / 'profile/robot_control.urdf', tip_frame=session.side + '_gripper_frame')
        self.k = np.asarray(load_json(session.root / 'intrinsics.json')['K'])
        self.d = np.asarray(load_json(session.root / 'intrinsics.json')['D'])
        config = load_json(session.root / 'camera.json')
        _, board = make_board(config)
        self.texture = board.draw((1400, 1000), marginSize=0)
        q = self.q()
        camera_T_board = np.eye(4)
        camera_T_board[:3, :3] = cv2.Rodrigues(np.array([.15, -.25, .05]))[0]
        camera_T_board[:3, 3] = [-.175, -.125, .7]
        # OpenCV 4.5 Charuco draw pixels run down; board object Y runs up.
        # Keep the physical board frame consistent with the detector's corner IDs.
        draw_T_board = np.diag([1., -1., -1., 1.])
        draw_T_board[1, 3] = .25
        camera_T_board = camera_T_board @ draw_T_board
        self.base_T_board = self.fk.compute(tuple(q), tuple(q.values())).as_homogeneous_matrix() @ np.array(session.manifest['gripper_T_camera']) @ camera_T_board
        self.displacement = 0.0
        # Supersample then area-filter to avoid pixel aliasing moving board corners.
        self.scale = 3
        self.render_k = self.k.copy()
        self.render_k[:2] *= self.scale
        self.render_k[0, 2] += (self.scale - 1) / 2
        self.render_k[1, 2] += (self.scale - 1) / 2
        raw = np.indices((480*self.scale, 640*self.scale)).transpose(1, 2, 0)[:, :, ::-1].astype(np.float32).reshape(-1, 1, 2)
        ideal = cv2.undistortPoints(raw, self.render_k, self.d, P=self.render_k).reshape(480*self.scale, 640*self.scale, 2)
        self.mapx, self.mapy = ideal[:, :, 0].copy(), ideal[:, :, 1].copy()
        self.cache_key = self.cache_image = None

    def q(self):
        q = {j.name: j.radians(int.from_bytes(self.motors.registers[j.motor_id][56:58], 'little')) for j in self.maps}
        for i, name in enumerate(('head_pan_joint', 'head_tilt_joint')):
            raw = int.from_bytes(self.motors.registers[13+i][56:58], 'little')
            q[name] = (raw - self.head['reference_ticks'][i]) * self.head['encoder_directions'][i] * self.head['encoder_scale_rad_per_tick']
        return q

    def open(self, *_): return True
    def set(self, *_): pass
    def release(self): pass

    def truth(self, q, displacement):
        base_T_gripper = self.fk.compute(tuple(q), tuple(q.values())).as_homogeneous_matrix()
        base_T_gripper[0, 3] += displacement
        return np.linalg.inv(self.base_T_board) @ base_T_gripper @ np.asarray(self.session.manifest['gripper_T_tcp'])

    def read(self):
        time.sleep(.015)
        q = self.q()
        key = (*q.values(), self.displacement)
        if key == self.cache_key:
            return True, self.cache_image.copy()
        camera = self.fk.compute(tuple(q), tuple(q.values())).as_homogeneous_matrix() @ np.asarray(self.session.manifest['gripper_T_camera'])
        camera[0, 3] += self.displacement  # Independent injected fixture displacement in base X.
        camera_T_board = np.linalg.inv(camera) @ self.base_T_board
        obj = np.array([[0, .25, 0], [.35, .25, 0], [.35, 0, 0], [0, 0, 0]], dtype=float)
        rotation = cv2.Rodrigues(camera_T_board[:3, :3])[0]
        dst = cv2.projectPoints(obj, rotation, camera_T_board[:3, 3], self.render_k, np.zeros(5))[0].reshape(4, 2).astype(np.float32)
        src = np.array([[0, 0], [1399, 0], [1399, 999], [0, 999]], dtype=np.float32)
        image = cv2.warpPerspective(self.texture, cv2.getPerspectiveTransform(src, dst), (640*self.scale, 480*self.scale), borderValue=255)
        image = cv2.remap(image, self.mapx, self.mapy, cv2.INTER_LINEAR, borderValue=255)
        image = cv2.resize(image, (640, 480), interpolation=cv2.INTER_AREA)
        self.cache_key, self.cache_image = key, cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
        return True, self.cache_image.copy()


@pytest.mark.parametrize('side', ['left', 'right'])
def test_five_taught_points_moveit_replay_and_independent_errors(tmp_path, monkeypatch, side):
    from scservo_sdk import PortHandler
    original_timeout = PortHandler.setPacketTimeout

    def pty_timeout(port, length):
        original_timeout(port, length)
        port.packet_timeout = max(port.packet_timeout, 100.)

    # The SDK's USB latency default is too short for a Python PTY worker under rendering load.
    monkeypatch.setattr(PortHandler, 'setPacketTimeout', pty_timeout)
    session = create_session(side, tmp_path / side, ROOT / 'configs/robot/tcp_waypoints.json')
    policy = load_json(session.root / 'policy.json')
    policy['ros_domain_id'] = 178
    (session.root / 'policy.json').write_text(json.dumps(policy))
    # Test-only PTY scheduling allowance. Motion limits remain the operating values.
    control = yaml.safe_load((session.root / 'control.yaml').read_text())
    control['serial_timeout_ms'], control['max_state_age_s'] = 100, .5
    (session.root / 'control.yaml').write_text(yaml.safe_dump(control))
    tree = ET.parse(session.root / 'profile/robot_control.urdf')
    hardware = tree.find('ros2_control/hardware')
    hardware.find("param[@name='serial_timeout_ms']").text = '100'
    hardware.find("param[@name='max_state_age_s']").text = '0.5'
    motors = emulator.Servos(tree.findall('ros2_control/joint'))
    try:
        for key in ('arm_port', 'head_port'):
            hardware.find(f"param[@name='{key}']").text = motors.path
        tree.write(session.root / 'profile/robot_control.urdf', encoding='unicode')
        manifest = load_json(session.root / 'session.json')
        manifest['arm_port'] = manifest['head_port'] = motors.path
        (session.root / 'session.json').write_text(json.dumps(manifest))
        session = freeze_again(session)
        os.environ['ROS_DOMAIN_ID'] = str(policy['ros_domain_id'])
        poses = valid_fixture_poses(session)
        maps = [JointMapping(**row) for row in load_json(session.root / 'mapping.json')]
        head_map = load_json(session.root / 'head_mapping.json')

        def manual_pose(q):
            for j in maps:
                raw = round(j.zero_tick + j.direction * (q[j.name] - j.offset_rad) * 4096 / math.tau)
                assert j.range_min <= raw <= j.range_max
                motors.registers[j.motor_id][56:58] = raw.to_bytes(2, 'little')
            for i, name in enumerate(('head_pan_joint', 'head_tilt_joint')):
                raw = round(head_map['reference_ticks'][i] + q[name] / (head_map['encoder_directions'][i]*head_map['encoder_scale_rad_per_tick']))
                motors.registers[13+i][56:58] = raw.to_bytes(2, 'little')

        manual_pose(poses[0])
        renderer = RenderedWrist(session, motors)
        monkeypatch.setattr(cv2, 'VideoCapture', lambda: renderer)
        bus, camera = TeachBus(session), CameraMeasurement(session)
        try:
            for number, pose in enumerate(poses, 1):
                bus.release()
                manual_pose(pose)
                bus.hold()
                measurement, image = camera.measure(bus.read)
                measurement['torque_hold_verified'] = bus.powered()
                session.commit(number, measurement, image)
        finally:
            camera.close()
            bus.close()
        # Above the raster/PnP noise floor; include actual encoder return error in ground truth.
        renderer.displacement = .020
        errors = []

        def stop_after_report():
            deadline = time.monotonic() + 120
            while time.monotonic() < deadline:
                reports = list((session.root / 'runs').glob('*/report.json'))
                for path in reports:
                    try:
                        report = load_json(path)
                    except json.JSONDecodeError:
                        continue
                    if report['status'] == 'completed':
                        try:
                            cli.request(session, 'stop')
                        except Exception as error:
                            errors.append(str(error))
                        return
                time.sleep(.1)
            errors.append('Replay did not complete within test deadline')
            try:
                cli.request(session, 'stop')
            except Exception:
                pass

        watcher = threading.Thread(target=stop_after_report, daemon=True)
        watcher.start()
        output = cli.replay(session, (1, 2, 3, 4, 5))
        watcher.join(20)
        assert not errors and not watcher.is_alive()
        report = load_json(output / 'report.json')
        assert report['status'] == 'completed'
        assert len(report['points']) == 5
        assert report['shutdown']['controllers_stopped']
        for point in report['points']:
            assert point['status'] == 'measured'
            taught = session.point(point['point'])
            arrival_file = output / f'{point["order"]:03d}_point_{point["point"]}' / 'measurement.json'
            arrival = load_json(arrival_file)
            assert max(abs(v) for v in point['joint_error_rad'].values()) <= control['controller']['goal_tolerance']
            expected = pose_error(renderer.truth(taught['positions_rad'], 0),
                                  renderer.truth(arrival['positions_rad'], renderer.displacement))
            # Raster corner localization induces several mm at this viewing distance.
            # Comparing vectors also checks frame direction and includes quantized FJT endpoints.
            measured_delta = np.array(point['board_video_tcp']['delta_xyz_mm'])
            assert np.linalg.norm(measured_delta - expected['delta_xyz_mm']) < 8
            assert point['board_video_tcp']['position_mm'] > point['encoder_tcp']['position_mm'] + 5
        expected = set(range(1, 6)) if side == 'left' else set(range(7, 12))
        assert {w['id'] for w in motors.writes} == expected
        assert {w['address'] for w in motors.writes} <= {40, 41, 42, 44, 46, 48}
        assert len((output / 'errors.csv').read_text().splitlines()) == 6
        for motor in expected:
            assert int.from_bytes(motors.registers[motor][48:50], 'little') == 800
            assert int.from_bytes(motors.registers[motor][46:48], 'little') == 800
            assert motors.registers[motor][41] == 254
    finally:
        motors.close()
