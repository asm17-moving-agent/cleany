"""모터 드라이버 없이 설치된 검출기 선택과 내부에 포함된 장애물을 검사합니다."""
from contextlib import contextmanager
import json
import os
import re
from pathlib import Path
import signal
import subprocess
import time

import pytest

if not os.environ.get('ROS_DISTRO'):
    pytest.skip('ROS 2 environment is not active', allow_module_level=True)

from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, RobotState
from moveit_msgs.srv import ApplyPlanningScene, GetStateValidity
import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from shape_msgs.msg import SolidPrimitive


def _start(log, domain: int, detector: str | None, model_profile="cad_final"):
    command = ['ros2', 'launch', 'cleany_moveit_config', 'move_group.launch.py',
               'use_rviz:=false', 'allow_trajectory_execution:=false', 'model_profile:=' + model_profile]
    if detector is not None:
        command.append('collision_detector:=' + detector)
    return subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
        start_new_session=True, env={**os.environ, 'ROS_LOCALHOST_ONLY': '1',
                                    'ROS_DOMAIN_ID': str(domain)})


def _stop(process) -> None:
    if process.poll() is None:
        process.send_signal(signal.SIGINT)
        try:
            process.wait(timeout=15)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait(timeout=5)
            pytest.fail('MoveIt failed to stop after SIGINT')


@contextmanager
def _runtime(tmp_path: Path, detector: str | None, model_profile="cad_final"):
    domain = int(os.environ.get('CLEANY_TEST_ROS_DOMAIN_ID', 30 + os.getpid() % 180))
    context = Context()
    rclpy.init(args=[], context=context, domain_id=domain)
    node = rclpy.create_node('solid_collision_test', context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    log_path = tmp_path / 'move_group.log'
    with log_path.open('w') as log:
        process = _start(log, domain, detector, model_profile)
        try:
            def call(client, request):
                deadline = time.monotonic() + 40
                while not client.wait_for_service(timeout_sec=.1):
                    assert process.poll() is None, log_path.read_text()
                    assert time.monotonic() < deadline, log_path.read_text()
                future = client.call_async(request)
                while not future.done():
                    assert process.poll() is None, log_path.read_text()
                    assert time.monotonic() < deadline, log_path.read_text()
                    executor.spin_once(timeout_sec=.02)
                return future.result()
            yield node, call
        finally:
            _stop(process)
            executor.shutdown()
            node.destroy_node()
            context.shutdown()
    assert process.returncode == 0, log_path.read_text()
    expected = detector or 'cleany/SolidFCL'
    assert 'Verified collision detector: ' + expected in log_path.read_text()


@pytest.mark.parametrize('detector', [None, 'FCL'])
def test_installed_detector_catches_contained_obstacles(tmp_path: Path, detector):
    fixture = json.loads((Path(__file__).parent / 'data/contained_collision_probes.json').read_text())
    with _runtime(tmp_path, detector, "jetson_20261009") as (node, call):
        apply = node.create_client(ApplyPlanningScene, '/apply_planning_scene')
        validity = node.create_client(GetStateValidity, '/check_state_validity')
        for row in fixture['probes']:
            state = RobotState()
            state.joint_state.name = fixture['joint_names']
            state.joint_state.position = fixture['poses'][row['pose_index']]
            query = GetStateValidity.Request(robot_state=state)
            assert call(validity, query).valid, row['source_index']
            obstacle = CollisionObject(id='contained_probe', operation=CollisionObject.ADD)
            obstacle.header.frame_id = 'base_link'
            obstacle.primitives = [SolidPrimitive(type=SolidPrimitive.SPHERE,
                                                 dimensions=[row['sphere']['radius']])]
            pose = Pose()
            pose.orientation.w = 1.
            pose.position.x, pose.position.y, pose.position.z = row['sphere']['center']
            obstacle.primitive_poses = [pose]
            update = ApplyPlanningScene.Request()
            update.scene.is_diff = True
            update.scene.world.collision_objects = [obstacle]
            assert call(apply, update).success
            result = call(validity, query)
            if detector is None:
                assert not result.valid, row['source_index']
                links = {body for contact in result.contacts
                         for body in (contact.contact_body_1, contact.contact_body_2)
                         if 'contained_probe' in (contact.contact_body_1, contact.contact_body_2)}
                assert set(row['expected_links']) <= links, row['source_index']
            else:
                assert result.valid, row['source_index']
            obstacle.operation = CollisionObject.REMOVE
            update.scene.world.collision_objects = [obstacle]
            assert call(apply, update).success


def test_unavailable_detector_stops_planning(tmp_path: Path):
    log_path = tmp_path / 'rejected_detector.log'
    with log_path.open('w') as log:
        process = _start(log, int(os.environ.get('CLEANY_TEST_ROS_DOMAIN_ID', 30 + os.getpid() % 180)), 'cleany/MissingDetector')
        try:
            process.wait(timeout=40)
        finally:
            _stop(process)
    assert 'Requested collision detector cleany/MissingDetector did not load' in log_path.read_text()
    assert not re.search(r'\[INFO\].*Verified collision detector:', log_path.read_text())


def test_latest_cad_panel_detects_contained_obstacle(tmp_path: Path):
    manifest = json.loads((Path(__file__).parents[2] /
        'cleany_description/config/geometry/cad_final.json').read_text())
    with _runtime(tmp_path, None) as (node, call):
        validity = node.create_client(GetStateValidity, '/check_state_validity')
        apply = node.create_client(ApplyPlanningScene, '/apply_planning_scene')
        state = RobotState()
        state.joint_state.name = list(manifest['registration_angles_rad'])
        state.joint_state.position = list(manifest['registration_angles_rad'].values())
        query = GetStateValidity.Request(robot_state=state)
        baseline = call(validity, query)
        assert baseline.valid, str(baseline.contacts)
        obstacle = CollisionObject(id='cad_panel_probe', operation=CollisionObject.ADD)
        obstacle.header.frame_id = 'base_link'
        obstacle.primitives = [SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[.0005])]
        pose = Pose()
        pose.orientation.w = 1.
        # 최신 측면 패널 내부의 CAD 좌표 (220, -202.5, 400)mm입니다.
        pose.position.x, pose.position.y, pose.position.z = (.202475, .220027, .115503)
        obstacle.primitive_poses = [pose]
        update = ApplyPlanningScene.Request()
        update.scene.is_diff = True
        update.scene.world.collision_objects = [obstacle]
        assert call(apply, update).success
        result = call(validity, query)
        assert not result.valid
        pairs = [{c.contact_body_1, c.contact_body_2} for c in result.contacts]
        assert {'base_link', 'cad_panel_probe'} in pairs
