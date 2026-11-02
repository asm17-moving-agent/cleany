"""Check installed detector selection and contained obstacles without motor drivers."""
from contextlib import contextmanager
import json
import os
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


def _start(log, domain: int, detector: str | None):
    command = ['ros2', 'launch', 'cleany_moveit_config', 'move_group.launch.py',
               'use_rviz:=false', 'allow_trajectory_execution:=false']
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
def _runtime(tmp_path: Path, detector: str | None):
    domain = 30 + os.getpid() % 180
    context = Context()
    rclpy.init(args=[], context=context, domain_id=domain)
    node = rclpy.create_node('solid_collision_test', context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    log_path = tmp_path / 'move_group.log'
    with log_path.open('w') as log:
        process = _start(log, domain, detector)
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
    with _runtime(tmp_path, detector) as (node, call):
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
        process = _start(log, 30 + os.getpid() % 180, 'cleany/MissingDetector')
        try:
            process.wait(timeout=40)
        finally:
            _stop(process)
    assert 'Requested collision detector cleany/MissingDetector did not load' in log_path.read_text()
    assert 'Verified collision detector:' not in log_path.read_text()
