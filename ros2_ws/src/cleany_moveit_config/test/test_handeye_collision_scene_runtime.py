from __future__ import annotations

import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time
from uuid import uuid4

import pytest


if os.environ.get('ROS_DISTRO') is None:
    pytest.skip('ROS 2 environment is not active', allow_module_level=True)

from moveit_msgs.msg import PlanningSceneComponents
from moveit_msgs.srv import GetPlanningScene
import rclpy
from rclpy.node import Node


EXPECTED_PRIMITIVE_COUNTS = {
    'handeye_table': 1,
    'handeye_target_stand': 3,
    'charuco_target': 1,
}


def _log_tail(path: Path, lines: int = 120) -> str:
    try:
        return '\n'.join(
            path.read_text(encoding='utf-8', errors='replace').splitlines()[
                -lines:
            ]
        )
    except OSError as error:
        return f'<could not read launch log: {error}>'


def _stop(process: subprocess.Popen[bytes]) -> None:
    if process.poll() is not None:
        return
    os.killpg(process.pid, signal.SIGINT)
    try:
        process.wait(timeout=20.0)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGTERM)
        try:
            process.wait(timeout=10.0)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait(timeout=5.0)


@pytest.mark.skipif(
    os.environ.get('CLEANY_SKIP_ROS_RUNTIME_TESTS') == '1',
    reason='ROS runtime tests explicitly disabled',
)
def test_handeye_collision_objects_are_applied_to_moveit() -> None:
    domain_id = str(40 + uuid4().int % 150)
    environment = os.environ.copy()
    environment['ROS_DOMAIN_ID'] = domain_id
    previous_domain_id = os.environ.get('ROS_DOMAIN_ID')
    os.environ['ROS_DOMAIN_ID'] = domain_id
    try:
        with tempfile.TemporaryDirectory(
            prefix='cleany_moveit_scene_'
        ) as temp_dir:
            log_path = Path(temp_dir) / 'moveit.log'
            environment['ROS_LOG_DIR'] = str(Path(temp_dir) / 'ros_logs')
            with log_path.open('wb') as log_stream:
                moveit = subprocess.Popen(
                    [
                        'ros2',
                        'launch',
                        'cleany_moveit_config',
                        'mock_planning.launch.py',
                        'use_rviz:=false',
                        'use_sim_time:=false',
                    ],
                    cwd=temp_dir,
                    env=environment,
                    stdout=log_stream,
                    stderr=subprocess.STDOUT,
                    start_new_session=True,
                )
            rclpy.init()
            probe = Node('handeye_collision_scene_runtime_probe')
            try:
                client = probe.create_client(
                    GetPlanningScene, '/get_planning_scene'
                )
                deadline = time.monotonic() + 90.0
                while not client.wait_for_service(timeout_sec=0.5):
                    if moveit.poll() is not None:
                        pytest.fail(
                            'MoveIt exited before exposing planning scene\n'
                            + _log_tail(log_path)
                        )
                    if time.monotonic() >= deadline:
                        pytest.fail(
                            'timed out waiting for /get_planning_scene\n'
                            + _log_tail(log_path)
                        )

                applied = subprocess.run(
                    [
                        'ros2',
                        'run',
                        'cleany_moveit_config',
                        'apply_handeye_collision_scene',
                        '--ros-args',
                        '-p',
                        'service_wait_timeout_sec:=30.0',
                    ],
                    cwd=temp_dir,
                    env=environment,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.STDOUT,
                    timeout=60.0,
                    check=False,
                )
                assert applied.returncode == 0, (
                    applied.stdout.decode(errors='replace')
                    + '\nMoveIt log tail:\n'
                    + _log_tail(log_path)
                )

                request = GetPlanningScene.Request()
                request.components.components = (
                    PlanningSceneComponents.WORLD_OBJECT_NAMES
                    | PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
                )
                future = client.call_async(request)
                rclpy.spin_until_future_complete(
                    probe, future, timeout_sec=30.0
                )
                assert future.done(), _log_tail(log_path)
                assert future.exception() is None, _log_tail(log_path)
                response = future.result()
                assert response is not None, _log_tail(log_path)
                objects = {
                    item.id: item
                    for item in response.scene.world.collision_objects
                }
                assert set(objects) >= set(EXPECTED_PRIMITIVE_COUNTS), (
                    _log_tail(log_path)
                )
                for object_id, primitive_count in (
                    EXPECTED_PRIMITIVE_COUNTS.items()
                ):
                    collision_object = objects[object_id]
                    assert collision_object.header.frame_id == 'base_link'
                    assert len(collision_object.primitives) == primitive_count
                    assert (
                        len(collision_object.primitive_poses)
                        == primitive_count
                    )
            finally:
                probe.destroy_node()
                rclpy.shutdown()
                _stop(moveit)
    finally:
        if previous_domain_id is None:
            os.environ.pop('ROS_DOMAIN_ID', None)
        else:
            os.environ['ROS_DOMAIN_ID'] = previous_domain_id
