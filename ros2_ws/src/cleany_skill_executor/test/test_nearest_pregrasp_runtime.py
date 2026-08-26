from __future__ import annotations

import os
from pathlib import Path
import signal
import subprocess
import tempfile
import threading
import time
from uuid import uuid4

import pytest


if os.environ.get('ROS_DISTRO') is None:
    pytest.skip('ROS 2 environment is not active', allow_module_level=True)

from cleany_interfaces.action import InspectScene
from cleany_interfaces.msg import (
    DetectedObject2D,
    DetectedObject3D,
    GraspCandidate,
)
from cleany_interfaces.srv import PlanGrasp
import rclpy
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from cleany_skill_executor.core.grasp_selection import quaternion_axis


TARGET = (0.296499, 0.521873, 0.705470)
ORIENTATION = (-0.308136, -0.164442, 0.887690, 0.300028)
APPROACH = quaternion_axis(ORIENTATION, (0.0, -1.0, 0.0))


class FakeNearestObjectProviders(Node):
    def __init__(self) -> None:
        super().__init__('fake_nearest_object_providers')
        self.inspected_ids: list[int] = []
        self._callbacks = ReentrantCallbackGroup()
        self._inspection = ActionServer(
            self,
            InspectScene,
            '/perception/inspect_scene',
            execute_callback=self._inspect,
            callback_group=self._callbacks,
        )
        self._grasp = self.create_service(
            PlanGrasp,
            '/grasp/plan',
            self._plan,
            callback_group=self._callbacks,
        )

    def destroy_node(self) -> None:
        self._inspection.destroy()
        super().destroy_node()

    def _inspect(self, goal_handle):
        result = InspectScene.Result()
        if not goal_handle.request.snapshot_id:
            result.success = True
            result.error_code = InspectScene.Result.ERROR_NONE
            result.message = 'two ranked objects'
            result.detections.snapshot_id = 'nearest-runtime-snapshot'
            result.detections.header.frame_id = 'camera_optical_frame'
            result.detections.detections = [
                self._detection(1, 'nearest-unsegmentable', 0.30, 0.95),
                self._detection(2, 'reachable-box', 0.55, 0.85),
            ]
            goal_handle.succeed()
            return result

        object_id = int(goal_handle.request.selected_object_id)
        self.inspected_ids.append(object_id)
        if object_id == 1:
            result.success = False
            result.error_code = InspectScene.Result.ERROR_MASK
            result.message = 'synthetic nearest-object mask failure'
            goal_handle.abort()
            return result

        result.success = True
        result.error_code = InspectScene.Result.ERROR_NONE
        result.message = 'reachable object inspected'
        result.objects.snapshot_id = goal_handle.request.snapshot_id
        result.objects.header.frame_id = 'base_link'
        result.objects.objects = [self._target_object(object_id)]
        result.target_cloud.header.frame_id = 'base_link'
        result.context_cloud.header.frame_id = 'base_link'
        goal_handle.succeed()
        return result

    @staticmethod
    def _detection(
        object_id: int,
        label: str,
        distance_m: float,
        confidence: float,
    ) -> DetectedObject2D:
        result = DetectedObject2D()
        result.object_id = object_id
        result.label = label
        result.confidence = confidence
        result.distance_valid = True
        result.distance_m = distance_m
        result.x_max = 10.0
        result.y_max = 10.0
        return result

    @staticmethod
    def _target_object(object_id: int) -> DetectedObject3D:
        result = DetectedObject3D()
        result.object_id = object_id
        result.label = 'reachable-box'
        result.confidence = 0.85
        result.obb_pose.position.x = TARGET[0]
        result.obb_pose.position.y = TARGET[1]
        result.obb_pose.position.z = TARGET[2]
        result.obb_pose.orientation.w = 1.0
        result.obb_size.x = 0.03
        result.obb_size.y = 0.03
        result.obb_size.z = 0.03
        return result

    def _plan(self, request, response):
        response.success = True
        response.error_code = PlanGrasp.Response.ERROR_NONE
        response.message = 'synthetic reachable candidate'
        candidate = GraspCandidate()
        candidate.header.frame_id = 'base_link'
        candidate.snapshot_id = request.snapshot_id
        candidate.object_id = request.object_id
        (
            candidate.tcp_pose.position.x,
            candidate.tcp_pose.position.y,
            candidate.tcp_pose.position.z,
        ) = TARGET
        (
            candidate.tcp_pose.orientation.x,
            candidate.tcp_pose.orientation.y,
            candidate.tcp_pose.orientation.z,
            candidate.tcp_pose.orientation.w,
        ) = ORIENTATION
        (
            candidate.approach_direction.x,
            candidate.approach_direction.y,
            candidate.approach_direction.z,
        ) = APPROACH
        candidate.required_opening_m = 0.03
        candidate.grasp_depth_m = 0.015
        candidate.score = 0.9
        candidate.target_object = request.target_object
        response.candidates = [candidate]
        return response

def _log_text(path: Path) -> str:
    try:
        return path.read_text(encoding='utf-8', errors='replace')
    except OSError:
        return ''


def _stop_launch(process: subprocess.Popen[bytes]) -> None:
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


def test_nearest_failure_falls_back_and_executes_pregrasp() -> None:
    domain_id = 20 + uuid4().int % 180
    previous_domain_id = os.environ.get('ROS_DOMAIN_ID')
    os.environ['ROS_DOMAIN_ID'] = str(domain_id)
    rclpy.init(args=[])
    provider = FakeNearestObjectProviders()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(provider)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    try:
        with tempfile.TemporaryDirectory(
            prefix='cleany_nearest_pregrasp_test_'
        ) as temporary_directory:
            temporary_root = Path(temporary_directory)
            log_path = temporary_root / 'launch.log'
            environment = os.environ.copy()
            environment.update(
                {
                    'ROS_HOME': str(temporary_root / 'ros_home'),
                    'ROS_LOG_DIR': str(temporary_root / 'ros_log'),
                }
            )
            with log_path.open('wb') as launch_log:
                process = subprocess.Popen(
                    [
                        'ros2',
                        'launch',
                        'cleany_skill_executor',
                        'nearest_pregrasp_runtime.launch.py',
                    ],
                    env=environment,
                    cwd=temporary_root,
                    stdout=launch_log,
                    stderr=subprocess.STDOUT,
                    start_new_session=True,
                )
                try:
                    deadline = time.monotonic() + 150.0
                    while time.monotonic() < deadline:
                        text = _log_text(log_path)
                        if process.poll() is not None:
                            pytest.fail(
                                'nearest pre-grasp launch exited early\n'
                                + text[-12000:]
                            )
                        if 'NEAREST PREGRASP FAILED' in text:
                            pytest.fail(
                                'nearest pre-grasp reported failure\n'
                                + text[-12000:]
                            )
                        if 'NEAREST PREGRASP COMPLETE:' in text:
                            break
                        time.sleep(0.1)
                    else:
                        pytest.fail(
                            'timed out waiting for nearest pre-grasp\n'
                            + _log_text(log_path)[-12000:]
                        )
                finally:
                    _stop_launch(process)

            text = _log_text(log_path)
            assert provider.inspected_ids == [1, 2]
            assert 'Trying nearest object_id=1' in text
            assert 'Skipping object 1: inspection failed' in text
            assert 'Trying nearest object_id=2' in text
            assert (
                'MoveIt execution succeeded: nearest-object pre-grasp'
                in text
            )
            assert 'object_id=2 label=reachable-box' in text
    finally:
        executor.shutdown()
        spin_thread.join(timeout=2.0)
        provider.destroy_node()
        rclpy.shutdown()
        if previous_domain_id is None:
            os.environ.pop('ROS_DOMAIN_ID', None)
        else:
            os.environ['ROS_DOMAIN_ID'] = previous_domain_id
