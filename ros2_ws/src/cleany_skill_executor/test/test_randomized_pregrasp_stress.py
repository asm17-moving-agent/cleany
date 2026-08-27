from __future__ import annotations

import json
import math
import os
from pathlib import Path
import random
import signal
import subprocess
import tempfile
import time
from uuid import uuid4

import pytest


if os.environ.get('ROS_DISTRO') is None:
    pytest.skip('ROS 2 environment is not active', allow_module_level=True)

from action_msgs.msg import GoalStatus
from cleany_interfaces.action import SelectReachableGrasp
from cleany_interfaces.msg import GraspCandidate
from moveit_msgs.msg import CollisionObject, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive

from cleany_skill_executor.grasp_execution_demo import GraspExecutionDemo


ITERATIONS = int(os.environ.get('CLEANY_RANDOM_STRESS_ITERATIONS', '0'))
SEED = int(os.environ.get('CLEANY_RANDOM_STRESS_SEED', '20260826'))


def _matrix_to_quaternion(matrix: tuple[tuple[float, ...], ...]) -> tuple[float, ...]:
    m00, m01, m02 = matrix[0]
    m10, m11, m12 = matrix[1]
    m20, m21, m22 = matrix[2]
    trace = m00 + m11 + m22
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        return (
            (m21 - m12) / scale,
            (m02 - m20) / scale,
            (m10 - m01) / scale,
            0.25 * scale,
        )
    diagonal = (m00, m11, m22)
    index = max(range(3), key=diagonal.__getitem__)
    i, j, k = ((0, 1, 2), (1, 2, 0), (2, 0, 1))[index]
    rows = matrix
    scale = math.sqrt(1.0 + rows[i][i] - rows[j][j] - rows[k][k]) * 2.0
    result = [0.0, 0.0, 0.0, (rows[k][j] - rows[j][k]) / scale]
    result[i] = 0.25 * scale
    result[j] = (rows[i][j] + rows[j][i]) / scale
    result[k] = (rows[i][k] + rows[k][i]) / scale
    return tuple(result)


def _orientation(
    approach: tuple[float, float, float], closing_hint: tuple[float, float, float]
) -> tuple[float, float, float, float]:
    approach_norm = math.sqrt(sum(value * value for value in approach))
    approach = tuple(value / approach_norm for value in approach)
    projection = sum(a * c for a, c in zip(approach, closing_hint))
    closing = tuple(c - projection * a for a, c in zip(approach, closing_hint))
    closing_norm = math.sqrt(sum(value * value for value in closing))
    closing = tuple(value / closing_norm for value in closing)
    local_y = tuple(-value for value in approach)
    local_z = (
        closing[1] * local_y[2] - closing[2] * local_y[1],
        closing[2] * local_y[0] - closing[0] * local_y[2],
        closing[0] * local_y[1] - closing[1] * local_y[0],
    )
    matrix = tuple(
        (closing[row], local_y[row], local_z[row]) for row in range(3)
    )
    return _matrix_to_quaternion(matrix)


def _candidate_messages(
    *,
    iteration: int,
    label: str,
    position: tuple[float, float, float],
    size: tuple[float, float, float],
) -> list[GraspCandidate]:
    grasp_depth_m = 0.015
    messages: list[GraspCandidate] = []
    for index, closing_yaw_deg in enumerate(range(0, 180, 15)):
        tilt = math.radians(16.0)
        approach = (
            math.sin(tilt),
            0.0,
            -math.cos(tilt),
        )
        tcp_position = tuple(
            value - grasp_depth_m * component
            for value, component in zip(position, approach)
        )
        yaw = math.radians(closing_yaw_deg)
        orientation = _orientation(
            approach,
            (math.cos(yaw), math.sin(yaw), 0.0),
        )
        candidate = GraspCandidate()
        candidate.header.frame_id = 'base_link'
        candidate.snapshot_id = f'random-stress-{iteration}'
        candidate.object_id = iteration + 1
        candidate.tcp_pose.position.x = tcp_position[0]
        candidate.tcp_pose.position.y = tcp_position[1]
        candidate.tcp_pose.position.z = tcp_position[2]
        (
            candidate.tcp_pose.orientation.x,
            candidate.tcp_pose.orientation.y,
            candidate.tcp_pose.orientation.z,
            candidate.tcp_pose.orientation.w,
        ) = orientation
        (
            candidate.approach_direction.x,
            candidate.approach_direction.y,
            candidate.approach_direction.z,
        ) = approach
        candidate.required_opening_m = min(size[0], size[1]) + 0.01
        candidate.grasp_depth_m = grasp_depth_m
        candidate.score = 1.0 - index * 0.05
        candidate.target_object.object_id = candidate.object_id
        candidate.target_object.label = label
        candidate.target_object.confidence = 1.0
        candidate.target_object.obb_pose.position.x = position[0]
        candidate.target_object.obb_pose.position.y = position[1]
        candidate.target_object.obb_pose.position.z = position[2]
        candidate.target_object.obb_pose.orientation.w = 1.0
        (
            candidate.target_object.obb_size.x,
            candidate.target_object.obb_size.y,
            candidate.target_object.obb_size.z,
        ) = size
        messages.append(candidate)
    return messages


class StressProbe(GraspExecutionDemo):
    def __init__(self) -> None:
        super().__init__('randomized_pregrasp_stress_probe')
        self._apply_scene = self.create_client(
            ApplyPlanningScene, '/apply_planning_scene'
        )

    def apply_distractor(
        self,
        *,
        object_id: str,
        label: str,
        position: tuple[float, float, float],
        size: tuple[float, float, float],
    ) -> None:
        collision = CollisionObject()
        collision.header.frame_id = 'base_link'
        collision.id = object_id
        collision.operation = CollisionObject.ADD
        primitive = SolidPrimitive()
        primitive.type = (
            SolidPrimitive.CYLINDER if label == 'can' else SolidPrimitive.BOX
        )
        primitive.dimensions = (
            [size[2], size[0] / 2.0]
            if label == 'can'
            else list(size)
        )
        collision.primitives = [primitive]
        pose = candidate_pose(position)
        collision.primitive_poses = [pose]
        self._apply_collision(collision)

    def remove_object(self, object_id: str) -> None:
        collision = CollisionObject()
        collision.id = object_id
        collision.operation = CollisionObject.REMOVE
        self._apply_collision(collision)

    def _apply_collision(self, collision: CollisionObject) -> None:
        if not self._apply_scene.wait_for_service(timeout_sec=5.0):
            raise RuntimeError('planning scene service unavailable')
        scene = PlanningScene()
        scene.is_diff = True
        scene.robot_state.is_diff = True
        scene.world.collision_objects = [collision]
        request = ApplyPlanningScene.Request()
        request.scene = scene
        response = self._future(
            self._apply_scene.call_async(request), 5.0, 'planning scene update'
        )
        if not response.success:
            raise RuntimeError('planning scene update failed')


def candidate_pose(position: tuple[float, float, float]):
    from geometry_msgs.msg import Pose

    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = position
    pose.orientation.w = 1.0
    return pose


def _stop_launch(process: subprocess.Popen[bytes]) -> None:
    if process.poll() is not None:
        return
    os.killpg(process.pid, signal.SIGINT)
    try:
        process.wait(timeout=20.0)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGTERM)
        process.wait(timeout=10.0)


@pytest.mark.skipif(
    ITERATIONS <= 0,
    reason='set CLEANY_RANDOM_STRESS_ITERATIONS to run the stress test',
)
def test_randomized_box_and_can_pregrasp_stress() -> None:
    rng = random.Random(SEED)
    domain_id = 20 + uuid4().int % 180
    previous_domain = os.environ.get('ROS_DOMAIN_ID')
    os.environ['ROS_DOMAIN_ID'] = str(domain_id)
    rclpy.init(args=[])
    probe = StressProbe()
    records: list[dict[str, object]] = []
    try:
        with tempfile.TemporaryDirectory(
            prefix='cleany_randomized_pregrasp_'
        ) as temporary_directory:
            root = Path(temporary_directory)
            log_path = root / 'launch.log'
            result_path = Path(
                os.environ.get(
                    'CLEANY_RANDOM_STRESS_RESULT',
                    str(Path.cwd() / 'randomized_pregrasp_stress.json'),
                )
            )
            environment = os.environ.copy()
            environment.update(
                {'ROS_HOME': str(root / 'ros_home'), 'ROS_LOG_DIR': str(root / 'ros_log')}
            )
            with log_path.open('wb') as launch_log:
                process = subprocess.Popen(
                    [
                        'ros2', 'launch', 'cleany_skill_executor',
                        'randomized_pregrasp_stress.launch.py',
                    ],
                    env=environment,
                    cwd=root,
                    stdout=launch_log,
                    stderr=subprocess.STDOUT,
                    start_new_session=True,
                )
                try:
                    assert probe._selection.wait_for_server(timeout_sec=60.0)
                    assert probe._move_group.wait_for_server(timeout_sec=60.0)
                    probe._wait_for_joint_state(60.0)
                    initial = dict(probe._joint_positions)
                    for iteration in range(ITERATIONS):
                        started = time.monotonic()
                        box = (
                            rng.uniform(0.435, 0.465),
                            rng.uniform(-0.20, -0.14),
                            0.385,
                        )
                        can = (
                            rng.uniform(0.435, 0.465),
                            rng.uniform(0.14, 0.20),
                            0.395,
                        )
                        target_label = 'box' if iteration % 2 == 0 else 'can'
                        target = box if target_label == 'box' else can
                        target_size = (0.08, 0.06, 0.08) if target_label == 'box' else (0.07, 0.07, 0.10)
                        other_label = 'can' if target_label == 'box' else 'box'
                        other = can if target_label == 'box' else box
                        other_size = (0.07, 0.07, 0.10) if other_label == 'can' else (0.08, 0.06, 0.08)
                        distractor_id = f'random_stress_distractor_{iteration}'
                        record: dict[str, object] = {
                            'iteration': iteration + 1,
                            'box': box,
                            'can': can,
                            'target': target_label,
                        }
                        try:
                            probe.apply_distractor(
                                object_id=distractor_id,
                                label=other_label,
                                position=other,
                                size=other_size,
                            )
                            goal = SelectReachableGrasp.Goal()
                            goal.candidates = _candidate_messages(
                                iteration=iteration,
                                label=target_label,
                                position=target,
                                size=target_size,
                            )
                            selection_started = time.monotonic()
                            handle = probe._future(
                                probe._selection.send_goal_async(goal),
                                10.0,
                                'randomized selection goal',
                            )
                            wrapped = probe._future(
                                handle.get_result_async(),
                                130.0,
                                'randomized selection result',
                            )
                            record['selection_sec'] = time.monotonic() - selection_started
                            result = wrapped.result
                            if wrapped.status != GoalStatus.STATUS_SUCCEEDED or not result.success:
                                record.update(
                                    success=False,
                                    stage='selection',
                                    error_code=int(result.error_code),
                                    message=result.message,
                                )
                                continue
                            execution_started = time.monotonic()
                            probe._move_to(
                                result.selected_arm,
                                result.pregrasp_joint_state,
                                f'random-pregrasp-{iteration + 1}',
                            )
                            probe._verify_feedback(result.pregrasp_joint_state)
                            record['execution_sec'] = time.monotonic() - execution_started
                            return_state = type(result.pregrasp_joint_state)()
                            return_state.name = list(result.pregrasp_joint_state.name)
                            return_state.position = [initial[name] for name in return_state.name]
                            probe._move_to(
                                result.selected_arm,
                                return_state,
                                f'random-return-{iteration + 1}',
                            )
                            probe._verify_feedback(return_state)
                            record.update(
                                success=True,
                                arm=result.selected_arm,
                                candidate=int(result.selected_candidate_index),
                            )
                        except Exception as error:
                            record.update(success=False, stage='runtime', message=str(error))
                        finally:
                            try:
                                probe.remove_object(distractor_id)
                            except Exception as error:
                                record['cleanup_error'] = str(error)
                            record['total_sec'] = time.monotonic() - started
                            records.append(record)
                            print('RANDOM_STRESS ' + json.dumps(record), flush=True)
                finally:
                    _stop_launch(process)

            successes = sum(bool(item.get('success')) for item in records)
            report = {
                'seed': SEED,
                'iterations': ITERATIONS,
                'successes': successes,
                'failures': ITERATIONS - successes,
                'success_rate': successes / ITERATIONS,
                'records': records,
                'launch_log_tail': log_path.read_text(
                    encoding='utf-8', errors='replace'
                )[-20000:],
            }
            result_path.write_text(
                json.dumps(report, indent=2, ensure_ascii=False), encoding='utf-8'
            )
            assert len(records) == ITERATIONS
    finally:
        probe.destroy_node()
        rclpy.shutdown()
        if previous_domain is None:
            os.environ.pop('ROS_DOMAIN_ID', None)
        else:
            os.environ['ROS_DOMAIN_ID'] = previous_domain
