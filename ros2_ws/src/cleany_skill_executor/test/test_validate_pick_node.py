from __future__ import annotations

import threading
import time

import pytest
import rclpy
from action_msgs.msg import GoalStatus
from cleany_interfaces.action import InspectScene, SelectReachableGrasp, ValidatePick
from cleany_interfaces.msg import DetectedObject2D, DetectedObject3D, GraspCandidate
from cleany_interfaces.srv import PlanGrasp
from rclpy.action import ActionClient, ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.parameter import Parameter

from cleany_skill_executor.validate_pick_node import ValidatePickNode


def _wait(future, timeout=5.0):
    deadline = time.monotonic() + timeout
    while not future.done() and time.monotonic() < deadline:
        time.sleep(0.01)
    assert future.done()
    return future.result()


def _detection(object_id, label, depth, width, height):
    value = DetectedObject2D()
    value.object_id = object_id
    value.label = label
    value.confidence = 0.9
    value.x_max = 20.0
    value.y_max = 20.0
    value.median_depth_m = depth
    value.estimated_width_m = width
    value.estimated_height_m = height
    return value


def _candidate(object_id=2):
    value = GraspCandidate()
    value.header.frame_id = 'base_link'
    value.snapshot_id = 'snapshot-1'
    value.object_id = object_id
    value.tcp_pose.position.x = 0.5
    value.tcp_pose.position.y = 0.2
    value.tcp_pose.position.z = 0.8
    value.tcp_pose.orientation.w = 1.0
    value.approach_direction.y = -1.0
    value.required_opening_m = 0.04
    value.grasp_depth_m = 0.02
    value.score = 0.8
    value.target_object.object_id = object_id
    value.target_object.obb_pose.orientation.w = 1.0
    value.target_object.obb_size.x = 0.04
    value.target_object.obb_size.y = 0.04
    value.target_object.obb_size.z = 0.1
    return value


@pytest.mark.parametrize('failure_stage', ['', 'inspect', 'grasp', 'select', 'tf'])
def test_validate_pick_filters_before_sam_and_forwards_moveit_result(failure_stage):
    rclpy.init(args=[])
    fake = rclpy.create_node('validate_pick_fake_dependencies')
    client_node = rclpy.create_node('validate_pick_test_client')
    selected_ids = []
    required_arms = []
    group = ReentrantCallbackGroup()

    def inspect(goal_handle):
        result = InspectScene.Result()
        result.success = True
        result.error_code = result.ERROR_NONE
        result.detections.snapshot_id = 'snapshot-1'
        result.detections.detections = [
            _detection(1, 'keyboard', 0.2, 0.3, 0.12),
            _detection(2, 'can', 0.4, 0.05, 0.08),
            _detection(3, 'can', 0.5, 0.05, 0.08),
        ]
        if goal_handle.request.snapshot_id:
            selected_ids.append(goal_handle.request.selected_object_id)
            if goal_handle.request.selected_object_id == 2 and failure_stage in ('inspect', 'tf'):
                result.success = False
                result.error_code = result.ERROR_MASK if failure_stage == 'inspect' else result.ERROR_TF
                result.message = 'synthetic inspection failure'
                goal_handle.abort()
                return result
            obj = DetectedObject3D()
            obj.object_id = goal_handle.request.selected_object_id
            obj.label = 'can'
            obj.obb_pose.orientation.w = 1.0
            obj.obb_size.x = 0.04
            obj.obb_size.y = 0.04
            obj.obb_size.z = 0.1
            result.objects.header.frame_id = 'base_link'
            result.target_cloud.header.frame_id = 'camera_optical_frame'
            result.context_cloud.header.frame_id = 'camera_optical_frame'
            result.objects.objects = [obj]
        goal_handle.succeed()
        return result

    def grasp(_request, response):
        assert _request.target_object_header.frame_id == 'base_link'
        assert _request.context_cloud.header.frame_id == 'camera_optical_frame'
        response.success = True
        response.error_code = response.ERROR_NONE
        if _request.object_id == 2 and failure_stage == 'grasp':
            response.success = False
            response.error_code = response.ERROR_NO_GRASP_CANDIDATE
            response.message = 'synthetic no candidates'
        else:
            response.candidates = [_candidate(_request.object_id)]
        return response

    def select(goal_handle):
        required_arms.append(goal_handle.request.required_arm)
        result = SelectReachableGrasp.Result()
        result.success = True
        result.error_code = result.ERROR_NONE
        if goal_handle.request.candidates[0].object_id == 2 and failure_stage == 'select':
            result.success = False
            result.error_code = result.ERROR_NO_REACHABLE_GRASP
            result.message = 'synthetic unreachable target'
            goal_handle.abort()
            return result
        result.selected_candidate_index = 0
        result.selected_arm = 'left'
        result.selected_candidate = goal_handle.request.candidates[0]
        result.fk_pregrasp_pose.orientation.w = 1.0
        result.fk_grasp_pose.orientation.w = 1.0
        result.gripper_joint_position = 0.5
        result.planner_id = 'RRTConnectkConfigDefault'
        result.latency_stages = ['plan_grasp']
        result.latency_seconds = [0.1]
        goal_handle.succeed()
        return result

    inspect_server = ActionServer(
        fake, InspectScene, '/test/inspect', execute_callback=inspect,
        callback_group=group,
    )
    grasp_service = fake.create_service(
        PlanGrasp, '/test/grasp', grasp, callback_group=group
    )
    selection_server = ActionServer(
        fake, SelectReachableGrasp, '/test/select', execute_callback=select,
        callback_group=group,
    )
    node = ValidatePickNode(
        parameter_overrides=[
            Parameter('action_name', value='/test/validate'),
            Parameter('inspection_action_name', value='/test/inspect'),
            Parameter('grasp_service_name', value='/test/grasp'),
            Parameter('selection_action_name', value='/test/select'),
            Parameter('required_arm', value='left'),
            Parameter('server_wait_timeout_sec', value=1.0),
        ]
    )
    executor = MultiThreadedExecutor(num_threads=6)
    executor.add_node(fake)
    executor.add_node(node)
    executor.add_node(client_node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    try:
        client = ActionClient(client_node, ValidatePick, '/test/validate')
        assert client.wait_for_server(timeout_sec=2.0)
        goal_future = client.send_goal_async(ValidatePick.Goal())
        handle = _wait(goal_future, 2.0)
        assert handle is not None and handle.accepted
        result_future = handle.get_result_async()
        wrapped = _wait(result_future, 5.0)
        if failure_stage == 'tf':
            assert wrapped.status == GoalStatus.STATUS_ABORTED
            assert not wrapped.result.success
            assert selected_ids == [2]
            return
        assert wrapped.status == GoalStatus.STATUS_SUCCEEDED
        assert wrapped.result.success is True
        assert wrapped.result.selected_detection.object_id == (3 if failure_stage else 2)
        assert wrapped.result.selected_arm == 'left'
        assert wrapped.result.planner_id == 'RRTConnectkConfigDefault'
        assert selected_ids == ([2, 3] if failure_stage else [2])
        assert required_arms == (['left', 'left'] if failure_stage == 'select' else ['left'])
    finally:
        executor.shutdown()
        thread.join(timeout=2.0)
        inspect_server.destroy()
        selection_server.destroy()
        fake.destroy_service(grasp_service)
        node.destroy_node()
        fake.destroy_node()
        client_node.destroy_node()
        rclpy.shutdown()
