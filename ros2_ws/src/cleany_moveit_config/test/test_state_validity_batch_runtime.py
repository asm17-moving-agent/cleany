"""Compare the extension with native MoveIt: no motor or camera processes."""
import json
import os
import time
from copy import deepcopy

import pytest
from cleany_interfaces.srv import CheckStateValidityBatch
from moveit_msgs.msg import CollisionObject, Constraints, JointConstraint
from moveit_msgs.srv import ApplyPlanningScene, GetMotionPlan, GetPlannerParams, GetStateValidity
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose
from runtime_support import call, moveit_runtime
from test_head_collision_runtime import state

pytestmark = pytest.mark.skipif(os.environ.get('CLEANY_TEST_BATCH_VALIDITY') != '1',
    reason='Enable with make test-manipulation-speed')


@pytest.mark.parametrize('side', ['left', 'right'])
def test_return_planner_loads_fast_profile_and_rejects_a_blocked_scene(tmp_path, side):
    with moveit_runtime(tmp_path, head=True) as (node, process):
        planner_id = 'RRTConnectReturnkConfigDefault'
        params = node.create_client(GetPlannerParams, '/get_planner_params')
        response = call(node, process, params, GetPlannerParams.Request(
            pipeline_id='ompl', planner_config=planner_id, group=f'{side}_grasp_arm'))
        settings = dict(zip(response.params.keys, response.params.values))
        assert settings['simplify_solutions'] == '0'
        assert float(settings['longest_valid_segment_fraction']) == .001
        current = state()
        names = [f'{side}_{suffix}_joint' for suffix in (
            'shoulder_yaw', 'shoulder_pitch', 'elbow_pitch', 'wrist_pitch', 'wrist_roll')]
        target = dict(zip(current.joint_state.name, current.joint_state.position))
        target[names[0]] += .05
        planner = node.create_client(GetMotionPlan, '/plan_kinematic_path')
        request = GetMotionPlan.Request()
        m = request.motion_plan_request
        m.pipeline_id, m.planner_id, m.group_name = 'ompl', planner_id, f'{side}_grasp_arm'
        m.start_state = current
        m.num_planning_attempts, m.allowed_planning_time = 1, 4.
        # These plan-only times are discarded before the hardware retiming.
        m.max_velocity_scaling_factor = m.max_acceleration_scaling_factor = 1.
        m.goal_constraints = [Constraints(joint_constraints=[JointConstraint(
            joint_name=name, position=target[name], tolerance_above=.0001,
            tolerance_below=.0001, weight=1.) for name in names])]
        began = time.monotonic()
        result = call(node, process, planner, request).motion_plan_response
        elapsed = time.monotonic() - began
        assert result.error_code.val == result.error_code.SUCCESS
        trajectory = result.trajectory.joint_trajectory
        assert trajectory.points
        duration = trajectory.points[-1].time_from_start
        assert duration.sec + duration.nanosec/1e9 > 0
        samples = []
        for point in trajectory.points:
            sample = deepcopy(current)
            positions = {**dict(zip(sample.joint_state.name, sample.joint_state.position)),
                         **dict(zip(trajectory.joint_names, point.positions))}
            sample.joint_state.position = [positions[name] for name in sample.joint_state.name]
            samples.append(sample)
        batch = node.create_client(CheckStateValidityBatch, '/check_state_validity_batch')
        for start in range(0, len(samples), 32):
            checked = call(node, process, batch,
                CheckStateValidityBatch.Request(robot_states=samples[start:start+32]))
            assert checked.success and checked.first_invalid_index == -1
        (tmp_path/f'{side}-return-planning.json').write_text(json.dumps({
            'wall_sec': elapsed, 'planner_sec': result.planning_time,
            'waypoints': len(samples), 'settings': settings}, indent=2))
        obstacle = CollisionObject(id='blocked_return', operation=CollisionObject.ADD)
        obstacle.header.frame_id = 'base_link'
        obstacle.primitives = [SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[4., 4., 4.])]
        obstacle.primitive_poses = [Pose()]
        obstacle.primitive_poses[0].orientation.w = 1.
        update = ApplyPlanningScene.Request()
        update.scene.is_diff = True
        update.scene.world.collision_objects = [obstacle]
        apply = node.create_client(ApplyPlanningScene, '/apply_planning_scene')
        assert call(node, process, apply, update).success
        result = call(node, process, planner, request).motion_plan_response
        assert result.error_code.val != result.error_code.SUCCESS


def test_batch_matches_native_complete_scene_and_records_rpc_times(tmp_path):
    with moveit_runtime(tmp_path, head=True) as (node, process):
        native = node.create_client(GetStateValidity, '/check_state_validity')
        batch = node.create_client(CheckStateValidityBatch, '/check_state_validity_batch')
        sample = state()
        sample.is_diff = True
        baseline = call(node, process, native, GetStateValidity.Request(robot_state=sample))
        assert baseline.valid, [(c.contact_body_1, c.contact_body_2) for c in baseline.contacts]
        began = time.monotonic()
        for _ in range(128):
            assert call(node, process, native, GetStateValidity.Request(robot_state=sample)).valid
        native_sec = time.monotonic()-began
        began = time.monotonic()
        for _ in range(4):
            response = call(node, process, batch,
                CheckStateValidityBatch.Request(robot_states=[sample]*32))
            assert response.success, response.message
            assert list(response.valid) == [True]*32 and response.first_invalid_index == -1
        batch_sec = time.monotonic()-began
        (tmp_path/'batch-timings.json').write_text(json.dumps({
            'states': 128, 'native_requests': 128, 'batch_requests': 4,
            'native_sec': native_sec, 'batch_sec': batch_sec}, indent=2))
        # Place an actual obstacle in the head; both services must reject it.
        obstacle = CollisionObject(id='batch_head_obstacle', operation=CollisionObject.ADD)
        obstacle.header.frame_id = 'base_link'
        obstacle.primitives = [SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[.1, .1, .1])]
        obstacle.primitive_poses = [Pose()]
        obstacle.primitive_poses[0].position.z = .6
        obstacle.primitive_poses[0].orientation.w = 1.
        apply = node.create_client(ApplyPlanningScene, '/apply_planning_scene')
        request = ApplyPlanningScene.Request()
        request.scene.is_diff = True
        request.scene.world.collision_objects = [obstacle]
        assert call(node, process, apply, request).success
        single = call(node, process, native, GetStateValidity.Request(robot_state=sample))
        assert not single.valid
        result = call(node, process, batch, CheckStateValidityBatch.Request(robot_states=[sample]*32))
        assert result.success and result.first_invalid_index == 0 and list(result.valid) == [False]
        assert {(c.contact_body_1, c.contact_body_2) for c in result.contacts} == {
            (c.contact_body_1, c.contact_body_2) for c in single.contacts}
