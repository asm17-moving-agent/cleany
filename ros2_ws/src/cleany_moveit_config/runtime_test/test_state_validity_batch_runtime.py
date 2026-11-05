"""Compare the extension with native MoveIt: no motor or camera processes."""
import json
import math
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


def timed_path_samples(current, trajectory):
    """Check timing and sample the position/velocity/acceleration quintics."""
    times = [p.time_from_start.sec + p.time_from_start.nanosec/1e9 for p in trajectory.points]
    assert times[0] == 0. and all(b > a for a, b in zip(times, times[1:]))
    assert all(math.isfinite(v) for p in trajectory.points
               for values in (p.positions, p.velocities, p.accelerations) for v in values)
    samples = []
    def add(positions):
        sample = deepcopy(current)
        merged = {**dict(zip(sample.joint_state.name, sample.joint_state.position)),
                  **dict(zip(trajectory.joint_names, positions))}
        sample.joint_state.position = [merged[name] for name in sample.joint_state.name]
        samples.append(sample)
    add(trajectory.points[0].positions)
    for i, (first, last) in enumerate(zip(trajectory.points, trajectory.points[1:])):
        dt = times[i+1]-times[i]
        assert len(first.velocities) == len(first.accelerations) == len(trajectory.joint_names)
        assert len(last.velocities) == len(last.accelerations) == len(trajectory.joint_names)
        coefficients = []
        for q0, v0, a0, q1, v1, a1 in zip(first.positions, first.velocities, first.accelerations,
                                         last.positions, last.velocities, last.accelerations):
            c0, c1, c2 = q0, v0*dt, a0*dt*dt/2
            dq, dv, da = q1-c0-c1-c2, v1*dt-c1-2*c2, a1*dt*dt-2*c2
            coefficients.append((c0, c1, c2, 10*dq-4*dv+da/2, -15*dq+7*dv-da, 6*dq-3*dv+da/2))
        # Bernstein coefficients bound the quartic derivative on [0, 1].
        bound = max(abs(sum((k+1)*c[k+1]*math.comb(i, k)/math.comb(4, k)
                            for k in range(i+1))) for c in coefficients for i in range(5))
        count = max(2, math.ceil(bound/.01))
        for step in range(1, count+1):
            u = step/count
            add([sum(value*u**power for power, value in enumerate(c)) for c in coefficients])
    return samples


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
        assert all(abs(value-target[name]) <= .00011
                   for name, value in zip(trajectory.joint_names, trajectory.points[-1].positions))
        samples = timed_path_samples(current, trajectory)
        batch = node.create_client(CheckStateValidityBatch, '/check_state_validity_batch')
        for start in range(0, len(samples), 32):
            checked = call(node, process, batch,
                CheckStateValidityBatch.Request(robot_states=samples[start:start+32]))
            assert checked.success and checked.first_invalid_index == -1
        (tmp_path/f'{side}-return-planning.json').write_text(json.dumps({
            'wall_sec': elapsed, 'planner_sec': result.planning_time,
            'waypoints': len(trajectory.points), 'checked_samples': len(samples),
            'duration_sec': duration.sec + duration.nanosec/1e9, 'settings': settings}, indent=2))
        invalid = deepcopy(request)
        invalid.motion_plan_request.max_velocity_scaling_factor = -1.
        rejected = call(node, process, planner, invalid).motion_plan_response
        assert rejected.error_code.val == rejected.error_code.INVALID_MOTION_PLAN
        assert not rejected.trajectory.joint_trajectory.points
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
