"""Opt-in native hybrid IK/planning/controller checks, without motor drivers.

Targets are synthesized from the runtime SO-101 model. cuRobo, MoveIt, DDS,
ros2_control GenericSystem and arm trajectory controllers are real processes.
This does not execute perception networks or verify physical grasping.
"""
from copy import deepcopy
import json
import os
import time

import pytest

from test_curobo_gpu import ros_node  # Isolated DDS domain; mock hardware only.
from cleany_skill_executor.core.physical_grasp_selection import (
    ARM_JOINT_NAMES, JointSolution, PoseValue, PregraspRequest,
)
from cleany_skill_executor.hybrid_adapter import HybridGraspAdapter
from cleany_skill_executor.physical_moveit_adapter import MoveItAdapterConfig
from cleany_skill_executor.trajectory_io import path_to_message

pytestmark = pytest.mark.skipif(
    os.environ.get('CLEANY_RUN_CUROBO') != '1',
    reason='Enable with make test-motion-hybrid-gpu in the pinned CUDA environment',
)


def wait(node, future, timeout=30.):
    import rclpy
    end = time.monotonic()+timeout
    while not future.done():
        assert time.monotonic() < end, 'Native ROS operation timed out'
        rclpy.spin_once(node, timeout_sec=.01)
    return future.result()


@pytest.fixture(scope='module')
def native(ros_node):
    from control_msgs.action import FollowJointTrajectory
    from controller_manager_msgs.srv import ListHardwareComponents
    from rclpy.action import ActionClient
    from rclpy.qos import DurabilityPolicy, QoSProfile
    from sensor_msgs.msg import JointState
    from std_msgs.msg import String
    import rclpy

    description, measured = [], []
    subscriptions = [
        ros_node.create_subscription(String, '/robot_description',
            lambda msg: description.append(msg.data),
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)),
        ros_node.create_subscription(JointState, '/joint_states',
            lambda msg: measured.__setitem__(slice(None), [msg]), 10),
    ]
    actions = {arm: ActionClient(ros_node, FollowJointTrajectory,
        f'/{arm}_arm_controller/follow_joint_trajectory') for arm in ARM_JOINT_NAMES}
    hardware = ros_node.create_client(ListHardwareComponents,
        '/controller_manager/list_hardware_components')
    assert hardware.wait_for_service(timeout_sec=30.)
    components = wait(ros_node, hardware.call_async(ListHardwareComponents.Request())).component
    assert len(components) == 1
    assert components[0].class_type == 'mock_components/GenericSystem'
    assert components[0].state.label == 'active'
    assert all(client.wait_for_server(timeout_sec=30.) for client in actions.values())
    end = time.monotonic()+30.
    while not description or not measured:
        assert time.monotonic() < end
        rclpy.spin_once(ros_node, timeout_sec=.02)
    adapter = HybridGraspAdapter(ros_node, MoveItAdapterConfig(
        use_verified_local_fk=True, align_wrist_roll=True,
        pose_refinement_iterations=10, validate_motion_paths=True,
        planning_timeout_sec=10., planning_attempts=3,
        velocity_scaling=.5, acceleration_scaling=.5))
    adapter.set_robot_description(description[-1])
    assert set(adapter._state_joint_names) == {
        *ARM_JOINT_NAMES['left'], *ARM_JOINT_NAMES['right'],
        'left_gripper_joint', 'right_gripper_joint', 'head_pan_joint', 'head_tilt_joint'}
    adapter.set_deadline(time.monotonic()+600.)
    try:
        yield ros_node, adapter, measured, actions
    finally:
        adapter.close()
        for client in actions.values():
            client.destroy()
        for subscription in subscriptions:
            ros_node.destroy_subscription(subscription)
        ros_node.destroy_client(hardware)


def execute(node, client, trajectory, measured):
    from control_msgs.action import FollowJointTrajectory
    from action_msgs.msg import GoalStatus
    import rclpy

    feedback = []
    goal = FollowJointTrajectory.Goal(trajectory=trajectory)
    handle = wait(node, client.send_goal_async(goal,
        feedback_callback=lambda msg: feedback.append(msg.feedback)))
    assert handle.accepted
    result = wait(node, handle.get_result_async(), timeout=45.)
    assert result.status == GoalStatus.STATUS_SUCCEEDED
    assert result.result.error_code == FollowJointTrajectory.Result.SUCCESSFUL
    expected = dict(zip(trajectory.joint_names, trajectory.points[-1].positions))
    end = time.monotonic()+3.
    while True:
        actual = dict(zip(measured[-1].name, measured[-1].position))
        error = max(abs(actual[name]-value) for name, value in expected.items())
        if error <= .002 and feedback:
            return error, len(feedback)
        assert time.monotonic() < end, 'Controller result lacks measured arrival/feedback'
        rclpy.spin_once(node, timeout_sec=.01)


@pytest.mark.parametrize('arm', ['left', 'right'])
def test_native_batch8_seed16_cpu_plans_and_mock_controller(native, arm, tmp_path,
                                                          record_property):
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
    node, adapter, measured, actions = native
    start = JointSolution(ARM_JOINT_NAMES[arm], (0., .65, .8, -.4, 0.))
    actual = dict(zip(measured[-1].name, measured[-1].position))
    setup = JointTrajectory(joint_names=list(start.names), points=[
        JointTrajectoryPoint(positions=[actual[n] for n in start.names], velocities=[0.]*5),
        JointTrajectoryPoint(positions=list(start.positions), velocities=[0.]*5)])
    setup.points[-1].time_from_start.sec = 2
    execute(node, actions[arm], setup, measured)
    adapter.set_current_state(deepcopy(measured[-1]))
    adapter.set_deadline(time.monotonic()+600.)
    opening = .4
    adapter.set_gripper_position(arm, opening)
    targets = [adapter.forward_kinematics(arm, JointSolution(start.names,
        (.08+i*.003, .7, .85, -.4, .1))) for i in range(8)]
    requests = [PregraspRequest(arm, target, opening) for target in targets]
    batches = []
    for stage in ('first_for_arm', 'warm_1', 'warm_2'):
        adapter.set_current_state(deepcopy(measured[-1]))
        began = time.monotonic()
        adapter.prepare_pregrasps(requests)
        statistics = deepcopy(adapter.last_ik_statistics)
        accepted = sum(value is not None for value in adapter._pregrasp_cache.values())
        batches.append(dict(stage=stage, seconds=time.monotonic()-began,
                            accepted_targets=accepted, statistics=statistics))
        assert accepted == 8, statistics
        assert statistics['primary_batches'] == 1
        assert statistics['fallback_batches'] == 0
        assert statistics['batch_calls'][0]['padded_targets'] == 8
        assert statistics['batch_calls'][0]['seeds'] == 16
    adapter.set_gripper_position(arm, opening)
    pregrasp = adapter.solve_aligned_ik(arm, targets[0], None)
    assert pregrasp is not None
    before = len(adapter.last_ik_statistics['batch_calls'])
    assert adapter.solve_aligned_ik(arm, targets[0], None) == pregrasp
    assert len(adapter.last_ik_statistics['batch_calls']) == before
    grasp_target = adapter.forward_kinematics(arm, JointSolution(start.names,
        (.10, .71, .86, -.40, .1)))
    began = time.monotonic()
    grasp = adapter.solve_aligned_ik(arm, grasp_target, pregrasp.solution)
    grasp_seconds = time.monotonic()-began
    assert grasp is not None
    assert adapter.state_is_valid(arm, pregrasp.solution)
    assert adapter.state_is_valid(arm, grasp.solution)
    began = time.monotonic()
    paths = [adapter.plan(arm, pregrasp.solution, start),
             adapter.plan(arm, grasp.solution, pregrasp.solution)]
    plan_seconds = time.monotonic()-began
    assert all(path is not None for path in paths)
    for path in paths:
        assert all(adapter.state_is_valid(arm, JointSolution(path.joint_names, q))
                   for q in path.points)
    # The arm driver is GenericSystem. No serial interface exists in this launch.
    # An opening changes only candidate geometry; this test executes arm joints.
    execution = []
    for path in paths:
        trajectory = path_to_message(path)
        began = time.monotonic()
        error, feedback_count = execute(node, actions[arm], trajectory, measured)
        execution.append(dict(seconds=time.monotonic()-began,
            path_points=len(path.points), maximum_joint_error_rad=error,
            feedback_messages=feedback_count))
    import torch
    report = dict(scope='native_hybrid_ik_cpu_plan_mock_arm_execution', arm=arm,
        physical_motors_accessed=False, sudo_used=False, gpu=torch.cuda.get_device_name(0),
        runtime_model='SO-101 arms + live head, 14 measured joints',
        primary_seeds=16, candidate_count=8, batches=batches,
        grasp_ik_seconds=grasp_seconds, cpu_two_paths_seconds=plan_seconds,
        execution=execution, cuda_allocated_bytes=torch.cuda.memory_allocated(),
        cuda_reserved_bytes=torch.cuda.memory_reserved(),
        final_ik_statistics=adapter.last_ik_statistics,
        full_collection_flow=False, perception_models_executed=False,
        depth_octomap_present=False)
    (tmp_path/f'native_hybrid_{arm}.json').write_text(json.dumps(report, indent=2)+'\n')
    record_property('native_report', json.dumps(report))
    print(f'\n{arm}: '+json.dumps({key:report[key] for key in
        ('batches','grasp_ik_seconds','cpu_two_paths_seconds','execution')}))


def test_native_head_updates_locked_geometry_without_recreating_solver(native):
    node, adapter, measured, _ = native
    state = deepcopy(measured[-1])
    adapter.set_current_state(state)
    adapter.set_deadline(time.monotonic()+600.)
    adapter.set_gripper_position('left', .4)
    names = ARM_JOINT_NAMES['left']
    q = JointSolution(names, tuple(dict(zip(state.name, state.position))[n] for n in names))
    target = adapter.forward_kinematics('left', q)
    requests = [PregraspRequest('left', target, .4)]*8
    adapter.prepare_pregrasps(requests)
    engine = adapter._gpus[16]._engine
    cache = engine._batch_ik_caches[('left', 'left_grasp_tcp', 8)]
    solver = cache[0][('left', 'left_grasp_tcp')]
    before = solver.kinematics.get_state(engine.tensor.to_device([[
        dict(zip(q.names,q.positions))[n] for n in solver.joint_names]])).link_spheres_tensor.clone()
    state.position[state.name.index('head_tilt_joint')] += .05
    adapter.set_current_state(state)
    assert dict(zip(adapter._merged_state(None).joint_state.name,
                    adapter._merged_state(None).joint_state.position))['head_tilt_joint'] == pytest.approx(.455)
    adapter.prepare_pregrasps(requests)
    assert cache[0][('left', 'left_grasp_tcp')] is solver
    assert cache[1][('left', 'left_grasp_tcp')]['head_tilt_joint'] == pytest.approx(.455)
    after = solver.kinematics.get_state(engine.tensor.to_device([[
        dict(zip(q.names,q.positions))[n] for n in solver.joint_names]])).link_spheres_tensor
    assert not engine.torch.allclose(before, after)
    assert all(adapter._pregrasp_cache.values())


def test_native_unreachable_targets_retry32_and_return_no_solution(native, tmp_path,
                                                                record_property):
    node, adapter, measured, _ = native
    adapter.set_current_state(deepcopy(measured[-1]))
    adapter.set_deadline(time.monotonic()+600.)
    targets = [PregraspRequest('left', PoseValue((80.+i,80.,80.), (0.,0.,0.,1.)), .4)
               for i in range(8)]
    began = time.monotonic()
    adapter.prepare_pregrasps(targets)
    assert all(value is None for value in adapter._pregrasp_cache.values())
    statistics = deepcopy(adapter.last_ik_statistics)
    assert statistics['primary_batches'] == 2 and statistics['fallback_batches'] == 1
    assert statistics['pregrasp_retry_arms'] == ['left']
    assert [call['seeds'] for call in statistics['batch_calls']] == [16,16,32]
    assert statistics['primary_retry_targets'] == 8
    assert statistics['cpu_ik_fallback_calls'] == []
    report = dict(scope='native_unreachable_candidate_rejection',
        seconds=time.monotonic()-began, statistics=statistics,
        physical_motors_accessed=False, controller_commands=0)
    (tmp_path/'native_hybrid_unreachable.json').write_text(json.dumps(report, indent=2)+'\n')
    record_property('native_report', json.dumps(report))
