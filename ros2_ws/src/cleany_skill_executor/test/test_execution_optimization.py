"""Fast paths must retain collision, freshness, cancellation and endpoint gates."""
from types import SimpleNamespace as NS

import numpy as np
import pytest
from moveit_msgs.msg import MoveItErrorCodes, RobotTrajectory
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from cleany_skill_executor.core.physical_grasp_selection import (
    ARM_JOINT_NAMES, REQUIRED_JOINT_NAMES, InfrastructureError, JointSolution, PlannedPath, PoseValue)
from cleany_skill_executor.hardware_execution import HardwareExecution
from cleany_skill_executor.physical_moveit_adapter import MoveItAdapterConfig, MoveItGraspAdapter


def approach_fixture(*, fraction=1., collision=False, detour=False):
    names = ARM_JOINT_NAMES['left']
    first, last = (0.,)*5, (.2, 0., 0., 0., 0.)
    middle = (.1, .2 if detour else 0., 0., 0., 1.3)
    trajectory = JointTrajectory(joint_names=list(names), points=[
        JointTrajectoryPoint(positions=list(q)) for q in (first, middle, last)])
    response = NS(fraction=fraction, error_code=MoveItErrorCodes(val=1),
                  solution=RobotTrajectory(joint_trajectory=trajectory))
    adapter = MoveItGraspAdapter(object(), MoveItAdapterConfig(
        validate_motion_paths=True, cartesian_approach_first=True, preserve_scene_attachments=True),
        ik_client=object(), fk_client=object(), validity_client=object(),
        plan_client=object(), cartesian_client=object())
    adapter.set_current_state(JointState(name=list(REQUIRED_JOINT_NAMES),
                                       position=[0.]*len(REQUIRED_JOINT_NAMES)))
    adapter._joint_limits = {n: (-2., 2., 1.) for n in REQUIRED_JOINT_NAMES}
    adapter.set_gripper_position('left', .4)
    calls = []
    def cartesian(client, request, timeout):
        assert request.avoid_collisions and request.start_state.is_diff
        calls.append(('cartesian', request))
        return response
    def batch(states, call):
        calls.append(('collision_batch', len(states)))
        for state in states:
            q = dict(zip(state.joint_state.name, state.joint_state.position))
            assert q['left_gripper_joint'] == .4 and q['right_gripper_joint'] == 0.
            assert q['left_wrist_roll_joint'] == 0.
        return NS(first_invalid_index=0 if collision else -1)
    adapter._call = cartesian
    adapter._batch_validity = NS(batch_size=32, check=batch)
    adapter._verify_local_fk = lambda *_: None
    adapter.forward_kinematics = lambda arm, q: PoseValue(q.positions[:3], (0., 0., 0., 1.))
    def fallback(*args, corridor=None):
        calls.append(('rrt', args))
        assert corridor is not None
        return adapter._validated_cubic_path('left', PlannedPath(names, (first, last)),
                                             corridor=corridor)
    adapter.plan = fallback
    args = ('left', JointSolution(names, last), JointSolution(names, first),
            PoseValue(first[:3], (0., 0., 0., 1.)), PoseValue(last[:3], (0., 0., 0., 1.)))
    return adapter, response, args, calls


def test_cartesian_approach_restores_roll_endpoints_and_checks_one_sample_set():
    adapter, _, args, calls = approach_fixture()
    result = adapter.plan_approach(*args)
    assert result.points[0] == args[2].positions and result.points[-1] == args[1].positions
    assert result.times and len(result.velocities) == len(result.points)
    assert [kind for kind, _ in calls] == ['cartesian', 'collision_batch']
    assert adapter.last_plan_route == 'cartesian_approach'


@pytest.mark.parametrize('options', [dict(fraction=.9), dict(detour=True), dict(collision=True)])
def test_unusable_cartesian_proposal_falls_back_to_one_final_rrt_validation(options):
    adapter, _, args, calls = approach_fixture(**options)
    result = adapter.plan_approach(*args)
    assert sum(kind=='rrt' for kind, _ in calls) == 1
    assert (result is None) == options.get('collision', False)


def test_cartesian_cancellation_and_corrupt_response_do_not_fall_back():
    adapter, response, args, calls = approach_fixture()
    response.fraction = float('nan')
    with pytest.raises(InfrastructureError):
        adapter.plan_approach(*args)
    adapter._canceled = True
    with pytest.raises(InterruptedError):
        adapter.plan_approach(*args)
    assert all(kind != 'rrt' for kind, _ in calls)


@pytest.mark.parametrize('available', [False, True])
@pytest.mark.parametrize('blocked', [False, True])
def test_gripper_sweep_batch_keeps_every_sample_and_full_robot_state(available, blocked):
    from cleany_skill_executor.state_validity_batch import StateValidityBatch
    adapter, _, args, _ = approach_fixture()
    names = [*REQUIRED_JOINT_NAMES, 'head_pan_joint', 'head_tilt_joint']
    adapter._state_joint_names = tuple(names)
    adapter.set_current_state(JointState(name=names, position=[0.]*len(names)))
    batch_client, seen, calls = NS(service_is_ready=lambda: available), [], []
    adapter._batch_validity = StateValidityBatch(object(), batch_size=32, client=batch_client)
    def check(client, request, timeout):
        states = request.robot_states if client is batch_client else [request.robot_state]
        calls.append(len(states))
        valid = []
        for state in states:
            q = dict(zip(state.joint_state.name, state.joint_state.position))
            assert q['right_gripper_joint'] == 0. and q['head_pan_joint'] == 0.
            seen.append(q['left_gripper_joint'])
            valid.append(not (blocked and -.05 < seen[-1] < .1))
            if not valid[-1]:
                break
        return (NS(success=True, message='', valid=valid, contacts=[],
                   first_invalid_index=-1 if all(valid) else len(valid)-1)
                if client is batch_client else NS(valid=valid[0]))
    adapter._call = check
    assert adapter.gripper_sweep_is_valid('left', args[2], .784, -.374, .05) is not blocked
    assert seen[0] == .784
    if not blocked:
        assert seen == pytest.approx(np.linspace(.784, -.374, 25))
        assert len(calls) == (1 if available else 25)


@pytest.mark.parametrize('collision', [False, True])
def test_real_moveit_fallback_validates_the_final_cubic_once(collision):
    from types import MethodType
    from action_msgs.msg import GoalStatus
    from rclpy.task import Future

    adapter, _, args, calls = approach_fixture(fraction=.9, collision=collision)
    trajectory = JointTrajectory(joint_names=list(args[1].names), points=[
        JointTrajectoryPoint(positions=list(q)) for q in (args[2].positions, args[1].positions)])
    done = Future()
    done.set_result(NS(status=GoalStatus.STATUS_SUCCEEDED, result=NS(
        error_code=MoveItErrorCodes(val=MoveItErrorCodes.SUCCESS),
        planned_trajectory=RobotTrajectory(joint_trajectory=trajectory))))
    def send(goal):
        assert goal.planning_options.plan_only
        accepted = Future()
        accepted.set_result(NS(accepted=True, get_result_async=lambda: done))
        return accepted
    adapter._plan_client = NS(wait_for_server=lambda **_: True, send_goal_async=send)
    adapter.plan = MethodType(MoveItGraspAdapter.plan, adapter)
    result = adapter.plan_approach(*args)
    assert (result is None) is collision
    assert [kind for kind, _ in calls] == ['cartesian', 'collision_batch']


def test_local_fk_memoization_is_invalidated_and_never_hides_cancellation():
    names = ARM_JOINT_NAMES['left']
    adapter = MoveItGraspAdapter(object(), MoveItAdapterConfig(use_verified_local_fk=True),
        ik_client=object(), fk_client=object(), validity_client=object(), plan_client=object())
    state = JointState(name=list(REQUIRED_JOINT_NAMES), position=[0.]*len(REQUIRED_JOINT_NAMES))
    adapter.set_current_state(state)
    calls = []
    def local(q):
        calls.append(q)
        return np.array([q[names[0]], 0., 0.]), np.eye(3)
    adapter._local_fk = {'left': NS(pose=local)}
    adapter._remote_forward_kinematics = lambda *_: PoseValue((.2, 0., 0.), (0., 0., 0., 1.))
    q = JointSolution(names, (.2, 0., 0., 0., 0.))
    assert adapter.forward_kinematics('left', q) == adapter.forward_kinematics('left', q)
    assert len(calls) == 2  # one parity calculation and one cached value
    adapter.set_gripper_position('left', .4)
    adapter.forward_kinematics('left', q)
    assert len(calls) == 4
    adapter.set_current_state(state)
    adapter.forward_kinematics('left', q)
    assert len(calls) == 6
    adapter._canceled = True
    with pytest.raises(InterruptedError):
        adapter.forward_kinematics('left', q)


@pytest.mark.parametrize('change', ['none', 'pose', 'submission', 'recovery', 'future'])
def test_scene_barrier_reuse_requires_identical_pose_and_execution_generation(change):
    requests, logs = [], []
    state = JointState(name=['joint'], position=[0.])
    node = NS(get_clock=lambda: NS(now=lambda: NS(nanoseconds=20)),
              scene_monitor=NS(wait=lambda **kw: requests.append(kw)),
              get_logger=lambda: NS(info=logs.append))
    hw = NS(node=node, monitor=lambda: state, motion_submissions=1, communication_recovery_count=2)
    barrier = dict(stamp_ns=10, positions={'joint': 0.}, submissions=1, generation=2)
    if change == 'pose': state.position = [.001]
    if change == 'submission': hw.motion_submissions += 1
    if change == 'recovery': hw.communication_recovery_count += 1
    if change == 'future': barrier['stamp_ns'] = 30
    HardwareExecution.wait_for_fresh_sensor_scene(hw, 'test', barrier=barrier)
    assert requests[0]['after_stamp_ns'] == (10 if change=='none' else 20)


@pytest.mark.parametrize('defer', [False, True])
@pytest.mark.parametrize('reject', [False, True])
def test_deferred_pregrasp_keeps_final_dispatch_gate_and_cleanup(defer, reject):
    from cleany_skill_executor.pregrasp_pipeline import PregraspPipeline
    calls = []
    names = ARM_JOINT_NAMES['left']
    state = JointState(name=list(REQUIRED_JOINT_NAMES), position=[0.]*12)
    trajectory = JointTrajectory(joint_names=list(names), points=[
        JointTrajectoryPoint(positions=[q]*5, velocities=[0.]*5) for q in (0., .1)])
    selected = NS(selected_arm='left', planning_start_state=state,
                  pregrasp_trajectory=trajectory, gripper_joint_position=.3,
                  selected_candidate=NS(required_opening_m=.04))
    def dispatch(arm, path, jaw, opening, direction, **kwargs):
        calls.append('final_scene_wait')
        calls.append('final_gate')
        assert kwargs['scene_barrier'] == 'settled'
        assert path.points[0].positions == trajectory.points[0].positions
        if reject:
            raise RuntimeError('final command collision')
        calls.append('submit')
        return state
    motion = NS(set_current_state=lambda *_: None, set_gripper_position=lambda *_: None,
        gripper_sweep_is_valid=lambda *_: calls.append('sweep') or True,
        _validated_cubic_path=lambda *_: calls.append('preliminary_cubic') or
            PlannedPath(names, ((0.,)*5, (.1,)*5), (0., 1.), ((0.,)*5, (0.,)*5)))
    node = NS(verify_simulator=lambda *_: calls.append('outer_verify'), measured=lambda: state,
        real=NS(value=lambda *_: .03, fault=False, capture_scene_barrier=lambda *_: 'settled',
                execute_pregrasp_parallel=dispatch),
        scene=NS(begin=lambda *_: None, restore=lambda: calls.append('restore')),
        scene_monitor=NS(wait=lambda **_: calls.append('preliminary_scene_wait')), motion=motion,
        config={'pregrasp_opening_margin_m': .01}, _defer_pregrasp_validation=defer,
        arm_config=NS(opening=lambda width: width*10, width=.1, opened=1., closed=0.),
        release_grasp_scene_freeze=lambda: None)
    if reject:
        with pytest.raises(RuntimeError, match='final command collision'):
            PregraspPipeline.execute_pregrasp(node, selected)
    else:
        PregraspPipeline.execute_pregrasp(node, selected)
    assert ('outer_verify' in calls) is not defer
    assert ('preliminary_scene_wait' in calls) is not defer
    assert calls.count('final_scene_wait') == 1
    assert ('sweep' in calls) is not defer
    assert ('preliminary_cubic' in calls) is not defer
    assert ('submit' in calls) is not reject
    assert calls[-1] == 'restore'
