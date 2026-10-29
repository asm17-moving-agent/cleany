import json
from types import SimpleNamespace

import pytest

from action_msgs.msg import GoalStatus
from moveit_msgs.msg import MoveItErrorCodes, RobotState
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

from cleany_skill_executor.core.physical_grasp_selection import (
    ARM_JOINT_NAMES, JointSolution, PoseAlignment, PoseValue,
    REQUIRED_JOINT_NAMES,
)
from cleany_skill_executor.physical_moveit_adapter import MoveItAdapterConfig, MoveItGraspAdapter


@pytest.mark.parametrize('arm', ['left', 'right'])
@pytest.mark.parametrize('clear', [True, False])
def test_direct_grasp_selection_prefers_validated_joint_connection_and_falls_back(arm, clear):
    from cleany_skill_executor.core.physical_grasp_selection import PlannedPath
    names = ARM_JOINT_NAMES[arm]
    state = JointState(name=list(names), position=[.1]*5)
    goal = JointSolution(names, (.2, .3, .4, .5, .6))
    calls = []
    path = PlannedPath(names, ((.1,)*5, goal.positions))
    def validate(selected_arm, proposed):
        assert selected_arm == arm and proposed == path
        calls.append('validate_direct')
        return path if clear else None
    def plan(selected_arm, target, start):
        assert selected_arm == arm and target == goal and start is None
        calls.append('rrt_from_current')
        return path
    adapter = SimpleNamespace(_check_active=lambda: None, _config=MoveItAdapterConfig(),
        _merged_state=lambda start: RobotState(joint_state=state),
        _validated_cubic_path=validate, plan=plan)
    adapter._direct_joint_path = lambda arm, goal: MoveItGraspAdapter._direct_joint_path(adapter, arm, goal)
    assert MoveItGraspAdapter.plan_direct_grasp(adapter, arm, goal) == path
    assert calls == (['validate_direct'] if clear else ['validate_direct', 'rrt_from_current'])


@pytest.mark.parametrize('margin,accepted', [(1., False), (20., True)])
@pytest.mark.parametrize('planner_id', [None, 'RRTConnectReturnkConfigDefault'])
@pytest.mark.parametrize('trajectory_scaling', [None, 1.])
def test_successful_plan_delayed_by_postprocessing_respects_response_margin(
        margin, accepted, planner_id, trajectory_scaling):
    from rclpy.task import Future
    from moveit_msgs.msg import RobotTrajectory
    from trajectory_msgs.msg import JointTrajectory
    from cleany_skill_executor.core.physical_grasp_selection import InfrastructureError
    clock = [0.]
    names = ARM_JOINT_NAMES['right']
    planned = JointTrajectory(joint_names=list(names), points=[
        JointTrajectoryPoint(positions=[0.]*5), JointTrajectoryPoint(positions=[.1]*5)])
    result = Future()
    handle = SimpleNamespace(accepted=True, get_result_async=lambda: result)
    sent = []
    def send(goal):
        assert goal.planning_options.plan_only
        assert goal.request.allowed_planning_time == 4.
        assert goal.request.planner_id == (planner_id or 'RRTConnectkConfigDefault')
        assert goal.request.max_velocity_scaling_factor == (trajectory_scaling or .1)
        assert goal.request.max_acceleration_scaling_factor == (trajectory_scaling or .1)
        sent.append(goal)
        future = Future()
        future.set_result(handle)
        return future
    def spin(seconds):
        clock[0] += seconds
        if clock[0] >= 6.7 and not result.done():
            result.set_result(SimpleNamespace(status=GoalStatus.STATUS_SUCCEEDED,
                result=SimpleNamespace(error_code=MoveItErrorCodes(val=MoveItErrorCodes.SUCCESS),
                    planned_trajectory=RobotTrajectory(joint_trajectory=planned))))
    adapter = MoveItGraspAdapter(object(), MoveItAdapterConfig(
        planning_timeout_sec=4., planning_response_margin_sec=margin),
        ik_client=object(), validity_client=object(),
        plan_client=SimpleNamespace(wait_for_server=lambda **kwargs: True, send_goal_async=send),
        spin_once=spin, monotonic=lambda: clock[0])
    adapter._merged_state = lambda start: RobotState()
    goal = JointSolution(names, (.1,)*5)
    if accepted:
        path = adapter.plan('right', goal, None, planner_id=planner_id,
                            trajectory_scaling=trajectory_scaling)
        assert path is not None and path.points[-1] == goal.positions
        assert clock[0] == pytest.approx(6.7, abs=.011)
    else:
        with pytest.raises(InfrastructureError, match='MoveGroup action timed out'):
            adapter.plan('right', goal, None, planner_id=planner_id,
                         trajectory_scaling=trajectory_scaling)
        assert clock[0] == pytest.approx(5., abs=.011)
    assert len(sent) == 1
    assert adapter._config.velocity_scaling == .1 and adapter._config.acceleration_scaling == .1


@pytest.mark.parametrize('scaling', [0., -1., 1.01, float('nan'), float('inf')])
def test_invalid_plan_only_trajectory_scaling_fails_before_sending(scaling):
    adapter = MoveItGraspAdapter(object(), ik_client=object(), validity_client=object(),
                                plan_client=object())
    with pytest.raises(ValueError, match='trajectory scaling'):
        adapter.plan('left', JointSolution(ARM_JOINT_NAMES['left'], (.1,)*5), None,
                     trajectory_scaling=scaling)


def test_pregrasp_tolerance_restores_after_solver_failure():
    adapter = MoveItGraspAdapter(object(),
        MoveItAdapterConfig(position_tolerance_m=.002, pregrasp_position_tolerance_m=.010),
        ik_client=object(), validity_client=object(), plan_client=object())
    original = adapter._config

    def interrupted(*_):
        assert adapter._config.position_tolerance_m == .010
        raise InterruptedError('canceled')

    adapter.solve_aligned_ik = interrupted
    with pytest.raises(InterruptedError, match='canceled'):
        adapter.solve_pregrasp_ik('left', PoseValue((.4, .2, .3), (0., 0., 0., 1.)))
    assert adapter._config is original


@pytest.mark.parametrize('mismatch', [False, True])
def test_local_fk_checks_remote_first_and_invalidates_on_state_opening_changes(mismatch):
    import numpy as np
    from cleany_skill_executor.core.physical_grasp_selection import InfrastructureError
    adapter = MoveItGraspAdapter(object(), MoveItAdapterConfig(use_verified_local_fk=True),
        ik_client=object(), validity_client=object(), plan_client=object())
    adapter.set_current_state(current_state())
    names = ARM_JOINT_NAMES['right']
    adapter._local_fk = {'right': SimpleNamespace(pose=lambda state: (
        np.array([state[names[0]], 0., 0.]), np.eye(3)))}
    calls = []
    def remote(arm, q):
        calls.append(q)
        return PoseValue((q.positions[0] + (.1 if mismatch else 0.), 0., 0.), (0., 0., 0., 1.))
    adapter._remote_forward_kinematics = remote
    solution = JointSolution(names, (0.,)*5)
    if mismatch:
        with pytest.raises(InfrastructureError, match='disagrees'):
            adapter.forward_kinematics('right', solution)
        assert not adapter._verified_fk_arms
        return
    adapter.forward_kinematics('right', solution)
    moved = JointSolution(names, (.3, 0., 0., 0., 0.))
    assert adapter.forward_kinematics('right', moved).position == (.3, 0., 0.)
    assert len(calls) == 1
    adapter.set_gripper_position('right', 1.)
    adapter.forward_kinematics('right', moved)
    assert len(calls) == 2
    adapter.set_current_state(current_state())
    adapter.forward_kinematics('right', solution)
    assert len(calls) == 3


@pytest.mark.parametrize('value', [float('nan'), float('inf')])
def test_local_fk_rejects_nonfinite_remote_reference(value):
    from cleany_skill_executor.core.physical_grasp_selection import InfrastructureError
    adapter = MoveItGraspAdapter(object(), MoveItAdapterConfig(use_verified_local_fk=True),
        ik_client=object(), validity_client=object(), plan_client=object())
    adapter._local_fk = {'right': object()}
    adapter._remote_forward_kinematics = lambda *_: PoseValue((value, 0., 0.), (0., 0., 0., 1.))
    with pytest.raises(InfrastructureError, match='not finite'):
        adapter.forward_kinematics('right', JointSolution(ARM_JOINT_NAMES['right'], (0.,)*5))
    assert not adapter._verified_fk_arms


@pytest.mark.parametrize('arm', ['left', 'right'])
def test_current_tcp_direction_uses_local_measured_state_without_remote_calls(arm):
    import math
    from cleany_skill_executor.core.urdf_fk import UrdfChain
    from cleany_skill_executor.core.alignment import tcp_axes
    adapter = MoveItGraspAdapter(object(), MoveItAdapterConfig(),
        ik_client=object(), validity_client=object(), plan_client=object())
    state = current_state()
    state.position[state.name.index(f'{arm}_wrist_roll_joint')] = math.pi / 2
    adapter.set_current_state(state)
    adapter._local_fk[arm] = UrdfChain(f'''<robot name="test">
        <joint name="{arm}_wrist_roll_joint" type="revolute">
          <parent link="base_link"/><child link="{arm}_grasp_tcp"/>
          <origin xyz=".1 .2 .3"/><axis xyz="0 0 1"/>
        </joint></robot>''', 'base_link', f'{arm}_grasp_tcp')
    adapter._remote_forward_kinematics = lambda *args: pytest.fail('Direction ordering must not call MoveIt FK')
    adapter._call = lambda *args: pytest.fail('Direction ordering must not make service calls')
    adapter.set_gripper_position(arm, 1.)
    pose = adapter.current_tcp_pose(arm)
    assert pose.position == pytest.approx((.1, .2, .3))
    _, closing = tcp_axes(pose.orientation_xyzw)
    assert closing == pytest.approx((0., 1., 0.))
    assert not adapter._verified_fk_arms  # An ordering hint is not validation evidence.


def test_current_tcp_direction_requires_measured_state_and_runtime_model():
    from cleany_skill_executor.core.physical_grasp_selection import InfrastructureError
    adapter = MoveItGraspAdapter(object(), MoveItAdapterConfig(),
        ik_client=object(), validity_client=object(), plan_client=object())
    with pytest.raises(InfrastructureError, match='Current joint state'):
        adapter.current_tcp_pose('right')
    adapter.set_current_state(current_state())
    with pytest.raises(InfrastructureError, match='robot_description'):
        adapter.current_tcp_pose('right')


def test_wrist_seed_variants_use_runtime_limit_not_old_hardcoded_limit():
    adapter = SimpleNamespace(
        _config=MoveItAdapterConfig(ik_seed_wrist_offsets_degrees=(-90., 90.)),
        _joint_limits={'left_wrist_roll_joint': (-1.0, 3.0, 1.0)},
    )
    solution = JointSolution(ARM_JOINT_NAMES['left'], (0., 0., 0., 0., 2.5))
    variants = MoveItGraspAdapter._wrist_variants(adapter, solution)
    assert tuple(item.positions[-1] for item in variants) == (2.5 - 1.5707963267948966, 3.0)


def test_wrist_variants_try_exact_preferred_roll_before_offset_seeds():
    adapter = SimpleNamespace(
        _config=MoveItAdapterConfig(ik_seed_wrist_offsets_degrees=(0., 45.)),
        _joint_limits={'left_wrist_roll_joint': (-1.0, 3.0, 1.0)},
    )
    solution = JointSolution(ARM_JOINT_NAMES['left'], (0., 0., 0., 0., 2.5))

    variants = MoveItGraspAdapter._wrist_variants(adapter, solution, 0.4)

    assert tuple(item.positions[-1] for item in variants) == (0.4, 2.5, 3.0)


def test_best_feasible_alignment_prefers_roll_nearest_measured_state():
    adapter = MoveItGraspAdapter.__new__(MoveItGraspAdapter)
    adapter._current_state = current_state()
    wrist = ARM_JOINT_NAMES['left'][-1]
    measured = dict(zip(adapter._current_state.name, adapter._current_state.position))[wrist]

    def aligned(roll, approach_error):
        return PoseAlignment(
            JointSolution(ARM_JOINT_NAMES['left'], (0., 0., 0., 0., roll)),
            PoseValue((0., 0., 0.), (0., 0., 0., 1.)),
            0.001, approach_error, 1.0,
        )

    near = aligned(measured + 0.1, 2.0)
    far = aligned(measured + 1.0, 1.0)
    assert MoveItGraspAdapter._best_alignment(adapter, [far, near]) is near


def test_grasp_alignment_can_prefer_pregrasp_roll_over_measured_roll():
    adapter = MoveItGraspAdapter.__new__(MoveItGraspAdapter)
    adapter._current_state = current_state()

    def aligned(roll):
        return PoseAlignment(
            JointSolution(ARM_JOINT_NAMES['left'], (0., 0., 0., 0., roll)),
            PoseValue((0., 0., 0.), (0., 0., 0., 1.)),
            0.001, 1.0, 1.0,
        )

    measured_near = aligned(4.1)
    pregrasp_near = aligned(1.1)
    assert MoveItGraspAdapter._best_alignment(
        adapter, [measured_near, pregrasp_near], preferred_roll=1.0
    ) is pregrasp_near


def test_wrist_motion_cost_includes_pregrasp_and_approach_travel():
    adapter = MoveItGraspAdapter(
        object(), ik_client=object(), validity_client=object(), plan_client=object()
    )
    state = current_state()
    wrist = ARM_JOINT_NAMES['left'][-1]
    state.position[state.name.index(wrist)] = 0.5
    adapter.set_current_state(state)
    pregrasp = JointSolution(ARM_JOINT_NAMES['left'], (0., 0., 0., 0., 0.8))
    grasp = JointSolution(ARM_JOINT_NAMES['left'], (0., 0., 0., 0., 0.9))

    assert adapter.wrist_motion_cost('left', pregrasp, grasp) == pytest.approx(0.4)


def test_cubic_validation_rejects_collision_between_clear_endpoints():
    from cleany_skill_executor.core.physical_grasp_selection import PlannedPath
    from cleany_skill_executor.physical_moveit_adapter import MoveItAdapterConfig
    names = ARM_JOINT_NAMES['left']
    checked = []
    def valid(_, solution):
        checked.append(solution.positions[0])
        return not .08 < solution.positions[0] < .12
    adapter = SimpleNamespace(_config=MoveItAdapterConfig(),
        _verify_local_fk=lambda *_: None, _joint_limits={name: (-2., 2., 1.) for name in names},
        state_is_valid=valid)
    adapter._path_samples = lambda path: MoveItGraspAdapter._path_samples(adapter, path)
    path = PlannedPath(names, ((0.,)*5, (.2, 0., 0., 0., 0.)))
    assert MoveItGraspAdapter._validated_cubic_path(adapter, 'left', path) is None
    assert any(.08 < q < .12 for q in checked)


def test_optional_cubic_validation_deadline_never_accepts_a_partially_checked_path():
    from cleany_skill_executor.core.physical_grasp_selection import PlannedPath
    names = ARM_JOINT_NAMES['left']
    clock, checked = [0.], []
    def valid(_, solution):
        checked.append(solution.positions)
        clock[0] += .2
        return True
    adapter = SimpleNamespace(_config=MoveItAdapterConfig(state_validity_batch_size=1),
        _monotonic=lambda: clock[0], _verify_local_fk=lambda *_: None,
        _joint_limits={name: (-2., 2., 1.) for name in names}, state_is_valid=valid)
    path = PlannedPath(names, ((0.,)*5, (.2, 0., 0., 0., 0.)))
    assert MoveItGraspAdapter._validated_cubic_path(adapter, 'left', path, deadline_monotonic=.35) is None
    assert len(checked) == 2


def test_expired_optional_cubic_validation_never_starts_fk_or_collision_calls():
    adapter = SimpleNamespace(_monotonic=lambda: 2.,
        _verify_local_fk=lambda *_: pytest.fail('Expired refinement issued FK'))
    assert MoveItGraspAdapter._validated_cubic_path(adapter, 'left', None, deadline_monotonic=1.) is None


def test_optional_cubic_deadline_is_checked_between_full_state_batches():
    from cleany_skill_executor.core.physical_grasp_selection import PlannedPath
    names, clock, checked = ARM_JOINT_NAMES['left'], [0.], []
    def batch(states, call):
        checked.append(states)
        clock[0] += .2
        return SimpleNamespace(first_invalid_index=-1)
    adapter = SimpleNamespace(_config=MoveItAdapterConfig(), _monotonic=lambda: clock[0],
        _verify_local_fk=lambda *_: None, _check_active=lambda: None,
        _merged_state=lambda _: RobotState(),
        _batch_validity=SimpleNamespace(batch_size=1, check=batch),
        _joint_limits={name: (-2., 2., 1.) for name in names})
    path = PlannedPath(names, ((0.,)*5, (.2, 0., 0., 0., 0.)))
    assert MoveItGraspAdapter._validated_cubic_path(adapter, 'left', path, deadline_monotonic=.35) is None
    assert len(checked) == 2


class Future:
    def __init__(self, value):
        self.value = value

    def done(self):
        return True

    def result(self):
        return self.value

    def cancel(self):
        pass


class ServiceClient:
    def __init__(self, response):
        self.response = response
        self.requests = []

    def service_is_ready(self):
        return True

    def call_async(self, request):
        self.requests.append(request)
        return Future(self.response)


class GoalHandle:
    accepted = True

    def __init__(self, result):
        self.result = result

    def get_result_async(self):
        return Future(self.result)


class PlanClient:
    def __init__(self, wrapped):
        self.wrapped = wrapped
        self.goals = []

    def wait_for_server(self, timeout_sec):
        return True

    def send_goal_async(self, goal):
        self.goals.append(goal)
        return Future(GoalHandle(self.wrapped))


def current_state():
    state = JointState()
    state.name = list(REQUIRED_JOINT_NAMES)
    state.position = [float(index) for index in range(len(state.name))]
    state.velocity = [0.0] * len(state.name)
    return state


@pytest.mark.parametrize('arm', ['left', 'right'])
@pytest.mark.parametrize('obstructed', [False, True])
def test_pregrasp_shortcut_checks_intermediate_full_states_and_falls_back(arm, obstructed):
    from moveit_msgs.msg import RobotTrajectory
    from trajectory_msgs.msg import JointTrajectory
    names = ARM_JOINT_NAMES[arm]
    start = (0.,) * 5
    end = (1., 1., 0., 0., 0.)
    detour = (0., 1., 0., 0., 0.)
    trajectory = JointTrajectory(joint_names=list(names), points=[
        JointTrajectoryPoint(positions=list(q)) for q in (start, detour, end)])
    wrapped = SimpleNamespace(status=GoalStatus.STATUS_SUCCEEDED, result=SimpleNamespace(
        error_code=MoveItErrorCodes(val=1), planned_trajectory=RobotTrajectory(joint_trajectory=trajectory)))
    plan = PlanClient(wrapped)
    adapter = MoveItGraspAdapter(object(), MoveItAdapterConfig(
        validate_motion_paths=True, pregrasp_joint_shortcut_enabled=True),
        ik_client=object(), validity_client=object(), plan_client=plan)
    state = JointState(name=list(REQUIRED_JOINT_NAMES), position=[0.] * len(REQUIRED_JOINT_NAMES))
    opposite = 'left' if arm == 'right' else 'right'
    state.position[state.name.index(f'{opposite}_gripper_joint')] = .7
    adapter.set_current_state(state)
    adapter.set_gripper_position(arm, .3)
    adapter._joint_limits = {name: (-2., 2., 1.) for name in REQUIRED_JOINT_NAMES}
    adapter._verify_local_fk = lambda *_: None  # FK parity is covered by the runtime integration.
    checked = []
    def check(client, request, timeout):
        q = dict(zip(request.robot_state.joint_state.name, request.robot_state.joint_state.position))
        assert q[f'{opposite}_gripper_joint'] == .7
        assert q[f'{arm}_gripper_joint'] == .3
        checked.append(q)
        collision = obstructed and all(.4 < q[name] < .6 for name in names[:2])
        return SimpleNamespace(valid=not collision, contacts=[], constraint_result=[])
    adapter._call = check
    path = adapter.plan(arm, JointSolution(names, end), None)
    assert path is not None and path.points[-1] == end
    assert len(checked) > 2  # Interior states, not just the endpoints, were checked.
    assert len(plan.goals) == int(obstructed)
    assert path.points == ((start, detour, end) if obstructed else (start, end))


def test_pregrasp_shortcut_cannot_be_enabled_without_path_validation():
    with pytest.raises(ValueError, match='full path validation'):
        MoveItAdapterConfig(pregrasp_joint_shortcut_enabled=True)


def test_candidate_arm_switch_preserves_measured_opposite_gripper():
    adapter = MoveItGraspAdapter(object(), ik_client=object(), validity_client=object(), plan_client=object())
    original = current_state()
    adapter.set_current_state(original)
    adapter.set_gripper_position('left', 1.)
    adapter.set_gripper_position('right', .7)
    state = adapter._merged_state(None).joint_state
    values = dict(zip(state.name, state.position))
    measured = dict(zip(original.name, original.position))
    assert values['left_gripper_joint'] == measured['left_gripper_joint']
    assert values['right_gripper_joint'] == .7


def test_closure_sweep_queries_full_robot_and_keeps_opposite_arm():
    client = ServiceClient(SimpleNamespace(valid=True))
    adapter = MoveItGraspAdapter(object(), ik_client=object(), validity_client=client, plan_client=object())
    adapter.set_current_state(current_state())
    adapter._joint_limits = {'left_gripper_joint': (-.374532977628, 1.74532919957, 1.)}
    solution = JointSolution(ARM_JOINT_NAMES['left'], (0.,)*5)
    assert adapter.gripper_sweep_is_valid('left', solution, .5, .3, .05)
    values = []
    for request in client.requests:
        assert request.group_name == ''
        state = dict(zip(request.robot_state.joint_state.name, request.robot_state.joint_state.position))
        assert state['right_shoulder_yaw_joint'] == 6.
        values.append(state['left_gripper_joint'])
    assert values[0] == .5 and values[-1] == .3
    assert max(abs(a-b) for a, b in zip(values, values[1:])) <= .05+1e-10


@pytest.mark.parametrize('collision_search', [False, True])
def test_ik_seed_is_a_complete_robot_state_and_preserves_other_arm(collision_search):
    solution = RobotState()
    solution.joint_state = current_state()
    response = SimpleNamespace(
        error_code=SimpleNamespace(val=MoveItErrorCodes.SUCCESS),
        solution=solution,
    )
    ik = ServiceClient(response)
    adapter = MoveItGraspAdapter(object(), MoveItAdapterConfig(ik_collision_search=collision_search),
        ik_client=ik, validity_client=object(), plan_client=object())
    adapter.set_current_state(current_state())
    seed = JointSolution(ARM_JOINT_NAMES['left'], (10.0, 11.0, 12.0, 13.0, 14.0))
    adapter.solve_position_ik('left', (0.4, 0.2, 0.8), seed)
    request = ik.requests[0].ik_request
    assert request.group_name == 'left_grasp_arm'
    assert request.ik_link_name == 'left_grasp_tcp'
    assert request.avoid_collisions is collision_search
    assert tuple(request.robot_state.joint_state.name) == REQUIRED_JOINT_NAMES
    values = dict(zip(request.robot_state.joint_state.name, request.robot_state.joint_state.position))
    assert tuple(values[name] for name in ARM_JOINT_NAMES['left']) == seed.positions
    assert values['right_shoulder_yaw_joint'] == 6.0


@pytest.mark.parametrize('allow_second', [False, True])
def test_deferred_ik_collision_checks_eligible_variants_before_selecting(allow_second):
    adapter = MoveItGraspAdapter(object(), MoveItAdapterConfig(ik_collision_search=False),
        ik_client=object(), validity_client=object(), plan_client=object())
    names = ARM_JOINT_NAMES['right']
    target = PoseValue((0., 0., 0.), (0., 0., 0., 1.))
    blocked = PoseAlignment(JointSolution(names, (0.,)*5), target, 0., 0., 0.)
    allowed = PoseAlignment(JointSolution(names, (0., 0., 0., 0., .5)), target, 0., 0., 0.)
    adapter._seed_bases = lambda arm, seed: [blocked.solution]
    adapter._solve_position_ik = lambda *args, **kwargs: blocked.solution if kwargs['link_name'].endswith('grasp_tcp') else None
    def record(arm, target, solution, evaluated, seen, preferred):
        if solution is not None:
            evaluated.extend([blocked, allowed])
    adapter._record_wrist_alignments = record
    checked = []
    def valid(arm, solution):
        checked.append(solution)
        return allow_second and solution == allowed.solution
    adapter.state_is_valid = valid
    result = adapter.solve_aligned_ik('right', target, None)
    assert result == (allowed if allow_second else None)
    assert checked == [blocked.solution, allowed.solution]


def test_second_segment_has_explicit_pregrasp_start_state():
    trajectory = SimpleNamespace(joint_trajectory=SimpleNamespace(points=[JointTrajectoryPoint()]))
    wrapped = SimpleNamespace(
        status=GoalStatus.STATUS_SUCCEEDED,
        result=SimpleNamespace(
            error_code=SimpleNamespace(val=MoveItErrorCodes.SUCCESS),
            planned_trajectory=trajectory,
        ),
    )
    plan = PlanClient(wrapped)
    adapter = MoveItGraspAdapter(object(), ik_client=object(), validity_client=object(), plan_client=plan)
    adapter.set_current_state(current_state())
    adapter.set_gripper_position('right', 1.25)
    pregrasp = JointSolution(ARM_JOINT_NAMES['right'], (20.0, 21.0, 22.0, 23.0, 24.0))
    grasp = JointSolution(ARM_JOINT_NAMES['right'], (25.0, 26.0, 27.0, 28.0, 29.0))
    assert adapter.plan('right', grasp, pregrasp)
    request = plan.goals[0].request
    assert plan.goals[0].planning_options.plan_only is True
    assert request.group_name == 'right_grasp_arm'
    assert request.planner_id == 'RRTConnectkConfigDefault'
    start = dict(zip(request.start_state.joint_state.name, request.start_state.joint_state.position))
    assert tuple(start[name] for name in ARM_JOINT_NAMES['right']) == pregrasp.positions
    assert start['right_gripper_joint'] == 1.25
    assert [constraint.joint_name for constraint in request.goal_constraints[0].joint_constraints] == list(ARM_JOINT_NAMES['right'])


def test_out_of_bounds_measured_joint_is_rejected_without_clamping():
    import pytest
    adapter = MoveItGraspAdapter(object(), ik_client=object(), validity_client=object(), plan_client=object())
    state = current_state()
    joint = 'right_shoulder_pitch_joint'
    adapter._joint_limits = {joint: (-.22, 3.37, 1.)}
    state.position[state.name.index(joint)] = 3.397767
    with pytest.raises(ValueError, match='right_shoulder_pitch_joint'):
        adapter.set_current_state(state)
    assert adapter._current_state is None
    assert state.position[state.name.index(joint)] == 3.397767


def test_candidate_joint_limit_rejection_records_bounds_without_calling_moveit():
    logs = []
    node = SimpleNamespace(get_logger=lambda: SimpleNamespace(info=logs.append))
    adapter = MoveItGraspAdapter(node, ik_client=object(), validity_client=object(), plan_client=object())
    names = ARM_JOINT_NAMES['right']
    adapter._joint_limits = {name: (-1., 1., 1.) for name in names}
    adapter._call = lambda *_: pytest.fail('Out-of-range IK must not call MoveIt')
    solution = JointSolution(names, (0., 0., 1.2, 0., 0.))
    assert adapter.state_is_valid('right', solution) is False
    record = json.loads(logs[0].split('=', 1)[1])
    assert record['reason'] == 'joint_limits'
    assert record['joint_limit_violations'] == [dict(
        joint=names[2], position_rad=1.2, lower_rad=-1., upper_rad=1.)]
    assert solution.positions[2] == 1.2


@pytest.mark.parametrize('contact_count', [0, 10])
def test_moveit_rejection_records_actual_full_state_and_bounded_contacts(contact_count):
    from moveit_msgs.msg import ContactInformation
    logs, calls = [], []
    node = SimpleNamespace(get_logger=lambda: SimpleNamespace(info=logs.append))
    adapter = MoveItGraspAdapter(node, MoveItAdapterConfig(validate_motion_paths=True),
        ik_client=object(), validity_client=object(), plan_client=object())
    state = current_state()
    adapter.set_current_state(state)
    adapter.set_gripper_position('right', .75)
    contacts = [ContactInformation(contact_body_1='<octomap>', contact_body_2='right_upper_arm_link',
                                   depth=.002) for _ in range(contact_count)]
    def call(client, request, timeout):
        calls.append(request)
        return SimpleNamespace(valid=False, contacts=contacts, constraint_result=[])
    adapter._call = call
    solution = JointSolution(ARM_JOINT_NAMES['right'], (.1,) * 5)
    assert adapter.state_is_valid('right', solution) is False
    assert len(calls) == 1 and calls[0].group_name == ''
    record = json.loads(logs[0].split('=', 1)[1])
    assert record['reason'] == ('collision' if contact_count else 'moveit_state_invalid')
    assert record['contact_count'] == contact_count
    assert len(record['contacts']) == min(contact_count, 8)
    if contact_count:
        assert record['contacts'][0] == dict(body_1='<octomap>', body_2='right_upper_arm_link', depth_m=.002)
    assert record['robot_joint_positions']['right_gripper_joint'] == .75
    assert record['robot_joint_positions']['right_elbow_pitch_joint'] == .1
    assert record['robot_joint_positions']['left_elbow_pitch_joint'] == state.position[
        state.name.index('left_elbow_pitch_joint')]
    adapter._call = lambda *_: SimpleNamespace(valid=True)
    assert adapter.state_is_valid('right', solution) is True
    assert adapter.last_state_validity_rejection is None
    assert len(logs) == 1
