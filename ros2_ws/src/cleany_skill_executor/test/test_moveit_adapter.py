import math
import numpy as np
import pytest
from types import SimpleNamespace

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import MoveItErrorCodes, RobotState
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

from cleany_skill_executor.core.grasp_selection import (
    ARM_JOINT_NAMES,
    REQUIRED_JOINT_NAMES,
    JointSolution,
    InfrastructureError,
)
from cleany_skill_executor.moveit_adapter import (
    MoveItAdapterConfig,
    MoveItGraspAdapter,
)


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


def test_aim_ik_response_margin_does_not_extend_solver_budget():
    adapter = MoveItGraspAdapter(object(), config=MoveItAdapterConfig(ik_response_margin_sec=5.),
        ik_client=object(), fk_client=object(), validity_client=object(), plan_client=object())
    adapter.set_current_state(current_state())
    calls = []
    adapter._call = lambda client, request, timeout: calls.append((request, timeout)) or SimpleNamespace(
        error_code=SimpleNamespace(val=MoveItErrorCodes.NO_IK_SOLUTION))
    seed = JointSolution(ARM_JOINT_NAMES['left'], (0., 0., 0., 0., 0.))
    assert adapter._solve_aim_tip_position_ik('left', (.4, 0., .4), seed) is None
    request, timeout = calls[0]
    assert request.ik_request.timeout.sec == 1
    assert request.ik_request.timeout.nanosec == 0
    assert timeout == 6.


@pytest.mark.parametrize('valid', [False, True])
def test_analytical_roll_candidate_is_collision_checked_before_use(valid):
    pose = PoseStamped().pose
    pose.orientation.w = 1.
    checked = []
    adapter = MoveItGraspAdapter(object(), config=MoveItAdapterConfig(
        align_grasp_wrist_roll=True, grasp_closing_sign_invariant=False),
        ik_client=object(), fk_client=object(), validity_client=object(), plan_client=object())
    adapter._grasp_pose = lambda arm, solution: pose
    adapter.state_is_valid = lambda arm, state: checked.append(state) or valid
    original = JointSolution(ARM_JOINT_NAMES['left'], (0., 1., 2., .5, 0.))
    corrected = adapter._aligned_grasp_solution('left', original, (0., 0., 1.))
    assert len(checked) == 1
    assert checked[0].positions[:-1] == original.positions[:-1]
    assert checked[0].positions[-1] == pytest.approx(math.pi/2)
    assert corrected == (checked[0] if valid else None)


def test_visibility_checks_complete_state_and_requires_an_evaluation_result():
    from moveit_msgs.msg import ConstraintEvalResult, VisibilityConstraint
    response = SimpleNamespace(valid=True, constraint_result=[ConstraintEvalResult(result=True)])
    client = ServiceClient(response)
    adapter = MoveItGraspAdapter(object(), ik_client=object(), fk_client=object(),
                                validity_client=client, plan_client=object())
    adapter.set_current_state(current_state())
    joint = JointSolution(ARM_JOINT_NAMES['left'], (0., 1., 1., 0., 0.))
    with pytest.raises(InfrastructureError, match='not configured'):
        adapter.pregrasp_is_visible('left', joint)
    constraint = VisibilityConstraint(target_radius=.08, cone_sides=16)
    adapter.set_visibility_constraint(constraint)
    constraint.target_radius = .01  # Caller mutation must not shrink the check.
    assert adapter.pregrasp_is_visible('left', joint)
    request = client.requests[-1]
    assert request.constraints.visibility_constraints[0].target_radius == .08
    assert tuple(request.robot_state.joint_state.name) == REQUIRED_JOINT_NAMES
    response.constraint_result[0].result = False
    assert not adapter.pregrasp_is_visible('left', joint)
    response.constraint_result = []
    with pytest.raises(InfrastructureError, match='did not evaluate'):
        adapter.pregrasp_is_visible('left', joint)
    adapter.set_visibility_constraint(None)
    with pytest.raises(InfrastructureError, match='not configured'):
        adapter.pregrasp_is_visible('left', joint)


def test_ik_seed_is_a_complete_robot_state_and_preserves_other_arm():
    solution = RobotState()
    solution.joint_state = current_state()
    response = SimpleNamespace(
        error_code=SimpleNamespace(val=MoveItErrorCodes.SUCCESS),
        solution=solution,
    )
    ik = ServiceClient(response)
    adapter = MoveItGraspAdapter(
        object(),
        ik_client=ik,
        fk_client=object(),
        validity_client=object(),
        plan_client=object(),
    )
    adapter.set_current_state(current_state())
    seed = JointSolution(
        ARM_JOINT_NAMES['left'],
        (10.0, 11.0, 12.0, 13.0, 14.0),
    )
    adapter.solve_position_ik('left', (0.4, 0.2, 0.8), seed)
    request = ik.requests[0].ik_request
    assert request.group_name == 'left_grasp_arm'
    assert request.ik_link_name == 'left_grasp_tcp'
    assert request.avoid_collisions is True
    assert tuple(request.robot_state.joint_state.name) == REQUIRED_JOINT_NAMES
    values = dict(
        zip(
            request.robot_state.joint_state.name,
            request.robot_state.joint_state.position,
            strict=True,
        )
    )
    assert tuple(values[name] for name in ARM_JOINT_NAMES['left']) == seed.positions
    assert values['right_shoulder_yaw_joint'] == 6.0


@pytest.mark.parametrize('response_margin', [1., 5.])
def test_second_segment_has_explicit_pregrasp_start_state(response_margin):
    trajectory = SimpleNamespace(
        joint_trajectory=SimpleNamespace(points=[JointTrajectoryPoint()])
    )
    wrapped = SimpleNamespace(
        status=GoalStatus.STATUS_SUCCEEDED,
        result=SimpleNamespace(
            error_code=SimpleNamespace(val=MoveItErrorCodes.SUCCESS),
            planned_trajectory=trajectory,
        ),
    )
    plan = PlanClient(wrapped)
    adapter = MoveItGraspAdapter(
        object(),
        config=MoveItAdapterConfig(planning_response_margin_sec=response_margin),
        ik_client=object(),
        fk_client=object(),
        validity_client=object(),
        plan_client=plan,
    )
    adapter.set_current_state(current_state())
    response_timeouts = []
    adapter._wait_future = lambda future, timeout: response_timeouts.append(timeout) or future.result()
    pregrasp = JointSolution(
        ARM_JOINT_NAMES['right'],
        (20.0, 21.0, 22.0, 23.0, 24.0),
    )
    grasp = JointSolution(
        ARM_JOINT_NAMES['right'],
        (25.0, 26.0, 27.0, 28.0, 29.0),
    )
    assert adapter.plan('right', grasp, pregrasp)
    request = plan.goals[0].request
    assert request.allowed_planning_time == 4.
    assert response_timeouts == [4.+response_margin, 4.+response_margin]
    assert plan.goals[0].planning_options.plan_only is True
    assert request.group_name == 'right_grasp_arm'
    start = dict(
        zip(
            request.start_state.joint_state.name,
            request.start_state.joint_state.position,
            strict=True,
        )
    )
    assert tuple(start[name] for name in ARM_JOINT_NAMES['right']) == pregrasp.positions
    constraints = request.goal_constraints[0].joint_constraints
    assert [constraint.joint_name for constraint in constraints] == list(
        ARM_JOINT_NAMES['right']
    )


def test_carry_ik_preserves_scene_attachments_and_complete_joint_feedback():
    solution = RobotState(joint_state=current_state())
    ik = ServiceClient(SimpleNamespace(
        error_code=SimpleNamespace(val=MoveItErrorCodes.SUCCESS),
        solution=solution,
    ))
    adapter = MoveItGraspAdapter(
        object(), MoveItAdapterConfig(preserve_scene_attachments=True),
        ik_client=ik, fk_client=object(), validity_client=object(),
        plan_client=object(),
    )
    adapter.set_current_state(current_state())
    adapter.solve_position_ik('left', (0.2, 0.5, 0.4), None)
    state = ik.requests[0].ik_request.robot_state
    assert state.is_diff  # Empty attachment array must not detach the payload.
    assert tuple(state.joint_state.name) == REQUIRED_JOINT_NAMES
    assert list(state.joint_state.position) == list(current_state().position)


def test_grasp_search_explores_whole_arm_seeds_without_relaxing_pose(monkeypatch):
    adapter = MoveItGraspAdapter(
        object(), MoveItAdapterConfig(
            pregrasp_aim_attempts=1, grasp_pose_seed_attempts=2),
        ik_client=object(), fk_client=object(), validity_client=object(),
        plan_client=object(),
    )
    seed = JointSolution(ARM_JOINT_NAMES['left'], (0., 1., 1., 0., 0.))
    alternate = JointSolution(seed.names, (0.5, 1.5, 2., 0., 0.))
    calls = []

    def extra_seeds(arm, original, count, *, target_position):
        assert (arm, original, count, target_position) == (
            'left', seed, 2, (0.5, 0.2, 0.8))
        return (alternate,)

    def solve(arm, point, trial):
        calls.append(trial)
        return trial

    monkeypatch.setattr(adapter, '_aim_seed_solutions', extra_seeds)
    monkeypatch.setattr(adapter, 'solve_position_ik', solve)
    monkeypatch.setattr(adapter, '_grasp_pose_errors',
                        lambda arm, trial, *args:
                        (0., 0., 0. if trial == alternate else 90.))
    result = adapter.solve_grasp_ik(
        'left', (0.5, 0.2, 0.8), (0., -1., 0.), (1., 0., 0.), seed)
    assert calls == [seed, alternate]
    assert result == (alternate,)


def _fk_response(tcp, aim):
    tcp_pose = PoseStamped()
    (
        tcp_pose.pose.position.x,
        tcp_pose.pose.position.y,
        tcp_pose.pose.position.z,
    ) = tcp
    tcp_pose.pose.orientation.w = 1.0
    aim_pose = PoseStamped()
    (
        aim_pose.pose.position.x,
        aim_pose.pose.position.y,
        aim_pose.pose.position.z,
    ) = aim
    return SimpleNamespace(
        error_code=SimpleNamespace(val=MoveItErrorCodes.SUCCESS),
        pose_stamped=[tcp_pose, aim_pose],
    )


def _grasp_fk_response(tcp):
    tcp_pose = PoseStamped()
    (
        tcp_pose.pose.position.x,
        tcp_pose.pose.position.y,
        tcp_pose.pose.position.z,
    ) = tcp
    tcp_pose.pose.orientation.w = 1.0
    return SimpleNamespace(
        error_code=SimpleNamespace(val=MoveItErrorCodes.SUCCESS),
        pose_stamped=[tcp_pose],
    )


def test_aimed_pregrasp_uses_virtual_tip_and_accepts_matching_direction():
    solution = RobotState()
    solution.joint_state = current_state()
    ik = ServiceClient(
        SimpleNamespace(
            error_code=SimpleNamespace(val=MoveItErrorCodes.SUCCESS),
            solution=solution,
        )
    )
    fk = ServiceClient(
        _fk_response((0.36, 0.2, 0.8), (0.5, 0.2, 0.8))
    )
    adapter = MoveItGraspAdapter(
        object(),
        ik_client=ik,
        fk_client=fk,
        validity_client=object(),
        plan_client=object(),
    )
    adapter.set_current_state(current_state())
    seed = JointSolution(ARM_JOINT_NAMES['left'], (1.0, 2.0, 3.0, 4.0, 5.0))

    result = adapter.solve_aimed_pregrasp_ik(
        'left',
        (0.5, 0.2, 0.8),
        (1.0, 0.0, 0.0),
        (1.0, 0.0, 0.0),
        (0.36, 0.2, 0.8),
        seed,
    )

    assert result
    request = ik.requests[0].ik_request
    assert request.group_name == 'left_pregrasp_aim_arm'
    assert request.ik_link_name == 'left_pregrasp_aim_tip'
    assert (
        request.pose_stamped.pose.position.x,
        request.pose_stamped.pose.position.y,
        request.pose_stamped.pose.position.z,
    ) == (0.5, 0.2, 0.8)
    assert fk.requests[0].fk_link_names == [
        'left_grasp_tcp',
        'left_pregrasp_aim_tip',
    ]


def test_aimed_pregrasp_uses_current_arm_state_when_seed_ik_failed():
    solution = RobotState()
    solution.joint_state = current_state()
    ik = ServiceClient(
        SimpleNamespace(
            error_code=SimpleNamespace(val=MoveItErrorCodes.SUCCESS),
            solution=solution,
        )
    )
    fk = ServiceClient(
        _fk_response((0.36, 0.2, 0.8), (0.5, 0.2, 0.8))
    )
    adapter = MoveItGraspAdapter(
        object(),
        ik_client=ik,
        fk_client=fk,
        validity_client=object(),
        plan_client=object(),
    )
    state = current_state()
    adapter.set_current_state(state)

    result = adapter.solve_aimed_pregrasp_ik(
        'left',
        (0.5, 0.2, 0.8),
        (1.0, 0.0, 0.0),
        (1.0, 0.0, 0.0),
        (0.36, 0.2, 0.8),
        None,
    )

    assert result
    requested = dict(
        zip(
            ik.requests[0].ik_request.robot_state.joint_state.name,
            ik.requests[0].ik_request.robot_state.joint_state.position,
            strict=True,
        )
    )
    current = dict(zip(state.name, state.position, strict=True))
    assert all(
        requested[name] == current[name]
        for name in ARM_JOINT_NAMES['left']
    )


def test_aimed_pregrasp_rejects_fk_direction_that_disagrees_with_candidate():
    solution = RobotState()
    solution.joint_state = current_state()
    ik = ServiceClient(
        SimpleNamespace(
            error_code=SimpleNamespace(val=MoveItErrorCodes.SUCCESS),
            solution=solution,
        )
    )
    fk = ServiceClient(
        _fk_response((0.5, 0.34, 0.8), (0.5, 0.2, 0.8))
    )
    adapter = MoveItGraspAdapter(
        object(),
        ik_client=ik,
        fk_client=fk,
        validity_client=object(),
        plan_client=object(),
    )
    adapter.set_current_state(current_state())
    seed = JointSolution(ARM_JOINT_NAMES['left'], (1.0, 2.0, 3.0, 4.0, 5.0))

    result = adapter.solve_aimed_pregrasp_ik(
        'left',
        (0.5, 0.2, 0.8),
        (1.0, 0.0, 0.0),
        (1.0, 0.0, 0.0),
        (0.36, 0.2, 0.8),
        seed,
    )

    assert result == ()


def test_aimed_pregrasp_rejects_ray_beyond_maximum_approach_error():
    solution = RobotState()
    solution.joint_state = current_state()
    ik = ServiceClient(
        SimpleNamespace(
            error_code=SimpleNamespace(val=MoveItErrorCodes.SUCCESS),
            solution=solution,
        )
    )
    component = 0.14 / math.sqrt(2.0)
    fk = ServiceClient(
        _fk_response(
            (0.5 - component, 0.2, 0.8 - component),
            (0.5, 0.2, 0.8),
        )
    )
    adapter = MoveItGraspAdapter(
        object(),
        ik_client=ik,
        fk_client=fk,
        validity_client=object(),
        plan_client=object(),
    )
    adapter.set_current_state(current_state())
    seed = JointSolution(ARM_JOINT_NAMES['left'], (1.0, 2.0, 3.0, 4.0, 5.0))

    result = adapter.solve_aimed_pregrasp_ik(
        'left',
        (0.5, 0.2, 0.8),
        (1.0, 0.0, 0.0),
        (1.0, 0.0, 0.0),
        (0.36, 0.2, 0.8),
        seed,
    )

    assert result == ()


def test_grasp_ik_is_fk_verified_against_position_and_both_axes():
    solution = RobotState()
    solution.joint_state = current_state()
    ik = ServiceClient(
        SimpleNamespace(
            error_code=SimpleNamespace(val=MoveItErrorCodes.SUCCESS),
            solution=solution,
        )
    )
    fk = ServiceClient(_grasp_fk_response((0.5, 0.2, 0.8)))
    adapter = MoveItGraspAdapter(
        object(),
        ik_client=ik,
        fk_client=fk,
        validity_client=object(),
        plan_client=object(),
    )
    adapter.set_current_state(current_state())
    seed = JointSolution(ARM_JOINT_NAMES['left'], (1.0, 2.0, 3.0, 0.0, 0.0))

    accepted = adapter.solve_grasp_ik(
        'left',
        (0.5, 0.2, 0.8),
        (0.0, -1.0, 0.0),
        (1.0, 0.0, 0.0),
        seed,
    )
    rejected = adapter.solve_grasp_ik(
        'left',
        (0.5, 0.2, 0.8),
        (1.0, 0.0, 0.0),
        (1.0, 0.0, 0.0),
        seed,
    )

    assert accepted
    assert rejected == ()
    assert fk.requests[0].fk_link_names == ['left_grasp_tcp']


def test_asymmetric_grasp_rejects_closing_axis_sign_flip() -> None:
    solution = RobotState()
    solution.joint_state = current_state()
    ik = ServiceClient(
        SimpleNamespace(
            error_code=SimpleNamespace(val=MoveItErrorCodes.SUCCESS),
            solution=solution,
        )
    )
    response = _grasp_fk_response((0.5, 0.2, 0.8))
    response.pose_stamped[0].pose.orientation.y = 1.0
    response.pose_stamped[0].pose.orientation.w = 0.0
    fk = ServiceClient(response)
    seed = JointSolution(
        ARM_JOINT_NAMES['left'], (1.0, 2.0, 3.0, 0.0, 0.0)
    )

    symmetric = MoveItGraspAdapter(
        object(),
        ik_client=ik,
        fk_client=fk,
        validity_client=object(),
        plan_client=object(),
    )
    symmetric.set_current_state(current_state())
    asymmetric = MoveItGraspAdapter(
        object(),
        MoveItAdapterConfig(grasp_closing_sign_invariant=False),
        ik_client=ik,
        fk_client=fk,
        validity_client=object(),
        plan_client=object(),
    )
    asymmetric.set_current_state(current_state())

    args = (
        'left',
        (0.5, 0.2, 0.8),
        (0.0, -1.0, 0.0),
        (1.0, 0.0, 0.0),
        seed,
    )
    assert symmetric.solve_grasp_ik(*args)
    assert asymmetric.solve_grasp_ik(*args) == ()


def test_asymmetric_pregrasp_also_rejects_closing_axis_sign_flip() -> None:
    solution = RobotState()
    solution.joint_state = current_state()
    ik = ServiceClient(SimpleNamespace(
        error_code=SimpleNamespace(val=MoveItErrorCodes.SUCCESS),
        solution=solution,
    ))
    response = _fk_response((0.5, 0.34, 0.8), (0.5, 0.2, 0.8))
    # A half-turn around local Y preserves approach but reverses the fixed jaw.
    response.pose_stamped[0].pose.orientation.y = 1.0
    response.pose_stamped[0].pose.orientation.w = 0.0
    fk = ServiceClient(response)
    seed = JointSolution(ARM_JOINT_NAMES['left'], (1.0, 2.0, 3.0, 0.0, 0.0))
    for symmetric in (True, False):
        adapter = MoveItGraspAdapter(
            object(), MoveItAdapterConfig(grasp_closing_sign_invariant=symmetric),
            ik_client=ik, fk_client=fk, validity_client=object(), plan_client=object(),
        )
        adapter.set_current_state(current_state())
        result = adapter.solve_aimed_pregrasp_ik(
            'left', (0.5, 0.2, 0.8), (0.0, -1.0, 0.0), (1.0, 0.0, 0.0),
            (0.5, 0.34, 0.8), seed,
        )
        assert bool(result) is symmetric
        # The same asymmetric mode must still accept a correctly directed jaw.
        response.pose_stamped[0].pose.orientation.y = 0.0
        response.pose_stamped[0].pose.orientation.w = 1.0
        assert adapter.solve_aimed_pregrasp_ik(
            'left', (0.5, 0.2, 0.8), (0.0, -1.0, 0.0), (1.0, 0.0, 0.0),
            (0.5, 0.34, 0.8), seed,
        )
        response.pose_stamped[0].pose.orientation.y = 1.0
        response.pose_stamped[0].pose.orientation.w = 0.0


def test_wrist_roll_seeds_are_distributed_inside_robot_limits():
    config = MoveItAdapterConfig(pregrasp_aim_attempts=8)
    adapter = MoveItGraspAdapter(
        object(),
        config,
        ik_client=object(),
        fk_client=object(),
        validity_client=object(),
        plan_client=object(),
    )
    seed = JointSolution(ARM_JOINT_NAMES['left'], (0.0, 0.0, 0.0, 0.0, 10.0))

    values = adapter._wrist_roll_seeds(seed, 8)

    assert len(values) == 8
    assert min(values) == config.wrist_roll_lower_rad
    assert max(values) == config.wrist_roll_upper_rad
    assert all(
        config.wrist_roll_lower_rad
        <= value
        <= config.wrist_roll_upper_rad
        for value in values
    )


def test_aimed_pregrasp_seeds_cover_all_arm_joints():
    adapter = MoveItGraspAdapter(
        object(),
        ik_client=object(),
        fk_client=object(),
        validity_client=object(),
        plan_client=object(),
    )
    adapter.set_current_state(current_state())
    candidate_seed = JointSolution(
        ARM_JOINT_NAMES['left'],
        (0.1, 0.2, 0.3, 0.4, 0.5),
    )

    seeds = adapter._aim_seed_solutions('left', candidate_seed, 8)

    assert len(seeds) == 8
    assert seeds[0] == candidate_seed
    for joint_index in range(5):
        assert len({seed.positions[joint_index] for seed in seeds}) > 2


def test_aimed_pregrasp_prioritizes_location_seed_wrist_variants():
    adapter = MoveItGraspAdapter(
        object(),
        ik_client=object(),
        fk_client=object(),
        validity_client=object(),
        plan_client=object(),
    )
    adapter.set_current_state(current_state())
    candidate_seed = JointSolution(
        ARM_JOINT_NAMES['left'],
        (0.1, 0.2, 0.3, 0.4, 0.5),
    )

    positive = adapter._aim_seed_solutions(
        'left',
        candidate_seed,
        8,
        target_position=(0.45, 0.18, 0.40),
    )
    negative = adapter._aim_seed_solutions(
        'left',
        candidate_seed,
        8,
        target_position=(0.45, -0.18, 0.40),
    )
    without_target = adapter._aim_seed_solutions('left', candidate_seed, 8)

    assert positive[0] == candidate_seed
    assert positive[1].positions[:4] == candidate_seed.positions[:4]
    assert positive[1].positions[-1] > negative[1].positions[-1]
    assert without_target[0] == candidate_seed
    assert without_target[1] == adapter._current_arm_solution('left')


def test_aimed_pregrasp_seeds_fill_attempts_when_initial_seeds_duplicate():
    adapter = MoveItGraspAdapter(
        object(),
        ik_client=object(),
        fk_client=object(),
        validity_client=object(),
        plan_client=object(),
    )
    adapter.set_current_state(current_state())
    candidate_seed = adapter._current_arm_solution('left')

    seeds = adapter._aim_seed_solutions('left', candidate_seed, 16)

    assert len(seeds) == 16
    assert len({seed.positions for seed in seeds}) == 16


def test_valid_ik_solutions_prefer_normalized_motion_with_double_wrist_weight():
    adapter = MoveItGraspAdapter(
        object(),
        ik_client=object(),
        fk_client=object(),
        validity_client=object(),
        plan_client=object(),
    )
    adapter.set_current_state(current_state())
    names = ARM_JOINT_NAMES['left']
    near = JointSolution(names, (0.01, 1.01, 2.01, 3.01, 4.05))
    wrist_far = JointSolution(names, (0.0, 1.0, 2.0, 3.0, 3.0))
    ranked = adapter._unique_ranked_solutions(
        [
            (0.0, 0.0, 0.0, wrist_far, 0.0, 0.0),
            (5.0, 5.0, 0.001, near, 0.0, 0.0),
        ]
    )
    assert ranked == (near, wrist_far)


def test_open_clearance_query_overrides_only_selected_gripper_and_keeps_feedback():
    valid = ServiceClient(SimpleNamespace(valid=False))
    adapter = MoveItGraspAdapter(object(), MoveItAdapterConfig(),
        ik_client=object(), fk_client=object(), validity_client=valid, plan_client=object())
    state = current_state()
    original = dict(zip(state.name, state.position))
    adapter.set_current_state(state)
    solution = JointSolution(ARM_JOINT_NAMES['left'], (0., 1., 1., 0., 0.))
    assert not adapter.open_grasp_is_valid('left', solution, 1.4)
    request = valid.requests[-1]
    positions = dict(zip(request.robot_state.joint_state.name, request.robot_state.joint_state.position))
    assert request.group_name == ''
    assert positions['left_gripper_joint'] == 1.4
    assert positions['right_gripper_joint'] == original['right_gripper_joint']
    assert dict(zip(state.name, state.position)) == original


@pytest.mark.parametrize('blocked', [False, True])
def test_closure_sweep_checks_intermediate_states_and_stops_on_collision(blocked):
    adapter = MoveItGraspAdapter(object(), MoveItAdapterConfig(),
        ik_client=object(), fk_client=object(), validity_client=object(), plan_client=object())
    samples = []
    def check(arm, solution, position):
        samples.append(position)
        return not (blocked and .25 <= position <= .35)
    adapter._gripper_state_is_valid = check
    assert adapter.gripper_sweep_is_valid('left', object(), 1.4, -.3, .05) is not blocked
    assert samples[0] == 1.4
    assert max(abs(a-b) for a, b in zip(samples, samples[1:])) <= .05 + 1e-12
    if blocked:
        assert .25 <= samples[-1] <= .35
    else:
        assert samples[-1] == -.3


@pytest.mark.parametrize('opening,closing,step', [(1.4,-.4,.05), (1.4,1.5,.05),
                                                   (float('nan'),-.3,.05), (1.4,-.3,0.)])
def test_invalid_closure_sweep_rejected(opening, closing, step):
    adapter = MoveItGraspAdapter(object(), MoveItAdapterConfig(),
        ik_client=object(), fk_client=object(), validity_client=object(), plan_client=object())
    with pytest.raises(ValueError, match='sweep'):
        adapter.gripper_sweep_is_valid('left', object(), opening, closing, step)


def test_grasp_diagnostic_reports_first_ranked_solution_not_unselected_best_pose(monkeypatch):
    logs = []
    node = SimpleNamespace(get_logger=lambda: SimpleNamespace(info=logs.append))
    adapter = MoveItGraspAdapter(node, MoveItAdapterConfig(pregrasp_aim_attempts=2),
        ik_client=object(), fk_client=object(), validity_client=object(), plan_client=object())
    adapter.set_current_state(current_state())
    seed = adapter._current_arm_solution('left')
    near = adapter._with_wrist_roll(seed, 2.)
    far = adapter._with_wrist_roll(seed, -2.)
    monkeypatch.setattr(adapter, '_wrist_roll_seeds', lambda *args: (2., -2.))
    monkeypatch.setattr(adapter, 'solve_position_ik', lambda arm, position, trial: trial)
    monkeypatch.setattr(adapter, '_grasp_pose_errors', lambda arm, trial, *args:
                        (0., 5., 5.) if trial == near else (0., 1., 1.))
    ranked = adapter.solve_grasp_ik('left', (.4, .2, .4), (0., 0., -1.), (1., 0., 0.), seed)
    assert ranked == (near, far)
    assert len(logs) == 1
    assert 'First ranked grasp: approach_error=5.00deg' in logs[0]


@pytest.mark.parametrize('arm', ['left', 'right'])
@pytest.mark.parametrize('value,accepted', [(1.65806272933, False), (1.654, False),
                                          (1.650, True), (-1.658, False), (-1.650, True)])
def test_selection_avoids_saturated_joint_endpoints_without_relaxing_bounds(arm, value, accepted):
    valid = ServiceClient(SimpleNamespace(valid=True))
    adapter = MoveItGraspAdapter(object(), MoveItAdapterConfig(joint_limit_margin_rad=.005),
        ik_client=object(), fk_client=object(), validity_client=valid, plan_client=object())
    adapter.set_current_state(current_state())
    solution = JointSolution(ARM_JOINT_NAMES[arm], (0., 1., 1., value, 0.))
    assert adapter.state_is_valid(arm, solution) == accepted
    assert bool(valid.requests) == accepted
    ranked = adapter._unique_ranked_solutions([(1., 1., 0., solution, 0., 0.)])
    assert ranked == ((solution,) if accepted else ())


@pytest.mark.parametrize('margin', [-.01, float('nan'), float('inf'), 2.])
def test_invalid_endpoint_margin_fails_closed(margin):
    with pytest.raises(ValueError, match='margin'):
        MoveItAdapterConfig(joint_limit_margin_rad=margin)
@pytest.mark.parametrize('refinement', [0, 1])
def test_grasp_does_not_inherit_loose_pregrasp_position_tolerance(refinement):
    solution = RobotState(joint_state=current_state())
    adapter = MoveItGraspAdapter(object(), config=MoveItAdapterConfig(
        pregrasp_position_tolerance_m=.020, grasp_position_tolerance_m=.0015,
        pose_refinement_iterations=refinement, pregrasp_aim_attempts=1),
        ik_client=ServiceClient(SimpleNamespace(error_code=SimpleNamespace(val=1), solution=solution)),
        fk_client=ServiceClient(_grasp_fk_response((.51, .2, .8))),
        validity_client=object(), plan_client=object())
    adapter.set_current_state(current_state())
    adapter._local_fk = {'left': SimpleNamespace(pose=lambda _: (np.array((.51,.2,.8)), np.eye(3)))}
    seed = JointSolution(ARM_JOINT_NAMES['left'], (1., 2., 3., 0., 0.))
    assert adapter.solve_grasp_ik('left', (.5,.2,.8), (0.,-1.,0.), (1.,0.,0.), seed) == ()


@pytest.mark.parametrize('grasp_attempts,expected', [(0, 12), (16, 16)])
def test_refined_grasp_honors_configured_seed_budget(grasp_attempts, expected):
    adapter = MoveItGraspAdapter(object(), config=MoveItAdapterConfig(
        pose_refinement_iterations=1, pregrasp_aim_attempts=12,
        grasp_pose_seed_attempts=grasp_attempts),
        ik_client=object(), fk_client=ServiceClient(_grasp_fk_response((.5, .2, .8))),
        validity_client=object(), plan_client=object())
    adapter.set_current_state(current_state())
    adapter._local_fk = {'left': SimpleNamespace(
        pose=lambda _: (np.array((.5, .2, .8)), np.eye(3)))}
    calls = []
    def seeds(arm, seed, attempts, *, target_position):
        calls.append(attempts)
        return ()
    adapter._aim_seed_solutions = seeds
    seed = JointSolution(ARM_JOINT_NAMES['left'], (1., 2., 3., 0., 0.))
    assert adapter.solve_grasp_ik('left', (.5,.2,.8), (0.,-1.,0.), (1.,0.,0.), seed) == ()
    assert calls == [expected]
