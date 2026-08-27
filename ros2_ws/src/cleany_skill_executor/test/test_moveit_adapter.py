import math
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


def test_second_segment_has_explicit_pregrasp_start_state():
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
        ik_client=object(),
        fk_client=object(),
        validity_client=object(),
        plan_client=plan,
    )
    adapter.set_current_state(current_state())
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
