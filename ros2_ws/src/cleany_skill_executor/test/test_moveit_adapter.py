from types import SimpleNamespace

from action_msgs.msg import GoalStatus
from moveit_msgs.msg import MoveItErrorCodes, RobotState
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

from cleany_skill_executor.core.grasp_selection import ARM_JOINT_NAMES, JointSolution, REQUIRED_JOINT_NAMES
from cleany_skill_executor.moveit_adapter import MoveItGraspAdapter


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
    adapter = MoveItGraspAdapter(object(), ik_client=ik, validity_client=object(), plan_client=object())
    adapter.set_current_state(current_state())
    seed = JointSolution(ARM_JOINT_NAMES['left'], (10.0, 11.0, 12.0, 13.0, 14.0))
    adapter.solve_position_ik('left', (0.4, 0.2, 0.8), seed)
    request = ik.requests[0].ik_request
    assert request.group_name == 'left_grasp_arm'
    assert request.ik_link_name == 'left_grasp_tcp'
    assert request.avoid_collisions is True
    assert tuple(request.robot_state.joint_state.name) == REQUIRED_JOINT_NAMES
    values = dict(zip(request.robot_state.joint_state.name, request.robot_state.joint_state.position))
    assert tuple(values[name] for name in ARM_JOINT_NAMES['left']) == seed.positions
    assert values['right_shoulder_yaw_joint'] == 6.0


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
    pregrasp = JointSolution(ARM_JOINT_NAMES['right'], (20.0, 21.0, 22.0, 23.0, 24.0))
    grasp = JointSolution(ARM_JOINT_NAMES['right'], (25.0, 26.0, 27.0, 28.0, 29.0))
    assert adapter.plan('right', grasp, pregrasp)
    request = plan.goals[0].request
    assert plan.goals[0].planning_options.plan_only is True
    assert request.group_name == 'right_grasp_arm'
    start = dict(zip(request.start_state.joint_state.name, request.start_state.joint_state.position))
    assert tuple(start[name] for name in ARM_JOINT_NAMES['right']) == pregrasp.positions
    assert [constraint.joint_name for constraint in request.goal_constraints[0].joint_constraints] == list(ARM_JOINT_NAMES['right'])
