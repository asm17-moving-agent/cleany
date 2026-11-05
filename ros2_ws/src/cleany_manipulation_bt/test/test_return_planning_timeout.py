"""Load the deployed return timing into the real adapter; no motor I/O."""
from pathlib import Path
from types import SimpleNamespace as NS
import pytest
import rclpy
from action_msgs.msg import GoalStatus
from moveit_msgs.msg import MoveItErrorCodes, RobotState, RobotTrajectory
from rclpy.context import Context
from rclpy.parameter import Parameter
from rclpy.task import Future
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import yaml
from cleany_skill_executor.core.physical_grasp_selection import (
    ARM_JOINT_NAMES, InfrastructureError, JointSolution,
)
from cleany_skill_executor.pregrasp_pipeline import PregraspPipeline


@pytest.fixture
def pipeline_factory():
    context = Context()
    rclpy.init(context=context)
    nodes = []

    def create(parameters):
        node = object.__new__(PregraspPipeline)
        nodes.append(node)
        PregraspPipeline.__init__(node, mode='mujoco', context=context,
            parameter_overrides=[Parameter('use_sim_time', value=True), *parameters])
        return node

    try:
        yield create
    finally:
        for node in nodes:
            node.destroy_node()
        context.shutdown()


@pytest.mark.parametrize('response_sec,accepted', [(6.212312, True), (25., False)])
def test_deployed_return_profile_accepts_logged_delay_and_still_bounds_wait(
        pipeline_factory, response_sec, accepted):
    config = yaml.safe_load((Path(__file__).resolve().parents[1] /
                             'config/manipulation_jetson.yaml').read_text())['/**']['ros__parameters']
    node = pipeline_factory([Parameter(name, value=config[name]) for name in (
        'planning_timeout_sec', 'planning_response_margin_sec')])
    adapter = node.motion
    names = ARM_JOINT_NAMES['left']
    goal = JointSolution(names, (.1,)*5)
    trajectory = JointTrajectory(joint_names=list(names), points=[
        JointTrajectoryPoint(positions=[0.]*5),
        JointTrajectoryPoint(positions=list(goal.positions))])
    result = Future()
    clock, requests = [0.], []

    def send(request):
        assert request.planning_options.plan_only
        assert request.request.allowed_planning_time == 4.
        requests.append(request)
        response = Future()
        response.set_result(NS(accepted=True, get_result_async=lambda: result))
        return response

    def spin(delay):
        clock[0] += delay
        if clock[0] >= response_sec and not result.done():
            result.set_result(NS(status=GoalStatus.STATUS_SUCCEEDED, result=NS(
                error_code=MoveItErrorCodes(val=MoveItErrorCodes.SUCCESS),
                planned_trajectory=RobotTrajectory(joint_trajectory=trajectory))))

    adapter._plan_client = NS(wait_for_server=lambda **_: True, send_goal_async=send)
    adapter._spin_once, adapter._monotonic = spin, lambda: clock[0]
    adapter._merged_state = lambda _: RobotState()
    adapter._validated_cubic_path = lambda _, path: path
    if accepted:
        path = adapter.plan('left', goal, None)
        assert path.points[-1] == goal.positions
        assert clock[0] == pytest.approx(response_sec, abs=.011)
    else:
        with pytest.raises(InfrastructureError, match='MoveGroup action timed out'):
            adapter.plan('left', goal, None)
        assert clock[0] == pytest.approx(24., abs=.011)
    assert len(requests) == 1


@pytest.mark.parametrize('name', ['planning_timeout_sec', 'planning_response_margin_sec'])
@pytest.mark.parametrize('value', [0., -1., float('nan'), float('inf')])
def test_invalid_planning_timeouts_are_rejected_before_adapter_creation(
        pipeline_factory, name, value):
    with pytest.raises(ValueError, match=name):
        pipeline_factory([Parameter(name, value=value)])
