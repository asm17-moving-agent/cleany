from copy import deepcopy
from types import SimpleNamespace

import pytest
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import (
    AttachedCollisionObject, Constraints, ContactInformation, JointConstraint, MotionPlanRequest, PositionConstraint,
)
from moveit_msgs.srv import GetPositionFK, GetPositionIK, GetStateValidity
from shape_msgs.msg import SolidPrimitive

from cleany_skill_executor.core.cartesian import CartesianPose, validate_corridor_samples
from cleany_skill_executor.seeded_cartesian import JointMotionLimit, SeededCartesianConfig, SeededCartesianPlanner


@pytest.fixture
def runtime():
    request = MotionPlanRequest(group_name='left_grasp_arm', max_velocity_scaling_factor=.2,
                                max_acceleration_scaling_factor=.4)
    request.start_state.is_diff = True
    request.start_state.joint_state.name = ['left_test', 'left_gripper_joint', 'right_test']
    request.start_state.joint_state.position = [0., .99, .3]
    attached = AttachedCollisionObject(link_name='left_gripper_frame')
    attached.object.id = 'observed_target'
    request.start_state.attached_collision_objects = [attached]
    request.goal_constraints = [Constraints(joint_constraints=[JointConstraint(joint_name='left_test', position=.05)])]
    corridor = PositionConstraint(link_name='left_grasp_tcp', weight=1.)
    corridor.header.frame_id = 'base_link'
    corridor.constraint_region.primitives = [SolidPrimitive(type=3, dimensions=[.052, .001])]
    request.path_constraints.position_constraints = [corridor]
    states, ik_requests = [], []

    def ik(query):
        ik_requests.append(deepcopy(query))
        result = GetPositionIK.Response()
        result.error_code.val = 1
        result.solution = deepcopy(query.ik_request.robot_state)
        result.solution.joint_state.position[0] = query.ik_request.pose_stamped.pose.position.x
        return result

    def fk(query):
        result = GetPositionFK.Response()
        result.error_code.val = 1
        pose = PoseStamped()
        pose.pose.position.x = query.robot_state.joint_state.position[0]
        pose.pose.orientation.w = 1.
        result.pose_stamped = [pose]
        return result

    def valid(query):
        states.append(deepcopy(query.robot_state))
        return GetStateValidity.Response(valid=True)

    planner = SeededCartesianPlanner(ik, fk, valid, {'left_test': JointMotionLimit(-1., 1., 1., 1.)},
                                     SeededCartesianConfig())
    start = CartesianPose((0., 0., 0.), (0., 0., 0., 1.))
    target = CartesianPose((.05, 0., 0.), start.orientation)
    return SimpleNamespace(planner=planner, request=request, start=start, target=target,
                           states=states, ik_requests=ik_requests)


def test_uses_collision_aware_ik_and_validates_executed_cubic_samples(runtime):
    result, poses = runtime.planner.plan(runtime.request, runtime.start, runtime.target)
    validate_corridor_samples(poses, runtime.start, runtime.target, .001, .09, .01)
    assert len(runtime.states) == len(result.joint_trajectory.points) > 50
    assert all(query.ik_request.avoid_collisions for query in runtime.ik_requests)
    assert all(query.ik_request.constraints == runtime.request.path_constraints for query in runtime.ik_requests)
    for state in runtime.states + [query.ik_request.robot_state for query in runtime.ik_requests]:
        assert state.is_diff
        assert list(state.joint_state.position[1:]) == [.99, .3]
        assert state.attached_collision_objects[0].object.id == 'observed_target'
    assert list(result.joint_trajectory.points[0].velocities) == [0.]
    assert result.joint_trajectory.points[-1].velocities == pytest.approx([0.], abs=1e-12)
    assert all(len(point.accelerations) == 0 for point in result.joint_trajectory.points)


def test_failed_ik_stops_before_any_trajectory_is_returned(runtime):
    runtime.planner.solve_ik = lambda _: GetPositionIK.Response()
    with pytest.raises(RuntimeError, match='IK failed'):
        runtime.planner.plan(runtime.request, runtime.start, runtime.target)
    assert runtime.states == []


def test_local_refinement_checks_runtime_model_and_retains_all_moveit_validation(runtime):
    runtime.planner.config = SeededCartesianConfig(local_refinement_iterations=40)
    runtime.planner.set_robot_description('''<robot name="test">
      <joint name="left_test" type="prismatic"><parent link="base_link"/>
      <child link="left_grasp_tcp"/><axis xyz="1 0 0"/></joint></robot>''')
    runtime.planner.solve_ik = lambda _: pytest.fail('Local refinement must not use unregularized IK')
    trajectory, poses = runtime.planner.plan(runtime.request, runtime.start, runtime.target)
    assert len(runtime.states) == len(trajectory.joint_trajectory.points)
    validate_corridor_samples(poses, runtime.start, runtime.target, .001, .09, .01)
    runtime.planner.set_robot_description('''<robot name="wrong">
      <joint name="left_test" type="prismatic"><parent link="base_link"/>
      <child link="left_grasp_tcp"/><origin xyz=".01 0 0"/><axis xyz="1 0 0"/></joint></robot>''')
    with pytest.raises(RuntimeError, match='disagrees'):
        runtime.planner.plan(runtime.request, runtime.start, runtime.target)


def test_collision_between_ik_knots_rejects_the_plan(runtime):
    calls = []
    def collision(query):
        calls.append(query.robot_state.joint_state.position[0])
        return GetStateValidity.Response(valid=len(calls) != 7)
    runtime.planner.check_state = collision
    with pytest.raises(RuntimeError, match='cubic collision/constraint failure'):
        runtime.planner.plan(runtime.request, runtime.start, runtime.target)
    assert len(calls) == 7


def test_collision_diagnostic_contains_bounded_contact_location(runtime):
    contact = ContactInformation(contact_body_1='<octomap>', contact_body_2='left_moving_jaw_link', depth=.001)
    contact.header.frame_id = 'base_link'
    contact.position.x, contact.position.y, contact.position.z = .4, .1, .42
    runtime.planner.check_state = lambda _: GetStateValidity.Response(
        valid=False, contacts=[contact]*12)
    with pytest.raises(RuntimeError) as error:
        runtime.planner.plan(runtime.request, runtime.start, runtime.target)
    message = str(error.value)
    assert 'position=(0.4000,0.1000,0.4200) frame=base_link depth=0.001000m' in message
    assert message.count('position=') == 8


def test_position_limit_violation_rejects_before_fk(runtime):
    runtime.planner.limits['left_test'] = JointMotionLimit(-1., .04, 1., 1.)
    with pytest.raises(RuntimeError, match='position limit'):
        runtime.planner.plan(runtime.request, runtime.start, runtime.target)
    assert runtime.states == []


def test_does_not_accept_full_robot_state_that_would_erase_attachments(runtime):
    runtime.request.start_state.is_diff = False
    with pytest.raises(ValueError, match='feedback diff'):
        runtime.planner.plan(runtime.request, runtime.start, runtime.target)


def test_missing_fk_fails_closed(runtime):
    runtime.planner.compute_fk = lambda _: GetPositionFK.Response()
    with pytest.raises(RuntimeError, match='FK unavailable'):
        runtime.planner.plan(runtime.request, runtime.start, runtime.target)
