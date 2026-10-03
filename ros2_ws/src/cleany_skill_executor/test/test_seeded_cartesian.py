from copy import deepcopy
import math
from types import SimpleNamespace

from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import (
    AttachedCollisionObject,
    Constraints,
    ContactInformation,
    JointConstraint,
    MotionPlanRequest,
    PositionConstraint,
)
from moveit_msgs.srv import GetPositionFK, GetPositionIK, GetStateValidity
import numpy as np
import pytest
from shape_msgs.msg import SolidPrimitive

from cleany_skill_executor.core.cartesian import CartesianPose, validate_corridor_samples
from cleany_skill_executor.core.joint_path import CubicJointPath
from cleany_skill_executor.core.pose_refinement import refine_pose
from cleany_skill_executor.core.urdf_fk import UrdfChain
from cleany_skill_executor.seeded_cartesian import (
    JointMotionLimit,
    SeededCartesianConfig,
    SeededCartesianPlanner,
)


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


def test_local_refinement_uses_intersection_of_joint_path_bounds(runtime, monkeypatch):
    from cleany_skill_executor import seeded_cartesian as module
    original = module.refine_pose
    seen = []
    def refine(residual, seed, bounds, iterations):
        seen.append(bounds)
        return original(residual, seed, bounds, iterations)
    monkeypatch.setattr(module, 'refine_pose', refine)
    runtime.planner.config = SeededCartesianConfig(local_refinement_iterations=40)
    runtime.planner.set_robot_description('''<robot name="test">
      <joint name="left_test" type="prismatic"><parent link="base_link"/>
      <child link="left_grasp_tcp"/><axis xyz="1 0 0"/></joint></robot>''')
    runtime.request.path_constraints.joint_constraints = [JointConstraint(
        joint_name='left_test', position=.025, tolerance_below=.026, tolerance_above=.026)]
    runtime.planner.plan(runtime.request, runtime.start, runtime.target)
    assert seen
    assert all(bounds[0] == pytest.approx((-.001, .051)) for bounds in seen)


def test_old_endpoint_outside_wrist_bound_is_rejected_before_services(runtime):
    runtime.request.path_constraints.joint_constraints = [JointConstraint(
        joint_name='left_test', position=0., tolerance_below=.02, tolerance_above=.02)]
    with pytest.raises(RuntimeError, match='endpoint violates'):
        runtime.planner.plan(runtime.request, runtime.start, runtime.target)
    assert not runtime.ik_requests and not runtime.states


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


@pytest.mark.parametrize('positions', [[], [(0.,)], [(0.,), (math.nan,)], [(0.,), (1., 2.)]])
def test_invalid_path_rejected(positions):
    with pytest.raises(ValueError):
        CubicJointPath(positions)


def test_starts_and_ends_at_rest_without_joint_overshoot():
    path = CubicJointPath([(0., 1.), (.2, .9), (.5, .6), (.51, .7)])
    samples = path.samples(.002, 5)
    assert samples[0].positions == (0., 1.)
    assert samples[-1].positions == pytest.approx((.51, .7))
    assert samples[0].derivatives == (0., 0.)
    assert samples[-1].derivatives == pytest.approx((0., 0.), abs=1e-14)
    for segment in range(3):
        for fraction in np.linspace(0., 1., 100):
            sample = path.sample(segment, fraction)
            for j in range(2):
                low, high = sorted(path.positions[segment:segment+2, j])
                assert low-1e-12 <= sample.positions[j] <= high+1e-12


def test_analytical_velocity_and_acceleration_bounds_cover_dense_samples():
    path = CubicJointPath([(0., 0.), (.2, -.05), (.21, -.08), (.8, -.05)])
    velocity, acceleration = path.derivative_bounds()
    duration = path.minimum_duration([.1, .08], [.2, .15])
    assert np.all(velocity/duration <= np.array([.1, .08])+1e-12)
    assert np.all(acceleration/duration**2 <= np.array([.2, .15])+1e-12)
    for segment in range(3):
        previous = None
        for fraction in np.linspace(0., 1., 1001):
            sample = path.sample(segment, fraction)
            assert np.all(np.abs(sample.derivatives) <= velocity+1e-12)
            if previous is not None:
                measured = np.abs(np.subtract(sample.derivatives, previous.derivatives))/.001*3
                assert np.all(measured <= acceleration+1e-10)
            previous = sample


def test_dense_samples_reconstruct_same_cubic_used_by_jtc():
    path = CubicJointPath([(0.,), (.2,), (.4,)])
    samples = path.samples(.01, 10)
    for first, second in zip(samples, samples[1:]):
        # Hermite midpoint reconstructed from emitted positions and velocities.
        h = second.progress-first.progress
        midpoint = .5*(first.positions[0]+second.positions[0]) + h/8*(first.derivatives[0]-second.derivatives[0])
        progress = (first.progress+second.progress)/2
        segment = min(int(progress*2), 1)
        assert midpoint == pytest.approx(path.sample(segment, progress*2-segment).positions[0], abs=1e-12)
        assert abs(second.positions[0]-first.positions[0]) <= .01+1e-12


def test_sampling_and_timing_limits_fail_closed():
    path = CubicJointPath([(0.,), (1.,)])
    with pytest.raises(ValueError, match='sample budget'):
        path.samples(.001, 1, 100)
    for limit in (0., -1., math.nan, math.inf):
        with pytest.raises(ValueError):
            path.minimum_duration([limit], [1.])
        with pytest.raises(ValueError):
            path.samples(limit, 1)


def test_local_derivative_sampling_avoids_global_oversampling():
    path = CubicJointPath([(float(i)*.001,) for i in range(100)] + [(1.,)])
    samples = path.samples(.002, 2, maximum_points=2000)
    assert len(samples) < 1000
    assert max(abs(b.positions[0]-a.positions[0]) for a, b in zip(samples, samples[1:])) <= .002+1e-12
    assert samples[0].progress == 0 and samples[-1].progress == 1


def test_bounded_refinement_converges_and_never_queries_outside_limits():
    def residual(q):
        assert np.all(q >= -1) and np.all(q <= 1)
        return np.array([q[0]-.4, np.sin(q[1])-.3])
    result = refine_pose(residual, [-.8,.8], [(-1,1),(-1,1)])
    assert result == pytest.approx((.4,np.arcsin(.3)),abs=1e-4)


def test_unreachable_residual_stays_bounded_and_nan_rejected():
    assert refine_pose(lambda q:q-3,[0.],[(-1,1)])[0] <= 1
    with pytest.raises(ValueError):
        refine_pose(lambda q:np.array([np.nan]),[0.],[(-1,1)])


def test_runtime_urdf_chain_composes_origin_rotation_and_joint_motion():
    xml = '''<robot name="test"><joint name="q" type="revolute">
      <parent link="base"/><child link="arm"/><origin xyz="1 0 0"/>
      <axis xyz="0 0 1"/></joint><joint name="tool" type="fixed">
      <parent link="arm"/><child link="tcp"/><origin xyz="1 0 0"/></joint></robot>'''
    chain = UrdfChain(xml,'base','tcp')
    p,r = chain.pose({'q':np.pi/2})
    np.testing.assert_allclose(p,[1,1,0],atol=1e-12)
    np.testing.assert_allclose(r[:,0],[0,1,0],atol=1e-12)
    with pytest.raises(ValueError):
        UrdfChain(xml,'missing','tcp')
