from copy import deepcopy
import math
from types import SimpleNamespace

import pytest
from geometry_msgs.msg import Pose
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, RobotTrajectory
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

from cleany_skill_executor.core.cartesian import (
    CartesianPose, execution_wall_timeout, interpolate_orientation, line_corridor, orientation_error_rad,
    sampled_time_scale, validate_corridor_samples, validate_pose_endpoint,
)
from cleany_skill_executor.core.grasp_selection import REQUIRED_JOINT_NAMES, quaternion_axis
from cleany_skill_executor.core.gripper import aligned_wrist_rolls
from cleany_skill_executor.nearest_pregrasp_coordinator import NearestPregraspCoordinator as Coordinator

IDENTITY = (0., 0., 0., 1.)


@pytest.mark.parametrize('duration,factor,expected', [
    (8.763, 2., 60.), (13.863, 10., 148.63), (40., 2., 90.)])
def test_execution_deadline_scales_with_planned_duration(duration, factor, expected):
    assert execution_wall_timeout(duration, factor, 10., 60.) == pytest.approx(expected)


@pytest.mark.parametrize('values', [
    (0.,2.,10.,60.), (1.,.5,10.,60.), (1.,2.,-1.,60.), (1.,2.,10.,0.),
    (float('nan'),2.,10.,60.), (1.,float('inf'),10.,60.), (1e308,10.,10.,60.)])
def test_execution_deadline_rejects_invalid_configuration(values):
    with pytest.raises(ValueError):
        execution_wall_timeout(*values)


@pytest.mark.parametrize('desired,expected', [((0., 0., 1.), math.pi/2),
                                            ((0., 0., -1.), -math.pi/2)])
def test_wrist_alignment_uses_signed_rotation_about_negative_y(desired, expected):
    result = aligned_wrist_rolls(approach=(0., -1., 0.), closing=(1., 0., 0.),
                                 desired=desired, current=0., lower=-2.74, upper=2.84,
                                 sign_invariant=False)
    assert result == pytest.approx([expected])


def test_asymmetric_roll_does_not_accept_reversed_closing_and_respects_limits():
    args = dict(approach=(0., -1., 0.), closing=(1., 0., 0.), desired=(-1., 0., 0.),
                current=0., lower=-2.74, upper=2.84)
    assert aligned_wrist_rolls(**args, sign_invariant=False) == ()
    assert aligned_wrist_rolls(**args, sign_invariant=True) == (0.,)
    args['current'] = .5
    assert aligned_wrist_rolls(**args, sign_invariant=False) == pytest.approx([.5-math.pi])


def test_wrist_alignment_rejects_degenerate_and_nonfinite_vectors():
    args = dict(approach=(0., -1., 0.), closing=(1., 0., 0.), desired=(0., 1., 0.),
                current=0., lower=-2.74, upper=2.84, sign_invariant=False)
    assert aligned_wrist_rolls(**args) == ()
    args['current'] = float('nan')
    with pytest.raises(ValueError):
        aligned_wrist_rolls(**args)


@pytest.mark.parametrize('end', [(0., 0., -1.), (0., 0., 1.), (1., 0., 0.), (.2, -.4, -.7)])
def test_corridor_cylinder_tracks_segment_including_negative_z(end):
    cylinder = line_corridor((0., 0., 0.), end, .001)
    length = math.sqrt(sum(x*x for x in end))
    assert quaternion_axis(cylinder.orientation, (0., 0., 1.)) == pytest.approx([x/length for x in end])
    assert cylinder.center == pytest.approx([x/2 for x in end])
    assert cylinder.length == pytest.approx(length+.002)
    assert cylinder.radius == .001


@pytest.mark.parametrize('position,orientation', [
    ((float('nan'), 0., 0.), IDENTITY), ((0., 0., 0.), (0., 0., 0., 0.)),
    ((0., 0., 0.), (0., 0., 0., float('inf'))),
])
def test_invalid_fk_pose_fails_closed(position, orientation):
    with pytest.raises(ValueError):
        CartesianPose(position, orientation)


def test_position_only_endpoint_regression_from_attempt26():
    target = CartesianPose((.350477811, .127736494, .389190644),
                           (.099400480, .701204399, .700837046, .085201938))
    actual = CartesianPose(target.position, (.232284215, .695693171, .677790331, .051529821))
    assert math.degrees(orientation_error_rad(actual.orientation, target.orientation)) == pytest.approx(15.954469, abs=1e-5)
    with pytest.raises(ValueError, match='endpoint mismatch'):
        validate_pose_endpoint(actual, target, .001, .01)


def test_quaternion_sign_does_not_introduce_a_rotation():
    assert orientation_error_rad(IDENTITY, (0., 0., 0., -1.)) == 0
    assert interpolate_orientation(IDENTITY, (0., 0., 0., -1.), .5) == IDENTITY


@pytest.mark.parametrize('offset,angle,accepted', [(0., 0., True), (.0009, 4., True),
                                                  (.0011, 0., False), (0., 6., False)])
def test_corridor_checks_intermediate_translation_and_orientation(offset, angle, accepted):
    start, end = CartesianPose((0., 0., 0.), IDENTITY), CartesianPose((0., 0., -.1), IDENTITY)
    half = math.radians(angle)/2
    samples = [start, CartesianPose((offset, 0., -.05), (math.sin(half), 0., 0., math.cos(half))), end]
    if accepted:
        lateral, rotation = validate_corridor_samples(samples, start, end, .001, math.radians(5), .01)
        assert lateral == pytest.approx(offset)
        assert rotation == pytest.approx(math.radians(angle))
    else:
        with pytest.raises(ValueError, match='corridor violated'):
            validate_corridor_samples(samples, start, end, .001, math.radians(5), .01)


def test_corridor_rejects_backtracking_and_missing_samples():
    start, end = CartesianPose((0., 0., 0.), IDENTITY), CartesianPose((0., 0., .1), IDENTITY)
    samples = [start, CartesianPose((0., 0., .07), IDENTITY), CartesianPose((0., 0., .02), IDENTITY), end]
    with pytest.raises(ValueError, match='corridor violated'):
        validate_corridor_samples(samples, start, end, .001, .1, .01)
    with pytest.raises(ValueError, match='at least two'):
        validate_corridor_samples([], start, end, .001, .1, .01)


def test_timing_only_slows_and_bounds_sampled_rates():
    samples = [CartesianPose((0., 0., x), IDENTITY) for x in (0., .05, .1)]
    factor = sampled_time_scale(samples, [0., .1, .2], .02, .1, .08, 2.)
    assert factor >= 50.
    assert sampled_time_scale(samples, [0., 10., 20.], .02, .1, .08, 2.) >= 2.
    with pytest.raises(ValueError, match='increase'):
        sampled_time_scale(samples, [0., 0., 1.], .02, .1, .08, 2.)
    with pytest.raises(ValueError):
        sampled_time_scale(samples, [0., 1., 2.], .02, .1, .08, .5)


def test_joint_corridor_goal_preserves_selected_joints_and_scene_attachments():
    joint_goal = MoveGroup.Goal()
    joint_goal.request.goal_constraints = [Constraints(name='selected')]
    start, target = Pose(), Pose()
    start.position.z, target.position.z = .55, .39
    values = {'lin_acceleration_scaling': .4, 'lin_position_tolerance_m': .001}
    joints = JointState(name=['left_shoulder_yaw_joint'], position=[.2])
    node = SimpleNamespace(
        _execution_goal=lambda arm, state, label: joint_goal if state is joints else None,
        _joint_positions={name: .3 for name in REQUIRED_JOINT_NAMES},
        _pose_position=Coordinator._pose_position,
        get_parameter=lambda name: SimpleNamespace(value=values[name]))
    goal = Coordinator._joint_corridor_goal(node, 'left', joints, start, target, 'approach', .2)
    assert goal.request.pipeline_id == 'ompl'
    assert goal.request.goal_constraints[0].name == 'selected'
    assert goal.request.start_state.is_diff
    assert set(goal.request.start_state.joint_state.name) == set(REQUIRED_JOINT_NAMES)
    assert goal.planning_options.plan_only and not goal.planning_options.replan
    constraint = goal.request.path_constraints.position_constraints[0]
    assert constraint.link_name == 'left_grasp_tcp'
    assert constraint.constraint_region.primitives[0].dimensions == pytest.approx([.162, .001])


def test_retiming_changes_only_time_and_derivatives():
    trajectory = RobotTrajectory()
    for i in range(3):
        point = JointTrajectoryPoint(positions=[float(i)], velocities=[2.], accelerations=[3.])
        point.time_from_start.sec = i
        trajectory.joint_trajectory.points.append(point)
    original = deepcopy(trajectory)
    samples = [CartesianPose((0., 0., i*.05), IDENTITY) for i in range(3)]
    values = {'cartesian_translation_speed_m_s': .1, 'cartesian_rotation_speed_rad_s': .5,
              'cartesian_translation_acceleration_m_s2': .2, 'lin_acceleration_scaling': .4,
              'corridor_time_margin': 2.}
    node = SimpleNamespace(get_parameter=lambda name: SimpleNamespace(value=values[name]),
                           get_logger=lambda: SimpleNamespace(info=lambda _: None))
    Coordinator._slow_corridor_plan(node, trajectory, samples, .2)
    end = trajectory.joint_trajectory.points[-1].time_from_start
    scale = (end.sec + end.nanosec*1e-9)/2
    assert scale >= 5.
    for before, after in zip(original.joint_trajectory.points, trajectory.joint_trajectory.points):
        assert after.positions == before.positions
        assert after.velocities == pytest.approx([2/scale])
        assert after.accelerations == pytest.approx([3/scale**2])


def test_bad_cartesian_endpoint_never_reaches_execute_action():
    target, wrong = Pose(), Pose()
    target.orientation.w = 1.
    wrong.orientation.z, wrong.orientation.w = math.sin(.14), math.cos(.14)
    trajectory = RobotTrajectory()
    trajectory.joint_trajectory.joint_names = ['left_wrist_roll_joint']
    trajectory.joint_trajectory.points = [JointTrajectoryPoint(positions=[0.]), JointTrajectoryPoint(positions=[.28])]
    result = SimpleNamespace(status=4, result=SimpleNamespace(
        error_code=SimpleNamespace(val=1), planned_trajectory=trajectory))
    handle = SimpleNamespace(accepted=True, get_result_async=lambda: result)
    executed = []
    traced = []
    node = SimpleNamespace(
        _wait_arm_stationary=lambda _: None,
        _tcp_pose=lambda arm, override=None: target if override is None else wrong,
        _cartesian_pose=Coordinator._cartesian_pose,
        _arm_joint_state=lambda _: JointState(name=['left_wrist_roll_joint']),
        _linear_goal=lambda *args: MoveGroup.Goal(),
        _move_group=SimpleNamespace(send_goal_async=lambda _: handle),
        _execute_trajectory=SimpleNamespace(send_goal_async=executed.append),
        _record_pipeline_message=lambda label, message: traced.append(label),
        _future=lambda value, *args: value,
        get_parameter=lambda name: SimpleNamespace(value={
            'lin_position_tolerance_m': .001, 'lin_orientation_tolerance_rad': .01}[name]))
    node._validate_cartesian_plan = lambda *args, **kwargs: Coordinator._validate_cartesian_plan(node, *args, **kwargs)
    with pytest.raises(RuntimeError, match='rejected before execution.*endpoint mismatch'):
        Coordinator._execute_linear(node, 'left', target, 'approach', velocity_scaling=.2)
    assert executed == []
    assert traced == ['motion_request', 'motion_result']


def test_seeded_path_still_cannot_bypass_cartesian_orientation_gate():
    start, target = Pose(), Pose()
    start.position.z, target.position.z = .5, .4
    start.orientation.w = target.orientation.w = 1.
    trajectory = RobotTrajectory()
    trajectory.joint_trajectory.joint_names = ['left_wrist_roll_joint']
    trajectory.joint_trajectory.points = [JointTrajectoryPoint(positions=[q]) for q in (0., 1., 0.)]
    poses = [CartesianPose((0., 0., .5), IDENTITY),
             CartesianPose((0., 0., .45), (0., 0., 1., 0.)),
             CartesianPose((0., 0., .4), IDENTITY)]
    executed, traces = [], []
    node = SimpleNamespace(
        _wait_arm_stationary=lambda _: None,
        _tcp_pose=lambda _: start,
        _cartesian_pose=Coordinator._cartesian_pose,
        _arm_joint_state=lambda _: JointState(name=['left_wrist_roll_joint']),
        _joint_corridor_goal=lambda *args: MoveGroup.Goal(),
        _seeded_cartesian=SimpleNamespace(plan=lambda *args: (trajectory, poses)),
        _execute_trajectory=SimpleNamespace(send_goal_async=executed.append),
        _record_pipeline_message=lambda label, _: traces.append(label),
        get_parameter=lambda name: SimpleNamespace(value={
            'lin_position_tolerance_m': .001, 'lin_orientation_tolerance_rad': .01,
            'corridor_orientation_tolerance_deg': 5.}[name]))
    node._validate_cartesian_plan = lambda *args, **kwargs: Coordinator._validate_cartesian_plan(node, *args, **kwargs)
    with pytest.raises(RuntimeError, match='corridor violated'):
        Coordinator._execute_linear(node, 'left', target, 'seeded', velocity_scaling=.2,
                                     joint_target=JointState(name=['left_wrist_roll_joint'], position=[0.]))
    assert executed == []
    assert traces == ['motion_request', 'motion_result']
