from cleany_skill_executor.core.cartesian import validate_height_correctable_endpoint
import math

import pytest

from cleany_skill_executor.core.cartesian import CartesianPose, execution_wall_timeout, interpolate_orientation, line_corridor, orientation_error_rad, sampled_time_scale, validate_corridor_samples, validate_pose_endpoint, validate_position_endpoint
from cleany_skill_executor.core.alignment import (
    quaternion_axis,
)
from cleany_skill_executor.core.wrist_alignment import aligned_wrist_rolls

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


def test_position_endpoint_ignores_orientation_but_still_checks_translation():
    target = CartesianPose((.3, .1, .4), IDENTITY)
    rotated = CartesianPose(target.position, (1., 0., 0., 0.))

    validate_position_endpoint(rotated, target, .001)
    with pytest.raises(ValueError, match='endpoint mismatch'):
        validate_position_endpoint(
            CartesianPose((.302, .1, .4), rotated.orientation), target, .001)


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


def test_position_only_corridor_ignores_orientation_but_keeps_line_check():
    start = CartesianPose((0., 0., 0.), IDENTITY)
    end = CartesianPose((0., 0., -.1), IDENTITY)
    flipped = (1., 0., 0., 0.)

    lateral, rotation = validate_corridor_samples(
        [start, CartesianPose((0., 0., -.05), flipped),
         CartesianPose(end.position, flipped)],
        start, end, .001, None, None)

    assert lateral == 0.
    assert rotation == 0.
    with pytest.raises(ValueError, match='corridor violated'):
        validate_corridor_samples(
            [start, CartesianPose((.002, 0., -.05), flipped), end],
            start, end, .001, None, None)


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


@pytest.mark.parametrize('xyz,angle,accepted', [
    ((.316879,.056145,.508824), 5.589, True),
    ((.351,.058715,.508824), 0., False),
    ((.319833,.058715,.508824), 21., False),
    ((.319833,.058715,.591823), 0., False),
    ((.319833,.058715,.571823), 0., False),
    ((.319833,.058715,.489822), 0., True),
])
def test_height_shortfall_keeps_xy_rotation_and_overshoot_guards(xyz, angle, accepted):
    half = math.radians(angle)/2
    actual = CartesianPose(xyz, (0.,0.,math.sin(half),math.cos(half)))
    target = CartesianPose((.319833,.058715,.540822), (0.,0.,0.,1.))
    if accepted:
        validate_height_correctable_endpoint(actual, target, .030, math.radians(20.))
    else:
        with pytest.raises(ValueError):
            validate_height_correctable_endpoint(actual, target, .030, math.radians(20.))


from cleany_skill_executor.core.cartesian import bounded_cartesian_step, coordinate_median, estimate_measured_path_pose, grasp_capture_metrics, observation_corrected_pose

from cleany_skill_executor.core.alignment import quaternion_from_rotation, rotation_from_quaternion

def test_recorded_path_starts_at_measured_tcp_after_large_height_correction():
    measured = CartesianPose((.429300486860051, .1379370516226814, .5821843963492025),
                             (.4194112687, .3485305305, .5357298996, .6446814187))
    command = CartesianPose((.4312287445669679, .13757803825922232, .6130968495419921),
                            (.3845307879, .3186808912, .5534167934, .6665646372))
    assert math.dist(measured.position, command.position) > .03
    start = estimate_measured_path_pose(measured, command, command)
    assert start.position == pytest.approx(measured.position)
    assert orientation_error_rad(start.orientation, measured.orientation) < 1e-7
    next_command = CartesianPose((command.position[0]-.01, command.position[1]+.02,
                                 command.position[2]-.05), command.orientation)
    predicted = estimate_measured_path_pose(measured, command, next_command)
    assert predicted.position == pytest.approx((measured.position[0]-.01,
                                                measured.position[1]+.02, measured.position[2]-.05))
    assert orientation_error_rad(predicted.orientation, measured.orientation) < 1e-7


def test_recorded_rotation_applies_world_delta_to_measured_orientation():
    measured = CartesianPose((0., 0., .5), (math.sin(.1), 0., 0., math.cos(.1)))
    command = CartesianPose((0., 0., .6), (0., math.sin(.3), 0., math.cos(.3)))
    delta = rotation_from_quaternion((0., 0., math.sin(.15), math.cos(.15)))
    endpoint = CartesianPose((0., 0., .55), quaternion_from_rotation(
        delta @ rotation_from_quaternion(command.orientation)))
    predicted = estimate_measured_path_pose(measured, command, endpoint)
    expected = quaternion_from_rotation(delta @ rotation_from_quaternion(measured.orientation))
    assert predicted.position == pytest.approx((0., 0., .45))
    assert orientation_error_rad(predicted.orientation, expected) < 1e-7


def test_motor_support_conversion_round_trips_the_unchanged_physical_goal():
    measured = CartesianPose((.4, .1, .5), (math.sin(.1), 0., 0., math.cos(.1)))
    command = CartesianPose((.41, .09, .54), (0., math.sin(.2), 0., math.cos(.2)))
    goal = CartesianPose((.42, .12, .6), (0., 0., math.sin(.25), math.cos(.25)))
    motor_goal = estimate_measured_path_pose(command, measured, goal)
    assert motor_goal.position == pytest.approx((.43, .11, .64))
    physical = estimate_measured_path_pose(measured, command, motor_goal)
    assert physical.position == pytest.approx(goal.position)
    assert orientation_error_rad(physical.orientation, goal.orientation) < 1e-7
    assert goal.position == (.42, .12, .6)


def test_visual_correction_translates_the_grasp_and_bounds_short_steps():
    planned = CartesianPose((.3, .1, .4), IDENTITY)
    corrected = observation_corrected_pose(
        planned, (.2, .1, .3), (.212, .095, .3), .03)
    assert corrected.position == pytest.approx((.312, .095, .4))
    first = bounded_cartesian_step(
        CartesianPose((.3, .1, .4), IDENTITY), corrected, .005)
    assert math.dist(first.position, planned.position) == pytest.approx(.005)
    assert first.orientation == IDENTITY
    with pytest.raises(ValueError, match='exceeds'):
        observation_corrected_pose(
            planned, (.2, .1, .3), (.25, .1, .3), .03)


def test_visual_center_median_discards_one_frame_jump():
    assert coordinate_median([
        (.3, .1, .4),
        (.34, .1, .4),
        (.301, .099, .4),
    ]) == pytest.approx((.301, .1, .4))
    with pytest.raises(ValueError, match='odd set'):
        coordinate_median([(.3, .1, .4), (.31, .1, .4)])


def test_grasp_capture_corridor_separates_approach_from_lateral_error():
    current = CartesianPose((0., 0., 0.), IDENTITY)
    target = CartesianPose((.003, -.02, 0.), IDENTITY)
    metrics = grasp_capture_metrics(current, target)

    assert metrics.approach_remaining_m == pytest.approx(.02)
    assert metrics.lateral_error_m == pytest.approx(.003)
    assert metrics.inside(.045, .005, .001)
    assert not metrics.inside(.015, .005, .001)
    assert not grasp_capture_metrics(
        current, CartesianPose((.02, -.02, 0.), IDENTITY),
    ).inside(.045, .005, .001)
