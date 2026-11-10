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
