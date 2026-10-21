"""Bounded approach geometry; no ROS nodes, camera reads or motor commands."""
from dataclasses import replace
import math

import pytest

from cleany_skill_executor.core.approach_path import approach_detours, validate_detour_samples
from cleany_skill_executor.core.cartesian import CartesianPose
from cleany_skill_executor.core.collection import configuration_issues


START = CartesianPose((0., .08, 0.), (0., 0., 0., 1.))
TARGET = CartesianPose((0., 0., 0.), (0., 0., 0., 1.))


def routes(start=START, target=TARGET, **options):
    return approach_detours(start, target, **{
        'offset_m': .02, 'final_straight_m': .04, 'maximum_extra_length_m': .04,
        'attempts': 4, **options})


def samples(route):
    points = (START, *route.waypoints)
    return [CartesianPose(tuple(a+(b-a)*fraction/10 for a, b in zip(
        left.position, right.position, strict=True)), TARGET.orientation)
        for left, right in zip(points, points[1:]) for fraction in range(11)]


def validate(values, route):
    validate_detour_samples(values, START, route, radius_m=.005, maximum_extra_length_m=.04)


def test_four_detours_keep_goal_and_straight_final_approach():
    choices = routes()
    assert [r.name for r in choices] == ['closing_plus', 'closing_minus', 'tool_z_plus', 'tool_z_minus']
    for route in choices:
        via, entry, goal = route.waypoints
        assert goal is TARGET
        assert entry.position == pytest.approx((0., .04, 0.))
        assert math.hypot(via.position[0], via.position[2]) == pytest.approx(.02)
        validate(samples(route), route)


def test_detours_follow_the_rotated_tool_axes():
    angle = math.pi/4
    target = CartesianPose(TARGET.position, (0., 0., math.sin(angle), math.cos(angle)))
    start = CartesianPose((-.08, 0., 0.), target.orientation)
    route = routes(start, target)[0]
    assert route.waypoints[0].position == pytest.approx((-.06, .02, 0.))
    assert route.waypoints[-2].position == pytest.approx((-.04, 0., 0.))
    assert route.waypoints[-1] is target


def test_final_entry_and_path_budget_prevent_unbounded_detours():
    assert routes(replace(START, position=(0., .03, 0.))) == ()
    assert routes(maximum_extra_length_m=.001) == ()
    assert len(routes(attempts=2)) == 2


@pytest.mark.parametrize('options', [
    {'offset_m': 0.}, {'offset_m': math.nan}, {'final_straight_m': -1.},
    {'maximum_extra_length_m': math.inf}, {'attempts': 0}, {'attempts': True}, {'attempts': 5},
])
def test_invalid_detour_limits_are_rejected(options):
    with pytest.raises(ValueError):
        routes(**options)


@pytest.mark.parametrize('invalid', ['start', 'goal', 'sideways_final_entry', 'reverse'])
def test_sampled_fk_must_follow_the_detour_and_reach_the_original_goal(invalid):
    route = routes()[0]
    values = samples(route)
    if invalid == 'start':
        values[0] = replace(values[0], position=(.02, .08, 0.))
    elif invalid == 'goal':
        values[-1] = replace(values[-1], position=(.02, 0., 0.))
    elif invalid == 'sideways_final_entry':
        values[-6] = replace(values[-6], position=(.02, .02, 0.))
    else:
        values[15] = values[2]
    with pytest.raises(ValueError):
        validate(values, route)


@pytest.mark.parametrize('key,value', [
    ('approach_collision_replanning_enabled', 'true'),
    ('approach_collision_scene_refresh_attempts', -1),
    ('approach_collision_scene_refresh_attempts', 3),
    ('approach_collision_detour_attempts', 5),
    ('approach_collision_detour_offset_m', True),
    ('approach_collision_final_straight_m', math.nan),
    ('approach_collision_max_extra_length_m', 0.),
])
def test_configuration_rejects_invalid_replanning_limits(key, value):
    config = {'arms': {'right': {}}, 'approach_collision_replanning_enabled': True,
              'approach_collision_scene_refresh_attempts': 1,
              'approach_collision_detour_attempts': 4,
              'approach_collision_detour_offset_m': .02,
              'approach_collision_final_straight_m': .04,
              'approach_collision_max_extra_length_m': .04}
    assert not any('approach_collision' in issue for issue in configuration_issues(config))
    config[key] = value
    assert any(key in issue for issue in configuration_issues(config))
