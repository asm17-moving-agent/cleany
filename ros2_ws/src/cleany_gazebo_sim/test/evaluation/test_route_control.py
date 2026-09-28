from math import hypot, pi
from pathlib import Path

import yaml

from cleany_gazebo_sim.route_control import (
    Pose2D,
    RouteLimits,
    RouteTracker,
    limit_linear_acceleration,
    normalize_angle,
    waypoints_from_flat,
)


PACKAGE_ROOT = Path(__file__).resolve().parents[2]
ROUTE_CONFIG = (
    PACKAGE_ROOT / 'config' / 'study_cafe' / 'study_cafe_route.yaml'
)


def _limits() -> RouteLimits:
    return RouteLimits(0.15, 0.25, 0.2, 1.2, 0.09, 0.08, 0.15)


def test_study_cafe_route_is_closed_and_covers_evaluation_zones() -> None:
    config = yaml.safe_load(ROUTE_CONFIG.read_text(encoding='utf-8'))
    params = config['ground_truth_route_follower']['ros__parameters']
    waypoints = waypoints_from_flat(params['waypoints_xy'])

    assert params['max_linear_speed'] == 0.20
    assert params['max_angular_speed'] == 0.30
    assert params['max_linear_acceleration'] == 0.20
    assert params['heading_tolerance'] == 0.08
    assert params['turn_in_place_threshold'] == 0.15
    expected = (
        (-1.865, -4.705),
        (-5.65, -4.705),
        (5.65, -4.705),
        (1.865, -4.705),
        (1.865, -1.585),
        (5.65, -1.585),
        (-5.65, -1.585),
        (-1.865, -1.585),
        (-1.865, 1.585),
        (-5.65, 1.585),
        (5.65, 1.585),
        (1.865, 1.585),
        (1.865, 4.705),
        (5.65, 4.705),
        (-5.65, 4.705),
        (-1.865, 4.705),
        (-1.865, -4.705),
    )

    assert tuple((point.x, point.y) for point in waypoints) == expected
    assert waypoints[0] == waypoints[-1]
    assert {point.x for point in waypoints} == {-5.65, -1.865, 1.865, 5.65}
    assert {point.y for point in waypoints} == {
        -4.705, -1.585, 1.585, 4.705
    }
    length = sum(
        hypot(second.x - first.x, second.y - first.y)
        for first, second in zip(waypoints, waypoints[1:])
    )
    assert abs(length - 94.30) < 1e-9


def test_route_tracker_turns_before_driving_when_target_is_sideways() -> None:
    tracker = RouteTracker(
        waypoints_from_flat((0.0, 0.0, 0.0, 1.0)), _limits()
    )

    command = tracker.command(Pose2D(0.0, 0.0, 0.0))

    assert command.waypoint_index == 1
    assert command.linear_x == 0.0
    assert command.angular_z == 0.25
    assert not command.completed


def test_route_tracker_drives_forward_and_stops_at_route_end() -> None:
    tracker = RouteTracker(
        waypoints_from_flat((0.0, 0.0, 1.0, 0.0)), _limits()
    )

    driving = tracker.command(Pose2D(0.0, 0.0, 0.0))
    completed = tracker.command(Pose2D(1.0, 0.0, 0.0))

    assert driving.linear_x == 0.15
    assert driving.angular_z == 0.0
    assert not driving.completed
    assert completed.completed
    assert completed.linear_x == 0.0
    assert completed.angular_z == 0.0


def test_route_tracker_uses_hysteresis_between_turning_and_driving() -> None:
    tracker = RouteTracker(
        waypoints_from_flat((0.0, 0.0, 1.0, 0.0)), _limits()
    )

    still_turning = tracker.command(Pose2D(0.0, 0.0, -0.10))
    aligned = tracker.command(Pose2D(0.0, 0.0, -0.07))
    small_drift = tracker.command(Pose2D(0.0, 0.0, -0.10))
    reenter_turn = tracker.command(Pose2D(0.0, 0.0, -0.16))

    assert still_turning.linear_x == 0.0
    assert aligned.linear_x > 0.0
    assert small_drift.linear_x > 0.0
    assert reenter_turn.linear_x == 0.0


def test_linear_acceleration_is_ramped_but_stops_immediately() -> None:
    first = limit_linear_acceleration(0.0, 0.20, 0.20, 0.05)
    second = limit_linear_acceleration(0.01, 0.20, 0.20, 0.05)

    assert abs(first - 0.01) < 1e-9
    assert abs(second - 0.02) < 1e-9
    assert limit_linear_acceleration(0.12, 0.0, 0.20, 0.05) == 0.0


def test_normalize_angle_wraps_to_shortest_rotation() -> None:
    assert abs(normalize_angle(3.0 * pi) - pi) < 1e-9
    assert abs(normalize_angle(-3.0 * pi) + pi) < 1e-9
