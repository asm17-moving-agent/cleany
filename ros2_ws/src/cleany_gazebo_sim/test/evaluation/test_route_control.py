from math import hypot, pi
from pathlib import Path

import yaml

from cleany_gazebo_sim.route_control import (
    Pose2D,
    RouteLimits,
    RouteTracker,
    normalize_angle,
    waypoints_from_flat,
)


PACKAGE_ROOT = Path(__file__).resolve().parents[2]
ROUTE_CONFIG = (
    PACKAGE_ROOT / 'config' / 'study_cafe' / 'study_cafe_route.yaml'
)


def _limits() -> RouteLimits:
    return RouteLimits(0.15, 0.25, 1.2, 0.09, 0.45)


def test_study_cafe_route_is_closed_and_covers_evaluation_zones() -> None:
    config = yaml.safe_load(ROUTE_CONFIG.read_text(encoding='utf-8'))
    params = config['ground_truth_route_follower']['ros__parameters']
    waypoints = waypoints_from_flat(params['waypoints_xy'])

    assert params['max_linear_speed'] == 0.25
    assert params['max_angular_speed'] == 0.5
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


def test_normalize_angle_wraps_to_shortest_rotation() -> None:
    assert abs(normalize_angle(3.0 * pi) - pi) < 1e-9
    assert abs(normalize_angle(-3.0 * pi) + pi) < 1e-9
