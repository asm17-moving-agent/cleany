from dataclasses import replace

import pytest

from cleany_gazebo_sim.route_control import Pose2D, RouteLimits, RouteTracker, Waypoint


def _limits() -> RouteLimits:
    return RouteLimits(
        max_linear_speed=.25,
        max_angular_speed=.5,
        max_linear_acceleration=.2,
        heading_gain=1.2,
        position_tolerance=.09,
        heading_tolerance=.08,
        turn_in_place_threshold=.45,
    )


def test_facility_gain_slows_approach_without_changing_legacy_default():
    points = [Waypoint(0., 0.), Waypoint(.2, 0.)]
    legacy = RouteTracker(points, _limits())
    slow = RouteTracker(points, replace(_limits(), position_gain=.4))
    assert legacy.command(Pose2D(0., 0., 0.)).linear_x == pytest.approx(.2)
    assert slow.command(Pose2D(0., 0., 0.)).linear_x == pytest.approx(.08)


def test_invalid_approach_gain_rejected():
    with pytest.raises(ValueError):
        replace(_limits(), position_gain=0.)
