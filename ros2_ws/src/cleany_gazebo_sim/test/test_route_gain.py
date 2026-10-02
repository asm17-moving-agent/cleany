import pytest

from cleany_gazebo_sim.route_control import Pose2D, RouteLimits, RouteTracker, Waypoint


def test_facility_gain_slows_approach_without_changing_legacy_default():
    points = [Waypoint(0., 0.), Waypoint(.2, 0.)]
    legacy = RouteTracker(points, RouteLimits(.25, .5, 1.2, .09, .45))
    slow = RouteTracker(points, RouteLimits(.25, .5, 1.2, .09, .45, position_gain=.4))
    assert legacy.command(Pose2D(0., 0., 0.)).linear_x == pytest.approx(.2)
    assert slow.command(Pose2D(0., 0., 0.)).linear_x == pytest.approx(.08)


def test_invalid_approach_gain_rejected():
    with pytest.raises(ValueError):
        RouteLimits(.25, .5, 1.2, .09, .45, position_gain=0.)
