from dataclasses import replace
import math
import numpy as np
import pytest
from cleany_tof.core import (Settings, project, coverage, limit_velocity, estimate_plane,
                             ApproachController, Plane)


def scene(distance=1., yaw=0., settings=None):
    s = settings or Settings(calibrated=True)
    rays, _ = project([1.]*64, [.001]*64, [5]*64, [1]*64, [True]*64, s)
    origin = np.array((s.sensor_x, s.sensor_y, s.sensor_z))
    rays -= origin
    slope = -math.tan(yaw)
    ranges = (distance+slope*origin[1]-origin[0])/(rays[:, 0]-slope*rays[:, 1])
    return project(ranges, [.001]*64, [5]*64, [1]*64, [True]*64, s)


@pytest.mark.parametrize('yaw', [-.2, 0., .2])
def test_plane_geometry_and_sign_with_offset(yaw):
    s = Settings(calibrated=True, sensor_x=.30, sensor_y=.02, sensor_yaw=.03)
    points, good = scene(1., yaw, s)
    plane = estimate_plane(points, good, s)
    assert plane.distance == pytest.approx(1., abs=1e-6)
    assert plane.yaw == pytest.approx(yaw, abs=1e-6)


def test_quality_never_becomes_free_space():
    d = [1.]*64
    d[0] = math.nan
    _, good = project(d, [.001]*64, [5]*64, [1]*64, [True]*64, Settings())
    assert not good[0] and not coverage(good, Settings())
    with pytest.raises(ValueError):
        project([1.], [], [], [], [], Settings())


def test_guard_braking_direction_measured_speed_and_unknown():
    s = Settings(calibrated=True)
    good = np.ones(64, dtype=bool)
    points = np.tile([.80, 0., 0.], (64, 1))
    output, reason = limit_velocity((.25, 0., 0.), (0., 0., 0.), points, good, .1, s)
    assert 0 < output[0] < .25 and reason == 'LIMITED'
    assert limit_velocity((.25, 0., 0.), (.5, 0., 0.), points, good, .1, s)[0] == (0., 0., 0.)
    assert limit_velocity((-.1, 0., 0.), (0., 0., 0.), points, good, .1, s)[0][0] == -.1
    assert limit_velocity((0., 0., .3), (0., 0., 0.), points-.05, good, .1, s)[0][2] < .3
    for age in (-.01, .21, math.nan):
        assert limit_velocity((.1, 0., 0.), (0., 0., 0.), points, good, age, s)[0] == (0., 0., 0.)
    assert limit_velocity((.1, 0., 0.), (0., 0., 0.), points, good, .1, replace(s, calibrated=False))[0] == (0., 0., 0.)
    points[0] = (.55, 0., 0.)
    assert limit_velocity((.1, 0., 0.), (0., 0., 0.), points, good, .1, s)[0] == (0., 0., 0.)


def test_no_acceleration_or_curvature_change():
    s = Settings(calibrated=True)
    points, good = scene(.85)
    cmd = (.2, .1, .15)
    result, _ = limit_velocity(cmd, (0., 0., 0.), points, good, .1, s)
    assert all(0 <= a/b <= 1 for a, b in zip(result, cmd))
    assert result[0]/cmd[0] == pytest.approx(result[2]/cmd[2])


def test_foreground_and_sparse_columns_reject_plane():
    s = Settings(calibrated=True)
    points, good = scene(1.)
    points[2*8+4, 0] -= .1
    with pytest.raises(ValueError, match='ambiguous'):
        estimate_plane(points, good, s)
    good[:] = False
    good[:4] = True
    with pytest.raises(ValueError):
        estimate_plane(points, good, s)


def test_approach_alignment_timeout_overshoot_and_stationary_success():
    s = Settings(calibrated=True)
    c = ApproachController(.75, 0., s)
    cmd, state = c.step(Plane(1., .15, 0.), (0., 0., 0.), 0.)
    assert cmd[0] == 0 and cmd[2] > 0 and state == 'ALIGNING'
    cmd, _ = c.step(Plane(.9, 0., 0.), (0., 0., 0.), .2)
    assert 0 < cmd[0] <= .04
    assert c.step(Plane(.75, 0., 0.), (.02, 0., 0.), .3)[1] == 'SETTLING'
    assert c.step(Plane(.75, 0., 0.), (0., 0., 0.), .4)[1] == 'SETTLING'
    assert c.step(Plane(.75, 0., 0.), (0., 0., 0.), 1.1)[1] == 'SUCCEEDED'
    assert ApproachController(.75, 0., s).step(Plane(.70, 0., 0.), (0., 0., 0.), .1)[1] == 'OVERSHOOT'
    assert ApproachController(.75, 0., s).step(Plane(1., 0., 0.), (0., 0., 0.), 31)[1] == 'TIMEOUT'
    c = ApproachController(.75, 0., s)
    c.step(Plane(1., 0., 0.), (0., 0., 0.), 0.)
    assert c.step(Plane(1., 0., 0.), (0., 0., 0.), 6.)[1] == 'NO_PROGRESS'
    with pytest.raises(ValueError):
        ApproachController(.6, 0., s)


@pytest.mark.parametrize('kwargs', [dict(deceleration=0), dict(margin=-1.), dict(sensor_yaw=math.nan),
    dict(required_zones=()), dict(required_zones=(64,)), dict(min_valid=0), dict(source_timeout_s=math.inf)])
def test_invalid_safety_configuration_rejected(kwargs):
    with pytest.raises(ValueError):
        Settings(**kwargs)
