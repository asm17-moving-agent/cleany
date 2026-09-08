from dataclasses import replace

import numpy as np
import pytest

from cleany_skill_executor.core.can_rgbd import CameraProjection, rotation_matrix_from_quaternion
from cleany_skill_executor.core.reobservation import reobservation_centers, sphere_in_view


CAMERA = CameraProjection(500., 500., 319.5, 239.5, (.2, 0., .8),
                          (1., 0., 0., 0., -1., 0., 0., 0., -1.))


def centers(**overrides):
    args = dict(camera=CAMERA, width=640, height=480,
                current_center=np.array((.4, .15, .5)), radius_m=.035,
                minimum_z_m=.45, margin_px=24., maximum_translation_m=.4)
    args.update(overrides)
    return reobservation_centers(**args)


def test_candidates_preserve_lift_height_and_fit_whole_sphere():
    result = centers()
    assert len(result) == 3
    np.testing.assert_allclose(result[0], (.2, 0., .5), atol=1e-12)
    for center in result:
        assert center[2] == pytest.approx(.5)
        assert sphere_in_view(center, .035, CAMERA, 640, 480, 24.)
        # Random sphere surface points independently project into the inset.
        rays = np.random.default_rng(7).normal(size=(5000, 3))
        points = center + .035 * rays / np.linalg.norm(rays, axis=1)[:, None]
        optical = (points - CAMERA.translation_base) @ np.array(CAMERA.rotation_base_from_optical).reshape(3, 3)
        pixels = optical[:, :2]/optical[:, 2:] * (CAMERA.fx, CAMERA.fy) + (CAMERA.cx, CAMERA.cy)
        assert np.all(pixels >= 24.)
        assert np.all(pixels <= (615., 455.))


@pytest.mark.parametrize('overrides', [
    dict(radius_m=.3), dict(minimum_z_m=.51), dict(maximum_translation_m=.001),
    dict(camera=replace(CAMERA, rotation_base_from_optical=(1., 0., 0., 0., 1., 0., 0., 0., 1.))),
    dict(camera=replace(CAMERA, rotation_base_from_optical=(0., 0., 1., 1., 0., 0., 0., 1., 0.))),
])
def test_unobservable_or_excessive_motion_has_no_candidate(overrides):
    assert centers(**overrides) == ()


@pytest.mark.parametrize('overrides', [
    dict(radius_m=0.), dict(radius_m=float('nan')), dict(margin_px=-1.),
    dict(margin_px=240.), dict(maximum_translation_m=0.), dict(width=0),
    dict(minimum_z_m=float('inf')), dict(current_center=np.array((1., 2.))),
    dict(current_center=np.array((1., 2., float('nan')))),
])
def test_invalid_geometry_fails_closed(overrides):
    with pytest.raises(ValueError, match='invalid reobservation'):
        centers(**overrides)


def test_center_visible_is_insufficient_if_sphere_crosses_edge_or_camera_plane():
    assert sphere_in_view(np.array((.2, 0., .5)), .035, CAMERA, 640, 480, 24.)
    assert not sphere_in_view(np.array((.37, 0., .5)), .035, CAMERA, 640, 480, 24.)
    assert not sphere_in_view(np.array((.2, 0., .79)), .035, CAMERA, 640, 480, 24.)


def test_attempt36_calibration_needs_more_distance_but_keeps_required_lift():
    camera = CameraProjection(625.2213755265125, 625.2213755265125, 320., 240.,
        (.140751687191, -.002000000139, .766322294556),
        tuple(rotation_matrix_from_quaternion(-.678504049029, .678504051465,
                                              -.199078512, .199078511286).flat))
    args = dict(camera=camera, current_center=np.array((.3636868601, .1204484426, .5537833539)),
                radius_m=.0953, minimum_z_m=.4476632402, maximum_translation_m=.20)
    assert centers(**args) == ()
    candidates = centers(**args, maximum_lowering_m=.10)
    assert candidates
    for target in candidates:
        assert target[2] >= args['minimum_z_m']+.02-1e-12
        assert target[2] <= args['current_center'][2]
        assert np.linalg.norm(target-args['current_center']) <= .20
        assert sphere_in_view(target, .0953, camera, 640, 480, 24.)


@pytest.mark.parametrize('kwargs', [dict(maximum_lowering_m=-.1),
    dict(maximum_lowering_m=float('nan')), dict(height_clearance_m=0.)])
def test_lowering_limits_fail_closed(kwargs):
    with pytest.raises(ValueError):
        centers(**kwargs)
