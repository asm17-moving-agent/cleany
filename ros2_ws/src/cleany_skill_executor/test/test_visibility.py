import math

import pytest

from cleany_skill_executor.core.grasp_selection import quaternion_axis
from cleany_skill_executor.core.visibility import enclosing_visibility_cone


def test_near_corners_and_polygon_sides_are_enclosed():
    cone = enclosing_visibility_cone((0., 0., 1.), (0., 0., 0.),
                                     (.2, .2, .2), (0., 0., 0., 1.), padding_m=0.)
    assert cone.radius_m == pytest.approx(math.sqrt(.02)/.9/math.cos(math.pi/16))
    assert quaternion_axis(cone.orientation, (0., 0., 1.)) == pytest.approx((0., 0., 1.))


def test_rigid_rotation_and_translation_preserve_envelope():
    original = enclosing_visibility_cone((0., 0., 1.), (0., 0., 0.),
                                         (.2, .1, .15), (0., 0., 0., 1.))
    rotated = enclosing_visibility_cone((2., 2., 3.), (1., 2., 3.),
                                        (.2, .1, .15), (0., 2**-.5, 0., 2**-.5))
    assert rotated.radius_m == pytest.approx(original.radius_m)
    assert quaternion_axis(rotated.orientation, (0., 0., 1.)) == pytest.approx((1., 0., 0.))


def test_camera_below_target_has_normalized_disc_orientation():
    cone = enclosing_visibility_cone((0., 0., -1.), (0., 0., 0.),
                                     (.2, .1, .15), (0., 0., 0., 1.))
    assert quaternion_axis(cone.orientation, (0., 0., 1.)) == pytest.approx((0., 0., -1.))


@pytest.mark.parametrize('camera,size,q,padding,sides', [
    ((0., 0., 0.), (.1, .1, .1), (0., 0., 0., 1.), 0., 16),
    ((0., 0., .01), (.1, .1, .1), (0., 0., 0., 1.), 0., 16),
    ((math.nan, 0., 1.), (.1, .1, .1), (0., 0., 0., 1.), 0., 16),
    ((0., 0., 1.), (-.1, .1, .1), (0., 0., 0., 1.), 0., 16),
    ((0., 0., 1.), (.1, .1, .1), (0., 0., 0., 2.), 0., 16),
    ((0., 0., 1.), (.1, .1, .1), (0., 0., 0., 1.), -.1, 16),
    ((0., 0., 1.), (.1, .1, .1), (0., 0., 0., 1.), 0., 2),
])
def test_invalid_or_behind_camera_geometry_is_rejected(camera, size, q, padding, sides):
    with pytest.raises(ValueError):
        enclosing_visibility_cone(camera, (0., 0., 0.), size, q, padding_m=padding, sides=sides)
