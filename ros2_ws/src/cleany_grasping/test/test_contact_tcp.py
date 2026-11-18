"""Predictor-origin conversion uses measured pose depth without shifting raw data."""
import numpy as np
import pytest

from cleany_grasping.core.models import PointCloud, RawGrasp
from cleany_grasping.core.selector import rank_grasps


@pytest.mark.parametrize('depth', [0., .025, .04])
@pytest.mark.parametrize('angle', [0., .7, -1.57])
def test_predictor_origin_is_converted_to_contact_tcp(depth, angle):
    rotation = np.array([[np.cos(angle), -np.sin(angle), 0.],
                         [np.sin(angle), np.cos(angle), 0.], [0., 0., 1.]])
    contact = np.array([.1, -.2, .5])
    origin = contact-depth*rotation[:, 0]
    raw = RawGrasp(rotation=rotation, translation=origin.copy(), width_m=.02, depth_m=depth, score=.8)
    cloud = PointCloud(points=np.array([contact-.002, contact+.002]), colors=np.zeros((2,3)))
    result, = rank_grasps((raw,), cloud)
    np.testing.assert_allclose(result.translation, contact, atol=1e-12)
    np.testing.assert_allclose(result.approach_direction, rotation[:,0], atol=1e-12)
    np.testing.assert_array_equal(raw.translation, origin)
