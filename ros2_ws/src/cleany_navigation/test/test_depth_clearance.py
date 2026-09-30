import numpy as np
import pytest

from cleany_navigation.depth_clearance import prepare_depth_points, project_depth_image


def test_optical_depth_projects_to_x_forward_camera_axes():
    depth = np.array([[2., 2., 2.], [2., 2., 2.]])
    actual = project_depth_image(depth, (2, 4, 1, 1), pixel_stride=1)
    np.testing.assert_allclose(actual[[0, 1, 2, 4]], [[2, 1, 0.5], [2, 0, 0.5], [2, -1, 0.5], [2, 0, 0]])


def test_infinite_center_pixel_clears_but_nan_and_near_clipped_do_not():
    depth = np.array([[np.nan, np.inf, -np.inf]])
    actual = project_depth_image(depth, (2, 2, 1, 0), pixel_stride=1, positive_infinity_is_free=True)
    np.testing.assert_allclose(actual, [[5, 0, 0]])


def test_invalid_depth_never_becomes_free_without_explicit_simulation_option():
    points = np.array([[[1, 0.1, 0.2], [np.inf]*3, [-np.inf]*3, [np.nan]*3, [0]*3]])
    actual = prepare_depth_points(points, np.array([0]), np.arange(5), (10, 10, 2, 0))
    np.testing.assert_allclose(actual, [[1, 0.1, 0.2]])


def test_only_positive_infinity_gets_camera_intrinsic_ray_endpoints():
    points = np.array([[[np.inf]*3, [2, 0.5, -0.2], [-np.inf]*3, [np.nan]*3]])
    actual = prepare_depth_points(points, np.array([1]), np.arange(4), (10, 20, 2, 3),
                                  positive_infinity_is_free=True, clearing_distance=5)
    np.testing.assert_allclose(actual, [[5, 1, 0.5], [2, 0.5, -0.2]])
    assert np.linalg.norm(actual[0]) > 4.0  # Cannot become a marked obstacle within the configured 4 m range.


@pytest.mark.parametrize('intrinsics', [(0, 1, 0, 0), (1, -1, 0, 0), (np.nan, 1, 0, 0)])
def test_invalid_intrinsics_rejected(intrinsics):
    with pytest.raises(ValueError):
        prepare_depth_points(np.zeros((1, 1, 3)), np.array([0]), np.array([0]), intrinsics)


@pytest.mark.parametrize('encoding,dtype,scale', [('32FC1', '<f4', 1.), ('16UC1', '<u2', 1000.),
                                                ('32FC1', '>f4', 1.), ('16UC1', '>u2', 1000.)])
def test_depth_decoding_preserves_units_endianness_and_padding(encoding, dtype, scale):
    from cleany_navigation.depth_clearance import decode_depth_image
    pixels = np.array([[1., 2.], [0., 3.]])
    raw = (pixels*scale).astype(dtype)
    stride = 2*raw.dtype.itemsize+4
    data = b''.join(row.tobytes()+b'\x00'*4 for row in raw)
    decoded = decode_depth_image(data, 2, 2, stride, encoding, dtype.startswith('>'))
    np.testing.assert_allclose(decoded, pixels)
    with pytest.raises(ValueError, match='Truncated'):
        decode_depth_image(data[:-1], 2, 2, stride, encoding)
    with pytest.raises(ValueError, match='Expected'):
        decode_depth_image(data, 2, 2, stride, 'rgb8')
