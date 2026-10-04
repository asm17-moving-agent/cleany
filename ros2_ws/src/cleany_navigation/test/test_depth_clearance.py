import numpy as np
import pytest

from cleany_navigation.depth_clearance import prepare_depth_points, project_depth_image


def test_self_occlusion_discards_hits_behind_rail_without_erasing_visible_obstacles():
    from cleany_navigation.body_geometry import BodyBox
    from cleany_navigation.depth_clearance import self_observation_mask
    rail = BodyBox('rail', np.array([1., 0., 0.]), np.eye(3), np.array([.01, .1, .1]))
    points = np.array([[1., 0., 0.], [1.06, 0., 0.], [2., .5, 0.], [.8, 0., 0.]])
    np.testing.assert_array_equal(self_observation_mask(points, np.zeros(3), [rail], .001),
                                  [False, False, True, True])


def test_self_occlusion_uses_rotated_box_and_parallel_ray_bounds():
    from cleany_navigation.body_geometry import BodyBox
    from cleany_navigation.depth_clearance import self_observation_mask
    rotation = np.array([[0., -1., 0.], [1., 0., 0.], [0., 0., 1.]])
    box = BodyBox('rail', np.array([0., 1., 0.]), rotation, np.array([.01, .1, .1]))
    points = np.array([[0., 2., 0.], [.5, 2., 0.], [0., .5, 0.]])
    np.testing.assert_array_equal(self_observation_mask(points, np.zeros(3), [box], 0.), [False, True, True])


def test_self_filter_matches_scalar_closed_segment_intersection():
    """Self hits/occluded rays must never become free-space observations."""
    from cleany_navigation.body_geometry import BodyBox
    from cleany_navigation.depth_clearance import self_observation_mask
    rng = np.random.default_rng(43)
    origin = np.array([.13, -.002, .79])
    boxes = []
    for i in range(6):
        rotation, _ = np.linalg.qr(rng.normal(size=(3, 3)))
        if np.linalg.det(rotation) < 0:
            rotation[:, 0] *= -1
        boxes.append(BodyBox(str(i), rng.uniform(-1, 1, 3), rotation,
                             rng.uniform(.01, .4, 3)))
    points = rng.uniform(-3, 3, (500, 3))
    points = np.vstack((points, origin, [np.nan, 0, 0], [np.inf, 0, 0]))

    def visible(point):
        if not np.isfinite(point).all():
            return False
        for box in boxes:
            start = (origin-box.center)@box.rotation
            delta = (point-origin)@box.rotation
            enter, leave = 0., 1.
            for axis in range(3):
                half = box.half_size[axis]+.001
                if abs(delta[axis]) < 1e-12:
                    if abs(start[axis]) > half:
                        break
                else:
                    a, b = sorted(((-half-start[axis])/delta[axis],
                                   (half-start[axis])/delta[axis]))
                    enter, leave = max(enter, a), min(leave, b)
                if enter > leave:
                    break
            else:
                return False
        return True

    with np.errstate(invalid='ignore'):
        actual = self_observation_mask(points, origin, boxes, .001)
    np.testing.assert_array_equal(actual, [visible(p) for p in points])
    assert self_observation_mask(np.empty((0, 3)), origin, boxes, .001).size == 0


def test_self_filter_boundary_contact_and_sensor_inside_are_occluded():
    from cleany_navigation.body_geometry import BodyBox
    from cleany_navigation.depth_clearance import self_observation_mask
    box = BodyBox('rail', np.array([1., 0., 0.]), np.eye(3), np.full(3, .1))
    points = np.array([[.9, 0, 0], [.9-1e-10, 0, 0], [2, .2, 0],
                       [2, .2+1e-8, 0], [0, 0, 0]])
    np.testing.assert_array_equal(self_observation_mask(points, np.zeros(3), [box], 0.),
                                  [False, True, False, False, True])
    np.testing.assert_array_equal(self_observation_mask(points, box.center, [box], 0.),
                                  np.zeros(len(points), dtype=bool))


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


def test_floor_is_not_marked_but_low_obstacles_are_retained():
    from cleany_navigation.depth_clearance import obstacle_height_mask
    points = np.array([[1., 0., -.38], [1., 0., -.33], [1., 0., -.32], [1., 0., 1.7]])
    np.testing.assert_array_equal(obstacle_height_mask(points, -.33, 1.62), [False, True, True, False])
    np.testing.assert_array_equal(points[:, 2], [-.38, -.33, -.32, 1.7])
