from types import SimpleNamespace

import numpy as np
import pytest
from sensor_msgs.msg import CameraInfo, Image

from cleany_perception.depth_scene_cloud import depth_scene_cloud
from cleany_perception.depth_scene_node import DepthSceneNode


def messages(array, encoding='32FC1', bigendian=False, padding=0):
    dtype = ('>' if bigendian else '<') + (
        'f4' if encoding == '32FC1' else 'u2'
    )
    values = np.asarray(array, dtype=dtype)
    image = Image()
    image.header.frame_id = 'depth_optical'
    image.header.stamp.sec = 10
    image.height, image.width = values.shape
    image.encoding = encoding
    image.is_bigendian = bigendian
    image.step = values.shape[1] * values.dtype.itemsize + padding
    image.data = b''.join(row.tobytes() + bytes(padding) for row in values)
    info = CameraInfo()
    info.header = image.header
    info.height, info.width = values.shape
    info.k = [2., 0., 0., 0., 2., 0., 0., 0., 1.]
    return image, info


def test_full_depth_projection_has_no_detection_dependency():
    depth, info = messages([[1, 1], [0.5, 0.5]])
    cloud = depth_scene_cloud(depth, info, 0.1, 2.0, 1)
    xyz = np.frombuffer(cloud.data, dtype='<f4').reshape(-1, 3)
    np.testing.assert_allclose(
        xyz, [[0, 0, 1], [0.5, 0, 1], [0, 0.25, 0.5], [0.25, 0.25, 0.5]]
    )
    assert cloud.header == depth.header
    assert cloud.row_step == cloud.width * cloud.point_step


@pytest.mark.parametrize('bigendian', [False, True])
def test_uint16_depth_scale_and_padded_rows(bigendian):
    depth, info = messages([[1000, 0], [500, 3000]], '16UC1', bigendian, 4)
    cloud = depth_scene_cloud(depth, info, 0.1, 2.0, 1)
    xyz = np.frombuffer(cloud.data, dtype='<f4').reshape(-1, 3)
    np.testing.assert_allclose(xyz[:, 2], [1., 0.5])


def test_missing_depth_not_filled_and_stride_samples_full_view():
    depth, info = messages([[np.nan, 1, 1], [np.inf, 0, 3], [1, 1, 1]])
    cloud = depth_scene_cloud(depth, info, 0.1, 2.0, 2)
    assert cloud.width == 3
    depth, info = messages([[np.nan, 0]])
    assert depth_scene_cloud(depth, info, 0.1, 2.0, 1).width == 0


@pytest.mark.parametrize('fault', [
    'frame', 'size', 'intrinsics', 'distortion', 'nan_distortion',
])
def test_reject_incompatible_calibration(fault):
    depth, info = messages([[1, 1]])
    # Replace the shared Header for the frame-mismatch test.
    if fault == 'frame':
        from std_msgs.msg import Header
        info.header = Header(frame_id='wrong_camera')
    elif fault == 'size':
        info.width = 100
    elif fault == 'intrinsics':
        info.k[0] = 0.
    elif fault == 'distortion':
        info.d = [0.1]
    else:
        info.d = [float('nan')]
    with pytest.raises(ValueError):
        depth_scene_cloud(depth, info, 0.1, 2.0, 1)


@pytest.mark.parametrize('age', [-1., 2.])
def test_node_does_not_republish_future_or_stale_depth(age):
    depth, info = messages([[1]])
    published = []
    node = SimpleNamespace(
        _depth=depth, _info=info, _published_stamp=None,
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(
            nanoseconds=int((10 + age) * 1e9))),
        get_parameter=lambda name: SimpleNamespace(value=1.0),
        _publisher=SimpleNamespace(publish=published.append),
    )
    DepthSceneNode._publish(node)
    assert published == []
