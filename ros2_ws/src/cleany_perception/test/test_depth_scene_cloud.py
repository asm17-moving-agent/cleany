from copy import deepcopy
from types import SimpleNamespace

import numpy as np
import pytest
from sensor_msgs.msg import CameraInfo, Image, PointCloud2

from cleany_perception.depth_scene_cloud import depth_scene_cloud
from cleany_perception.depth_scene_node import DepthSceneNode
from cleany_perception.scene_cloud_receipt_node import receipt_for_cloud


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
        _updates_enabled=True,
        _depths={(10, 0): depth}, _infos={(10, 0): info}, _published_stamp=None,
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(
            nanoseconds=int((10 + age) * 1e9))),
        get_parameter=lambda name: SimpleNamespace(value=1.0),
        _publisher=SimpleNamespace(publish=published.append,
                                   get_subscription_count=lambda: 1),
    )
    DepthSceneNode._publish(node)
    assert published == []


def test_node_skips_projection_without_cloud_subscriber(monkeypatch):
    depth, info = messages([[1]])
    monkeypatch.setattr(
        'cleany_perception.depth_scene_node.depth_scene_cloud',
        lambda *args: pytest.fail('Projected depth without a subscriber'),
    )
    node = SimpleNamespace(
        _updates_enabled=True, _depth=depth, _info=info,
        _publisher=SimpleNamespace(get_subscription_count=lambda: 0),
    )
    DepthSceneNode._publish(node)


def test_full_view_retains_obstacle_outside_old_pick_roi():
    values = np.zeros((480, 640), dtype=np.float32)
    values[200:240, 20:60] = .6
    values[200:240, 300:340] = .8
    depth, info = messages(values)
    full = depth_scene_cloud(depth, info, .1, 2., 4)
    restricted = depth_scene_cloud(depth, info, .1, 2., 4,
                                  workspace_roi_pixels=[140.,90.,500.,430.])
    assert full.width == 200
    assert restricted.width == 100
    xyz = np.frombuffer(full.data, dtype='<f4').reshape(-1, 3)
    assert np.count_nonzero(np.isclose(xyz[:, 2], .6)) == 100
    assert not np.any(np.isclose(
        np.frombuffer(restricted.data, dtype='<f4').reshape(-1, 3)[:, 2], .6))


def test_node_default_full_view_and_explicit_roi_are_independent():
    depth, info = messages(np.ones((4, 4), dtype=np.float32))
    parameters = dict(minimum_depth_m=.1, maximum_depth_m=2., pixel_stride=1,
        depth_16u_scale_m=.001, maximum_depth_age_sec=1., workspace_roi_pixels=[-1.]*4)
    published = []
    node = SimpleNamespace(_updates_enabled=True, _published_stamp=None,
        _depths={(10, 0): depth}, _infos={(10, 0): info},
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=10_000_000_000)),
        get_parameter=lambda name: SimpleNamespace(value=parameters[name]),
        get_logger=lambda: SimpleNamespace(warning=lambda *args, **kwargs: None),
        _publisher=SimpleNamespace(publish=published.append, get_subscription_count=lambda: 1))
    DepthSceneNode._publish(node)
    assert published[-1].width == 16
    parameters['workspace_roi_pixels'] = [1., 1., 3., 3.]
    node._published_stamp = None
    DepthSceneNode._publish(node)
    assert published[-1].width == 4


def diagnostic_node(depth, info, *, subscribers=1, enabled=True):
    parameters = dict(minimum_depth_m=.1, maximum_depth_m=2., pixel_stride=1,
        depth_16u_scale_m=.001, maximum_depth_age_sec=1.,
        workspace_roi_pixels=[-1.]*4, diagnostic_interval_sec=5.)
    published, warnings, informational = [], [], []
    node = SimpleNamespace(
        _updates_enabled=enabled, _published_stamp=None,
        _depths={(depth.header.stamp.sec, depth.header.stamp.nanosec): depth},
        _infos={(info.header.stamp.sec, info.header.stamp.nanosec): info},
        _created_wall_sec=0., _last_publication_wall_sec=None,
        _last_diagnostic_wall_sec=None, _input_counts={'depth': 15, 'info': 15},
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=10_500_000_000)),
        get_parameter=lambda name: SimpleNamespace(value=parameters[name]),
        get_logger=lambda: SimpleNamespace(warning=lambda text, **kwargs: warnings.append(text),
                                           info=informational.append),
        _publisher=SimpleNamespace(publish=published.append,
                                   get_subscription_count=lambda: subscribers),
    )
    return node, published, warnings, informational


def test_fresh_unmatched_inputs_are_diagnosed_without_approximating_timestamps(monkeypatch):
    depth, info = messages([[1.]])
    info = deepcopy(info)
    info.header.stamp.nanosec = 100_000_000
    node, published, warnings, _ = diagnostic_node(depth, info)
    wall = [10.]
    monkeypatch.setattr('cleany_perception.depth_scene_node.time.monotonic', lambda: wall[0])
    DepthSceneNode._publish(node)
    assert published == []
    assert len(warnings) == 1
    assert 'reason=unmatched_timestamps' in warnings[0]
    assert 'depth_received=15 info_received=15' in warnings[0]
    assert 'matching_stamps=0' in warnings[0]
    assert 'depth_age_sec=0.500 info_age_sec=0.400 pair_age_sec=not_received' in warnings[0]
    wall[0] = 14.9
    DepthSceneNode._publish(node)
    assert len(warnings) == 1
    wall[0] = 15.
    DepthSceneNode._publish(node)
    assert len(warnings) == 2


@pytest.mark.parametrize('fault,reason', [
    ('depth', 'missing_depth'), ('info', 'missing_camera_info'),
    ('future', 'future_depth'), ('empty', 'no_valid_depth_points'),
    ('paused', 'updates_paused'), ('subscriber', 'no_cloud_subscriber'),
])
def test_diagnostics_distinguish_input_failures_from_intentional_waits(monkeypatch, fault, reason):
    depth, info = messages([[0. if fault == 'empty' else 1.]])
    if fault == 'future':
        depth.header.stamp.sec = info.header.stamp.sec = 12
    node, published, warnings, informational = diagnostic_node(
        depth, info, subscribers=0 if fault == 'subscriber' else 1,
        enabled=fault != 'paused')
    if fault == 'depth':
        node._depths.clear()
    elif fault == 'info':
        node._infos.clear()
    monkeypatch.setattr('cleany_perception.depth_scene_node.time.monotonic', lambda: 10.)
    DepthSceneNode._publish(node)
    assert published == []
    logs = informational if fault in ('paused', 'subscriber') else warnings
    assert len(logs) == 1
    assert f'reason={reason}' in logs[0]


def test_stalled_node_only_recovers_with_fresh_capture_and_keeps_its_stamp(monkeypatch):
    depth, info = messages([[1.]])
    depth.header.stamp.sec = info.header.stamp.sec = 9
    node, published, warnings, informational = diagnostic_node(depth, info)
    monkeypatch.setattr('cleany_perception.depth_scene_node.time.monotonic', lambda: 10.)
    DepthSceneNode._publish(node)
    assert published == []
    assert 'reason=stale_depth' in warnings[0]
    fresh_depth, fresh_info = messages([[1.]])
    node._depths[(10, 0)] = fresh_depth
    node._infos[(10, 0)] = fresh_info
    DepthSceneNode._publish(node)
    assert len(published) == 1
    assert published[0].header == fresh_depth.header
    assert 'Scene cloud recovered: points=1 capture_age_sec=0.500' in informational[0]
    assert node._last_diagnostic_wall_sec is None


def cloud():
    message = PointCloud2(width=2, height=1, point_step=12, row_step=24, data=bytes(24))
    message.header.frame_id = 'rgb_optical'
    message.header.stamp.sec = 7
    return message


def test_capture_stamp_is_preserved_not_replaced_by_current_time():
    source = cloud()
    before = deepcopy(source)
    receipt = receipt_for_cloud(source)
    assert receipt == before.header
    source.header.stamp.sec = 99
    assert receipt.stamp.sec == 7


@pytest.mark.parametrize('field,value', [
    ('width', 0), ('height', 0), ('point_step', 0), ('row_step', 1), ('data', bytes(23)),
])
def test_empty_or_truncated_cloud_never_creates_a_receipt(field, value):
    source = cloud()
    setattr(source, field, value)
    assert receipt_for_cloud(source) is None


def test_missing_frame_or_capture_stamp_is_rejected():
    source = cloud()
    source.header.frame_id = ''
    assert receipt_for_cloud(source) is None
    source = cloud()
    source.header.stamp.sec = 0
    assert receipt_for_cloud(source) is None
