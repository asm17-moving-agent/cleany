from __future__ import annotations

import math

import numpy as np
import pytest
from rclpy.time import Time

from cleany_mujoco_sim.rgbd import (
    RgbdSensorConfig,
    camera_info_msg,
    camera_intrinsics,
    depth_image_msg,
    rgb_image_msg,
    sanitize_depth,
)






def test_camera_intrinsics_use_vertical_field_of_view():
    fx, fy, cx, cy = camera_intrinsics(640, 480, 42.0)
    expected_focal_length = 240.0 / math.tan(math.radians(21.0))

    assert fx == pytest.approx(expected_focal_length)
    assert fy == pytest.approx(expected_focal_length)
    assert cx == pytest.approx(319.5)
    assert cy == pytest.approx(239.5)


def test_sanitize_depth_converts_invalid_and_far_pixels_to_nan():
    depth = np.array(
        [[0.0, 0.5, 2.0], [np.inf, np.nan, -1.0]],
        dtype=np.float64,
    )

    sanitized = sanitize_depth(depth, far_plane_m=2.0)

    assert sanitized.dtype == np.float32
    assert sanitized[0, 1] == pytest.approx(0.5)
    assert np.isnan(sanitized[0, 0])
    assert np.isnan(sanitized[0, 2])
    assert np.isnan(sanitized[1]).all()


def test_rgbd_message_helpers_preserve_shape_stamp_and_frames():
    stamp = Time(nanoseconds=1_234_567_890)
    rgb = np.arange(18, dtype=np.uint8).reshape(2, 3, 3)
    depth = np.array([[0.25, np.nan, 0.5], [1.0, 1.5, 2.0]], dtype=np.float32)

    rgb_msg = rgb_image_msg(rgb, stamp, 'rgb_optical')
    depth_msg = depth_image_msg(depth, stamp, 'depth_optical')
    info_msg = camera_info_msg(3, 2, 60.0, stamp, 'rgb_optical')

    assert (rgb_msg.height, rgb_msg.width, rgb_msg.step) == (2, 3, 9)
    assert rgb_msg.encoding == 'rgb8'
    assert rgb_msg.header.frame_id == 'rgb_optical'
    assert bytes(rgb_msg.data) == rgb.tobytes()
    assert (depth_msg.height, depth_msg.width, depth_msg.step) == (2, 3, 12)
    assert depth_msg.encoding == '32FC1'
    assert depth_msg.header.frame_id == 'depth_optical'
    assert bytes(depth_msg.data) == depth.astype('<f4').tobytes()
    assert info_msg.header.stamp == rgb_msg.header.stamp
    assert info_msg.width == 3
    assert info_msg.height == 2
    assert info_msg.k[0] == pytest.approx(info_msg.k[4])
    assert info_msg.k[2] == pytest.approx(1.0)
    assert info_msg.k[5] == pytest.approx(0.5)




@pytest.mark.parametrize(
    ('kwargs', 'message'),
    [
        ({'width': 0}, 'width and height'),
        ({'rate_hz': 0.0}, 'rate_hz'),
        ({'camera_name': ''}, 'camera_name'),
    ],
)
def test_rgbd_sensor_config_rejects_invalid_values(kwargs, message):
    with pytest.raises(ValueError, match=message):
        RgbdSensorConfig(**kwargs)
