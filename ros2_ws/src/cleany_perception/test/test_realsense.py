"""Saved profile checks and opt-in, read-only physical camera acceptance."""

import os
from pathlib import Path
import time

import pytest
import yaml


def test_saved_realsense_profile():
    config = Path(__file__).parents[1] / 'config' / 'realsense.yaml'
    params = yaml.safe_load(config.read_text())['/camera']['ros__parameters']
    assert params['rgb_camera.color_profile'] == '640,480,30'
    assert params['depth_module.depth_profile'] == '640,480,30'
    assert params['rgb_camera.color_format'] == 'RGB8'
    assert params['depth_module.depth_format'] == 'Z16'
    for name in ('enable_sync', 'align_depth.enable',
                 'rgb_camera.enable_auto_exposure',
                 'rgb_camera.enable_auto_white_balance',
                 'depth_module.enable_auto_exposure'):
        assert params[name] is True
    for name in ('colorizer.enable', 'hole_filling_filter.enable',
                 'decimation_filter.enable', 'pointcloud.enable'):
        assert params[name] is False


def test_saved_pregrasp_realsense_profile_limits_stream_rate():
    config = Path(__file__).parents[1] / 'config' / 'realsense_pregrasp.yaml'
    params = yaml.safe_load(config.read_text())['/camera']['ros__parameters']
    assert params['rgb_camera.color_profile'] == '640,480,15'
    assert params['depth_module.depth_profile'] == '640,480,15'
    assert params['enable_sync'] is True
    assert params['align_depth.enable'] is True


@pytest.mark.skipif(
    os.environ.get('CLEANY_TEST_REALSENSE') != '1',
    reason='Requires a running realsense.launch.py and physical camera',
)
def test_live_aligned_snapshot():
    import numpy as np
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import CameraInfo, Image

    from cleany_perception.rgbd_snapshot import (
        RgbdSnapshotBuffer, snapshot_from_messages, stamp_nanoseconds,
    )

    rclpy.init()
    node = rclpy.create_node('realsense_acceptance')
    buffer = RgbdSnapshotBuffer()
    subscriptions = []
    for kind, topic, message_type in (
        ('color', '/camera/color/image_raw', Image),
        ('color_info', '/camera/color/camera_info', CameraInfo),
        ('depth', '/camera/depth/image_raw', Image),
        ('depth_info', '/camera/depth/camera_info', CameraInfo),
    ):
        subscriptions.append(node.create_subscription(
            message_type, topic, getattr(buffer, 'add_' + kind),
            qos_profile_sensor_data,
        ))
    try:
        deadline = time.monotonic() + 25.0
        stamps = []
        arrivals = []
        sequence = 0
        while time.monotonic() < deadline and len(stamps) < 90:
            rclpy.spin_once(node, timeout_sec=0.1)
            if buffer.sequence <= sequence:
                continue
            messages = buffer.wait_for_new(sequence, 0.1)
            sequence = messages.sequence
            snapshot = snapshot_from_messages(messages)
            assert snapshot.rgb.shape == (480, 640, 3)
            assert snapshot.depth_m.shape == (480, 640)
            assert messages.color.encoding == 'rgb8'
            assert messages.depth.encoding == '16UC1'
            assert (messages.color.header.frame_id
                    == messages.depth.header.frame_id)
            assert len({stamp_nanoseconds(msg) for msg in (
                messages.color, messages.color_info,
                messages.depth, messages.depth_info,
            )}) == 1
            assert np.count_nonzero(snapshot.depth_m > 0) > 0
            stamps.append(messages.stamp_ns)
            arrivals.append(time.monotonic())
        assert len(stamps) == 90, f'Only {len(stamps)} RGB-D frames received'
        received_fps = (len(stamps) - 1) / (arrivals[-1] - arrivals[0])
        stamp_fps = (len(stamps) - 1) * 1e9 / (stamps[-1] - stamps[0])
        print(f'RGBD_OK frames={len(stamps)} '
              f'received_fps={received_fps:.2f} stamp_fps={stamp_fps:.2f}')
    finally:
        node.destroy_node()
        rclpy.shutdown()
