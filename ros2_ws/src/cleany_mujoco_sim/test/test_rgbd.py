from __future__ import annotations

import math

import mujoco
import numpy as np
import pytest
from rclpy.time import Time

from cleany_mujoco_sim.extensions import MujocoSimulationContext
from cleany_mujoco_sim.rgbd import (
    GroundTruthSpec,
    RgbdSensorBridge,
    RgbdSensorConfig,
    camera_info_msg,
    camera_intrinsics,
    depth_image_msg,
    rgb_image_msg,
    sanitize_depth,
)
from cleany_mujoco_sim.scene_loader import load_model
from cleany_mujoco_sim.state import initialize_joint_positions


class _RecordingPublisher:
    def __init__(self) -> None:
        self.messages = []

    def publish(self, message) -> None:
        self.messages.append(message)


class _RecordingNode:
    def __init__(self) -> None:
        self.publishers = {}

    def create_publisher(self, _message_type, topic, _qos):
        publisher = _RecordingPublisher()
        self.publishers[topic] = publisher
        return publisher


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


def test_rgbd_sensor_bridge_renders_aligned_frames_and_ground_truth(
    rgbd_pick_scene_path,
):
    model, data = load_model(rgbd_pick_scene_path)
    initialize_joint_positions(
        model,
        data,
        ['head_tilt_joint'],
        [1.0],
    )
    mujoco.mj_forward(model, data)
    context = MujocoSimulationContext(model=model, data=data)
    node = _RecordingNode()
    config = RgbdSensorConfig()
    bridge = RgbdSensorBridge(
        node,
        context,
        config,
        (
            GroundTruthSpec('pick_box_geom', 'box'),
            GroundTruthSpec('pick_can_geom', 'can'),
        ),
    )

    try:
        stamp = Time(nanoseconds=2_500_000_000)
        bridge.after_step(context, stamp)

        assert set(node.publishers) == {
            config.color_image_topic,
            config.color_info_topic,
            config.depth_image_topic,
            config.depth_info_topic,
            config.ground_truth_topic,
        }
        assert all(
            len(publisher.messages) == 1
            for publisher in node.publishers.values()
        )

        color = node.publishers[config.color_image_topic].messages[0]
        color_info = node.publishers[config.color_info_topic].messages[0]
        depth = node.publishers[config.depth_image_topic].messages[0]
        depth_info = node.publishers[config.depth_info_topic].messages[0]
        ground_truth = node.publishers[config.ground_truth_topic].messages[0]

        assert (color.width, color.height, color.encoding) == (
            640,
            480,
            'rgb8',
        )
        assert color.header.frame_id == config.color_frame_id
        assert (depth.width, depth.height, depth.encoding) == (
            640,
            480,
            '32FC1',
        )
        assert depth.header.frame_id == config.depth_frame_id
        assert color.header.stamp == depth.header.stamp
        assert color_info.header.stamp == depth_info.header.stamp
        assert color_info.k == pytest.approx(depth_info.k)
        assert np.frombuffer(color.data, dtype=np.uint8).any()

        depth_array = np.frombuffer(depth.data, dtype='<f4').reshape(480, 640)
        finite_depth = depth_array[np.isfinite(depth_array)]
        assert finite_depth.size > 0
        assert np.all(finite_depth > 0.0)

        assert ground_truth.header.frame_id == 'base_link'
        assert ground_truth.header.stamp == color.header.stamp
        assert ground_truth.snapshot_id == 'sim-0000000000000000000'
        assert [obj.object_id for obj in ground_truth.objects] == [1, 2]
        assert [obj.label for obj in ground_truth.objects] == ['box', 'can']
        box, can = ground_truth.objects
        assert box.obb_pose.position.y < 0.0
        assert can.obb_pose.position.y > 0.0
        assert (
            box.obb_size.x,
            box.obb_size.y,
            box.obb_size.z,
        ) == pytest.approx((0.10, 0.08, 0.08))
        assert (
            can.obb_size.x,
            can.obb_size.y,
            can.obb_size.z,
        ) == pytest.approx((0.07, 0.07, 0.10))

        bridge.after_step(context, stamp)
        assert all(
            len(publisher.messages) == 1
            for publisher in node.publishers.values()
        )

        data.time = 0.2
        bridge.after_step(context, Time(nanoseconds=2_600_000_000))
        assert all(
            len(publisher.messages) == 1
            for publisher in node.publishers.values()
        )

        bridge.after_step(context, Time(nanoseconds=2_700_000_000))
        assert all(
            len(publisher.messages) == 2
            for publisher in node.publishers.values()
        )
    finally:
        bridge.close()

    assert bridge.closed
    with pytest.raises(RuntimeError, match='renderer is closed'):
        bridge.capture(context, Time())


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
