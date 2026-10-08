from __future__ import annotations

import time

import rclpy
from cleany_interfaces.msg import DetectedObject3DArray
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image

from cleany_mujoco_sim.rgbd_sim_node import MujocoRgbdSimNode


def _make_node(scene_path) -> MujocoRgbdSimNode:
    parameters = {
        'scene_path': str(scene_path),
        'publish_rate_hz': 1000.0,
        'headless': True,
        'scan_enabled': False,
        'initial_joint_names': ['head_tilt_joint'],
        'initial_joint_positions': [0.8],
        'rgbd_rate_hz': 5.0,
        'ground_truth_geom_names': ['pick_box_geom', 'pick_can_geom'],
        'ground_truth_labels': ['box', 'can'],
    }
    return MujocoRgbdSimNode(
        namespace='test_rgbd_sim',
        parameter_overrides=[
            Parameter(name, value=value) for name, value in parameters.items()
        ],
    )


def test_rgbd_sim_node_publishes_synchronized_sensor_contracts(
    rgbd_pick_scene_path,
):
    rclpy.init(args=[])
    node = None
    try:
        node = _make_node(rgbd_pick_scene_path)
        received = {
            'color': [],
            'color_info': [],
            'depth': [],
            'depth_info': [],
            'ground_truth': [],
        }
        node.create_subscription(
            Image,
            'camera/color/image_raw',
            received['color'].append,
            qos_profile_sensor_data,
        )
        node.create_subscription(
            CameraInfo,
            'camera/color/camera_info',
            received['color_info'].append,
            qos_profile_sensor_data,
        )
        node.create_subscription(
            Image,
            'camera/depth/image_raw',
            received['depth'].append,
            qos_profile_sensor_data,
        )
        node.create_subscription(
            CameraInfo,
            'camera/depth/camera_info',
            received['depth_info'].append,
            qos_profile_sensor_data,
        )
        node.create_subscription(
            DetectedObject3DArray,
            'ground_truth/objects',
            received['ground_truth'].append,
            qos_profile_sensor_data,
        )

        node._on_timer()
        deadline = time.time() + 2.0
        while not all(received.values()) and time.time() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)

        assert all(received.values())
        color = received['color'][0]
        color_info = received['color_info'][0]
        depth = received['depth'][0]
        depth_info = received['depth_info'][0]
        ground_truth = received['ground_truth'][0]
        assert (color.width, color.height, color.encoding) == (
            640,
            480,
            'rgb8',
        )
        assert (depth.width, depth.height, depth.encoding) == (
            640,
            480,
            '32FC1',
        )
        assert color.header.stamp == depth.header.stamp
        assert color_info.header.stamp == depth_info.header.stamp
        assert color.header.frame_id == 'head_camera_rgb_optical_frame'
        assert depth.header.frame_id == 'head_camera_depth_optical_frame'
        assert ground_truth.header.frame_id == 'base_link'
        assert [obj.label for obj in ground_truth.objects] == ['box', 'can']
    finally:
        if node is not None:
            node.destroy_node()
            assert node._rgbd_bridge.closed
        rclpy.shutdown()
