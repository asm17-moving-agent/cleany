"""Check preview timestamps and stale TF handling on an isolated ROS domain."""

import json
from types import SimpleNamespace

import numpy as np
import pytest

rclpy = pytest.importorskip('rclpy')
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import CameraInfo, Image
from cleany_navigation.depth_obstacle_preview_node import DepthObstaclePreviewNode
from cleany_navigation.point_cloud import cloud_xyz


def test_missing_tf_clears_old_output_even_with_new_images(monkeypatch, tmp_path):
    monkeypatch.setenv('ROS_DOMAIN_ID', '226')
    config = tmp_path/'filter.json'
    config.write_text(json.dumps({'floor_normal':[0,0,1], 'floor_offset':0, 'min_neighbors':1}))
    boxes = tmp_path/'boxes.json'
    boxes.write_text(json.dumps({'frame':'base_link','boxes':[]}))
    rclpy.init(args=['--ros-args','-p',f'filter_file:={config}','-p',f'body_boxes_file:={boxes}','-p','pixel_stride:=1'])
    node = None
    try:
        node = DepthObstaclePreviewNode()
        outputs = []
        node.obstacles = SimpleNamespace(publish=outputs.append)
        info = CameraInfo()
        info.width, info.height = 2, 2
        info.header.frame_id = 'optical_test'
        info.k = [100.,0.,0.,0.,100.,0.,0.,0.,1.]
        node.on_info(info)
        image = Image()
        image.width, image.height, image.step, image.encoding = 2, 2, 4, '16UC1'
        image.data = np.full((2,2),1000,dtype='<u2').tobytes()
        image.header.frame_id = 'optical_test'
        image.header.stamp = node.get_clock().now().to_msg()
        node.last_good_at -= 1
        node.on_depth(image)
        assert len(outputs) == 1 and len(cloud_xyz(outputs[-1])) == 0
        t = TransformStamped()
        t.header.frame_id, t.child_frame_id = 'base_link', 'optical_test'
        t.transform.rotation.w = 1.
        node.buffer.set_transform_static(t, 'test')
        node.process_pending()
        assert len(cloud_xyz(outputs[-1])) > 0
        assert outputs[-1].header.stamp == image.header.stamp
        # A continuing camera stream cannot keep old obstacles alive after TF loss.
        # tf2 clear() retains static caches; use an empty buffer to model loss.
        node.buffer = type(node.buffer)()
        node.last_good_at -= 1
        image.header.stamp = node.get_clock().now().to_msg()
        node.on_depth(image)
        assert len(cloud_xyz(outputs[-1])) == 0
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
