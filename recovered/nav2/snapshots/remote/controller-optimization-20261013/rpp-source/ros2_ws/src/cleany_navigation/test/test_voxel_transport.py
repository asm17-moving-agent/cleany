import json
import numpy as np
import pytest

rclpy = pytest.importorskip('rclpy')
from builtin_interfaces.msg import Time
from nav_msgs.msg import OccupancyGrid
from rclpy.serialization import deserialize_message, serialize_message
from sensor_msgs.msg import PointCloud2
from cleany_navigation.point_cloud import cloud_xyz, xyz_cloud
from cleany_navigation.obstacle_memory_node import ObstacleMemoryNode


def test_xyz_roundtrip_empty_cloud_and_truncation():
    points = np.array([[1., 2., 3.], [-.5, 0., 1.]], dtype=np.float32)
    msg = xyz_cloud(points, 'map', Time(sec=2))
    msg = deserialize_message(serialize_message(msg), PointCloud2)
    np.testing.assert_array_equal(cloud_xyz(msg), points)
    assert cloud_xyz(xyz_cloud(np.empty((0, 3)), 'map', Time())).shape == (0, 3)
    msg.data = msg.data[:-1]
    with pytest.raises(ValueError, match='Truncated'):
        cloud_xyz(msg)


def test_memory_node_initial_map_publishes_3d_metadata(tmp_path, monkeypatch):
    monkeypatch.setenv("ROS_DOMAIN_ID", "222")
    rclpy.init(args=['--ros-args', '-p', f'output_directory:={tmp_path}',
                    '-p', 'publish_3d:=true', '-p', 'mark_all_depth_points:=true'])
    node = None
    try:
        node = ObstacleMemoryNode()
        msg = OccupancyGrid()
        msg.header.frame_id = 'map'
        msg.info.width = msg.info.height = 4
        msg.info.resolution = .05
        msg.info.origin.orientation.w = 1.
        msg.data = [0]*16
        node.on_map(msg)
        status = json.loads((tmp_path/'voxel_status.json').read_text())
        assert status['enabled'] and status['occupied_voxels'] == 0
        np.testing.assert_allclose(status['voxel_size'], [.05, .05, .1])
        assert status['source_stamps'] == {}  # Initial empty map must not imply fresh sensors.
        assert node.values['mark_all_depth_points']
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


def test_memory_retries_exact_sensor_tf_and_preserves_signed_grid(monkeypatch):
    from types import SimpleNamespace
    from geometry_msgs.msg import TransformStamped
    from sensor_msgs.msg import LaserScan
    monkeypatch.setenv('ROS_DOMAIN_ID', '222')
    rclpy.init(args=['--ros-args', '-p', 'publish_3d:=true'])
    node = ObstacleMemoryNode()
    try:
        grid = OccupancyGrid()
        grid.header.frame_id = 'map'
        grid.info.width = grid.info.height = 20
        grid.info.resolution = .1
        grid.info.origin.orientation.w = 1.
        grid.data = [0]*400
        node.on_map(grid)
        scan = LaserScan()
        scan.header.frame_id = 'laser'
        scan.header.stamp = node.get_clock().now().to_msg()
        scan.range_min, scan.range_max = .01, 5.
        scan.ranges = [1.]
        node.latest['lidar'] = scan
        base = TransformStamped()
        base.header.frame_id, base.child_frame_id = 'map', 'base_link'
        base.header.stamp = scan.header.stamp
        base.transform.rotation.w = 1.
        node.buffer.set_transform(base, 'base_pose')
        node.update()
        assert node.counts['lidar'] == 0 and node.source_stamps == {}
        tf = TransformStamped()
        tf.header.frame_id, tf.child_frame_id = 'map', 'laser'
        tf.header.stamp = scan.header.stamp
        tf.transform.rotation.w = 1.
        tf.transform.translation.x = tf.transform.translation.y = .15
        tf.transform.translation.z = .15
        node.buffer.set_transform(tf, 'delayed_sensor_tf')
        outputs = []
        node.grid_publishers['navigation_grid'] = SimpleNamespace(publish=outputs.append)
        node.update()
        assert node.counts['lidar'] == 1
        assert node.processed['lidar'] == (scan.header.stamp.sec, scan.header.stamp.nanosec)
        transported = deserialize_message(serialize_message(outputs[-1]), OccupancyGrid)
        np.testing.assert_array_equal(transported.data, node.memory.grid().ravel())
        assert set(transported.data) == {-1, 0, 100}
        scan.header.stamp.sec -= 2  # An available older TF must not admit stale data.
        tf.header.stamp = scan.header.stamp
        node.buffer.set_transform(tf, 'stale_sensor_tf')
        node.update()
        assert node.counts['lidar'] == 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_expired_evidence_is_unknown_in_diagnostics_but_blocks_shared_navigation(monkeypatch):
    from types import SimpleNamespace
    monkeypatch.setenv('ROS_DOMAIN_ID', '222')
    rclpy.init(args=['--ros-args', '-p', 'publish_3d:=true'])
    node = ObstacleMemoryNode()
    try:
        grid = OccupancyGrid()
        grid.header.frame_id = 'map'
        grid.info.width = grid.info.height = 20
        grid.info.resolution = .1
        grid.info.origin.orientation.w = 1.
        grid.data = [0]*400
        node.on_map(grid)
        node.center = np.array([0., 0.])
        node.memory.observe('depth', np.array([.15, .15, .5]),
            np.array([[1.05, .15, .5]]), np.array([True]), 100.)
        assert node.memory.expire(111., 10.) == 1
        outputs, clouds = [], []
        node.grid_publishers['navigation_grid'] = SimpleNamespace(publish=outputs.append)
        node.local_publisher = SimpleNamespace(publish=clouds.append)
        node.publish()
        transported = deserialize_message(serialize_message(outputs[-1]), OccupancyGrid)
        assert transported.data[30] == 100
        assert node.memory.grid()[1, 10] == -1
        assert cloud_xyz(clouds[-1]).shape == (1, 3)
        node.memory.observe('depth', np.array([.15, .15, .5]),
            np.array([[1.5, .15, .5]]), np.array([False]), 112.)
        node.publish()
        assert outputs[-1].data[30] == 0
        assert cloud_xyz(clouds[-1]).shape == (0, 3)
    finally:
        node.destroy_node()
        rclpy.shutdown()
