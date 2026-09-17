import json
import numpy as np
import pytest

rclpy = pytest.importorskip('rclpy')
from builtin_interfaces.msg import Time
from nav_msgs.msg import OccupancyGrid
from cleany_gazebo_sim.point_cloud import cloud_xyz, xyz_cloud
from cleany_gazebo_sim.obstacle_memory_node import ObstacleMemoryNode


def test_xyz_roundtrip_empty_cloud_and_truncation():
    points = np.array([[1., 2., 3.], [-.5, 0., 1.]], dtype=np.float32)
    msg = xyz_cloud(points, 'map', Time(sec=2))
    np.testing.assert_array_equal(cloud_xyz(msg), points)
    assert cloud_xyz(xyz_cloud(np.empty((0, 3)), 'map', Time())).shape == (0, 3)
    msg.data = msg.data[:-1]
    with pytest.raises(ValueError, match='Truncated'):
        cloud_xyz(msg)


def test_memory_node_initial_map_publishes_3d_metadata(tmp_path):
    rclpy.init(args=['--ros-args', '-p', f'output_directory:={tmp_path}',
                    '-p', 'publish_3d:=true', '-p', 'mark_all_depth_points:=true'], domain_id=222)
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
