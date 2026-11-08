"""Exercise the installed VoxelLayer on an isolated domain, without motion."""

import os
from pathlib import Path
import subprocess
import time

import numpy as np
import pytest
import yaml

rclpy = pytest.importorskip('rclpy')
from ament_index_python.packages import get_package_prefix
from geometry_msgs.msg import TransformStamped
from lifecycle_msgs.srv import ChangeState
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from tf2_ros import StaticTransformBroadcaster

from cleany_navigation.point_cloud import xyz_cloud


def test_mark_retain_until_observed_free_and_clear(tmp_path):
    package = Path(__file__).resolve().parents[1]
    layer = yaml.safe_load((package/'config/phone_depth_local.yaml').read_text())['local_costmap']['local_costmap']['ros__parameters']['depth_layer']
    parameters = dict(global_frame='odom', robot_base_frame='base_link', rolling_window=False,
                      width=6, height=6, origin_x=-3., origin_y=-3., resolution=.05,
                      update_frequency=10., publish_frequency=10., always_send_full_costmap=True,
                      robot_radius=.1, plugins=['depth_layer'], depth_layer=layer,
                      transform_tolerance=.3, use_sim_time=False)
    config = tmp_path/'costmap.yaml'
    config.write_text(yaml.safe_dump({'/**': {'ros__parameters': parameters}}))
    log = (tmp_path/'costmap.log').open('w')
    binary = Path(get_package_prefix('nav2_costmap_2d'))/'lib/nav2_costmap_2d/nav2_costmap_2d'
    process = subprocess.Popen([str(binary), '--ros-args', '--params-file', str(config),
                                '-r', 'costmap:=/test/grid'],
                               env={**os.environ, 'ROS_DOMAIN_ID': '192'}, stdout=log, stderr=log)
    rclpy.init(domain_id=192)
    node = rclpy.create_node('depth_costmap_test')
    marks = node.create_publisher(PointCloud2, '/perception/depth/nav_obstacle_points', qos_profile_sensor_data)
    rays = node.create_publisher(PointCloud2, '/perception/depth/nav_points', qos_profile_sensor_data)
    maps = []
    node.create_subscription(OccupancyGrid, '/test/grid', maps.append, 10)
    broadcaster = StaticTransformBroadcaster(node)
    transforms = []
    for parent, child, height in [('odom', 'base_link', 0.), ('base_link', 'd435_depth_frame', 1.)]:
        msg = TransformStamped()
        msg.header.frame_id, msg.child_frame_id = parent, child
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.transform.rotation.w = 1.
        msg.transform.translation.z = height
        transforms.append(msg)
    broadcaster.sendTransform(transforms)

    def pump(seconds):
        end = time.monotonic()+seconds
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=.02)

    def transition(client, identifier):
        request = ChangeState.Request()
        request.transition.id = identifier
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=5.)
        assert future.done() and future.result().success

    def observe(hit, clearing, expected, seconds=1.5):
        start = len(maps)
        end = time.monotonic()+seconds
        while time.monotonic() < end:
            stamp = node.get_clock().now().to_msg()
            marks.publish(xyz_cloud(np.asarray(hit).reshape(-1, 3), 'base_link', stamp))
            rays.publish(xyz_cloud(np.asarray(clearing).reshape(-1, 3), 'd435_depth_frame', stamp))
            pump(.1)
        assert len(maps) > start, (tmp_path/'costmap.log').read_text()
        msg = maps[-1]
        x = int((1.-msg.info.origin.position.x)/msg.info.resolution)
        y = int((0.-msg.info.origin.position.y)/msg.info.resolution)
        cells = np.asarray(msg.data).reshape(msg.info.height, msg.info.width)
        # Float transport can put an exact cell boundary in an adjacent cell.
        occupied = bool(np.any(cells[y-1:y+2, x-1:x+2] == 100))
        assert occupied == expected, cells[y-1:y+2, x-1:x+2]

    try:
        end = time.monotonic()+5.
        service = None
        while time.monotonic() < end and service is None:
            pump(.1)
            service = next((name for name, types in node.get_service_names_and_types()
                            if name.endswith('/change_state') and 'lifecycle_msgs/srv/ChangeState' in types), None)
        assert service, (tmp_path/'costmap.log').read_text()
        client = node.create_client(ChangeState, service)
        assert client.wait_for_service(timeout_sec=2.)
        transition(client, 1)
        transition(client, 3)
        hit = [[1., 0., .6]]
        observe(hit, [[1., 0., -.4]], True)
        # Looking at the floor elsewhere does not erase the elevated obstacle.
        observe([], [[1., .5, -1.]], True)
        # Observing the background through its old voxel clears it.
        observe([], [[2., 0., -.8]], False)
    finally:
        node.destroy_node()
        rclpy.shutdown()
        process.terminate()
        try:
            process.wait(timeout=5.)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(timeout=2.)
        log.close()
