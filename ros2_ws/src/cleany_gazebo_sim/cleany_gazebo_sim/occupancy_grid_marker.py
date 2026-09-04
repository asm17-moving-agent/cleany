from __future__ import annotations

from dataclasses import dataclass
import math

import rclpy
from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker


@dataclass(frozen=True)
class GridGeometry:
    width: int
    height: int
    resolution: float
    origin_x: float
    origin_y: float
    origin_yaw: float


def occupied_cell_centers(
    data: list[int] | tuple[int, ...],
    geometry: GridGeometry,
    occupied_threshold: int,
) -> tuple[tuple[float, float], ...]:
    """Return occupied-cell centers expressed in the map frame."""
    if geometry.width <= 0 or geometry.height <= 0:
        raise ValueError('grid dimensions must be positive')
    if geometry.resolution <= 0.0:
        raise ValueError('grid resolution must be positive')
    if len(data) != geometry.width * geometry.height:
        raise ValueError('occupancy data length does not match dimensions')
    if not 0 <= occupied_threshold <= 100:
        raise ValueError('occupied threshold must be between 0 and 100')

    cos_yaw = math.cos(geometry.origin_yaw)
    sin_yaw = math.sin(geometry.origin_yaw)
    centers: list[tuple[float, float]] = []
    for index, occupancy in enumerate(data):
        if occupancy < occupied_threshold:
            continue
        cell_x = (index % geometry.width + 0.5) * geometry.resolution
        cell_y = (index // geometry.width + 0.5) * geometry.resolution
        centers.append(
            (
                geometry.origin_x + cos_yaw * cell_x - sin_yaw * cell_y,
                geometry.origin_y + sin_yaw * cell_x + cos_yaw * cell_y,
            )
        )
    return tuple(centers)


class OccupancyGridMarker(Node):
    def __init__(self) -> None:
        super().__init__('occupancy_grid_marker')
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('marker_topic', '/slam_map_marker')
        self.declare_parameter('occupied_threshold', 65)
        self._occupied_threshold = int(
            self.get_parameter('occupied_threshold').value
        )
        latched_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        marker_topic = str(self.get_parameter('marker_topic').value)
        map_topic = str(self.get_parameter('map_topic').value)
        self._publisher = self.create_publisher(
            Marker, marker_topic, latched_qos
        )
        self.create_subscription(
            OccupancyGrid, map_topic, self._on_map, latched_qos
        )
        self.get_logger().info(
            f'Rendering occupied cells from {map_topic} on {marker_topic}'
        )

    def _on_map(self, message: OccupancyGrid) -> None:
        orientation = message.info.origin.orientation
        yaw = math.atan2(
            2.0
            * (
                orientation.w * orientation.z
                + orientation.x * orientation.y
            ),
            1.0
            - 2.0
            * (
                orientation.y * orientation.y
                + orientation.z * orientation.z
            ),
        )
        geometry = GridGeometry(
            width=message.info.width,
            height=message.info.height,
            resolution=message.info.resolution,
            origin_x=message.info.origin.position.x,
            origin_y=message.info.origin.position.y,
            origin_yaw=yaw,
        )
        centers = occupied_cell_centers(
            list(message.data), geometry, self._occupied_threshold
        )

        marker = Marker()
        marker.header = message.header
        marker.ns = 'slam_occupied_cells'
        marker.id = 0
        marker.type = Marker.CUBE_LIST
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = geometry.resolution
        marker.scale.y = geometry.resolution
        marker.scale.z = 0.025
        marker.color = ColorRGBA(r=0.05, g=0.12, b=0.24, a=1.0)
        marker.points = [Point(x=x, y=y, z=0.0) for x, y in centers]
        self._publisher.publish(marker)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = OccupancyGridMarker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
