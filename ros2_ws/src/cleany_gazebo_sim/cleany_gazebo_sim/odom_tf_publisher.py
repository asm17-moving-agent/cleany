from __future__ import annotations

from copy import deepcopy

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


class GazeboOdomTfPublisher(Node):
    def __init__(self) -> None:
        super().__init__('gazebo_odom_tf_publisher')
        self.declare_parameter('odom_frame_id', 'odom')
        self.declare_parameter('base_frame_id', 'base_link')
        self._odom_frame_id = str(self.get_parameter('odom_frame_id').value)
        self._base_frame_id = str(self.get_parameter('base_frame_id').value)

        self._publisher = self.create_publisher(Odometry, 'odom', 10)
        self._tf_broadcaster = TransformBroadcaster(self)
        self.create_subscription(Odometry, 'gazebo_odom', self._on_odometry, 10)

    def _on_odometry(self, message: Odometry) -> None:
        output = deepcopy(message)
        output.header.frame_id = self._odom_frame_id
        output.child_frame_id = self._base_frame_id
        self._publisher.publish(output)

        transform = TransformStamped()
        transform.header = output.header
        transform.child_frame_id = self._base_frame_id
        transform.transform.translation.x = output.pose.pose.position.x
        transform.transform.translation.y = output.pose.pose.position.y
        transform.transform.translation.z = output.pose.pose.position.z
        transform.transform.rotation = output.pose.pose.orientation
        self._tf_broadcaster.sendTransform(transform)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = GazeboOdomTfPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        except KeyboardInterrupt:
            # ros2 launch can signal the process again while rclpy tears down.
            pass
