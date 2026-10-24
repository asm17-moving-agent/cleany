"""Gazebo-only TF for the measured pan of the frozen CAD navigation model."""
from __future__ import annotations

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster

from cleany_navigation.urdf_geometry import rotation, rpy


def quaternion(matrix: np.ndarray) -> tuple[float, float, float, float]:
    cr, cp, cy = np.cos(np.array(rpy(matrix))/2)
    sr, sp, sy = np.sin(np.array(rpy(matrix))/2)
    return (sr*cp*cy-cr*sp*sy, cr*sp*cy+sr*cp*sy,
            cr*cp*sy-sr*sp*cy, cr*cp*cy+sr*sp*sy)


class GazeboPanTfPublisher(Node):
    def __init__(self) -> None:
        super().__init__('gazebo_pan_tf_publisher')
        for name, value in dict(translation=[0., 0., 0.], rotation_rpy=[0., 0., 0.],
                               pan_pivot=[0., 0., 0.]).items():
            self.declare_parameter(name, value)
        self.camera = np.array(self.get_parameter('translation').value)
        self.pivot = np.array(self.get_parameter('pan_pivot').value)
        self.rotation = rotation(self.get_parameter('rotation_rpy').value)
        self.tf = TransformBroadcaster(self)
        self.create_subscription(JointState, 'joint_states', self.on_joint, qos_profile_sensor_data)

    def on_joint(self, msg: JointState) -> None:
        if 'head_pan_joint' not in msg.name:
            return
        index = msg.name.index('head_pan_joint')
        if index >= len(msg.position) or not np.isfinite(msg.position[index]):
            return
        turn = rotation([0., 0., msg.position[index]])
        xyz = self.pivot+turn@(self.camera-self.pivot)
        q = quaternion(turn@self.rotation)
        tf = TransformStamped()
        tf.header.stamp = msg.header.stamp
        tf.header.frame_id, tf.child_frame_id = 'base_link', 'head_camera_depth_frame'
        tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z = map(float, xyz)
        tf.transform.rotation.x, tf.transform.rotation.y, tf.transform.rotation.z, tf.transform.rotation.w = map(float, q)
        self.tf.sendTransform(tf)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = GazeboPanTfPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
