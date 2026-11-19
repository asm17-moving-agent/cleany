"""고정된 CAD Navigation 모델에서 관측한 pan 각도를 반영하는 Gazebo 전용 TF입니다."""
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
    """관측한 head pan 각도로 시뮬레이션 카메라 장착 변환을 발행합니다.

    Subscriptions:
        joint_states (sensor_msgs/msg/JointState): head_pan_joint 각도(rad).

    TF:
        관절 시각으로 base_link → head_camera_depth_frame을 발행합니다.
        설정된 장착 형상을 pan_pivot 주위로 회전합니다.

    QoS:
        관절 입력은 sensor_data를 사용합니다.

    Notes:
        pan 위치가 없거나 유한하지 않으면 무시합니다. robot_state_publisher 대신
        이 노드가 카메라 변환을 소유하는 구성에서만 사용합니다.
    """
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
