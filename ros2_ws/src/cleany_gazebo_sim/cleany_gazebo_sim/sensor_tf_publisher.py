from __future__ import annotations

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

from cleany_gazebo_sim.static_transform import StaticTransformSpec


class GazeboSensorTfPublisher(Node):
    """Gazebo 모델의 고정 LiDAR 및 IMU 장착 변환을 발행합니다.

    TF:
        parent_frame_id에서 lidar_frame_id와 imu_frame_id로의 정적 변환을 발행합니다.
        좌표계 이름, 위치(m), quaternion(xyzw)을 파라미터로 지정해야 합니다.
        이 클래스는 장착 위치 기본값을 제공하지 않습니다.

    Notes:
        발행 전에 변환 설정을 검증합니다. 카메라 pan은 GazeboPanTfPublisher가
        별도로 담당합니다.
    """
    def __init__(self) -> None:
        super().__init__('gazebo_sensor_tf_publisher')
        self.declare_parameter('parent_frame_id', Parameter.Type.STRING)
        sensor_names = ('lidar', 'imu')
        for sensor_name in sensor_names:
            self.declare_parameter(
                f'{sensor_name}_frame_id', Parameter.Type.STRING
            )
            self.declare_parameter(
                f'{sensor_name}_translation', Parameter.Type.DOUBLE_ARRAY
            )
            self.declare_parameter(
                f'{sensor_name}_rotation_xyzw', Parameter.Type.DOUBLE_ARRAY
            )

        parent_frame_id = self.get_parameter('parent_frame_id').value
        transforms = [
            self._read_transform(parent_frame_id, sensor_name)
            for sensor_name in sensor_names
        ]

        self._tf_broadcaster = StaticTransformBroadcaster(self)
        self._tf_broadcaster.sendTransform(
            [self._to_message(transform) for transform in transforms]
        )
        children = ', '.join(
            transform.child_frame_id for transform in transforms
        )
        self.get_logger().info(
            'Published static sensor frames from '
            f'{parent_frame_id}: {children}'
        )

    def _read_transform(
        self, parent_frame_id: str, sensor_name: str
    ) -> StaticTransformSpec:
        return StaticTransformSpec.from_values(
            parent_frame_id=parent_frame_id,
            child_frame_id=self.get_parameter(
                f'{sensor_name}_frame_id'
            ).value,
            translation=self.get_parameter(
                f'{sensor_name}_translation'
            ).value,
            rotation_xyzw=self.get_parameter(
                f'{sensor_name}_rotation_xyzw'
            ).value,
        )

    def _to_message(self, transform: StaticTransformSpec) -> TransformStamped:
        message = TransformStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = transform.parent_frame_id
        message.child_frame_id = transform.child_frame_id
        message.transform.translation.x = transform.translation[0]
        message.transform.translation.y = transform.translation[1]
        message.transform.translation.z = transform.translation[2]
        message.transform.rotation.x = transform.rotation_xyzw[0]
        message.transform.rotation.y = transform.rotation_xyzw[1]
        message.transform.rotation.z = transform.rotation_xyzw[2]
        message.transform.rotation.w = transform.rotation_xyzw[3]
        return message


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = GazeboSensorTfPublisher()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        except KeyboardInterrupt:
            pass
