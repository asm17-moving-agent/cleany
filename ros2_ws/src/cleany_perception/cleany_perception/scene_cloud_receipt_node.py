"""필터링된 점군 수신을 coordinator action 콜백과 분리합니다."""
from __future__ import annotations

from copy import deepcopy

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header


def receipt_for_cloud(cloud: PointCloud2) -> Header | None:
    """실제 비어 있지 않은 완전한 점군의 촬영 시각만 그대로 전달합니다."""
    stamp = cloud.header.stamp.sec*1_000_000_000 + cloud.header.stamp.nanosec
    if (stamp <= 0 or not cloud.header.frame_id or cloud.width <= 0 or cloud.height <= 0
            or cloud.point_step <= 0 or cloud.row_step < cloud.width*cloud.point_step
            or len(cloud.data) < cloud.row_step*cloud.height):
        return None
    return deepcopy(cloud.header)


class SceneCloudReceiptNode(Node):
    """데이터를 복사하지 않고 완전한 필터링 점군 수신을 확인합니다.

    Subscriptions:
        /perception/scene_cloud_filtered (sensor_msgs/msg/PointCloud2): mapping 출력.

    Publishers:
        /perception/scene_cloud_receipt (std_msgs/msg/Header): 구조가 유효하고 비어
            있지 않은 점군의 촬영 stamp와 좌표계를 그대로 전달합니다.

    QoS:
        입력은 best_effort, depth 1이며 수신 확인 출력은 reliable, depth 1입니다.

    Notes:
        토픽 이름은 변경 가능한 기본값입니다. 수신 확인은 메시지 도착의 근거이며
        MoveIt이 계획 장면에 점군을 반영했다는 뜻은 아닙니다.
    """
    def __init__(self) -> None:
        super().__init__('scene_cloud_receipt')
        self.declare_parameter('filtered_cloud_topic', '/perception/scene_cloud_filtered')
        self.declare_parameter('receipt_topic', '/perception/scene_cloud_receipt')
        self._publisher = self.create_publisher(
            Header, str(self.get_parameter('receipt_topic').value),
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE))
        self.create_subscription(
            PointCloud2, str(self.get_parameter('filtered_cloud_topic').value),
            self._on_cloud, QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))

    def _on_cloud(self, cloud: PointCloud2) -> None:
        receipt = receipt_for_cloud(cloud)
        if receipt is not None:
            self._publisher.publish(receipt)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = SceneCloudReceiptNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RuntimeError:
        # Humble은 컨텍스트 종료 중 점군을 변환하는 take_message를 중단할 수 있습니다.
        # ROS가 활성인 동안의 런타임 실패는 숨기지 않습니다.
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
