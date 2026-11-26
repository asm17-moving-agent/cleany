"""관측 전송의 Backend 인증과 재연결을 소유합니다."""
import json

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String

from .observation_relay import ObservationRelay


class RobotObservationBridge(Node):
    """읽기 전용 관측 JSON을 인증된 WebSocket으로 전달합니다.

    Subscriptions:
        /telemetry/robot_observations (std_msgs/msg/String): input_topic 기본값,
            reliable volatile depth 100.
    Notes:
        ROS publisher, 서비스와 명령 경로가 없습니다. ACK가 없으면 같은 ID로
        재시도하며 입력이 만료되면 폐기합니다. 토큰은 재연결마다 파일에서 읽습니다.
    """

    def __init__(self, **kwargs) -> None:
        super().__init__("robot_observation_bridge", **kwargs)
        values = dict(input_topic="/telemetry/robot_observations",
                      url="ws://127.0.0.1:8080/api/robots/cleany-01/telemetry/ws",
                      device_token_file="", rate=10.0, input_timeout=3.0, battery_timeout=30.0,
                      supervisor_timeout=5.0, reconnect_initial=1.0, reconnect_max=30.0,
                      event_capacity=100)
        for key, value in values.items():
            self.declare_parameter(key, value)
        params = {key: self.get_parameter(key).value for key in values}
        topic = params.pop("input_topic")
        self.relay = ObservationRelay(**params)
        self.create_subscription(String, topic, self.receive, 100)
        self.relay.start()

    def receive(self, message: String) -> None:
        try:
            if len(message.data) > 262144:
                raise ValueError("oversized observation")
            self.relay.update(json.loads(message.data))
        except (ValueError, TypeError, KeyError):
            self.get_logger().warning("Invalid robot observation ignored", throttle_duration_sec=5)

    def destroy_node(self) -> bool:
        self.relay.stop()
        return super().destroy_node()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = RobotObservationBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
