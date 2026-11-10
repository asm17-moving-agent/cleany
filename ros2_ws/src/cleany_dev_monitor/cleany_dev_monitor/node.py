"""실시간 ROS·웹 관측과 독립 기록 노드의 client를 담당합니다."""
import threading
import rclpy
from cleany_logger.observer import ObservationNode
from .core import Hub
from .slam_reader import SlamReader
from .server import serve


class MonitorNode(ObservationNode):
    """실시간 관측을 웹에 제공하고 독립 Logger의 제어 client를 연결합니다.

    Subscriptions:
        센서 및 런타임 입력은 ObservationNode의 설정된 관측 소스를 따릅니다.

    Services:
        LoggerClient가 logger_namespace 기본값 /cleany_logger의 RecordingControl
            서비스를 호출합니다. 이 노드가 기록 서비스를 소유하지는 않습니다.

    Notes:
        record=False로 초기화해 로컬 기록 writer를 만들지 않습니다. 웹 서버는 별도
        worker에서 동작하며 노드 종료는 해당 worker를 멈춥니다. 독립 Logger의
        session 종료는 명시적인 기록 제어 요청으로 처리합니다.
    """

    def __init__(self, **kwargs):
        super().__init__(node_name="cleany_dev_monitor", hub_factory=Hub,
                         slam_factory=SlamReader, record=False,
                         extra_defaults={"logger_namespace": "/cleany_logger", "logger_timeout_s": 3.0}, **kwargs)
        from .logger_client import LoggerClient
        self.logger_client = LoggerClient(self, self.config)
        self.stop_event = threading.Event()
        self.worker = threading.Thread(target=serve, args=(self.hub, self.config, self.stop_event, self.logger_client), daemon=True)
        self.worker.start()

    def destroy_node(self):
        self.stop_event.set()
        self.worker.join(timeout=5)
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MonitorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
