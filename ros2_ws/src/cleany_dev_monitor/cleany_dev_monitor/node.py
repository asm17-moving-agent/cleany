"""Live ROS/web observer and client of the independent recording node."""
import threading
import rclpy
from cleany_logger.observer import ObservationNode
from .core import Hub
from .slam_reader import SlamReader
from .server import serve


class MonitorNode(ObservationNode):
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
