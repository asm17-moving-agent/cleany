"""Live ROS/web observer and client of the independent recording node."""
import threading
import rclpy
from rclpy.clock import Clock, ClockType
from cleany_logger.observer import ObservationNode
from .camera import preview_supported
from .core import Hub
from .proposals import DEFAULTS as PROPOSAL_DEFAULTS
from .request import read_request
from .slam_reader import SlamReader
from .server import serve


class MonitorNode(ObservationNode):
    """Serve live observations and optional operator controls with independent logging.

    Notes:
        The shared observer owns ROS subscriptions. LoggerClient controls the
        separate logger; the monitor has no recording writer. Preparation-file
        progress is displayed locally before a manipulation Action exists.
    """

    def __init__(self, **kwargs):
        super().__init__(node_name="cleany_dev_monitor", hub_factory=Hub,
                         slam_factory=SlamReader, record=False,
                         extra_defaults={**PROPOSAL_DEFAULTS,
                             "logger_namespace": "/cleany_logger", "logger_timeout_s": 3.0,
                             "control_enabled": False,
                             "control_directory": "~/.local/state/cleany/dev-monitor/operator",
                             "control_repository_root": "", "control_session_seconds": 3600,
                             "manipulation_request_file": ""}, **kwargs)
        from .logger_client import LoggerClient
        self.logger_client = LoggerClient(self, self.config)
        self.create_timer(1.0, self.command_progress, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.stop_event = threading.Event()
        self.worker = threading.Thread(target=serve, args=(self.hub, self.config, self.stop_event, self.logger_client), daemon=True)
        self.worker.start()

    def camera_preview_supported(self, topic: str) -> bool:
        return preview_supported(topic)

    def command_progress(self):
        path = self.config['manipulation_request_file']
        if not path or self.config['manipulation_observation_mode'] != 'server':
            return
        try:
            data = read_request(path)
        except (OSError, ValueError, TypeError, KeyError):
            return
        self.hub.put('manipulation_request', 'current', data)

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
