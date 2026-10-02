"""Local-only ROS event to BehaviorTree Viewer monitor bridge."""

from dataclasses import fields
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from cleany_interfaces.msg import ManipulationExecutionRecord
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
import zmq

from .manipulation.monitor import MonitorProjection, Progress


class ManipulationMonitorNode(Node):
    def __init__(self, **kwargs) -> None:
        super().__init__('manipulation_monitor', **kwargs)
        preview = Path(get_package_share_directory('cleany_skill_executor')) / 'docs/groot2_table_cleanup.xml'
        self.declare_parameter('tree_xml', str(preview))
        self.declare_parameter('monitor_port', 1666)
        self.declare_parameter('event_topic', 'manipulation/execution_events')
        self.projection = MonitorProjection(self.get_parameter('tree_xml').value)
        port = self.get_parameter('monitor_port').value
        if not 1 <= port <= 65535:
            raise ValueError('monitor_port must be between 1 and 65535')
        self.context_zmq = zmq.Context()
        self.socket = self.context_zmq.socket(zmq.REP)
        self.socket.setsockopt(zmq.LINGER, 0)
        try:
            self.socket.bind(f'tcp://127.0.0.1:{port}')
        except Exception:
            self.socket.close()
            self.context_zmq.term()
            raise
        self.subscription = self.create_subscription(
            ManipulationExecutionRecord, self.get_parameter('event_topic').value,
            self._event, QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE,
                                   durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.timer = self.create_timer(0.02, self._poll)
        self.get_logger().info(f'Local mock monitor ready: 127.0.0.1:{port}')

    def _event(self, message: ManipulationExecutionRecord) -> None:
        event = Progress(**{field.name: getattr(message, field.name) for field in fields(Progress)})
        if self.projection.update(event):
            self.get_logger().info(f'{event.execution_id} {event.stage} {event.status}')

    def _poll(self) -> None:
        for _ in range(10):
            try:
                request = self.socket.recv_multipart(flags=zmq.NOBLOCK)
            except zmq.Again:
                return
            self.socket.send_multipart(self.projection.reply(request))

    def destroy_node(self):
        self.timer.cancel()
        self.socket.close()
        self.context_zmq.term()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = ManipulationMonitorNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
