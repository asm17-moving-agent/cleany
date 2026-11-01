"""ROS mapping session endpoint; process and save operations run off the executor."""

from pathlib import Path
import signal
from concurrent.futures import ThreadPoolExecutor
import threading
import time
import yaml

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.signals import SignalHandlerOptions
from rclpy.qos import (
    QoSProfile,
    DurabilityPolicy,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan
from std_srvs.srv import SetBool, Trigger
from cartographer_ros_msgs.srv import WriteState
from tf2_ros import Buffer, TransformListener
from cleany_interfaces.msg import MappingStatus, TeleopStatus
from cleany_interfaces.srv import MappingControl, RecordingControl

from .mapping_backend import MappingBackend
from .mapping_session import SessionController, Busy
from .mapping_storage import planar_origin, save_grid


class MappingNode(Node):
    """Expose asynchronous mapping operations and fresh sensor/session state.

    Subscriptions:
        scan (LaserScan), odom (Odometry), map (OccupancyGrid), and
        teleop/status (TeleopStatus): Readiness and final-save inputs.
    Publishers:
        mapping/status (MappingStatus): Reliable transient-local session state.
    Services:
        mapping/control (MappingControl): Accept reset/stop operations.
    Notes:
        Lifecycle RPCs run in the session worker; the executor remains available
        for acknowledgements. The bringup runtime owns permanent hardware.
    """

    def __init__(self):
        super().__init__("mapping_session")
        self.declare_parameter("profile", "")
        self.declare_parameter("output_root", "~/.local/state/cleany/mapping")
        profile_path = Path(self.get_parameter("profile").value).expanduser()
        profile = yaml.safe_load(profile_path.read_text())
        self.backend = MappingBackend(
            self,
            profile,
            profile_path,
            Path(self.get_parameter("output_root").value).expanduser(),
        )
        self.pause_client = self.create_client(SetBool, "teleop/pause")
        self.odom_reset = self.create_client(Trigger, "wheel_odometry/reset")
        self.write_client = self.create_client(WriteState, "write_state")
        self.logger_clients = {
            operation: self.create_client(
                RecordingControl, "/cleany_logger/" + operation
            )
            for operation in ("get_config", "configure", "start", "stop")
        }
        self.tf_buffer = Buffer()
        self.listener = TransformListener(self.tf_buffer, self)
        self.controller = SessionController(self.backend, self.backend.output_root)
        qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.status = self.create_publisher(MappingStatus, "mapping/status", qos)
        self.create_subscription(
            LaserScan,
            "scan",
            lambda msg: self.receive("scan", msg),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Odometry, "odom", lambda msg: self.receive("odom", msg), 10
        )
        self.create_subscription(
            TeleopStatus, "teleop/status", lambda msg: self.receive("teleop", msg), 10
        )
        self.create_subscription(OccupancyGrid, "map", self.map_received, qos)
        self.create_service(MappingControl, "mapping/control", self.control)
        self.create_timer(0.2, self.publish_status)
        self.create_timer(20.0, self.backend.periodic_save)
        self.last_snapshot = 0.0
        self.storage = ThreadPoolExecutor(
            max_workers=1, thread_name_prefix="mapping-storage"
        )
        self.snapshot_future = None

    def receive(self, name, message):
        with self.backend.lock:
            setattr(self.backend, name, message)
            setattr(self.backend, name + "_at", time.monotonic())

    def map_received(self, message):
        stamp = message.header.stamp.sec * 10**9 + message.header.stamp.nanosec
        if (
            not message.info.width
            or not message.info.height
            or stamp < getattr(self.backend, "started_ros", 0)
        ):
            return
        self.receive("grid", message)
        if (
            self.backend.output
            and not self.backend.saving_final
            and time.monotonic() - self.last_snapshot >= 5
            and (self.snapshot_future is None or self.snapshot_future.done())
        ):
            output = self.backend.output

            def write():
                try:
                    with self.backend.storage_lock:
                        save_grid(
                            output,
                            message.info.width,
                            message.info.height,
                            message.info.resolution,
                            planar_origin(message.info.origin),
                            message.data,
                        )
                except (ValueError, OSError) as error:
                    self.backend.save_error = str(error)

            self.snapshot_future = self.storage.submit(write)
            self.last_snapshot = time.monotonic()

    def control(self, request, response):
        try:
            result = self.controller.request_operation(
                request.operation, request.request_id
            )
            response.accepted = True
            response.operation_id = result["operation_id"]
        except (ValueError, Busy) as error:
            response.reason = str(error)
        return response

    def publish_status(self):
        snapshot = self.controller.snapshot()
        if self.backend.save_error and snapshot["phase"] == "running":
            snapshot["message"] += " · 주기 저장 확인 필요: " + self.backend.save_error
        snapshot["stop_confirmed"] = snapshot["stop_confirmed"] is True
        for name in ("operation_id", "request_id", "output", "previous_output"):
            snapshot[name] = snapshot[name] or ""
        self.status.publish(MappingStatus(**snapshot))


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = MappingNode()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    node.controller.start()
    closed = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: closed.set())
    signal.signal(signal.SIGTERM, lambda *_: closed.set())
    try:
        closed.wait()
    finally:
        try:
            node.controller.close()
        finally:
            node.storage.shutdown(wait=True)
            executor.shutdown(timeout_sec=2)
            thread.join(timeout=2)
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
