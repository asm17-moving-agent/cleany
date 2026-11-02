"""ROS ports for guarded phone control and serialized mapping requests."""

import threading

from rclpy.qos import (
    QoSProfile,
    DurabilityPolicy,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from cleany_base_interfaces.msg import WheelState
from cleany_base_interfaces.srv import SetBaseEnabled
from cleany_interfaces.msg import TeleopStatus, MappingStatus
from cleany_interfaces.srv import MappingControl
from geometry_msgs.msg import Twist
from std_srvs.srv import SetBool

from .mapping_status import MappingStatusCache

from .driver_transport import DriverControl, DriverTransport, Geometry


class MappingBridge:
    def __init__(self, node, timeout_s=1.0):
        self.node = node
        self.lock = threading.RLock()
        self.cache = MappingStatusCache(timeout_s)
        self.client = node.create_client(MappingControl, "mapping/control")
        node.create_subscription(
            MappingStatus,
            "mapping/status",
            self.observe,
            QoSProfile(
                depth=1,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                reliability=ReliabilityPolicy.RELIABLE,
            ),
        )

    def observe(self, message):
        with self.lock:
            self.cache.observe(
                {
                    field: getattr(message, field)
                    for field in message.get_fields_and_field_types()
                }
            )

    def status(self):
        with self.lock:
            return self.cache.status()

    def request_reset(self, request_id):
        if not isinstance(request_id, str) or not 1 <= len(request_id) <= 80:
            raise ValueError("잘못된 초기화 요청 번호입니다.")
        if not self.client.service_is_ready():
            raise RuntimeError("매핑 관리자가 연결되지 않았습니다.")
        future = self.client.call_async(
            MappingControl.Request(request_id=request_id, operation="reset")
        )
        done = threading.Event()
        future.add_done_callback(lambda _: done.set())
        if not done.wait(2):
            raise TimeoutError(
                "초기화 접수 응답 지연. 같은 요청 번호로 상태를 다시 확인하세요."
            )
        response = future.result()
        if not response.accepted:
            raise RuntimeError(response.reason)
        return {"operation_id": response.operation_id}


def bind(node, profile, lock, *, event_sink=None, mapping_status_timeout_s=1.0):
    """Create no WheelCommand publisher: only the base driver owns that topic."""
    geo = profile["geometry"]
    geometry = Geometry(
        geo["wheel_radius_m"], geo["wheelbase_m"], geo["wheel_separation_m"]
    )
    velocity = node.create_publisher(Twist, "cmd_vel", 10)
    permission = node.create_client(SetBaseEnabled, "base/set_enabled")

    def request(client, sequence, enabled):
        return permission.call_async(
            SetBaseEnabled.Request(
                client_id=client, request_sequence=sequence, enabled=enabled
            )
        )

    def send(axes):
        message = Twist()
        message.linear.x, message.linear.y, message.angular.z = axes
        velocity.publish(message)

    control = DriverControl(
        DriverTransport(request, send, geometry), event_sink=event_sink
    )

    def feedback(message):
        with lock:
            control.feedback(message)

    node.create_subscription(
        WheelState, "base/wheel_state", feedback, qos_profile_sensor_data
    )

    def pause(request, response):
        with lock:
            control.paused = request.data
            if request.data:
                control.stop("지도 초기화 중 · 조종 입력 차단", source="maintenance")
        response.success = True
        response.message = "Permission updated; STOP requires fresh MCU acknowledgement"
        return response

    node.create_service(SetBool, "teleop/pause", pause)
    status_pub = node.create_publisher(TeleopStatus, "teleop/status", 10)

    def tick_status():
        with lock:
            control.publisher_count = node.count_publishers("base/wheel_command") + max(
                0, node.count_publishers("cmd_vel") - 1
            )
            control.subscriber_count = (
                min(
                    node.count_subscribers("base/wheel_command"),
                    node.count_subscribers("cmd_vel"),
                )
                if permission.service_is_ready()
                else 0
            )
            state = control.status()
            status_pub.publish(
                TeleopStatus(
                    **{
                        key: (state[key] or 0) if key == "boot_id" else state[key]
                        for key in (
                            "ready",
                            "paused",
                            "connected",
                            "stop_confirmed",
                            "enabled",
                            "boot_id",
                        )
                    }
                )
            )

    return control, tick_status, MappingBridge(node, mapping_status_timeout_s)
