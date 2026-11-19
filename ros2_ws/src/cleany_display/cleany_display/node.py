"""로컬 얼굴 화면에 ROS 구독 상태를 전달합니다."""

import argparse
import json
from math import hypot
from pathlib import Path
import threading
import time

from ament_index_python.packages import get_package_share_directory
from cleany_interfaces.msg import ManipulationExecutionRecord, MissionResult, MissionStatus
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import BatteryState
from std_msgs.msg import String

from .server import StateStore, make_server
from .state import DisplayState

# 화면에서 쓰지 않는 중첩 관측과 보고서까지 변환하지 않습니다.
DISPLAY_FIELDS = {
    'mission': ('mission_id', 'state', 'bt_stage'),
    'result': ('mission_id', 'outcome', 'needs_human_review', 'completed_tasks'),
    'manipulation': ('updated_at_ns', 'human_confirmation_required', 'record_state',
                     'substage', 'has_result', 'status'),
    'battery': ('percentage', 'power_supply_status'),
}


class DisplayNode(Node):
    """로봇 상태를 관찰하고 loopback HTTP로 얼굴 화면을 제공합니다.

    Subscriptions:
        /mission/status (cleany_interfaces/msg/MissionStatus): Mission FSM 상태.
        /mission/result (cleany_interfaces/msg/MissionResult): 최종 미션 보고.
        /manipulation/execution_events (cleany_interfaces/msg/ManipulationExecutionRecord):
            수거 진행 상태. 위 세 토픽은 reliable, transient-local QoS를 사용합니다.
        /wheel/odom (nav_msgs/msg/Odometry): 측정 평면 속도. sensor QoS를 사용합니다.
        /battery_state (sensor_msgs/msg/BatteryState): 충전 상태. sensor QoS를 사용합니다.
        /cleany/display/state (std_msgs/msg/String): 화면만 바꾸는 JSON 요청.
            volatile sensor QoS이며 모든 토픽 이름은 설정 기본값입니다.

    Notes:
        로봇 명령, 서비스 호출이나 action goal은 보내지 않습니다. 이벤트 기반 상태는
        발행자가 존재하는 동안 유지하고 센서 데이터에는 만료 시간을 적용합니다.
    """

    def __init__(self, config: dict, store: StateStore) -> None:
        super().__init__('cleany_display')
        self.config, self.store = config, store
        self.state = DisplayState(config)
        self.available_sources: set[str] = set()
        self.next_graph_check = 0.0
        durable = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        types = {'mission': MissionStatus, 'result': MissionResult,
                 'manipulation': ManipulationExecutionRecord, 'odometry': Odometry,
                 'battery': BatteryState, 'display': String}
        self.owned_subscriptions = []
        for source, topic in config['topics'].items():
            if not topic:
                continue
            qos = durable if source in ('mission', 'result', 'manipulation') else qos_profile_sensor_data
            self.owned_subscriptions.append(self.create_subscription(
                types[source], topic, lambda msg, key=source: self.receive(key, msg), qos))
        self.create_timer(0.2, self.update)

    def receive(self, source: str, msg: object) -> None:
        now = time.monotonic()
        if source == 'display':
            try:
                data = json.loads(msg.data)
                valid = isinstance(data, dict) and self.state.override(data, now)
            except (ValueError, TypeError):
                valid = False
            if not valid:
                self.get_logger().warning('Ignored invalid display override', throttle_duration_sec=5)
        elif source == 'odometry':
            self.state.ingest(source, {'linear': hypot(msg.twist.twist.linear.x, msg.twist.twist.linear.y),
                                       'angular': msg.twist.twist.angular.z}, now)
        else:
            data = {field: getattr(msg, field) for field in DISPLAY_FIELDS[source]}
            self.state.ingest(source, data, now)
        self.update()

    def update(self) -> None:
        now = time.monotonic()
        if now >= self.next_graph_check:
            self.available_sources = {key for key, topic in self.config['topics'].items()
                               if topic and self.count_publishers(topic)}
            self.next_graph_check = now + 1.0
        value = self.state.snapshot(now, self.available_sources, time.time())
        value['poll_ms'] = self.config['poll_ms']
        value['rendering'] = self.config.get('rendering', {})
        self.store.set(value)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument('--config')
    options, ros_args = parser.parse_known_args()
    share = Path(get_package_share_directory('cleany_display'))
    config = json.loads(Path(options.config or share / 'config/display.json').read_text())
    store = StateStore()
    server = make_server(config['host'], int(config['port']), share / 'web', store)
    rclpy.init(args=ros_args)
    node = DisplayNode(config, store)
    worker = threading.Thread(target=server.serve_forever, daemon=True)
    worker.start()
    node.get_logger().info(f"Display listening on http://{config['host']}:{config['port']}")
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        server.shutdown()
        server.server_close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
