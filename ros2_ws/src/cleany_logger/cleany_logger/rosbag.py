"""기록 worker만 독점 소유하는 표준 rosbag2 CDR 저장소입니다."""

import json
import sqlite3
from pathlib import Path

from .recording import RawMessage


class RosbagWriter:
    """원시 기록을 요청할 때만 ROS 의존성을 로드하며 다른 방식으로 대체하지 않습니다."""

    def __init__(self, path: Path, *, split_diagnostics: bool = True):
        import rosbag2_py

        self.api = rosbag2_py
        self.path = Path(path)
        self.split_diagnostics = split_diagnostics
        self.diagnostics = None
        self.message_types: dict[str, str] = {}
        self.writer = rosbag2_py.SequentialWriter()
        self.topics: dict[str, str] = {}
        self.writer.open(
            rosbag2_py.StorageOptions(
                uri=str(path), storage_id="sqlite3", max_cache_size=0,
                storage_preset_profile="resilient",
            ),
            rosbag2_py.ConverterOptions("", ""),
        )

    def write(self, message: RawMessage) -> None:
        previous_type = self.message_types.get(message.topic)
        if previous_type and previous_type != message.message_type:
            raise ValueError(f"ROS type changed on {message.topic}; start a new recording")
        self.message_types[message.topic] = message.message_type
        diagnostic = (
            message.message_type.startswith(('visualization_msgs/', 'cartographer_ros_msgs/'))
            or message.message_type == 'nav_msgs/msg/OccupancyGrid'
            or (message.category == 'slam' and message.message_type == 'sensor_msgs/msg/PointCloud2')
        )
        if self.split_diagnostics and diagnostic:
            if self.diagnostics is None:
                self.diagnostics = RosbagWriter(self.path / 'diagnostics', split_diagnostics=False)
            self.diagnostics.write(message)
            return
        previous = self.topics.get(message.topic)
        if previous and previous != message.message_type:
            raise ValueError(
                f"ROS type changed on {message.topic}; start a new recording"
            )
        if not previous:
            # rosbag2는 transient_local 컨텍스트를 재생할 때 이 프로파일을 사용합니다.
            qos = json.dumps(
                [
                    {
                        "history": 1,
                        "depth": 100,
                        "reliability": 1 if message.reliable else 2,
                        "durability": 1 if message.latched else 2,
                        "deadline": {"sec": 9223372036, "nsec": 854775807},
                        "lifespan": {"sec": 9223372036, "nsec": 854775807},
                        "liveliness": 1,
                        "liveliness_lease_duration": {
                            "sec": 9223372036,
                            "nsec": 854775807,
                        },
                        "avoid_ros_namespace_conventions": False,
                    }
                ]
            )
            self.writer.create_topic(
                self.api.TopicMetadata(
                    name=message.topic,
                    type=message.message_type,
                    serialization_format="cdr",
                    offered_qos_profiles=qos,
                )
            )
            self.topics[message.topic] = message.message_type
        self.writer.write(message.topic, message.payload, message.timestamp_ns)

    def close(self) -> None:
        diagnostics, self.diagnostics = self.diagnostics, None
        writer, self.writer = self.writer, None
        try:
            if diagnostics is not None:
                diagnostics.close()
        finally:
            if writer is not None:
                # 최근 Humble에는 close()가 있으며 이전 빌드는 소멸자에서 마무리합니다.
                if hasattr(writer, "close"):
                    writer.close()
                del writer


def read_cdr(
    directory: Path, topic: str, timestamp_ns: int, occurrence: int = 0
) -> bytes:
    """bag을 전체 로드하지 않고 고정 sqlite3 rosbag2 백엔드의 인덱스로 조회합니다."""
    for path in sorted(directory.rglob("*.db3")):
        with sqlite3.connect(f"file:{path}?mode=ro", uri=True) as db:
            row = db.execute(
                "SELECT data FROM messages WHERE timestamp=? AND topic_id IN "
                "(SELECT id FROM topics WHERE name=?) ORDER BY id LIMIT 1 OFFSET ?",
                (timestamp_ns, topic, occurrence),
            ).fetchone()
            if row is None:
                occurrence -= db.execute(
                    "SELECT count(*) FROM messages WHERE timestamp=? AND topic_id IN "
                    "(SELECT id FROM topics WHERE name=?)",
                    (timestamp_ns, topic),
                ).fetchone()[0]
        if row:
            return bytes(row[0])
    raise FileNotFoundError("Recorded rosbag2 message is unavailable")
