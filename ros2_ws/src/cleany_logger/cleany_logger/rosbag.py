"""Standard rosbag2 CDR storage, owned exclusively by the recording worker."""

import json
import sqlite3
from pathlib import Path

from .recording import RawMessage


class RosbagWriter:
    """Load the ROS dependency only when raw recording is requested; never fall back."""

    def __init__(self, path: Path):
        import rosbag2_py

        self.api = rosbag2_py
        self.writer = rosbag2_py.SequentialWriter()
        self.topics: dict[str, str] = {}
        self.writer.open(
            rosbag2_py.StorageOptions(
                uri=str(path), storage_id="sqlite3", max_cache_size=0
            ),
            rosbag2_py.ConverterOptions("", ""),
        )

    def write(self, message: RawMessage) -> None:
        previous = self.topics.get(message.topic)
        if previous and previous != message.message_type:
            raise ValueError(
                f"ROS type changed on {message.topic}; start a new recording"
            )
        if not previous:
            # rosbag2 uses this profile when replaying transient-local context.
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
        writer, self.writer = self.writer, None
        if writer is not None:
            # Recent Humble has close(); older builds finalize in the destructor.
            if hasattr(writer, "close"):
                writer.close()
            del writer


def read_cdr(
    directory: Path, topic: str, timestamp_ns: int, occurrence: int = 0
) -> bytes:
    """Indexed lookup in our fixed sqlite3 rosbag2 backend, without loading a bag."""
    for path in sorted(directory.glob("*.db3")):
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
