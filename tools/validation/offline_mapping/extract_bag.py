"""Extract the recorded Floor 19 Cartographer inputs into one uncompressed ROS 2 bag."""

import argparse
import json
from collections import Counter
from pathlib import Path

import rosbag2_py as bag
import yaml
from rclpy.serialization import deserialize_message, serialize_message
from tf2_msgs.msg import TFMessage

TOPICS = {
    "/floor19/scan": "/scan",
    "/floor19/odom": "/odom",
    "/floor19/tf_static": "/tf_static",
    "/floor19/tf": "/tf",
}


def extract(source: Path, output: Path) -> None:
    if output.exists():
        raise FileExistsError(output)
    meta = yaml.safe_load((source / "metadata.yaml").read_text())["rosbag2_bagfile_information"]
    expected = {
        t["topic_metadata"]["name"]: t["message_count"]
        for t in meta["topics_with_message_count"]
        if t["topic_metadata"]["name"] in TOPICS
    }
    if set(expected) != set(TOPICS) or not all(expected.values()):
        raise ValueError(f"Missing recording input: {expected}")
    reader = (
        bag.SequentialCompressionReader() if meta["compression_format"] else bag.SequentialReader()
    )
    converter = bag.ConverterOptions("", "")
    reader.open(bag.StorageOptions(uri=str(source), storage_id="sqlite3"), converter)
    reader.set_filter(bag.StorageFilter(topics=list(TOPICS)))
    writer = bag.SequentialWriter()
    writer.open(bag.StorageOptions(uri=str(output), storage_id="sqlite3"), converter)
    for topic in reader.get_all_topics_and_types():
        if topic.name in TOPICS:
            topic.name = TOPICS[topic.name]
            writer.create_topic(topic)
    counts = Counter()
    odom_tf_count = 0
    while reader.has_next():
        topic, payload, stamp = reader.read_next()
        counts[topic] += 1
        if topic == "/floor19/tf":
            transforms = deserialize_message(payload, TFMessage).transforms
            odom_tf = [
                t
                for t in transforms
                if (t.header.frame_id, t.child_frame_id) == ("odom", "base_link")
            ]
            if not odom_tf:
                continue
            payload = serialize_message(TFMessage(transforms=odom_tf))
            odom_tf_count += len(odom_tf)
        writer.write(TOPICS[topic], payload, stamp)
    del writer
    if counts != expected or not odom_tf_count:
        raise RuntimeError(f"Extracted {counts}, expected {expected}")
    print(
        json.dumps({"source_messages_read": counts, "odom_tf_kept": odom_tf_count}, sort_keys=True)
    )


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    extract(args.source, args.output)
