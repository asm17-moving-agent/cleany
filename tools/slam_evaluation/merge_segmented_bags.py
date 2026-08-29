#!/usr/bin/env python3
"""Merge restarted ROS bags while making simulation stamps monotonic."""

from __future__ import annotations

import argparse
from pathlib import Path
from typing import Any

import rosbag2_py
from builtin_interfaces.msg import Time
from rclpy.serialization import deserialize_message, serialize_message
from rosidl_runtime_py.utilities import get_message


NANOSECONDS = 1_000_000_000
SEGMENT_GAP_NS = 2_000_000


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output', type=Path)
    parser.add_argument('segments', nargs='+', type=Path)
    return parser.parse_args()


def open_reader(path: Path) -> rosbag2_py.SequentialReader:
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(path), storage_id='sqlite3'),
        rosbag2_py.ConverterOptions('cdr', 'cdr'),
    )
    return reader


def time_to_ns(stamp: Time) -> int:
    return int(stamp.sec) * NANOSECONDS + int(stamp.nanosec)


def set_time(stamp: Time, nanoseconds: int) -> None:
    stamp.sec, stamp.nanosec = divmod(nanoseconds, NANOSECONDS)


def shifted_message(
    topic: str, data: bytes, message_type: type[Any], shift_ns: int
) -> bytes:
    message = deserialize_message(data, message_type)
    if topic == '/clock':
        set_time(message.clock, time_to_ns(message.clock) + shift_ns)
    elif topic == '/tf_static':
        for transform in message.transforms:
            stamp = transform.header.stamp
            set_time(stamp, time_to_ns(stamp) + shift_ns)
    elif hasattr(message, 'header'):
        stamp = message.header.stamp
        set_time(stamp, time_to_ns(stamp) + shift_ns)
    else:
        return data
    return serialize_message(message)


def segment_clock_bounds(
    path: Path, clock_type: type[Any]
) -> tuple[int, int]:
    first: int | None = None
    last: int | None = None
    reader = open_reader(path)
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic != '/clock':
            continue
        value = time_to_ns(deserialize_message(data, clock_type).clock)
        first = value if first is None else first
        if last is not None and value < last:
            raise ValueError(f'non-monotonic /clock in {path}')
        last = value
    if first is None or last is None or last <= first:
        raise ValueError(f'missing or stationary /clock in {path}')
    return first, last


def main() -> None:
    options = parse_args()
    if options.output.exists():
        raise FileExistsError(f'refusing to overwrite {options.output}')
    for segment in options.segments:
        if not (segment / 'metadata.yaml').is_file():
            raise FileNotFoundError(f'incomplete segment bag: {segment}')

    first_reader = open_reader(options.segments[0])
    topics = first_reader.get_all_topics_and_types()
    topic_types = {topic.name: topic.type for topic in topics}
    message_types = {
        name: get_message(type_name) for name, type_name in topic_types.items()
    }
    clock_type = message_types['/clock']

    writer = rosbag2_py.SequentialWriter()
    writer.open(
        rosbag2_py.StorageOptions(
            uri=str(options.output), storage_id='sqlite3'
        ),
        rosbag2_py.ConverterOptions('cdr', 'cdr'),
    )
    for topic in topics:
        writer.create_topic(topic)

    next_clock_ns = 0
    next_storage_ns: int | None = None
    for index, segment in enumerate(options.segments):
        reader = open_reader(segment)
        current_topics = {
            topic.name: topic.type for topic in reader.get_all_topics_and_types()
        }
        if current_topics != topic_types:
            raise ValueError(f'topic contract differs in {segment}')
        first_clock_ns, last_clock_ns = segment_clock_bounds(
            segment, clock_type
        )
        shift_ns = next_clock_ns - first_clock_ns
        first_storage_ns: int | None = None
        last_output_storage_ns: int | None = None
        while reader.has_next():
            topic, data, storage_ns = reader.read_next()
            if first_storage_ns is None:
                first_storage_ns = storage_ns
                if next_storage_ns is None:
                    next_storage_ns = storage_ns
            assert next_storage_ns is not None
            output_storage_ns = next_storage_ns + storage_ns - first_storage_ns
            adjusted = shifted_message(
                topic, data, message_types[topic], shift_ns
            )
            writer.write(topic, adjusted, output_storage_ns)
            last_output_storage_ns = output_storage_ns
        if last_output_storage_ns is None:
            raise ValueError(f'empty segment bag: {segment}')
        next_storage_ns = last_output_storage_ns + 1
        next_clock_ns += last_clock_ns - first_clock_ns + SEGMENT_GAP_NS
        print(
            f'merged segment {index:02d}: '
            f'{(last_clock_ns - first_clock_ns) / NANOSECONDS:.3f} sim s'
        )


if __name__ == '__main__':
    main()
