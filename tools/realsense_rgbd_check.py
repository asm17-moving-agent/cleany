#!/usr/bin/env python3
"""Validate a synchronized, color-aligned RealSense RGB-D stream."""

from __future__ import annotations

import argparse
from dataclasses import dataclass, field
import json
import time
from typing import Any, Sequence

import numpy as np


TOPIC_KINDS = ('color', 'color_info', 'depth', 'depth_info')


def stamp_nanoseconds(message: Any) -> int:
    return int(message.header.stamp.sec) * 1_000_000_000 + int(
        message.header.stamp.nanosec
    )


def valid_depth_ratio(message: Any, depth_scale_m: float) -> float:
    pixels = int(message.width) * int(message.height)
    if pixels <= 0 or len(message.data) == 0:
        raise ValueError('empty depth image')
    if message.encoding == '16UC1':
        if not 1e-6 <= depth_scale_m <= 0.1:
            raise ValueError(f'abnormal depth scale: {depth_scale_m}')
        values = np.frombuffer(message.data, dtype=np.uint16, count=pixels)
        valid = int(np.count_nonzero(values))
    elif message.encoding == '32FC1':
        values = np.frombuffer(message.data, dtype=np.float32, count=pixels)
        valid = int(np.count_nonzero(np.isfinite(values) & (values > 0)))
    else:
        raise ValueError(f'unsupported depth encoding: {message.encoding}')

    return valid / pixels


def validate_bundle(
    bundle: dict[str, Any],
    depth_scale_m: float,
) -> dict[str, Any]:
    color = bundle['color']
    color_info = bundle['color_info']
    depth = bundle['depth']
    depth_info = bundle['depth_info']

    dimensions = {
        (int(message.width), int(message.height))
        for message in (color, color_info, depth, depth_info)
    }
    if len(dimensions) != 1:
        raise ValueError(f'RGB-D dimensions differ: {sorted(dimensions)}')

    frames = {
        message.header.frame_id
        for message in (color, color_info, depth, depth_info)
    }
    if len(frames) != 1 or not next(iter(frames)):
        raise ValueError(f'RGB-D optical frames differ: {sorted(frames)}')

    if color.encoding not in ('rgb8', 'bgr8'):
        raise ValueError(f'unsupported color encoding: {color.encoding}')
    if depth.encoding not in ('16UC1', '32FC1'):
        raise ValueError(f'unsupported depth encoding: {depth.encoding}')

    color_k = tuple(float(value) for value in color_info.k)
    depth_k = tuple(float(value) for value in depth_info.k)
    if len(color_k) != 9 or len(depth_k) != 9:
        raise ValueError('CameraInfo intrinsic matrix must have nine values')
    if any(abs(left - right) > 1e-6 for left, right in zip(color_k, depth_k)):
        raise ValueError('aligned RGB-D intrinsics differ')
    if color_k[0] <= 0 or color_k[4] <= 0:
        raise ValueError('CameraInfo focal lengths must be positive')

    return {
        'width': int(color.width),
        'height': int(color.height),
        'frame_id': color.header.frame_id,
        'color_encoding': color.encoding,
        'depth_encoding': depth.encoding,
        'intrinsics': {
            'fx': color_k[0],
            'fy': color_k[4],
            'cx': color_k[2],
            'cy': color_k[5],
        },
        'valid_depth_ratio': valid_depth_ratio(depth, depth_scale_m),
    }


@dataclass
class RgbdMonitor:
    depth_scale_m: float
    maximum_pending_stamps: int = 64
    topic_counts: dict[str, int] = field(
        default_factory=lambda: {kind: 0 for kind in TOPIC_KINDS}
    )
    pending: dict[int, dict[str, Any]] = field(default_factory=dict)
    complete_count: int = 0
    invalid_count: int = 0
    errors: list[str] = field(default_factory=list)
    first_complete_monotonic: float | None = None
    last_complete_monotonic: float | None = None
    maximum_complete_gap_seconds: float = 0.0
    first_stamp_ns: int | None = None
    last_stamp_ns: int | None = None
    minimum_valid_depth_ratio: float = 1.0
    latest_details: dict[str, Any] | None = None

    def add(self, kind: str, message: Any, now: float | None = None) -> None:
        if kind not in TOPIC_KINDS:
            raise ValueError(f'unknown RGB-D topic kind: {kind}')
        self.topic_counts[kind] += 1
        stamp_ns = stamp_nanoseconds(message)
        bundle = self.pending.setdefault(stamp_ns, {})
        bundle[kind] = message
        if all(name in bundle for name in TOPIC_KINDS):
            observed_at = time.monotonic() if now is None else now
            try:
                details = validate_bundle(bundle, self.depth_scale_m)
            except ValueError as error:
                self.invalid_count += 1
                if len(self.errors) < 20:
                    self.errors.append(str(error))
            else:
                if self.last_complete_monotonic is not None:
                    gap = observed_at - self.last_complete_monotonic
                    self.maximum_complete_gap_seconds = max(
                        self.maximum_complete_gap_seconds,
                        gap,
                    )
                if self.first_complete_monotonic is None:
                    self.first_complete_monotonic = observed_at
                    self.first_stamp_ns = stamp_ns
                self.last_complete_monotonic = observed_at
                self.last_stamp_ns = stamp_ns
                self.complete_count += 1
                self.minimum_valid_depth_ratio = min(
                    self.minimum_valid_depth_ratio,
                    details['valid_depth_ratio'],
                )
                self.latest_details = details
            del self.pending[stamp_ns]

        while len(self.pending) > self.maximum_pending_stamps:
            del self.pending[min(self.pending)]

    def report(
        self,
        requested_duration_seconds: float,
        elapsed_seconds: float,
        minimum_rate_hz: float,
        maximum_gap_seconds: float,
        started_at_monotonic: float = 0.0,
    ) -> dict[str, Any]:
        stream_duration = 0.0
        if self.first_stamp_ns is not None and self.last_stamp_ns is not None:
            stream_duration = (self.last_stamp_ns - self.first_stamp_ns) / 1e9
        rate_hz = (
            (self.complete_count - 1) / stream_duration
            if self.complete_count > 1 and stream_duration > 0
            else 0.0
        )
        startup_delay = (
            self.first_complete_monotonic - started_at_monotonic
            if self.first_complete_monotonic is not None
            else elapsed_seconds
        )
        trailing_gap = (
            started_at_monotonic
            + elapsed_seconds
            - self.last_complete_monotonic
            if self.last_complete_monotonic is not None
            else elapsed_seconds
        )
        observed_maximum_gap = max(
            startup_delay,
            self.maximum_complete_gap_seconds,
            trailing_gap,
        )
        checks = {
            'all_topics_received': all(
                count > 0 for count in self.topic_counts.values()
            ),
            'exact_timestamp_bundles_received': self.complete_count > 0,
            'no_invalid_bundles': self.invalid_count == 0,
            'valid_depth_present': self.minimum_valid_depth_ratio > 0.0
            if self.complete_count
            else False,
            'minimum_rate_met': rate_hz >= minimum_rate_hz,
            'maximum_gap_met': observed_maximum_gap <= maximum_gap_seconds,
            'requested_duration_met': elapsed_seconds
            >= requested_duration_seconds * 0.99,
        }
        return {
            'requested_duration_seconds': requested_duration_seconds,
            'elapsed_seconds': elapsed_seconds,
            'topic_counts': self.topic_counts,
            'exact_bundle_count': self.complete_count,
            'invalid_bundle_count': self.invalid_count,
            'pending_stamp_count': len(self.pending),
            'stream_duration_seconds': stream_duration,
            'exact_bundle_rate_hz': rate_hz,
            'depth_scale_m': self.depth_scale_m,
            'startup_delay_seconds': startup_delay,
            'maximum_complete_gap_seconds': self.maximum_complete_gap_seconds,
            'trailing_gap_seconds': trailing_gap,
            'observed_maximum_gap_seconds': observed_maximum_gap,
            'minimum_valid_depth_ratio': self.minimum_valid_depth_ratio
            if self.complete_count
            else None,
            'latest_bundle': self.latest_details,
            'errors': self.errors,
            'checks': checks,
            'passed': all(checks.values()),
        }


def _parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--duration', type=float, default=300.0)
    parser.add_argument('--minimum-rate', type=float, default=5.0)
    parser.add_argument('--maximum-gap', type=float, default=2.0)
    parser.add_argument('--depth-scale', type=float, default=0.001)
    parser.add_argument('--output', type=str)
    parser.add_argument(
        '--color-image-topic',
        default='/camera/camera/color/image_raw',
    )
    parser.add_argument(
        '--color-info-topic',
        default='/camera/camera/color/camera_info',
    )
    parser.add_argument(
        '--depth-image-topic',
        default='/camera/camera/aligned_depth_to_color/image_raw',
    )
    parser.add_argument(
        '--depth-info-topic',
        default='/camera/camera/aligned_depth_to_color/camera_info',
    )
    options = parser.parse_args(argv)
    if options.duration <= 0:
        parser.error('--duration must be positive')
    return options


def main(argv: Sequence[str] | None = None) -> int:
    options = _parse_args(argv)
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import CameraInfo, Image

    rclpy.init()
    node = Node('realsense_rgbd_check')
    monitor = RgbdMonitor(depth_scale_m=options.depth_scale)
    subscriptions = (
        node.create_subscription(
            Image,
            options.color_image_topic,
            lambda message: monitor.add('color', message),
            qos_profile_sensor_data,
        ),
        node.create_subscription(
            CameraInfo,
            options.color_info_topic,
            lambda message: monitor.add('color_info', message),
            qos_profile_sensor_data,
        ),
        node.create_subscription(
            Image,
            options.depth_image_topic,
            lambda message: monitor.add('depth', message),
            qos_profile_sensor_data,
        ),
        node.create_subscription(
            CameraInfo,
            options.depth_info_topic,
            lambda message: monitor.add('depth_info', message),
            qos_profile_sensor_data,
        ),
    )
    del subscriptions

    started_at = time.monotonic()
    try:
        while time.monotonic() - started_at < options.duration:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    elapsed = time.monotonic() - started_at
    report = monitor.report(
        options.duration,
        elapsed,
        options.minimum_rate,
        options.maximum_gap,
        started_at,
    )
    rendered = json.dumps(report, indent=2) + '\n'
    if options.output:
        with open(options.output, 'w', encoding='utf-8') as output_file:
            output_file.write(rendered)
    else:
        print(rendered, end='')
    node.destroy_node()
    rclpy.shutdown()
    return 0 if report['passed'] else 2


if __name__ == '__main__':
    raise SystemExit(main())
