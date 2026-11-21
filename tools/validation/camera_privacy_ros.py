#!/usr/bin/env python3
"""Exercise the privacy node and preview relay on an isolated ROS domain using a labeled image."""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import time
from urllib.error import HTTPError
from urllib.parse import urlencode
from urllib.request import urlopen

import cv2
import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

from cleany_perception.privacy import FILL
from cleany_perception.rgbd_snapshot import rgb_array

RAW = '/privacy_fixture/raw'
REDACTED = '/camera/head/color/image_redacted'


def request(origin, topic):
    url = origin + '/api/camera?' + urlencode({'topic': topic})
    try:
        with urlopen(url, timeout=3) as reply:
            return reply.status, reply.headers, reply.read()
    except HTTPError as exc:
        return exc.code, exc.headers, exc.read()


def run(args):
    pixels = cv2.imread(args.fixture)
    if pixels is None or pixels.shape[:2] != (480, 640):
        raise ValueError('Expected the labeled 640x480 card fixture')
    rclpy.init()
    node = rclpy.create_node('camera_privacy_replay_probe')
    pub = node.create_publisher(Image, RAW, qos_profile_sensor_data)
    latencies = []
    stamps = set()
    def received(msg):
        assert msg.header.frame_id == 'privacy_fixture_optical'
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        assert stamp in stamps, 'Output frame was not correlated with its own source'
        out = rgb_array(msg)[:, :, ::-1]
        assert np.all(out[310:318, 385:455] == FILL), 'Card text exposed on ROS'
        assert np.array_equal(out[248:252, 245:250], pixels[248:252, 245:250]), 'Grasp target lost'
        latencies.append(1000 * (node.get_clock().now().nanoseconds * 1e-9 - stamp))
    sub = node.create_subscription(Image, REDACTED, received, qos_profile_sensor_data)
    msg = Image(height=480, width=640, encoding='bgr8', step=1920, data=pixels.tobytes())
    msg.header.frame_id = 'privacy_fixture_optical'
    try:
        start = time.monotonic()
        while time.monotonic() - start < args.seconds:
            msg.header.stamp = node.get_clock().now().to_msg()
            stamps.add(msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)
            pub.publish(msg)
            rclpy.spin_once(node, timeout_sec=.035)
            request(args.origin, REDACTED)
            time.sleep(.035)
        status, headers, data = request(args.origin, REDACTED)
        assert len(latencies) >= 8 and status == 200, (len(latencies), status)
        assert headers['X-Camera-Topic'] == REDACTED
        jpeg = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
        assert jpeg is not None
        assert np.max(np.abs(jpeg[310:318, 385:455].astype(int)-FILL)) < 12
        assert request(args.origin, RAW)[0] == 400
        # Keep requesting while only stale or unsupported inputs are offered.
        for mode in ('stale', 'unsupported'):
            msg.encoding = 'bgr8' if mode == 'stale' else '16UC1'
            start = time.monotonic()
            while time.monotonic() - start < 3.:
                msg.header.stamp = node.get_clock().now().to_msg()
                if mode == 'stale':
                    msg.header.stamp.sec -= 10
                pub.publish(msg)
                request(args.origin, REDACTED)
                time.sleep(.1)
            assert request(args.origin, REDACTED)[0] == 202, mode
        print(json.dumps(dict(ros_frames=len(latencies), seconds=args.seconds, http_status=status,
            stale_http=202, rejected_encoding_http=202, raw_http=400, median_ms=float(np.median(latencies)),
            p95_ms=float(np.percentile(latencies,95)), max_ms=max(latencies)), indent=2))
    finally:
        node.destroy_subscription(sub)
        node.destroy_node()
        rclpy.shutdown()


def capture_live(path: Path, seconds: float) -> None:
    """Observe the validation publisher without issuing any camera or motion command."""
    rclpy.init()
    node = rclpy.create_node('privacy_live_evidence')
    frames = []
    def sample(msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        now = node.get_clock().now().nanoseconds * 1e-9
        if not 0 <= now - stamp <= 1.1:
            return
        if not frames:
            image = rgb_array(msg)[:, :, ::-1]
            if not cv2.imwrite(str(path), image):
                raise RuntimeError('Cannot write redacted live frame')
        frames.append({'stamp': stamp, 'age_ms': (now - stamp)*1000,
                       'frame_id': msg.header.frame_id, 'width': msg.width, 'height': msg.height})
    node.create_subscription(Image, '/privacy_validation/head_redacted', sample, qos_profile_sensor_data)
    try:
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.2)
        if not frames:
            raise RuntimeError('No fresh redacted live frames')
        print(json.dumps({'frames': frames}, indent=2))
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    mode = p.add_mutually_exclusive_group(required=True)
    mode.add_argument('--fixture')
    mode.add_argument('--capture-live', type=Path)
    p.add_argument('--origin', default='http://127.0.0.1:18769')
    p.add_argument('--seconds', type=float, default=18.)
    args = p.parse_args()
    if args.capture_live:
        capture_live(args.capture_live, args.seconds)
    else:
        run(args)
