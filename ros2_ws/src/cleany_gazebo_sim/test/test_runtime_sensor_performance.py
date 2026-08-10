from __future__ import annotations

import math
import os
from pathlib import Path
import signal
import struct
import subprocess
import tempfile
import time
from typing import Callable
from uuid import uuid4
import zlib

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
import pytest
import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Image, LaserScan

from cleany_gazebo_sim.world_generator import materialize_mecanum_wheel_world
from conftest import RuntimeTestOptions


CAMERA_TOPICS = (
    '/camera/head/color/image_raw',
    '/camera/head/depth/image_raw',
    '/camera/left_wrist/color/image_raw',
    '/camera/right_wrist/color/image_raw',
)
LIDAR_TOPIC = '/scan'
PROFILE_TOPICS = {
    'lidar_nav': (LIDAR_TOPIC,),
    'head_rgbd': CAMERA_TOPICS[:2],
    'left_wrist': (CAMERA_TOPICS[2],),
    'right_wrist': (CAMERA_TOPICS[3],),
    'all_cameras': CAMERA_TOPICS,
}
CAMERA_EXPECTATIONS = {
    '/camera/head/color/image_raw': (
        640, 480, 'rgb8', 'head_camera_rgb_optical_frame'
    ),
    '/camera/head/depth/image_raw': (
        640, 480, '32FC1', 'head_camera_depth_optical_frame'
    ),
    '/camera/left_wrist/color/image_raw': (
        640, 360, 'rgb8', 'left_wrist_rgb_optical_frame'
    ),
    '/camera/right_wrist/color/image_raw': (
        640, 360, 'rgb8', 'right_wrist_rgb_optical_frame'
    ),
}
VALIDATION_WALLS = (
    ('front', '3 0 1.5 0 0 0', '0.2 6 3', '0.8 0.1 0.1 1'),
    ('back', '-3 0 1.5 0 0 0', '0.2 6 3', '0.1 0.8 0.1 1'),
    ('left', '0 3 1.5 0 0 0', '6 0.2 3', '0.1 0.1 0.8 1'),
    ('right', '0 -3 1.5 0 0 0', '6 0.2 3', '0.8 0.8 0.1 1'),
)


class SensorMeasurements:
    def __init__(self) -> None:
        self.latest_sim_sec: float | None = None
        self.seen_topics: set[str] = set()
        self.message_counts = {
            topic: 0 for topic in (*CAMERA_TOPICS, LIDAR_TOPIC)
        }
        self.latest_images: dict[str, Image] = {}
        self.image_checksums = {topic: set() for topic in CAMERA_TOPICS}
        self.latest_scan: LaserScan | None = None
        self.measuring = False

    def clock_callback(self, message: Clock) -> None:
        self.latest_sim_sec = (
            float(message.clock.sec) + float(message.clock.nanosec) / 1e9
        )

    def image_callback(self, topic: str) -> Callable[[Image], None]:
        def callback(message: Image) -> None:
            self.seen_topics.add(topic)
            self.latest_images[topic] = message
            if len(self.image_checksums[topic]) < 2:
                self.image_checksums[topic].add(
                    zlib.adler32(message.data)
                )
            if self.measuring:
                self.message_counts[topic] += 1

        return callback

    def scan_callback(self, message: LaserScan) -> None:
        self.seen_topics.add(LIDAR_TOPIC)
        self.latest_scan = message
        if self.measuring:
            self.message_counts[LIDAR_TOPIC] += 1


def _launch_command(
    profile: str,
    sensor_profile: str,
    world_path: Path,
) -> list[str]:
    launch_file = (
        'gazebo_sim.launch.py'
        if profile == 'fortress'
        else 'gazebo_harmonic.launch.py'
    )
    return [
        'ros2',
        'launch',
        'cleany_gazebo_sim',
        launch_file,
        'headless:=true',
        f'sensor_profile:={sensor_profile}',
        f'world:={world_path}',
    ]


def _validation_scene_sdf() -> str:
    walls = []
    for name, pose, size, color in VALIDATION_WALLS:
        stripe_visuals = []
        for index, horizontal_position in enumerate(
            (-2.0, -1.5, -1.0, -0.5, 0.0, 0.5, 1.0, 1.5, 2.0)
        ):
            stripe_color = (
                '1 1 1 1' if index % 2 == 0 else '0.05 0.05 0.05 1'
            )
            if name == 'front':
                stripe_pose = f'-0.11 {horizontal_position} 0 0 0 0'
                stripe_size = '0.02 0.45 2.8'
            elif name == 'back':
                stripe_pose = f'0.11 {horizontal_position} 0 0 0 0'
                stripe_size = '0.02 0.45 2.8'
            elif name == 'left':
                stripe_pose = f'{horizontal_position} -0.11 0 0 0 0'
                stripe_size = '0.45 0.02 2.8'
            else:
                stripe_pose = f'{horizontal_position} 0.11 0 0 0 0'
                stripe_size = '0.45 0.02 2.8'
            stripe_visuals.append(
                f'''<visual name="stripe_{index}">
  <pose>{stripe_pose}</pose>
  <geometry><box><size>{stripe_size}</size></box></geometry>
  <material>
    <ambient>{stripe_color}</ambient>
    <diffuse>{stripe_color}</diffuse>
    <emissive>{stripe_color}</emissive>
  </material>
</visual>'''
            )
        walls.append(
            f'''<model name="runtime_sensor_test_{name}_wall">
  <static>true</static>
  <pose>{pose}</pose>
  <link name="wall">
    <collision name="collision">
      <geometry><box><size>{size}</size></box></geometry>
    </collision>
    <visual name="visual">
      <geometry><box><size>{size}</size></box></geometry>
      <material>
        <ambient>{color}</ambient>
        <diffuse>{color}</diffuse>
      </material>
    </visual>
    {''.join(stripe_visuals)}
  </link>
</model>'''
        )
    return '\n'.join(walls)


def _write_validation_world(profile: str, output_path: Path) -> None:
    package_share = Path(get_package_share_directory('cleany_gazebo_sim'))
    world_name = (
        'cleany_mecanum_prototype.sdf'
        if profile == 'fortress'
        else 'cleany_mecanum_harmonic.sdf'
    )
    materialized_world = materialize_mecanum_wheel_world(
        package_share / 'worlds' / world_name
    ).read_text(encoding='utf-8')
    if materialized_world.count('</world>') != 1:
        pytest.fail('runtime validation world must contain exactly one world')
    output_path.write_text(
        materialized_world.replace(
            '</world>', f'{_validation_scene_sdf()}\n  </world>'
        ),
        encoding='utf-8',
    )


def _assert_profile_environment(profile: str) -> None:
    expected_ros_distro = 'humble' if profile == 'fortress' else 'jazzy'
    actual_ros_distro = os.environ.get('ROS_DISTRO')
    if actual_ros_distro != expected_ros_distro:
        pytest.fail(
            f'{profile} requires ROS_DISTRO={expected_ros_distro}; '
            f'current value is {actual_ros_distro!r}'
        )


def _launch_log_tail(log_path: Path, line_count: int = 80) -> str:
    try:
        lines = log_path.read_text(
            encoding='utf-8', errors='replace'
        ).splitlines()
    except OSError as error:
        return f'<could not read launch log: {error}>'
    return '\n'.join(lines[-line_count:])


def _assert_launch_running(
    process: subprocess.Popen[bytes], log_path: Path
) -> None:
    return_code = process.poll()
    if return_code is not None:
        pytest.fail(
            f'Gazebo launch exited early with code {return_code}.\n'
            f'Launch log tail:\n{_launch_log_tail(log_path)}'
        )


def _spin_for(
    node: Node,
    seconds: float,
    process: subprocess.Popen[bytes],
    log_path: Path,
) -> None:
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        _assert_launch_running(process, log_path)
        remaining = deadline - time.monotonic()
        rclpy.spin_once(node, timeout_sec=min(0.1, remaining))


def _drive_for(
    node: Node,
    publisher: Publisher,
    seconds: float,
    process: subprocess.Popen[bytes],
    log_path: Path,
) -> None:
    command = Twist()
    command.linear.x = 0.1
    command.angular.z = 0.2
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        _assert_launch_running(process, log_path)
        publisher.publish(command)
        rclpy.spin_once(node, timeout_sec=0.1)
    publisher.publish(Twist())


def _wait_for_topics(
    node: Node,
    measurements: SensorMeasurements,
    expected_topics: set[str],
    timeout_sec: float,
    process: subprocess.Popen[bytes],
    log_path: Path,
) -> None:
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        _assert_launch_running(process, log_path)
        if (
            measurements.latest_sim_sec is not None
            and expected_topics <= measurements.seen_topics
        ):
            return
        rclpy.spin_once(node, timeout_sec=0.1)

    missing = sorted(expected_topics - measurements.seen_topics)
    clock_state = (
        'received' if measurements.latest_sim_sec is not None else 'missing'
    )
    pytest.fail(
        f'sensor startup timed out after {timeout_sec:.1f}s; '
        f'/clock={clock_state}, missing topics={missing}\n'
        f'Launch log tail:\n{_launch_log_tail(log_path)}'
    )


def _camera_validation_errors(
    measurements: SensorMeasurements,
    camera_topics: tuple[str, ...],
) -> list[str]:
    errors = []
    for topic in camera_topics:
        expectation = CAMERA_EXPECTATIONS[topic]
        width, height, encoding, frame_id = expectation
        message = measurements.latest_images[topic]
        prefix = f'{topic}:'
        if (message.width, message.height) != (width, height):
            errors.append(
                f'{prefix} dimensions {message.width}x{message.height} '
                f'!= {width}x{height}'
            )
        if message.encoding != encoding:
            errors.append(
                f'{prefix} encoding {message.encoding!r} != {encoding!r}'
            )
        if message.header.frame_id != frame_id:
            errors.append(
                f'{prefix} frame_id {message.header.frame_id!r} '
                f'!= {frame_id!r}'
            )
        if message.step <= 0:
            errors.append(f'{prefix} empty row step')
        if len(message.data) != message.step * message.height:
            errors.append(f'{prefix} payload size != step * height')
        stamp_sec = (
            float(message.header.stamp.sec)
            + float(message.header.stamp.nanosec) / 1e9
        )
        if stamp_sec <= 0.0:
            errors.append(f'{prefix} zero timestamp')

        if message.encoding == 'rgb8' and message.step > 0:
            pixel_count = message.width * message.height
            pixel_stride = max(1, pixel_count // 4096)
            sampled_pixels = set()
            for pixel_index in range(0, pixel_count, pixel_stride):
                row, column = divmod(pixel_index, message.width)
                offset = row * message.step + column * 3
                sampled_pixels.add(tuple(message.data[offset:offset + 3]))
            if sampled_pixels == {(0, 0, 0)}:
                errors.append(f'{prefix} frame is entirely black')
            elif len(sampled_pixels) <= 1:
                errors.append(f'{prefix} frame is uniform')
        elif message.encoding == '32FC1' and message.step > 0:
            byte_order = '>' if message.is_bigendian else '<'
            pixel_count = message.width * message.height
            pixel_stride = max(1, pixel_count // 4096)
            finite_depths = []
            for pixel_index in range(0, pixel_count, pixel_stride):
                row, column = divmod(pixel_index, message.width)
                offset = row * message.step + column * 4
                depth = struct.unpack_from(
                    f'{byte_order}f', message.data, offset
                )[0]
                if math.isfinite(depth):
                    finite_depths.append(depth)
            if not finite_depths:
                errors.append(f'{prefix} frame has no finite depth values')
            elif not all(0.1 <= value <= 10.0 for value in finite_depths):
                errors.append(f'{prefix} finite depth is outside clip range')

        if len(measurements.image_checksums[topic]) <= 1:
            errors.append(f'{prefix} frame did not change during movement')
    return errors


def _lidar_validation_errors(
    measurements: SensorMeasurements,
) -> tuple[list[str], int]:
    errors = []
    message = measurements.latest_scan
    if message is None:
        return ['/scan: no message stored'], 0
    if message.header.frame_id != 'lidar_link':
        errors.append(
            f'/scan: frame_id {message.header.frame_id!r} != lidar_link'
        )
    if len(message.ranges) != 360:
        errors.append(f'/scan: range count {len(message.ranges)} != 360')
    if message.angle_increment <= 0.0:
        errors.append('/scan: angle_increment is not positive')
    if not math.isclose(message.range_min, 0.15, abs_tol=1e-4):
        errors.append(f'/scan: range_min {message.range_min} != 0.15')
    if not math.isclose(message.range_max, 12.0, abs_tol=1e-4):
        errors.append(f'/scan: range_max {message.range_max} != 12.0')
    if any(math.isnan(value) for value in message.ranges):
        errors.append('/scan: contains NaN ranges')
    finite_ranges = [
        value for value in message.ranges if math.isfinite(value)
    ]
    if not finite_ranges:
        errors.append('/scan: contains no finite obstacle ranges')
    elif not all(
        message.range_min <= value <= message.range_max
        for value in finite_ranges
    ):
        errors.append('/scan: finite ranges outside declared limits')
    stamp_sec = (
        float(message.header.stamp.sec)
        + float(message.header.stamp.nanosec) / 1e9
    )
    if stamp_sec <= 0.0:
        errors.append('/scan: zero timestamp')
    return errors, len(finite_ranges)


def _stop_process_group(process: subprocess.Popen[bytes]) -> None:
    if process.poll() is not None:
        return
    os.killpg(process.pid, signal.SIGINT)
    try:
        process.wait(timeout=10.0)
        return
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGTERM)
    try:
        process.wait(timeout=5.0)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGKILL)
        process.wait(timeout=5.0)


def _assert_thresholds(
    options: RuntimeTestOptions,
    rtf: float,
    sim_hz: dict[str, float],
    expected_topics: tuple[str, ...],
) -> None:
    if options.min_rtf is not None:
        assert rtf >= options.min_rtf, (
            f'RTF {rtf:.3f} is below minimum {options.min_rtf:.3f}'
        )
    if options.min_camera_sim_hz is not None:
        for topic in expected_topics:
            if topic == LIDAR_TOPIC:
                continue
            assert sim_hz[topic] >= options.min_camera_sim_hz, (
                f'{topic} sim Hz {sim_hz[topic]:.3f} is below minimum '
                f'{options.min_camera_sim_hz:.3f}'
            )
    if (
        options.min_lidar_sim_hz is not None
        and LIDAR_TOPIC in expected_topics
    ):
        assert sim_hz[LIDAR_TOPIC] >= options.min_lidar_sim_hz, (
            f'{LIDAR_TOPIC} sim Hz {sim_hz[LIDAR_TOPIC]:.3f} is below minimum '
            f'{options.min_lidar_sim_hz:.3f}'
        )


def test_sensor_profile_and_rtf(
    runtime_test_options: RuntimeTestOptions,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    options = runtime_test_options
    _assert_profile_environment(options.profile)
    expected_topics = PROFILE_TOPICS[options.sensor_profile]
    expected_topic_set = set(expected_topics)

    monkeypatch.setenv('ROS_DOMAIN_ID', str(100 + os.getpid() % 100))
    partition = f'cleany_runtime_test_{uuid4().hex}'
    partition_variable = (
        'IGN_PARTITION' if options.profile == 'fortress' else 'GZ_PARTITION'
    )
    monkeypatch.setenv(partition_variable, partition)
    launch_environment = os.environ.copy()

    rclpy.init()
    node = Node('gazebo_runtime_sensor_test')
    measurements = SensorMeasurements()
    node.create_subscription(
        Clock,
        '/clock',
        measurements.clock_callback,
        qos_profile_sensor_data,
    )
    for topic in CAMERA_TOPICS:
        node.create_subscription(
            Image,
            topic,
            measurements.image_callback(topic),
            qos_profile_sensor_data,
        )
    node.create_subscription(
        LaserScan,
        LIDAR_TOPIC,
        measurements.scan_callback,
        qos_profile_sensor_data,
    )
    cmd_vel_publisher = node.create_publisher(Twist, '/cmd_vel', 10)

    temp_prefix = 'cleany-gazebo-runtime-'
    with tempfile.TemporaryDirectory(prefix=temp_prefix) as temp_dir:
        log_path = Path(temp_dir) / 'launch.log'
        world_path = Path(temp_dir) / 'runtime_validation_world.sdf'
        _write_validation_world(options.profile, world_path)
        with log_path.open('wb') as launch_log:
            process = subprocess.Popen(
                _launch_command(
                    options.profile,
                    options.sensor_profile,
                    world_path,
                ),
                env=launch_environment,
                stdout=launch_log,
                stderr=subprocess.STDOUT,
                start_new_session=True,
            )
            try:
                _wait_for_topics(
                    node,
                    measurements,
                    expected_topic_set,
                    options.startup_timeout_sec,
                    process,
                    log_path,
                )
                drive_sec = min(2.0, options.warmup_sec / 2.0)
                _drive_for(
                    node,
                    cmd_vel_publisher,
                    drive_sec,
                    process,
                    log_path,
                )
                _spin_for(
                    node,
                    options.warmup_sec - drive_sec,
                    process,
                    log_path,
                )

                sim_start = measurements.latest_sim_sec
                assert sim_start is not None
                wall_start = time.monotonic()
                measurements.measuring = True
                _spin_for(
                    node,
                    options.measure_sec,
                    process,
                    log_path,
                )
                measurements.measuring = False
                wall_elapsed = time.monotonic() - wall_start
                sim_end = measurements.latest_sim_sec
                assert sim_end is not None
                sim_elapsed = sim_end - sim_start

                assert sim_elapsed > 0.0, 'simulation time did not advance'
                missing_during_measurement = [
                    topic
                    for topic in expected_topics
                    if measurements.message_counts[topic] == 0
                ]
                assert not missing_during_measurement, (
                    'no messages received during measurement for '
                    f'{missing_during_measurement}'
                )
                unexpected_topics = sorted(
                    measurements.seen_topics - expected_topic_set
                )
                assert not unexpected_topics, (
                    f'disabled sensor topics received: {unexpected_topics}'
                )
                camera_topics = tuple(
                    topic for topic in expected_topics if topic != LIDAR_TOPIC
                )
                validation_errors = _camera_validation_errors(
                    measurements,
                    camera_topics,
                )
                finite_lidar_ranges = 0
                if LIDAR_TOPIC in expected_topic_set:
                    lidar_errors, finite_lidar_ranges = _lidar_validation_errors(
                        measurements
                    )
                    validation_errors.extend(lidar_errors)

                lidar_summary = (
                    f', finite_lidar_ranges={finite_lidar_ranges}/360'
                    if LIDAR_TOPIC in expected_topic_set
                    else ''
                )

                rtf = sim_elapsed / wall_elapsed
                wall_hz = {
                    topic: measurements.message_counts[topic] / wall_elapsed
                    for topic in expected_topics
                }
                sim_hz = {
                    topic: measurements.message_counts[topic] / sim_elapsed
                    for topic in expected_topics
                }

                print(
                    f'\nGazebo runtime result: profile={options.profile}, '
                    f'sensor_profile={options.sensor_profile}, '
                    f'wall={wall_elapsed:.3f}s, sim={sim_elapsed:.3f}s, '
                    f'RTF={rtf:.3f}{lidar_summary}'
                )
                for topic in expected_topics:
                    print(
                        f'  {topic}: '
                        f'count={measurements.message_counts[topic]}, '
                        f'wall_hz={wall_hz[topic]:.3f}, '
                        f'sim_hz={sim_hz[topic]:.3f}'
                    )

                assert not validation_errors, (
                    'sensor data validation failed:\n- '
                    + '\n- '.join(validation_errors)
                )
                _assert_thresholds(options, rtf, sim_hz, expected_topics)
            finally:
                _stop_process_group(process)
                node.destroy_node()
                rclpy.shutdown()
