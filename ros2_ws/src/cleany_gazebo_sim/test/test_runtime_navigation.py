from __future__ import annotations

import math
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time
from uuid import uuid4

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import pytest
import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu, LaserScan
from tf2_ros import Buffer, TransformException, TransformListener

from cleany_gazebo_sim.world_generator import materialize_mecanum_wheel_world
from conftest import RuntimeTestOptions


LIDAR_TOPIC = '/scan'
IMU_TOPIC = '/imu/data'
ODOM_TOPIC = '/odom'
MEASURED_TOPICS = (LIDAR_TOPIC, IMU_TOPIC, ODOM_TOPIC)
CAMERA_SENSOR_NAMES = (
    'head_realsense_rgb',
    'head_realsense_depth',
    'left_wrist_rgb',
    'right_wrist_rgb',
)
VALIDATION_WALLS = (
    ('front', '3 0 1 0 0 0', '0.2 6 2'),
    ('back', '-3 0 1 0 0 0', '0.2 6 2'),
    ('left', '0 3 1 0 0 0', '6 0.2 2'),
    ('right', '0 -3 1 0 0 0', '6 0.2 2'),
)


class NavigationMeasurements:
    def __init__(self) -> None:
        self.latest_sim_sec: float | None = None
        self.seen_topics: set[str] = set()
        self.message_counts = {topic: 0 for topic in MEASURED_TOPICS}
        self.latest_scan: LaserScan | None = None
        self.latest_imu: Imu | None = None
        self.latest_odom: Odometry | None = None
        self.max_drive_angular_z = 0.0
        self.measuring = False
        self.driving = False

    def clock_callback(self, message: Clock) -> None:
        self.latest_sim_sec = (
            float(message.clock.sec) + float(message.clock.nanosec) / 1e9
        )

    def scan_callback(self, message: LaserScan) -> None:
        self.seen_topics.add(LIDAR_TOPIC)
        self.latest_scan = message
        if self.measuring:
            self.message_counts[LIDAR_TOPIC] += 1

    def imu_callback(self, message: Imu) -> None:
        self.seen_topics.add(IMU_TOPIC)
        self.latest_imu = message
        if self.measuring:
            self.message_counts[IMU_TOPIC] += 1
        if self.driving:
            self.max_drive_angular_z = max(
                self.max_drive_angular_z,
                abs(message.angular_velocity.z),
            )

    def odom_callback(self, message: Odometry) -> None:
        self.seen_topics.add(ODOM_TOPIC)
        self.latest_odom = message
        if self.measuring:
            self.message_counts[ODOM_TOPIC] += 1


def _launch_command(profile: str, world_path: Path) -> list[str]:
    package_share = Path(get_package_share_directory('cleany_gazebo_sim'))
    if profile == 'fortress':
        launch_file = 'gazebo_sim.launch.py'
        bridge_file = 'navigation_bridge.yaml'
    else:
        launch_file = 'gazebo_harmonic.launch.py'
        bridge_file = 'navigation_bridge_harmonic.yaml'
    bridge_path = package_share / 'config' / bridge_file
    return [
        'ros2',
        'launch',
        'cleany_gazebo_sim',
        launch_file,
        'headless:=true',
        f'world:={world_path}',
        f'bridge_config:={bridge_path}',
    ]


def _disable_camera_sensors(world_sdf: str) -> str:
    result = world_sdf
    for sensor_name in CAMERA_SENSOR_NAMES:
        start = result.find(f'<sensor name="{sensor_name}"')
        if start < 0:
            pytest.fail(f'camera sensor {sensor_name!r} is missing from world')
        end = result.find('</sensor>', start)
        if end < 0:
            pytest.fail(f'camera sensor {sensor_name!r} is malformed')
        sensor_sdf = result[start:end]
        sensor_sdf = sensor_sdf.replace(
            '<always_on>true</always_on>',
            '<always_on>false</always_on>',
        )
        result = result[:start] + sensor_sdf + result[end:]
    return result


def _validation_scene_sdf() -> str:
    walls = []
    for name, pose, size in VALIDATION_WALLS:
        walls.append(
            f'''<model name="navigation_test_{name}_wall">
  <static>true</static>
  <pose>{pose}</pose>
  <link name="wall">
    <collision name="collision">
      <geometry><box><size>{size}</size></box></geometry>
    </collision>
    <visual name="visual">
      <geometry><box><size>{size}</size></box></geometry>
      <material><diffuse>0.7 0.7 0.7 1</diffuse></material>
    </visual>
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
    materialized_world = _disable_camera_sensors(materialized_world)
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


def _launch_log_tail(log_path: Path, line_count: int = 100) -> str:
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
    measurements: NavigationMeasurements,
    seconds: float,
    process: subprocess.Popen[bytes],
    log_path: Path,
) -> None:
    command = Twist()
    command.linear.x = 0.1
    command.angular.z = 0.2
    measurements.driving = True
    deadline = time.monotonic() + seconds
    try:
        while time.monotonic() < deadline:
            _assert_launch_running(process, log_path)
            publisher.publish(command)
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        measurements.driving = False
        publisher.publish(Twist())


def _tf_available(buffer: Buffer) -> bool:
    return all(
        buffer.can_transform(target, source, Time())
        for target, source in (
            ('base_link', 'lidar_link'),
            ('base_link', 'imu_link'),
            ('odom', 'lidar_link'),
            ('odom', 'imu_link'),
        )
    )


def _wait_for_ready(
    node: Node,
    measurements: NavigationMeasurements,
    tf_buffer: Buffer,
    timeout_sec: float,
    process: subprocess.Popen[bytes],
    log_path: Path,
) -> None:
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        _assert_launch_running(process, log_path)
        if (
            measurements.latest_sim_sec is not None
            and measurements.seen_topics == set(MEASURED_TOPICS)
            and _tf_available(tf_buffer)
        ):
            return
        rclpy.spin_once(node, timeout_sec=0.1)

    missing = sorted(set(MEASURED_TOPICS) - measurements.seen_topics)
    pytest.fail(
        f'navigation startup timed out after {timeout_sec:.1f}s; '
        f'missing topics={missing}, tf_ready={_tf_available(tf_buffer)}\n'
        f'Launch log tail:\n{_launch_log_tail(log_path)}'
    )


def _stamp_seconds(message) -> float:
    return (
        float(message.header.stamp.sec)
        + float(message.header.stamp.nanosec) / 1e9
    )


def _lidar_validation_errors(scan: LaserScan) -> tuple[list[str], int]:
    errors = []
    if scan.header.frame_id != 'lidar_link':
        errors.append(f'/scan: unexpected frame {scan.header.frame_id!r}')
    if _stamp_seconds(scan) <= 0.0:
        errors.append('/scan: zero timestamp')
    if len(scan.ranges) != 360:
        errors.append(f'/scan: range count {len(scan.ranges)} != 360')
    if not math.isclose(scan.range_min, 0.15, abs_tol=1e-4):
        errors.append(f'/scan: range_min {scan.range_min} != 0.15')
    if not math.isclose(scan.range_max, 12.0, abs_tol=1e-4):
        errors.append(f'/scan: range_max {scan.range_max} != 12.0')

    finite_ranges = [value for value in scan.ranges if math.isfinite(value)]
    if not finite_ranges:
        errors.append('/scan: no finite ranges')
        return errors, 0
    ranges_are_valid = all(
        scan.range_min <= value <= scan.range_max
        for value in finite_ranges
    )
    if not ranges_are_valid:
        errors.append('/scan: finite range outside declared limits')
    saturated_count = sum(
        value <= scan.range_min + 0.05 for value in finite_ranges
    )
    if saturated_count > 9 * len(finite_ranges) // 10:
        errors.append(
            f'/scan: {saturated_count}/{len(finite_ranges)} ranges are '
            'saturated near range_min'
        )
    if max(finite_ranges) - min(finite_ranges) < 0.1:
        errors.append('/scan: obstacle ranges lack spatial diversity')
    return errors, len(finite_ranges)


def _imu_validation_errors(
    imu: Imu, max_drive_angular_z: float
) -> list[str]:
    errors = []
    if imu.header.frame_id != 'imu_link':
        errors.append(f'/imu/data: unexpected frame {imu.header.frame_id!r}')
    if _stamp_seconds(imu) <= 0.0:
        errors.append('/imu/data: zero timestamp')

    orientation = imu.orientation
    angular_velocity = imu.angular_velocity
    linear_acceleration = imu.linear_acceleration
    values = (
        orientation.x,
        orientation.y,
        orientation.z,
        orientation.w,
        angular_velocity.x,
        angular_velocity.y,
        angular_velocity.z,
        linear_acceleration.x,
        linear_acceleration.y,
        linear_acceleration.z,
    )
    if not all(math.isfinite(value) for value in values):
        errors.append('/imu/data: contains non-finite values')
        return errors

    quaternion_norm = math.sqrt(
        orientation.x ** 2
        + orientation.y ** 2
        + orientation.z ** 2
        + orientation.w ** 2
    )
    if not math.isclose(quaternion_norm, 1.0, abs_tol=1e-2):
        errors.append(
            f'/imu/data: quaternion norm {quaternion_norm:.3f} != 1'
        )
    acceleration_norm = math.sqrt(
        linear_acceleration.x ** 2
        + linear_acceleration.y ** 2
        + linear_acceleration.z ** 2
    )
    if not math.isclose(acceleration_norm, 9.8, abs_tol=1.5):
        errors.append(
            f'/imu/data: acceleration norm {acceleration_norm:.3f} '
            'is not near gravity'
        )
    if max_drive_angular_z < 0.05:
        errors.append(
            f'/imu/data: max drive angular z {max_drive_angular_z:.3f} '
            'did not respond to rotation'
        )
    return errors


def _yaw(odom: Odometry) -> float:
    orientation = odom.pose.pose.orientation
    return math.atan2(
        2.0 * (
            orientation.w * orientation.z
            + orientation.x * orientation.y
        ),
        1.0 - 2.0 * (
            orientation.y * orientation.y
            + orientation.z * orientation.z
        ),
    )


def _assert_odometry_changed(start: Odometry, end: Odometry) -> None:
    start_position = start.pose.pose.position
    end_position = end.pose.pose.position
    distance = math.hypot(
        end_position.x - start_position.x,
        end_position.y - start_position.y,
    )
    yaw_difference = math.atan2(
        math.sin(_yaw(end) - _yaw(start)),
        math.cos(_yaw(end) - _yaw(start)),
    )
    assert distance > 0.02 or abs(yaw_difference) > 0.03, (
        'odometry did not change during cmd_vel; '
        f'distance={distance:.3f}, yaw={yaw_difference:.3f}'
    )


def _lookup_transform(buffer: Buffer, target: str, source: str):
    try:
        return buffer.lookup_transform(target, source, Time())
    except TransformException as error:
        pytest.fail(f'could not transform {target} <- {source}: {error}')


def _assert_sensor_transforms(buffer: Buffer) -> None:
    lidar = _lookup_transform(buffer, 'base_link', 'lidar_link')
    imu = _lookup_transform(buffer, 'base_link', 'imu_link')
    _lookup_transform(buffer, 'odom', 'lidar_link')
    _lookup_transform(buffer, 'odom', 'imu_link')

    lidar_translation = lidar.transform.translation
    imu_translation = imu.transform.translation
    assert math.isclose(lidar_translation.x, 0.32, abs_tol=1e-6)
    assert math.isclose(lidar_translation.y, 0.0, abs_tol=1e-6)
    assert math.isclose(lidar_translation.z, -0.18, abs_tol=1e-6)
    assert math.isclose(imu_translation.x, 0.0, abs_tol=1e-6)
    assert math.isclose(imu_translation.y, 0.0, abs_tol=1e-6)
    assert math.isclose(imu_translation.z, 0.0, abs_tol=1e-6)


def _assert_thresholds(
    options: RuntimeTestOptions,
    rtf: float,
    sim_hz: dict[str, float],
) -> None:
    if options.min_rtf is not None:
        assert rtf >= options.min_rtf
    thresholds = {
        LIDAR_TOPIC: options.min_lidar_sim_hz,
        IMU_TOPIC: options.min_imu_sim_hz,
    }
    for topic, threshold in thresholds.items():
        if threshold is not None:
            assert sim_hz[topic] >= threshold, (
                f'{topic} sim Hz {sim_hz[topic]:.3f} is below '
                f'minimum {threshold:.3f}'
            )


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


def test_navigation_sensor_tf_runtime(
    runtime_test_options: RuntimeTestOptions,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    options = runtime_test_options
    _assert_profile_environment(options.profile)

    monkeypatch.setenv('ROS_DOMAIN_ID', str(100 + os.getpid() % 100))
    partition = f'cleany_navigation_test_{uuid4().hex}'
    partition_variable = (
        'IGN_PARTITION' if options.profile == 'fortress' else 'GZ_PARTITION'
    )
    monkeypatch.setenv(partition_variable, partition)
    launch_environment = os.environ.copy()

    rclpy.init()
    node = Node('gazebo_navigation_runtime_test')
    measurements = NavigationMeasurements()
    node.create_subscription(
        Clock,
        '/clock',
        measurements.clock_callback,
        qos_profile_sensor_data,
    )
    node.create_subscription(
        LaserScan,
        LIDAR_TOPIC,
        measurements.scan_callback,
        qos_profile_sensor_data,
    )
    node.create_subscription(
        Imu,
        IMU_TOPIC,
        measurements.imu_callback,
        qos_profile_sensor_data,
    )
    node.create_subscription(
        Odometry,
        ODOM_TOPIC,
        measurements.odom_callback,
        10,
    )
    cmd_vel_publisher = node.create_publisher(Twist, '/cmd_vel', 10)
    tf_buffer = Buffer()
    tf_listener = TransformListener(tf_buffer, node, spin_thread=False)

    with tempfile.TemporaryDirectory(
        prefix='cleany-gazebo-navigation-'
    ) as temp_dir:
        log_path = Path(temp_dir) / 'launch.log'
        world_path = Path(temp_dir) / 'navigation_validation_world.sdf'
        _write_validation_world(options.profile, world_path)
        with log_path.open('wb') as launch_log:
            process = subprocess.Popen(
                _launch_command(options.profile, world_path),
                env=launch_environment,
                stdout=launch_log,
                stderr=subprocess.STDOUT,
                start_new_session=True,
            )
            try:
                _wait_for_ready(
                    node,
                    measurements,
                    tf_buffer,
                    options.startup_timeout_sec,
                    process,
                    log_path,
                )
                _spin_for(
                    node,
                    options.warmup_sec,
                    process,
                    log_path,
                )

                start_odom = measurements.latest_odom
                sim_start = measurements.latest_sim_sec
                assert start_odom is not None
                assert sim_start is not None
                wall_start = time.monotonic()
                measurements.measuring = True
                drive_sec = min(3.0, options.measure_sec / 2.0)
                _drive_for(
                    node,
                    cmd_vel_publisher,
                    measurements,
                    drive_sec,
                    process,
                    log_path,
                )
                _spin_for(
                    node,
                    options.measure_sec - drive_sec,
                    process,
                    log_path,
                )
                measurements.measuring = False
                wall_elapsed = time.monotonic() - wall_start
                sim_end = measurements.latest_sim_sec
                assert sim_end is not None
                sim_elapsed = sim_end - sim_start
                assert sim_elapsed > 0.0, 'simulation time did not advance'

                missing = [
                    topic
                    for topic, count in measurements.message_counts.items()
                    if count == 0
                ]
                assert not missing, (
                    f'no messages received during measurement for {missing}'
                )
                assert measurements.latest_scan is not None
                assert measurements.latest_imu is not None
                assert measurements.latest_odom is not None

                errors, finite_lidar_ranges = _lidar_validation_errors(
                    measurements.latest_scan
                )
                errors.extend(
                    _imu_validation_errors(
                        measurements.latest_imu,
                        measurements.max_drive_angular_z,
                    )
                )
                _assert_odometry_changed(
                    start_odom, measurements.latest_odom
                )
                _assert_sensor_transforms(tf_buffer)

                rtf = sim_elapsed / wall_elapsed
                wall_hz = {
                    topic: count / wall_elapsed
                    for topic, count in measurements.message_counts.items()
                }
                sim_hz = {
                    topic: count / sim_elapsed
                    for topic, count in measurements.message_counts.items()
                }
                print(
                    f'\nGazebo navigation result: profile={options.profile}, '
                    f'wall={wall_elapsed:.3f}s, sim={sim_elapsed:.3f}s, '
                    f'RTF={rtf:.3f}, '
                    f'finite_lidar_ranges={finite_lidar_ranges}/360'
                )
                for topic in MEASURED_TOPICS:
                    print(
                        f'  {topic}: '
                        f'count={measurements.message_counts[topic]}, '
                        f'wall_hz={wall_hz[topic]:.3f}, '
                        f'sim_hz={sim_hz[topic]:.3f}'
                    )
                print(
                    '  max IMU angular z during drive: '
                    f'{measurements.max_drive_angular_z:.3f} rad/s'
                )

                assert not errors, (
                    'navigation sensor validation failed:\n- '
                    + '\n- '.join(errors)
                )
                _assert_thresholds(options, rtf, sim_hz)
            finally:
                _stop_process_group(process)
                del tf_listener
                node.destroy_node()
                rclpy.shutdown()
