from __future__ import annotations

from dataclasses import dataclass
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time
from typing import Callable
from uuid import uuid4

import pytest


if os.environ.get('ROS_DISTRO') is None:
    pytest.skip('ROS 2 environment is not active', allow_module_level=True)

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo, Image, JointState
from tf2_msgs.msg import TFMessage

from cleany_mujoco_sim.camera_contract import (
    CAMERA_D,
    CAMERA_FRAME_ID,
    CAMERA_HEIGHT,
    CAMERA_K,
    CAMERA_P,
    CAMERA_R,
    CAMERA_WIDTH,
    DISTORTION_MODEL,
    PUBLIC_IMAGE_TOPIC,
    PUBLIC_INFO_TOPIC,
)


Stamp = tuple[int, int]


def _stamp(message: Image | CameraInfo | JointState) -> Stamp:
    return (message.header.stamp.sec, message.header.stamp.nanosec)


def _nanoseconds(stamp: Stamp) -> int:
    return stamp[0] * 1_000_000_000 + stamp[1]


@dataclass(frozen=True)
class ImageRecord:
    frame_id: str
    width: int
    height: int
    encoding: str
    is_bigendian: int
    step: int
    data_length: int


@dataclass(frozen=True)
class CameraInfoRecord:
    frame_id: str
    width: int
    height: int
    distortion_model: str
    d: tuple[float, ...]
    k: tuple[float, ...]
    r: tuple[float, ...]
    p: tuple[float, ...]


class CameraRuntimeProbe(Node):
    def __init__(self) -> None:
        super().__init__('handeye_camera_runtime_probe')
        self.images: dict[Stamp, ImageRecord] = {}
        self.infos: dict[Stamp, CameraInfoRecord] = {}
        self.joint_stamps: list[Stamp] = []
        self.clock_nanoseconds: list[int] = []
        self.tf_children: set[str] = set()
        self.create_subscription(
            Image,
            PUBLIC_IMAGE_TOPIC,
            self._on_image,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            CameraInfo,
            PUBLIC_INFO_TOPIC,
            self._on_info,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            JointState,
            '/joint_states',
            self._on_joint_state,
            qos_profile_sensor_data,
        )
        self.create_subscription(Clock, '/clock', self._on_clock, 100)
        self.create_subscription(TFMessage, '/tf', self._on_tf, 100)
        self.create_subscription(
            TFMessage,
            '/tf_static',
            self._on_tf,
            QoSProfile(
                depth=1,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                reliability=ReliabilityPolicy.RELIABLE,
            ),
        )

    def _on_image(self, message: Image) -> None:
        self.images[_stamp(message)] = ImageRecord(
            frame_id=message.header.frame_id,
            width=message.width,
            height=message.height,
            encoding=message.encoding,
            is_bigendian=message.is_bigendian,
            step=message.step,
            data_length=len(message.data),
        )

    def _on_info(self, message: CameraInfo) -> None:
        self.infos[_stamp(message)] = CameraInfoRecord(
            frame_id=message.header.frame_id,
            width=message.width,
            height=message.height,
            distortion_model=message.distortion_model,
            d=tuple(message.d),
            k=tuple(message.k),
            r=tuple(message.r),
            p=tuple(message.p),
        )

    def _on_joint_state(self, message: JointState) -> None:
        stamp = _stamp(message)
        if stamp != (0, 0):
            self.joint_stamps.append(stamp)

    def _on_clock(self, message: Clock) -> None:
        stamp = (message.clock.sec, message.clock.nanosec)
        self.clock_nanoseconds.append(_nanoseconds(stamp))

    def _on_tf(self, message: TFMessage) -> None:
        self.tf_children.update(
            transform.child_frame_id for transform in message.transforms
        )

    def matched_stamps(self) -> list[Stamp]:
        return sorted(self.images.keys() & self.infos.keys())


def _log_tail(path: Path, lines: int = 180) -> str:
    try:
        return '\n'.join(
            path.read_text(encoding='utf-8', errors='replace').splitlines()[
                -lines:
            ]
        )
    except OSError as error:
        return f'<could not read launch log: {error}>'


def _assert_launch_running(
    process: subprocess.Popen[bytes],
    log_path: Path,
) -> None:
    return_code = process.poll()
    if return_code is not None:
        pytest.fail(
            f'hand-eye backend exited early with code {return_code}\n'
            f'launch log tail:\n{_log_tail(log_path)}'
        )


def _wait_for(
    predicate: Callable[[], bool],
    *,
    timeout_sec: float,
    node: Node,
    process: subprocess.Popen[bytes],
    log_path: Path,
    description: str,
) -> None:
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        _assert_launch_running(process, log_path)
        if predicate():
            return
        rclpy.spin_once(node, timeout_sec=0.1)
    pytest.fail(
        f'timed out waiting for {description}\n'
        f'launch log tail:\n{_log_tail(log_path)}'
    )


def _stop_launch(process: subprocess.Popen[bytes]) -> None:
    if process.poll() is not None:
        return
    os.killpg(process.pid, signal.SIGINT)
    try:
        process.wait(timeout=20.0)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGTERM)
        try:
            process.wait(timeout=10.0)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait(timeout=5.0)


def test_wrist_camera_and_robot_feedback_share_simulation_timeline() -> None:
    with tempfile.TemporaryDirectory(
        prefix='cleany_handeye_camera_test_'
    ) as tmp:
        temp_root = Path(tmp)
        log_path = temp_root / 'launch.log'
        domain_id = 20 + uuid4().int % 180
        environment = os.environ.copy()
        environment.update(
            {
                'ROS_DOMAIN_ID': str(domain_id),
                'ROS_HOME': str(temp_root / 'ros_home'),
                'ROS_LOG_DIR': str(temp_root / 'ros_log'),
            }
        )
        with log_path.open('wb') as launch_log:
            process = subprocess.Popen(
                [
                    'ros2',
                    'launch',
                    'cleany_mujoco_sim',
                    'handeye_backend.launch.py',
                    'headless:=true',
                    'sim_speed_factor:=1.0',
                ],
                env=environment,
                cwd=temp_root,
                stdout=launch_log,
                stderr=subprocess.STDOUT,
                start_new_session=True,
            )
            rclpy.init(args=None, domain_id=domain_id)
            probe = CameraRuntimeProbe()
            try:
                _wait_for(
                    lambda: len(probe.matched_stamps()) >= 3
                    and len(probe.joint_stamps) >= 10
                    and len(probe.clock_nanoseconds) >= 10,
                    timeout_sec=90.0,
                    node=probe,
                    process=process,
                    log_path=log_path,
                    description=(
                        'paired wrist images/CameraInfo, joint states and '
                        'clock'
                    ),
                )

                public_topics = {
                    name
                    for name, _ in probe.get_topic_names_and_types()
                    if name.startswith('/left_wrist_camera/')
                }
                assert public_topics == {PUBLIC_IMAGE_TOPIC, PUBLIC_INFO_TOPIC}

                matched = probe.matched_stamps()
                assert matched[0] != (0, 0)
                camera_times = [_nanoseconds(stamp) for stamp in matched]
                assert camera_times == sorted(set(camera_times))
                assert camera_times[-1] > camera_times[0]

                for stamp in matched:
                    image = probe.images[stamp]
                    info = probe.infos[stamp]
                    assert image == ImageRecord(
                        frame_id=CAMERA_FRAME_ID,
                        width=CAMERA_WIDTH,
                        height=CAMERA_HEIGHT,
                        encoding='rgb8',
                        is_bigendian=0,
                        step=CAMERA_WIDTH * 3,
                        data_length=CAMERA_WIDTH * CAMERA_HEIGHT * 3,
                    )
                    assert info == CameraInfoRecord(
                        frame_id=CAMERA_FRAME_ID,
                        width=CAMERA_WIDTH,
                        height=CAMERA_HEIGHT,
                        distortion_model=DISTORTION_MODEL,
                        d=CAMERA_D,
                        k=CAMERA_K,
                        r=CAMERA_R,
                        p=CAMERA_P,
                    )

                joint_times = [
                    _nanoseconds(stamp) for stamp in probe.joint_stamps
                ]
                assert max(probe.clock_nanoseconds) > min(
                    probe.clock_nanoseconds
                )
                assert min(probe.clock_nanoseconds) <= camera_times[0]
                assert camera_times[-1] <= max(probe.clock_nanoseconds)
                assert all(
                    min(
                        abs(camera_time - joint_time)
                        for joint_time in joint_times
                    )
                    <= 50_000_000
                    for camera_time in camera_times
                )

                # The frame label is valid message metadata, but the unknown
                # calibration edge must not be injected into canonical TF.
                assert CAMERA_FRAME_ID not in probe.tf_children
            finally:
                probe.destroy_node()
                rclpy.shutdown()
                _stop_launch(process)
                launch_log.flush()

        log_text = log_path.read_text(encoding='utf-8', errors='replace')
        assert 'camera contract ready: 640x480' in log_text
        assert 'Starting the camera rendering loop' in log_text
        assert 'Resized offscreen buffer to 640 x 480' in log_text
        assert 'camera collection blocked' not in log_text
        assert 'TypeError' not in log_text
        assert 'exit code 1' not in log_text
