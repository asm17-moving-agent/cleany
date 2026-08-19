from dataclasses import dataclass

import pytest

from cleany_handeye_calibration.camera_acquisition import (
    CAMERA_D,
    CAMERA_DISTORTION_MODEL,
    CAMERA_FRAME_ID,
    CAMERA_HEIGHT,
    CAMERA_K,
    CAMERA_P,
    CAMERA_R,
    CAMERA_WIDTH,
)
from cleany_handeye_calibration.ros_camera_adapter import (
    RosCameraMessageError,
    RosExactCameraPairAdapter,
    camera_info_frame_from_ros,
    image_frame_from_ros,
    ros_stamp_to_ns,
)


@dataclass
class Stamp:
    sec: int
    nanosec: int


@dataclass
class Header:
    stamp: Stamp
    frame_id: str = CAMERA_FRAME_ID


@dataclass
class ImageMessage:
    header: Header
    width: int = CAMERA_WIDTH
    height: int = CAMERA_HEIGHT
    encoding: str = 'rgb8'
    is_bigendian: int = 0
    step: int = CAMERA_WIDTH * 3
    data: bytes | bytearray = bytes(CAMERA_WIDTH * CAMERA_HEIGHT * 3)


@dataclass
class CameraInfoMessage:
    header: Header
    width: int = CAMERA_WIDTH
    height: int = CAMERA_HEIGHT
    distortion_model: str = CAMERA_DISTORTION_MODEL
    d: tuple[float, ...] = CAMERA_D
    k: tuple[float, ...] = CAMERA_K
    r: tuple[float, ...] = CAMERA_R
    p: tuple[float, ...] = CAMERA_P


def _header(stamp_ns: int) -> Header:
    return Header(Stamp(stamp_ns // 1_000_000_000, stamp_ns % 1_000_000_000))


def test_ros_stamp_conversion_requires_valid_builtin_time_fields():
    assert ros_stamp_to_ns(Stamp(12, 34)) == 12_000_000_034

    for invalid in (
        Stamp(-1, 0),
        Stamp(0, -1),
        Stamp(0, 1_000_000_000),
        Stamp(True, 0),
        object(),
    ):
        with pytest.raises(RosCameraMessageError):
            ros_stamp_to_ns(invalid)


def test_ros_converters_copy_exact_image_and_camera_info_fields():
    payload = bytearray(CAMERA_WIDTH * CAMERA_HEIGHT * 3)
    image = image_frame_from_ros(ImageMessage(_header(123), data=payload))
    info = camera_info_frame_from_ros(CameraInfoMessage(_header(123)))
    payload[0] = 255

    assert image.stamp_ns == 123
    assert image.data[0] == 0
    assert image.frame_id == CAMERA_FRAME_ID
    assert info.stamp_ns == 123
    assert info.k == CAMERA_K
    assert info.d == CAMERA_D
    assert info.r == CAMERA_R
    assert info.p == CAMERA_P


def test_ros_adapter_pairs_only_the_same_source_stamp_after_settle():
    adapter = RosExactCameraPairAdapter(queue_capacity=2)
    adapter.on_image(ImageMessage(_header(101)))
    adapter.on_camera_info(CameraInfoMessage(_header(102)))
    adapter.on_camera_info(CameraInfoMessage(_header(201)))
    update = adapter.on_image(ImageMessage(_header(201)))

    pair = adapter.wait_for_first_compatible_frame(
        settled_stamp_ns=200,
        timeout_sec=0.1,
    )

    assert update.pair_ready
    assert pair.image.stamp_ns == 201
    assert pair.camera_info.stamp_ns == 201
    assert pair.image.frame_id == pair.camera_info.frame_id


@pytest.mark.parametrize('is_bigendian', [-1, 2, 'false'])
def test_ros_image_converter_rejects_non_bool_endianness(is_bigendian):
    with pytest.raises(RosCameraMessageError, match='is_bigendian'):
        image_frame_from_ros(
            ImageMessage(_header(100), is_bigendian=is_bigendian)
        )
