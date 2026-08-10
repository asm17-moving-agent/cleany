import numpy as np
import pytest
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image

from cleany_perception.core.models import FailureKind, InspectionFailure
from cleany_perception.rgbd_snapshot import (
    RgbdSnapshotBuffer,
    SynchronizedRgbdMessages,
    snapshot_from_messages,
)


def _image(width, height, encoding, array, frame='camera', stamp_ns=1):
    message = Image()
    message.header.stamp = Time(nanoseconds=stamp_ns).to_msg()
    message.header.frame_id = frame
    message.width = width
    message.height = height
    message.encoding = encoding
    message.is_bigendian = False
    message.step = len(array.tobytes()) // height
    message.data = array.tobytes()
    return message


def _info(width, height, frame='camera', stamp_ns=1):
    message = CameraInfo()
    message.header.stamp = Time(nanoseconds=stamp_ns).to_msg()
    message.header.frame_id = frame
    message.width = width
    message.height = height
    message.k = [100.0, 0.0, 1.5, 0.0, 100.0, 0.5, 0.0, 0.0, 1.0]
    return message


def _messages(depth_encoding='32FC1', stamp_ns=1):
    rgb = np.arange(24, dtype=np.uint8).reshape(2, 4, 3)
    if depth_encoding == '32FC1':
        depth = np.full((2, 4), 0.75, dtype='<f4')
    else:
        depth = np.full((2, 4), 750, dtype='<u2')
    return SynchronizedRgbdMessages(
        sequence=1,
        stamp_ns=stamp_ns,
        color=_image(4, 2, 'rgb8', rgb, stamp_ns=stamp_ns),
        color_info=_info(4, 2, stamp_ns=stamp_ns),
        depth=_image(4, 2, depth_encoding, depth, stamp_ns=stamp_ns),
        depth_info=_info(4, 2, stamp_ns=stamp_ns),
    )


@pytest.mark.parametrize('encoding', ['32FC1', '16UC1'])
def test_snapshot_conversion_supports_meter_and_scaled_depth(encoding):
    snapshot = snapshot_from_messages(_messages(encoding))

    assert snapshot.rgb.shape == (2, 4, 3)
    assert snapshot.depth_m == pytest.approx(np.full((2, 4), 0.75))
    assert snapshot.intrinsics.fx == pytest.approx(100.0)
    assert snapshot.source_frame == 'camera'


def test_snapshot_conversion_rejects_mismatched_intrinsics():
    messages = _messages()
    messages.depth_info.k[0] = 99.0

    with pytest.raises(InspectionFailure) as raised:
        snapshot_from_messages(messages)

    assert raised.value.kind == FailureKind.DEPTH


def test_snapshot_buffer_only_releases_exact_timestamp_bundle():
    buffer = RgbdSnapshotBuffer()
    first = _messages(stamp_ns=10)
    second = _messages(stamp_ns=20)

    buffer.add_color(first.color)
    buffer.add_color_info(first.color_info)
    buffer.add_depth(first.depth)
    buffer.add_depth_info(second.depth_info)
    assert buffer.sequence == 0

    buffer.add_depth_info(first.depth_info)
    synchronized = buffer.wait_for_new(0, timeout_seconds=0.1)

    assert synchronized.stamp_ns == 10
    assert synchronized.sequence == 1

    buffer.add_color(first.color)
    buffer.add_color_info(first.color_info)
    buffer.add_depth(first.depth)
    buffer.add_depth_info(first.depth_info)
    assert buffer.sequence == 1


def test_snapshot_buffer_reports_timeout_and_cancel():
    buffer = RgbdSnapshotBuffer()

    with pytest.raises(InspectionFailure) as timed_out:
        buffer.wait_for_new(0, timeout_seconds=0.01)
    assert timed_out.value.kind == FailureKind.RGBD_TIMEOUT

    with pytest.raises(InspectionFailure) as cancelled:
        buffer.wait_for_new(0, timeout_seconds=0.1, cancelled=lambda: True)
    assert cancelled.value.kind == FailureKind.CANCELLED
