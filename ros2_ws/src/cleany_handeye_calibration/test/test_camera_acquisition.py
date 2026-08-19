from dataclasses import replace
import threading
import time

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
    CameraAcquisitionTimeout,
    CameraInfoFrame,
    CameraPairRejectionReason,
    ExactCameraPairBuffer,
    ExactCameraPairPort,
    ImageFrame,
    validate_camera_pair,
)


RGB_BYTES = bytes(CAMERA_WIDTH * CAMERA_HEIGHT * 3)


def _image(stamp_ns: int = 100) -> ImageFrame:
    return ImageFrame(
        stamp_ns=stamp_ns,
        frame_id=CAMERA_FRAME_ID,
        width=CAMERA_WIDTH,
        height=CAMERA_HEIGHT,
        encoding='rgb8',
        is_bigendian=False,
        step=CAMERA_WIDTH * 3,
        data=RGB_BYTES,
    )


def _camera_info(stamp_ns: int = 100) -> CameraInfoFrame:
    return CameraInfoFrame(
        stamp_ns=stamp_ns,
        frame_id=CAMERA_FRAME_ID,
        width=CAMERA_WIDTH,
        height=CAMERA_HEIGHT,
        distortion_model=CAMERA_DISTORTION_MODEL,
        d=CAMERA_D,
        k=CAMERA_K,
        r=CAMERA_R,
        p=CAMERA_P,
    )


def test_exact_nonzero_pair_matches_full_fixed_camera_contract():
    result = validate_camera_pair(_image(), _camera_info())

    assert result.compatible
    assert result.pair is not None
    assert result.pair.stamp_ns == 100
    assert result.pair.image.frame_id == CAMERA_FRAME_ID
    assert result.pair.image.width == 640
    assert result.pair.image.height == 480
    assert result.pair.camera_info.k == CAMERA_K
    assert result.pair.camera_info.d == CAMERA_D
    assert result.pair.camera_info.r == CAMERA_R
    assert result.pair.camera_info.p == CAMERA_P


@pytest.mark.parametrize(
    ('image_change', 'info_change', 'settled_stamp_ns', 'reason'),
    [
        ({'stamp_ns': 0}, {'stamp_ns': 0}, None,
         CameraPairRejectionReason.ZERO_STAMP),
        ({'stamp_ns': 100}, {'stamp_ns': 101}, None,
         CameraPairRejectionReason.STAMP_MISMATCH),
        ({'frame_id': 'wrong'}, {}, None,
         CameraPairRejectionReason.FRAME_ID_MISMATCH),
        ({'encoding': 'bgr8'}, {}, None,
         CameraPairRejectionReason.IMAGE_ENCODING_MISMATCH),
        ({'width': 639}, {}, None,
         CameraPairRejectionReason.IMAGE_DIMENSIONS_MISMATCH),
        ({'is_bigendian': True}, {}, None,
         CameraPairRejectionReason.IMAGE_ENDIANNESS_MISMATCH),
        ({'step': 1}, {}, None,
         CameraPairRejectionReason.IMAGE_STEP_MISMATCH),
        ({'data': RGB_BYTES[:-1]}, {}, None,
         CameraPairRejectionReason.IMAGE_DATA_LENGTH_MISMATCH),
        ({}, {'height': 479}, None,
         CameraPairRejectionReason.CAMERA_INFO_DIMENSIONS_MISMATCH),
        ({}, {'distortion_model': 'rational_polynomial'}, None,
         CameraPairRejectionReason.DISTORTION_MODEL_MISMATCH),
        ({}, {'k': (1.0,) + CAMERA_K[1:]}, None,
         CameraPairRejectionReason.CAMERA_K_MISMATCH),
        ({}, {'d': (1.0,) + CAMERA_D[1:]}, None,
         CameraPairRejectionReason.CAMERA_D_MISMATCH),
        ({}, {'r': (0.0,) + CAMERA_R[1:]}, None,
         CameraPairRejectionReason.CAMERA_R_MISMATCH),
        ({}, {'p': (1.0,) + CAMERA_P[1:]}, None,
         CameraPairRejectionReason.CAMERA_P_MISMATCH),
        ({}, {}, 100, CameraPairRejectionReason.BEFORE_OR_AT_SETTLE),
    ],
)
def test_incompatible_image_camera_info_pairs_are_explicitly_rejected(
    image_change,
    info_change,
    settled_stamp_ns,
    reason,
):
    result = validate_camera_pair(
        replace(_image(), **image_change),
        replace(_camera_info(), **info_change),
        settled_stamp_ns=settled_stamp_ns,
    )

    assert not result.compatible
    assert result.rejection is not None
    assert result.rejection.reason == reason


def test_buffer_is_bounded_and_reports_duplicate_and_overflow():
    buffer = ExactCameraPairBuffer(queue_capacity=2)

    buffer.add_image(_image(101))
    duplicate = buffer.add_image(_image(101))
    buffer.add_image(_image(102))
    overflow = buffer.add_image(_image(103))

    assert duplicate.rejections[0].reason == (
        CameraPairRejectionReason.DUPLICATE_IMAGE_STAMP
    )
    assert overflow.rejections[0].reason == (
        CameraPairRejectionReason.QUEUE_OVERFLOW
    )
    assert overflow.rejections[0].stamp_ns == 101
    assert buffer.pending_image_count == 2
    assert buffer.ready_pair_count == 0


def test_buffer_selects_earliest_compatible_pair_strictly_after_settle():
    buffer = ExactCameraPairBuffer(queue_capacity=4)
    for stamp in (300, 100, 200):
        buffer.add_image(_image(stamp))
        buffer.add_camera_info(_camera_info(stamp))

    selected = buffer.pop_first_after(100)
    next_selected = buffer.pop_first_after(100)

    assert selected is not None
    assert selected.pair is not None
    assert selected.pair.stamp_ns == 200
    assert next_selected is not None
    assert next_selected.pair is not None
    assert next_selected.pair.stamp_ns == 300
    assert buffer.pop_first_after(100) is None


def test_buffer_does_not_pair_different_stamps_and_bounds_both_streams():
    buffer = ExactCameraPairBuffer(queue_capacity=1)

    buffer.add_image(_image(100))
    update = buffer.add_camera_info(_camera_info(101))

    assert not update.pair_ready
    assert buffer.ready_pair_count == 0
    assert buffer.pending_image_count == 1
    assert buffer.pending_camera_info_count == 1


def test_port_waits_for_first_post_settle_pair_from_callbacks():
    port = ExactCameraPairPort(queue_capacity=4)
    port.push_image(_image(100))
    port.push_camera_info(_camera_info(100))

    def publish() -> None:
        time.sleep(0.01)
        port.push_camera_info(_camera_info(201))
        port.push_image(_image(201))

    publisher = threading.Thread(target=publish)
    publisher.start()
    try:
        pair = port.wait_for_first_compatible_frame(
            settled_stamp_ns=200,
            timeout_sec=0.5,
        )
    finally:
        publisher.join(timeout=1.0)

    assert pair.stamp_ns == 201


def test_port_timeout_uses_bounded_monotonic_wall_time():
    port = ExactCameraPairPort(queue_capacity=1)
    started = time.monotonic()

    with pytest.raises(CameraAcquisitionTimeout) as raised:
        port.wait_for_first_compatible_frame(
            settled_stamp_ns=10_000_000_000,
            timeout_sec=0.01,
        )

    assert time.monotonic() - started < 0.5
    assert raised.value.rejections == ()
