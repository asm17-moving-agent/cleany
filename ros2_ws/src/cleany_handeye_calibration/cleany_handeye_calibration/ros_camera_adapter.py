"""Small ROS-message boundary for exact wrist-camera acquisition.

The conversion functions are duck typed on purpose: importing this module and
testing conversion does not require a ROS graph or even generated message type
support.  A node only needs to connect its two subscription callbacks to
``on_image`` and ``on_camera_info``.
"""

from __future__ import annotations

from typing import Any, Callable

from cleany_handeye_calibration.camera_acquisition import (
    CameraContract,
    CameraFramePair,
    CameraQueueUpdate,
    DEFAULT_CAMERA_CONTRACT,
    ExactCameraPairPort,
    ImageFrame,
    CameraInfoFrame,
)


NANOSECONDS_PER_SECOND = 1_000_000_000


class RosCameraMessageError(ValueError):
    """A ROS message cannot be represented by the acquisition core."""


def ros_stamp_to_ns(stamp: Any) -> int:
    """Convert builtin_interfaces/Time-like ``sec``/``nanosec`` fields."""

    try:
        sec = stamp.sec
        nanosec = stamp.nanosec
    except AttributeError as error:
        raise RosCameraMessageError(
            'ROS stamp must expose sec and nanosec'
        ) from error
    if (
        isinstance(sec, bool)
        or not isinstance(sec, int)
        or sec < 0
    ):
        raise RosCameraMessageError(
            'ROS stamp sec must be a non-negative integer'
        )
    if (
        isinstance(nanosec, bool)
        or not isinstance(nanosec, int)
        or not 0 <= nanosec < NANOSECONDS_PER_SECOND
    ):
        raise RosCameraMessageError(
            'ROS stamp nanosec must be in [0, 1000000000)'
        )
    return sec * NANOSECONDS_PER_SECOND + nanosec


def image_frame_from_ros(message: Any) -> ImageFrame:
    """Copy a sensor_msgs/Image-like object into the pure core model."""

    try:
        header = message.header
        endian_value = message.is_bigendian
        if endian_value not in (False, True, 0, 1):
            raise RosCameraMessageError(
                'Image is_bigendian must be bool-compatible 0 or 1'
            )
        return ImageFrame(
            stamp_ns=ros_stamp_to_ns(header.stamp),
            frame_id=header.frame_id,
            width=message.width,
            height=message.height,
            encoding=message.encoding,
            is_bigendian=bool(endian_value),
            step=message.step,
            data=message.data,
        )
    except RosCameraMessageError:
        raise
    except (AttributeError, TypeError, ValueError) as error:
        raise RosCameraMessageError(
            f'invalid ROS Image message: {error}'
        ) from error


def camera_info_frame_from_ros(message: Any) -> CameraInfoFrame:
    """Copy geometry-affecting sensor_msgs/CameraInfo fields."""

    try:
        header = message.header
        return CameraInfoFrame(
            stamp_ns=ros_stamp_to_ns(header.stamp),
            frame_id=header.frame_id,
            width=message.width,
            height=message.height,
            distortion_model=message.distortion_model,
            d=message.d,
            k=message.k,
            r=message.r,
            p=message.p,
        )
    except RosCameraMessageError:
        raise
    except (AttributeError, TypeError, ValueError) as error:
        raise RosCameraMessageError(
            f'invalid ROS CameraInfo message: {error}'
        ) from error


class RosExactCameraPairAdapter:
    """ROS callback adapter implementing the camera acquisition port."""

    def __init__(
        self,
        *,
        contract: CameraContract = DEFAULT_CAMERA_CONTRACT,
        queue_capacity: int = 8,
        rejection_capacity: int = 32,
        monotonic: Callable[[], float] | None = None,
    ) -> None:
        options: dict[str, Any] = {
            'contract': contract,
            'queue_capacity': queue_capacity,
            'rejection_capacity': rejection_capacity,
        }
        if monotonic is not None:
            options['monotonic'] = monotonic
        self._port = ExactCameraPairPort(**options)

    @property
    def port(self) -> ExactCameraPairPort:
        return self._port

    def on_image(self, message: Any) -> CameraQueueUpdate:
        return self._port.push_image(image_frame_from_ros(message))

    def on_camera_info(self, message: Any) -> CameraQueueUpdate:
        return self._port.push_camera_info(
            camera_info_frame_from_ros(message)
        )

    def wait_for_first_compatible_frame(
        self,
        *,
        settled_stamp_ns: int,
        timeout_sec: float,
    ) -> CameraFramePair:
        return self._port.wait_for_first_compatible_frame(
            settled_stamp_ns=settled_stamp_ns,
            timeout_sec=timeout_sec,
        )
