"""ROS-independent wrist-camera acquisition contracts.

ROS timestamps select data, while a monotonic wall clock bounds waiting.  The
module intentionally has no ROS imports so exact-pair and settle-gate behavior
can be exercised without creating a ROS graph.
"""

from __future__ import annotations

from collections import OrderedDict, deque
from dataclasses import dataclass
from enum import Enum
import math
import threading
import time
from typing import Callable, Protocol, Sequence


CAMERA_FRAME_ID = 'left_wrist_rgb_optical_frame'
CAMERA_ENCODING = 'rgb8'
CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480
CAMERA_DISTORTION_MODEL = 'plumb_bob'
CAMERA_D = (0.0, 0.0, 0.0, 0.0, 0.0)
CAMERA_K = (
    227.751496,
    0.0,
    319.5,
    0.0,
    227.751496,
    239.5,
    0.0,
    0.0,
    1.0,
)
CAMERA_R = (
    1.0,
    0.0,
    0.0,
    0.0,
    1.0,
    0.0,
    0.0,
    0.0,
    1.0,
)
CAMERA_P = (
    227.751496,
    0.0,
    319.5,
    0.0,
    0.0,
    227.751496,
    239.5,
    0.0,
    0.0,
    0.0,
    1.0,
    0.0,
)


def _positive_integer(value: int, *, field_name: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
        raise ValueError(f'{field_name} must be a positive integer')
    return value


def _non_negative_integer(value: int, *, field_name: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        raise ValueError(f'{field_name} must be a non-negative integer')
    return value


def _text(value: str, *, field_name: str) -> str:
    if not isinstance(value, str) or not value or value != value.strip():
        raise ValueError(f'{field_name} must be a non-empty trimmed string')
    return value


def _finite_tuple(
    values: Sequence[float],
    *,
    field_name: str,
    length: int,
) -> tuple[float, ...]:
    try:
        result = tuple(float(value) for value in values)
    except (TypeError, ValueError) as error:
        raise ValueError(f'{field_name} must be numeric') from error
    if len(result) != length:
        raise ValueError(
            f'{field_name} must contain {length} values, got {len(result)}'
        )
    if not all(math.isfinite(value) for value in result):
        raise ValueError(f'{field_name} must contain only finite values')
    return result


@dataclass(frozen=True, slots=True)
class CameraContract:
    """Expected runtime CameraInfo and raw RGB image profile."""

    frame_id: str = CAMERA_FRAME_ID
    encoding: str = CAMERA_ENCODING
    width: int = CAMERA_WIDTH
    height: int = CAMERA_HEIGHT
    distortion_model: str = CAMERA_DISTORTION_MODEL
    d: tuple[float, ...] = CAMERA_D
    k: tuple[float, ...] = CAMERA_K
    r: tuple[float, ...] = CAMERA_R
    p: tuple[float, ...] = CAMERA_P
    calibration_tolerance: float = 1.0e-6

    def __post_init__(self) -> None:
        object.__setattr__(
            self,
            'frame_id',
            _text(self.frame_id, field_name='frame_id'),
        )
        object.__setattr__(
            self,
            'encoding',
            _text(self.encoding, field_name='encoding'),
        )
        object.__setattr__(
            self,
            'width',
            _positive_integer(self.width, field_name='width'),
        )
        object.__setattr__(
            self,
            'height',
            _positive_integer(self.height, field_name='height'),
        )
        object.__setattr__(
            self,
            'distortion_model',
            _text(
                self.distortion_model,
                field_name='distortion_model',
            ),
        )
        object.__setattr__(
            self,
            'd',
            _finite_tuple(self.d, field_name='D', length=5),
        )
        object.__setattr__(
            self,
            'k',
            _finite_tuple(self.k, field_name='K', length=9),
        )
        object.__setattr__(
            self,
            'r',
            _finite_tuple(self.r, field_name='R', length=9),
        )
        object.__setattr__(
            self,
            'p',
            _finite_tuple(self.p, field_name='P', length=12),
        )
        try:
            tolerance = float(self.calibration_tolerance)
        except (TypeError, ValueError) as error:
            raise ValueError(
                'calibration_tolerance must be numeric'
            ) from error
        if not math.isfinite(tolerance) or tolerance < 0.0:
            raise ValueError(
                'calibration_tolerance must be finite and non-negative'
            )
        object.__setattr__(self, 'calibration_tolerance', tolerance)

    @property
    def bytes_per_pixel(self) -> int:
        if self.encoding in {'rgb8', 'bgr8'}:
            return 3
        if self.encoding == 'mono8':
            return 1
        raise ValueError(
            f'unsupported raw image encoding: {self.encoding!r}'
        )

    @property
    def step(self) -> int:
        return self.width * self.bytes_per_pixel


DEFAULT_CAMERA_CONTRACT = CameraContract()


@dataclass(frozen=True, slots=True)
class ImageFrame:
    """A copied ROS Image payload at the core boundary."""

    stamp_ns: int
    frame_id: str
    width: int
    height: int
    encoding: str
    is_bigendian: bool
    step: int
    data: bytes

    def __post_init__(self) -> None:
        object.__setattr__(
            self,
            'stamp_ns',
            _non_negative_integer(self.stamp_ns, field_name='stamp_ns'),
        )
        object.__setattr__(
            self,
            'frame_id',
            _text(self.frame_id, field_name='frame_id'),
        )
        object.__setattr__(
            self,
            'width',
            _positive_integer(self.width, field_name='width'),
        )
        object.__setattr__(
            self,
            'height',
            _positive_integer(self.height, field_name='height'),
        )
        object.__setattr__(
            self,
            'encoding',
            _text(self.encoding, field_name='encoding'),
        )
        if not isinstance(self.is_bigendian, bool):
            raise ValueError('is_bigendian must be a bool')
        object.__setattr__(
            self,
            'step',
            _non_negative_integer(self.step, field_name='step'),
        )
        try:
            payload = bytes(self.data)
        except (TypeError, ValueError) as error:
            raise ValueError('data must be bytes-like') from error
        object.__setattr__(self, 'data', payload)


@dataclass(frozen=True, slots=True)
class CameraInfoFrame:
    """CameraInfo fields that affect image geometry and PnP."""

    stamp_ns: int
    frame_id: str
    width: int
    height: int
    distortion_model: str
    d: tuple[float, ...]
    k: tuple[float, ...]
    r: tuple[float, ...]
    p: tuple[float, ...]

    def __post_init__(self) -> None:
        object.__setattr__(
            self,
            'stamp_ns',
            _non_negative_integer(self.stamp_ns, field_name='stamp_ns'),
        )
        object.__setattr__(
            self,
            'frame_id',
            _text(self.frame_id, field_name='frame_id'),
        )
        object.__setattr__(
            self,
            'width',
            _positive_integer(self.width, field_name='width'),
        )
        object.__setattr__(
            self,
            'height',
            _positive_integer(self.height, field_name='height'),
        )
        object.__setattr__(
            self,
            'distortion_model',
            _text(
                self.distortion_model,
                field_name='distortion_model',
            ),
        )
        object.__setattr__(
            self,
            'd',
            _finite_tuple(self.d, field_name='D', length=5),
        )
        object.__setattr__(
            self,
            'k',
            _finite_tuple(self.k, field_name='K', length=9),
        )
        object.__setattr__(
            self,
            'r',
            _finite_tuple(self.r, field_name='R', length=9),
        )
        object.__setattr__(
            self,
            'p',
            _finite_tuple(self.p, field_name='P', length=12),
        )


@dataclass(frozen=True, slots=True)
class CameraFramePair:
    image: ImageFrame
    camera_info: CameraInfoFrame

    def __post_init__(self) -> None:
        if not isinstance(self.image, ImageFrame):
            raise ValueError('image must be an ImageFrame')
        if not isinstance(self.camera_info, CameraInfoFrame):
            raise ValueError('camera_info must be a CameraInfoFrame')
        if self.image.stamp_ns != self.camera_info.stamp_ns:
            raise ValueError('Image and CameraInfo stamps must match exactly')
        if self.image.frame_id != self.camera_info.frame_id:
            raise ValueError('Image and CameraInfo frame IDs must match')

    @property
    def stamp_ns(self) -> int:
        return self.image.stamp_ns


class CameraPairRejectionReason(str, Enum):
    ZERO_STAMP = 'zero_stamp'
    STAMP_MISMATCH = 'stamp_mismatch'
    FRAME_ID_MISMATCH = 'frame_id_mismatch'
    IMAGE_ENCODING_MISMATCH = 'image_encoding_mismatch'
    IMAGE_DIMENSIONS_MISMATCH = 'image_dimensions_mismatch'
    IMAGE_ENDIANNESS_MISMATCH = 'image_endianness_mismatch'
    IMAGE_STEP_MISMATCH = 'image_step_mismatch'
    IMAGE_DATA_LENGTH_MISMATCH = 'image_data_length_mismatch'
    CAMERA_INFO_DIMENSIONS_MISMATCH = 'camera_info_dimensions_mismatch'
    DISTORTION_MODEL_MISMATCH = 'distortion_model_mismatch'
    CAMERA_K_MISMATCH = 'camera_k_mismatch'
    CAMERA_D_MISMATCH = 'camera_d_mismatch'
    CAMERA_R_MISMATCH = 'camera_r_mismatch'
    CAMERA_P_MISMATCH = 'camera_p_mismatch'
    BEFORE_OR_AT_SETTLE = 'before_or_at_settle'
    DUPLICATE_IMAGE_STAMP = 'duplicate_image_stamp'
    DUPLICATE_CAMERA_INFO_STAMP = 'duplicate_camera_info_stamp'
    QUEUE_OVERFLOW = 'queue_overflow'


@dataclass(frozen=True, slots=True)
class CameraPairRejection:
    reason: CameraPairRejectionReason
    detail: str
    stamp_ns: int | None = None


@dataclass(frozen=True, slots=True)
class CameraPairValidation:
    pair: CameraFramePair | None = None
    rejection: CameraPairRejection | None = None

    def __post_init__(self) -> None:
        if (self.pair is None) == (self.rejection is None):
            raise ValueError('exactly one of pair or rejection is required')

    @property
    def compatible(self) -> bool:
        return self.pair is not None


def _mismatch(
    reason: CameraPairRejectionReason,
    detail: str,
    *,
    stamp_ns: int | None,
) -> CameraPairValidation:
    return CameraPairValidation(
        rejection=CameraPairRejection(reason, detail, stamp_ns)
    )


def _calibration_matches(
    actual: Sequence[float],
    expected: Sequence[float],
    *,
    tolerance: float,
) -> bool:
    return len(actual) == len(expected) and all(
        math.isclose(
            left,
            right,
            rel_tol=0.0,
            abs_tol=tolerance,
        )
        for left, right in zip(actual, expected, strict=True)
    )


def validate_camera_pair(
    image: ImageFrame,
    camera_info: CameraInfoFrame,
    *,
    contract: CameraContract = DEFAULT_CAMERA_CONTRACT,
    settled_stamp_ns: int | None = None,
) -> CameraPairValidation:
    """Return an exact compatible pair or one explicit rejection reason."""

    if not isinstance(image, ImageFrame):
        raise ValueError('image must be an ImageFrame')
    if not isinstance(camera_info, CameraInfoFrame):
        raise ValueError('camera_info must be a CameraInfoFrame')
    if not isinstance(contract, CameraContract):
        raise ValueError('contract must be a CameraContract')
    if settled_stamp_ns is not None:
        settled_stamp_ns = _non_negative_integer(
            settled_stamp_ns,
            field_name='settled_stamp_ns',
        )
    stamp = image.stamp_ns
    if stamp == 0 or camera_info.stamp_ns == 0:
        return _mismatch(
            CameraPairRejectionReason.ZERO_STAMP,
            'Image and CameraInfo require nonzero ROS stamps',
            stamp_ns=stamp,
        )
    if stamp != camera_info.stamp_ns:
        return _mismatch(
            CameraPairRejectionReason.STAMP_MISMATCH,
            'Image and CameraInfo stamps must match exactly',
            stamp_ns=stamp,
        )
    if (
        image.frame_id != camera_info.frame_id
        or image.frame_id != contract.frame_id
    ):
        return _mismatch(
            CameraPairRejectionReason.FRAME_ID_MISMATCH,
            'Image and CameraInfo must use '
            f'{contract.frame_id!r}',
            stamp_ns=stamp,
        )
    if image.encoding != contract.encoding:
        return _mismatch(
            CameraPairRejectionReason.IMAGE_ENCODING_MISMATCH,
            f'Image encoding must be {contract.encoding!r}',
            stamp_ns=stamp,
        )
    if (image.width, image.height) != (contract.width, contract.height):
        return _mismatch(
            CameraPairRejectionReason.IMAGE_DIMENSIONS_MISMATCH,
            'Image dimensions do not match the camera contract',
            stamp_ns=stamp,
        )
    if image.is_bigendian:
        return _mismatch(
            CameraPairRejectionReason.IMAGE_ENDIANNESS_MISMATCH,
            '8-bit RGB image must use little/native byte order',
            stamp_ns=stamp,
        )
    if image.step != contract.step:
        return _mismatch(
            CameraPairRejectionReason.IMAGE_STEP_MISMATCH,
            f'Image step must be {contract.step}',
            stamp_ns=stamp,
        )
    expected_length = image.step * image.height
    if len(image.data) != expected_length:
        return _mismatch(
            CameraPairRejectionReason.IMAGE_DATA_LENGTH_MISMATCH,
            f'Image data length must be {expected_length}',
            stamp_ns=stamp,
        )
    if (camera_info.width, camera_info.height) != (
        contract.width,
        contract.height,
    ):
        return _mismatch(
            CameraPairRejectionReason.CAMERA_INFO_DIMENSIONS_MISMATCH,
            'CameraInfo dimensions do not match the camera contract',
            stamp_ns=stamp,
        )
    if camera_info.distortion_model != contract.distortion_model:
        return _mismatch(
            CameraPairRejectionReason.DISTORTION_MODEL_MISMATCH,
            'CameraInfo distortion model does not match the contract',
            stamp_ns=stamp,
        )
    calibration_fields = (
        ('K', camera_info.k, contract.k,
         CameraPairRejectionReason.CAMERA_K_MISMATCH),
        ('D', camera_info.d, contract.d,
         CameraPairRejectionReason.CAMERA_D_MISMATCH),
        ('R', camera_info.r, contract.r,
         CameraPairRejectionReason.CAMERA_R_MISMATCH),
        ('P', camera_info.p, contract.p,
         CameraPairRejectionReason.CAMERA_P_MISMATCH),
    )
    for name, actual, expected, reason in calibration_fields:
        if not _calibration_matches(
            actual,
            expected,
            tolerance=contract.calibration_tolerance,
        ):
            return _mismatch(
                reason,
                f'CameraInfo {name} does not match the camera contract',
                stamp_ns=stamp,
            )
    if settled_stamp_ns is not None and stamp <= settled_stamp_ns:
        return _mismatch(
            CameraPairRejectionReason.BEFORE_OR_AT_SETTLE,
            'Frame stamp must be strictly after settle completion',
            stamp_ns=stamp,
        )
    return CameraPairValidation(pair=CameraFramePair(image, camera_info))


@dataclass(frozen=True, slots=True)
class CameraQueueUpdate:
    pair_ready: bool
    rejections: tuple[CameraPairRejection, ...] = ()


class ExactCameraPairBuffer:
    """Bounded exact-stamp Image/CameraInfo queues."""

    def __init__(
        self,
        *,
        contract: CameraContract = DEFAULT_CAMERA_CONTRACT,
        queue_capacity: int = 8,
    ) -> None:
        if not isinstance(contract, CameraContract):
            raise ValueError('contract must be a CameraContract')
        self._contract = contract
        self._capacity = _positive_integer(
            queue_capacity,
            field_name='queue_capacity',
        )
        self._images: OrderedDict[int, ImageFrame] = OrderedDict()
        self._camera_infos: OrderedDict[int, CameraInfoFrame] = OrderedDict()
        self._ready: deque[CameraFramePair] = deque()

    @property
    def queue_capacity(self) -> int:
        return self._capacity

    @property
    def pending_image_count(self) -> int:
        return len(self._images)

    @property
    def pending_camera_info_count(self) -> int:
        return len(self._camera_infos)

    @property
    def ready_pair_count(self) -> int:
        return len(self._ready)

    def _overflow_rejection(
        self,
        *,
        stream_name: str,
        stamp_ns: int,
    ) -> CameraPairRejection:
        return CameraPairRejection(
            CameraPairRejectionReason.QUEUE_OVERFLOW,
            f'dropped oldest unmatched {stream_name} from bounded queue',
            stamp_ns,
        )

    def add_image(self, image: ImageFrame) -> CameraQueueUpdate:
        if not isinstance(image, ImageFrame):
            raise ValueError('image must be an ImageFrame')
        if image.stamp_ns in self._images:
            return CameraQueueUpdate(
                False,
                (CameraPairRejection(
                    CameraPairRejectionReason.DUPLICATE_IMAGE_STAMP,
                    'duplicate Image stamp was rejected',
                    image.stamp_ns,
                ),),
            )
        camera_info = self._camera_infos.pop(image.stamp_ns, None)
        if camera_info is not None:
            validation = validate_camera_pair(
                image,
                camera_info,
                contract=self._contract,
            )
            if validation.pair is not None:
                rejections = self._append_ready(validation.pair)
                return CameraQueueUpdate(True, rejections)
            assert validation.rejection is not None
            return CameraQueueUpdate(False, (validation.rejection,))
        self._images[image.stamp_ns] = image
        rejections: tuple[CameraPairRejection, ...] = ()
        if len(self._images) > self._capacity:
            dropped_stamp, _ = self._images.popitem(last=False)
            rejections = (
                self._overflow_rejection(
                    stream_name='Image',
                    stamp_ns=dropped_stamp,
                ),
            )
        return CameraQueueUpdate(False, rejections)

    def add_camera_info(
        self,
        camera_info: CameraInfoFrame,
    ) -> CameraQueueUpdate:
        if not isinstance(camera_info, CameraInfoFrame):
            raise ValueError('camera_info must be a CameraInfoFrame')
        if camera_info.stamp_ns in self._camera_infos:
            return CameraQueueUpdate(
                False,
                (CameraPairRejection(
                    CameraPairRejectionReason.DUPLICATE_CAMERA_INFO_STAMP,
                    'duplicate CameraInfo stamp was rejected',
                    camera_info.stamp_ns,
                ),),
            )
        image = self._images.pop(camera_info.stamp_ns, None)
        if image is not None:
            validation = validate_camera_pair(
                image,
                camera_info,
                contract=self._contract,
            )
            if validation.pair is not None:
                rejections = self._append_ready(validation.pair)
                return CameraQueueUpdate(True, rejections)
            assert validation.rejection is not None
            return CameraQueueUpdate(False, (validation.rejection,))
        self._camera_infos[camera_info.stamp_ns] = camera_info
        rejections: tuple[CameraPairRejection, ...] = ()
        if len(self._camera_infos) > self._capacity:
            dropped_stamp, _ = self._camera_infos.popitem(last=False)
            rejections = (
                self._overflow_rejection(
                    stream_name='CameraInfo',
                    stamp_ns=dropped_stamp,
                ),
            )
        return CameraQueueUpdate(False, rejections)

    def _append_ready(
        self,
        pair: CameraFramePair,
    ) -> tuple[CameraPairRejection, ...]:
        self._ready.append(pair)
        if len(self._ready) <= self._capacity:
            return ()
        dropped = self._ready.popleft()
        return (
            self._overflow_rejection(
                stream_name='compatible pair',
                stamp_ns=dropped.stamp_ns,
            ),
        )

    def pop_first_after(
        self,
        settled_stamp_ns: int,
    ) -> CameraPairValidation | None:
        """Discard pre-settle pairs and return the earliest eligible pair."""

        settled = _non_negative_integer(
            settled_stamp_ns,
            field_name='settled_stamp_ns',
        )
        eligible: list[CameraFramePair] = []
        while self._ready:
            pair = self._ready.popleft()
            if pair.stamp_ns > settled:
                eligible.append(pair)
        if not eligible:
            return None
        eligible.sort(key=lambda pair: pair.stamp_ns)
        selected = eligible.pop(0)
        self._ready.extend(eligible)
        return validate_camera_pair(
            selected.image,
            selected.camera_info,
            contract=self._contract,
            settled_stamp_ns=settled,
        )


class MonotonicDeadline:
    """Injectable monotonic deadline used only for bounded wall waits."""

    def __init__(
        self,
        timeout_sec: float,
        *,
        monotonic: Callable[[], float] = time.monotonic,
    ) -> None:
        try:
            timeout = float(timeout_sec)
        except (TypeError, ValueError) as error:
            raise ValueError('timeout_sec must be numeric') from error
        if not math.isfinite(timeout) or timeout <= 0.0:
            raise ValueError('timeout_sec must be finite and positive')
        if not callable(monotonic):
            raise ValueError('monotonic must be callable')
        self._monotonic = monotonic
        started = float(monotonic())
        if not math.isfinite(started):
            raise ValueError('monotonic clock returned a non-finite value')
        self._deadline = started + timeout

    def remaining_sec(self) -> float:
        now = float(self._monotonic())
        if not math.isfinite(now):
            raise RuntimeError('monotonic clock returned a non-finite value')
        return max(0.0, self._deadline - now)

    @property
    def expired(self) -> bool:
        return self.remaining_sec() <= 0.0


class CameraAcquisitionTimeout(TimeoutError):
    """No compatible post-settle pair arrived before the wall deadline."""

    def __init__(
        self,
        message: str,
        *,
        rejections: Sequence[CameraPairRejection] = (),
    ) -> None:
        super().__init__(message)
        self.rejections = tuple(rejections)


class CameraAcquisitionPort(Protocol):
    def wait_for_first_compatible_frame(
        self,
        *,
        settled_stamp_ns: int,
        timeout_sec: float,
    ) -> CameraFramePair:
        """Return the first valid exact pair strictly after settle."""


class ExactCameraPairPort:
    """Thread-safe callback-to-blocking-port bridge with bounded queues."""

    def __init__(
        self,
        *,
        contract: CameraContract = DEFAULT_CAMERA_CONTRACT,
        queue_capacity: int = 8,
        rejection_capacity: int = 32,
        monotonic: Callable[[], float] = time.monotonic,
    ) -> None:
        self._buffer = ExactCameraPairBuffer(
            contract=contract,
            queue_capacity=queue_capacity,
        )
        self._condition = threading.Condition()
        self._rejections: deque[CameraPairRejection] = deque(
            maxlen=_positive_integer(
                rejection_capacity,
                field_name='rejection_capacity',
            )
        )
        self._monotonic = monotonic

    @property
    def recent_rejections(self) -> tuple[CameraPairRejection, ...]:
        with self._condition:
            return tuple(self._rejections)

    def push_image(self, image: ImageFrame) -> CameraQueueUpdate:
        with self._condition:
            update = self._buffer.add_image(image)
            self._rejections.extend(update.rejections)
            self._condition.notify_all()
            return update

    def push_camera_info(
        self,
        camera_info: CameraInfoFrame,
    ) -> CameraQueueUpdate:
        with self._condition:
            update = self._buffer.add_camera_info(camera_info)
            self._rejections.extend(update.rejections)
            self._condition.notify_all()
            return update

    def wait_for_first_compatible_frame(
        self,
        *,
        settled_stamp_ns: int,
        timeout_sec: float,
    ) -> CameraFramePair:
        settled = _non_negative_integer(
            settled_stamp_ns,
            field_name='settled_stamp_ns',
        )
        deadline = MonotonicDeadline(
            timeout_sec,
            monotonic=self._monotonic,
        )
        with self._condition:
            while True:
                validation = self._buffer.pop_first_after(settled)
                if validation is not None:
                    if validation.pair is not None:
                        return validation.pair
                    assert validation.rejection is not None
                    self._rejections.append(validation.rejection)
                remaining = deadline.remaining_sec()
                if remaining <= 0.0:
                    raise CameraAcquisitionTimeout(
                        'timed out waiting for a compatible Image/'
                        'CameraInfo pair after settle completion',
                        rejections=self._rejections,
                    )
                self._condition.wait(timeout=remaining)
