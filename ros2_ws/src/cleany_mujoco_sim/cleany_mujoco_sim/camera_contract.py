from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Any, Mapping, Sequence


CAMERA_NAME = 'left_wrist_rgb'
CAMERA_FRAME_ID = 'left_wrist_rgb_optical_frame'
CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480
CAMERA_FOVY_DEG = 93.0
CAMERA_PUBLISH_RATE_HZ = 10.0
CAMERA_FOCAL_LENGTH_PX = 227.751496
CAMERA_K = (
    CAMERA_FOCAL_LENGTH_PX,
    0.0,
    319.5,
    0.0,
    CAMERA_FOCAL_LENGTH_PX,
    239.5,
    0.0,
    0.0,
    1.0,
)
CAMERA_D = (0.0, 0.0, 0.0, 0.0, 0.0)
CAMERA_R = (1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0)
CAMERA_P = (
    CAMERA_FOCAL_LENGTH_PX,
    0.0,
    319.5,
    0.0,
    0.0,
    CAMERA_FOCAL_LENGTH_PX,
    239.5,
    0.0,
    0.0,
    0.0,
    1.0,
    0.0,
)
DISTORTION_MODEL = 'plumb_bob'
FOCAL_LENGTH_FORMULA = (
    'fy=(height/2)/tan(fovy_deg*pi/360); fx=fy; '
    'cx=(width-1)/2; cy=(height-1)/2'
)

VENDOR_IMAGE_TOPIC = '/left_wrist_rgb/color'
VENDOR_INFO_TOPIC = '/left_wrist_rgb/camera_info'
VENDOR_DEPTH_TOPIC = '/left_wrist_rgb/depth'
INTERNAL_IMAGE_TOPIC = '/cleany/internal/mujoco/left_wrist_camera/image_raw'
INTERNAL_INFO_TOPIC = '/cleany/internal/mujoco/left_wrist_camera/camera_info'
INTERNAL_DEPTH_TOPIC = '/cleany/internal/mujoco/left_wrist_camera/depth'
PUBLIC_IMAGE_TOPIC = '/left_wrist_camera/image_raw'
PUBLIC_INFO_TOPIC = '/left_wrist_camera/camera_info'


class CameraContractError(ValueError):
    """Raised when a camera manifest cannot guarantee the public contract."""


@dataclass(frozen=True)
class CameraContract:
    camera_name: str
    width: int
    height: int
    fovy_deg: float
    publish_rate_hz: float
    frame_id: str
    distortion_model: str
    d: tuple[float, ...]
    k: tuple[float, ...]
    r: tuple[float, ...]
    p: tuple[float, ...]
    focal_length_formula: str
    vendor_image_topic: str
    vendor_info_topic: str
    vendor_depth_topic: str
    internal_image_topic: str
    internal_info_topic: str
    internal_depth_topic: str
    public_image_topic: str
    public_info_topic: str


def focal_length_px(*, height: int, fovy_deg: float) -> float:
    return (float(height) / 2.0) / math.tan(math.radians(fovy_deg) / 2.0)


def _mapping(value: Any, label: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise CameraContractError(f'{label} must be a mapping')
    return value


def _number(value: Any, label: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise CameraContractError(f'{label} must be a number')
    result = float(value)
    if not math.isfinite(result):
        raise CameraContractError(f'{label} must be finite')
    return result


def _integer(value: Any, label: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise CameraContractError(f'{label} must be an integer')
    return value


def _string(value: Any, label: str) -> str:
    if not isinstance(value, str) or not value:
        raise CameraContractError(f'{label} must be a non-empty string')
    return value


def _vector(value: Any, length: int, label: str) -> tuple[float, ...]:
    if (
        not isinstance(value, Sequence)
        or isinstance(value, (str, bytes))
        or len(value) != length
    ):
        raise CameraContractError(f'{label} must contain {length} numbers')
    return tuple(
        _number(item, f'{label}[{index}]')
        for index, item in enumerate(value)
    )


def _assert_close(
    actual: Sequence[float],
    expected: Sequence[float],
    label: str,
    *,
    tolerance: float = 1.0e-9,
) -> None:
    if len(actual) != len(expected) or any(
        not math.isclose(float(left), float(right), abs_tol=tolerance)
        for left, right in zip(actual, expected, strict=True)
    ):
        raise CameraContractError(
            f'{label} must be {list(expected)!r}, got {list(actual)!r}'
        )


def camera_contract_from_scene(scene: Mapping[str, Any]) -> CameraContract:
    rendering = _mapping(
        scene.get('camera_rendering'),
        'scene.camera_rendering',
    )
    model = _mapping(rendering.get('model'), 'scene.camera_rendering.model')
    topics = _mapping(rendering.get('topics'), 'scene.camera_rendering.topics')
    vendor = _mapping(topics.get('vendor'), 'camera_rendering.topics.vendor')
    internal = _mapping(
        topics.get('internal'),
        'camera_rendering.topics.internal',
    )
    public = _mapping(topics.get('public'), 'camera_rendering.topics.public')

    width = _integer(rendering.get('width'), 'camera_rendering.width')
    height = _integer(rendering.get('height'), 'camera_rendering.height')
    fovy_deg = _number(rendering.get('fovy_deg'), 'camera_rendering.fovy_deg')
    publish_rate_hz = _number(
        rendering.get('publish_rate_hz'), 'camera_rendering.publish_rate_hz'
    )
    d = _vector(model.get('D'), 5, 'camera_rendering.model.D')
    k = _vector(model.get('K'), 9, 'camera_rendering.model.K')
    r = _vector(model.get('R'), 9, 'camera_rendering.model.R')
    p = _vector(model.get('P'), 12, 'camera_rendering.model.P')

    contract = CameraContract(
        camera_name=_string(
            rendering.get('camera_name'), 'camera_rendering.camera_name'
        ),
        width=width,
        height=height,
        fovy_deg=fovy_deg,
        publish_rate_hz=publish_rate_hz,
        frame_id=_string(
            model.get('frame_id'),
            'camera_rendering.model.frame_id',
        ),
        distortion_model=_string(
            model.get('distortion_model'),
            'camera_rendering.model.distortion_model',
        ),
        d=d,
        k=k,
        r=r,
        p=p,
        focal_length_formula=_string(
            model.get('focal_length_formula'),
            'camera_rendering.model.focal_length_formula',
        ),
        vendor_image_topic=_string(vendor.get('image'), 'vendor.image'),
        vendor_info_topic=_string(
            vendor.get('camera_info'),
            'vendor.camera_info',
        ),
        vendor_depth_topic=_string(vendor.get('depth'), 'vendor.depth'),
        internal_image_topic=_string(internal.get('image'), 'internal.image'),
        internal_info_topic=_string(
            internal.get('camera_info'), 'internal.camera_info'
        ),
        internal_depth_topic=_string(internal.get('depth'), 'internal.depth'),
        public_image_topic=_string(public.get('image'), 'public.image'),
        public_info_topic=_string(
            public.get('camera_info'),
            'public.camera_info',
        ),
    )
    validate_camera_contract(contract)
    return contract


def validate_camera_contract(contract: CameraContract) -> None:
    expected_scalars = {
        'camera_name': CAMERA_NAME,
        'width': CAMERA_WIDTH,
        'height': CAMERA_HEIGHT,
        'fovy_deg': CAMERA_FOVY_DEG,
        'publish_rate_hz': CAMERA_PUBLISH_RATE_HZ,
        'frame_id': CAMERA_FRAME_ID,
        'distortion_model': DISTORTION_MODEL,
        'focal_length_formula': FOCAL_LENGTH_FORMULA,
        'vendor_image_topic': VENDOR_IMAGE_TOPIC,
        'vendor_info_topic': VENDOR_INFO_TOPIC,
        'vendor_depth_topic': VENDOR_DEPTH_TOPIC,
        'internal_image_topic': INTERNAL_IMAGE_TOPIC,
        'internal_info_topic': INTERNAL_INFO_TOPIC,
        'internal_depth_topic': INTERNAL_DEPTH_TOPIC,
        'public_image_topic': PUBLIC_IMAGE_TOPIC,
        'public_info_topic': PUBLIC_INFO_TOPIC,
    }
    for field, expected in expected_scalars.items():
        actual = getattr(contract, field)
        if actual != expected:
            raise CameraContractError(
                f'camera contract {field} must be {expected!r}, got {actual!r}'
            )
    if contract.publish_rate_hz <= 0.0:
        raise CameraContractError('camera publish rate must be positive')

    calculated_focal_length = focal_length_px(
        height=contract.height,
        fovy_deg=contract.fovy_deg,
    )
    if not math.isclose(
        calculated_focal_length,
        CAMERA_FOCAL_LENGTH_PX,
        abs_tol=5.0e-7,
    ):
        raise CameraContractError(
            'camera FOV formula does not reproduce the declared focal length: '
            f'{calculated_focal_length}'
        )
    _assert_close(contract.d, CAMERA_D, 'camera D')
    _assert_close(contract.k, CAMERA_K, 'camera K', tolerance=5.0e-7)
    _assert_close(contract.r, CAMERA_R, 'camera R')
    _assert_close(contract.p, CAMERA_P, 'camera P', tolerance=5.0e-7)
