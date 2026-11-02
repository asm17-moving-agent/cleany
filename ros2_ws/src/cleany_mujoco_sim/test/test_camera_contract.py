from __future__ import annotations

from copy import deepcopy
from pathlib import Path

import pytest
import yaml
from sensor_msgs.msg import Image

from cleany_mujoco_sim.camera_contract import (
    CAMERA_D,
    CAMERA_FOCAL_LENGTH_PX,
    CAMERA_K,
    CAMERA_P,
    CAMERA_R,
    CameraContractError,
    camera_contract_from_scene,
    focal_length_px,
    load_camera_contract,
)
from cleany_mujoco_sim.camera_contract_adapter import camera_info_for_image


PACKAGE_ROOT = Path(__file__).parents[1]
CAMERA_CONFIG = PACKAGE_ROOT / 'config' / 'wrist_camera.yaml'


def test_camera_contract_is_exact_and_formula_derived() -> None:
    camera = load_camera_contract(CAMERA_CONFIG)
    assert (camera.width, camera.height, camera.fovy_deg) == (640, 480, 93.0)
    assert camera.frame_id == 'left_wrist_rgb_optical_frame'
    assert camera.distortion_model == 'plumb_bob'
    assert camera.d == CAMERA_D == (0.0, 0.0, 0.0, 0.0, 0.0)
    assert camera.k == CAMERA_K
    assert camera.r == CAMERA_R
    assert camera.p == CAMERA_P
    assert focal_length_px(
        height=camera.height,
        fovy_deg=camera.fovy_deg,
    ) == pytest.approx(CAMERA_FOCAL_LENGTH_PX, abs=5.0e-7)
    assert camera.public_image_topic == '/left_wrist_camera/image_raw'
    assert camera.public_info_topic == '/left_wrist_camera/camera_info'
    assert camera.vendor_image_topic != camera.public_image_topic
    assert camera.internal_image_topic.startswith('/cleany/internal/mujoco/')


@pytest.mark.parametrize(
    ('path', 'bad_value', 'match'),
    (
        (('width',), 320, 'width'),
        (('height',), 240, 'height'),
        (('fovy_deg',), 90.0, 'fovy_deg'),
        (('model', 'D'), [0.0] * 4, '5 numbers'),
        (('model', 'K'), [1.0] * 9, 'camera K'),
        (('model', 'R'), [0.0] * 9, 'camera R'),
        (('model', 'P'), [0.0] * 12, 'camera P'),
    ),
)
def test_camera_contract_mismatch_blocks_preflight(
    path: tuple[str, ...],
    bad_value: object,
    match: str,
) -> None:
    data = yaml.safe_load(CAMERA_CONFIG.read_text(encoding='utf-8'))
    rendering = deepcopy(data['camera_rendering'])
    destination = rendering
    for key in path[:-1]:
        destination = destination[key]
    destination[path[-1]] = bad_value
    with pytest.raises(CameraContractError, match=match):
        camera_contract_from_scene({'camera_rendering': rendering})


def test_camera_info_normalizer_preserves_source_stamp_exactly() -> None:
    contract = load_camera_contract(CAMERA_CONFIG)
    image = Image()
    image.header.stamp.sec = 123
    image.header.stamp.nanosec = 456789
    image.header.frame_id = 'vendor_frame'
    image.width = 640
    image.height = 480
    info = camera_info_for_image(image, contract)
    assert (info.header.stamp.sec, info.header.stamp.nanosec) == (
        123,
        456789,
    )
    assert image.header.frame_id == 'vendor_frame'
    assert info.header.frame_id == 'left_wrist_rgb_optical_frame'
    assert (info.width, info.height) == (640, 480)
    assert info.distortion_model == 'plumb_bob'
    assert tuple(info.d) == CAMERA_D
    assert tuple(info.k) == CAMERA_K
    assert tuple(info.r) == CAMERA_R
    assert tuple(info.p) == CAMERA_P
