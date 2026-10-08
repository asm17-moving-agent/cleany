from __future__ import annotations

from copy import deepcopy
import math
from pathlib import Path

import mujoco
import numpy as np
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
)
from cleany_mujoco_sim.camera_contract_adapter import camera_info_for_image
from cleany_mujoco_sim.ground_truth_evaluation import (
    RigidTransform,
    camera_ground_truth,
    evaluate_transform,
)
from cleany_mujoco_sim.scene_loader import materialize_scene
from cleany_mujoco_sim.scene_manifest import load_handeye_scene_manifest


PACKAGE_ROOT = Path(__file__).parents[1]
WORKSPACE_SOURCE = PACKAGE_ROOT.parent
DESCRIPTION_MODEL = (
    WORKSPACE_SOURCE / 'cleany_description' / 'mjcf' / 'cleany.xml'
)
HAND_EYE_SCENE = PACKAGE_ROOT / 'scenes' / 'handeye.xml.in'
DEFAULT_SCENE = PACKAGE_ROOT / 'scenes' / 'default.xml.in'
MANIFEST = PACKAGE_ROOT / 'config' / 'handeye_scene.yaml'


def _quaternion_matrix_xyzw(values: tuple[float, ...]) -> np.ndarray:
    x, y, z, w = values
    return np.asarray(
        [
            [
                1.0 - 2.0 * (y * y + z * z),
                2.0 * (x * y - z * w),
                2.0 * (x * z + y * w),
            ],
            [
                2.0 * (x * y + z * w),
                1.0 - 2.0 * (x * x + z * z),
                2.0 * (y * z - x * w),
            ],
            [
                2.0 * (x * z - y * w),
                2.0 * (y * z + x * w),
                1.0 - 2.0 * (x * x + y * y),
            ],
        ]
    )


def test_manifest_camera_contract_is_exact_and_formula_derived() -> None:
    manifest = load_handeye_scene_manifest(MANIFEST)
    camera = manifest.camera_contract
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
    data = yaml.safe_load(MANIFEST.read_text(encoding='utf-8'))
    rendering = deepcopy(data['scene']['camera_rendering'])
    destination = rendering
    for key in path[:-1]:
        destination = destination[key]
    destination[path[-1]] = bad_value
    with pytest.raises(CameraContractError, match=match):
        camera_contract_from_scene({'camera_rendering': rendering})


def test_camera_info_normalizer_preserves_source_stamp_exactly() -> None:
    contract = load_handeye_scene_manifest(MANIFEST).camera_contract
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


def test_handeye_materialization_sets_only_temporary_camera_resolution(
) -> None:
    canonical_before = DESCRIPTION_MODEL.read_bytes()
    default_before = DEFAULT_SCENE.read_bytes()
    handeye_path = materialize_scene(HAND_EYE_SCENE)
    handeye_model = mujoco.MjModel.from_xml_path(str(handeye_path))
    camera_id = mujoco.mj_name2id(
        handeye_model,
        mujoco.mjtObj.mjOBJ_CAMERA,
        'left_wrist_rgb',
    )
    assert tuple(handeye_model.cam_resolution[camera_id]) == (640, 480)
    assert handeye_model.cam_fovy[camera_id] == pytest.approx(93.0)

    default_path = materialize_scene(DEFAULT_SCENE)
    default_model = mujoco.MjModel.from_xml_path(str(default_path))
    default_camera_id = mujoco.mj_name2id(
        default_model,
        mujoco.mjtObj.mjOBJ_CAMERA,
        'left_wrist_rgb',
    )
    assert tuple(default_model.cam_resolution[default_camera_id]) != (
        640,
        480,
    )
    assert DESCRIPTION_MODEL.read_bytes() == canonical_before
    assert DEFAULT_SCENE.read_bytes() == default_before


def test_camera_gt_matches_compiled_optical_site_and_is_evaluation_only(
) -> None:
    manifest = load_handeye_scene_manifest(MANIFEST)
    truth = camera_ground_truth(manifest.data)
    model = mujoco.MjModel.from_xml_path(
        str(materialize_scene(HAND_EYE_SCENE))
    )
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    body_id = mujoco.mj_name2id(
        model,
        mujoco.mjtObj.mjOBJ_BODY,
        'Fixed_Jaw',
    )
    site_id = mujoco.mj_name2id(
        model,
        mujoco.mjtObj.mjOBJ_SITE,
        'left_wrist_rgb_optical_frame',
    )
    body_rotation = data.xmat[body_id].reshape(3, 3)
    compiled_translation = body_rotation.T @ (
        data.site_xpos[site_id] - data.xpos[body_id]
    )
    compiled_rotation = (
        body_rotation.T @ data.site_xmat[site_id].reshape(3, 3)
    )
    assert truth.parent_frame == 'left_gripper_frame'
    assert truth.child_frame == 'left_wrist_rgb_optical_frame'
    assert compiled_translation == pytest.approx(
        truth.translation_m,
        abs=1.0e-12,
    )
    assert compiled_rotation == pytest.approx(
        _quaternion_matrix_xyzw(truth.quaternion_xyzw),
        abs=1.0e-12,
    )
    assert evaluate_transform(truth, truth).translation_m == 0.0
    assert evaluate_transform(truth, truth).rotation_deg == 0.0
    sign_equivalent = RigidTransform(
        parent_frame=truth.parent_frame,
        child_frame=truth.child_frame,
        translation_m=truth.translation_m,
        quaternion_xyzw=tuple(-value for value in truth.quaternion_xyzw),
    )
    assert math.isclose(
        evaluate_transform(sign_equivalent, truth).rotation_deg,
        0.0,
        abs_tol=1.0e-12,
    )


def test_handeye_profile_does_not_publish_camera_ground_truth_tf() -> None:
    launch_source = (
        PACKAGE_ROOT / 'launch' / 'handeye_backend.launch.py'
    ).read_text(encoding='utf-8')
    evaluator_source = (
        PACKAGE_ROOT
        / 'cleany_mujoco_sim'
        / 'ground_truth_evaluation.py'
    ).read_text(encoding='utf-8')
    control_urdf = (
        WORKSPACE_SOURCE
        / 'cleany_description'
        / 'urdf'
        / 'cleany_control.urdf.xacro'
    ).read_text(encoding='utf-8')
    assert 'static_transform_publisher' not in launch_source
    assert 'TransformBroadcaster' not in launch_source
    assert 'rclpy' not in evaluator_source
    assert 'tf2_ros' not in evaluator_source
    assert 'left_wrist_rgb_optical_frame' not in control_urdf
