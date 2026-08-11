from __future__ import annotations

import math
from pathlib import Path
import re

import mujoco
import numpy as np
import pytest
import yaml

from cleany_mujoco_sim.scene_loader import (
    materialize_control_scene,
    materialize_scene,
)
from cleany_mujoco_sim.scene_manifest import (
    PhysicalMeasurementRequiredError,
    load_handeye_scene_manifest,
    preflight_manifest,
)


PACKAGE_ROOT = Path(__file__).parents[1]
SCENE_TEMPLATE = PACKAGE_ROOT / 'scenes' / 'handeye.xml.in'
MANIFEST_PATH = PACKAGE_ROOT / 'config' / 'handeye_scene.yaml'
MOVEIT_COLLISION_CONFIG = (
    PACKAGE_ROOT.parent
    / 'cleany_moveit_config'
    / 'config'
    / 'handeye_collision_objects.yaml'
)


def _body_id(model, name: str) -> int:
    identifier = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)
    assert identifier >= 0
    return identifier


def _geom_id(model, name: str) -> int:
    identifier = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, name)
    assert identifier >= 0
    return identifier


def _relative_transform(data, parent_id: int, child_id: int):
    parent_rotation = data.xmat[parent_id].reshape(3, 3)
    child_rotation = data.xmat[child_id].reshape(3, 3)
    translation = parent_rotation.T @ (
        data.xpos[child_id] - data.xpos[parent_id]
    )
    rotation = parent_rotation.T @ child_rotation
    return translation, rotation


def _quaternion_matrix_xyzw(values: list[float]) -> np.ndarray:
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


def test_manifest_assets_and_measurement_preflight() -> None:
    manifest = load_handeye_scene_manifest(MANIFEST_PATH)
    preflight_manifest(manifest, profile='simulation')
    with pytest.raises(
        PhysicalMeasurementRequiredError,
        match='has not been measured',
    ):
        preflight_manifest(manifest, profile='physical')

    measurement = manifest.data['target']['board']['physical_measurement']
    assert measurement['status'] == 'not_measured'
    assert measurement['square_length_m'] is None
    assert measurement['marker_length_m'] is None
    assert measurement['object_point_length_source'] == 'physical_measurement'

    assets = {asset.format: asset for asset in manifest.printable_assets}
    assert set(assets) == {'svg', 'pdf'}
    assert assets['svg'].media_width_m == pytest.approx(0.210)
    assert assets['svg'].media_height_m == pytest.approx(0.150)
    svg = assets['svg'].path.read_text(encoding='utf-8')
    assert 'width="210mm" height="150mm"' in svg
    assert svg.count('    M ') == 211
    pdf = assets['pdf'].path.read_bytes()
    assert pdf.startswith(b'%PDF-1.4\n')
    media_box = re.search(
        rb'/MediaBox \[0 0 ([0-9.]+) ([0-9.]+)\]', pdf
    )
    assert media_box is not None
    assert float(media_box.group(1)) * 25.4 / 72.0 == pytest.approx(
        210.0, abs=1.0e-4
    )
    assert float(media_box.group(2)) * 25.4 / 72.0 == pytest.approx(
        150.0, abs=1.0e-4
    )


def test_scene_compiles_with_world_weld_and_opencv_target_frame() -> None:
    scene_path = materialize_scene(SCENE_TEMPLATE)
    model = mujoco.MjModel.from_xml_path(str(scene_path))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)

    equality_id = mujoco.mj_name2id(
        model,
        mujoco.mjtObj.mjOBJ_EQUALITY,
        'handeye_chassis_world_weld',
    )
    assert equality_id >= 0
    assert model.eq_type[equality_id] == mujoco.mjtEq.mjEQ_WELD
    assert model.eq_obj1id[equality_id] == _body_id(model, 'chassis')
    assert model.eq_obj2id[equality_id] == 0

    manifest = yaml.safe_load(MANIFEST_PATH.read_text(encoding='utf-8'))
    target_metadata = manifest['target']
    frame = target_metadata['board']['coordinate_frame']
    assert frame == {
        'origin': 'printed_board_bottom_left',
        'x_axis': 'printed_board_right',
        'y_axis': 'printed_board_up',
        'z_axis': 'out_of_printed_board',
        'object_points_api': (
            'OpenCV_4.5.4_CharucoBoard.chessboardCorners'
        ),
    }
    target_id = _body_id(model, target_metadata['frame_id'])
    expected_world_pose = target_metadata['mujoco_geometry']['world_T_target']
    assert data.xpos[target_id] == pytest.approx(
        expected_world_pose['translation_m'], abs=1.0e-12
    )
    assert data.xmat[target_id].reshape(3, 3) == pytest.approx(
        _quaternion_matrix_xyzw(expected_world_pose['quaternion_xyzw']),
        abs=1.0e-12,
    )

    x_axis, y_axis, z_axis = data.xmat[target_id].reshape(3, 3).T
    assert x_axis == pytest.approx((0.0, -1.0, 0.0), abs=1.0e-12)
    assert y_axis == pytest.approx((0.0, 0.0, 1.0), abs=1.0e-12)
    assert z_axis == pytest.approx((-1.0, 0.0, 0.0), abs=1.0e-12)


def test_base_target_transform_is_invariant_during_arm_motion() -> None:
    scene_path = materialize_scene(SCENE_TEMPLATE)
    model = mujoco.MjModel.from_xml_path(str(scene_path))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    chassis_id = _body_id(model, 'chassis')
    target_id = _body_id(model, 'charuco_target')
    before_translation, before_rotation = _relative_transform(
        data, chassis_id, target_id
    )

    arm_targets = {
        'left_shoulder_yaw_joint': 0.30,
        'left_shoulder_pitch_joint': 0.45,
        'left_elbow_pitch_joint': 0.55,
        'left_wrist_pitch_joint': -0.25,
        'left_wrist_roll_joint': 0.20,
    }
    for actuator_name, target in arm_targets.items():
        actuator_id = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_ACTUATOR, actuator_name
        )
        assert actuator_id >= 0
        data.ctrl[actuator_id] = target
    for _ in range(500):
        mujoco.mj_step(model, data)

    assert data.time == pytest.approx(1.0)
    after_translation, after_rotation = _relative_transform(
        data, chassis_id, target_id
    )
    # Equality constraints are solved numerically.  The observed residual must
    # stay in the tens-of-micrometres range under a full second of arm motion.
    assert after_translation == pytest.approx(before_translation, abs=2.0e-5)
    assert after_rotation == pytest.approx(before_rotation, abs=5.0e-6)


def test_compiled_geometry_matches_manifest_and_moveit_collision_config() -> None:
    scene_path = materialize_scene(SCENE_TEMPLATE)
    model = mujoco.MjModel.from_xml_path(str(scene_path))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    manifest = yaml.safe_load(MANIFEST_PATH.read_text(encoding='utf-8'))
    collision = yaml.safe_load(
        MOVEIT_COLLISION_CONFIG.read_text(encoding='utf-8')
    )
    objects = {item['id']: item for item in collision['objects']}
    assert collision['planning_frame'] == 'base_link'
    assert set(objects) == set(
        manifest['planning_scene']['required_object_ids']
    )

    geometry = manifest['target']['mujoco_geometry']
    backing_id = _geom_id(model, geometry['backing_geom_name'])
    assert 2.0 * model.geom_size[backing_id] == pytest.approx(
        geometry['backing_size_m'], abs=1.0e-12
    )
    ink_names = [
        mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, index)
        for index in range(model.ngeom)
    ]
    assert sum(
        name is not None and name.startswith(geometry['ink_geom_prefix'])
        for name in ink_names
    ) == geometry['ink_rectangle_count']

    base_id = _body_id(model, 'chassis')
    base_rotation = data.xmat[base_id].reshape(3, 3)
    compiled_pairs = (
        ('handeye_table', ('handeye_table_top',)),
        (
            'handeye_target_stand',
            (
                'handeye_stand_base',
                'handeye_stand_post',
                'handeye_stand_crossbar',
            ),
        ),
        ('charuco_target', ('charuco_target_backing',)),
    )
    for object_id, geom_names in compiled_pairs:
        configured = objects[object_id]['primitives']
        assert len(configured) == len(geom_names)
        for primitive, geom_name in zip(configured, geom_names):
            geom_id = _geom_id(model, geom_name)
            assert primitive['dimensions_m'] == pytest.approx(
                2.0 * model.geom_size[geom_id], abs=1.0e-12
            )
            base_translation = base_rotation.T @ (
                data.geom_xpos[geom_id] - data.xpos[base_id]
            )
            base_geom_rotation = (
                base_rotation.T @ data.geom_xmat[geom_id].reshape(3, 3)
            )
            assert primitive['pose']['translation_m'] == pytest.approx(
                base_translation, abs=1.0e-12
            )
            assert base_geom_rotation == pytest.approx(
                _quaternion_matrix_xyzw(
                    primitive['pose']['quaternion_xyzw']
                ),
                abs=1.0e-12,
            )


def test_handeye_scene_retains_humble_control_materialization() -> None:
    control_scene = materialize_control_scene(SCENE_TEMPLATE)
    model = mujoco.MjModel.from_xml_path(str(control_scene))
    assert model.nu == 14
    assert model.nkey == 1
    assert mujoco.mj_name2id(
        model,
        mujoco.mjtObj.mjOBJ_BODY,
        'charuco_target',
    ) >= 0
    assert mujoco.mj_name2id(
        model,
        mujoco.mjtObj.mjOBJ_EQUALITY,
        'handeye_chassis_world_weld',
    ) >= 0
