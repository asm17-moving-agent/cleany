import ast
import hashlib
import math
from pathlib import Path

import mujoco
import numpy as np
import pytest
import yaml

from cleany_mujoco_sim.rgbd import RgbdSensorConfig
from cleany_mujoco_sim.scene_loader import (
    materialize_control_scene,
    materialize_scene,
)


PACKAGE_ROOT = Path(__file__).parents[1]
WORKSPACE_SOURCE = PACKAGE_ROOT.parent
SCENE_TEMPLATE = PACKAGE_ROOT / 'scenes' / 'study_cafe.xml.in'
GRASP_SCENE_TEMPLATE = (
    PACKAGE_ROOT / 'scenes' / 'study_cafe_grasp_execution.xml.in'
)
MUJOCO_LAYOUT = PACKAGE_ROOT / 'config' / 'study_cafe_layout.yaml'
ASSET_MANIFEST = PACKAGE_ROOT / 'config' / 'study_cafe_assets.yaml'
GAZEBO_LAYOUT = (
    WORKSPACE_SOURCE
    / 'cleany_gazebo_sim'
    / 'config'
    / 'study_cafe'
    / 'study_cafe_layout.yaml'
)


@pytest.fixture(scope='module')
def study_cafe_model() -> mujoco.MjModel:
    scene_path = materialize_scene(SCENE_TEMPLATE)
    return mujoco.MjModel.from_xml_path(str(scene_path))


def _body_id(model: mujoco.MjModel, name: str) -> int:
    body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)
    assert body_id >= 0
    return body_id


def _geom_id(model: mujoco.MjModel, name: str) -> int:
    geom_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, name)
    assert geom_id >= 0
    return geom_id


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _obj_extent(path: Path) -> tuple[float, float, float]:
    vertices = [
        tuple(float(value) for value in line.split()[1:4])
        for line in path.read_text(encoding='utf-8').splitlines()
        if line.startswith('v ')
    ]
    assert vertices
    return tuple(
        max(vertex[axis] for vertex in vertices)
        - min(vertex[axis] for vertex in vertices)
        for axis in range(3)
    )


def test_mujoco_layout_snapshot_matches_gazebo_source() -> None:
    mujoco_layout = yaml.safe_load(MUJOCO_LAYOUT.read_text(encoding='utf-8'))
    gazebo_layout = yaml.safe_load(GAZEBO_LAYOUT.read_text(encoding='utf-8'))
    mujoco_layout.pop('mujoco')

    assert mujoco_layout == gazebo_layout


def test_tabletop_asset_files_match_provenance_manifest() -> None:
    manifest = yaml.safe_load(ASSET_MANIFEST.read_text(encoding='utf-8'))

    assert manifest['schema_version'] == 1
    assert set(manifest['assets']) == {'cup', 'phone', 'wallet'}
    for item in manifest['assets'].values():
        asset_path = PACKAGE_ROOT / 'assets' / item['imported_file']
        assert asset_path.is_file()
        assert _sha256(asset_path) == item['imported_sha256']
        assert _obj_extent(asset_path) == pytest.approx(
            item['normalized_extent_m'],
            abs=1e-6,
        )


@pytest.mark.parametrize('template', [SCENE_TEMPLATE, GRASP_SCENE_TEMPLATE])
def test_study_cafe_uses_scoped_non_saturating_shadow_lights(template: Path) -> None:
    model = mujoco.MjModel.from_xml_path(str(materialize_scene(template)))
    assert model.nlight == 2  # The canonical robot's generic light is replaced.
    key = model.light('study_cafe_work_area_key').id
    fill = model.light('study_cafe_room_fill').id
    assert model.light_castshadow[key]
    assert model.light_type[key] == mujoco.mjtLightType.mjLIGHT_SPOT
    assert not model.light_castshadow[fill]
    assert model.light_type[fill] == mujoco.mjtLightType.mjLIGHT_DIRECTIONAL
    assert model.vis.quality.shadowsize == 4096
    assert model.vis.map.shadowclip * model.stat.extent == pytest.approx(12.0)
    np.testing.assert_allclose(model.vis.headlight.ambient, (.12, .12, .12))
    np.testing.assert_allclose(model.light_pos[key], (-3.13, -3.60, 2.45))
    assert model.light_cutoff[key] == pytest.approx(55.0)
    assert model.light_exponent[key] == pytest.approx(2.0)
    # Nominal cone footprint spans neighboring desks, not only the target.
    assert 2 * (2.45 - .72) * math.tan(math.radians(55)) > 4.9


def test_study_cafe_scene_compiles_with_all_static_environment_bodies(
    study_cafe_model: mujoco.MjModel,
) -> None:
    expected_prefix_counts = {
        'wall_': 4,
        'demo_desk_': 48,
        'office_chair_': 47,
        'desk_partition_': 24,
        'desk_monitor_': 48,
    }
    body_names = [
        mujoco.mj_id2name(
            study_cafe_model,
            mujoco.mjtObj.mjOBJ_BODY,
            body_id,
        )
        or ''
        for body_id in range(study_cafe_model.nbody)
    ]

    for prefix, expected_count in expected_prefix_counts.items():
        matching_names = [
            name for name in body_names if name.startswith(prefix)
        ]
        assert len(matching_names) == expected_count
        assert all(
            study_cafe_model.body_jntnum[
                _body_id(study_cafe_model, name)
            ]
            == 0
            for name in matching_names
        )

    data = mujoco.MjData(study_cafe_model)
    mujoco.mj_step(study_cafe_model, data)
    assert data.time == pytest.approx(study_cafe_model.opt.timestep)


def test_tabletop_objects_use_visuals_and_dynamic_collisions(
    study_cafe_model: mujoco.MjModel,
) -> None:
    expected = {
        'cup': {
            'position': (-3.25, -3.73, 0.72),
            'mass': 0.008,
            'type': mujoco.mjtGeom.mjGEOM_CYLINDER,
            'size': (0.0275, 0.00035, 0.0),
            'visual_type': mujoco.mjtGeom.mjGEOM_MESH,
        },
        'wallet': {
            'position': (-3.27, -3.57, 0.72),
            'mass': 0.12,
            'type': mujoco.mjtGeom.mjGEOM_MESH,
            'size': None,
            'visual_type': mujoco.mjtGeom.mjGEOM_MESH,
        },
        'tissue': {
            'position': (-3.02, -3.625, 0.72),
            'mass': 0.001,
            'type': mujoco.mjtGeom.mjGEOM_MESH,
            'size': None,
            'visual_type': mujoco.mjtGeom.mjGEOM_MESH,
        },
        'lego': {
            'position': (-2.97, -3.76, 0.72),
            'mass': 0.0023,
            'type': mujoco.mjtGeom.mjGEOM_BOX,
            'size': (0.0159, 0.0079, 0.0048),
            'visual_type': mujoco.mjtGeom.mjGEOM_BOX,
        },
    }
    for name, properties in expected.items():
        body_name = f'study_cafe_{name}'
        body = _body_id(study_cafe_model, body_name)
        visual = _geom_id(study_cafe_model, f'{body_name}_visual')
        collision = _geom_id(study_cafe_model, f'{body_name}_collision')

        assert study_cafe_model.body_pos[body] == pytest.approx(
            properties['position']
        )
        assert study_cafe_model.body_mass[body] == pytest.approx(
            properties['mass']
        )
        assert study_cafe_model.body_jntnum[body] == 1
        joint = int(study_cafe_model.body_jntadr[body])
        assert study_cafe_model.jnt_type[joint] == mujoco.mjtJoint.mjJNT_FREE

        assert study_cafe_model.geom_type[visual] == properties['visual_type']
        assert study_cafe_model.geom_contype[visual] == 0
        assert study_cafe_model.geom_conaffinity[visual] == 0

        assert study_cafe_model.geom_type[collision] == properties['type']
        if properties['size'] is not None:
            assert study_cafe_model.geom_size[collision] == pytest.approx(properties['size'])
        assert study_cafe_model.geom_contype[collision] == 1
        assert study_cafe_model.geom_conaffinity[collision] == 1
        assert study_cafe_model.geom_group[collision] == 3

    for old in ('study_cafe_phone', 'study_cafe_box'):
        assert mujoco.mj_name2id(study_cafe_model, mujoco.mjtObj.mjOBJ_BODY, old) == -1
    assert mujoco.mj_name2id(
        study_cafe_model,
        mujoco.mjtObj.mjOBJ_GEOM,
        'study_cafe_box_sleeve_label_visual',
    ) == -1


def test_tabletop_objects_settle_on_robot_desk(
    study_cafe_model: mujoco.MjModel,
) -> None:
    data = mujoco.MjData(study_cafe_model)
    for _ in range(1000):
        mujoco.mj_step(study_cafe_model, data)

    desk_geom = _geom_id(
        study_cafe_model,
        'demo_desk_43__tabletop_back',
    )
    for name in ('cup', 'wallet', 'tissue', 'lego'):
        body_name = f'study_cafe_{name}'
        body = _body_id(study_cafe_model, body_name)
        collision = _geom_id(
            study_cafe_model,
            f'{body_name}_collision',
        )
        if name == 'wallet':
            # Its non-flat mesh base can tilt; check actual support, not body origin.
            mesh = study_cafe_model.geom_dataid[collision]
            first = study_cafe_model.mesh_vertadr[mesh]
            vertices = study_cafe_model.mesh_vert[first:first+study_cafe_model.mesh_vertnum[mesh]]
            points = vertices @ data.geom_xmat[collision].reshape(3, 3).T + data.geom_xpos[collision]
            assert points[:, 2].min() == pytest.approx(.72, abs=.0001)
        else:
            assert data.xpos[body][2] == pytest.approx(0.72, abs=.003 if name == 'tissue' else 2e-5)
        assert any(
            {int(contact.geom1), int(contact.geom2)}
            == {desk_geom, collision}
            for contact in data.contact
        )


def test_wallet_collision_reuses_visible_mesh_without_hidden_box(
    study_cafe_model: mujoco.MjModel,
) -> None:
    model = study_cafe_model
    visual = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, 'study_cafe_wallet_visual')
    collision = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, 'study_cafe_wallet_collision')
    assert model.geom_type[collision] == mujoco.mjtGeom.mjGEOM_MESH
    assert model.geom_dataid[collision] == model.geom_dataid[visual]
    assert model.geom_pos[collision] == pytest.approx(model.geom_pos[visual])
    assert model.geom_quat[collision] == pytest.approx(model.geom_quat[visual])
    assert model.body_mass[model.geom_bodyid[collision]] == pytest.approx(.12)
    assert model.geom_friction[collision] == pytest.approx((1.5, .08, .02))


def test_tabletop_objects_fit_head_camera_at_one_radian(
    study_cafe_model: mujoco.MjModel,
) -> None:
    data = mujoco.MjData(study_cafe_model)
    head_tilt = mujoco.mj_name2id(
        study_cafe_model,
        mujoco.mjtObj.mjOBJ_JOINT,
        'head_tilt_joint',
    )
    data.qpos[study_cafe_model.jnt_qposadr[head_tilt]] = 1.0
    mujoco.mj_forward(study_cafe_model, data)

    camera_config = RgbdSensorConfig()
    camera = mujoco.mj_name2id(
        study_cafe_model,
        mujoco.mjtObj.mjOBJ_CAMERA,
        camera_config.camera_name,
    )
    camera_rotation = data.cam_xmat[camera].reshape(3, 3)
    half_fovy = math.radians(study_cafe_model.cam_fovy[camera]) / 2.0
    aspect_ratio = camera_config.width / camera_config.height

    for name in ('cup', 'wallet', 'tissue', 'lego'):
        body = _body_id(study_cafe_model, f'study_cafe_{name}')
        if name in ('cup', 'tissue', 'lego'):
            half_x, half_y, height = {
                'cup': (.0425, .0425, .095),
                'tissue': (.03, .025, .045),
                'lego': (.0159, .0079, .0114),
            }[name]
            vertices = np.asarray(
                [
                    (x, y, z)
                    for x in (-half_x, half_x)
                    for y in (-half_y, half_y)
                    for z in (0.0, height)
                ]
            )
        else:
            vertices = np.asarray(
                [
                    tuple(float(value) for value in line.split()[1:4])
                    for line in (
                        PACKAGE_ROOT / 'assets' / f'study_cafe_{name}.obj'
                    ).read_text(encoding='utf-8').splitlines()
                    if line.startswith('v ')
                ]
            )
        world_vertices = (
            vertices @ data.xmat[body].reshape(3, 3).T
            + data.xpos[body]
        )
        camera_vertices = (
            world_vertices - data.cam_xpos[camera]
        ) @ camera_rotation
        depth = -camera_vertices[:, 2]
        normalized_x = camera_vertices[:, 0] / (
            depth * math.tan(half_fovy) * aspect_ratio
        )
        normalized_y = camera_vertices[:, 1] / (
            depth * math.tan(half_fovy)
        )

        assert np.all(depth > 0.0)
        assert np.max(np.abs(normalized_x)) < 0.9
        assert np.max(np.abs(normalized_y)) < 0.9


def test_study_cafe_entity_positions_match_gazebo_layout(
    study_cafe_model: mujoco.MjModel,
) -> None:
    expected_positions = {
        'wall_north': (0.0, 5.55, 1.25),
        'desk_partition_01': (-5.53, 3.17, 0.66),
        'demo_desk_01': (-5.53, 3.555, 0.0),
        'desk_monitor_01': (-5.53, 3.27, 0.0),
        'office_chair_01': (-5.53, 3.94, 0.0),
    }
    for name, expected in expected_positions.items():
        assert study_cafe_model.body_pos[
            _body_id(study_cafe_model, name)
        ] == pytest.approx(expected)

    chair_quat = study_cafe_model.body_quat[
        _body_id(study_cafe_model, 'office_chair_01')
    ]
    assert chair_quat == pytest.approx(
        (math.sqrt(0.5), 0.0, 0.0, -math.sqrt(0.5))
    )


def test_study_cafe_robot_replaces_nearest_chair_and_faces_desk(
    study_cafe_model: mujoco.MjModel,
) -> None:
    chassis = _body_id(study_cafe_model, 'chassis')
    assert study_cafe_model.body_pos[chassis] == pytest.approx(
        (-3.13, -4.12, 0.38)
    )
    assert study_cafe_model.body_quat[chassis] == pytest.approx(
        (
            math.sqrt(0.5),
            0.0,
            0.0,
            math.sqrt(0.5),
        )
    )
    assert mujoco.mj_name2id(
        study_cafe_model,
        mujoco.mjtObj.mjOBJ_BODY,
        'office_chair_43',
    ) == -1


def test_docked_robot_does_not_start_in_collision_with_desk(
    study_cafe_model: mujoco.MjModel,
) -> None:
    data = mujoco.MjData(study_cafe_model)
    mujoco.mj_forward(study_cafe_model, data)
    chassis = _body_id(study_cafe_model, 'chassis')
    desk = _body_id(study_cafe_model, 'demo_desk_43')
    robot_bodies = set()
    for body_id in range(study_cafe_model.nbody):
        ancestor = body_id
        while ancestor > 0:
            if ancestor == chassis:
                robot_bodies.add(body_id)
                break
            ancestor = int(study_cafe_model.body_parentid[ancestor])
    robot_geoms = {
        geom_id
        for geom_id in range(study_cafe_model.ngeom)
        if int(study_cafe_model.geom_bodyid[geom_id]) in robot_bodies
    }
    desk_geoms = set(
        range(
            int(study_cafe_model.body_geomadr[desk]),
            int(
                study_cafe_model.body_geomadr[desk]
                + study_cafe_model.body_geomnum[desk]
            ),
        )
    )

    assert not any(
        {int(contact.geom1), int(contact.geom2)} & robot_geoms
        and {int(contact.geom1), int(contact.geom2)} & desk_geoms
        for contact in data.contact
    )


def test_study_cafe_primitive_dimensions_and_collisions_match_gazebo(
    study_cafe_model: mujoco.MjModel,
) -> None:
    expected_sizes = {
        'wall_north__body': (6.21, 0.08, 1.25),
        'demo_desk_01__tabletop_back': (0.60, 0.355, 0.02),
        'desk_monitor_01__monitor_panel': (0.31, 0.0175, 0.18),
        'office_chair_01__seat': (0.26, 0.275, 0.04),
    }
    for name, expected in expected_sizes.items():
        geom_id = _geom_id(study_cafe_model, name)
        assert study_cafe_model.geom_size[geom_id] == pytest.approx(expected)
        assert study_cafe_model.geom_contype[geom_id] == 1
        assert study_cafe_model.geom_conaffinity[geom_id] == 1
        assert study_cafe_model.geom_group[geom_id] == 2

    screen = _geom_id(
        study_cafe_model,
        'desk_monitor_01__monitor_screen_visual',
    )
    assert study_cafe_model.geom_size[screen] == pytest.approx(
        (0.299, 0.001, 0.168)
    )
    assert study_cafe_model.geom_contype[screen] == 0
    assert study_cafe_model.geom_conaffinity[screen] == 0


def test_study_cafe_launch_selects_dedicated_scene_and_viewer() -> None:
    launch_path = PACKAGE_ROOT / 'launch' / 'mujoco_study_cafe.launch.py'
    source = launch_path.read_text(encoding='utf-8')
    ast.parse(source)

    assert "'study_cafe.xml.in'" in source
    assert "default_value='false'" in source
    assert "'mujoco_sim.launch.py'" in source


def test_scene_loader_rejects_duplicate_environment_tokens(
    tmp_path: Path,
) -> None:
    source = SCENE_TEMPLATE.read_text(encoding='utf-8')
    duplicate_template = tmp_path / 'duplicate_study_cafe.xml.in'
    duplicate_template.write_text(
        source.replace(
            '</worldbody>',
            '  @CLEANY_STUDY_CAFE_ENVIRONMENT@\n  </worldbody>',
        ),
        encoding='utf-8',
    )

    with pytest.raises(ValueError, match='exactly one'):
        materialize_scene(duplicate_template)


def test_grasp_execution_scene_has_fixed_base_and_compound_object_contacts() -> None:
    scene_path = materialize_control_scene(
        GRASP_SCENE_TEMPLATE,
        initial_joint_positions={'head_tilt_joint': 1.0},
    )
    model = mujoco.MjModel.from_xml_path(str(scene_path))
    data = mujoco.MjData(model)
    keyframe = mujoco.mj_name2id(
        model,
        mujoco.mjtObj.mjOBJ_KEY,
        'handeye_ros2_control_home',
    )
    mujoco.mj_resetDataKeyframe(model, data, keyframe)
    mujoco.mj_forward(model, data)

    head_tilt = mujoco.mj_name2id(
        model,
        mujoco.mjtObj.mjOBJ_JOINT,
        'head_tilt_joint',
    )
    assert data.qpos[model.jnt_qposadr[head_tilt]] == pytest.approx(1.0)
    assert mujoco.mj_name2id(
        model,
        mujoco.mjtObj.mjOBJ_EQUALITY,
        'study_cafe_grasp_chassis_world_weld',
    ) >= 0
    assert model.npair == 10*(33+9+2)  # Cup shell, LEGO body+studs, tissue, wallet.
    cup = _geom_id(model, 'study_cafe_cup_collision')
    cup_pairs = [i for i in range(model.npair)
                 if cup in (model.pair_geom1[i], model.pair_geom2[i])]
    assert len(cup_pairs) == 10
    for pair in cup_pairs:
        assert model.pair_dim[pair] == 6
        assert model.pair_friction[pair] == pytest.approx((6, 6, 0.2, 0.1, 0.1))
        jaw = model.pair_geom2[pair] if model.pair_geom1[pair] == cup else model.pair_geom1[pair]
        assert 'jaw_contact' in mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, jaw)
        assert model.geom_friction[jaw] == pytest.approx((3, 0.2, 0.1))
        assert model.geom_priority[jaw] == model.geom_priority[cup] == 0
        assert model.geom_solmix[jaw] == model.geom_solmix[cup] == 1
        assert model.pair_solref[pair] == pytest.approx((0.002, 1))
        assert model.pair_solimp[pair] == pytest.approx((0.99, 0.9999, 0.0001, 0.5, 2))
    assert model.geom_friction[cup] == pytest.approx((1.5, 0.08, 0.02))
    cup_body = int(model.geom_bodyid[cup])
    shell_pairs = [i for i in range(model.npair)
                   if cup_body in (model.geom_bodyid[model.pair_geom1[i]],
                                   model.geom_bodyid[model.pair_geom2[i]])]
    assert len(shell_pairs) == 330
    for pair in shell_pairs:
        assert model.pair_solref[pair] == pytest.approx((0.002, 1))
        assert model.pair_solimp[pair] == pytest.approx((0.99, 0.9999, 0.0001, 0.5, 2))

    for label in ('tissue', 'wallet'):
        geom = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, f'study_cafe_{label}_collision')
        pairs = [i for i in range(model.npair)
                 if geom in (model.pair_geom1[i], model.pair_geom2[i])]
        assert len(pairs) == 10
        for pair in pairs:
            assert model.pair_dim[pair] == 6
            # Preserve the original max-combined object/jaw friction.
            assert model.pair_friction[pair] == pytest.approx((3, 3, .2, .1, .1))
            assert model.pair_solref[pair] == pytest.approx((.002, 1))
            assert model.pair_solimp[pair] == pytest.approx((.99, .9999, .0001, .5, 2))

    camera = mujoco.mj_name2id(
        model,
        mujoco.mjtObj.mjOBJ_CAMERA,
        'head_realsense_rgb',
    )
    assert tuple(model.cam_resolution[camera]) == (640, 480)

    box = _body_id(model, 'study_cafe_lego')
    assert model.body_jntnum[box] == 1
    box_joint = int(model.body_jntadr[box])
    assert model.jnt_type[box_joint] == mujoco.mjtJoint.mjJNT_FREE
