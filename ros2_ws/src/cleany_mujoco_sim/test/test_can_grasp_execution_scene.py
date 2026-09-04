from pathlib import Path

import mujoco
import pytest

from cleany_mujoco_sim.scene_loader import materialize_control_scene


def test_can_grasp_scene_matches_camera_and_moveit_geometry() -> None:
    template = (
        Path(__file__).parents[1]
        / 'scenes'
        / 'can_grasp_execution_demo.xml.in'
    )
    scene = materialize_control_scene(template)
    model = mujoco.MjModel.from_xml_path(str(scene))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)

    chassis = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_BODY, 'chassis'
    )
    table = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_BODY, 'pick_table'
    )
    can = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, 'pick_can')
    can_geom = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_GEOM, 'pick_can_geom'
    )
    can_joint = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_JOINT, 'pick_can_freejoint'
    )
    table_geom = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_GEOM, 'pick_tabletop'
    )
    camera = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_CAMERA, 'pick_demo_rgbd'
    )

    assert data.xpos[table] - data.xpos[chassis] == pytest.approx(
        (0.700, -0.002, 0.330)
    )
    assert data.xpos[can] - data.xpos[chassis] == pytest.approx(
        (0.440, 0.160, 0.395)
    )
    assert model.geom_size[table_geom] == pytest.approx(
        (0.385, 0.600, 0.015)
    )
    assert model.geom_size[can_geom] == pytest.approx((0.035, 0.050, 0.0))
    assert can_joint >= 0
    assert model.jnt_type[can_joint] == mujoco.mjtJoint.mjJNT_FREE
    assert model.body_mass[can] == pytest.approx(0.060)
    assert model.geom_contype[table_geom] == 1
    assert model.geom_contype[can_geom] == 1
    assert model.geom_conaffinity[can_geom] == 1
    assert mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_BODY, 'can_demo_route_obstacle'
    ) == -1
    assert model.cam_resolution[camera] == pytest.approx((640, 480))
    assert model.cam_fovy[camera] == pytest.approx(42.0)


def test_box_grasp_scene_has_dynamic_tabletop_target() -> None:
    template = (
        Path(__file__).parents[1]
        / 'scenes'
        / 'box_grasp_execution_demo.xml.in'
    )
    scene = materialize_control_scene(template)
    model = mujoco.MjModel.from_xml_path(str(scene))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)

    chassis = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_BODY, 'chassis'
    )
    target = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_BODY, 'grasp_test_block'
    )
    geom = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_GEOM, 'grasp_test_block_geom'
    )
    joint = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_JOINT, 'grasp_test_block_freejoint'
    )

    assert data.xpos[target] - data.xpos[chassis] == pytest.approx(
        (0.440, 0.160, 0.395)
    )
    assert model.geom_size[geom] == pytest.approx((0.025, 0.035, 0.050))
    assert model.jnt_type[joint] == mujoco.mjtJoint.mjJNT_FREE
    assert model.body_mass[target] == pytest.approx(0.080)
    assert model.opt.timestep == pytest.approx(0.001)
    assert model.opt.iterations == 100
    assert model.opt.ls_iterations == 20
    assert model.opt.noslip_iterations == 20
    assert model.geom_priority[geom] == 0
    assert model.geom_condim[geom] == 6
    assert model.geom_friction[geom] == pytest.approx((2.0, 0.12, 0.06))
    table = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_GEOM, 'pick_tabletop'
    )
    assert model.geom_condim[table] == 4
    assert model.geom_friction[table] == pytest.approx((2.0, 0.08, 0.02))
    assert mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_BODY, 'pick_obstacle'
    ) == -1
    assert model.npair == 10
    for pair in range(model.npair):
        assert model.pair_dim[pair] == 6
        assert model.pair_friction[pair] == pytest.approx(
            (5.0, 5.0, 0.4, 0.15, 0.15)
        )
        assert model.pair_solimp[pair] == pytest.approx(
            (0.99, 0.9999, 0.0001, 0.5, 2.0)
        )
