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
        (0.540, 0.160, 0.395)
    )
    assert model.geom_size[table_geom] == pytest.approx(
        (0.385, 0.600, 0.015)
    )
    assert model.geom_size[can_geom] == pytest.approx((0.035, 0.050, 0.0))
    assert model.geom_contype[table_geom] == 1
    assert model.geom_contype[can_geom] == 1
    assert model.geom_conaffinity[can_geom] == 1
    assert model.cam_resolution[camera] == pytest.approx((640, 480))
    assert model.cam_fovy[camera] == pytest.approx(42.0)
