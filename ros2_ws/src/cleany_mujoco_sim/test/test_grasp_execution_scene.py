from pathlib import Path

import mujoco
import pytest

from cleany_mujoco_sim.scene_loader import materialize_control_scene


def test_grasp_execution_scene_matches_demo_target_contract() -> None:
    template = (
        Path(__file__).parents[1]
        / 'scenes'
        / 'grasp_execution_demo.xml.in'
    )
    scene = materialize_control_scene(template)
    model = mujoco.MjModel.from_xml_path(str(scene))

    body_id = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_BODY, 'demo_grasp_target'
    )
    geom_id = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_GEOM, 'demo_grasp_target_geom'
    )
    chassis_id = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_BODY, 'chassis'
    )
    camera_id = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_CAMERA, 'left_wrist_rgb'
    )

    assert body_id >= 0
    assert geom_id >= 0
    assert chassis_id >= 0
    assert camera_id >= 0
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    assert data.xpos[body_id] == pytest.approx((0.09, 0.6696, 0.9958))
    assert data.xpos[body_id] - data.xpos[chassis_id] == pytest.approx(
        (0.09, 0.6696, 0.6158)
    )
    assert model.geom_size[geom_id] == pytest.approx((0.015, 0.015, 0.015))
    assert model.geom_contype[geom_id] == 0
    assert model.geom_conaffinity[geom_id] == 0
    assert model.cam_resolution[camera_id] == pytest.approx((640, 480))
    assert any(
        object_id == chassis_id for object_id in model.eq_obj1id
    )
