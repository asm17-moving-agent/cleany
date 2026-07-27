from pathlib import Path

import mujoco


HARDWARE_SCENE = Path(__file__).parents[1] / 'hardware' / 'scene.xml'


def test_hardware_scene_compiles_with_rigid_camera_mounts() -> None:
    # Given: the source MJCF used by the simulator
    # When: MuJoCo compiles the complete model, including both wrist cameras
    model = mujoco.MjModel.from_xml_path(str(HARDWARE_SCENE))

    # Then: both camera sensors are present in a valid compiled model
    camera_names = {
        mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_CAMERA, camera_id)
        for camera_id in range(model.ncam)
    }
    assert {'right_wrist_rgb', 'left_wrist_rgb'} <= camera_names
