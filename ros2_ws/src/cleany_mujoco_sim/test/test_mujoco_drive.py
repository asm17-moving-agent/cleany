import mujoco
import pytest

from cleany_mujoco_sim.mecanum_kinematics import WheelSpeeds
from cleany_mujoco_sim.mujoco_drive import MujocoMecanumDrive
from cleany_mujoco_sim.scene_loader import load_model
from cleany_mujoco_sim.wheel_speed_controller import (
    PidGains,
    VelocityControllerConfig,
)


@pytest.fixture
def controller_config() -> VelocityControllerConfig:
    return VelocityControllerConfig(
        gains=PidGains(kp=1.0, ki=5.0, kd=0.0),
        voltage_limit=10.8,
        no_load_speed=10.815,
    )


@pytest.fixture
def xlerobot_model_data(cleany_scene_path):
    return load_model(cleany_scene_path)


def test_drive_maps_each_wheel_to_its_motor(
    xlerobot_model_data,
    controller_config: VelocityControllerConfig,
):
    model, data = xlerobot_model_data
    drive = MujocoMecanumDrive(model, controller_config)

    voltages = drive.apply_control(
        data,
        WheelSpeeds(1.0, 0.0, 0.0, 0.0),
        model.opt.timestep,
    )

    assert voltages.front_left > 0.0
    assert voltages.front_right == 0.0
    assert voltages.rear_left == 0.0
    assert voltages.rear_right == 0.0
    for wheel_name, actuator_name in (
        ('front_left', 'front_left_drive'),
        ('front_right', 'front_right_drive'),
        ('rear_left', 'rear_left_drive'),
        ('rear_right', 'rear_right_drive'),
    ):
        actuator_id = mujoco.mj_name2id(
            model,
            mujoco.mjtObj.mjOBJ_ACTUATOR,
            actuator_name,
        )
        assert data.ctrl[actuator_id] == pytest.approx(
            getattr(voltages, wheel_name)
        )


def test_drive_rejects_model_without_expected_wheels(
    model_data,
    controller_config: VelocityControllerConfig,
):
    model, _ = model_data

    with pytest.raises(ValueError, match='wheel joint not found'):
        MujocoMecanumDrive(model, controller_config)
