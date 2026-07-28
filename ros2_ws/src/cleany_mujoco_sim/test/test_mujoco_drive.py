import math
from pathlib import Path

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
def xlerobot_model_data():
    scene_path = Path(__file__).parents[1] / 'hardware' / 'scene.xml'
    return load_model(scene_path)


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


def test_drive_tracks_forward_targets_and_stops(
    xlerobot_model_data,
    controller_config: VelocityControllerConfig,
):
    model, data = xlerobot_model_data
    drive = MujocoMecanumDrive(model, controller_config)
    chassis_id = mujoco.mj_name2id(
        model,
        mujoco.mjtObj.mjOBJ_BODY,
        'chassis',
    )
    target = WheelSpeeds(4.0, 4.0, 4.0, 4.0)
    peak_voltage = 0.0

    for _ in range(1000):
        voltages = drive.apply_control(data, target, model.opt.timestep)
        peak_voltage = max(
            peak_voltage,
            abs(voltages.front_left),
            abs(voltages.front_right),
            abs(voltages.rear_left),
            abs(voltages.rear_right),
        )
        mujoco.mj_step(model, data)

    measured = drive.measured_speeds(data)
    assert (
        measured.front_left,
        measured.front_right,
        measured.rear_left,
        measured.rear_right,
    ) == pytest.approx((4.0, 4.0, 4.0, 4.0), abs=0.2)
    assert data.xpos[chassis_id, 0] > 0.4
    assert peak_voltage <= controller_config.voltage_limit

    drive.reset()
    stopped = WheelSpeeds(0.0, 0.0, 0.0, 0.0)
    for _ in range(500):
        drive.apply_control(data, stopped, model.opt.timestep)
        mujoco.mj_step(model, data)

    measured = drive.measured_speeds(data)
    assert max(
        abs(measured.front_left),
        abs(measured.front_right),
        abs(measured.rear_left),
        abs(measured.rear_right),
    ) < 0.05


@pytest.mark.parametrize(
    ('target', 'motion'),
    [
        (WheelSpeeds(-4.0, 4.0, 4.0, -4.0), 'left'),
        (WheelSpeeds(-3.0, 3.0, -3.0, 3.0), 'counterclockwise'),
    ],
)
def test_drive_preserves_mecanum_motion_directions(
    xlerobot_model_data,
    controller_config: VelocityControllerConfig,
    target: WheelSpeeds,
    motion: str,
):
    model, data = xlerobot_model_data
    drive = MujocoMecanumDrive(model, controller_config)
    chassis_id = mujoco.mj_name2id(
        model,
        mujoco.mjtObj.mjOBJ_BODY,
        'chassis',
    )

    for _ in range(500):
        drive.apply_control(data, target, model.opt.timestep)
        mujoco.mj_step(model, data)

    if motion == 'left':
        assert data.xpos[chassis_id, 1] > 0.1
    else:
        quaternion = data.xquat[chassis_id]
        yaw = math.atan2(
            2.0 * (
                quaternion[0] * quaternion[3]
                + quaternion[1] * quaternion[2]
            ),
            1.0 - 2.0 * (
                quaternion[2] ** 2
                + quaternion[3] ** 2
            ),
        )
        assert yaw > 0.2


def test_drive_rejects_model_without_expected_wheels(
    model_data,
    controller_config: VelocityControllerConfig,
):
    model, _ = model_data

    with pytest.raises(ValueError, match='wheel joint not found'):
        MujocoMecanumDrive(model, controller_config)
