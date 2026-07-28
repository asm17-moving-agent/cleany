from __future__ import annotations

import mujoco

from cleany_mujoco_sim.mecanum_kinematics import WheelSpeeds
from cleany_mujoco_sim.wheel_speed_controller import (
    VelocityControllerConfig,
    WheelSpeedController,
    WheelVoltages,
)


_WHEEL_BINDINGS = (
    ('front_left', 'front_left_wheel_joint', 'front_left_drive'),
    ('front_right', 'front_right_wheel_joint', 'front_right_drive'),
    ('rear_left', 'left_wheel_joint', 'rear_left_drive'),
    ('rear_right', 'right_wheel_joint', 'rear_right_drive'),
)


class MujocoMecanumDrive:
    def __init__(
        self,
        model: mujoco.MjModel,
        controller_config: VelocityControllerConfig,
    ) -> None:
        self._controller = WheelSpeedController(controller_config)
        self._joint_dof_ids: dict[str, int] = {}
        self._actuator_ids: dict[str, int] = {}

        for wheel_name, joint_name, actuator_name in _WHEEL_BINDINGS:
            joint_id = mujoco.mj_name2id(
                model,
                mujoco.mjtObj.mjOBJ_JOINT,
                joint_name,
            )
            actuator_id = mujoco.mj_name2id(
                model,
                mujoco.mjtObj.mjOBJ_ACTUATOR,
                actuator_name,
            )
            if joint_id < 0:
                raise ValueError(f'MuJoCo wheel joint not found: {joint_name}')
            if actuator_id < 0:
                raise ValueError(f'MuJoCo drive actuator not found: {actuator_name}')
            if model.actuator_trnid[actuator_id, 0] != joint_id:
                raise ValueError(
                    f'MuJoCo actuator {actuator_name} is not bound to {joint_name}'
                )

            control_range = model.actuator_ctrlrange[actuator_id]
            if (
                control_range[0] > -controller_config.voltage_limit
                or control_range[1] < controller_config.voltage_limit
            ):
                raise ValueError(
                    f'MuJoCo actuator voltage range is too small: {actuator_name}'
                )

            self._joint_dof_ids[wheel_name] = int(model.jnt_dofadr[joint_id])
            self._actuator_ids[wheel_name] = actuator_id

    def measured_speeds(self, data: mujoco.MjData) -> WheelSpeeds:
        return WheelSpeeds(
            front_left=float(data.qvel[self._joint_dof_ids['front_left']]),
            front_right=float(data.qvel[self._joint_dof_ids['front_right']]),
            rear_left=float(data.qvel[self._joint_dof_ids['rear_left']]),
            rear_right=float(data.qvel[self._joint_dof_ids['rear_right']]),
        )

    def apply_control(
        self,
        data: mujoco.MjData,
        target: WheelSpeeds,
        dt: float,
    ) -> WheelVoltages:
        voltages = self._controller.update(
            target,
            self.measured_speeds(data),
            dt,
        )
        data.ctrl[self._actuator_ids['front_left']] = voltages.front_left
        data.ctrl[self._actuator_ids['front_right']] = voltages.front_right
        data.ctrl[self._actuator_ids['rear_left']] = voltages.rear_left
        data.ctrl[self._actuator_ids['rear_right']] = voltages.rear_right
        return voltages

    def reset(self) -> None:
        self._controller.reset()
