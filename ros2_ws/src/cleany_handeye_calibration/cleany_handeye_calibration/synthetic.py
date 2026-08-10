"""Deterministic, mathematically consistent eye-in-hand datasets for tests."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from cleany_handeye_calibration.models import (
    CalibrationSample,
    SampleSplit,
)
from cleany_handeye_calibration.solver import (
    DEFAULT_HAND_EYE_FRAMES,
    HandEyeFrameConvention,
)
from cleany_handeye_calibration.transforms import RigidTransform


@dataclass(frozen=True, slots=True)
class SyntheticHandEyeDataset:
    calibration_samples: tuple[CalibrationSample, ...]
    held_out_samples: tuple[CalibrationSample, ...]
    gripper_T_camera_ground_truth: RigidTransform
    base_T_target_ground_truth: RigidTransform
    frame_convention: HandEyeFrameConvention
    random_seed: int


def _random_base_gripper_pose(
    rng: np.random.Generator,
    index: int,
    frames: HandEyeFrameConvention,
) -> RigidTransform:
    axis = rng.normal(size=3)
    axis /= np.linalg.norm(axis)
    angle = float(rng.uniform(-1.3, 1.3))
    if abs(angle) < 0.25:
        angle = 0.25 if index % 2 == 0 else -0.25
    translation = rng.uniform(
        low=np.array((0.15, -0.30, 0.20)),
        high=np.array((0.55, 0.30, 0.75)),
    )
    return RigidTransform.from_rodrigues(
        parent_frame=frames.base_frame,
        child_frame=frames.gripper_frame,
        translation_m=translation,
        rodrigues_vector=axis * angle,
    )


def generate_synthetic_hand_eye_dataset(
    *,
    calibration_pose_count: int = 20,
    held_out_pose_count: int = 5,
    random_seed: int = 42,
    frame_convention: HandEyeFrameConvention = DEFAULT_HAND_EYE_FRAMES,
) -> SyntheticHandEyeDataset:
    """Generate samples satisfying ``bTg @ gTc @ cTt == bTt`` exactly."""

    if calibration_pose_count < 3:
        raise ValueError('calibration_pose_count must be at least 3')
    if held_out_pose_count < 0:
        raise ValueError('held_out_pose_count must be non-negative')
    if isinstance(random_seed, bool) or not isinstance(random_seed, int):
        raise ValueError('random_seed must be an integer')

    gripper_T_camera = RigidTransform.from_rodrigues(
        parent_frame=frame_convention.gripper_frame,
        child_frame=frame_convention.camera_frame,
        translation_m=(0.03, -0.02, 0.08),
        rodrigues_vector=(0.12, -0.25, 0.09),
    )
    base_T_target = RigidTransform.from_rodrigues(
        parent_frame=frame_convention.base_frame,
        child_frame=frame_convention.target_frame,
        translation_m=(0.62, 0.08, 0.50),
        rodrigues_vector=(0.05, 0.02, -0.15),
    )
    rng = np.random.default_rng(random_seed)
    calibration_samples: list[CalibrationSample] = []
    held_out_samples: list[CalibrationSample] = []
    total_count = calibration_pose_count + held_out_pose_count
    for index in range(total_count):
        base_T_gripper = _random_base_gripper_pose(
            rng,
            index,
            frame_convention,
        )
        camera_T_target = (
            gripper_T_camera.inverse()
            @ base_T_gripper.inverse()
            @ base_T_target
        )
        if index < calibration_pose_count:
            split = SampleSplit.CALIBRATION
            split_index = index + 1
            pose_id = f'calibration_{split_index:03d}'
            destination = calibration_samples
        else:
            split = SampleSplit.HELD_OUT
            split_index = index - calibration_pose_count + 1
            pose_id = f'held_out_{split_index:03d}'
            destination = held_out_samples
        destination.append(
            CalibrationSample(
                sample_id=f'synthetic_{pose_id}',
                pose_id=pose_id,
                split=split,
                base_T_gripper=base_T_gripper,
                camera_T_target=camera_T_target,
            )
        )

    return SyntheticHandEyeDataset(
        calibration_samples=tuple(calibration_samples),
        held_out_samples=tuple(held_out_samples),
        gripper_T_camera_ground_truth=gripper_T_camera,
        base_T_target_ground_truth=base_T_target,
        frame_convention=frame_convention,
        random_seed=random_seed,
    )
