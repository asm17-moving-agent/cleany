from dataclasses import replace

from cleany_handeye_calibration.camera_acquisition import (
    CAMERA_D,
    CAMERA_DISTORTION_MODEL,
    CAMERA_FRAME_ID,
    CAMERA_HEIGHT,
    CAMERA_K,
    CAMERA_P,
    CAMERA_R,
    CAMERA_WIDTH,
    CameraInfoFrame,
)
from cleany_handeye_calibration.joint_state_sync import (
    DEFAULT_DUAL_ARM_JOINT_CONTRACT,
    LEFT_ARM_JOINT_NAMES,
)
from cleany_handeye_calibration.models import (
    CalibrationSample,
    JointPose,
    PositionTarget,
    SampleSplit,
    TimedJointSample,
)
from cleany_handeye_calibration.schema import CalibrationSampleRecord
from cleany_handeye_calibration.target_detector import (
    INNER_CORNER_COUNT,
    analyze_charuco_corners,
)
from cleany_handeye_calibration.transforms import RigidTransform


JOINT_NAMES = DEFAULT_DUAL_ARM_JOINT_CONTRACT.required_joint_names


def evaluation_records():
    detection = analyze_charuco_corners(
        tuple(range(INNER_CORNER_COUNT)),
        tuple(
            (100.0 + (index % 6) * 40.0, 80.0 + (index // 6) * 50.0)
            for index in range(INNER_CORNER_COUNT)
        ),
    )
    seed = JointPose(LEFT_ARM_JOINT_NAMES, (0.0, 0.1, 0.2, 0.3, 0.4))
    records = []
    for index in range(25):
        split = (
            SampleSplit.CALIBRATION
            if index < 20
            else SampleSplit.HELD_OUT
        )
        split_index = index + 1 if index < 20 else index - 19
        prefix = 'calibration' if index < 20 else 'held_out'
        sample_id = f'sample_{prefix}_{split_index:03d}'
        pose_id = f'{prefix}_{split_index:03d}'
        stamp = 10_000 + index * 10
        base = RigidTransform.from_rodrigues(
            parent_frame='base_link',
            child_frame='left_gripper_frame',
            translation_m=(0.30 + index * 0.001, 0.18, 0.52),
            rodrigues_vector=(0.03 * index, -0.01 * index, 0.02),
        )
        camera = RigidTransform.from_rodrigues(
            parent_frame=CAMERA_FRAME_ID,
            child_frame='charuco_target',
            translation_m=(0.01, -0.02, 0.45),
            rodrigues_vector=(0.2, -0.1, 0.05),
        )
        positions = tuple(index * 0.001 + joint / 10.0 for joint in range(12))
        records.append(
            CalibrationSampleRecord(
                sample=CalibrationSample(
                    sample_id=sample_id,
                    pose_id=pose_id,
                    split=split,
                    base_T_gripper=base,
                    camera_T_target=camera,
                ),
                calibration_arm='left',
                planning_group='left_arm',
                position_target=PositionTarget(
                    'base_link',
                    (0.3, 0.2, 0.5),
                ),
                ik_seed=seed,
                resolved_ik=replace(
                    seed,
                    positions_rad=tuple(
                        value + index * 0.001
                        for value in seed.positions_rad
                    ),
                ),
                image_stamp_ns=stamp,
                joint_state_before_stamp_ns=stamp - 5,
                joint_state_after_stamp_ns=stamp + 5,
                joint_interpolation_ratio=0.5,
                interpolated_joints=TimedJointSample(
                    stamp,
                    JOINT_NAMES,
                    positions,
                    (0.0,) * 12,
                ),
                camera_info=CameraInfoFrame(
                    stamp,
                    CAMERA_FRAME_ID,
                    CAMERA_WIDTH,
                    CAMERA_HEIGHT,
                    CAMERA_DISTORTION_MODEL,
                    CAMERA_D,
                    CAMERA_K,
                    CAMERA_R,
                    CAMERA_P,
                ),
                target_detection=detection,
                pnp_method='SOLVEPNP_IPPE',
                pnp_reprojection_rmse_px=0.2,
                pnp_ambiguous=False,
                pnp_selected_candidate_index=0,
                image_path=f'images/{sample_id}.png',
            )
        )
    return tuple(records)
