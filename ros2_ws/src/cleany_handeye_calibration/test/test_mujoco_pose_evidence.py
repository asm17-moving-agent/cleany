from pathlib import Path

from cleany_handeye_calibration.joint_state_sync import LEFT_ARM_JOINT_NAMES
from cleany_handeye_calibration.models import JointPose
from cleany_handeye_calibration.mujoco_pose_evidence import (
    MujocoPoseEvidenceEvaluator,
)


SCENE_PATH = (
    Path(__file__).parents[2]
    / 'cleany_mujoco_sim'
    / 'scenes'
    / 'handeye.xml.in'
)


def _evaluator():
    return MujocoPoseEvidenceEvaluator(
        SCENE_PATH,
        minimum_camera_depth_m=0.18,
        image_border_fraction=0.08,
    )


def test_scene_evidence_accepts_visible_clear_pose():
    result = _evaluator().evaluate(
        JointPose(
            LEFT_ARM_JOINT_NAMES,
            (-0.794, 2.002, 1.933, 1.412, 0.179),
        )
    )

    assert result.target_visible
    assert result.camera_front
    assert result.minimum_camera_depth_m > 0.18
    assert result.minimum_collision_distance_m > 0.30


def test_scene_evidence_rejects_zero_pose_visibility():
    result = _evaluator().evaluate(
        JointPose(LEFT_ARM_JOINT_NAMES, (0.0,) * 5)
    )

    assert not result.target_visible
