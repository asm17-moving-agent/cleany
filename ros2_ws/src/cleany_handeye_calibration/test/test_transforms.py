from dataclasses import FrozenInstanceError
import math

import numpy as np
import pytest

from cleany_handeye_calibration.transforms import (
    RigidTransform,
    quaternion_xyzw_from_rotation_matrix,
    rodrigues_from_rotation_matrix,
    rotation_matrix_from_quaternion_xyzw,
    rotation_matrix_from_rodrigues,
    validate_rotation_matrix,
)


def _assert_transform_close(
    actual: RigidTransform,
    expected: RigidTransform,
    *,
    atol: float = 1.0e-10,
) -> None:
    assert actual.parent_frame == expected.parent_frame
    assert actual.child_frame == expected.child_frame
    np.testing.assert_allclose(
        actual.as_homogeneous_matrix(),
        expected.as_homogeneous_matrix(),
        rtol=0.0,
        atol=atol,
    )


def test_eye_in_hand_transform_convention_composes_to_base_target():
    base_T_gripper = RigidTransform.from_rodrigues(
        parent_frame='base_link',
        child_frame='left_gripper_frame',
        translation_m=(0.31, 0.18, 0.52),
        rodrigues_vector=(0.3, -0.1, 0.2),
    )
    gripper_T_camera = RigidTransform.from_rodrigues(
        parent_frame='left_gripper_frame',
        child_frame='left_wrist_rgb_optical_frame',
        translation_m=(0.03, -0.02, 0.08),
        rodrigues_vector=(0.12, -0.25, 0.09),
    )
    base_T_target = RigidTransform.from_rodrigues(
        parent_frame='base_link',
        child_frame='charuco_target',
        translation_m=(0.62, 0.08, 0.50),
        rodrigues_vector=(0.05, 0.02, -0.15),
    )
    camera_T_target = (
        gripper_T_camera.inverse()
        @ base_T_gripper.inverse()
        @ base_T_target
    )

    reconstructed = (
        base_T_gripper @ gripper_T_camera @ camera_T_target
    )

    _assert_transform_close(reconstructed, base_T_target)


def test_transform_inverse_round_trip_preserves_frames_and_values():
    transform = RigidTransform.from_rodrigues(
        parent_frame='parent',
        child_frame='child',
        translation_m=(0.1, -0.2, 0.3),
        rodrigues_vector=(0.4, 0.2, -0.3),
    )

    _assert_transform_close(
        transform @ transform.inverse(),
        RigidTransform.identity('parent'),
    )
    _assert_transform_close(transform.inverse().inverse(), transform)


def test_disconnected_transform_composition_is_rejected():
    base_T_gripper = RigidTransform.identity('base_link')
    camera_T_target = RigidTransform(
        parent_frame='camera',
        child_frame='target',
        rotation_matrix=np.eye(3),
        translation_m=(0.0, 0.0, 1.0),
    )

    with pytest.raises(ValueError, match='disconnected transforms'):
        base_T_gripper @ camera_T_target


@pytest.mark.parametrize(
    'rodrigues_vector',
    [
        (0.0, 0.0, 0.0),
        (0.2, -0.4, 0.3),
        (math.pi - 1.0e-8, 0.0, 0.0),
    ],
)
def test_rotation_matrix_quaternion_and_rodrigues_round_trip(
    rodrigues_vector,
):
    rotation = rotation_matrix_from_rodrigues(rodrigues_vector)

    quaternion = quaternion_xyzw_from_rotation_matrix(rotation)
    quaternion_round_trip = rotation_matrix_from_quaternion_xyzw(
        quaternion
    )
    rodrigues = rodrigues_from_rotation_matrix(rotation)
    rodrigues_round_trip = rotation_matrix_from_rodrigues(rodrigues)

    np.testing.assert_allclose(
        quaternion_round_trip,
        rotation,
        rtol=0.0,
        atol=1.0e-10,
    )
    np.testing.assert_allclose(
        rodrigues_round_trip,
        rotation,
        rtol=0.0,
        atol=1.0e-8,
    )
    assert quaternion[3] >= -np.finfo(np.float64).eps


@pytest.mark.parametrize(
    'rotation',
    [
        np.diag((1.0, 1.0, -1.0)),
        np.diag((1.0, 1.0, 1.01)),
        np.array(
            [
                [1.0, 0.0, 0.0],
                [0.0, float('nan'), 0.0],
                [0.0, 0.0, 1.0],
            ]
        ),
        np.eye(4),
    ],
)
def test_invalid_rotation_matrix_is_rejected(rotation):
    with pytest.raises(ValueError):
        validate_rotation_matrix(rotation)


def test_non_unit_quaternion_is_rejected():
    with pytest.raises(ValueError, match='unit norm'):
        rotation_matrix_from_quaternion_xyzw((0.0, 0.0, 0.0, 2.0))


def test_invalid_homogeneous_bottom_row_is_rejected():
    matrix = np.eye(4)
    matrix[3, 0] = 0.1

    with pytest.raises(ValueError, match='bottom row'):
        RigidTransform.from_homogeneous_matrix(
            parent_frame='parent',
            child_frame='child',
            matrix=matrix,
        )


def test_transform_copies_input_arrays_and_is_immutable():
    rotation = np.eye(3)
    translation = np.array((0.1, 0.2, 0.3))
    transform = RigidTransform(
        parent_frame='parent',
        child_frame='child',
        rotation_matrix=rotation,
        translation_m=translation,
    )

    rotation[0, 0] = 9.0
    translation[0] = 9.0

    assert transform.rotation_matrix[0][0] == 1.0
    assert transform.translation_m[0] == 0.1
    with pytest.raises(FrozenInstanceError):
        transform.parent_frame = 'different'
