import numpy as np
import pytest

from cleany_skill_executor.core.rgbd_projection import (
    CameraProjection,
    render_grasp_overlay,
    rotation_matrix_from_quaternion,
)


def test_camera_projection_rejects_non_finite_translation() -> None:
    with pytest.raises(ValueError, match='translation'):
        CameraProjection(
            fx=10.0,
            fy=10.0,
            cx=1.0,
            cy=1.0,
            translation_base=(0.0, float('nan'), 0.0),
            rotation_base_from_optical=tuple(np.eye(3).reshape(-1)),
        )


def test_quaternion_rotation_matrix_is_normalized() -> None:
    rotation = rotation_matrix_from_quaternion(0.0, 0.0, 2.0, 2.0)

    assert rotation.T @ rotation == pytest.approx(np.eye(3), abs=1.0e-12)
    assert np.linalg.det(rotation) == pytest.approx(1.0)


def test_quaternion_rotation_matrix_rejects_zero() -> None:
    with pytest.raises(ValueError, match='finite and non-zero'):
        rotation_matrix_from_quaternion(0.0, 0.0, 0.0, 0.0)


def test_grasp_overlay_draws_candidates_and_selected_angle_panel() -> None:
    rgb = np.zeros((240, 320, 3), dtype=np.uint8)
    camera = CameraProjection(
        fx=200.0,
        fy=200.0,
        cx=159.5,
        cy=119.5,
        translation_base=(0.0, 0.0, 0.0),
        rotation_base_from_optical=tuple(np.eye(3).reshape(-1)),
    )

    rendered = render_grasp_overlay(
        rgb,
        camera,
        np.asarray(((0.0, 0.0, 1.0), (0.1, 0.0, 1.0))),
        np.asarray(((0.0, -1.0, 0.0), (0.0, 0.0, -1.0))),
        np.asarray((0.8, 0.7)),
        np.asarray((0.07, 0.06)),
        selected_index=1,
        selected_arm='left',
    )

    assert rendered.shape == rgb.shape
    assert rendered.dtype == np.uint8
    assert np.any(rendered[:, :, 1] == 255)
