import numpy as np
import pytest

from cleany_skill_executor.core.can_rgbd import (
    CameraProjection,
    render_grasp_overlay,
    segment_red_can,
)


def test_red_can_pixels_are_projected_into_base_frame() -> None:
    rgb = np.zeros((12, 16, 3), dtype=np.uint8)
    rgb[4:8, 6:10] = (220, 30, 20)
    depth = np.full((12, 16), 2.0, dtype=np.float32)
    camera = CameraProjection(
        fx=10.0,
        fy=10.0,
        cx=7.5,
        cy=5.5,
        translation_base=(1.0, 2.0, 3.0),
        rotation_base_from_optical=(
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0,
        ),
    )

    result = segment_red_can(
        rgb,
        depth,
        camera,
        context_margin_pixels=1,
        minimum_target_pixels=10,
    )

    assert result.target_points.shape == (16, 3)
    assert result.context_points.shape == (36, 3)
    assert result.target_points[:, 2] == pytest.approx(5.0)
    assert result.target_points[:, 0].min() == pytest.approx(0.7)
    assert result.target_points[:, 0].max() == pytest.approx(1.3)
    assert np.all(result.target_colors == (220, 30, 20))


def test_red_can_segmentation_rejects_missing_target() -> None:
    camera = CameraProjection(
        fx=10.0,
        fy=10.0,
        cx=1.0,
        cy=1.0,
        translation_base=(0.0, 0.0, 0.0),
        rotation_base_from_optical=tuple(np.eye(3).reshape(-1)),
    )

    with pytest.raises(ValueError, match='found only 0 pixels'):
        segment_red_can(
            np.zeros((3, 3, 3), dtype=np.uint8),
            np.ones((3, 3), dtype=np.float32),
            camera,
            minimum_target_pixels=1,
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


def test_segmentation_rejects_non_positive_cloud_limit() -> None:
    camera = CameraProjection(
        fx=10.0,
        fy=10.0,
        cx=1.0,
        cy=1.0,
        translation_base=(0.0, 0.0, 0.0),
        rotation_base_from_optical=tuple(np.eye(3).reshape(-1)),
    )
    with pytest.raises(ValueError, match='point-cloud limits'):
        segment_red_can(
            np.zeros((3, 3, 3), dtype=np.uint8),
            np.ones((3, 3), dtype=np.float32),
            camera,
            target_maximum_points=0,
        )


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
