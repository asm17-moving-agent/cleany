#!/usr/bin/env python3
"""Render SLAM occupied cells over the fixed Gazebo top-view reference."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import cv2
import matplotlib.pyplot as plt
import numpy as np
import yaml
from PIL import Image


HEIGHT_COLORS = {
    16.5: (255, 23, 68),
    26: (0, 172, 193),
    45: (255, 143, 0),
    70: (124, 77, 255),
}
ALGORITHM_LABELS = {
    "slam_toolbox": "slam_toolbox\nLiDAR + wheel",
    "cartographer": "Cartographer 2D\nLiDAR + wheel",
    "cartographer_imu": "Cartographer 2D + IMU\nLiDAR + wheel + IMU",
    "rtabmap": "RTAB-Map 2D\nLiDAR + wheel",
}

# The top-view projection is fixed because every bag was recorded in the same world.
WORLD_CORNERS = np.float32(
    [(-6.175, 5.533), (6.175, 5.533), (6.175, -5.517), (-6.175, -5.517)]
)
INITIAL_WORLD_POSE = (-1.865, -4.705, np.pi / 2.0)
CAMERA_HEIGHT_M = 18.0
CAMERA_HORIZONTAL_FOV_RAD = 0.72


def recover_reference(overlays: list[Path]) -> np.ndarray:
    images = np.stack([np.asarray(Image.open(path).convert("RGB")) for path in overlays])
    reference = np.median(images, axis=0).astype(np.uint8)
    variation = images.max(axis=0).astype(np.int16) - images.min(axis=0).astype(np.int16)
    affected = (variation.max(axis=2) > 12).astype(np.uint8) * 255
    affected = cv2.dilate(affected, np.ones((3, 3), np.uint8), iterations=1)
    recovered = cv2.inpaint(
        cv2.cvtColor(reference, cv2.COLOR_RGB2BGR),
        affected,
        3,
        cv2.INPAINT_TELEA,
    )[..., ::-1]
    # The old overlays all contain a legend at the upper left. The room is
    # symmetric across its vertical axis, so use the unobstructed opposite side
    # to recover that small reference-only region.
    mirrored = np.fliplr(recovered)
    recovered[8:80, 18:410] = mirrored[8:80, 18:410]
    return recovered


def world_to_image_homography(image_shape: tuple[int, ...]) -> np.ndarray:
    height, width = image_shape[:2]
    visible_width = 2.0 * CAMERA_HEIGHT_M * np.tan(
        CAMERA_HORIZONTAL_FOV_RAD / 2.0
    )
    pixels_per_meter = width / visible_width
    image_corners = np.float32(
        [
            (width / 2.0 + x * pixels_per_meter, height / 2.0 - y * pixels_per_meter)
            for x, y in WORLD_CORNERS
        ]
    )
    return cv2.getPerspectiveTransform(WORLD_CORNERS, image_corners)


def project(points: np.ndarray, homography: np.ndarray) -> np.ndarray:
    return cv2.perspectiveTransform(points.reshape(1, -1, 2), homography)[0]


def map_corners(run: Path) -> tuple[np.ndarray, np.ndarray]:
    with (run / "map_final.yaml").open(encoding="utf-8") as stream:
        metadata = yaml.safe_load(stream)
    pixels = np.asarray(Image.open(run / "map_final.pgm"))
    height, width = pixels.shape
    resolution = float(metadata["resolution"])
    origin_x, origin_y = (float(value) for value in metadata["origin"][:2])
    map_xy = np.float32(
        [
            (origin_x, origin_y + height * resolution),
            (origin_x + width * resolution, origin_y + height * resolution),
            (origin_x + width * resolution, origin_y),
            (origin_x, origin_y),
        ]
    )
    x0, y0, yaw0 = INITIAL_WORLD_POSE
    c, s = np.cos(yaw0), np.sin(yaw0)
    world_xy = np.column_stack(
        (
            x0 + c * map_xy[:, 0] - s * map_xy[:, 1],
            y0 + s * map_xy[:, 0] + c * map_xy[:, 1],
        )
    ).astype(np.float32)
    return pixels, world_xy


def render_one(reference: np.ndarray, run: Path, height_cm: float, homography: np.ndarray) -> np.ndarray:
    pixels, world_corners = map_corners(run)
    destination = project(world_corners, homography).astype(np.float32)
    source = np.float32(
        [(0, 0), (pixels.shape[1], 0), (pixels.shape[1], pixels.shape[0]), (0, pixels.shape[0])]
    )
    map_to_image = cv2.getPerspectiveTransform(source, destination)
    occupied = (pixels <= 100).astype(np.uint8) * 255
    warped = cv2.warpPerspective(
        occupied,
        map_to_image,
        (reference.shape[1], reference.shape[0]),
        flags=cv2.INTER_NEAREST,
    )
    result = reference.astype(np.float32)
    color = np.asarray(HEIGHT_COLORS[height_cm], dtype=np.float32)
    mask = warped > 0
    result[mask] = result[mask] * 0.28 + color * 0.72
    return np.clip(result, 0, 255).astype(np.uint8)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--results-root", type=Path, default=Path("ros2_ws/slam_results"))
    args = parser.parse_args()
    output = args.results_root / "algorithm_comparison"
    output.mkdir(parents=True, exist_ok=True)
    reference_path = output / "gazebo_top_view_reference.png"
    if not reference_path.exists():
        raise FileNotFoundError(
            f"capture a fresh Gazebo image first: {reference_path}"
        )
    reference = np.asarray(Image.open(reference_path).convert("RGB"))
    homography = world_to_image_homography(reference.shape)

    rendered: dict[tuple[str, int], np.ndarray] = {}
    for algorithm in ALGORITHM_LABELS:
        for height in HEIGHT_COLORS:
            token = "16p5" if height == 16.5 else f"{height:g}"
            run = args.results_root / "algorithm_compare_runs" / algorithm / f"{token}cm"
            image = render_one(reference, run, height, homography)
            rendered[algorithm, height] = image
            Image.fromarray(image).save(run / "gazebo_overlay.png")

    metrics = json.loads((output / "metrics.json").read_text(encoding="utf-8"))
    lookup = {(row["algorithm"], row["height_cm"]): row for row in metrics}
    figure, axes = plt.subplots(4, 4, figsize=(18, 16), constrained_layout=True)
    figure.patch.set_facecolor("#f4f6f8")
    for row_index, algorithm in enumerate(ALGORITHM_LABELS):
        for column_index, height in enumerate(HEIGHT_COLORS):
            axis = axes[row_index, column_index]
            axis.imshow(rendered[algorithm, height])
            axis.set_axis_off()
            item = lookup[algorithm, height]
            axis.set_title(
                f"{height:g} cm  |  ATE {item['ate_translation_rmse_m'] * 100:.1f} cm  |  "
                f"RPE {item['rpe_1s_translation_rmse_m'] * 100:.1f} cm",
                fontsize=11,
                color=np.asarray(HEIGHT_COLORS[height]) / 255.0,
                fontweight="bold",
            )
            if column_index == 0:
                axis.text(
                    -0.04,
                    0.5,
                    ALGORITHM_LABELS[algorithm],
                    transform=axis.transAxes,
                    ha="right",
                    va="center",
                    fontsize=12,
                    fontweight="bold",
                )
    figure.suptitle(
        "Cleany offline SLAM comparison — identical sensor bags, 2.5× replay",
        fontsize=20,
        fontweight="bold",
    )
    figure.savefig(output / "comparison_overlays_4x4.png", dpi=150)
    plt.close(figure)


if __name__ == "__main__":
    main()
