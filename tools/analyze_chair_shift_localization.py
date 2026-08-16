#!/usr/bin/env python3
"""Analyze fixed-map localization before and after deterministic chair moves."""

from __future__ import annotations

import csv
import json
import math
from dataclasses import asdict, dataclass
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from rclpy.serialization import deserialize_message

from analyze_slam_algorithm_comparison import (
    Pose,
    deduplicate,
    interpolate,
    open_reader,
    read_estimate,
    read_input,
    relative_delta,
    stamp_seconds,
    wrap,
    yaw_of,
)


@dataclass(frozen=True)
class Alignment:
    rotation_rad: float
    translation_x_m: float
    translation_y_m: float
    yaw_offset_rad: float


@dataclass(frozen=True)
class LocalizationMetrics:
    height_cm: float
    condition: str
    ate_translation_rmse_m: float
    ate_yaw_rmse_deg: float
    rpe_1s_translation_rmse_m: float
    rpe_1s_yaw_rmse_deg: float
    final_translation_error_m: float
    final_yaw_error_deg: float
    max_map_odom_translation_jump_m: float
    max_map_odom_yaw_jump_deg: float
    max_tf_gap_s: float
    trajectory_samples: int


def paired_samples(
    estimate: list[Pose], truth: list[Pose]
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    start = max(estimate[0].stamp, truth[0].stamp)
    end = min(estimate[-1].stamp, truth[-1].stamp - 1.0)
    selected = [pose for pose in estimate if start <= pose.stamp <= end]
    stride = max(1, round(len(selected) / max(1.0, end - start) / 20.0))
    selected = selected[::stride]
    stamps = np.asarray([pose.stamp for pose in selected])
    estimate_array = np.asarray([[pose.x, pose.y, pose.yaw] for pose in selected])
    return stamps, estimate_array, interpolate(truth, stamps)


def fit_alignment(estimate: list[Pose], truth: list[Pose]) -> Alignment:
    _, source, target = paired_samples(estimate, truth)
    source_center = source[:, :2].mean(axis=0)
    target_center = target[:, :2].mean(axis=0)
    covariance = (source[:, :2] - source_center).T @ (
        target[:, :2] - target_center
    )
    u, _, vt = np.linalg.svd(covariance)
    rotation = vt.T @ u.T
    if np.linalg.det(rotation) < 0.0:
        vt[-1, :] *= -1.0
        rotation = vt.T @ u.T
    translation = target_center - rotation @ source_center
    yaw_offset = math.atan2(
        float(np.sin(target[:, 2] - source[:, 2]).sum()),
        float(np.cos(target[:, 2] - source[:, 2]).sum()),
    )
    return Alignment(
        math.atan2(rotation[1, 0], rotation[0, 0]),
        float(translation[0]),
        float(translation[1]),
        yaw_offset,
    )


def apply_alignment(poses: np.ndarray, alignment: Alignment) -> np.ndarray:
    c, s = math.cos(alignment.rotation_rad), math.sin(alignment.rotation_rad)
    rotation = np.asarray(((c, -s), (s, c)))
    result = poses.copy()
    result[:, :2] = (rotation @ poses[:, :2].T).T + np.asarray(
        (alignment.translation_x_m, alignment.translation_y_m)
    )
    result[:, 2] = wrap(poses[:, 2] + alignment.yaw_offset_rad)
    return result


def map_odom_jumps(path: Path) -> tuple[float, float]:
    reader, types = open_reader(path)
    poses: list[Pose] = []
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic != "/tf":
            continue
        message = deserialize_message(data, types[topic])
        for transform in message.transforms:
            if (
                transform.header.frame_id.lstrip("/") == "map"
                and transform.child_frame_id.lstrip("/") == "odom"
            ):
                item = transform.transform
                poses.append(Pose(
                    stamp_seconds(transform.header.stamp),
                    float(item.translation.x),
                    float(item.translation.y),
                    yaw_of(item.rotation),
                ))
    poses = deduplicate(poses)
    translation_jumps: list[float] = []
    yaw_jumps: list[float] = []
    for previous, current in zip(poses, poses[1:]):
        if current.stamp - previous.stamp <= 0.5:
            translation_jumps.append(math.hypot(
                current.x - previous.x, current.y - previous.y
            ))
            yaw_jumps.append(abs(float(wrap(current.yaw - previous.yaw))))
    return (
        max(translation_jumps, default=0.0),
        math.degrees(max(yaw_jumps, default=0.0)),
    )


def evaluate(
    height_cm: float,
    condition: str,
    estimate: list[Pose],
    truth: list[Pose],
    alignment: Alignment,
    result_bag: Path,
) -> tuple[LocalizationMetrics, np.ndarray, np.ndarray]:
    stamps, estimate_array, truth_array = paired_samples(estimate, truth)
    estimate_array = apply_alignment(estimate_array, alignment)
    xy_error = np.linalg.norm(estimate_array[:, :2] - truth_array[:, :2], axis=1)
    yaw_error = np.asarray(wrap(estimate_array[:, 2] - truth_array[:, 2]))
    rpe_xy: list[float] = []
    rpe_yaw: list[float] = []
    for first, stamp in enumerate(stamps):
        second = int(np.searchsorted(stamps, stamp + 1.0))
        if second >= len(stamps) or stamps[second] - stamp > 1.15:
            continue
        estimate_delta = relative_delta(estimate_array, first, second)
        truth_delta = relative_delta(truth_array, first, second)
        rpe_xy.append(float(np.linalg.norm(estimate_delta[:2] - truth_delta[:2])))
        rpe_yaw.append(abs(float(wrap(estimate_delta[2] - truth_delta[2]))))
    translation_jump, yaw_jump = map_odom_jumps(result_bag)
    max_gap = float(np.max(np.diff(stamps))) if len(stamps) > 1 else math.inf
    metrics = LocalizationMetrics(
        height_cm,
        condition,
        float(np.sqrt(np.mean(xy_error**2))),
        math.degrees(float(np.sqrt(np.mean(yaw_error**2)))),
        float(np.sqrt(np.mean(np.square(rpe_xy)))),
        math.degrees(float(np.sqrt(np.mean(np.square(rpe_yaw))))),
        float(xy_error[-1]),
        abs(math.degrees(float(yaw_error[-1]))),
        translation_jump,
        yaw_jump,
        max_gap,
        len(stamps),
    )
    return metrics, estimate_array, truth_array


def render(
    records: list[LocalizationMetrics],
    trajectories: dict[float, tuple[np.ndarray, np.ndarray]],
    output: Path,
) -> None:
    heights = (12.0, 16.5, 26.0)
    colors = {12.0: "#ff315f", 16.5: "#ff9700", 26.0: "#06b6d4"}
    figure = plt.figure(figsize=(16, 10), constrained_layout=True)
    grid = figure.add_gridspec(2, 3, height_ratios=(2.0, 1.25))
    for column, height in enumerate(heights):
        axis = figure.add_subplot(grid[0, column])
        estimate, truth = trajectories[height]
        axis.plot(truth[:, 0], truth[:, 1], color="#263445", lw=2, label="ground truth")
        axis.plot(
            estimate[:, 0], estimate[:, 1], color=colors[height], lw=1.5,
            label="shifted-chair localization",
        )
        axis.set_title(f"{height:g} cm")
        axis.set_aspect("equal")
        axis.grid(alpha=0.25)
        axis.legend(fontsize=9)
    metrics = {
        (item.height_cm, item.condition): item for item in records
    }
    labels = [f"{height:g} cm" for height in heights]
    x = np.arange(len(heights))
    panels = (
        ("ATE translation (cm)", "ate_translation_rmse_m", 100.0),
        ("1 s RPE translation (cm)", "rpe_1s_translation_rmse_m", 100.0),
        ("Max map→odom jump (cm)", "max_map_odom_translation_jump_m", 100.0),
    )
    for column, (title, field, scale) in enumerate(panels):
        axis = figure.add_subplot(grid[1, column])
        baseline = [scale * getattr(metrics[(height, "baseline")], field) for height in heights]
        shifted = [scale * getattr(metrics[(height, "shifted")], field) for height in heights]
        axis.bar(x - 0.18, baseline, 0.36, color="#aab3bf", label="original chairs")
        axis.bar(
            x + 0.18, shifted, 0.36,
            color=[colors[height] for height in heights], label="12 chairs moved",
        )
        axis.set_title(title)
        axis.set_xticks(x, labels)
        axis.grid(axis="y", alpha=0.25)
        axis.legend(fontsize=8)
    figure.suptitle(
        "Fixed-map slam_toolbox localization — original vs 20 cm chair shifts",
        fontsize=18,
    )
    figure.savefig(output, dpi=160)
    plt.close(figure)


def main() -> None:
    results = Path("ros2_ws/slam_results")
    study = results / "chair_shift_localization"
    height_tokens = ((12.0, "12"), (16.5, "16p5"), (26.0, "26"))
    records: list[LocalizationMetrics] = []
    trajectories: dict[float, tuple[np.ndarray, np.ndarray]] = {}
    alignments: dict[str, dict] = {}
    for height, token in height_tokens:
        estimates: dict[str, list[Pose]] = {}
        truths: dict[str, list[Pose]] = {}
        inputs = {
            "baseline": results / "algorithm_compare_inputs" / f"input_{token}cm_trial1",
            "shifted": study / "inputs" / f"{token}cm_shifted",
        }
        for condition in ("baseline", "shifted"):
            run = study / "runs" / f"{token}cm" / condition
            if not (run / "run_complete").exists():
                raise RuntimeError(f"incomplete localization run: {run}")
            estimates[condition] = read_estimate(run / "result_bag")
            truths[condition] = read_input(inputs[condition])[0]
        alignment = fit_alignment(estimates["baseline"], truths["baseline"])
        alignments[token] = asdict(alignment)
        for condition in ("baseline", "shifted"):
            run_bag = study / "runs" / f"{token}cm" / condition / "result_bag"
            metrics, estimate_array, truth_array = evaluate(
                height, condition, estimates[condition], truths[condition],
                alignment, run_bag,
            )
            records.append(metrics)
            if condition == "shifted":
                trajectories[height] = (estimate_array, truth_array)
    study.mkdir(parents=True, exist_ok=True)
    rows = [asdict(record) for record in records]
    (study / "metrics.json").write_text(
        json.dumps(rows, indent=2) + "\n", encoding="utf-8"
    )
    (study / "alignments.json").write_text(
        json.dumps(alignments, indent=2) + "\n", encoding="utf-8"
    )
    with (study / "metrics.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    lines = [
        "# Chair-shift fixed-map localization",
        "",
        "Twelve chairs were moved 0.20 m toward their desks and rotated "
        "alternately by ±10°. Each height loads its original fixed posegraph.",
        "The baseline-derived map-to-world alignment is reused unchanged for "
        "the shifted condition.",
        "",
        "| Height | Original ATE | Shifted ATE | Δ ATE | Original RPE | Shifted RPE | Max shifted correction jump |",
        "|---:|---:|---:|---:|---:|---:|---:|",
    ]
    by_key = {(row.height_cm, row.condition): row for row in records}
    for height, _ in height_tokens:
        base = by_key[(height, "baseline")]
        shifted = by_key[(height, "shifted")]
        lines.append(
            f"| {height:g} cm | {100*base.ate_translation_rmse_m:.2f} cm | "
            f"{100*shifted.ate_translation_rmse_m:.2f} cm | "
            f"{100*(shifted.ate_translation_rmse_m-base.ate_translation_rmse_m):+.2f} cm | "
            f"{100*base.rpe_1s_translation_rmse_m:.2f} cm | "
            f"{100*shifted.rpe_1s_translation_rmse_m:.2f} cm | "
            f"{100*shifted.max_map_odom_translation_jump_m:.2f} cm |"
        )
    lines.append("")
    (study / "SUMMARY.md").write_text("\n".join(lines), encoding="utf-8")
    render(records, trajectories, study / "localization_comparison.png")
    print("\n".join(lines))


if __name__ == "__main__":
    main()
