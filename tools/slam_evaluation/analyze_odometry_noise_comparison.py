#!/usr/bin/env python3
"""Compute metrics for the 30 cm odometry-noise SLAM matrix."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

from rclpy.serialization import deserialize_message

from analyze_slam_algorithm_comparison import (
    Pose,
    deduplicate,
    map_metrics,
    open_reader,
    read_estimate,
    read_input,
    stamp_seconds,
    trajectory_metrics,
    yaw_of,
)


LEVELS = ("level0", "level1", "level2", "level3")
ALGORITHMS = ("slam_toolbox", "cartographer")


def read_odometry(path: Path, topic_name: str = "/odom") -> list[Pose]:
    """Read one odometry pose stream from a bag."""
    reader, types = open_reader(path)
    poses: list[Pose] = []
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic != topic_name:
            continue
        message = deserialize_message(data, types[topic])
        pose = message.pose.pose
        poses.append(
            Pose(
                stamp_seconds(message.header.stamp),
                float(pose.position.x),
                float(pose.position.y),
                yaw_of(pose.orientation),
            )
        )
    return deduplicate(poses)


def analyze(root: Path, experiment_name: str) -> list[dict[str, object]]:
    """Analyze all eight algorithm and odometry-level combinations."""
    records: list[dict[str, object]] = []
    for level in LEVELS:
        input_path = (
            root / experiment_name / "inputs" / level / "input_30cm_trial1"
        )
        truth, scans, finite, beams = read_input(input_path)
        input_odometry = trajectory_metrics(
            read_odometry(input_path), truth
        )
        for algorithm in ALGORITHMS:
            run_path = root / experiment_name / "runs" / algorithm / level
            if not (run_path / "run_complete").is_file():
                raise RuntimeError(f"incomplete run: {run_path}")
            trajectory = trajectory_metrics(
                read_estimate(run_path / "result_bag"), truth
            )
            map_values = map_metrics(run_path)
            records.append(
                {
                    "odometry_level": level,
                    "algorithm": algorithm,
                    "height_cm": 30.0,
                    "ate_translation_rmse_m": trajectory[0],
                    "ate_yaw_rmse_deg": trajectory[1],
                    "rpe_1s_translation_rmse_m": trajectory[2],
                    "rpe_1s_yaw_rmse_deg": trajectory[3],
                    "final_translation_error_m": trajectory[4],
                    "final_yaw_error_deg": trajectory[5],
                    "trajectory_samples": trajectory[6],
                    "input_odom_ate_translation_rmse_m": (
                        input_odometry[0]
                    ),
                    "input_odom_rpe_1s_translation_rmse_m": (
                        input_odometry[2]
                    ),
                    "input_odom_ate_yaw_rmse_deg": input_odometry[1],
                    "input_scans": scans,
                    "finite_scan_returns_percent": 100.0 * finite / beams,
                    "occupied_cells": map_values[0],
                    "known_cells": map_values[1],
                    "known_area_m2": map_values[2],
                    "map_width_cells": map_values[3],
                    "map_height_cells": map_values[4],
                    "map_resolution_m": map_values[5],
                }
            )
    return records


def write_outputs(records: list[dict[str, object]], output: Path) -> None:
    """Write machine-readable metrics and a concise Markdown summary."""
    output.mkdir(parents=True, exist_ok=True)
    (output / "metrics.json").write_text(
        json.dumps(records, indent=2) + "\n", encoding="utf-8"
    )
    with (output / "metrics.csv").open(
        "w", newline="", encoding="utf-8"
    ) as stream:
        writer = csv.DictWriter(stream, fieldnames=list(records[0]))
        writer.writeheader()
        writer.writerows(records)

    lines = [
        "# 30 cm odometry-noise SLAM comparison",
        "",
        (
            "All cells reuse the same 30 cm measured-LiDAR route. Odom "
            "levels are deterministic synthetic robustness inputs, not "
            "measured hardware noise. ATE uses scale-fixed SE(2) alignment; "
            "RPE uses an approximately 1 s interval."
        ),
        "",
        "## Input odometry versus ground truth",
        "",
        "| Odom level | Input ATE | Input RPE 1 s | Input yaw RMSE |",
        "|---|---:|---:|---:|",
    ]
    for level in LEVELS:
        row = next(item for item in records if item["odometry_level"] == level)
        lines.append(
            f"| {level} "
            "| "
            f"{100 * float(row['input_odom_ate_translation_rmse_m']):.2f} cm "
            "| "
            f"{100 * float(row['input_odom_rpe_1s_translation_rmse_m']):.2f} cm "
            f"| {float(row['input_odom_ate_yaw_rmse_deg']):.2f} deg |"
        )
    lines.extend(
        [
            "",
            "## SLAM output versus ground truth",
            "",
            (
                "| Odom level | Algorithm | ATE | RPE 1 s "
                "| Yaw RMSE | Final error |"
            ),
            "|---|---|---:|---:|---:|---:|",
        ]
    )
    for row in records:
        lines.append(
            f"| {row['odometry_level']} | {row['algorithm']} "
            f"| {100 * float(row['ate_translation_rmse_m']):.2f} cm "
            f"| {100 * float(row['rpe_1s_translation_rmse_m']):.2f} cm "
            f"| {float(row['ate_yaw_rmse_deg']):.2f} deg "
            f"| {100 * float(row['final_translation_error_m']):.2f} cm |"
        )
    lines.extend(
        [
            "",
            "Each matrix cell contains one deterministic trial. Differences "
            "are descriptive and are not confidence intervals.",
            "",
        ]
    )
    (output / "SUMMARY.md").write_text("\n".join(lines), encoding="utf-8")


def main() -> None:
    """Parse arguments and analyze the odometry-noise matrix."""
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--results-root",
        type=Path,
        default=Path("ros2_ws/slam_results"),
    )
    parser.add_argument(
        "--experiment-name", default="odometry_noise"
    )
    args = parser.parse_args()
    records = analyze(args.results_root, args.experiment_name)
    output = args.results_root / args.experiment_name / "comparison"
    write_outputs(records, output)
    print(json.dumps(records, indent=2))


if __name__ == "__main__":
    main()
