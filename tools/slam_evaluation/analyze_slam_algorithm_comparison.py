#!/usr/bin/env python3
"""Compute trajectory and map metrics for the offline SLAM comparison."""

from __future__ import annotations

import argparse
import csv
import json
import math
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Iterable

import numpy as np
import rosbag2_py
import yaml
from PIL import Image
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


@dataclass(frozen=True)
class Pose:
    stamp: float
    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class RunMetrics:
    algorithm: str
    height_cm: float
    ate_translation_rmse_m: float
    ate_yaw_rmse_deg: float
    rpe_1s_translation_rmse_m: float
    rpe_1s_yaw_rmse_deg: float
    final_translation_error_m: float
    final_yaw_error_deg: float
    trajectory_samples: int
    input_scans: int
    finite_scan_returns_percent: float
    occupied_cells: int
    known_cells: int
    known_area_m2: float
    map_width_cells: int
    map_height_cells: int
    map_resolution_m: float


def wrap(angle: float | np.ndarray) -> float | np.ndarray:
    return (angle + np.pi) % (2.0 * np.pi) - np.pi


def stamp_seconds(stamp: object) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def yaw_of(q: object) -> float:
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def pose_from_transform(transform: object) -> tuple[float, float, float]:
    return (
        float(transform.translation.x),
        float(transform.translation.y),
        yaw_of(transform.rotation),
    )


def compose(a: tuple[float, float, float], b: tuple[float, float, float]) -> tuple[float, float, float]:
    ca, sa = math.cos(a[2]), math.sin(a[2])
    return (
        a[0] + ca * b[0] - sa * b[1],
        a[1] + sa * b[0] + ca * b[1],
        float(wrap(a[2] + b[2])),
    )


def open_reader(path: Path) -> tuple[rosbag2_py.SequentialReader, dict[str, type]]:
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(path), storage_id="mcap"),
        rosbag2_py.ConverterOptions("cdr", "cdr"),
    )
    types = {
        topic.name: get_message(topic.type)
        for topic in reader.get_all_topics_and_types()
    }
    return reader, types


def read_input(path: Path) -> tuple[list[Pose], int, int, int]:
    reader, types = open_reader(path)
    truth: list[Pose] = []
    scan_count = finite_count = beam_count = 0
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic == "/ground_truth/odom":
            msg = deserialize_message(data, types[topic])
            truth.append(
                Pose(
                    stamp_seconds(msg.header.stamp),
                    float(msg.pose.pose.position.x),
                    float(msg.pose.pose.position.y),
                    yaw_of(msg.pose.pose.orientation),
                )
            )
        elif topic == "/scan":
            msg = deserialize_message(data, types[topic])
            ranges = np.asarray(msg.ranges)
            scan_count += 1
            finite_count += int(np.isfinite(ranges).sum())
            beam_count += int(ranges.size)
    return deduplicate(truth), scan_count, finite_count, beam_count


def read_estimate(path: Path) -> list[Pose]:
    reader, types = open_reader(path)
    direct: list[Pose] = []
    composed: list[Pose] = []
    latest_odom_base: tuple[float, float, float] | None = None
    latest_map_odom: tuple[float, float, float] | None = None
    latest_stamp = 0.0
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic != "/tf":
            continue
        msg = deserialize_message(data, types[topic])
        changed = False
        for item in msg.transforms:
            parent = item.header.frame_id.lstrip("/")
            child = item.child_frame_id.lstrip("/")
            stamp = stamp_seconds(item.header.stamp)
            if parent == "map" and child in {"base_link", "base_footprint"}:
                x, y, yaw = pose_from_transform(item.transform)
                direct.append(Pose(stamp, x, y, yaw))
            elif parent == "map" and child == "odom":
                latest_map_odom = pose_from_transform(item.transform)
                latest_stamp = stamp
                changed = True
            elif parent == "odom" and child in {"base_link", "base_footprint"}:
                latest_odom_base = pose_from_transform(item.transform)
                latest_stamp = stamp
                changed = True
        if changed and latest_map_odom is not None and latest_odom_base is not None:
            x, y, yaw = compose(latest_map_odom, latest_odom_base)
            composed.append(Pose(latest_stamp, x, y, yaw))
    return deduplicate(direct if direct else composed)


def deduplicate(poses: Iterable[Pose]) -> list[Pose]:
    by_stamp = {round(p.stamp, 9): p for p in poses if p.stamp > 0.0}
    return sorted(by_stamp.values(), key=lambda pose: pose.stamp)


def interpolate(poses: list[Pose], stamps: np.ndarray) -> np.ndarray:
    source_t = np.asarray([pose.stamp for pose in poses])
    x = np.interp(stamps, source_t, [pose.x for pose in poses])
    y = np.interp(stamps, source_t, [pose.y for pose in poses])
    yaw_unwrapped = np.unwrap([pose.yaw for pose in poses])
    yaw = wrap(np.interp(stamps, source_t, yaw_unwrapped))
    return np.column_stack((x, y, yaw))


def align_estimate(estimate: np.ndarray, truth: np.ndarray) -> np.ndarray:
    estimate_xy = estimate[:, :2]
    truth_xy = truth[:, :2]
    estimate_center = estimate_xy.mean(axis=0)
    truth_center = truth_xy.mean(axis=0)
    covariance = (estimate_xy - estimate_center).T @ (truth_xy - truth_center)
    u, _, vt = np.linalg.svd(covariance)
    rotation = vt.T @ u.T
    if np.linalg.det(rotation) < 0.0:
        vt[-1, :] *= -1.0
        rotation = vt.T @ u.T
    translation = truth_center - rotation @ estimate_center
    result = estimate.copy()
    result[:, :2] = (rotation @ estimate_xy.T).T + translation
    rotation_yaw = math.atan2(rotation[1, 0], rotation[0, 0])
    result[:, 2] = wrap(result[:, 2] + rotation_yaw)
    return result


def relative_delta(poses: np.ndarray, first: int, second: int) -> np.ndarray:
    dx = poses[second, 0] - poses[first, 0]
    dy = poses[second, 1] - poses[first, 1]
    c, s = math.cos(poses[first, 2]), math.sin(poses[first, 2])
    return np.asarray((c * dx + s * dy, -s * dx + c * dy, wrap(poses[second, 2] - poses[first, 2])))


def trajectory_metrics(estimate: list[Pose], truth: list[Pose]) -> tuple[float, ...]:
    if len(estimate) < 2 or len(truth) < 2:
        raise RuntimeError("not enough trajectory poses")
    start = max(estimate[0].stamp, truth[0].stamp)
    # Exclude the one-second shutdown tail, where transient-local map/TF samples
    # from a just-stopped run can share the bag's final simulated timestamp.
    end = min(estimate[-1].stamp, truth[-1].stamp - 1.0)
    selected = [pose for pose in estimate if start <= pose.stamp <= end]
    # Cap the evaluation at 20 Hz so high-frequency TF publishers get no extra weight.
    selected = selected[:: max(1, round(len(selected) / max(1.0, end - start) / 20.0))]
    stamps = np.asarray([pose.stamp for pose in selected])
    estimate_array = np.asarray([[pose.x, pose.y, pose.yaw] for pose in selected])
    truth_array = interpolate(truth, stamps)
    estimate_array = align_estimate(estimate_array, truth_array)

    xy_error = np.linalg.norm(estimate_array[:, :2] - truth_array[:, :2], axis=1)
    yaw_offset = math.atan2(
        float(np.sin(truth_array[:, 2] - estimate_array[:, 2]).sum()),
        float(np.cos(truth_array[:, 2] - estimate_array[:, 2]).sum()),
    )
    estimate_array[:, 2] = wrap(estimate_array[:, 2] + yaw_offset)
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
        rpe_yaw.append(float(wrap(estimate_delta[2] - truth_delta[2])))

    return (
        float(np.sqrt(np.mean(xy_error**2))),
        math.degrees(float(np.sqrt(np.mean(yaw_error**2)))),
        float(np.sqrt(np.mean(np.square(rpe_xy)))),
        math.degrees(float(np.sqrt(np.mean(np.square(rpe_yaw))))),
        float(xy_error[-1]),
        abs(math.degrees(float(yaw_error[-1]))),
        len(selected),
    )


def map_metrics(run_path: Path) -> tuple[int, int, float, int, int, float]:
    with (run_path / "map_final.yaml").open(encoding="utf-8") as stream:
        metadata = yaml.safe_load(stream)
    pixels = np.asarray(Image.open(run_path / "map_final.pgm"))
    occupied = int((pixels <= 100).sum())
    known = int((pixels != 205).sum())
    resolution = float(metadata["resolution"])
    return (
        occupied,
        known,
        known * resolution**2,
        int(pixels.shape[1]),
        int(pixels.shape[0]),
        resolution,
    )


def extract_rtabmap_final_map(run_path: Path, end_stamp: float) -> None:
    """Save the latest live RTAB-Map grid before the shutdown transient tail."""
    reader, types = open_reader(run_path / "result_bag")
    selected: tuple[float, object] | None = None
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic != "/map":
            continue
        message = deserialize_message(data, types[topic])
        stamp = stamp_seconds(message.header.stamp)
        if stamp < end_stamp - 1.0 and (
            selected is None or stamp > selected[0]
        ):
            selected = (stamp, message)
    if selected is None:
        raise RuntimeError(f"no live RTAB-Map grid found in {run_path}")
    stamp, message = selected
    grid = np.asarray(message.data, dtype=np.int16).reshape(
        int(message.info.height), int(message.info.width)
    )
    pgm = np.full(grid.shape, 205, dtype=np.uint8)
    pgm[grid <= 25] = 254
    pgm[grid >= 65] = 0
    pgm = np.flipud(pgm)
    Image.fromarray(pgm).save(run_path / "map_final.pgm")
    Image.fromarray(pgm).save(run_path / "map_final.png")
    origin = message.info.origin
    metadata = {
        "image": "map_final.pgm",
        "mode": "trinary",
        "resolution": float(message.info.resolution),
        "origin": [
            float(origin.position.x),
            float(origin.position.y),
            yaw_of(origin.orientation),
        ],
        "negate": 0,
        "occupied_thresh": 0.65,
        "free_thresh": 0.196,
        "source_stamp": stamp,
    }
    with (run_path / "map_final.yaml").open("w", encoding="utf-8") as stream:
        yaml.safe_dump(metadata, stream, sort_keys=False)


def analyze(root: Path) -> list[RunMetrics]:
    input_cache: dict[float, tuple[list[Pose], int, int, int]] = {}
    metrics: list[RunMetrics] = []
    heights = ((16.5, "16p5"), (26.0, "26"), (45.0, "45"), (70.0, "70"))
    for algorithm in ("slam_toolbox", "cartographer", "cartographer_imu", "rtabmap"):
        for height, token in heights:
            input_path = root / "algorithm_compare_inputs" / f"input_{token}cm_trial1"
            run_path = root / "algorithm_compare_runs" / algorithm / f"{token}cm"
            if not (run_path / "run_complete").exists():
                raise RuntimeError(f"incomplete run: {run_path}")
            if height not in input_cache:
                input_cache[height] = read_input(input_path)
            truth, scans, finite, beams = input_cache[height]
            if algorithm == "rtabmap":
                extract_rtabmap_final_map(run_path, truth[-1].stamp)
            estimate = read_estimate(run_path / "result_bag")
            trajectory = trajectory_metrics(estimate, truth)
            occupied, known, known_area, width, map_height, resolution = map_metrics(run_path)
            metrics.append(
                RunMetrics(
                    algorithm,
                    height,
                    *trajectory,
                    scans,
                    100.0 * finite / beams,
                    occupied,
                    known,
                    known_area,
                    width,
                    map_height,
                    resolution,
                )
            )
    return metrics


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--results-root",
        type=Path,
        default=Path("ros2_ws/slam_results"),
    )
    parser.add_argument("--extract-rtabmap-run", type=Path)
    parser.add_argument("--input-bag", type=Path)
    args = parser.parse_args()
    if args.extract_rtabmap_run is not None:
        if args.input_bag is None:
            parser.error("--input-bag is required with --extract-rtabmap-run")
        truth, _, _, _ = read_input(args.input_bag)
        extract_rtabmap_final_map(args.extract_rtabmap_run, truth[-1].stamp)
        return
    output = args.results_root / "algorithm_comparison"
    output.mkdir(parents=True, exist_ok=True)
    metrics = analyze(args.results_root)
    records = [asdict(item) for item in metrics]
    with (output / "metrics.json").open("w", encoding="utf-8") as stream:
        json.dump(records, stream, indent=2)
        stream.write("\n")
    with (output / "metrics.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(records[0]))
        writer.writeheader()
        writer.writerows(records)
    by_algorithm = {
        algorithm: [row for row in records if row["algorithm"] == algorithm]
        for algorithm in ("slam_toolbox", "cartographer", "cartographer_imu", "rtabmap")
    }
    ranking = sorted(
        by_algorithm,
        key=lambda algorithm: sum(
            row["ate_translation_rmse_m"] for row in by_algorithm[algorithm]
        ),
    )
    lines = [
        "# Offline SLAM comparison summary",
        "",
        "All runs use the same height-specific sensor bags and 2.5x replay. "
        "ATE uses scale-fixed SE(2) alignment; RPE uses a 1 s interval.",
        "",
        "| Algorithm | Mean ATE | Mean 1 s RPE | Mean yaw RMSE | Mean known area |",
        "|---|---:|---:|---:|---:|",
    ]
    for algorithm in ranking:
        rows = by_algorithm[algorithm]
        lines.append(
            f"| {algorithm} | {100 * np.mean([row['ate_translation_rmse_m'] for row in rows]):.2f} cm "
            f"| {100 * np.mean([row['rpe_1s_translation_rmse_m'] for row in rows]):.2f} cm "
            f"| {np.mean([row['ate_yaw_rmse_deg'] for row in rows]):.2f} deg "
            f"| {np.mean([row['known_area_m2'] for row in rows]):.1f} m2 |"
        )
    lines.extend(
        [
            "",
            "Each run directory contains map_final.pgm/png/yaml, result_bag, logs, "
            "gazebo_overlay.png, and the algorithm-native graph artifact: "
            "slam_toolbox posegraph/data, Cartographer pbstream, or RTAB-Map db.",
            "",
        ]
    )
    (output / "SUMMARY.md").write_text("\n".join(lines), encoding="utf-8")
    print(json.dumps(records, indent=2))


if __name__ == "__main__":
    main()
