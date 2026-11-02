#!/usr/bin/env python3
"""Automated Multi-Speed Holonomic Odometry Evaluation with 2D LiDAR Ground Truth."""

from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
from typing import Any
import numpy as np

_active_subproc: subprocess.Popen | None = None


def _handle_signal(sig, frame):
    global _active_subproc
    print(f"\n[SIGNAL {sig}] Interrupting multi-speed evaluation...", file=sys.stderr)
    if _active_subproc is not None and _active_subproc.poll() is None:
        try:
            _active_subproc.terminate()
            _active_subproc.wait(timeout=2.0)
        except Exception:
            try:
                _active_subproc.kill()
            except Exception:
                pass
    sys.exit(130)


signal.signal(signal.SIGINT, _handle_signal)
signal.signal(signal.SIGTERM, _handle_signal)


def run_single_eval(
    pattern: str,
    distance: float,
    speed: float,
    output_path: Path,
    dry_run: bool = False,
) -> dict[str, Any] | None:
    """Run a single odometry evaluation pass via subprocess."""
    global _active_subproc
    script_dir = Path(__file__).resolve().parent
    cmd = [
        sys.executable,
        str(script_dir / "run_odometry_eval.py"),
        "--pattern", pattern,
        "--distance", str(distance),
        "--speed", str(speed),
        "--output", str(output_path),
    ]
    if dry_run:
        cmd.append("--dry-run")

    print(f"\n{'='*70}")
    print(f">> Executing: {' '.join(cmd)}")
    print(f"{'='*70}")

    proc = subprocess.Popen(cmd)
    _active_subproc = proc
    try:
        retcode = proc.wait()
    finally:
        _active_subproc = None

    if retcode != 0:
        print(f"Error: Evaluation run failed with code {retcode}", file=sys.stderr)
        return None

    if dry_run:
        return {"dry_run": True}

    if not output_path.exists():
        print(f"Error: Output file {output_path} was not created", file=sys.stderr)
        return None

    return json.loads(output_path.read_text(encoding="utf-8"))


def generate_multi_speed_plots(
    results_by_speed: dict[float, list[dict[str, Any]]],
    save_path: Path,
    title: str = "Multi-Speed Box Trajectory: Odometry vs LiDAR Ground Truth",
) -> None:
    """Generate comprehensive multi-speed comparative visualization."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    speeds = sorted(results_by_speed.keys())
    if not speeds:
        return

    fig, axes = plt.subplots(2, 2, figsize=(15, 12))
    ax_drift, ax_compare = axes[0, 0], axes[0, 1]
    ax_quality, ax_scatter = axes[1, 0], axes[1, 1]

    # 1. Speed vs Physical Ground-Truth Drift
    means_x = [np.mean([r["physical_ground_truth"]["error_x_mm"] for r in results_by_speed[s]]) for s in speeds]
    stds_x = [np.std([r["physical_ground_truth"]["error_x_mm"] for r in results_by_speed[s]]) for s in speeds]

    means_y = [np.mean([r["physical_ground_truth"]["error_y_mm"] for r in results_by_speed[s]]) for s in speeds]
    stds_y = [np.std([r["physical_ground_truth"]["error_y_mm"] for r in results_by_speed[s]]) for s in speeds]

    means_norm = [np.mean([r["physical_ground_truth"]["position_norm_mm"] for r in results_by_speed[s]]) for s in speeds]
    stds_norm = [np.std([r["physical_ground_truth"]["position_norm_mm"] for r in results_by_speed[s]]) for s in speeds]

    ax_drift.errorbar(speeds, means_x, yerr=stds_x, fmt="-o", capsize=5, label="Longitudinal Error (X)", color="#1f77b4", linewidth=2)
    ax_drift.errorbar(speeds, means_y, yerr=stds_y, fmt="-s", capsize=5, label="Lateral Error (Y)", color="#ff7f0e", linewidth=2)
    ax_drift.errorbar(speeds, means_norm, yerr=stds_norm, fmt="-^", capsize=5, label="2D Position Error Norm", color="#d62728", linewidth=2.5)
    ax_drift.set_xlabel("Linear Speed (m/s)")
    ax_drift.set_ylabel("Physical Return Error (mm)")
    ax_drift.set_title("Ground-Truth Return Error vs Speed (LiDAR ICP)")
    ax_drift.grid(True, linestyle="--", alpha=0.5)
    ax_drift.legend(loc="upper left")

    # 2. Odom Residual vs LiDAR Ground Truth (Slip Discrepancy)
    odom_norms = [np.mean([math.hypot(r["odom_residual"]["delta_x_m"] * 1000.0, r["odom_residual"]["delta_y_m"] * 1000.0) for r in results_by_speed[s]]) for s in speeds]
    width = 0.04
    x_indices = np.array(speeds)
    ax_compare.bar(x_indices - width/2, odom_norms, width=width, label="Wheel Odom Residual (ROS)", color="#2ca02c", alpha=0.8)
    ax_compare.bar(x_indices + width/2, means_norm, width=width, label="LiDAR Ground Truth Error", color="#d62728", alpha=0.8)
    ax_compare.set_xlabel("Linear Speed (m/s)")
    ax_compare.set_ylabel("Error Norm (mm)")
    ax_compare.set_title("Wheel Odometry vs Physical Ground Truth")
    ax_compare.set_xticks(speeds)
    ax_compare.grid(True, linestyle="--", alpha=0.5)
    ax_compare.legend(loc="upper left")

    # 3. LiDAR ICP Registration Quality & Mount Stability
    rmses = []
    inliers = []
    for s in speeds:
        s_rmses = [r["lidar_metrics"]["rmse_mm"] for r in results_by_speed[s] if "lidar_metrics" in r and r["lidar_metrics"]]
        s_inliers = [r["lidar_metrics"]["inlier_ratio_pct"] for r in results_by_speed[s] if "lidar_metrics" in r and r["lidar_metrics"]]
        rmses.append(np.mean(s_rmses) if s_rmses else 0.0)
        inliers.append(np.mean(s_inliers) if s_inliers else 0.0)

    ax_quality_twin = ax_quality.twinx()
    p1 = ax_quality.plot(speeds, rmses, "-o", color="#9467bd", label="ICP Residual RMSE (mm)", linewidth=2)
    p2 = ax_quality_twin.plot(speeds, inliers, "--s", color="#8c564b", label="Inlier Ratio (%)", linewidth=2)
    ax_quality.set_xlabel("Linear Speed (m/s)")
    ax_quality.set_ylabel("ICP RMSE (mm)", color="#9467bd")
    ax_quality_twin.set_ylabel("Inlier Ratio (%)", color="#8c564b")
    ax_quality.set_title("LiDAR Scan Match Quality & Mount Stability")
    ax_quality.grid(True, linestyle="--", alpha=0.5)
    plots = p1 + p2
    labels = [p.get_label() for p in plots]
    ax_quality.legend(plots, labels, loc="upper right")

    # 4. 2D Spatial Scatter of Final Return Positions
    colors = {speeds[0]: "#1f77b4", speeds[1]: "#ff7f0e", speeds[2]: "#d62728"} if len(speeds) == 3 else {}
    for s in speeds:
        c = colors.get(s, "#333333")
        xs = [r["physical_ground_truth"]["error_x_mm"] for r in results_by_speed[s]]
        ys = [r["physical_ground_truth"]["error_y_mm"] for r in results_by_speed[s]]
        ax_scatter.scatter(xs, ys, s=80, color=c, alpha=0.9, label=f"{s:.2f} m/s")
        for i, (x, y) in enumerate(zip(xs, ys)):
            ax_scatter.annotate(f"{s:.2f}m/s #{i+1}", (x + 1.0, y + 1.0), fontsize=9)

    ax_scatter.plot(0, 0, "k+", markersize=14, markeredgewidth=2.5, label="Start Origin (0,0)")
    ax_scatter.axhline(0, color="gray", linestyle=":", alpha=0.6)
    ax_scatter.axvline(0, color="gray", linestyle=":", alpha=0.6)
    ax_scatter.set_aspect("equal", "datalim")
    ax_scatter.set_xlabel("Longitudinal Return Error X (mm)")
    ax_scatter.set_ylabel("Lateral Return Error Y (mm)")
    ax_scatter.set_title("2D Spatial Return Distribution on Floor")
    ax_scatter.grid(True, linestyle="--", alpha=0.5)
    ax_scatter.legend(loc="lower left")

    plt.suptitle(title, fontsize=16, y=0.98)
    plt.tight_layout(rect=[0, 0, 1, 0.96])
    plt.savefig(save_path, dpi=150)
    plt.close(fig)
    print(f"\n[Multi-Speed Plot] Saved comparison figure to: {save_path}")


def main() -> int:
    parser = argparse.ArgumentParser(description="Multi-Speed Holonomic Odometry Evaluation")
    parser.add_argument("--speeds", type=float, nargs="+", default=[0.20, 0.35, 0.50],
                        help="List of speeds to evaluate in m/s (default: 0.20 0.35 0.50)")
    parser.add_argument("--runs-per-speed", type=int, default=2,
                        help="Number of repetitions per speed (default: 2)")
    parser.add_argument("--distance", type=float, default=0.5,
                        help="Box pattern side length in meters (default: 0.5m)")
    parser.add_argument("--pattern", type=str, default="box",
                        help="Motion pattern (default: box)")
    parser.add_argument("--dwell-between", type=float, default=3.0,
                        help="Dwell time between runs in seconds (default: 3.0s)")
    parser.add_argument("--dry-run", action="store_true",
                        help="Run dry-run verification without robot motion")
    args = parser.parse_args()

    timestamp = time.strftime("%Y%m%d_%H%M%S")
    out_dir = Path("artifacts/odometry_evaluation")
    out_dir.mkdir(parents=True, exist_ok=True)

    print(f"=== Starting Multi-Speed Batch Evaluation ===")
    print(f"Pattern: {args.pattern.upper()}, Distance: {args.distance}m (Total Loop: {args.distance*4:.1f}m)")
    print(f"Speeds to test: {args.speeds} m/s")
    print(f"Runs per speed: {args.runs_per_speed} (Total runs: {len(args.speeds) * args.runs_per_speed})")
    print(f"Dry-run: {args.dry_run}")

    all_results: list[dict[str, Any]] = []
    results_by_speed: dict[float, list[dict[str, Any]]] = {s: [] for s in args.speeds}

    run_count = 0
    total_runs = len(args.speeds) * args.runs_per_speed

    for spd in args.speeds:
        for rep in range(1, args.runs_per_speed + 1):
            run_count += 1
            print(f"\n[{run_count}/{total_runs}] Running Speed {spd:.2f} m/s (Repetition #{rep})...")
            run_json = out_dir / f"batch_eval_{args.pattern}_v{int(spd*100):02d}_run{rep}_{timestamp}.json"

            res = run_single_eval(
                pattern=args.pattern,
                distance=args.distance,
                speed=spd,
                output_path=run_json,
                dry_run=args.dry_run,
            )

            if res is None:
                print(f"Warning: Run at {spd} m/s rep #{rep} failed. Aborting remaining batch.", file=sys.stderr)
                return 1

            if not args.dry_run:
                res["batch_meta"] = {
                    "speed_mps": spd,
                    "repetition": rep,
                    "timestamp": timestamp,
                }
                all_results.append(res)
                results_by_speed[spd].append(res)

                if run_count < total_runs:
                    print(f"Pausing {args.dwell_between}s for robot and sensor to settle completely...")
                    time.sleep(args.dwell_between)

    if args.dry_run:
        print("\nDry-run completed successfully.")
        return 0

    # Generate multi-speed comparative plot
    plot_path = out_dir / f"multi_speed_{args.pattern}_comparison_{timestamp}.png"
    generate_multi_speed_plots(results_by_speed, plot_path)

    # Save aggregated batch results
    batch_json_path = out_dir / f"multi_speed_{args.pattern}_batch_{timestamp}.json"
    batch_summary = {
        "timestamp": timestamp,
        "pattern": args.pattern,
        "nominal_distance_m": args.distance * 4.0,
        "speeds_evaluated": args.speeds,
        "runs_per_speed": args.runs_per_speed,
        "plot_path": str(plot_path),
        "results_by_speed": {
            str(s): results_by_speed[s] for s in args.speeds
        },
    }
    batch_json_path.write_text(json.dumps(batch_summary, indent=2), encoding="utf-8")
    print(f"\n[Batch Summary JSON] Saved to: {batch_json_path}")

    # Print summary comparative table
    print("\n" + "=" * 80)
    print("=== Multi-Speed Odometry vs LiDAR Ground Truth Comparison Summary ===")
    print("=" * 80)
    print(f"{'Speed':^8} | {'Rep':^5} | {'Odom dX (mm)':^13} | {'Odom dY (mm)':^13} | {'LiDAR X (mm)':^13} | {'LiDAR Y (mm)':^13} | {'LiDAR Norm':^11} | {'RMSE':^8} | {'Inliers':^8}")
    print("-" * 80)

    for spd in args.speeds:
        for idx, r in enumerate(results_by_speed[spd], 1):
            odom_dx = r["odom_residual"]["delta_x_m"] * 1000.0
            odom_dy = r["odom_residual"]["delta_y_m"] * 1000.0
            lidar_x = r["physical_ground_truth"]["error_x_mm"]
            lidar_y = r["physical_ground_truth"]["error_y_mm"]
            norm_mm = r["physical_ground_truth"]["position_norm_mm"]
            lm = r.get("lidar_metrics", {})
            rmse = lm.get("rmse_mm", 0.0)
            inliers = lm.get("inlier_ratio_pct", 0.0)
            print(f"{spd:^8.2f} | {idx:^5} | {odom_dx:^+13.1f} | {odom_dy:^+13.1f} | {lidar_x:^+13.1f} | {lidar_y:^+13.1f} | {norm_mm:^11.1f} | {rmse:^8.1f} | {inliers:^7.1f}%")

    print("=" * 80)
    return 0


if __name__ == "__main__":
    sys.exit(main())
