#!/usr/bin/env python3
"""Offline replay of LiDAR scan matches in motion-response summaries."""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Any

import numpy as np

from evaluation_results import match_metadata, write_data_json
from lidar_scan_matcher import LidarScanMatcher
from planar_extrinsic import PlanarExtrinsic

ROOT = Path(__file__).resolve().parents[2]


def _compose(a: tuple[float, float, float], b: tuple[float, float, float]) -> tuple[float, float, float]:
    x, y, yaw = a
    bx, by, byaw = b
    c, s = math.cos(yaw), math.sin(yaw)
    return x + c * bx - s * by, y + s * bx + c * by, yaw + byaw


def _pose(data: dict) -> tuple[float, float, float]:
    return float(data["dx_m"]), float(data["dy_m"]), math.radians(float(data["dyaw_deg"]))


def _angle(value: float) -> float:
    return (value + math.pi) % (2 * math.pi) - math.pi


def replay_summary(
    path: Path, *, min_inlier_ratio: float | None = None,
    max_iterations: int = 50, convergence_config: dict | None = None,
) -> list[dict]:
    summary = json.loads(path.read_text(encoding="utf-8"))
    records = summary["records"]
    rows: list[dict] = []
    for record in records:
        result = record.get("result")
        if result is None:
            continue
        scan_path = Path(result["raw_scan_file"])
        if not scan_path.is_absolute():
            scan_path = ROOT / scan_path
        with np.load(scan_path) as scans:
            start, destination, returned = (np.asarray(scans[k]) for k in ("start", "destination", "returned"))
        extrinsic = PlanarExtrinsic(**result["extrinsic_calibration_applied"])
        settings = result.get("evaluation_settings", {})
        ratio = min_inlier_ratio if min_inlier_ratio is not None else settings.get("icp_min_inlier_ratio", .40)
        config = convergence_config if convergence_config is not None else settings.get("icp_convergence")
        matcher = LidarScanMatcher(
            voxel_size_m=.015, max_iterations=max_iterations, min_inlier_ratio=ratio,
            min_range_m=settings.get("lidar_min_range_m", .38),
            convergence_config=config,
        )
        odom_out, odom_in = _pose(result["outbound"]["odom_reported"]), _pose(result["inbound"]["odom_reported"])
        odom_loop = _compose(odom_out, odom_in)
        cases = (
            ("outbound", start, destination, odom_out, result["outbound"]),
            ("inbound", destination, returned, odom_in, result["inbound"]),
            ("loop", start, returned, odom_loop, None),
        )
        for leg, ref, target, odom, old in cases:
            guess = extrinsic.base_to_lidar(*odom)
            match = matcher.match(ref, target, initial_guess=guess)
            before = old.get("raw_lidar_icp") if old else result.get("raw_loop_icp")
            if old:
                before_valid = bool(old.get("is_valid"))
            else:
                before_valid = bool(
                    before.get("is_valid") if before and "is_valid" in before
                    else result["outbound"].get("is_valid") and result["inbound"].get("is_valid")
                )
            dx = match.delta_x_m - float(before.get("delta_x_m", 0)) if before else None
            dy = match.delta_y_m - float(before.get("delta_y_m", 0)) if before else None
            dyaw = _angle(match.delta_yaw_rad - float(before.get("delta_yaw_rad", 0))) if before else None
            rows.append({
                "source_summary": str(path), "trial": record.get("axis"),
                "speed": record.get("speed"), "kp": record.get("kp"),
                "source_scans": str(scan_path), "initial_guess_lidar": guess,
                "applied_min_inlier_ratio": ratio,
                "applied_voxel_size_m": .015, "applied_max_iterations": max_iterations,
                "repetition": record.get("repetition"), "leg": leg,
                "before": before, "after": match_metadata(match),
                "previously_valid": before_valid, "newly_valid": match.is_valid,
                "translation_change_m": math.hypot(dx, dy) if dx is not None else None,
                "yaw_change_rad": dyaw,
            })
    return rows


def build_report(rows: list[dict]) -> dict[str, Any]:
    previously = [r for r in rows if r["previously_valid"]]
    counts = {
        "previously_valid": len(previously),
        "newly_valid": sum(not r["previously_valid"] and r["newly_valid"] for r in rows),
        "still_invalid": sum(not r["previously_valid"] and not r["newly_valid"] for r in rows),
        "regressed": sum(r["previously_valid"] and not r["newly_valid"] for r in rows),
        "improved": sum(not r["previously_valid"] and r["newly_valid"] for r in rows),
    }
    return {
        "counts": counts,
        "max_translation_change_m": max((r["translation_change_m"] or 0 for r in previously), default=0),
        "max_yaw_change_rad": max((abs(r["yaw_change_rad"] or 0) for r in previously), default=0),
        "rows": rows,
    }


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("summaries", nargs="+", type=Path)
    parser.add_argument("--min-inlier-ratio", type=float)
    parser.add_argument("--max-iterations", type=int, default=50)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args(argv)
    rows = [row for path in args.summaries for row in replay_summary(
        path, min_inlier_ratio=args.min_inlier_ratio, max_iterations=args.max_iterations)]
    report = build_report(rows)
    write_data_json(args.output, report)
    print(json.dumps(report["counts"], sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
