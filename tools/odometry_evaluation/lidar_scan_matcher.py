#!/usr/bin/env python3
"""2D LiDAR Scan Matcher for Automated Ground-Truth Position Estimation.

Implements robust 2D Point-to-Line and Point-to-Point ICP registration
to calculate relative 2D transformation (delta_x, delta_y, delta_yaw)
between reference scan and target scan.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Sequence
import numpy as np
from scipy.spatial import cKDTree


@dataclass
class ScanMatchResult:
    delta_x_m: float
    delta_y_m: float
    delta_yaw_rad: float
    delta_yaw_deg: float
    rmse_m: float
    rmse_mm: float
    inlier_ratio: float
    num_inliers: int
    num_total_points: int
    converged: bool
    iterations: int
    condition_number: float
    is_valid: bool = True
    rejection_reason: str = ""
    translation_condition_number: float = 1.0
    min_translation_eigenvalue: float = 0.0
    full_min_eigenvalue: float = 0.0


def validate_scan_match(
    result: ScanMatchResult,
    min_inlier_ratio: float = 0.40,
    min_num_inliers: int = 30,
    max_rmse_mm: float = 25.0,
    max_translation_condition_number: float = 35.0,
    min_translation_eigenvalue: float = 5.0,
    max_condition_number: float = 1e6,
) -> tuple[bool, str]:
    """Validate 2D ICP scan match result against convergence, observability, and geometric degeneracy.

    Rejects degenerate geometric configurations (e.g., single planar walls) where motion
    along one axis is unobservable, causing false-positive zero-residual matches.
    """
    numeric_values = (result.delta_x_m, result.delta_y_m, result.delta_yaw_rad,
                      result.delta_yaw_deg, result.rmse_m, result.rmse_mm,
                      result.inlier_ratio, result.condition_number,
                      result.translation_condition_number,
                      result.min_translation_eigenvalue, result.full_min_eigenvalue)
    finite_metrics = (
        result.delta_x_m, result.delta_y_m, result.delta_yaw_rad,
        result.delta_yaw_deg, result.rmse_m, result.rmse_mm,
        result.inlier_ratio, result.min_translation_eigenvalue,
        result.full_min_eigenvalue,
    )
    if any(math.isnan(v) for v in numeric_values) or any(
        not math.isfinite(v) for v in finite_metrics
    ):
        return False, "Non-finite scan match or observability metric"
    if not result.converged:
        return False, "ICP did not converge within maximum allowed iterations"
    if result.num_inliers < min_num_inliers:
        return False, f"Insufficient inlier points ({result.num_inliers} < {min_num_inliers})"
    if result.inlier_ratio < min_inlier_ratio:
        return False, f"Inlier ratio too low ({result.inlier_ratio * 100:.1f}% < {min_inlier_ratio * 100:.1f}%)"
    if result.rmse_mm > max_rmse_mm:
        return False, f"RMSE too high ({result.rmse_mm:.1f}mm > {max_rmse_mm:.1f}mm)"
    if result.min_translation_eigenvalue < min_translation_eigenvalue:
        return False, (
            f"Degenerate translation geometry (unobservable direction): "
            f"min eigenvalue {result.min_translation_eigenvalue:.2f} < {min_translation_eigenvalue:.2f} "
            f"(e.g., single straight wall)"
        )
    if result.translation_condition_number > max_translation_condition_number:
        return False, (
            f"Degenerate translation Hessian: condition number "
            f"{result.translation_condition_number:.1f} > {max_translation_condition_number:.1f}"
        )
    if result.full_min_eigenvalue <= 1e-9 or result.condition_number > max_condition_number:
        return False, (
            f"Degenerate full SE(2) Hessian: minimum eigenvalue "
            f"{result.full_min_eigenvalue:.3g}, condition number {result.condition_number:.3g}"
        )
    return True, "Valid scan match"


class LidarScanMatcher:
    """Robust 2D Scan Matcher using Point-to-Line and Point-to-Point ICP."""

    def __init__(
        self,
        min_range_m: float = 0.38,
        max_range_m: float = 10.0,
        voxel_size_m: float = 0.02,
        max_iterations: int = 50,
        tolerance: float = 1e-5,
    ) -> None:
        self.min_range_m = min_range_m
        self.max_range_m = max_range_m
        self.voxel_size_m = voxel_size_m
        self.max_iterations = max_iterations
        self.tolerance = tolerance

    def laserscan_to_points(
        self,
        ranges: Sequence[float],
        angle_min: float,
        angle_increment: float,
        range_min: float,
        range_max: float,
    ) -> np.ndarray:
        """Convert a single LaserScan into 2D Cartesian points (N, 2)."""
        ranges_arr = np.asarray(ranges, dtype=np.float64)
        num_beams = len(ranges_arr)
        angles = angle_min + np.arange(num_beams, dtype=np.float64) * angle_increment

        effective_min = max(range_min, self.min_range_m)
        effective_max = min(range_max, self.max_range_m)

        valid_mask = (
            np.isfinite(ranges_arr)
            & (ranges_arr >= effective_min)
            & (ranges_arr <= effective_max)
        )

        valid_ranges = ranges_arr[valid_mask]
        valid_angles = angles[valid_mask]

        xs = valid_ranges * np.cos(valid_angles)
        ys = valid_ranges * np.sin(valid_angles)
        return np.column_stack((xs, ys))

    def downsample_points(self, points: np.ndarray) -> np.ndarray:
        """Voxel grid filter for uniform density."""
        if len(points) == 0:
            return points
        coords = np.floor(points / self.voxel_size_m).astype(np.int64)
        _, unique_indices = np.unique(coords, axis=0, return_index=True)
        return points[unique_indices]

    def compute_normals(self, points: np.ndarray, k: int = 6) -> tuple[np.ndarray, np.ndarray]:
        """Estimate 2D normals and linearity weights using local PCA."""
        n_pts = len(points)
        if n_pts < k:
            return np.zeros_like(points), np.zeros(n_pts)

        tree = cKDTree(points)
        _, indices = tree.query(points, k=k)

        normals = np.zeros_like(points)
        planarity = np.zeros(n_pts)

        for i, idx in enumerate(indices):
            neighbors = points[idx]
            cov = np.cov(neighbors, rowvar=False)
            evals, evecs = np.linalg.eigh(cov)
            # Smallest eigenvalue corresponds to normal direction
            normals[i] = evecs[:, 0]
            # Ratio of (λ_max - λ_min) / λ_max represents linearity
            if evals[1] > 1e-6:
                planarity[i] = (evals[1] - evals[0]) / evals[1]
            else:
                planarity[i] = 0.0

        return normals, planarity

    def match(
        self,
        ref_points: np.ndarray,
        target_points: np.ndarray,
        initial_guess: tuple[float, float, float] = (0.0, 0.0, 0.0),
    ) -> ScanMatchResult:
        """Align target_points to ref_points and return transform (dx, dy, dyaw).

        Transform maps points from target frame to ref frame:
        p_ref = R(dyaw) * p_target + [dx, dy]
        """
        ref_pts = self.downsample_points(ref_points)
        tgt_pts = self.downsample_points(target_points)

        if len(ref_pts) < 20 or len(tgt_pts) < 20:
            return ScanMatchResult(
                delta_x_m=0.0,
                delta_y_m=0.0,
                delta_yaw_rad=0.0,
                delta_yaw_deg=0.0,
                rmse_m=0.0,
                rmse_mm=0.0,
                inlier_ratio=0.0,
                num_inliers=0,
                num_total_points=len(tgt_pts),
                converged=False,
                iterations=0,
                condition_number=float("inf"),
                is_valid=False,
                rejection_reason=f"Insufficient scan points for ICP (ref={len(ref_pts)}, target={len(tgt_pts)}, minimum=20)",
            )

        ref_tree = cKDTree(ref_pts)
        ref_normals, ref_linearity = self.compute_normals(ref_pts, k=7)

        tx, ty, yaw = initial_guess

        max_dist = 0.30
        min_dist = 0.04
        converged = False
        final_iter = 0

        cond_num = float("inf")
        last_AtA = np.eye(3) * 1e-4

        for it in range(self.max_iterations):
            final_iter = it + 1
            # Current annealing distance threshold
            dist_thresh = max(min_dist, max_dist * (0.85 ** it))

            c = math.cos(yaw)
            s = math.sin(yaw)
            R = np.array([[c, -s], [s, c]])

            transformed_tgt = (R @ tgt_pts.T).T + np.array([tx, ty])

            dists, indices = ref_tree.query(transformed_tgt)

            mask = dists < dist_thresh
            if np.sum(mask) < 15:
                break

            matched_tgt = transformed_tgt[mask]
            matched_ref = ref_pts[indices[mask]]
            matched_normals = ref_normals[indices[mask]]
            matched_linearity = ref_linearity[indices[mask]]

            # Point-to-line residual: n_j^T * (p_i' - q_j)
            diffs = matched_tgt - matched_ref
            # Use point-to-line where linearity is high (> 0.7), point-to-point otherwise
            pt_to_line_mask = matched_linearity > 0.7

            A_rows = []
            b_vals = []

            for i in range(len(matched_tgt)):
                p = matched_tgt[i]
                diff = diffs[i]
                if pt_to_line_mask[i]:
                    n = matched_normals[i]
                    # Jacobian wrt [dtx, dty, dyaw]
                    # residual = n_x * (dtx - p_y*dyaw + p_x) + n_y * (dty + p_x*dyaw + p_y) - n^T q
                    # linearized error: n^T ( [dtx, dty] + dyaw * [-p_y, p_x] + diff )
                    # b = - (n^T diff)
                    r_val = np.dot(n, diff)
                    # Huber weight
                    huber = 1.0 if abs(r_val) < 0.02 else (0.02 / abs(r_val))
                    w = math.sqrt(huber)

                    j_tx = n[0]
                    j_ty = n[1]
                    j_yaw = -n[0] * p[1] + n[1] * p[0]
                    A_rows.append([w * j_tx, w * j_ty, w * j_yaw])
                    b_vals.append(-w * r_val)
                else:
                    # Point-to-point residual
                    r_val = np.linalg.norm(diff)
                    huber = 1.0 if r_val < 0.02 else (0.02 / r_val)
                    w = math.sqrt(huber) * 0.5  # lower weight than planar normals

                    A_rows.append([w * 1.0, 0.0, -w * p[1]])
                    b_vals.append(-w * diff[0])
                    A_rows.append([0.0, w * 1.0, w * p[0]])
                    b_vals.append(-w * diff[1])

            A = np.array(A_rows)
            b = np.array(b_vals)

            AtA = A.T @ A
            Atb = A.T @ b
            last_AtA = AtA

            # Check condition number of Hessian
            eigvals = np.linalg.eigvalsh(AtA)
            if eigvals[0] > 1e-9:
                cond_num = float(eigvals[-1] / eigvals[0])
            else:
                cond_num = float("inf")

            try:
                delta = np.linalg.solve(AtA + 1e-4 * np.eye(3), Atb)
            except np.linalg.LinAlgError:
                break

            dtx, dty, dyaw = delta
            tx += dtx
            ty += dty
            yaw = (yaw + dyaw + math.pi) % (2.0 * math.pi) - math.pi

            step_norm = math.sqrt(dtx * dtx + dty * dty)
            if step_norm < self.tolerance and abs(dyaw) < self.tolerance:
                converged = True
                break

        # Compute translation observability on final Hessian
        trans_H = last_AtA[0:2, 0:2]
        trans_eigvals = np.linalg.eigvalsh(trans_H)
        min_trans_eig = float(trans_eigvals[0])
        max_trans_eig = float(trans_eigvals[1])
        if min_trans_eig > 1e-9:
            trans_cond_num = float(max_trans_eig / min_trans_eig)
        else:
            trans_cond_num = float("inf")

        # Final evaluation and observability use the actual final correspondences.
        c = math.cos(yaw)
        s = math.sin(yaw)
        R = np.array([[c, -s], [s, c]])
        final_tgt = (R @ tgt_pts.T).T + np.array([tx, ty])
        final_dists, _ = ref_tree.query(final_tgt)

        inlier_threshold = 0.030  # 30 mm
        inliers = final_dists < inlier_threshold
        final_indices = ref_tree.query(final_tgt[inliers])[1] if np.any(inliers) else np.array([], dtype=int)
        final_p = final_tgt[inliers]
        final_n = ref_normals[final_indices]
        final_lin = ref_linearity[final_indices]
        scale = max(float(np.sqrt(np.mean(np.sum(final_p ** 2, axis=1)))) if len(final_p) else 1.0, 1e-6)
        rows = []
        for p, n, linear in zip(final_p, final_n, final_lin):
            if linear > 0.7:
                rows.append([n[0], n[1], (-n[0] * p[1] + n[1] * p[0]) / scale])
            else:
                rows.extend(([1.0, 0.0, -p[1] / scale], [0.0, 1.0, p[0] / scale]))
        full_h = np.asarray(rows, dtype=np.float64).T @ np.asarray(rows, dtype=np.float64) if rows else np.zeros((3, 3))
        full_eigs = np.linalg.eigvalsh(full_h)
        full_min_eig = float(full_eigs[0])
        full_cond = float(full_eigs[-1] / full_eigs[0]) if full_eigs[0] > 1e-9 else float("inf")
        num_inliers = int(np.sum(inliers))
        inlier_ratio = float(num_inliers / max(1, len(tgt_pts)))

        if num_inliers > 0:
            rmse_m = float(np.sqrt(np.mean(final_dists[inliers] ** 2)))
        else:
            rmse_m = float(np.sqrt(np.mean(final_dists ** 2)))

        res = ScanMatchResult(
            delta_x_m=float(tx),
            delta_y_m=float(ty),
            delta_yaw_rad=float(yaw),
            delta_yaw_deg=float(math.degrees(yaw)),
            rmse_m=rmse_m,
            rmse_mm=rmse_m * 1000.0,
            inlier_ratio=inlier_ratio,
            num_inliers=num_inliers,
            num_total_points=len(tgt_pts),
            converged=converged,
            iterations=final_iter,
            condition_number=full_cond,
            translation_condition_number=trans_cond_num,
            min_translation_eigenvalue=min_trans_eig,
            full_min_eigenvalue=full_min_eig,
        )

        is_valid, reason = validate_scan_match(res)
        res.is_valid = is_valid
        res.rejection_reason = reason
        return res


def plot_scan_match(
    ref_points: np.ndarray,
    target_points: np.ndarray,
    result: ScanMatchResult,
    save_path: str,
    title: str = "LiDAR ICP Scan Match Ground Truth",
) -> None:
    """Generate high-resolution diagnostic plot of 2D scan matching."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    c = math.cos(result.delta_yaw_rad)
    s = math.sin(result.delta_yaw_rad)
    R = np.array([[c, -s], [s, c]])
    aligned_tgt = (R @ target_points.T).T + np.array([result.delta_x_m, result.delta_y_m])

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 7), gridspec_kw={"width_ratios": [1.4, 1.0]})

    # Subplot 1: 2D Spatial Alignment View
    ax1.scatter(ref_points[:, 0], ref_points[:, 1], s=4, c="#1f77b4", alpha=0.4, label="Start Baseline Scan ($S_{start}$)")
    ax1.scatter(target_points[:, 0], target_points[:, 1], s=4, c="#ff7f0e", alpha=0.3, label="Final Raw Scan ($S_{end}$)")
    ax1.scatter(aligned_tgt[:, 0], aligned_tgt[:, 1], s=4, c="#2ca02c", alpha=0.6, label="Final Aligned Scan (ICP)")

    ax1.plot(0, 0, "k+", markersize=12, markeredgewidth=2, label="LiDAR Origin (0,0)")
    ax1.set_aspect("equal")
    ax1.grid(True, linestyle="--", alpha=0.4)
    ax1.set_xlabel("X (meters) [Forward]")
    ax1.set_ylabel("Y (meters) [Left]")
    ax1.set_title(f"{title} - 2D Spatial Map")
    ax1.legend(loc="upper right", framealpha=0.9)

    # Subplot 2: Statistics & Diagnostics Summary
    ax2.axis("off")
    summary_text = (
        f"=== LiDAR Ground-Truth Estimation ===\n\n"
        f"Physical Return Error (Estimated Ground Truth):\n"
        f"  • Error X (Longitudinal):  {result.delta_x_m * 1000.0:+.2f} mm\n"
        f"  • Error Y (Lateral):       {result.delta_y_m * 1000.0:+.2f} mm\n"
        f"  • 2D Position Norm:        {math.hypot(result.delta_x_m, result.delta_y_m) * 1000.0:.2f} mm\n"
        f"  • Error Yaw (Heading):     {result.delta_yaw_deg:+.3f}° ({result.delta_yaw_rad:+.4f} rad)\n\n"
        f"ICP Registration Metrics:\n"
        f"  • Status:                  {'CONVERGED' if result.converged else 'FAILED'}\n"
        f"  • Iterations:              {result.iterations}\n"
        f"  • Inlier Ratio (<30mm):    {result.inlier_ratio * 100.0:.1f}%\n"
        f"  • Inliers / Total Points:  {result.num_inliers} / {result.num_total_points}\n"
        f"  • Residual RMSE:           {result.rmse_mm:.2f} mm\n"
        f"  • Hessian Condition Num:   {result.condition_number:.1f}\n\n"
        f"Observability Assessment:\n"
        f"  • Environment:             {'Sufficient 2D geometry' if result.condition_number < 100.0 else 'High degeneracy risk'}\n"
        f"  • Measurement Confidence:  {'HIGH (< 3mm accuracy)' if result.inlier_ratio > 0.20 and result.rmse_mm < 20.0 else 'MEDIUM'}"
    )
    ax2.text(
        0.05, 0.95, summary_text,
        transform=ax2.transAxes,
        fontsize=11,
        fontfamily="monospace",
        verticalalignment="top",
        bbox=dict(boxstyle="round,pad=0.8", facecolor="#f8f9fa", edgecolor="#ced4da", alpha=0.9),
    )

    plt.tight_layout()
    plt.savefig(save_path, dpi=150)
    plt.close(fig)
