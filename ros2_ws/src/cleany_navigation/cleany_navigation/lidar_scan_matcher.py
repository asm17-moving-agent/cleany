#!/usr/bin/env python3
"""2D LiDAR 스캔 사이의 상대 위치를 추정합니다.

강건한 2D point-to-line 및 point-to-point ICP 정합으로 기준 스캔과
대상 스캔 사이의 상대 변환(delta_x, delta_y, delta_yaw)을 계산합니다.
스캔 기반 추정값이며 독립적인 위치 참값이 아닙니다.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import math
from typing import Sequence
import numpy as np
from scipy.spatial import cKDTree

from .icp_convergence import BoundedCycleDetector


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
    convergence_reason: str = ""
    final_step_translation_m: float = 0.0
    final_step_rotation_rad: float = 0.0
    convergence_settings: dict[str, float | int] = field(default_factory=dict)


def _additive_pose_jacobians(rotated_points: np.ndarray) -> np.ndarray:
    """가산 [tx, ty, yaw] 갱신에 대한 R(yaw) q + t의 미분입니다.

    yaw 지렛팔은 병진된 점 R q + t가 아닌 R q입니다.
    """
    jacobians = np.zeros((len(rotated_points), 2, 3))
    jacobians[:, 0, 0] = 1.0
    jacobians[:, 1, 1] = 1.0
    jacobians[:, 0, 2] = -rotated_points[:, 1]
    jacobians[:, 1, 2] = rotated_points[:, 0]
    return jacobians


def _linearized_system(
    rotated_points: np.ndarray, translation: np.ndarray,
    reference: np.ndarray, normals: np.ndarray, linearity: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    """최적화와 관측 가능성 검사에 동일한 강건 잔차와 Jacobian을 구성합니다."""
    jacobians = _additive_pose_jacobians(rotated_points)
    differences = rotated_points + translation - reference
    rows, values = [], []
    for jacobian, difference, normal, linear in zip(
        jacobians, differences, normals, linearity
    ):
        if linear > 0.7:
            residual = float(normal @ difference)
            weight = math.sqrt(1.0 if abs(residual) < .02 else .02 / abs(residual))
            rows.append(weight * (normal @ jacobian))
            values.append(-weight * residual)
        else:
            residual = float(np.linalg.norm(difference))
            weight = .5 * math.sqrt(1.0 if residual < .02 else .02 / residual)
            rows.extend(weight * jacobian)
            values.extend(-weight * difference)
    return np.asarray(rows).reshape(-1, 3), np.asarray(values)


def validate_scan_match(
    result: ScanMatchResult,
    min_inlier_ratio: float = 0.40,
    min_num_inliers: int = 30,
    max_rmse_mm: float = 25.0,
    max_translation_condition_number: float = 35.0,
    min_translation_eigenvalue: float = 5.0,
    max_condition_number: float = 1e6,
) -> tuple[bool, str]:
    """수렴, 관측 가능성과 기하 퇴화 조건으로 2D ICP 정합 결과를 검증합니다.

    단일 평면 벽처럼 한 축 움직임을 관측할 수 없어 잔차 0의 거짓 정합을
    만들 수 있는 퇴화 기하를 거부합니다.
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
    """point-to-line과 point-to-point ICP를 사용하는 강건 2D 스캔 정합기입니다."""

    def __init__(
        self,
        min_range_m: float = 0.38,
        max_range_m: float = 10.0,
        voxel_size_m: float = 0.02,
        max_iterations: int = 50,
        tolerance: float = 1e-5,
        min_inlier_ratio: float = 0.40,
        convergence_config: dict | None = None,
    ) -> None:
        if (isinstance(min_inlier_ratio, bool)
                or not isinstance(min_inlier_ratio, (int, float))
                or not math.isfinite(min_inlier_ratio)
                or not 0 < min_inlier_ratio <= 1):
            raise ValueError("min_inlier_ratio must be finite and in (0, 1]")
        if type(max_iterations) is not int or max_iterations < 1:
            raise ValueError("max_iterations must be a positive integer")
        if convergence_config is not None and not isinstance(convergence_config, dict):
            raise ValueError("convergence_config must be an object")
        settings = {
            "translation_tolerance_m": tolerance,
            "rotation_tolerance_rad": tolerance,
            **(convergence_config or {}),
        }
        detector = BoundedCycleDetector(**settings)
        self.convergence_settings = {
            key: getattr(detector, key) for key in (
                "translation_tolerance_m", "rotation_tolerance_rad",
                "max_translation_span_m", "max_rotation_span_rad", "max_period",
            )
        }
        self.min_range_m = min_range_m
        self.max_range_m = max_range_m
        self.voxel_size_m = voxel_size_m
        self.max_iterations = max_iterations
        self.tolerance = detector.translation_tolerance_m
        self.rotation_tolerance_rad = detector.rotation_tolerance_rad
        self.min_inlier_ratio = float(min_inlier_ratio)

    def laserscan_to_points(
        self,
        ranges: Sequence[float],
        angle_min: float,
        angle_increment: float,
        range_min: float,
        range_max: float,
    ) -> np.ndarray:
        """LaserScan 하나를 (N, 2) 형태의 2D Cartesian 점으로 변환합니다."""
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
        """균일한 밀도를 위한 voxel 격자 필터입니다."""
        if len(points) == 0:
            return points
        coords = np.floor(points / self.voxel_size_m).astype(np.int64)
        _, unique_indices = np.unique(coords, axis=0, return_index=True)
        return points[unique_indices]

    def compute_normals(self, points: np.ndarray, k: int = 6) -> tuple[np.ndarray, np.ndarray]:
        """국소 PCA로 2D 법선과 선형성 가중치를 추정합니다."""
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
            # 가장 작은 고유값이 법선 방향에 대응합니다.
            normals[i] = evecs[:, 0]
            # (λ_max - λ_min) / λ_max 비율은 선형성을 나타냅니다.
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
        """target_points를 ref_points에 정렬하고 (dx, dy, dyaw)를 반환합니다.

        변환은 대상 좌표계의 점을 기준 좌표계로 옮깁니다.
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
                convergence_reason="insufficient_scan_points",
                convergence_settings=self.convergence_settings.copy(),
            )

        ref_tree = cKDTree(ref_pts)
        ref_normals, ref_linearity = self.compute_normals(ref_pts, k=7)

        tx, ty, yaw = initial_guess

        max_dist = 0.30
        min_dist = 0.04
        converged = False
        final_iter = 0

        convergence_reason = "maximum_iterations"
        step_norm = dyaw = 0.0
        cycle_detector = BoundedCycleDetector(**self.convergence_settings)

        for it in range(self.max_iterations):
            final_iter = it + 1
            # 현재 annealing 거리 임계값입니다.
            dist_thresh = max(min_dist, max_dist * (0.85 ** it))

            c = math.cos(yaw)
            s = math.sin(yaw)
            R = np.array([[c, -s], [s, c]])

            rotated_tgt = (R @ tgt_pts.T).T
            transformed_tgt = rotated_tgt + np.array([tx, ty])

            dists, indices = ref_tree.query(transformed_tgt)

            mask = dists < dist_thresh
            if np.sum(mask) < 15:
                convergence_reason = "insufficient_correspondences"
                break

            matched_ref = ref_pts[indices[mask]]
            matched_normals = ref_normals[indices[mask]]
            matched_linearity = ref_linearity[indices[mask]]
            A, b = _linearized_system(
                rotated_tgt[mask], np.array([tx, ty]),
                matched_ref, matched_normals, matched_linearity,
            )

            AtA = A.T @ A
            Atb = A.T @ b

            try:
                delta = np.linalg.solve(AtA + 1e-4 * np.eye(3), Atb)
            except np.linalg.LinAlgError:
                convergence_reason = "linear_solve_failed"
                break
            if not np.all(np.isfinite(delta)):
                convergence_reason = "non_finite_update"
                break

            dtx, dty, dyaw = delta
            tx += dtx
            ty += dty
            yaw = (yaw + dyaw + math.pi) % (2.0 * math.pi) - math.pi

            step_norm = math.sqrt(dtx * dtx + dty * dty)
            final_stage = dist_thresh == min_dist
            if (final_stage and step_norm < self.tolerance
                    and abs(dyaw) < self.rotation_tolerance_rad):
                converged = True
                convergence_reason = "step_tolerance"
                break
            if cycle_detector.observe((tx, ty, yaw), final_stage=final_stage):
                converged = True
                convergence_reason = "bounded_cycle"
                break

        # 최종 평가와 관측 가능성은 실제 최종 대응점을 사용합니다.
        c = math.cos(yaw)
        s = math.sin(yaw)
        R = np.array([[c, -s], [s, c]])
        final_rotated = (R @ tgt_pts.T).T
        final_tgt = final_rotated + np.array([tx, ty])
        final_dists, final_indices = ref_tree.query(final_tgt)

        inlier_threshold = 0.030  # 30mm
        inliers = final_dists < inlier_threshold
        final_p = final_rotated[inliers]
        final_A, _ = _linearized_system(
            final_p, np.array([tx, ty]), ref_pts[final_indices[inliers]],
            ref_normals[final_indices[inliers]], ref_linearity[final_indices[inliers]],
        )
        trans_H = final_A[:, :2].T @ final_A[:, :2]
        trans_eigs = np.linalg.eigvalsh(trans_H)
        min_trans_eig = float(trans_eigs[0])
        trans_cond_num = (float(trans_eigs[-1] / trans_eigs[0])
                          if trans_eigs[0] > 1e-9 else float("inf"))
        # 병진과 독립적으로 대상의 RMS 거리로 yaw를 정규화합니다.
        scale = max(float(np.sqrt(np.mean(np.sum(final_p ** 2, axis=1))))
                    if len(final_p) else 1.0, 1e-6)
        final_A[:, 2] /= scale
        full_h = final_A.T @ final_A
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
            convergence_reason=convergence_reason,
            final_step_translation_m=float(step_norm),
            final_step_rotation_rad=float(dyaw),
            convergence_settings=self.convergence_settings.copy(),
        )

        is_valid, reason = validate_scan_match(
            res, min_inlier_ratio=self.min_inlier_ratio
        )
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
    """2D 스캔 정합의 고해상도 진단 그림을 생성합니다."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    c = math.cos(result.delta_yaw_rad)
    s = math.sin(result.delta_yaw_rad)
    R = np.array([[c, -s], [s, c]])
    aligned_tgt = (R @ target_points.T).T + np.array([result.delta_x_m, result.delta_y_m])

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 7), gridspec_kw={"width_ratios": [1.4, 1.0]})

    # 첫 번째 패널: 2D 공간 정렬
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

    # 두 번째 패널: 통계 및 진단 요약
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
        f"  • Translation Condition:   {result.translation_condition_number:.1f}\n"
        f"  • Min Translation Eigen:   {result.min_translation_eigenvalue:.2f}\n"
        f"  • Convergence Reason:      {result.convergence_reason}\n"
        f"  • Quality Decision:        {'VALID' if result.is_valid else 'REJECTED'}\n"
        f"  • Reason:                  {result.rejection_reason}"
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
