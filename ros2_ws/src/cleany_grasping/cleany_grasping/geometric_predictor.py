from __future__ import annotations

import math
from dataclasses import dataclass, replace

import numpy as np

from cleany_grasping.core.models import PointCloud, RawGrasp


@dataclass(frozen=True)
class GeometricGraspConfig:
    """Configuration for deterministic top-down parallel-jaw candidates."""

    maximum_gripper_width_m: float = 0.10
    opening_margin_m: float = 0.008
    grasp_depth_m: float = 0.025
    maximum_top_contact_depth_m: float = 0.0  # Zero retains volume-center contact.
    finger_thickness_m: float = 0.010
    finger_length_m: float = 0.045
    palm_depth_m: float = 0.018
    collision_clearance_m: float = 0.003
    plane_distance_threshold_m: float = 0.006
    plane_ransac_iterations: int = 160
    extent_trim_percentile: float = 0.5
    axis_search_step_degrees: float = 1.0
    yaw_offsets_degrees: tuple[float, ...] = (-20.0, -10.0, 0.0, 10.0, 20.0)
    approach_tilt_degrees: float = 0.0
    approach_tilt_direction: tuple[float, float, float] = (1.0, 0.0, 0.0)
    reject_robot_opposite_approach: bool = False
    robot_reference_position: tuple[float, float, float] = (0.0, 0.0, 0.0)
    maximum_candidates: int = 12
    approach_tilt_options: tuple[float, ...] = ()
    include_reverse_closing_axis: bool = False
    prefer_upward_closing_axis: bool = False
    longitudinal_offset_fractions: tuple[float, ...] = (0.0,)
    longitudinal_max_height_ratio: float = 0.4
    longitudinal_contact_height_offset_m: float = 0.0
    defer_support_plane_collision: bool = False
    approach_reference_positions: tuple[tuple[float, float, float], ...] = ()

    def __post_init__(self) -> None:
        positive = (
            self.maximum_gripper_width_m,
            self.opening_margin_m,
            self.grasp_depth_m,
            self.finger_thickness_m,
            self.finger_length_m,
            self.palm_depth_m,
            self.collision_clearance_m,
            self.plane_distance_threshold_m,
        )
        if not all(math.isfinite(value) and value > 0.0 for value in positive):
            raise ValueError('Geometric grasp dimensions must be finite and positive')
        if not math.isfinite(self.maximum_top_contact_depth_m) or self.maximum_top_contact_depth_m < 0:
            raise ValueError('Maximum top contact depth must be finite and nonnegative')
        if (not self.longitudinal_offset_fractions
                or not all(math.isfinite(v) and abs(v) < .5 for v in self.longitudinal_offset_fractions)
                or not math.isfinite(self.longitudinal_max_height_ratio)
                or not 0 < self.longitudinal_max_height_ratio < 1
                or not math.isfinite(self.longitudinal_contact_height_offset_m)
                or not 0 <= self.longitudinal_contact_height_offset_m <= .01):
            raise ValueError('Invalid longitudinal contact search bounds')
        if self.plane_ransac_iterations <= 0 or self.maximum_candidates <= 0:
            raise ValueError('Geometric grasp iteration and candidate limits must be positive')
        if not all(math.isfinite(v) and 0 <= v < 90
                   for v in self.approach_tilt_options):
            raise ValueError('Approach tilt options must be in [0, 90)')
        if any(len(p) != 3 or not all(math.isfinite(v) for v in p)
               for p in self.approach_reference_positions):
            raise ValueError('Approach reference positions must be finite 3-vectors')
        if not 0.0 <= self.extent_trim_percentile < 25.0:
            raise ValueError('Extent trim percentile must be in [0, 25)')
        if not 0.0 < self.axis_search_step_degrees <= 15.0:
            raise ValueError('Axis search step must be in (0, 15] degrees')
        if not self.yaw_offsets_degrees or not all(
            math.isfinite(value) for value in self.yaw_offsets_degrees
        ):
            raise ValueError('At least one finite yaw offset is required')
        tilt_direction = np.asarray(self.approach_tilt_direction, dtype=float)
        robot_reference = np.asarray(
            self.robot_reference_position,
            dtype=float,
        )
        if (
            not math.isfinite(self.approach_tilt_degrees)
            or not 0.0 <= self.approach_tilt_degrees < 90.0
            or tilt_direction.shape != (3,)
            or not np.isfinite(tilt_direction).all()
            or np.linalg.norm(tilt_direction) <= 1e-9
        ):
            raise ValueError(
                'Approach tilt must use a finite angle in [0, 90) and '
                'a non-zero 3-vector'
            )
        if (
            robot_reference.shape != (3,)
            or not np.isfinite(robot_reference).all()
        ):
            raise ValueError(
                'Robot reference position must contain three finite values'
            )


def _fit_support_plane(
    target_points: np.ndarray,
    context_points: np.ndarray,
    config: GeometricGraspConfig,
) -> tuple[np.ndarray, np.ndarray]:
    target_min = target_points.min(axis=0) - config.collision_clearance_m
    target_max = target_points.max(axis=0) + config.collision_clearance_m
    outside_target = np.any(
        (context_points < target_min) | (context_points > target_max), axis=1
    )
    support_points = context_points[outside_target]
    if support_points.shape[0] < 3:
        support_points = context_points
    if support_points.shape[0] < 3:
        raise ValueError('At least three context points are required to fit support plane')

    rng = np.random.default_rng(0)
    best_inliers = np.zeros(support_points.shape[0], dtype=bool)
    for _ in range(config.plane_ransac_iterations):
        indices = rng.choice(support_points.shape[0], size=3, replace=False)
        left, middle, right = support_points[indices]
        normal = np.cross(middle - left, right - left)
        length = float(np.linalg.norm(normal))
        if length <= 1e-9:
            continue
        normal /= length
        distances = np.abs((support_points - left) @ normal)
        inliers = distances <= config.plane_distance_threshold_m
        if int(inliers.sum()) > int(best_inliers.sum()):
            best_inliers = inliers
    if int(best_inliers.sum()) < 3:
        raise ValueError('Could not estimate a support plane from context cloud')

    inlier_points = support_points[best_inliers]
    plane_center = inlier_points.mean(axis=0)
    _, _, vh = np.linalg.svd(inlier_points - plane_center, full_matrices=False)
    normal = vh[-1]
    target_center = np.median(target_points, axis=0)
    if float((target_center - plane_center) @ normal) < 0.0:
        normal = -normal
    return normal / np.linalg.norm(normal), plane_center


def _fit_support_normal(target_points, context_points, config) -> np.ndarray:
    return _fit_support_plane(target_points, context_points, config)[0]


def _tangent_axes(points: np.ndarray, normal: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    centered = points - np.median(points, axis=0)
    tangent = centered - np.outer(centered @ normal, normal)
    covariance = tangent.T @ tangent
    values, vectors = np.linalg.eigh(covariance)
    first = vectors[:, int(np.argmax(values))]
    first -= float(first @ normal) * normal
    first /= np.linalg.norm(first)
    second = np.cross(normal, first)
    second /= np.linalg.norm(second)
    return first, second


def _robust_volume_center(
    points: np.ndarray,
    axes: tuple[np.ndarray, np.ndarray, np.ndarray],
    trim_percentile: float,
) -> np.ndarray:
    basis = np.column_stack(axes)
    coordinates = points @ basis
    low, high = np.percentile(
        coordinates,
        (trim_percentile, 100.0 - trim_percentile),
        axis=0,
    )
    return ((low + high) / 2.0) @ basis.T


def _minimum_width_axis(
    points: np.ndarray,
    normal: np.ndarray,
    initial_axis: np.ndarray,
    trim_percentile: float,
    step_degrees: float,
) -> np.ndarray:
    orthogonal = np.cross(normal, initial_axis)
    best_axis = initial_axis
    best_width = math.inf
    for angle_degrees in np.arange(0.0, 180.0, step_degrees):
        angle = math.radians(float(angle_degrees))
        axis = math.cos(angle) * initial_axis + math.sin(angle) * orthogonal
        coordinates = points @ axis
        low, high = np.percentile(
            coordinates,
            (trim_percentile, 100.0 - trim_percentile),
        )
        width = float(high - low)
        if width < best_width:
            best_width = width
            best_axis = axis
    return best_axis / np.linalg.norm(best_axis)


def _longitudinal_contacts(center, lateral, normal, target, config):
    """Search along thin objects with an explicit optional height correction.

    Keep a full finger-length footprint inside the observed robust extent.
    This is candidate generation, not a force-closure or reachability claim.
    """
    if config.longitudinal_offset_fractions == (0.0,):
        return (center,)
    direction = lateral - float(lateral @ normal) * normal
    length = float(np.linalg.norm(direction))
    if length < 1e-9:
        return (center,)
    direction /= length
    trim = config.extent_trim_percentile
    low, high = np.percentile(target @ direction, (trim, 100-trim))
    bottom, top = np.percentile(target @ normal, (trim, 100-trim))
    span = float(high-low)
    if span <= config.finger_length_m or top-bottom > config.longitudinal_max_height_ratio*span:
        return (center,)
    current = float(center @ direction)
    lower = float(low + config.finger_length_m/2 - current)
    upper = float(high - config.finger_length_m/2 - current)
    offsets = dict.fromkeys(round(float(np.clip(fraction*span, lower, upper)), 9)
                            for fraction in config.longitudinal_offset_fractions)
    return tuple(center + offset*direction + config.longitudinal_contact_height_offset_m*normal
                 for offset in offsets)


def _collides(
    translation: np.ndarray,
    rotation: np.ndarray,
    width_m: float,
    target_points: np.ndarray,
    context_points: np.ndarray,
    config: GeometricGraspConfig,
) -> bool:
    target_min = target_points.min(axis=0) - config.collision_clearance_m
    target_max = target_points.max(axis=0) + config.collision_clearance_m
    obstacles = context_points[
        np.any((context_points < target_min) | (context_points > target_max), axis=1)
    ]
    if obstacles.shape[0] == 0:
        return False
    local = (obstacles - translation) @ rotation
    target_local = (target_points - translation) @ rotation
    approach, closing, lateral = local[:, 0], local[:, 1], local[:, 2]
    half_opening = width_m / 2.0
    half_thickness = config.finger_thickness_m / 2.0
    half_length = config.finger_length_m / 2.0
    reach = float(np.max(target_local[:, 0])) - config.collision_clearance_m
    clearance = config.collision_clearance_m
    finger_collision = (
        (approach >= -clearance)
        & (approach <= reach)
        & (np.abs(np.abs(closing) - (half_opening + half_thickness)) <= half_thickness + clearance)
        & (np.abs(lateral) <= half_length + clearance)
    )
    palm_collision = (
        (approach >= -config.palm_depth_m - clearance)
        & (approach <= clearance)
        & (np.abs(closing) <= half_opening + config.finger_thickness_m + clearance)
        & (np.abs(lateral) <= half_length + clearance)
    )
    return bool(np.any(finger_collision | palm_collision))


class GeometricGraspPredictor:
    """Generate top-down candidates from segmented RGB-D point clouds."""

    def __init__(self, config: GeometricGraspConfig | None = None) -> None:
        self._config = config or GeometricGraspConfig()

    def predict(
        self,
        target_cloud: PointCloud,
        context_cloud: PointCloud,
        workspace_bounds: np.ndarray,
    ) -> tuple[RawGrasp, ...]:
        if self._config.approach_tilt_options or self._config.approach_reference_positions:
            directions = [self._config.approach_tilt_direction]
            if self._config.approach_reference_positions:
                center = np.median(target_cloud.points, axis=0)
                directions = [tuple(center-np.asarray(origin))
                              for origin in self._config.approach_reference_positions]
            groups = [
                GeometricGraspPredictor(replace(
                    self._config, approach_tilt_degrees=tilt,
                    approach_tilt_options=(), approach_reference_positions=(),
                    approach_tilt_direction=direction,
                )).predict(target_cloud, context_cloud, workspace_bounds)
                for tilt in (self._config.approach_tilt_options or (self._config.approach_tilt_degrees,))
                for direction in directions
            ]
            # Round-robin preserves approach diversity under the candidate
            # cap; each constituent list is already ordered by quality.
            diverse = [group[index]
                       for index in range(max(map(len, groups), default=0))
                       for group in groups if index < len(group)]
            return tuple(diverse[:self._config.maximum_candidates])
        del workspace_bounds
        target = target_cloud.points
        context = context_cloud.points
        if target.shape[0] < 3 or context.shape[0] < 3:
            raise ValueError('Geometric grasping needs at least three target and context points')
        normal, plane_center = _fit_support_plane(target, context, self._config)
        collision_context = context
        if self._config.defer_support_plane_collision:
            # Only the fitted plane is deferred, never other context objects.
            # Caller must register a perceived support patch for full-robot
            # open-jaw and closure-sweep collision checks before execution.
            collision_context = context[
                np.abs((context-plane_center) @ normal) > self._config.plane_distance_threshold_m]
        major, pca_minor = _tangent_axes(target, normal)
        minor = _minimum_width_axis(
            target,
            normal,
            pca_minor,
            self._config.extent_trim_percentile,
            self._config.axis_search_step_degrees,
        )
        major = np.cross(normal, minor)
        major /= np.linalg.norm(major)
        center = _robust_volume_center(
            target,
            (major, minor, normal),
            self._config.extent_trim_percentile,
        )
        approach = -normal
        if self._config.approach_tilt_degrees > 0.0:
            tilt_direction = np.asarray(
                self._config.approach_tilt_direction,
                dtype=float,
            )
            tilt_direction -= float(tilt_direction @ normal) * normal
            tilt_norm = float(np.linalg.norm(tilt_direction))
            if tilt_norm <= 1e-9:
                raise ValueError(
                    'Approach tilt direction must not be parallel to the '
                    'support normal'
                )
            tilt_direction /= tilt_norm
            tilt = math.radians(self._config.approach_tilt_degrees)
            approach = (
                math.cos(tilt) * approach
                + math.sin(tilt) * tilt_direction
            )
            approach /= np.linalg.norm(approach)
        if self._config.reject_robot_opposite_approach:
            robot_to_target = center[:2] - np.asarray(
                self._config.robot_reference_position[:2],
                dtype=float,
            )
            horizontal_approach = approach[:2]
            if (
                np.linalg.norm(robot_to_target) > 1.0e-9
                and np.linalg.norm(horizontal_approach) > 1.0e-9
                and float(robot_to_target @ horizontal_approach) <= 0.0
            ):
                return ()
        generated: list[RawGrasp] = []
        axes = ((minor, major, -minor, -major)
                if self._config.include_reverse_closing_axis
                else (minor, major))
        for base_axis in axes:
            for yaw_degrees in self._config.yaw_offsets_degrees:
                yaw = math.radians(yaw_degrees)
                closing = math.cos(yaw) * base_axis + math.sin(yaw) * np.cross(
                    normal, base_axis
                )
                closing -= float(closing @ approach) * approach
                closing /= np.linalg.norm(closing)
                lateral = np.cross(approach, closing)
                lateral /= np.linalg.norm(lateral)
                rotation = np.column_stack((approach, closing, lateral))
                closing_coordinates = (target - center) @ closing
                trim = self._config.extent_trim_percentile
                low, high = np.percentile(
                    closing_coordinates,
                    (trim, 100.0 - trim),
                )
                width = float(high - low) + self._config.opening_margin_m
                if width > self._config.maximum_gripper_width_m:
                    continue
                contact = center.copy()
                if self._config.maximum_top_contact_depth_m > 0:
                    top = float(np.percentile(target @ normal, 100.0-trim))
                    contact += max(0.0, top-self._config.maximum_top_contact_depth_m
                                   -float(contact @ normal)) * normal
                width_score = 1.0 - width / self._config.maximum_gripper_width_m
                alignment_score = 1.0 - min(abs(yaw_degrees), 45.0) / 45.0
                short_axis_bonus = 1.0 if abs(float(base_axis @ minor)) > .999 else 0.0
                score = 0.55 * width_score + 0.30 * alignment_score + 0.15 * short_axis_bonus
                for contact_point in _longitudinal_contacts(
                        contact, lateral, normal, target, self._config):
                    translation = contact_point - self._config.grasp_depth_m * approach
                    if _collides(translation, rotation, width, target, collision_context, self._config):
                        continue
                    generated.append(
                        RawGrasp(
                            rotation=rotation,
                            translation=translation,
                            width_m=width,
                            depth_m=self._config.grasp_depth_m,
                            score=float(np.clip(score, 0.0, 1.0)),
                        )
                    )
        if len(self._config.longitudinal_offset_fractions) > 1:
            # Stable tie-break: nearer contact first, without increasing scores.
            reference = np.asarray(self._config.robot_reference_position)
            generated.sort(key=lambda candidate: float(np.linalg.norm(candidate.translation-reference)))
        if self._config.prefer_upward_closing_axis:
            # For this asymmetric tool, +closing is the fixed-jaw side.
            # Only break equal-quality ties; do not inflate confidence scores.
            generated.sort(key=lambda candidate: (
                round(candidate.score, 10), float(candidate.rotation[:, 1] @ normal)), reverse=True)
        else:
            generated.sort(key=lambda candidate: candidate.score, reverse=True)
        return tuple(generated[: self._config.maximum_candidates])
