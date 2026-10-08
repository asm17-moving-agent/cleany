"""Simulation-only collision-clearance and target-visibility evidence."""

from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path

import mujoco
import numpy as np

from cleany_handeye_calibration.joint_state_sync import LEFT_ARM_JOINT_NAMES
from cleany_handeye_calibration.models import JointPose
from cleany_handeye_calibration.pnp import solve_planar_pnp
from cleany_handeye_calibration.target_detector import CharucoTargetDetector
from cleany_mujoco_sim.camera_contract import (
    CAMERA_D,
    CAMERA_FRAME_ID,
    CAMERA_K,
)
from cleany_mujoco_sim.scene_loader import resolve_scene_path


CAMERA_NAME = 'left_wrist_rgb'
TARGET_SITE_NAME = 'charuco_target_frame'
LEFT_ARM_ROOT_BODY = 'Base'
TARGET_WIDTH_M = 0.210
TARGET_HEIGHT_M = 0.150
CAMERA_WIDTH_PX = 640
CAMERA_HEIGHT_PX = 480
CAMERA_VERTICAL_FOV_DEG = 93.0
FIXTURE_GEOM_NAMES = (
    'handeye_table_top',
    'handeye_stand_base',
    'handeye_stand_post',
    'handeye_stand_crossbar',
    'charuco_target_backing',
)


def convex_separation_lower_bound(
    model: mujoco.MjModel, data: mujoco.MjData, first: int, second: int,
) -> float:
    """Certify separation of compiled box/mesh/sphere geometry in metres.

    A gap between full support intervals along a unit direction is a lower
    bound on Euclidean clearance. Unsupported shapes and overlap return zero.
    Sampled directions affect tightness, never the validity of the bound.
    """
    centers = [np.asarray(data.geom_xpos[i]) for i in (first, second)]
    rotations = [np.asarray(data.geom_xmat[i]).reshape(3, 3)
                 for i in (first, second)]
    axes = np.vstack((np.eye(3), rotations[0].T, rotations[1].T,
                      centers[0] - centers[1]))
    lengths = np.linalg.norm(axes, axis=1)
    axes = axes[np.isfinite(lengths) & (lengths > 1e-12)]
    lengths = np.linalg.norm(axes, axis=1)
    if not len(axes):
        return 0.0
    axes = axes / lengths[:, None]
    intervals = []
    for geom, center, rotation in zip((first, second), centers, rotations):
        local_axes = axes @ rotation
        projected_center = axes @ center
        shape = model.geom_type[geom]
        if shape == mujoco.mjtGeom.mjGEOM_BOX:
            radius = np.abs(local_axes) @ model.geom_size[geom]
            low, high = projected_center - radius, projected_center + radius
        elif shape == mujoco.mjtGeom.mjGEOM_SPHERE:
            radius = float(model.geom_size[geom, 0])
            low, high = projected_center - radius, projected_center + radius
        elif shape == mujoco.mjtGeom.mjGEOM_MESH:
            mesh = int(model.geom_dataid[geom])
            start = int(model.mesh_vertadr[mesh])
            count = int(model.mesh_vertnum[mesh])
            if count == 0:
                return 0.0
            vertices = np.asarray(model.mesh_vert[start:start + count],
                                  dtype=np.float64)
            projections = vertices @ local_axes.T + projected_center
            low, high = projections.min(axis=0), projections.max(axis=0)
        else:
            return 0.0
        if not np.isfinite(low).all() or not np.isfinite(high).all():
            return 0.0
        intervals.append((low, high))
    (low_a, high_a), (low_b, high_b) = intervals
    gap = float(np.max(np.maximum(low_a - high_b, low_b - high_a)))
    scale = max(1.0, *(float(np.max(np.abs(v)))
                       for interval in intervals for v in interval))
    roundoff = 32.0 * np.finfo(np.float64).eps * scale
    margins = np.asarray([model.geom_margin[first], model.geom_margin[second]])
    if not np.isfinite(margins).all():
        return 0.0
    # Negative contact margins must never enlarge a geometry clearance proof.
    margin = float(np.maximum(margins, 0.0).sum())
    return max(0.0, gap - margin - roundoff)


@dataclass(frozen=True, slots=True)
class MujocoPoseEvidence:
    minimum_collision_distance_m: float
    target_visible: bool
    camera_front: bool
    minimum_camera_depth_m: float
    base_gripper_position_m: tuple[float, float, float]


@dataclass(frozen=True, slots=True)
class RenderedTargetEvidence:
    detected: bool
    failure_reason: str | None
    corner_count: int
    covered_quadrants: tuple[str, ...]
    pnp_valid: bool
    pnp_failure_reason: str | None
    selected_reprojection_rmse_px: float | None


class MujocoRenderedTargetEvaluator:
    """Render the exact public camera view and run the production detector."""

    def __init__(self, scene_path: str | Path) -> None:
        source = Path(scene_path).expanduser().resolve(strict=True)
        resolved = resolve_scene_path(source)
        self._model = mujoco.MjModel.from_xml_path(str(resolved))
        self._data = mujoco.MjData(self._model)
        self._renderer = mujoco.Renderer(
            self._model,
            height=CAMERA_HEIGHT_PX,
            width=CAMERA_WIDTH_PX,
        )
        self._detector = CharucoTargetDetector()
        self._joint_qpos_addresses = tuple(
            int(
                self._model.jnt_qposadr[
                    self._required_id(mujoco.mjtObj.mjOBJ_JOINT, name)
                ]
            )
            for name in LEFT_ARM_JOINT_NAMES
        )

    def _required_id(self, object_type, name: str) -> int:
        object_id = int(mujoco.mj_name2id(self._model, object_type, name))
        if object_id < 0:
            raise ValueError(f'MuJoCo scene is missing {name!r}')
        return object_id

    def evaluate(self, pose: JointPose) -> RenderedTargetEvidence:
        if pose.joint_names != LEFT_ARM_JOINT_NAMES:
            raise ValueError('pose must use canonical left-arm joint order')
        mujoco.mj_resetData(self._model, self._data)
        for address, position in zip(
            self._joint_qpos_addresses,
            pose.positions_rad,
            strict=True,
        ):
            self._data.qpos[address] = position
        mujoco.mj_forward(self._model, self._data)
        self._renderer.update_scene(self._data, camera=CAMERA_NAME)
        detection = self._detector.detect(self._renderer.render())
        reason = (
            None
            if detection.failure_reason is None
            else detection.failure_reason.value
        )
        pnp_valid = False
        pnp_reason = None
        selected_rmse = None
        if detection.valid:
            pnp = solve_planar_pnp(
                detection,
                camera_matrix=np.asarray(CAMERA_K).reshape(3, 3),
                distortion_coefficients=CAMERA_D,
                camera_frame=CAMERA_FRAME_ID,
                target_frame='charuco_target',
            )
            pnp_valid = pnp.valid
            pnp_reason = (
                None
                if pnp.failure_reason is None
                else pnp.failure_reason.value
            )
            if pnp.selected_candidate_index is not None:
                selected = next(
                    candidate
                    for candidate in pnp.candidates
                    if candidate.index == pnp.selected_candidate_index
                )
                selected_rmse = selected.refined_reprojection_rmse_px
        return RenderedTargetEvidence(
            detected=detection.valid,
            failure_reason=reason,
            corner_count=len(detection.corner_ids),
            covered_quadrants=detection.covered_quadrants,
            pnp_valid=pnp_valid,
            pnp_failure_reason=pnp_reason,
            selected_reprojection_rmse_px=selected_rmse,
        )

    def close(self) -> None:
        self._renderer.close()


class MujocoPoseEvidenceEvaluator:
    """Evaluate a resolved left-arm pose in an isolated MuJoCo data model.

    MoveIt remains the authority for self-collision and planning. This model
    measures the left-arm-to-calibration-fixture clearance and analytically
    projects all four board corners through the exact MuJoCo camera pose.
    """

    def __init__(
        self,
        scene_path: str | Path,
        *,
        minimum_camera_depth_m: float,
        image_border_fraction: float,
    ) -> None:
        source = Path(scene_path).expanduser().resolve(strict=True)
        resolved = resolve_scene_path(source)
        self._model = mujoco.MjModel.from_xml_path(str(resolved))
        self._data = mujoco.MjData(self._model)
        self._minimum_camera_depth_m = float(minimum_camera_depth_m)
        self._image_border_fraction = float(image_border_fraction)
        if (
            not math.isfinite(self._minimum_camera_depth_m)
            or self._minimum_camera_depth_m <= 0.0
        ):
            raise ValueError('minimum_camera_depth_m must be positive')
        if (
            not math.isfinite(self._image_border_fraction)
            or not 0.0 < self._image_border_fraction < 1.0
        ):
            raise ValueError('image_border_fraction must be in (0, 1)')

        self._camera_id = self._required_id(
            mujoco.mjtObj.mjOBJ_CAMERA,
            CAMERA_NAME,
        )
        self._target_site_id = self._required_id(
            mujoco.mjtObj.mjOBJ_SITE,
            TARGET_SITE_NAME,
        )
        self._base_body_id = self._required_id(
            mujoco.mjtObj.mjOBJ_BODY,
            'chassis',
        )
        self._gripper_body_id = self._required_id(
            mujoco.mjtObj.mjOBJ_BODY,
            'Fixed_Jaw',
        )
        self._joint_qpos_addresses = tuple(
            int(
                self._model.jnt_qposadr[
                    self._required_id(mujoco.mjtObj.mjOBJ_JOINT, name)
                ]
            )
            for name in LEFT_ARM_JOINT_NAMES
        )
        left_root = self._required_id(
            mujoco.mjtObj.mjOBJ_BODY,
            LEFT_ARM_ROOT_BODY,
        )
        self._left_collision_geom_ids = tuple(
            geom_id
            for geom_id in range(self._model.ngeom)
            if self._is_descendant(
                int(self._model.geom_bodyid[geom_id]),
                left_root,
            )
            and (
                int(self._model.geom_contype[geom_id]) != 0
                or int(self._model.geom_conaffinity[geom_id]) != 0
            )
        )
        self._fixture_geom_ids = tuple(
            self._required_id(mujoco.mjtObj.mjOBJ_GEOM, name)
            for name in FIXTURE_GEOM_NAMES
        )
        if not self._left_collision_geom_ids:
            raise ValueError('MuJoCo model has no left-arm collision geoms')

    def _required_id(self, object_type, name: str) -> int:
        object_id = int(mujoco.mj_name2id(self._model, object_type, name))
        if object_id < 0:
            raise ValueError(f'MuJoCo scene is missing {name!r}')
        return object_id

    def _is_descendant(self, body_id: int, ancestor_id: int) -> bool:
        current = body_id
        while current > 0:
            if current == ancestor_id:
                return True
            current = int(self._model.body_parentid[current])
        return False

    def evaluate(self, pose: JointPose) -> MujocoPoseEvidence:
        if pose.joint_names != LEFT_ARM_JOINT_NAMES:
            raise ValueError('pose must use canonical left-arm joint order')
        mujoco.mj_resetData(self._model, self._data)
        for address, position in zip(
            self._joint_qpos_addresses,
            pose.positions_rad,
            strict=True,
        ):
            self._data.qpos[address] = position
        mujoco.mj_forward(self._model, self._data)

        clearance = min(
            self._geom_distance(left_geom, fixture_geom)
            for left_geom in self._left_collision_geom_ids
            for fixture_geom in self._fixture_geom_ids
        )
        visible, camera_front, minimum_depth = self._visibility()
        base_rotation = np.asarray(
            self._data.xmat[self._base_body_id],
            dtype=np.float64,
        ).reshape(3, 3)
        gripper_position = base_rotation.T @ (
            np.asarray(self._data.xpos[self._gripper_body_id])
            - np.asarray(self._data.xpos[self._base_body_id])
        )
        return MujocoPoseEvidence(
            minimum_collision_distance_m=max(0.0, clearance),
            target_visible=visible,
            camera_front=camera_front,
            minimum_camera_depth_m=minimum_depth,
            base_gripper_position_m=tuple(
                float(value) for value in gripper_position
            ),
        )

    def _geom_distance(self, first: int, second: int) -> float:
        from_to = np.zeros(6, dtype=np.float64)
        distance = float(
            mujoco.mj_geomDistance(
                self._model,
                self._data,
                first,
                second,
                2.0,
                from_to,
            )
        )
        if not math.isfinite(distance):
            return 0.0
        if distance > 0.0:
            return distance
        # mj_geomDistance can report zero for separated convex geoms. Only
        # replace it with an independently certified *lower* bound, never a
        # witness-segment length or a collision-envelope approximation.
        return max(distance, convex_separation_lower_bound(
            self._model, self._data, first, second,
        ))

    def _visibility(self) -> tuple[bool, bool, float]:
        target_position = np.asarray(
            self._data.site_xpos[self._target_site_id],
            dtype=np.float64,
        )
        target_rotation = np.asarray(
            self._data.site_xmat[self._target_site_id],
            dtype=np.float64,
        ).reshape(3, 3)
        camera_position = np.asarray(
            self._data.cam_xpos[self._camera_id],
            dtype=np.float64,
        )
        camera_rotation = np.asarray(
            self._data.cam_xmat[self._camera_id],
            dtype=np.float64,
        ).reshape(3, 3)

        local_corners = np.asarray(
            (
                (0.0, 0.0, 0.0),
                (TARGET_WIDTH_M, 0.0, 0.0),
                (0.0, TARGET_HEIGHT_M, 0.0),
                (TARGET_WIDTH_M, TARGET_HEIGHT_M, 0.0),
            ),
            dtype=np.float64,
        )
        world_corners = (
            target_position + local_corners @ target_rotation.T
        )
        camera_corners = (
            world_corners - camera_position
        ) @ camera_rotation
        depths = -camera_corners[:, 2]
        minimum_depth = float(np.min(depths))

        vertical_tangent = math.tan(
            math.radians(CAMERA_VERTICAL_FOV_DEG) / 2.0
        )
        horizontal_tangent = vertical_tangent * (
            CAMERA_WIDTH_PX / CAMERA_HEIGHT_PX
        )
        usable = 1.0 - self._image_border_fraction
        positive_depth = bool(
            np.all(depths >= self._minimum_camera_depth_m)
        )
        if positive_depth:
            normalized_x = np.abs(camera_corners[:, 0] / depths)
            normalized_y = np.abs(camera_corners[:, 1] / depths)
            in_frame = bool(
                np.all(normalized_x <= horizontal_tangent * usable)
                and np.all(normalized_y <= vertical_tangent * usable)
            )
        else:
            in_frame = False

        target_outward_normal = target_rotation[:, 2]
        camera_front = bool(
            np.dot(camera_position - target_position, target_outward_normal)
            > 0.0
        )
        return (
            positive_depth and in_frame and camera_front,
            camera_front,
            minimum_depth,
        )


__all__ = [
    'MujocoPoseEvidence',
    'MujocoPoseEvidenceEvaluator',
    'MujocoRenderedTargetEvaluator',
    'RenderedTargetEvidence',
]
