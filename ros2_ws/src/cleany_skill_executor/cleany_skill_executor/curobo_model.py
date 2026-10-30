"""Derive cuRobo configuration from the installed, authoritative robot model.

No source mesh or generated asset is changed. Sparse spheres guide cuRobo
optimization; complete convex envelopes (including base) validate on CUDA.
"""

from __future__ import annotations

from dataclasses import dataclass
from copy import copy
from functools import cached_property
import hashlib
import importlib.metadata
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np

from cleany_skill_executor.core.physical_grasp_selection import (
    ARM_JOINT_NAMES,
    REQUIRED_JOINT_NAMES,
)


@dataclass(frozen=True)
class CuRoboConfig:
    num_seeds: int = 32
    ik_position_threshold_m: float = 0.001
    joint_tolerance_rad: float = 0.002
    spheres_per_link: int = 64  # Historical name: cell count per mesh instance.
    sphere_budget_per_link: int = 16  # Merge within each rigid link; 0 keeps original fits.
    sphere_model: str = 'joint_simplified'
    sphere_surface_radius_m: float = 0.0005  # Outward padding on every enclosing ball.
    collision_hull_max_expansion_m: float = 0.001
    interpolation_dt: float = 0.05
    validation_step_rad: float = 0.025
    maximum_validation_states: int = 2000
    max_acceleration: float = 1.0
    max_jerk: float = 10.0
    use_cuda_graph: bool = True
    collision_margin_m: float = 0.0001
    moveit_shadow_validation: bool = False
    geometry_cache_directory: str = ''

    def __post_init__(self) -> None:
        if self.sphere_model not in ('joint_simplified', 'generated'):
            raise ValueError('Unknown collision sphere model')
        if (not math.isfinite(self.collision_hull_max_expansion_m)
                or self.collision_hull_max_expansion_m < 0):
            raise ValueError('GPU hull expansion must be finite and nonnegative')
        if type(self.sphere_budget_per_link) is not int or self.sphere_budget_per_link < 0:
            raise ValueError('Sphere budget must be a nonnegative integer')
        if (
            self.num_seeds < 2
            or self.maximum_validation_states < 2
            or self.spheres_per_link < 4
        ):
            raise ValueError("cuRobo seed and validation counts must be at least two")
        for value in (
            self.ik_position_threshold_m,
            self.joint_tolerance_rad,
            self.sphere_surface_radius_m,
            self.interpolation_dt,
            self.validation_step_rad,
            self.max_acceleration,
            self.max_jerk,
            self.collision_margin_m,
        ):
            if not math.isfinite(value) or value <= 0:
                raise ValueError(
                    "cuRobo distances, time and dynamic limits must be positive"
                )


class RobotModel:
    @cached_property
    def mast_obstacles(self) -> dict:
        from .curobo_mast import mast_cuboids

        return mast_cuboids(self.xml, self.config.sphere_surface_radius_m)

    def with_config(self, config: CuRoboConfig) -> RobotModel:
        """Share derived geometry while keeping each engine's tuning separate."""
        if (config.spheres_per_link, config.sphere_surface_radius_m, config.sphere_budget_per_link) != (
                self.config.spheres_per_link, self.config.sphere_surface_radius_m,
                self.config.sphere_budget_per_link):
            raise ValueError('Shared robot geometry requires identical sphere fitting settings')
        if config.collision_hull_max_expansion_m != self.config.collision_hull_max_expansion_m:
            raise ValueError('Shared robot geometry requires identical hull settings')
        if config.sphere_model != self.config.sphere_model:
            raise ValueError('Shared robot geometry requires identical sphere models')
        model = copy(self)
        model.config = config
        return model

    def __init__(self, urdf: str, srdf: str, config: CuRoboConfig) -> None:
        from .robot_geometry_cache import model_cache_path, load_model, save_model

        self.xml, self.config = urdf, config
        self.geometry_cache_hits = self.geometry_cache_misses = 0
        cache_path = model_cache_path(urdf, srdf, config)
        self.full_geometry_cache_hit = load_model(cache_path, self)
        if self.full_geometry_cache_hit:
            return
        # Optional dependencies are imported only for the cuRobo backend.
        import trimesh
        from ament_index_python.packages import get_package_share_directory
        from cleany_skill_executor.curobo_collision import convex_body, world_bodies
        from cleany_skill_executor.core.collision_geometry import covering_spheres, enclosing_polytope

        self.xml = urdf
        root = ET.fromstring(urdf)
        self.limits = {}
        self.velocity_limits = {}
        self.spheres = {}
        self.bodies = []
        self.geometry_cache_hits = 0
        self.geometry_cache_misses = 0
        fitted_meshes = {}
        fitted_hulls = {}
        for joint in root.findall("joint"):
            if joint.get("type") == "revolute":
                limit = joint.find("limit")
                self.limits[joint.get("name")] = (
                    float(limit.get("lower")),
                    float(limit.get("upper")),
                )
                self.velocity_limits[joint.get("name")] = float(limit.get("velocity"))
        required = set(REQUIRED_JOINT_NAMES)
        optional = {'head_pan_joint', 'head_tilt_joint'}
        if not required.issubset(self.limits) or set(self.limits) - required - optional:
            raise ValueError('cuRobo requires 12 arm/gripper joints and optional locked head joints')
        for link in root.findall("link"):
            name = link.get("name")
            spheres = []
            for collision in link.findall("collision"):
                origin = collision.find("origin")
                xyz = (
                    [float(v) for v in origin.get("xyz", "0 0 0").split()]
                    if origin is not None
                    else [0.0] * 3
                )
                rpy = (
                    [float(v) for v in origin.get("rpy", "0 0 0").split()]
                    if origin is not None
                    else [0.0] * 3
                )
                rotation = trimesh.transformations.euler_matrix(*rpy)[:3, :3]
                mesh = collision.find("geometry/mesh")
                if mesh is None:
                    box = collision.find("geometry/box")
                    if box is None:
                        cylinder = collision.find("geometry/cylinder")
                        sphere = collision.find("geometry/sphere")
                        if cylinder is not None:
                            shape = {'radius': float(cylinder.get('radius')),
                                     'height': float(cylinder.get('length'))}
                            kind = 'cylinder'
                        elif sphere is not None:
                            shape = {'radius': float(sphere.get('radius'))}
                            kind = 'sphere'
                        else:
                            raise ValueError(f"unsupported robot collision geometry: {name}")
                        if any(not math.isfinite(v) or v <= 0 for v in shape.values()):
                            raise ValueError(f"invalid robot primitive dimensions: {name}")
                        quat = trimesh.transformations.quaternion_from_matrix(
                            trimesh.transformations.euler_matrix(*rpy))
                        shape['pose'] = [*xyz, *quat]
                        # Reuse the enclosing primitive representation used for
                        # world geometry, keeping wheels/camera bodies present.
                        self.bodies.extend(world_bodies({kind: {name: shape}}))
                        continue
                    geometry = trimesh.creation.box(
                        extents=[float(v) for v in box.get("size").split()]
                    )
                    self.bodies.append(
                        convex_body(name, geometry.vertices @ rotation.T + xyz)
                    )
                    # Large base spheres would reject safe shoulder postures.
                    # Its exact box is instead checked by the GPU acceptance test.
                    continue
                uri = mesh.get("filename")
                if uri.startswith("package://"):
                    package, relative = uri[len("package://") :].split("/", 1)
                    path = Path(get_package_share_directory(package)) / relative
                else:
                    path = Path(uri)
                scale = tuple(float(v) for v in mesh.get("scale", "1 1 1").split())
                key = (str(path), scale)
                if name in ('base_link', 'top_base_link'):
                    geometry = trimesh.load_mesh(path, process=True)
                    geometry.apply_scale(scale)
                    source = geometry.convex_hull.vertices if name == 'top_base_link' else geometry.vertices
                    vertices = enclosing_polytope(source, config.collision_hull_max_expansion_m)
                    self.bodies.append(convex_body(name, vertices @ rotation.T + xyz))
                    # Full validation keeps both bodies. The fixed mast is
                    # represented by cuboids in every proposal checker.
                    continue
                if key not in fitted_meshes:
                    from cleany_skill_executor.geometry_cache import fitted_geometry

                    def fit():
                        geometry = trimesh.load_mesh(path, process=True)
                        geometry.apply_scale(scale)
                        centers, radii = covering_spheres(geometry.vertices, config.spheres_per_link,
                            config.sphere_surface_radius_m, faces=geometry.faces)
                        return centers, radii, geometry.convex_hull.vertices.copy()

                    versions = '/'.join(importlib.metadata.version(package)
                        for package in ('trimesh', 'nvidia_curobo'))
                    fitted_meshes[key], hit = fitted_geometry(path, scale,
                        config.spheres_per_link, config.sphere_surface_radius_m,
                        versions, config.geometry_cache_directory, fit)
                    self.geometry_cache_hits += int(hit)
                    self.geometry_cache_misses += int(not hit)
                centers, radii, vertices = fitted_meshes[key]
                if key not in fitted_hulls:
                    fitted_hulls[key] = enclosing_polytope(vertices, config.collision_hull_max_expansion_m)
                vertices = fitted_hulls[key]
                self.bodies.append(convex_body(name, vertices @ rotation.T + xyz))
                for center, radius in zip(centers, radii, strict=True):
                    spheres.append(
                        {
                            "center": (rotation @ center + xyz).tolist(),
                            "radius": float(radius),
                        }
                    )
            if spheres:
                self.spheres[name] = spheres
        self.collision_links = list(dict.fromkeys(body.owner for body in self.bodies))
        self.ignore = {name: [] for name in self.collision_links}
        for entry in ET.fromstring(srdf).findall("disable_collisions"):
            a, b = entry.get("link1"), entry.get("link2")
            if a in self.ignore and b in self.ignore:
                self.ignore[a].append(b)
        self.config = config
        self._reduce_collision_spheres()
        from .collision_sphere_model import apply_sphere_model

        apply_sphere_model(self)
        save_model(cache_path, self)

    def _reduce_collision_spheres(self) -> None:
        """Share conservative per-link reduction across every cuRobo backend.

        Mesh fitting/cache inputs and final convex validation bodies stay intact.
        Only the sparse optimization spheres are merged, before arm filtering.
        """
        from .core.sphere_reduction import reduce_spheres, sphere_cover_error

        budget = self.config.sphere_budget_per_link
        self.sphere_reduction = {'budget_per_link': budget, 'links': {}}
        for name, original in self.spheres.items():
            centers = np.asarray([item['center'] for item in original])
            radii = np.asarray([item['radius'] for item in original])
            reduced = (centers, radii) if budget == 0 else reduce_spheres(centers, radii, budget)
            error = sphere_cover_error(centers, radii, *reduced)
            if error > 1e-10:
                raise ValueError('Reduced sphere model lost collision coverage: ' + name)
            self.spheres[name] = [{'center': center.tolist(), 'radius': float(radius)}
                for center, radius in zip(*reduced, strict=True)]
            self.sphere_reduction['links'][name] = {
                'before': len(original), 'after': len(reduced[1]), 'cover_error_m': error}
        self.sphere_reduction['before'] = sum(item['before'] for item in self.sphere_reduction['links'].values())
        self.sphere_reduction['after'] = sum(item['after'] for item in self.sphere_reduction['links'].values())
        for key in ('before', 'after'):
            counts = {name: item[key] for name, item in self.sphere_reduction['links'].items()}
            self.sphere_reduction[key + '_self_collision_pairs'] = sum(
                counts[a] * counts[b] for a in counts for b in counts if a < b
                and b not in self.ignore.get(a, ()) and a not in self.ignore.get(b, ()))
        digest = hashlib.sha256()
        for body in self.bodies:
            digest.update(body.owner.encode())
            for values in (body.vertices, body.planes):
                digest.update(str((values.dtype.str, values.shape)).encode())
                digest.update(values.tobytes())
        self.sphere_reduction['validation_geometry_sha256'] = digest.hexdigest()

    def robot_config(
        self, path: str, arm: str, state: dict[str, float], tip: str
    ) -> dict:
        names = [*REQUIRED_JOINT_NAMES,
                 *(name for name in ('head_pan_joint', 'head_tilt_joint') if name in self.limits)]
        self.validate_state(state)
        if arm not in ARM_JOINT_NAMES or tip not in (
            f"{arm}_grasp_tcp",
            f"{arm}_pregrasp_aim_tip",
            f"{arm}_gripper_frame",
        ):
            raise ValueError("unsupported cuRobo arm or tip")
        robot = {
            "kinematics": {
                "urdf_path": path,
                "base_link": "base_link",
                "ee_link": tip,
                "link_names": [tip],
                "collision_link_names": list(self.spheres),
                "collision_spheres": self.spheres,
                "collision_sphere_buffer": 0.0,
                "self_collision_ignore": {
                    a: [b for b in others if b in self.spheres]
                    for a, others in self.ignore.items()
                    if a in self.spheres
                },
                "self_collision_buffer": {name: 0.0 for name in self.spheres},
                "use_global_cumul": True,
                "lock_joints": {
                    name: state[name]
                    for name in names
                    if name not in ARM_JOINT_NAMES[arm]
                },
                "cspace": {
                    "joint_names": names,
                    "retract_config": [state[name] for name in names],
                    "null_space_weight": [1.0] * len(names),
                    "cspace_distance_weight": [1.0] * len(names),
                    "max_acceleration": self.config.max_acceleration,
                    "max_jerk": self.config.max_jerk,
                    "position_limit_clip": 0.0,
                },
            }
        }
        return robot

    def validate_state(self, state: dict[str, float]) -> None:
        if set(state) != set(self.limits):
            raise ValueError('a complete, named runtime robot joint state is required')
        for name, (low, high) in self.limits.items():
            value = state[name]
            if not math.isfinite(value) or not low - 1e-6 <= value <= high + 1e-6:
                raise ValueError(f"joint outside URDF limits: {name}")


def validation_states(
    points, step: float, maximum: int
) -> tuple[tuple[float, ...], ...]:
    """Densify joint-space segments; never silently truncate validation."""
    if not math.isfinite(step) or step <= 0 or maximum < 1:
        raise ValueError("invalid validation spacing or budget")
    array = np.asarray(points, dtype=float)
    if (
        array.ndim != 2
        or len(array) == 0
        or array.shape[1] == 0
        or not np.isfinite(array).all()
    ):
        raise ValueError("invalid trajectory positions")
    counts = np.maximum(
        1, np.ceil(np.max(np.abs(np.diff(array, axis=0)), axis=1) / step)
    ).astype(int)
    if 1 + int(counts.sum()) > maximum:
        raise ValueError(
            "trajectory exceeds the configured collision-validation budget"
        )
    result = [tuple(array[0])]
    for a, b, count in zip(array[:-1], array[1:], counts):
        result.extend(
            tuple(a + (b - a) * index / count) for index in range(1, count + 1)
        )
    return tuple(result)
