"""Known fixture chair bounds for offline target generation, never live perception."""
from __future__ import annotations

from pathlib import Path
import numpy as np

from mesh_geometry import read_geometry


def chair_bounds(entries: list[dict], output: Path) -> dict[str, tuple[np.ndarray, np.ndarray]]:
    bounds = {}
    meshes = {}
    for entry in entries:
        if entry['kind'] != 'collision' or 'chair' not in entry['model']:
            continue
        matrix = np.asarray(entry['transform'], dtype=float)
        shape = entry['shape']
        if shape['type'] == 'box':
            extent = np.abs(matrix[:3, :3]) @ (np.asarray(shape['size'])/2.)
            lower, upper = matrix[:3, 3]-extent, matrix[:3, 3]+extent
        elif shape['type'] == 'cylinder':
            extent = shape['radius']*np.linalg.norm(matrix[:3, :2], axis=1)
            extent += shape['length']/2.*np.abs(matrix[:3, 2])
            lower, upper = matrix[:3, 3]-extent, matrix[:3, 3]+extent
        elif shape['type'] == 'mesh':
            asset = shape['asset']
            if asset not in meshes:
                meshes[asset] = np.asarray(read_geometry(output/asset)[0], dtype=float)
            points = meshes[asset]*np.asarray(shape.get('scale', [1., 1., 1.]))
            points = points @ matrix[:3, :3].T + matrix[:3, 3]
            lower, upper = points.min(axis=0), points.max(axis=0)
        else:
            raise ValueError(f"Unsupported chair collision shape: {shape['type']}")
        name = entry['model']
        if name in bounds:
            lower, upper = np.minimum(bounds[name][0], lower), np.maximum(bounds[name][1], upper)
        bounds[name] = (lower, upper)
    return bounds


def overlapping_chairs(point: np.ndarray, yaw: float, footprint: np.ndarray,
                       padding: float, chairs: dict) -> list[str]:
    """Conservative SAT test: rotated robot envelope versus fixture chair AABBs."""
    lower, upper = footprint.min(axis=0)-padding, footprint.max(axis=0)+padding
    local_center, robot_half = (lower+upper)/2., (upper-lower)/2.
    rotation = np.array([[np.cos(yaw), -np.sin(yaw)], [np.sin(yaw), np.cos(yaw)]])
    center = point + rotation @ local_center
    robot_world_half = np.abs(rotation) @ robot_half
    blocked = []
    for name, (lo, hi) in chairs.items():
        chair_center, chair_half = (lo[:2]+hi[:2])/2., (hi[:2]-lo[:2])/2.
        delta = chair_center-center
        if np.any(np.abs(delta) > robot_world_half+chair_half):
            continue
        if np.any(np.abs(rotation.T @ delta) > robot_half+np.abs(rotation.T) @ chair_half):
            continue
        blocked.append(name)
    return blocked
