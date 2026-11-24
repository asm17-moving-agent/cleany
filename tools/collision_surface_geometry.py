"""Bound complete triangle surfaces using convexity and adaptive subdivision.

Distance to a triangle is convex, so the largest corner distance bounds an
entire source triangle. Otherwise distance to a closed surface is 1-Lipschitz:
its value at the centroid plus the triangle radius is an upper bound. Triangles
that cannot yet be certified are bisected; every child must pass.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Protocol

import numpy as np
from scipy.spatial import cKDTree


class TriangleMesh(Protocol):
    vertices: np.ndarray
    faces: np.ndarray


def distances(points: np.ndarray, triangles: np.ndarray, reduce: bool = True) -> np.ndarray:
    """Point-to-triangle distances, with candidate triangles per input point."""
    a, b, c = triangles[:, :, 0, :], triangles[:, :, 1, :], triangles[:, :, 2, :]
    p = points[:, None, :]
    ab, ac, ap = b - a, c - a, p - a
    normal = np.cross(ab, ac)
    denom = np.einsum('ijk,ijk->ij', normal, normal)
    plane = np.einsum('ijk,ijk->ij', ap, normal)
    projected = p - normal * np.divide(
        plane, denom, out=np.zeros_like(plane), where=denom > 0)[:, :, None]
    q = projected - a
    d00 = np.einsum('ijk,ijk->ij', ab, ab)
    d01 = np.einsum('ijk,ijk->ij', ab, ac)
    d11 = np.einsum('ijk,ijk->ij', ac, ac)
    d20 = np.einsum('ijk,ijk->ij', q, ab)
    d21 = np.einsum('ijk,ijk->ij', q, ac)
    det = d00 * d11 - d01 * d01
    u = np.divide(d11 * d20 - d01 * d21, det, out=np.full_like(det, -1), where=det > 0)
    v = np.divide(d00 * d21 - d01 * d20, det, out=np.full_like(det, -1), where=det > 0)
    best = np.where(
        (u >= 0) & (v >= 0) & (u + v <= 1) & (denom > 0),
        np.divide(plane * plane, denom, out=np.full_like(denom, np.inf), where=denom > 0), np.inf)
    for start, end in ((a, b), (b, c), (c, a)):
        edge = end - start
        length = np.einsum('ijk,ijk->ij', edge, edge)
        t = np.clip(np.divide(
            np.einsum('ijk,ijk->ij', p - start, edge), length,
            out=np.zeros_like(length), where=length > 0), 0, 1)
        delta = p - start - edge * t[:, :, None]
        best = np.minimum(best, np.einsum('ijk,ijk->ij', delta, delta))
    return np.sqrt(best.min(axis=1) if reduce else best)


@dataclass
class _Node:
    low: np.ndarray
    high: np.ndarray
    faces: np.ndarray | None = None
    left: int | None = None
    right: int | None = None


class TriangleIndex:
    """Triangle AABB hierarchy; centroid lookup alone can miss large faces."""

    def __init__(self, triangles: np.ndarray):
        self.triangles = triangles
        self.nodes: list[_Node] = []
        tree = cKDTree(triangles.mean(axis=1), leafsize=16)

        def add(node) -> int:
            index = len(self.nodes)
            points = triangles[node.indices]
            record = _Node(points.min(axis=(0, 1)), points.max(axis=(0, 1)))
            self.nodes.append(record)
            if node.split_dim < 0:
                record.faces = node.indices
            else:
                record.left, record.right = add(node.lesser), add(node.greater)
            return index

        add(tree.tree)

    def query(self, points: np.ndarray, upper: np.ndarray) -> np.ndarray:
        best = upper.copy()
        stack = [(0, np.arange(len(points)))]
        while stack:
            number, selected = stack.pop()
            node = self.nodes[number]
            delta = np.maximum(np.maximum(node.low - points[selected], points[selected] - node.high), 0)
            selected = selected[np.linalg.norm(delta, axis=1) <= best[selected] + 1e-12]
            if not len(selected):
                continue
            if node.faces is not None:
                triangles = np.broadcast_to(
                    self.triangles[node.faces], (len(selected), len(node.faces), 3, 3))
                best[selected] = np.minimum(best[selected], distances(points[selected], triangles))
            else:
                assert node.left is not None and node.right is not None
                stack.append((node.left, selected))
                stack.append((node.right, selected))
        return best


def directed_bound(source: TriangleMesh, target: TriangleMesh, limit: float,
                   max_depth: int = 24) -> dict[str, float | int]:
    if not np.isfinite(limit) or limit <= 0:
        raise ValueError('Surface distance limit must be positive and finite')
    triangles = target.vertices[target.faces]
    tree = cKDTree(triangles.mean(axis=1))
    exact = TriangleIndex(triangles)
    pending = source.vertices[source.faces]
    verified, maximum = 0, 0.
    for depth in range(max_depth):
        unresolved = []
        for start in range(0, len(pending), 2048):
            batch = pending[start:start + 2048]
            center = batch.mean(axis=1)
            radius = np.linalg.norm(batch - center[:, None, :], axis=2).max(axis=1)
            _, index = tree.query(center, k=min(16, len(triangles)))
            candidates = triangles[np.asarray(index).reshape(len(center), -1)]
            corner_bound = np.maximum.reduce([
                distances(batch[:, i], candidates, False) for i in range(3)]).min(axis=1) + 1e-9
            direct = corner_bound <= limit
            verified += int(direct.sum())
            if direct.any():
                maximum = max(maximum, float(corner_bound[direct].max()))
            batch, center = batch[~direct], center[~direct]
            radius, candidates = radius[~direct], candidates[~direct]
            if not len(batch):
                continue
            distance = distances(center, candidates)
            refine = distance > limit * .5
            if refine.any():
                distance[refine] = exact.query(center[refine], distance[refine])
            if (distance > limit).any():
                raise ValueError(f'Surface distance upper bound {distance.max():.9f} exceeds {limit}')
            bound = distance + radius + 1e-9  # Outward numerical allowance.
            good = bound <= limit
            verified += int(good.sum())
            if good.any():
                maximum = max(maximum, float(bound[good].max()))
            unresolved.append(batch[~good])
        pending = np.concatenate(unresolved) if unresolved else np.empty((0, 3, 3))
        if not len(pending):
            return {'maximum_distance_bound_m': maximum, 'verified_surface_patches': verified,
                    'subdivision_depth': depth}
        # Bisect the longest edge; both children exactly partition the parent.
        lens = np.linalg.norm(pending[:, [1, 2, 0]] - pending[:, [0, 1, 2]], axis=2)
        edge = lens.argmax(axis=1)
        indices = (edge[:, None] + np.arange(3)[None, :]) % 3
        ordered = np.take_along_axis(pending, indices[:, :, None], axis=1)
        a, b, c = ordered[:, 0], ordered[:, 1], ordered[:, 2]
        mid = (a + b) * .5
        pending = np.concatenate([np.stack([a, mid, c], axis=1), np.stack([mid, b, c], axis=1)])
        if len(pending) > 4000000:
            raise ValueError('Surface verification subdivision budget exceeded')
    raise ValueError('Surface verification depth exceeded')
