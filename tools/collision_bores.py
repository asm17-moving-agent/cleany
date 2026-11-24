"""Recognize and cap short circular bores in collision-only CAD surfaces."""
from __future__ import annotations

from collections import defaultdict
from dataclasses import dataclass

import numpy as np
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import connected_components
import trimesh

from optimize_arm_collision_meshes import Mesh


@dataclass(frozen=True)
class Ring:
    indices: np.ndarray
    edges: np.ndarray
    center: np.ndarray
    normal: np.ndarray
    radius: float


def boundary(faces: np.ndarray) -> np.ndarray:
    edges = np.sort(faces[:, [0, 1, 1, 2, 2, 0]].reshape(-1, 2), axis=1)
    unique, counts = np.unique(edges, axis=0, return_counts=True)
    return unique[counts == 1]


def _rings(mesh: Mesh, max_radius: float) -> list[Ring]:
    a, b, c = mesh.vertices[mesh.faces].transpose(1, 0, 2)
    normals = np.cross(b-a, c-a)
    normals /= np.linalg.norm(normals, axis=1)[:, None]
    plane_offset = np.einsum('ij,ij->i', normals, a)
    bins = defaultdict(list)
    keys = np.rint(np.column_stack((normals*10_000, plane_offset/1e-5))).astype(int)
    for index, key in enumerate(keys):
        bins[tuple(key)].append(index)
    rings = []
    for group in bins.values():
        if len(group) < 8:
            continue
        edges = boundary(mesh.faces[group])
        ids, inverse = np.unique(edges, return_inverse=True)
        if not len(ids):
            continue
        inverse = inverse.reshape(-1, 2)
        graph = coo_matrix((np.ones(len(edges)), (inverse[:, 0], inverse[:, 1])), shape=(len(ids), len(ids)))
        count, labels = connected_components(graph, directed=False)
        degrees = np.bincount(inverse.ravel())
        for label in range(count):
            membership = labels == label
            indices = ids[membership]
            if len(indices) < 8 or np.any(degrees[membership] != 2):
                continue
            points = mesh.vertices[indices]
            center = points.mean(axis=0)
            radii = np.linalg.norm(points-center, axis=1)
            radius = float(radii.mean())
            normal = normals[group[0]]
            if 0 < radius <= max_radius and np.ptp(radii) <= radius*.075 and np.ptp((points-center) @ normal) < 2e-5:
                rings.append(Ring(indices,edges[membership[inverse[:, 0]]],center,normal,radius))
    return rings


def _cap(mesh: Mesh, ring: Ring) -> np.ndarray | None:
    """Triangulate the original rim with its planar surface orientation."""
    delta = mesh.vertices[ring.indices]-ring.center
    basis = delta[0]/np.linalg.norm(delta[0])
    other = np.cross(ring.normal,basis)
    order = np.argsort(np.arctan2(delta @ other, delta @ basis))
    ordered = ring.indices[order]
    edges = np.sort(np.column_stack((ordered,np.roll(ordered,-1))),axis=1)
    if set(map(tuple,edges)) != set(map(tuple,ring.edges)):
        return None
    return np.column_stack((np.full(len(ordered)-2,ordered[0]),ordered[1:-1],ordered[2:]))


def close_bores(mesh: Mesh, max_radius: float, max_length: float) -> tuple[Mesh, list[dict]]:
    """Fill only verified through bores with two intact rims and an inward wall."""
    body = trimesh.Trimesh(vertices=mesh.vertices,faces=mesh.faces,process=False)
    if not body.is_volume:
        return mesh, []
    rings = _rings(mesh,max_radius)
    masks = np.zeros(len(mesh.faces), dtype=bool)
    caps, report = [], []
    a, b, c = mesh.vertices[mesh.faces].transpose(1, 0, 2)
    face_normals = np.cross(b-a,c-a)
    for index, first in enumerate(rings):
        for second in rings[index+1:]:
            offset = second.center-first.center
            length = float(-offset @ first.normal)
            if (not .0002 <= length <= max_length or first.normal @ second.normal > -.999
                    or np.linalg.norm(offset+length*first.normal) > .00006
                    or abs(first.radius-second.radius) > .00004):
                continue
            delta = mesh.vertices-first.center
            along = delta @ first.normal
            radial = delta-along[:, None]*first.normal
            radius = np.linalg.norm(radial,axis=1)
            wall = (np.abs(radius-first.radius) <= first.radius*.04) & (along >= -length-1e-5) & (along <= 1e-5)
            found = wall[mesh.faces].all(axis=1)
            if masks[found].any() or found.sum() < len(first.indices):
                continue
            expected = np.vstack((first.edges,second.edges))
            if set(map(tuple,boundary(mesh.faces[found]))) != set(map(tuple,expected)):
                continue
            inward = (radial[mesh.faces[found]].mean(axis=1)*face_normals[found]).sum(axis=1)
            starts = _cap(mesh,first)
            ends = _cap(mesh,second)
            if (inward >= 0).any() or starts is None or ends is None:
                continue
            trial_faces = np.vstack((mesh.faces[~(masks|found)],*caps,starts,ends))
            trial = trimesh.Trimesh(vertices=mesh.vertices,faces=trial_faces,process=False)
            if not trial.is_volume or trial.euler_number != body.euler_number+2*(len(report)+1):
                continue
            masks |= found
            caps.extend((starts,ends))
            report.append({'radius_m':first.radius,'length_m':length,'removed_wall_triangles':int(found.sum()),
                           'cap_triangles':len(starts)+len(ends)})
    if not report:
        return mesh, []
    faces = np.vstack((mesh.faces[~masks],*caps))
    used, inverse = np.unique(faces,return_inverse=True)
    return Mesh(mesh.vertices[used],inverse.reshape(-1,3)), report
