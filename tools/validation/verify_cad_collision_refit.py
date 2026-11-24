#!/usr/bin/env python3
"""Check replacement meshes against the installed CAD and its cached GPU spheres."""
from __future__ import annotations

import argparse
from concurrent.futures import ProcessPoolExecutor
from functools import partial
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial import ConvexHull, cKDTree

from optimize_cad_collision import (
    PACKAGE, Mesh, MeshInput, TriangleIndex, clean, components, counts, directed_bound,
    group, maximum_plane_violation, read_mesh, sphere_gap, topology,
)


def collision_uris(root: ET.Element) -> dict[tuple[str, str], str]:
    """Extract mesh URIs and mask just these attributes for the URDF comparison."""
    result = {}
    for link in root.findall('link'):
        for element in link.findall('collision'):
            mesh = element.find('geometry/mesh')
            if mesh is None:
                continue
            key = (link.attrib['name'], element.attrib['name'])
            if key in result:
                raise ValueError(f'Duplicate collision name: {key}')
            result[key] = mesh.attrib.pop('filename')
    return result


def solid_hull(mesh: Mesh) -> ConvexHull:
    """Require one closed, convex component using the CAD detector's tolerances."""
    if topology(mesh) != (2, 0, 0) or len(components(mesh)) != 1:
        raise ValueError('Expected one closed manifold enclosure')
    a, b, c = mesh.vertices[mesh.faces].transpose(1, 0, 2)
    normals = np.cross(b-a, c-a)
    lengths = np.linalg.norm(normals, axis=1)
    if np.any(lengths <= 1e-18):
        raise ValueError('Degenerate enclosure face')
    normals /= lengths[:, None]
    center = mesh.vertices.mean(axis=0)
    center_offsets = np.einsum('ij,ij->i', normals, center-a)
    if np.any(np.abs(center_offsets) <= 1e-12):
        raise ValueError('Flat enclosure component')
    normals[center_offsets > 0] *= -1
    planes = np.column_stack((normals, -np.einsum('ij,ij->i', normals, a)))
    if maximum_plane_violation(mesh.vertices, planes) > 1e-9:
        raise ValueError('Faces do not bound a convex solid')
    return ConvexHull(mesh.vertices)


def verify_enclosure(source: Mesh, candidate: Mesh, limit: float) -> dict:
    """Bound a closed convex output by its farthest vertex from the source hull."""
    hull = solid_hull(candidate)
    violation = maximum_plane_violation(source.vertices, hull.equations)
    if violation > 0:
        raise ValueError(f'Enclosure cuts into the source by {violation} m')
    source_hull = solid_hull(source)
    index = TriangleIndex(source.vertices[source_hull.simplices])
    upper = cKDTree(source.vertices[source_hull.vertices]).query(candidate.vertices)[0]
    expansion = float(index.query(candidate.vertices, upper + 1e-12).max()) + 1e-9
    if expansion > limit:
        raise ValueError(f'Enclosure expands by {expansion} m')
    return {'maximum_source_plane_violation_m': violation, 'maximum_expansion_m': expansion}


def component_signatures(mesh: Mesh) -> list[tuple[int, int, int]]:
    return sorted(topology(piece) for piece in components(clean(mesh)))


def verify_mesh(job: tuple[tuple[str, str], MeshInput, Path], *, inner_limit: float) -> dict:
    (link, name), item, target = job
    if hashlib.sha256(target.read_bytes()).hexdigest() != target.stem:
        raise ValueError(f'Asset hash differs from its name: {target}')
    source, candidate = read_mesh(item.path), read_mesh(target)
    limit = item.solid_limit if item.enclosure else item.surface_limit
    if '_gripper_internal_' in link:
        limit = min(inner_limit, limit)
    try:
        if item.enclosure:
            proof = verify_enclosure(source, candidate, limit)
        else:
            original = clean(source)
            if component_signatures(original) != component_signatures(candidate):
                raise ValueError('Surface connectivity or boundary counts changed')
            proof = {'input_to_output': directed_bound(source, candidate, limit, max_depth=48),
                     'output_to_input': directed_bound(candidate, original, limit, max_depth=48)}
        gap = sphere_gap(candidate, item.spheres)
        if (gap or 0.) > max(1e-5, sphere_gap(source, item.spheres) or 0.):
            raise ValueError('Cached GPU sphere coverage exceeded')
    except ValueError as error:
        raise ValueError(f'{link}/{name}: {error}') from error
    return {'link': link, 'collision': name, 'asset': target.name, **counts(candidate),
            'limit_m': limit, 'proof': proof, 'gpu_sample_gap_m': gap}


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('reference', 'reference-package', 'previous', 'candidate', 'package', 'output'):
        parser.add_argument('--'+name, type=Path, required=True)
    for name in ('surface-mm', 'enclosure-mm', 'contact-mm', 'inner-mm'):
        parser.add_argument('--'+name, type=float, required=True)
    parser.add_argument('--jobs', type=int, default=4)
    args = parser.parse_args()
    limits = np.array([args.surface_mm, args.enclosure_mm, args.contact_mm, args.inner_mm])/1000
    if not np.all(np.isfinite(limits) & (limits > 0)) or args.jobs < 1:
        parser.error('Limits and the number of workers must be positive')
    reference, previous, candidate = [ET.parse(p).getroot() for p in
                                      (args.reference, args.previous, args.candidate)]
    uses = group(reference, args.reference_package, args.package, *limits[:3], 0., .25, None)
    inputs: dict[tuple[str, str], MeshInput] = {
        (link, name): item for item, rows in uses.items() for link, name, _ in rows}
    collision_uris(reference)
    before, after = collision_uris(previous), collision_uris(candidate)
    if len({ET.canonicalize(ET.tostring(root), strip_text=True)
            for root in (reference, previous, candidate)}) != 1:
        raise ValueError('The three URDFs differ beyond collision mesh filenames')
    jobs = []
    for key, uri in after.items():
        if before[key] == uri:
            continue
        if not uri.startswith(PACKAGE):
            raise ValueError(f'Unexpected collision mesh URI: {uri}')
        jobs.append((key, inputs[key], args.package/uri.removeprefix(PACKAGE)))
    with ProcessPoolExecutor(max_workers=args.jobs) as pool:
        records = list(pool.map(partial(verify_mesh, inner_limit=limits[3]), jobs))
    result = {name+'_sha256': hashlib.sha256(path.read_bytes()).hexdigest()
              for name, path in [('reference', args.reference), ('previous', args.previous),
                                 ('candidate', args.candidate)]}
    result.update(changed_mesh_instances=len(records), records=records)
    args.output.write_text(json.dumps(result, indent=2)+'\n')
    print(f'Checked {len(records)} changed collision mesh instances')


if __name__ == '__main__':
    main()
