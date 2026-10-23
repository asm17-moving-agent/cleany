"""Compare an offline geometry-based sphere fit with a saved cuRobo model.

This experiment partitions each link's component hulls directly. Every cell
is enclosed by a ball fitted to its geometry, not by merging existing balls.
The output is a review artifact and never changes robot configuration.
"""
from __future__ import annotations

import argparse
from collections import Counter
from dataclasses import dataclass
import hashlib
import itertools
import json
from pathlib import Path
import time

import numpy as np
from scipy.optimize import minimize
from scipy.spatial import ConvexHull, QhullError
from scipy.stats import qmc
import trimesh

from render_collision_visualization import load_geometry, moved
from optimize_arm_collision_meshes import Mesh, components


OPTIMIZER_STATUS = Counter()
GEOMETRY_STATUS = Counter()


@dataclass
class Cell:
    pieces: list[np.ndarray]
    center: np.ndarray
    radius: float


def enclosing_ball(points: np.ndarray) -> tuple[np.ndarray, float]:
    """Optimize the center, then recompute a strictly enclosing radius."""
    origin = (points.min(0) + points.max(0)) / 2
    scale = np.linalg.norm(points - origin, axis=1).max()
    p = (points - origin) / scale
    initial = np.r_[np.zeros(3), 1.]
    result = minimize(lambda x: x[3], initial, method='SLSQP',
        jac=lambda x: np.array([0., 0., 0., 1.]),
        constraints={'type': 'ineq', 'fun': lambda x: x[3] - ((p - x[:3]) ** 2).sum(1),
                     'jac': lambda x: np.column_stack((2 * (p - x[:3]), np.ones(len(p))))},
        options={'ftol': 1e-10, 'maxiter': 150})
    OPTIMIZER_STATUS['success' if result.success else str(result.message)] += 1
    candidates = [origin]
    if np.isfinite(result.x[:3]).all():
        candidates.append(result.x[:3] * scale + origin)
    # Solver convergence affects fit quality only. Recomputing the radius
    # against every vertex is the enclosure certificate in either case.
    center = min(candidates, key=lambda c: np.linalg.norm(points-c, axis=1).max())
    radius = np.linalg.norm(points - center, axis=1).max()
    return center, float(np.nextafter(radius, np.inf))


def cell(pieces: list[np.ndarray]) -> Cell:
    center, radius = enclosing_ball(np.unique(np.vstack(pieces), axis=0))
    return Cell(pieces, center, radius)


def clip(pieces: list[np.ndarray], direction: np.ndarray, cut: float) -> tuple[list, list]:
    """Clip every convex component through its hull edges, preserving volume."""
    sides = [], []
    for vertices in pieces:
        distance = vertices @ direction - cut
        if distance.max() <= 1e-12:
            sides[0].append(vertices)
            continue
        if distance.min() >= -1e-12:
            sides[1].append(vertices)
            continue
        hull = ConvexHull(vertices)
        edges = np.unique(np.sort(hull.simplices[:, [0, 1, 1, 2, 2, 0]].reshape(-1, 2), axis=1), axis=0)
        a, b = edges.T
        crossing = distance[a] * distance[b] < 0
        a, b = a[crossing], b[crossing]
        intersections = vertices[a] + (vertices[b] - vertices[a]) * (
            distance[a] / (distance[a] - distance[b]))[:, None]
        for index, mask in enumerate((distance <= 0, distance >= 0)):
            points = np.unique(np.vstack((vertices[mask], intersections)), axis=0)
            sides[index].append(points[ConvexHull(points).vertices])
    return sides


def best_split(parent: Cell, directions: np.ndarray, supports: np.ndarray) -> tuple[Cell, Cell]:
    points = np.unique(np.vstack(parent.pieces), axis=0)
    _, _, basis = np.linalg.svd(points - points.mean(0), full_matrices=False)
    best = None
    for direction in basis:
        extent = points @ direction
        for fraction in (.4, .5, .6):
            cut = extent.min() + fraction * np.ptp(extent)
            try:
                pieces = clip(parent.pieces, direction, cut)
            except QhullError:
                GEOMETRY_STATUS['rejected_numerically_degenerate_cut'] += 1
                continue
            if not all(pieces):
                continue
            children = cell(pieces[0]), cell(pieces[1])
            overreach = max(float((child.center @ directions.T + child.radius - supports).max())
                            for child in children)
            volume = sum(child.radius ** 3 for child in children)
            score = (-round(overreach, 10), -volume)
            if best is None or score > best[0]:
                best = score, *children
    if best is None:
        raise ValueError('Cannot partition a sphere cell')
    return best[1], best[2]


def canonical(pieces: list[np.ndarray]):
    """Share one fit for congruent links, including mirrored geometry."""
    points = np.unique(np.vstack(pieces), axis=0)
    origin = points.mean(0)
    _, _, basis = np.linalg.svd(points - origin, full_matrices=False)
    best = None
    for signs in itertools.product((-1, 1), repeat=3):
        frame = basis * np.array(signs)[:, None]
        keys = []
        for p in pieces:
            q = np.rint(((p - origin) @ frame.T) * 1e10).astype(np.int64)
            q = q[np.lexsort(q.T[::-1])]
            keys.append(tuple(q.ravel()))
        key = tuple(sorted(keys))
        if best is None or key < best[0]:
            best = key, frame
    key, frame = best
    return [np.array(k).reshape(-1, 3) / 1e10 for k in key], origin, frame, key


def fit(pieces: list[np.ndarray], budgets: tuple[int, ...]) -> dict[int, list[Cell]]:
    cells = [cell(pieces)]
    directions = np.vstack((trimesh.creation.icosphere(subdivisions=2).vertices, np.eye(3), -np.eye(3)))
    supports = (np.vstack(pieces) @ directions.T).max(0)
    outputs = {}
    while len(cells) <= max(budgets):
        if len(cells) in budgets:
            outputs[len(cells)] = list(cells)
        if len(cells) == max(budgets):
            break
        index = max(range(len(cells)), key=lambda i: (
            float((cells[i].center @ directions.T + cells[i].radius - supports).max()), cells[i].radius))
        parent = cells.pop(index)
        left, right = best_split(parent, directions, supports)
        old_volume = sum(ConvexHull(p).volume for p in parent.pieces)
        new_volume = sum(ConvexHull(p).volume for p in left.pieces + right.pieces)
        if not np.isclose(old_volume, new_volume, rtol=1e-8, atol=1e-15):
            raise ValueError('Cell subdivision did not preserve volume')
        cells.extend((left, right))
    return outputs


def gap(points, centers, radii):
    return np.concatenate([(np.linalg.norm(p[:, None] - centers, axis=2) - radii).min(1)
                           for p in np.array_split(points, max(1, len(points) // 2048))])


def support(points, directions):
    return np.maximum.reduce([(p @ directions.T).max(0)
                              for p in np.array_split(points, max(1, len(points) // 4096))])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--budgets', type=int, nargs='+', default=[16, 24, 32, 48, 64])
    parser.add_argument('--links', nargs='+')
    args = parser.parse_args()
    if any(budget < 1 for budget in args.budgets):
        parser.error('Sphere budgets must be positive integers')
    args.output.mkdir(parents=True, exist_ok=True)
    parts, transforms, raw, current, model, provenance = load_geometry(args.source)
    directions = np.vstack((trimesh.creation.icosphere(subdivisions=3).vertices, np.eye(3), -np.eye(3)))
    padding = json.loads((args.source/'fitting.json').read_text())['padding_m']
    arrays, certificates, fit_cache = {}, {}, {}
    report = {'production_configuration_changed': False, 'source': str(args.source.resolve()),
        'geometry_sha256': provenance['validation_geometry_sha256'], 'source_cache': provenance['full_cache'],
        'padding_m': padding, 'direction_samples': len(directions), 'volume_samples_per_link': 65536,
        'method': 'Component hulls split directly to reduce worst directional overreach; center-optimized enclosing balls; no ball-to-ball merging.',
        'links': {}}
    for name in current:
        if args.links and name not in args.links:
            continue
        start = time.monotonic()
        pieces, originals, surface_samples = [], [], []
        for part in parts:
            if part.owner != name or part.filename == 'primitive':
                continue
            transform = np.linalg.inv(transforms[name]) @ part.transform
            v = moved(part.vertices, transform)
            pieces += [component.vertices[ConvexHull(component.vertices).vertices]
                       for component in components(Mesh(v, part.faces))]
            old = moved(part.original_vertices, transform)
            originals.append(old)
            # Fixed barycentric face samples complement the cell volume proof.
            triangle = old[part.original_faces]
            for weights in ((1/3, 1/3, 1/3), (.6, .2, .2), (.2, .6, .2), (.2, .2, .6)):
                surface_samples.append(np.einsum('ijk,j->ik', triangle, weights))
        original = np.vstack(originals)
        samples = np.vstack([original, *surface_samples])
        normalized, origin, frame, key = canonical(pieces)
        shared = key in fit_cache
        if not shared:
            fit_cache[key] = fit(normalized, tuple(sorted(args.budgets)))
        candidates = {'current16': current[name]}
        for budget, cells in fit_cache[key].items():
            c = np.array([p.center for p in cells]) @ frame + origin
            r = np.array([p.radius for p in cells]) + padding + 2e-10
            label = f'direct{budget}'
            candidates[label] = c, r
            vertices = np.vstack([np.vstack(p.pieces) @ frame + origin for p in cells])
            owners = np.concatenate([np.full(sum(len(v) for v in p.pieces), i) for i, p in enumerate(cells)])
            residual = np.max(np.linalg.norm(vertices - c[owners], axis=1) - r[owners])
            assert residual <= -padding + 1e-9
            certificates[f'{name}__{label}__vertices'] = vertices
            certificates[f'{name}__{label}__owners'] = owners
        low = np.minimum.reduce([(c-r[:, None]).min(0) for c, r in candidates.values()])
        high = np.maximum.reduce([(c+r[:, None]).max(0) for c, r in candidates.values()])
        points = qmc.Sobol(3, scramble=True, seed=2410).random_base2(16)*(high-low)+low
        mesh_inside = np.zeros(len(points), bool)
        for p in pieces:
            equations = np.unique(np.round(ConvexHull(p).equations, 12), axis=0)
            for offset in range(0, len(points), 2048):
                chunk = points[offset:offset+2048]
                mesh_inside[offset:offset+len(chunk)] |= (chunk @ equations[:, :3].T + equations[:, 3] <= 1e-10).all(1)
        cad_support = support(original, directions)
        row = {'shared_canonical_fit': shared, 'component_hulls': len(pieces), 'coverage_samples': len(samples), 'variants': {}}
        for label, (c, r) in candidates.items():
            arrays[f'{name}__{label}__centers'] = c
            arrays[f'{name}__{label}__radii'] = r
            coverage = gap(samples, c, r)
            occupied = gap(points, c, r) <= 0
            excess = (c @ directions.T + r[:, None]).max(0) - cad_support
            row['variants'][label] = {'count': len(r), 'max_directional_cad_overreach_mm': float(excess.max()*1000),
                'p95_directional_cad_overreach_mm': float(np.percentile(excess,95)*1000),
                'max_radius_mm': float(r.max()*1000), 'uncovered_samples': int((coverage > 1e-9).sum()),
                'max_sample_gap_m': float(coverage.max()),
                'union_volume_cm3_estimate': float(occupied.mean()*np.prod(high-low)*1e6),
                'sphere_to_mesh_volume_ratio_estimate': float(occupied.sum()/mesh_inside.sum())}
            assert row['variants'][label]['uncovered_samples'] == 0, (name, label)
        row['fit_and_audit_seconds'] = time.monotonic()-start
        report['links'][name] = row
        report['optimizer_status'] = dict(OPTIMIZER_STATUS)
        report['geometry_status'] = dict(GEOMETRY_STATUS)
        print(name, {k:round(v['max_directional_cad_overreach_mm'],1) for k,v in row['variants'].items()},
              f'{row["fit_and_audit_seconds"]:.1f}s', flush=True)
        (args.output/'comparison.json').write_text(json.dumps(report, indent=2)+'\n')
        np.savez_compressed(args.output/'fits.npz', **arrays)
        np.savez_compressed(args.output/'certificates.npz', **certificates)
    report['script_sha256'] = hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
    (args.output/'comparison.json').write_text(json.dumps(report, indent=2)+'\n')


if __name__ == '__main__':
    main()
