"""Remove redundant offline sphere candidates with a solid-volume certificate.

Convex source cells are tetrahedralized and subdivided until each tetrahedron
fits in one retained sphere, with the original padding. A failed or exhausted
certificate keeps the sphere. Centers and radii are never enlarged or moved.
"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import time

import numpy as np
from scipy.spatial import ConvexHull

from compare_sphere_fits import canonical, fit, gap, support
from render_collision_visualization import load_geometry, moved
from optimize_arm_collision_meshes import Mesh, components
import trimesh


def tetra_volume(tetrahedra: np.ndarray) -> np.ndarray:
    delta = tetrahedra[:, 1:] - tetrahedra[:, :1]
    return np.abs(np.linalg.det(delta)) / 6


def certify_cell(vertices: np.ndarray, centers: np.ndarray, radii: np.ndarray,
                 padding: float, max_nodes: int = 32768,
                 failure_points: list[np.ndarray] | None = None):
    """Return covering tetrahedra and sphere indices, or no certificate.

    All coordinates and padding are metres. A sphere containing the four
    tetrahedron vertices contains its entire convex volume. Longest-edge
    bisection preserves the parent volume exactly apart from rounding.
    Optional failure points collect uncovered locations across the source cell,
    not budget failures. Completing this pass avoids refitting against only
    the first near-boundary gap while larger gaps remain unexplored.
    """
    hull = ConvexHull(vertices)
    vertices = vertices[hull.vertices]
    hull = ConvexHull(vertices)
    effective = radii - padding
    if np.any(effective <= 0):
        raise ValueError('Padding must be smaller than every sphere radius')
    residual = np.linalg.norm(vertices[:, None] - centers, axis=2) - effective
    if np.any(residual.min(1) > 1e-10):
        if failure_points is not None:
            failure_points.extend(vertices[residual.min(1) > 1e-10])
        return None
    origin = vertices.mean(0)
    roots = np.concatenate((np.broadcast_to(origin, (len(hull.simplices), 1, 3)), vertices[hull.simplices]), axis=1)
    single = np.flatnonzero(residual.max(0) <= 1e-10)
    if len(single):
        return roots, np.full(len(roots), single[0], dtype=int)
    stack = list(roots)
    leaves, owners = [], []
    uncovered = False
    examined = 0
    a, b = np.triu_indices(4, 1)
    while stack:
        tetra = stack.pop()
        examined += 1
        if examined > max_nodes:
            return None
        residual = np.linalg.norm(tetra[:, None] - centers, axis=2) - effective
        single = np.flatnonzero(residual.max(0) <= 1e-10)
        if len(single):
            leaves.append(tetra)
            owners.append(int(single[0]))
            continue
        if np.any(residual.min(1) > 1e-10):
            if failure_points is None:
                return None
            failure_points.extend(tetra[residual.min(1) > 1e-10])
            uncovered = True
            continue
        longest = int(np.argmax(np.linalg.norm(tetra[a] - tetra[b], axis=1)))
        i, j = int(a[longest]), int(b[longest])
        midpoint = (tetra[i] + tetra[j]) / 2
        if np.any((np.linalg.norm(midpoint-centers, axis=1)-effective) <= 1e-10):
            left, right = tetra.copy(), tetra.copy()
            left[i], right[j] = midpoint, midpoint
            stack.extend((left, right))
        else:
            if failure_points is None:
                return None
            failure_points.append(midpoint)
            uncovered = True
    if uncovered:
        return None
    result = np.array(leaves)
    if not np.isclose(tetra_volume(result).sum(), hull.volume, rtol=1e-8, atol=1e-15):
        raise ValueError('Tetrahedron certificate lost source volume')
    return result, np.array(owners)


def certify_model(cells, centers, radii, padding):
    results = []
    for vertices in cells:
        certificate = certify_cell(vertices, centers, radii, padding)
        if certificate is None:
            return None
        results.append(certificate)
    return results


def prune(cells, centers, radii, padding):
    points = np.unique(np.vstack(cells), axis=0)
    covers = np.linalg.norm(points[:, None] - centers, axis=2) <= radii - padding + 1e-10
    orders = [sorted(range(len(radii)), key=lambda i: (radii[i], i)),
              sorted(range(len(radii)), key=lambda i: (int(covers[:, i].sum()), radii[i], i)),
              sorted(range(len(radii)), key=lambda i: (-radii[i], i))]
    best = None
    for order in orders:
        kept = list(range(len(radii)))
        for removed in order:
            candidate = [i for i in kept if i != removed]
            if not candidate or not covers[:, candidate].any(1).all():
                continue
            if certify_model(cells, centers[candidate], radii[candidate], padding) is not None:
                kept = candidate
        score = len(kept), float((radii[kept]**3).sum()), tuple(kept)
        if best is None or score < best[0]:
            best = score, kept
    kept = best[1]
    return kept, certify_model(cells, centers[kept], radii[kept], padding)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--comparison', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--variant', default='direct16', choices=['direct16', 'direct32'])
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    previous = json.loads(args.comparison.read_text())
    source = Path(previous['source'])
    parts, transforms, raw, active, model, provenance = load_geometry(source)
    fits = np.load(args.comparison.parent / 'fits.npz', allow_pickle=False)
    padding = previous['padding_m']
    directions = np.vstack((trimesh.creation.icosphere(subdivisions=3).vertices, np.eye(3), -np.eye(3)))
    arrays, proof_arrays, fit_cache = {}, {}, {}
    report = {'source_comparison': str(args.comparison.resolve()), 'variant': args.variant,
        'source': str(source), 'geometry_sha256': previous['geometry_sha256'],
        'padding_m': padding, 'production_configuration_changed': False,
        'method': 'Delete fixed spheres only; certify every source cell volume with covered tetrahedra.',
        'limitations': ['Greedy removal over three deterministic orders; not a proof of minimum count.',
                        'Source-cell coverage is retained; the old sphere union outside robot geometry may shrink.',
                        'Base/primitives remain outside proposal spheres. No whole-planner or motion benchmark.'],
        'links': {}}
    for name in previous['links']:
        start = time.monotonic()
        c, r = fits[f'{name}__{args.variant}__centers'], fits[f'{name}__{args.variant}__radii']
        source_pieces = []
        for part in parts:
            if part.owner == name and part.filename != 'primitive':
                transform = np.linalg.inv(transforms[name]) @ part.transform
                vertices = moved(part.vertices, transform)
                source_pieces.extend(component.vertices[ConvexHull(component.vertices).vertices]
                    for component in components(Mesh(vertices, part.faces)))
        normalized, origin, frame, key = canonical(source_pieces)
        if key not in fit_cache:
            fit_cache[key] = fit(normalized, (len(r),))[len(r)]
        fitted = fit_cache[key]
        rebuilt_centers = np.array([cell.center for cell in fitted]) @ frame + origin
        rebuilt_radii = np.array([cell.radius for cell in fitted]) + padding + 2e-10
        np.testing.assert_allclose(rebuilt_centers, c, atol=1e-12, rtol=0)
        np.testing.assert_allclose(rebuilt_radii, r, atol=1e-12, rtol=0)
        # Keep disconnected solids separate: a convex hull spanning several
        # components would needlessly require coverage of the air between them.
        cells = [piece @ frame + origin for cell in fitted for piece in cell.pieces]
        kept, proofs = prune(cells, c, r, padding)
        assert proofs is not None
        nc, nr = c[kept], r[kept]
        original, samples = [], []
        for part in parts:
            if part.owner != name or part.filename == 'primitive':
                continue
            transform = np.linalg.inv(transforms[name]) @ part.transform
            old = moved(part.original_vertices, transform)
            original.append(old)
            triangles = old[part.original_faces]
            for weights in ((1/3, 1/3, 1/3), (.6,.2,.2), (.2,.6,.2), (.2,.2,.6)):
                samples.append(np.einsum('ijk,j->ik', triangles, weights))
        original = np.vstack(original)
        samples = np.vstack([original, *samples])
        coverage = gap(samples, nc, nr)
        assert coverage.max() <= 1e-9
        source_support = (c @ directions.T + r[:, None]).max(0)
        new_support = (nc @ directions.T + nr[:, None]).max(0)
        assert (new_support - source_support).max() <= 1e-12
        tetrahedra = np.concatenate([p[0] for p in proofs])
        indices = np.concatenate([p[1] for p in proofs])
        residual = (np.linalg.norm(tetrahedra-nc[indices,None],axis=2)-nr[indices,None]).max()
        assert residual <= -padding + 1e-10
        arrays[f'{name}__centers'], arrays[f'{name}__radii'] = nc, nr
        proof_arrays[f'{name}__tetrahedra'] = tetrahedra
        proof_arrays[f'{name}__sphere_indices'] = indices
        proof_arrays[f'{name}__cell_indices'] = np.concatenate([np.full(len(p[0]), i) for i,p in enumerate(proofs)])
        proof_arrays[f'{name}__source_vertices'] = np.vstack(cells)
        proof_arrays[f'{name}__source_piece_indices'] = np.concatenate([np.full(len(p), i) for i,p in enumerate(cells)])
        report['links'][name] = {'before': len(r), 'after': len(nr), 'kept_indices': kept,
            'coverage_samples': len(samples), 'uncovered_samples': int((coverage > 1e-9).sum()),
            'max_sample_gap_m': float(coverage.max()), 'certificate_tetrahedra': len(tetrahedra),
            'worst_tetrahedron_enclosure_residual_m': float(residual),
            'before_directional_overreach_mm': previous['links'][name]['variants'][args.variant]['max_directional_cad_overreach_mm'],
            'after_directional_overreach_mm': float((new_support-support(original,directions)).max()*1000),
            'seconds': time.monotonic()-start}
        print(name, len(r), '->',len(nr),'certificate tets',len(tetrahedra),f'{time.monotonic()-start:.2f}s',flush=True)
        (args.output/'pruning.json').write_text(json.dumps(report,indent=2)+'\n')
        np.savez_compressed(args.output/'pruned.npz',**arrays)
        np.savez_compressed(args.output/'certificates.npz',**proof_arrays)
    report['totals'] = {key: sum(row[key] for row in report['links'].values())
                        for key in ('before','after','coverage_samples','uncovered_samples','certificate_tetrahedra')}
    report['input_sha256'] = {str(path): hashlib.sha256(path.read_bytes()).hexdigest() for path in
        [args.comparison, args.comparison.parent/'fits.npz', args.comparison.parent/'certificates.npz',
         Path(__file__), Path(__file__).with_name('compare_sphere_fits.py')]}
    (args.output/'pruning.json').write_text(json.dumps(report,indent=2)+'\n')
    print('TOTAL',report['totals'],flush=True)


if __name__ == '__main__':
    main()
