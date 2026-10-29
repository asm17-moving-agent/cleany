"""Independently replay saved tetrahedra and joint-frame coverage certificates."""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial import ConvexHull
import trimesh

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT/'ros2_ws/src/cleany_skill_executor'))
from cleany_skill_executor.collision_sphere_model import collision_signatures, validate_joint_covers


def primitive_coverage(link: ET.Element, selector: dict, sphere: dict, padding: float):
    """Check continuous primitive solids analytically, independent of tetrahedra."""
    results, witnesses = [], []
    for collision in link.findall('collision'):
        if selector['collision_name_contains'] not in collision.get('name', ''):
            continue
        origin = collision.find('origin')
        xyz = np.fromstring(origin.get('xyz', '0 0 0') if origin is not None else '0 0 0', sep=' ')
        rpy = np.fromstring(origin.get('rpy', '0 0 0') if origin is not None else '0 0 0', sep=' ')
        rotation = trimesh.transformations.euler_matrix(*rpy)[:3, :3]
        local = (np.array(sphere['center'])-xyz) @ rotation
        box, cylinder = collision.find('geometry/box'), collision.find('geometry/cylinder')
        if box is not None:
            size = np.fromstring(box.get('size'), sep=' ')
            farthest = np.linalg.norm(np.abs(local)+size/2)
            witnesses.append(trimesh.creation.box(extents=size).vertices @ rotation.T+xyz)
        else:
            assert cylinder is not None
            farthest = np.hypot(np.linalg.norm(local[:2])+float(cylinder.get('radius')),
                               abs(local[2])+float(cylinder.get('length'))/2)
        margin = float(sphere['radius']-farthest)
        assert margin >= padding-1e-10, collision.get('name')
        results.append({'collision': collision.get('name'), 'margin_m': margin})
    assert len(results) == selector['expected_components']
    return results, np.concatenate(witnesses)


def primitive_union_coverage(link: ET.Element, selector: dict, spheres: list[dict], padding: float):
    """Certify whole primitive solids, including a continuous cylinder enclosure."""
    from fit_joint_spheres import primitive_components
    from prune_sphere_fit import certify_cell

    centers = np.array([s['center'] for s in spheres])
    radii = np.array([s['radius'] for s in spheres])
    checks = []
    for part in primitive_components(link, selector):
        proof = certify_cell(part, centers, radii, padding)
        assert proof is not None, 'Primitive solid is not covered'
        residual = np.linalg.norm(proof[0]-centers[proof[1], None], axis=2)-radii[proof[1], None]
        checks.append({'tetrahedra': len(proof[0]), 'minimum_margin_m': float(-residual.max())})
    return checks


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--asset', type=Path, required=True)
    parser.add_argument('--certificate', type=Path, required=True)
    parser.add_argument('--urdf', type=Path, required=True)
    parser.add_argument('--fitting-report', type=Path, help='Verify saved support-point lower bounds')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    data = json.loads(args.asset.read_text())
    assert hashlib.sha256(args.certificate.read_bytes()).hexdigest() == data['coverage_certificate_sha256']
    xml = args.urdf.read_text()
    validate_joint_covers(xml, data)
    signatures = collision_signatures(xml, set(data['links']),
        resolve=lambda uri: ROOT/'ros2_ws/src'/uri.removeprefix('package://'))
    proofs = np.load(args.certificate, allow_pickle=False)
    results = {}
    for name, entry in data['links'].items():
        assert signatures[name] == entry['collision_sha256']
        balls = list(entry['spheres'])
        for cover in data['joint_covers']:
            if cover['child'] == name:
                balls.append({'center': [0., 0., 0.], 'radius': cover['radius_m']})
        centers = np.array([b['center'] for b in balls]); radii = np.array([b['radius'] for b in balls])
        tetra = proofs[name+'__tetrahedra']; owners = proofs[name+'__sphere_indices']
        residual = np.max(np.linalg.norm(tetra-centers[owners, None],axis=2)-radii[owners, None])
        assert residual <= -data['padding_m']+1e-10
        source, pieces = proofs[name+'__source_vertices'], proofs[name+'__source_piece_indices']
        assignments = proofs[name+'__cell_indices']
        for i in np.unique(pieces):
            hull = ConvexHull(source[pieces == i]); cells = tetra[assignments == i]
            matrix = cells[:, 1:]-cells[:, :1]
            volume = np.abs(np.linalg.det(matrix)).sum()/6
            assert np.isclose(volume,hull.volume,rtol=1e-8,atol=1e-15), (name,i)
            assert (cells.reshape(-1,3) @ hull.equations[:, :3].T+hull.equations[:,3]).max() < 1e-9
        results[name] = {'tetrahedra':len(tetra),'enclosure_residual_m':float(residual)}
    bounds, housing_checks, primitive_checks, primitive_witnesses = {}, {}, {}, {}
    root = ET.fromstring(xml)
    primitive_groups = {}
    for cover in data['rigid_covers']:
        if 'primitive_selector' in cover:
            primitive_groups.setdefault((cover['link'], cover['region']), []).append(cover)
    for (name, region), covers in primitive_groups.items():
        count = len(covers)
        assert count in (1, 2) and all(c['sphere_count'] == count for c in covers)
        assert {c['part_index'] for c in covers} == set(range(count))
        selector = covers[0]['primitive_selector']
        assert all(c['primitive_selector'] == selector for c in covers)
        link = root.find(f"link[@name='{name}']")
        balls = [data['links'][name]['spheres'][c['sphere_index']] for c in covers]
        if count == 1:
            checks, witnesses = primitive_coverage(link, selector, balls[0], data['padding_m'])
            primitive_witnesses[(name, covers[0]['sphere_index'])] = witnesses
        else:
            checks = primitive_union_coverage(link, selector, balls, data['padding_m'])
        primitive_checks[f'{name}/{region}'] = checks
    if args.fitting_report:
        report = json.loads(args.fitting_report.read_text())
        from fit_joint_spheres import selected_components
        from prune_sphere_fit import certify_cell
        root = ET.fromstring(xml)
        for row in report.get('partitioned_regions', []):
            name = row['link']
            parts = selected_components(root.find(f"link[@name='{name}']"), row['selectors'])
            balls = [data['links'][name]['spheres'][i] for i in row['sphere_indices']]
            centers = np.array([b['center'] for b in balls])
            radii = np.array([b['radius'] for b in balls])
            assert len(balls) == 2
            assert all(certify_cell(part, centers, radii, data['padding_m']) is not None
                       for part in parts), name
            housing_checks[f"{name}/{row['region']}"] = {
                'link': name, 'sphere_indices': row['sphere_indices'],
                'spheres': 2, 'components': len(parts), 'complete_solid_coverage': True}
        for key, row in report['minimum_ball_bounds'].items():
            if 'points_m' not in row:
                continue
            name = row.get('link', key)
            points, weights = np.array(row['points_m']), np.array(row['weights'])
            assert (weights >= 0).all() and np.isclose(weights.sum(), 1., atol=1e-12)
            source = (primitive_witnesses[(name, row['sphere_index'])]
                      if 'primitive_selector' in row else proofs[name+'__source_vertices'])
            assert np.linalg.norm(points[:, None]-source, axis=2).min(axis=1).max() < 1e-9
            center = weights @ points
            lower = np.sqrt(weights @ np.square(points-center).sum(axis=1))
            radius = data['links'][name]['spheres'][row.get('sphere_index', 0)]['radius']
            gap = 2000*(radius-data['padding_m']-lower)
            assert gap >= -1e-8 and gap < 1e-4
            bounds[key] = {'diameter_lower_bound_mm':float(2000*(lower+data['padding_m'])),
                            'diameter_upper_bound_mm':float(2000*radius),
                            'diameter_gap_mm':float(gap)}
    args.output.write_text(json.dumps({'complete_solid_coverage':True,'joint_motion_invariant':True,
        'spheres':sum(len(e['spheres']) for e in data['links'].values()),'links':results,
        'verified_minimum_ball_bounds':bounds,
        'continuous_primitive_coverage':primitive_checks,
        'partitioned_housing_coverage':housing_checks},indent=2)+'\n')
    print('Certified',len(results),'links;',sum(r['tetrahedra'] for r in results.values()),'tetrahedra')


if __name__ == '__main__':
    main()
