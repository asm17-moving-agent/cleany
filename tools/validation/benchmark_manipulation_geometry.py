"""Compare CPU manipulation geometry against a saved source tree without motion."""
from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import platform
import statistics
import sys
import time
import xml.etree.ElementTree as ET

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
MODULES = {
    'geometry': 'ros2_ws/src/cleany_perception/cleany_perception/core/geometry.py',
    'fk': 'ros2_ws/src/cleany_skill_executor/cleany_skill_executor/core/urdf_fk.py',
    'selector': 'ros2_ws/src/cleany_grasping/cleany_grasping/core/selector.py',
}


def load_module(name: str, path: Path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


def compare_times(before, after, repeats: int) -> dict:
    before()
    after()
    samples = ([], [])
    for repeat in range(repeats):
        for index in ((0, 1) if repeat % 2 == 0 else (1, 0)):
            started = time.perf_counter()
            (before, after)[index]()
            samples[index].append(1000*(time.perf_counter()-started))
    old, new = (statistics.median(values) for values in samples)
    return {'before_median_ms': old, 'after_median_ms': new,
            'speedup': old/new, 'before_samples_ms': samples[0],
            'after_samples_ms': samples[1]}


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--baseline-root', type=Path, required=True)
    parser.add_argument('--urdf', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--repeats', type=int, default=7)
    args = parser.parse_args()
    if args.repeats < 1:
        parser.error('--repeats must be positive')
    for package in ('cleany_perception', 'cleany_skill_executor', 'cleany_grasping'):
        sys.path.insert(0, str(ROOT/'ros2_ws/src'/package))
    from cleany_perception.core.models import CameraIntrinsics
    from cleany_grasping.core.models import PointCloud, RawGrasp

    old = {key: load_module('baseline_'+key, args.baseline_root/path)
           for key, path in MODULES.items()}
    new = {key: load_module('current_'+key, ROOT/path)
           for key, path in MODULES.items()}
    results = {
        'environment': {'platform': platform.platform(), 'python': platform.python_version(),
                        'numpy': np.__version__,
                        'OPENBLAS_NUM_THREADS': os.getenv('OPENBLAS_NUM_THREADS')},
        'input': {'seed': 42, 'urdf': str(args.urdf),
                  'urdf_sha256': hashlib.sha256(args.urdf.read_bytes()).hexdigest(),
                  'baseline_root': str(args.baseline_root), 'repeats': args.repeats},
        'source_sha256': {
            key: {label: hashlib.sha256((base/path).read_bytes()).hexdigest()
                  for label, base in (('before', args.baseline_root), ('after', ROOT))}
            for key, path in MODULES.items()},
        'checks': {}, 'benchmarks': {}, 'robot_motion': False,
    }
    rng = np.random.default_rng(42)
    for height, width in ((480, 640), (720, 1280)):
        intrinsics = CameraIntrinsics(width, height, 600., 600., (width-1)/2, (height-1)/2)
        depth = rng.uniform(.25, 2., (height, width)).astype(np.float32)
        depth[::17, ::11] = np.nan
        depth[1::31, ::23] = np.inf
        mask = rng.random(depth.shape) > .15
        for stride in (1, 4):
            params = (depth, intrinsics, mask, .25, 2., stride)
            before = lambda: old['geometry'].deproject_masked_depth(*params)
            after = lambda: new['geometry'].deproject_masked_depth(*params)
            np.testing.assert_array_equal(before(), after())
            key = f'deproject_{width}x{height}_stride{stride}'
            results['checks'][key] = {'exact': True, 'points': len(after())}
            results['benchmarks'][key] = compare_times(before, after, args.repeats)

    xy = rng.uniform(-.3, .3, (8000, 2))
    plane_points = np.column_stack((xy, .8+rng.normal(0., .0001, len(xy))))
    for outliers in (0, 400):
        points = plane_points.copy()
        if outliers:
            points[:outliers, 2] += .05
        params = (points, 200, .006, 100, .35)
        before = lambda: old['geometry'].fit_plane_ransac(*params)
        after = lambda: new['geometry'].fit_plane_ransac(*params)
        left, right = before(), after()
        np.testing.assert_array_equal(left.normal, right.normal)
        assert left.offset == right.offset and left.inlier_count == right.inlier_count
        key = f'plane_8000_outliers{outliers}'
        results['checks'][key] = {'exact': True, 'inliers': right.inlier_count}
        results['benchmarks'][key] = compare_times(before, after, args.repeats)

    target = PointCloud(np.array([[-.1]*3, [.1]*3]), np.zeros((2, 3)))
    for count in (12, 1000):
        positions = rng.uniform(-.08, .08, (count, 3))
        angles = rng.uniform(-np.pi, np.pi, count)
        candidates = tuple(RawGrasp(
            np.array(((np.cos(a), -np.sin(a), 0.),
                      (np.sin(a), np.cos(a), 0.), (0., 0., 1.))),
            p, .04, 0., float(score))
            for p, a, score in zip(positions, angles, rng.random(count)))
        before = lambda: old['selector'].rank_grasps(candidates, target)
        after = lambda: new['selector'].rank_grasps(candidates, target)
        left, right = before(), after()
        assert len(left) == len(right)
        for a, b in zip(left, right):
            for field in ('rotation', 'translation', 'approach_direction',
                          'required_opening_m', 'depth_m', 'score'):
                np.testing.assert_array_equal(getattr(a, field), getattr(b, field))
        key = f'rank_grasps_{count}'
        results['checks'][key] = {'exact': True, 'retained': len(right)}
        results['benchmarks'][key] = compare_times(before, after, args.repeats)

    xml = args.urdf.read_text()
    bounds = {}
    for joint in ET.fromstring(xml).findall('joint'):
        if joint.get('type') == 'fixed':
            continue
        limit = joint.find('limit')
        low = float(limit.get('lower', -np.pi)) if limit is not None else -np.pi
        high = float(limit.get('upper', np.pi)) if limit is not None else np.pi
        bounds[joint.get('name')] = (low, high)
    for arm in ('left', 'right'):
        reference = old['fk'].UrdfChain(xml, 'base_link', f'{arm}_grasp_tcp')
        current = new['fk'].UrdfChain(xml, 'base_link', f'{arm}_grasp_tcp')
        states = [{name: float(rng.uniform(*limits)) for name, limits in bounds.items()}
                  for _ in range(2000)]
        before = lambda: [reference.pose(state) for state in states]
        after = lambda: [current.pose(state) for state in states]
        left, right = before(), after()
        maximum_error = max(float(np.max(np.abs(a-b)))
                            for lhs, rhs in zip(left, right) for a, b in zip(lhs, rhs))
        for lhs, rhs in zip(left, right):
            for a, b in zip(lhs, rhs):
                np.testing.assert_allclose(a, b, atol=2e-14, rtol=0.)
        key = f'fk_{arm}_2000'
        results['checks'][key] = {'maximum_absolute_error': maximum_error}
        results['benchmarks'][key] = compare_times(before, after, args.repeats)

    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(results, indent=2, allow_nan=False)+'\n')
    print(json.dumps({'checks': results['checks'], 'benchmarks': {
        key: {field: value for field, value in result.items() if not field.endswith('_samples_ms')}
        for key, result in results['benchmarks'].items()}}, indent=2))


if __name__ == '__main__':
    main()
