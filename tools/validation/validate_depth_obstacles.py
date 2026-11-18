#!/usr/bin/env python3
"""Fit a stationary recorded floor and validate obstacle filtering on held-out frames."""

import argparse
import json
from dataclasses import asdict
from pathlib import Path

import numpy as np

from cleany_navigation.body_geometry import BodyBox
from cleany_navigation.depth_obstacles import ObstacleFilter, fit_floor, filter_obstacles
from compare_depth_lidar_wall import camera_pose


def load_frame(data: dict, depth: np.ndarray, index: int, mount: dict, stride: int = 3):
    k = np.asarray(data['camera_info']['k']).reshape(3, 3)
    vv, uu = np.mgrid[0:depth.shape[1]:stride, 0:depth.shape[2]:stride]
    z = depth[index, ::stride, ::stride].ravel()*.001
    valid = (z > .25) & (z < 4)
    p = np.column_stack(((uu.ravel()-k[0, 2])*z/k[0, 0], (vv.ravel()-k[1, 2])*z/k[1, 1], z))[valid]
    r, origin = camera_pose(data, data['depth_times'][index]['stamp'], mount)
    return p @ r.T+origin, origin


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--capture', type=Path, required=True)
    parser.add_argument('--mount', type=Path, required=True)
    parser.add_argument('--boxes', type=Path, required=True)
    args = parser.parse_args()
    data = json.loads((args.capture/'capture.json').read_text())
    mount = json.loads(args.mount.read_text())
    depth = np.load(args.capture/'depth.npz')['depth_mm']
    boxes = [BodyBox(b['name'], b['center'], np.eye(3), b['half_size']) for b in json.loads(args.boxes.read_text())['boxes']]
    frames = []
    for i in range(len(depth)):
        try:
            p, origin = load_frame(data, depth, i, mount)
            fit = fit_floor(p)
            frames.append((i, p, origin, fit))
        except ValueError:
            continue
    if len(frames) < 6:
        raise RuntimeError('Need at least six supported floor frames')
    train = frames[:len(frames)//2]
    normal = np.median([x[3]['normal'] for x in train], axis=0)
    normal /= np.linalg.norm(normal)
    offset = float(np.median([x[3]['offset'] for x in train]))
    config = ObstacleFilter(tuple(normal), offset)
    (args.capture/'obstacle-filter.json').write_text(json.dumps(asdict(config), indent=2))
    reports = []
    for i, p, origin, fit in frames:
        output, counts = filter_obstacles(p, origin, boxes, config)
        height = p @ normal-offset
        reports.append({'frame': i, 'split': 'train' if i in [x[0] for x in train] else 'held_out',
                        'floor_fit': fit, 'counts': counts,
                        'retained_floor_band_points': int((abs(output @ normal-offset)<.03).sum())})
    report = {'config': asdict(config), 'frames': reports,
              'boundary': 'Stationary floor fit in CAD base coordinates. Independent ground truth and full robot body posture not measured. Filtering is preview only, no costmap or stop authority.'}
    (args.capture/'obstacle-validation.json').write_text(json.dumps(report, indent=2))
    i, p, origin, fit = frames[-1]
    output, counts = filter_obstacles(p, origin, boxes, config)
    np.savez_compressed(args.capture/'obstacle-preview.npz', raw=p, obstacles=output, origin=origin)
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(1, 2, figsize=(14, 6), layout='constrained')
    for ax, points, title in zip(axes, (p, output), ('Raw measured depth', 'Filtered obstacle hits')):
        heights = points @ normal-offset
        im = ax.scatter(points[:, 0], heights, c=points[:, 1], s=2, cmap='coolwarm', vmin=-1, vmax=1)
        ax.axhline(0, color='black', linestyle='--')
        ax.set(xlim=(0, 3), ylim=(-.2, 1.6), xlabel='Robot forward X (m)', ylabel='Height above fitted floor (m)', title=title)
    fig.colorbar(im, ax=axes, label='Robot left Y (m)')
    fig.savefig(args.capture/'obstacle-preview.png', dpi=150)
    print(json.dumps({'floor': {'normal': normal.tolist(), 'offset': offset}, 'usable_frames': len(frames), 'last_counts': counts}, indent=2))


if __name__ == '__main__':
    main()
