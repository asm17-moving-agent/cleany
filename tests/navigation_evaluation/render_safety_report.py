#!/usr/bin/env python3
"""Render the measured SCRUM-306 sensor, stop and Nav2 avoidance evidence."""

import argparse
import csv
import json
import math
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon, Rectangle
import numpy as np
from PIL import Image
import yaml


def rows(path):
    with path.open() as stream:
        return list(csv.DictReader(stream))


def column(data, name):
    return np.array([float(row[name]) for row in data])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--sensors', type=Path, required=True)
    parser.add_argument('--monitor', type=Path, required=True)
    parser.add_argument('--avoid', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    sensor = json.loads((args.sensors/'result.json').read_text())
    monitor = json.loads((args.monitor/'result.json').read_text())
    avoid = json.loads((args.avoid/'result.json').read_text())
    scenario = json.loads((args.avoid/'scenario.json').read_text())
    events = json.loads((args.avoid/'events.json').read_text())
    start = next(e['sim_s'] for e in events if e['phase']=='nav2_avoid' and 'goal' in e)
    end = next(e['sim_s'] for e in events if e['phase']=='nav2_settled')
    avoid['duration_sim_s'] = end-start
    avoid['final_goal_error_m'] = math.dist(avoid['final_truth'][:2], scenario['evaluation']['avoid']['goal'][:2])
    summary = {'sensors': {k:v for k,v in sensor.items() if k != 'feedback'},
               'monitor': {k:v for k,v in monitor.items() if k != 'feedback'},
               'avoid': {k:v for k,v in avoid.items() if k != 'feedback'},
               'runs': {'sensors':str(args.sensors.resolve()), 'monitor':str(args.monitor.resolve()), 'avoid':str(args.avoid.resolve())}}
    (args.output/'metrics.json').write_text(json.dumps(summary, indent=2))

    data = [r for r in rows(args.monitor/'samples.csv') if r['phase'].startswith('monitor_')]
    t = column(data, 'sim_s')
    t -= t[0]
    fig, axes = plt.subplots(1, 2, figsize=(13, 4.5), constrained_layout=True)
    cases = ['lidar_only', 'elevated', 'low']
    x = np.arange(len(cases))
    for i, stage in enumerate(['before', 'present', 'removed']):
        values = [sensor[c][stage]['lethal_costmap_cells'] for c in cases]
        bars = axes[0].bar(x+(i-1)*0.24, values, width=0.24, label=stage)
        axes[0].bar_label(bars, padding=2, fontsize=9)
    axes[0].set_xticks(x, ['LiDAR only\n1.4 m tall box', 'Depth only\nbox above scan plane', 'Blind spot\n0.15 m low box'])
    axes[0].set_ylabel('Lethal local-costmap cells in fixture ROI')
    axes[0].set_title('Sensor contribution and removal')
    axes[0].legend()
    for label, prefix in [('Monitor input', 'raw'), ('Monitor output', 'safe'), ('Gazebo actual speed', 'gt')]:
        axes[1].plot(t, np.hypot(column(data, prefix+'_vx'), column(data, prefix+'_vy')), label=label)
    action = column(data, 'action')
    axes[1].fill_between(t, 0, 0.17, where=action==1, alpha=0.12, color='red', label='StopZone / stop state')
    axes[1].set(xlabel='Simulation seconds since stimulus', ylabel='Planar speed (m/s)',
                title=f"Slowdown ratio {monitor['slowdown_ratio_median']:.2f}; actual stop {'PASS' if monitor['checks']['actual_stop'] else 'FAIL'}")
    axes[1].legend(fontsize=9)
    for ax in axes:
        ax.grid(axis='y', alpha=0.2)
    fig.suptitle('SCRUM-306 | 30 cm LiDAR | L0 wheel odometry | Gazebo Jazzy/Harmonic')
    fig.savefig(args.output/'sensors_and_stop.png', dpi=160)
    plt.close(fig)

    data = [r for r in rows(args.avoid/'samples.csv') if r['phase']=='nav2_avoid']
    gt = np.column_stack((column(data, 'gt_x'), column(data, 'gt_y')))
    tf = np.column_stack((column(data, 'tf_x'), column(data, 'tf_y')))
    t = column(data, 'sim_s')
    t -= t[0]
    metadata = yaml.safe_load((args.avoid/'map.yaml').read_text())
    image = np.array(Image.open(args.avoid/metadata['image']))
    ox, oy, _ = metadata['origin']
    resolution = metadata['resolution']
    config = scenario['evaluation']['avoid']
    fig, axes = plt.subplots(2, 1, figsize=(11, 8), gridspec_kw={'height_ratios':[2, 1]}, constrained_layout=True)
    ax = axes[0]
    ax.imshow(image, cmap='gray', vmin=0, vmax=255, extent=[ox, ox+image.shape[1]*resolution, oy, oy+image.shape[0]*resolution])
    ax.plot(gt[:, 0], gt[:, 1], color='#008080', linewidth=2.2, label='Gazebo actual route')
    ax.plot(tf[:, 0], tf[:, 1], color='#e38a21', linewidth=1, linestyle='--', label='AMCL/TF estimate')
    ax.plot([config['fixture_x']]*2, [config['fixture_initial_y'], config['fixture_final_y']], 'r:', label='Scripted obstacle entry')
    sx, sy = scenario['fixtures']['tall']['size'][:2]
    ax.add_patch(Rectangle((config['fixture_x']-sx/2, config['fixture_final_y']-sy/2), sx, sy, color='crimson', alpha=0.6, label='Obstacle final footprint'))
    ax.plot([0, config['goal'][0]], [0, config['goal'][1]], 'kx', markersize=9, label='Start / requested goal')
    clearance = column(data, 'clearance_m')
    if np.isfinite(clearance).any():
        index = np.nanargmin(clearance)
        pose = [gt[index, 0], gt[index, 1], float(data[index]['gt_yaw'])]
        hx, hy = scenario['robot_half_size']
        p = np.array([[-hx, -hy], [hx, -hy], [hx, hy], [-hx, hy]])
        c, s = np.cos(pose[2]), np.sin(pose[2])
        p = p @ np.array([[c, s], [-s, c]]) + pose[:2]
        ax.add_patch(Polygon(p, fill=False, edgecolor='#008080', linewidth=1.4, label='Padded base at closest approach'))
    ax.set(xlim=(-0.5, 4.6), ylim=(-1.4, 1.5), xlabel='Map x (m)', ylabel='Map y (m)', aspect='equal',
           title=f"Status {avoid['status']} | {avoid['duration_sim_s']:.1f} s | recoveries {avoid['max_recoveries']} | minimum padded clearance {100*avoid['min_padded_footprint_clearance_m']:.1f} cm")
    ax.legend(loc='upper right', fontsize=8)
    axes[1].plot(t, clearance, color='#008080')
    axes[1].axhline(scenario['evaluation']['thresholds']['min_clearance_m'], color='crimson', linestyle='--', label='Evaluation threshold')
    axes[1].set(xlabel='Simulation seconds since goal', ylabel='Padded footprint clearance (m)', ylim=(0, min(2, np.nanmax(clearance)+0.05)))
    axes[1].legend()
    axes[1].grid(alpha=0.2)
    fig.suptitle('Measured Nav2 dynamic-obstacle avoidance | fixed map/ground-truth reference')
    fig.savefig(args.output/'nav2_avoidance.png', dpi=160)
    plt.close(fig)


if __name__ == '__main__':
    main()
