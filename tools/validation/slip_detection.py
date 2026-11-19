#!/usr/bin/env python3
"""Read-only bag diagnostics for wheel/LiDAR motion disagreement, never control."""
from __future__ import annotations

import argparse
from concurrent.futures import ProcessPoolExecutor
from dataclasses import asdict, dataclass
from datetime import datetime
import hashlib
import json
import math
from pathlib import Path
import sqlite3
import sys
import time
from typing import Any

import numpy as np

MATCHER_ROOT = Path(__file__).resolve().parents[1] / 'odometry_evaluation'
sys.path.insert(0, str(MATCHER_ROOT))
from lidar_scan_matcher import LidarScanMatcher  # noqa: E402


sys.path.insert(0, str(Path(__file__).resolve().parents[2]/'ros2_ws/src/cleany_navigation'))
from cleany_navigation.slip_estimation import (Settings, wrap, compose, relative_pose, odom_delta, classify, events)


def header_time(message: Any) -> float:
    stamp = message.header.stamp
    result = stamp.sec + stamp.nanosec*1e-9
    if result <= 0:
        raise ValueError('Missing sensor header timestamp')
    return result


def extract(bag: Path, start: float, end: float, output: Path) -> dict[str, Any]:
    """Deserialize a read-only SQLite snapshot without creating a ROS node."""
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    scans, points, offsets, odom, wheels, transforms = [], [], [0], [], [], {}
    if bag.is_file():
        files = [bag]
    else:
        files = sorted(bag.glob('*.db3'), key=lambda p: int(p.stem.rsplit('_', 1)[-1]))
    if not files:
        raise ValueError('No rosbag2 SQLite files')
    frames = set()
    for file in files:
        with sqlite3.connect(f'file:{file.resolve()}?mode=ro', uri=True) as db:
            db.execute('BEGIN')
            topics = {name: (i, get_message(typ)) for i, name, typ in db.execute('select id,name,type from topics')}
            if '/tf_static' in topics:
                ident, cls = topics['/tf_static']
                for (blob,) in db.execute('select data from messages where topic_id=?', (ident,)):
                    for tr in deserialize_message(blob, cls).transforms:
                        if tr.header.frame_id == 'base_link':
                            q = tr.transform.rotation
                            if abs(q.x)+abs(q.y) > 1e-5:
                                continue
                            mount = [tr.transform.translation.x, tr.transform.translation.y,
                                math.atan2(2*q.w*q.z, 1-2*q.z*q.z)]
                            if tr.child_frame_id in transforms and not np.allclose(transforms[tr.child_frame_id], mount):
                                raise ValueError('Changing static extrinsic; analyze separately')
                            transforms[tr.child_frame_id] = mount
            for name in ['/scan_raw', '/odom', '/base/wheel_state']:
                if name not in topics:
                    continue
                ident, cls = topics[name]
                for received, blob in db.execute('select timestamp,data from messages where topic_id=? and timestamp between ? and ? order by timestamp',
                        (ident, int((start-2)*1e9), int((end+2)*1e9))):
                    m = deserialize_message(blob, cls)
                    if name == '/scan_raw':
                        t = header_time(m)+m.scan_time/2
                        if not start-1 <= t <= end+1:
                            continue
                        frames.add(m.header.frame_id)
                        r = np.array(m.ranges)
                        a = m.angle_min+np.arange(len(r))*m.angle_increment
                        valid = np.isfinite(r) & (r >= m.range_min) & (r <= m.range_max)
                        cloud = np.column_stack((r[valid]*np.cos(a[valid]), r[valid]*np.sin(a[valid])))
                        scans.append([t, m.scan_time, received/1e9-header_time(m)])
                        points.append(cloud)
                        offsets.append(offsets[-1]+len(cloud))
                    elif name == '/odom':
                        if m.header.frame_id != 'odom' or m.child_frame_id != 'base_link':
                            raise ValueError('Expected odom -> base_link pose')
                        p, q = m.pose.pose.position, m.pose.pose.orientation
                        odom.append([header_time(m), p.x, p.y, math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))])
                    else:
                        wheels.append([received/1e9, *m.commanded_rad_s, *m.velocity_rad_s])
    if len(frames) != 1 or next(iter(frames)) not in transforms:
        raise ValueError('Require a recorded planar base_link -> scan frame static transform')
    if len(scans) < 2 or len(odom) < 2:
        raise ValueError('Insufficient scan or odom data')
    scan_array = np.array(scans)
    if np.any(np.diff(scan_array[:, 0]) <= 0):
        raise ValueError('Non-monotonic scans; analyze each clock epoch separately')
    odom_array = np.array(sorted(odom))
    np.savez_compressed(output, scans=scan_array, points=np.concatenate(points), offsets=np.array(offsets),
                        odom=odom_array, wheels=np.array(wheels).reshape(-1, 9),
                        mount=np.array(transforms[next(iter(frames))]))
    return {'files': [str(f.resolve()) for f in files], 'scan_count': len(scans), 'odom_count': len(odom),
            'scan_frame': next(iter(frames)), 'mount': transforms[next(iter(frames))],
            'scan_duration_p50_p99': np.percentile(scan_array[:, 1], [50, 99]).tolist(),
            'receipt_minus_header_p50_p99': np.percentile(scan_array[:, 2], [50, 99]).tolist()}


_data: dict[str, np.ndarray]
_config: Settings
_matcher: LidarScanMatcher
_boxes: list[list[float]]


def worker_init(cache: str, settings: dict[str, Any], boxes: list[list[float]]) -> None:
    global _data, _config, _matcher, _boxes
    with np.load(cache) as data:
        _data = {k: data[k] for k in data.files}
    _config, _boxes = Settings(**settings), boxes
    _matcher = LidarScanMatcher(voxel_size_m=_config.voxel_m, max_iterations=50)


def cloud_at(index: int, far: bool = False) -> np.ndarray:
    from cleany_navigation.slip_monitor import base_cloud
    cloud = _data['points'][_data['offsets'][index]:_data['offsets'][index+1]]
    return base_cloud(cloud, _data['mount'], _boxes, _config, far)


def match_vector(match: Any) -> np.ndarray:
    return np.array([match.delta_x_m, match.delta_y_m, match.delta_yaw_rad])


def analyze_pair(pair: tuple[int, int]) -> dict[str, Any]:
    i, j = pair
    t0, t1 = _data['scans'][[i, j], 0]
    row: dict[str, Any] = {'t0': float(t0), 't1': float(t1), 'status': 'unknown', 'kind': 'unreliable_measurement'}
    durations = _data['scans'][[i, j], 1]
    if np.any(~np.isfinite(durations)) or np.any(durations <= 0) or np.max(durations) > _config.max_scan_duration_s or t1-t0 > _config.window_s*1.3:
        return {**row, 'reason': 'scan_timing'}
    try:
        wheel = odom_delta(_data['odom'], t0, t1, _config.max_odom_gap_s)
    except ValueError as error:
        return {**row, 'reason': str(error)}
    from cleany_navigation.slip_monitor import compare_clouds
    row.update(compare_clouds(cloud_at(i), cloud_at(j), cloud_at(i, True), cloud_at(j, True),
                              wheel, _matcher, _config))
    samples = _data['wheels'][(_data['wheels'][:, 0] >= t0) & (_data['wheels'][:, 0] <= t1)]
    if len(samples):
        row['commanded_wheel_mean_abs'] = float(np.mean(abs(samples[:, 1:5])))
        row['measured_wheel_mean_abs'] = float(np.mean(abs(samples[:, 5:9])))
    return row


def json_safe(value: Any) -> Any:
    if isinstance(value, dict):
        return {k: json_safe(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [json_safe(v) for v in value]
    if isinstance(value, float) and not math.isfinite(value):
        return None
    return value


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument('--bag', type=Path)
    source.add_argument('--cache', type=Path)
    parser.add_argument('--start', required=True, help='ISO-8601 timestamp with UTC offset')
    parser.add_argument('--end', required=True, help='ISO-8601 timestamp with UTC offset')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--config', type=Path, default=Path(__file__).with_suffix('.json'))
    parser.add_argument('--mask-file', type=Path, default=Path(__file__).resolve().parents[2]/'ros2_ws/src/cleany_navigation/config/lidar_self_mask_cad.yaml')
    parser.add_argument('--jobs', type=int, default=2)
    args = parser.parse_args()
    dates = [datetime.fromisoformat(v) for v in (args.start, args.end)]
    if any(d.utcoffset() is None for d in dates) or dates[0] >= dates[1] or not 1 <= args.jobs <= 4:
        parser.error('Require timezone-aware ordered dates and jobs in [1,4]')
    start, end = [d.timestamp() for d in dates]
    config = Settings(**json.loads(args.config.read_text()))
    args.output.mkdir(parents=True, exist_ok=False)
    import yaml
    boxes = yaml.safe_load(args.mask_file.read_text())['boxes']
    cache = args.cache.resolve() if args.cache else args.output/'inputs.npz'
    extraction = extract(args.bag, start, end, cache) if args.bag else {'cache': str(cache)}
    with np.load(cache) as data:
        stamps = data['scans'][:, 0]
    pairs = []
    for t in np.arange(start, end-config.window_s, config.stride_s):
        i = int(np.argmin(abs(stamps-t)))
        j = int(np.argmin(abs(stamps-(t+config.window_s))))
        if i != j and abs(stamps[i]-t) <= .15 and abs(stamps[j]-(t+config.window_s)) <= .15:
            if not pairs or pairs[-1] != (i, j):
                pairs.append((i, j))
    if not pairs:
        raise ValueError('No scan pairs in requested time range')
    metadata = {'settings': asdict(config), 'extraction': extraction, 'range': [args.start, args.end],
                'cache_sha256': hashlib.sha256(cache.read_bytes()).hexdigest(),
                'script_sha256': hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                'matcher_sha256': hashlib.sha256((MATCHER_ROOT/'lidar_scan_matcher.py').read_bytes()).hexdigest(),
                'convergence_sha256': hashlib.sha256((MATCHER_ROOT/'icp_convergence.py').read_bytes()).hexdigest(),
                'shared_core_sha256': {name: hashlib.sha256((Path(__file__).resolve().parents[2]/'ros2_ws/src/cleany_navigation/cleany_navigation'/name).read_bytes()).hexdigest() for name in ['slip_monitor.py','slip_estimation.py','lidar_scan_matcher.py','icp_convergence.py']},
                'mask_boxes': boxes, 'mode': 'offline_read_only', 'motion_commands_published': False,
                'timestamp_convention': 'scan header + scan_time/2; odom header interpolation; no wheel deskew/prior'}
    (args.output/'manifest.json').write_text(json.dumps(metadata, indent=2))
    rows = []
    begun = time.monotonic()
    with ProcessPoolExecutor(max_workers=args.jobs, initializer=worker_init,
                             initargs=(str(cache), asdict(config), boxes)) as pool:
        with (args.output/'windows.jsonl').open('w') as stream:
            for row in pool.map(analyze_pair, pairs, chunksize=2):
                rows.append(row)
                stream.write(json.dumps(json_safe(row), allow_nan=False)+'\n')
                if len(rows) % 20 == 0:
                    stream.flush()
                    print(json.dumps({'done': len(rows), 'total': len(pairs), 'elapsed_s': round(time.monotonic()-begun, 1)}), flush=True)
    summary = {'windows': len(rows), 'counts': {s: sum(r['status'] == s for r in rows) for s in ['suspect', 'no_evidence', 'unknown']},
               'events': events(rows, config), 'limitations': ['LiDAR motion is an estimate, not ground truth.',
               'Unknown windows must not be interpreted as no slip.', 'No per-beam motion compensation.',
               'Detects wheel-indicated motion disagreement; does not cover skidding with locked wheels.',
               'Overlapping one-second windows are not independent samples or exact physical onset times.',
               'Traction disagreement cannot distinguish floor slip from wheel spin against an obstruction.']}
    (args.output/'summary.json').write_text(json.dumps(summary, indent=2))
    print(json.dumps(summary), flush=True)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
