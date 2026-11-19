#!/usr/bin/env python3
"""Compare stationary wall fits; never tune or publish calibration automatically."""

import argparse
import json
from pathlib import Path

import numpy as np


def rotation(xyzw: list[float]) -> np.ndarray:
    q = np.asarray(xyzw, dtype=float)
    q /= np.linalg.norm(q)
    x, y, z, w = q
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
        [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)],
    ])


def fit_wall(points: np.ndarray, tolerance: float = 0.025) -> dict:
    """Fit a front-facing line/plane with RANSAC and TLS, in metres.

    The normal points forward (+X). Fits require >= 50% support; this is a
    measurement validity check, not a robot safety or calibration threshold.
    """
    dim = points.shape[1]
    if len(points) < 30:
        raise ValueError('Too few wall points')
    rng = np.random.default_rng(42)
    best = np.zeros(len(points), dtype=bool)
    for _ in range(200):
        sample = points[rng.choice(len(points), dim, replace=False)]
        if dim == 3:
            normal = np.cross(sample[1]-sample[0], sample[2]-sample[0])
        else:
            delta = sample[1]-sample[0]
            normal = np.array([-delta[1], delta[0]])
        norm = np.linalg.norm(normal)
        if norm < 1e-10:
            continue
        normal /= norm
        if abs(normal[0]) < 0.8:
            continue
        mask = np.abs(points @ normal - sample[0] @ normal) < tolerance
        if mask.sum() > best.sum():
            best = mask
    if best.mean() < 0.5:
        raise ValueError('No dominant front-facing wall')
    for _ in range(3):
        centre = points[best].mean(axis=0)
        _, _, vh = np.linalg.svd(points[best]-centre, full_matrices=False)
        normal = vh[-1]
        if normal[0] < 0:
            normal = -normal
        offset = float(centre @ normal)
        best = np.abs(points @ normal-offset) < tolerance
    if best.mean() < 0.5 or normal[0] < 0.8:
        raise ValueError('Unstable front-wall refinement')
    residual = points @ normal-offset
    return {'normal': normal.tolist(), 'offset_m': offset,
            'inlier_fraction': float(best.mean()),
            'inlier_rmse_m': float(np.sqrt(np.mean(residual[best]**2))),
            'all_abs_residual_p95_m': float(np.percentile(abs(residual), 95)),
            'yaw_deg': float(np.degrees(np.arctan2(normal[1], normal[0]))),
            'points': len(points)}


def camera_pose(data: dict, stamp: float, fallback: dict) -> tuple[np.ndarray, np.ndarray]:
    """Resolve the fresh measured head chain, or the explicit nominal snapshot."""
    optical = data['tf_static']['d435_depth_optical_frame']
    chain = [optical, data['tf_static']['d435_depth_frame']]
    dynamic = data.get('tf_dynamic', [])
    if dynamic:
        for child in ('d435_link', 'd435_tilt_link', 'd435_pan_link'):
            candidates = [t for t in dynamic if t['child'] == child]
            t = min(candidates, key=lambda t: abs(t['stamp']-stamp))
            if abs(t['stamp']-stamp) > .15:
                raise ValueError('No fresh head transform at camera exposure time')
            if chain[-1]['parent'] != child:
                raise ValueError('Unexpected dynamic head frame chain')
            chain.append(t)
    else:
        chain.append({'parent': fallback['parent'], 'xyz': fallback['translation_m'], 'xyzw': [0, 0, 0, 1]})
    if chain[-1]['parent'] != fallback['parent']:
        raise ValueError('Wrong head root frame')
    r, p = np.eye(3), np.zeros(3)
    for t in chain:
        rt = rotation(t['xyzw'])
        p, r = rt @ p + np.asarray(t['xyz']), rt @ r
    return r, p


def analyze(capture: Path, mount: Path) -> dict:
    data = json.loads((capture/'capture.json').read_text())
    config = json.loads(mount.read_text())
    if any(config['rpy_rad']):
        raise ValueError('This nominal-CAD comparison requires zero mount RPY')
    camera_translation = np.asarray(config['translation_m'])
    optical = data['tf_static']['d435_depth_optical_frame']
    depth_frame = data['tf_static']['d435_depth_frame']
    if optical['parent'] != 'd435_depth_frame' or depth_frame['parent'] != config['child']:
        raise ValueError('Unexpected camera TF chain')
    r_depth = rotation(depth_frame['xyzw'])
    r_camera = r_depth @ rotation(optical['xyzw'])
    camera_translation = camera_translation + np.asarray(depth_frame['xyz']) + r_depth @ np.asarray(optical['xyz'])
    lidar = data['tf_static'][data['scans'][0]['frame']]
    if lidar['parent'] != config['parent']:
        raise ValueError('LiDAR must have a direct transform from comparison frame')
    r_lidar = rotation(lidar['xyzw'])
    depths = np.load(capture/'depth.npz')['depth_mm'].astype(float)*0.001
    k = np.asarray(data['camera_info']['k']).reshape(3, 3)
    h, w = depths.shape[1:]
    # Central image ROI reduces side furniture and floor; retains vertical extent.
    vv, uu = np.mgrid[int(h*.2):int(h*.8):4, int(w*.2):int(w*.8):4]
    frames, failure, pairs = [], [], []
    for index, item in enumerate(data['depth_times']):
        try:
            r_camera, camera_translation = camera_pose(data, item['stamp'], config)
        except ValueError as exc:
            failure.append({'index': index, 'reason': str(exc)})
            continue
        scan = min(data['scans'], key=lambda s: abs(s['stamp']+s['scan_time']/2-item['stamp']))
        skew = abs(scan['stamp']+scan['scan_time']/2-item['stamp'])
        if skew > 0.15:
            failure.append({'index': index, 'reason': 'timestamp skew', 'skew_s': skew})
            continue
        z = depths[index, vv, uu].ravel()
        points = np.column_stack(((uu.ravel()-k[0, 2])*z/k[0, 0],
                                  (vv.ravel()-k[1, 2])*z/k[1, 1], z))
        valid = np.isfinite(z) & (z > .3) & (z < 5)
        points = points[valid] @ r_camera.T + camera_translation
        ranges = np.array([np.nan if x is None else x for x in scan['ranges']])
        angles = scan['angle_min'] + np.arange(len(ranges))*scan['angle_increment']
        valid_scan = np.isfinite(ranges) & (ranges > max(.3, scan['range_min'])) & (ranges < min(5, scan['range_max']))
        xyz = np.column_stack((ranges*np.cos(angles), ranges*np.sin(angles), np.zeros(len(ranges))))
        xyz = xyz[valid_scan] @ r_lidar.T + np.asarray(lidar['xyz'])
        front = (xyz[:, 0] > .3) & (abs(np.arctan2(xyz[:, 1], xyz[:, 0])) < np.deg2rad(30))
        xy = xyz[front, :2]
        try:
            camera_fit, lidar_fit = fit_wall(points), fit_wall(xy)
        except ValueError as exc:
            failure.append({'index': index, 'reason': str(exc)})
            continue
        n = np.array(camera_fit['normal'])
        camera_x = (camera_fit['offset_m']-n[2]*camera_translation[2])/n[0]
        lidar_x = lidar_fit['offset_m']/lidar_fit['normal'][0]
        frames.append({'index': index, 'stamp': item['stamp'], 'skew_s': skew,
                       'camera': camera_fit, 'lidar': lidar_fit,
                       'camera_wall_x_at_camera_height_m': float(camera_x),
                       'lidar_wall_x_m': float(lidar_x),
                       'delta_wall_x_m': float(camera_x-lidar_x),
                       'delta_yaw_deg': camera_fit['yaw_deg']-lidar_fit['yaw_deg'],
                       'camera_normal_tilt_deg': float(np.degrees(np.arctan2(n[2], np.linalg.norm(n[:2]))))})
        pairs.append((points, xy))
    if not frames:
        raise ValueError(f'No valid wall comparisons: {failure}')
    odom = data['odom']
    report = {'frames': frames, 'failed_frames': failure,
              'transform_source': 'measured_head_dynamic_tf' if data.get('tf_dynamic') else 'nominal_CAD_pose_without_joint_measurement',
              'max_odom_translation_speed_m_s': max((np.linalg.norm(v['v']) for v in odom), default=None),
              'max_odom_rotation_speed_rad_s': max((np.linalg.norm(v['w']) for v in odom), default=None),
              'mount': config,
              'boundary': 'Nominal CAD TF; observed common wall at different heights. Not full 6DoF calibration. Wall identity/flatness and stationary physical placement supplied by operator. LiDAR Z remains planar convention; height and tangential offsets are not validated.'}
    for field in ('delta_wall_x_m', 'delta_yaw_deg', 'camera_normal_tilt_deg',
                  'camera_wall_x_at_camera_height_m', 'lidar_wall_x_m'):
        values = [f[field] for f in frames]
        report[field] = {'median': float(np.median(values)), 'min': min(values), 'max': max(values)}
    (capture/'wall-comparison.json').write_text(json.dumps(report, indent=2))
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(1, 3, figsize=(16, 5), layout='constrained')
    points, xy = pairs[len(pairs)//2]
    axes[0].scatter(points[:, 1], points[:, 0], s=1, alpha=.25, label='Depth ROI (CAD TF)')
    axes[0].scatter(xy[:, 1], xy[:, 0], s=9, label='LiDAR front sector')
    axes[0].set(xlabel='Robot left Y (m)', ylabel='Robot forward X (m)', title='Wall projections')
    axes[0].legend()
    times = np.array([f['stamp'] for f in frames])-frames[0]['stamp']
    axes[1].plot(times, [f['delta_wall_x_m']*100 for f in frames], 'o-')
    axes[1].axhline(0, color='grey')
    axes[1].set(xlabel='Time (s)', ylabel='Camera - LiDAR (cm)', title='Wall distance difference')
    axes[2].plot(times, [f['delta_yaw_deg'] for f in frames], 'o-', label='Yaw difference')
    axes[2].plot(times, [f['camera_normal_tilt_deg'] for f in frames], 'o-', label='Camera wall normal tilt')
    axes[2].set(xlabel='Time (s)', ylabel='Degrees', title='Wall normal angles')
    axes[2].legend()
    fig.savefig(capture/'wall-comparison.png', dpi=150)
    plt.close(fig)
    delta = report['delta_wall_x_m']['median']*100
    yaw = report['delta_yaw_deg']['median']
    tilt = report['camera_normal_tilt_deg']['median']
    tf_label = '실측 관절각 + CAD 공칭 기구학' if data.get('tf_dynamic') else '관절각 미반영 CAD 공칭 자세'
    (capture/'index.html').write_text(f'''<!doctype html><html lang="ko"><meta charset="utf-8">
<title>D435 · LiDAR 벽 정합</title><style>
body{{max-width:1200px;margin:50px auto;padding:0 24px;background:#101925;color:#e5edf6;font:18px/1.7 sans-serif}}
h1{{font-size:36px}} img{{width:100%;background:white;border-radius:12px}} a{{color:#8bc8ff}}
.cards{{display:flex;gap:20px}} .cards p{{flex:1;background:#203147;padding:20px;border-radius:12px}}
strong{{display:block;font-size:30px;color:#8fe0ca}}
</style><h1>D435 · LiDAR 벽 정합</h1><p>실물 정지 관측 · {tf_label} · 자동 보정 미적용</p>
<div class="cards"><p>전방 거리 차이<strong>{delta:.2f} cm</strong>카메라 − LiDAR</p>
<p>벽 방향 차이<strong>{yaw:.2f}°</strong>수평 법선 방향</p>
<p>카메라 벽 기울기<strong>{tilt:.2f}°</strong>수직 벽 가정 시 각도 잔차</p></div>
<img src="wall-comparison.png" alt="벽 위치, 거리 차이와 법선 각도">
<p>깊이 {len(data['depth_times'])}프레임, LiDAR {len(data['scans'])}스캔 중 유효 비교 {len(frames)}개.
카메라 중앙 60% 영역의 3D 평면과 LiDAR 전방 ±30° 선을 독립적으로 맞췄다.
차체 Y=0에서 카메라 높이의 벽 X와 LiDAR 벽 X를 비교한다. 수직이며 동일한 벽이라는 현장 전제가 필요하다.</p>
<p>카메라 CAD 원점 높이 102.18cm, CAD 바닥 추정값 기준 약 111.73cm.
현재 LiDAR Z=0은 기존 2D 관례로 유지했다. 이번 결과는 높이, 벽에 평행한 위치,
전체 6자유도 보정을 검증하지 않는다. 법선 기울기는 실제 벽 기울기와 장착 오차를 분리하지 못한다.</p>
<p>현재 costmap·Collision Monitor에 깊이 입력은 연결하지 않았다.</p>
<p><a href="wall-comparison.json">수치와 분석 조건</a> · <a href="capture.json">수신 원본 메타데이터</a></p></html>''')
    return report


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--capture', type=Path, required=True)
    parser.add_argument('--mount', type=Path, required=True)
    args = parser.parse_args()
    result = analyze(args.capture, args.mount)
    print(json.dumps({k: v for k, v in result.items() if k not in ('frames', 'mount')}, indent=2))
