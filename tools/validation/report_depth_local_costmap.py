#!/usr/bin/env python3
"""Plot stationary chair observations alongside their live local costmaps."""

import argparse
import base64
import json
from pathlib import Path
from xml.etree import ElementTree as ET

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from matplotlib.colors import ListedColormap
import numpy as np


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run', type=Path)
    args = parser.parse_args()
    fig, axes = plt.subplots(2, 2, figsize=(11, 10), constrained_layout=True)
    results = {}
    # Fixed ROI around the chair seen in the recorded central depth cloud.
    roi = [.7, 1.4, -.35, .35]
    core = [.8, 1.2, -.4, .15]
    for col, stem in enumerate(('chair-present', 'chair-removed')):
        data = np.load(args.run/f'{stem}.npz')
        meta = json.loads((args.run/f'{stem}.json').read_text())
        pose = meta['costmap_to_base']
        q, t = pose['rotation'], pose['translation']
        yaw = np.arctan2(2*q['w']*q['z'], 1-2*q['z']**2)
        rot = np.array([[np.cos(yaw), -np.sin(yaw)], [np.sin(yaw), np.cos(yaw)]])
        points = data['marking'].copy()
        xy = points[:, :2]@rot.T + [t['x'], t['y']]
        info = meta['costmap']['info']
        origin = info['origin']['position']
        resolution = info['resolution']
        cells = data['costmap']
        xs = origin['x']+(np.arange(cells.shape[1])+.5)*resolution
        ys = origin['y']+(np.arange(cells.shape[0])+.5)*resolution
        xx, yy = np.meshgrid(xs, ys)
        mask = (xx >= roi[0]) & (xx <= roi[1]) & (yy >= roi[2]) & (yy <= roi[3])
        point_mask = (xy[:, 0] >= roi[0]) & (xy[:, 0] <= roi[1]) & (xy[:, 1] >= roi[2]) & (xy[:, 1] <= roi[3])
        results[stem] = dict(roi_occupied_cells=int(((cells == 100) & mask).sum()),
                             roi_marking_points=int(point_mask.sum()),
                             total_occupied_cells=int((cells == 100).sum()))
        core_mask = (xx >= core[0]) & (xx <= core[1]) & (yy >= core[2]) & (yy <= core[3])
        results[stem]['chair_core_occupied_cells'] = int(((cells == 100) & core_mask).sum())
        axes[0, col].scatter(xy[:, 0], xy[:, 1], c=points[:, 2], s=2, cmap='viridis', vmin=.192, vmax=1.4)
        axes[0, col].set_title(stem.replace('-', ' ').title()+' — measured depth hits')
        extent = [origin['x'], origin['x']+cells.shape[1]*resolution,
                  origin['y'], origin['y']+cells.shape[0]*resolution]
        display = np.where(cells == 100, 2, np.where(cells > 0, 1, 0))
        axes[1, col].imshow(display, origin='lower', extent=extent, vmin=0, vmax=2,
                           cmap=ListedColormap(['white', '#dce8f0', '#18212b']))
        axes[1, col].set_title(f"Local costmap — ROI occupied: {results[stem]['roi_occupied_cells']}")
        for ax in axes[:, col]:
            ax.add_patch(Rectangle((roi[0], roi[2]), roi[1]-roi[0], roi[3]-roi[2], fill=False, color='#00bcd4', lw=2))
            ax.add_patch(Rectangle((core[0], core[2]), core[1]-core[0], core[3]-core[2], fill=False, color='#009955', lw=1.5, linestyle='--'))
            ax.add_patch(Rectangle((-.2385, -.3274), .477, .6548, fill=False, color='#e74c3c', lw=2))
            ax.set(xlim=(-.5, 2.), ylim=(-1.5, 1.5), xlabel='Odom X (m)', ylabel='Odom Y (m)', aspect='equal')
    fig.savefig(args.run/'chair-costmap.png', dpi=170)
    plt.close(fig)
    results['roi_odom_m'] = roi
    results['chair_core_odom_m'] = core
    results['limitation'] = 'ROI is a measured-scene comparison, not semantic chair segmentation; combined LiDAR/depth costmap.'
    (args.run/'chair-comparison.json').write_text(json.dumps(results, indent=2)+'\n')
    suite = ET.parse(args.run/'tests.xml').getroot().find('testsuite')
    tests, failures = int(suite.attrib['tests']), int(suite.attrib['failures'])+int(suite.attrib['errors'])
    present, removed = results['chair-present'], results['chair-removed']
    png = base64.b64encode((args.run/'chair-costmap.png').read_bytes()).decode()
    html = f'''<!doctype html><html lang="ko"><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1"><title>Cleany · Depth local costmap</title>
<style>body{{font:16px/1.75 system-ui,sans-serif;margin:0;background:#eef2f6;color:#18212b}}
main{{max-width:1080px;margin:auto;padding:48px 24px}}h1{{font-size:34px;line-height:1.3}}
.label{{color:#167858;font-weight:700;letter-spacing:.08em}}.cards{{display:flex;gap:18px;flex-wrap:wrap}}
.card,section{{background:white;border-radius:16px;padding:24px;margin:18px 0}}.card{{flex:1;min-width:180px}}
.number{{font-size:32px;font-weight:750}}img{{width:100%;height:auto}}code{{background:#edf3f6;padding:2px 5px}}
table{{border-collapse:collapse;width:100%}}td,th{{padding:12px;text-align:left;border-bottom:1px solid #dde4ea}}
.muted{{color:#52616e}}a{{color:#096ca6}}footer{{font-size:13px;color:#52616e}}</style>
<main><div class="label">CLEANY / PHYSICAL INTEGRATION</div>
<h1>Depth 장애물 관측을<br>local costmap과 근접 정지에 연결</h1>
<p>2026-11-08 · 마일스톤 1–3 · 실물 정지 관측 + 격리 ROS 회귀 검증</p>
<div class="cards"><div class="card">의자 중심 구역의 장애물 셀<div class="number">{present['chair_core_occupied_cells']} → {removed['chair_core_occupied_cells']}</div>5cm 비용지도, 실물 의자 이동 전후</div>
<div class="card">관련 자동 검사<div class="number">{tests-failures} / {tests}</div>합성 ROS 입력과 실제 설치 Nav2 플러그인</div>
<div class="card">필수 depth 신선도<div class="number">0.5초</div>촬영 시각과 수신 중단을 기존 Monitor에서 검사</div></div>
<section><h2>적용한 구성</h2><p>LiDAR + D435 → local costmap → MPPI. 기존 Collision Monitor가 LiDAR와 필터된 depth로 근접 감속·정지를 담당한다. global costmap에 depth를 추가하지 않았으며, 별도 voxel guard와 누적 장애물 지도는 실행하지 않는다.</p>
<table><tr><th>입력</th><th>역할</th></tr><tr><td><code>/perception/depth/nav_obstacle_points</code></td><td>base_link z ≥ 0.192m의 필터 점. local 장애물 표시와 Monitor 입력.</td></tr>
<tr><td><code>/perception/depth/nav_points</code></td><td>카메라 좌표의 유효 관측. 바닥 관측을 포함해 실제로 빈 공간을 본 광선으로 제거.</td></tr>
<tr><td>센서 원점</td><td>촬영 시각 TF와 d435_depth_frame. 로봇 원점을 카메라 원점으로 대신 쓰지 않음.</td></tr></table>
<p>고립점과 고정 차체 반사를 제외한다. 무효 깊이, TF 누락, 오래된 영상은 정상 관측으로 갱신하지 않는다. 높이 하한 아래의 물체는 카메라 장애물 후보에서 제외된다.</p></section>
<section><h2>의자 표시와 제거 — 실물 관측</h2><img src="data:image/png;base64,{png}" alt="Chair present and removed: depth cloud and local costmap">
<p>상단은 필터된 실측 점군, 하단은 실제 local costmap이다. 검정은 장애물 셀, 연한 파랑은 주변 비용 영역이다. 빨강은 차체, 청록은 넓은 비교 구역, 초록 점선은 의자 중심 구역이다.</p>
<p>넓은 구역의 장애물 셀은 {present['roi_occupied_cells']} → {removed['roi_occupied_cells']}, depth 점은 {present['roi_marking_points']} → {removed['roi_marking_points']}개였다. 책상이 겹치는 가장자리 관측과 의자 중심 구역을 구분했다. 중심 구역은 {present['chair_core_occupied_cells']} → {removed['chair_core_occupied_cells']}개다. 구역은 기록 점군을 보고 지정했으며 의미 분할 정답 데이터가 아니다.</p>
<p class="muted">로봇은 정지 상태였고 odom 기준 차체 pose가 두 캡처에서 동일했다. costmap은 LiDAR와 depth가 합쳐진 결과이므로, 실제 의자 관측 전부가 depth만으로 만들어졌다고 해석하지 않는다.</p></section>
<section><h2>검증 범위</h2><ul><li>실제 설치 VoxelLayer: 표시, 다른 곳 관측 중 유지, 배경 재관측 후 제거.</li>
<li>격리 Collision Monitor: depth만으로 감속·정지, 작은 명령 유지, 센서 중단 후 정지와 재수신 복구.</li>
<li>실물: D435 토픽 수신, local costmap 반영, 의자 이동 전후 제거, 정지 확인 및 초기 위치 추정 복원.</li></ul>
<p>Humble 기본 Monitor가 stale source를 무시하는 동작을 격리 시험에서 발견해 같은 Monitor 안에 필수 depth 신선도 검사를 추가했다. 별도 속도 출력 주체는 추가하지 않았다.</p>
<p><strong>실제 회피 주행, 제동거리, 움직이는 사람에 대한 반응은 아직 검증하지 않았다.</strong> pan/tilt 변경 중 costmap 제거의 실물 회귀도 별도 시험 대상이다.</p></section>
<section><h2>재현과 원본</h2><p>실행 옵션은 <code>--depth-params …/config/phone_depth_local.yaml</code>. 이 옵션을 빼면 기존 LiDAR 전용으로 복구한다. 서비스 drop-in은 <code>30-depth-local.conf</code>이며 실행은 기존 transient service 체계를 유지했다. 재부팅 자동 시작은 이번 검증에 포함하지 않았다.</p>
<p><a href="manifest.json">Manifest</a> · <a href="tests.xml">테스트 결과</a> · <a href="chair-comparison.json">비교 수치</a> · <a href="chair-present.json">의자 있음</a> · <a href="chair-removed.json">의자 제거</a> · <a href="chair-costmap.png">원본 그림</a></p></section>
<footer>Cleany · {args.run.name} · 원본 JSON/NPZ와 적용 설정을 같은 실행 폴더에 보존.</footer></main></html>'''
    (args.run/'index.html').write_text(html)
    print(json.dumps(results))


if __name__ == '__main__':
    main()
