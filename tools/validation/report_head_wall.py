#!/usr/bin/env python3
"""Render an inspectable head wall-comparison series from saved results."""

import argparse
import hashlib
import html
import json
from pathlib import Path


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run', type=Path)
    args = parser.parse_args()
    rows = json.loads((args.run/'head-wall-series.json').read_text())
    labels = {'start': '시작', 'pan_plus': 'Pan +5°', 'pan_minus': 'Pan −5°',
              'tilt_plus': 'Tilt +5°', 'tilt_minus': 'Tilt −5°', 'return': '복귀'}
    table = ''.join(f'<tr><td>{i+1}</td><td><a href="{r["run"]}/{r["pose"]}/index.html">'
                    f'{labels[r["pose"]]}</a></td><td>{r["samples"]}</td>'
                    f'<td>{r["delta_wall_x_m"]*100:.2f} cm</td><td>{r["delta_yaw_deg"]:.2f}°</td>'
                    f'<td>{r["camera_normal_tilt_deg"]:.2f}°</td></tr>' for i, r in enumerate(rows))
    (args.run/'index.html').write_text(f'''<!doctype html><html lang="ko"><meta charset="utf-8">
<title>D435 pan·tilt와 LiDAR 벽 비교</title><style>
body{{max-width:1180px;margin:48px auto;padding:0 24px;background:#101924;color:#e3ebf5;font:17px/1.7 sans-serif}}
h1{{font-size:36px}}h2{{margin-top:36px}}a{{color:#8bc9fa}}strong{{color:#95e2cb}}
table{{width:100%;border-collapse:collapse;background:#1b2b3e}}td,th{{padding:12px;border-bottom:1px solid #35475b;text-align:left}}
img{{width:100%;border-radius:12px}}.note{{padding:18px;background:#343024;border-left:4px solid #d9ac60}}
</style><h1>D435 pan·tilt와 LiDAR 벽 비교</h1>
<p>실물 · 2026-11-08 · 차체 정지 · 실제 헤드 엔코더 + CAD 공칭 기구학</p>
<p><strong>헤드를 움직여도 전방 벽 거리 차이는 약 0.8–0.9cm로 유지됐다.</strong>
수평 방향 잔차 약 1.2–1.5°, 수직 법선 잔차 약 1.3–1.6°가 남는다.
이 결과는 운동 방향과 TF의 일관성 근거이며 완전한 외부 파라미터 보정 완료는 아니다.</p>
<table><tr><th>순서</th><th>자세</th><th>비교 표본</th><th>거리 차이</th><th>수평 방향 차이</th><th>수직 법선 잔차</th></tr>{table}</table>
<p>거리는 차체 Y=0에서 카메라 높이의 벽 X와 LiDAR 벽 X 차이다. 카메라 − LiDAR 부호다.
각 자세 링크에 원본 점 분포와 시계열, JSON이 있다.</p>
<img src="pan-tilt-wall-up/return/wall-comparison.png" alt="복귀 시 벽 비교">
<h2>관측 및 실패 경계</h2>
<p class="note">첫 실행은 tilt 아래 자세에서 복귀 시 4틱(약 0.35°) 차이에 머물러
3틱 허용 조건에서 중단됐다. 실패 자료를 보존하고 두 모터의 토크 OFF를 확인했다.
조건을 완화하지 않고 남은 tilt 위쪽과 복귀를 별도 실행했으며 이 구간은 완료됐다.</p>
<p>최종 엔코더 13번 1997, 14번 2134, 두 모터 토크 OFF. 읽기 전용 관절 TF 서비스는 실행 중이다.
시험의 RAM 가속도 5, 속도 80tick/s, 토크 한도 300은 남겼다. EEPROM과 차체 주행 설정은 변경하지 않았다.</p>
<h2>TF 기준</h2><p>base_link → d435_pan_link → d435_tilt_link → d435_link.
CAD의 horn 축과 공칭 D435 측정 원점을 사용하고 과거 관절 영점 2009/2067 및 방향 -1/+1을 적용했다.
카메라 지면 높이는 CAD 추정 약 111.7cm이며 실제 높이 보정은 아니다.</p>
<p>초기 고정 TF는 중지했다. 관절각을 무시한 첫 정지 결과는 <a href="wall-raw/index.html">별도로 보존</a>한다.
현재 LiDAR Z=0의 기존 평면 관례는 유지했으므로 바닥 높이 필터에 바로 사용하면 안 된다.
벽 한 면으로 높이와 벽에 평행한 위치를 검증할 수 없으며 실제 벽 기울기와 장착 오차도 분리되지 않는다.</p>
<p>깊이 입력의 costmap·Collision Monitor 연결과 자동 보정은 아직 적용하지 않았다.
합성 기하와 직렬 패킷·쓰기 제한 관련 검사 12개 통과. 실물 관측과 별도 근거다.</p>
<p><a href="head-wall-series.json">비교 수치</a> · <a href="cad-mount-provenance.json">CAD 공칭값 출처</a>
 · <a href="head-readback.json">연결 직후 모터 조회</a> · <a href="manifest.json">입력·소스 해시</a></p></html>''')
    repository = Path(__file__).resolve().parents[2]
    sources = [Path(__file__), *(Path(__file__).parent/name for name in (
        'head_pan_tilt_probe.py', 'read_head_servos.py', 'capture_depth_lidar.py',
        'compare_depth_lidar_wall.py', 'config/d435_pan_tilt.json', 'config/d435_cad_mount.json'))]
    inputs = [args.run/r['run']/r['pose']/filename for r in rows
              for filename in ('capture.json', 'depth.npz', 'wall-comparison.json')]
    manifest = {'source_hashes': {str(p.relative_to(repository)): hashlib.sha256(p.read_bytes()).hexdigest() for p in sources},
                'input_hashes': {str(p.relative_to(args.run)): hashlib.sha256(p.read_bytes()).hexdigest() for p in inputs}}
    (args.run/'manifest.json').write_text(json.dumps(manifest, indent=2))


if __name__ == '__main__':
    main()
