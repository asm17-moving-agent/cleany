#!/usr/bin/env python3
"""Plot recorded command direction against wheel and LiDAR displacement in one bag."""
from __future__ import annotations

import argparse
from datetime import datetime, timedelta, timezone
import html
import json
import math
from pathlib import Path
import sqlite3

import numpy as np

KST = timezone(timedelta(hours=9))


def direction(vector, minimum=0.015):
    """Return body-frame degrees only when displacement is large enough to orient."""
    x, y = vector[:2]
    if not all(math.isfinite(v) for v in (x, y)) or math.hypot(x, y) < minimum:
        return None
    return math.degrees(math.atan2(y, x))


def command_displacement(samples, odom, t0, t1, maximum_hold=.3):
    """Integrate recorded body commands in the window's initial odometry frame.

    Commands have receive timestamps, odometry has header timestamps. A sample
    is held for at most maximum_hold seconds so recording gaps do not invent
    intent. Wheel yaw aligns the command frames; it is not translation truth.
    """
    if not samples or len(odom) < 2 or odom[0][0] > t0 or odom[-1][0] < t1:
        return None, 0.
    times = np.array([v[0] for v in odom])
    yaws = np.unwrap([v[1] for v in odom])
    initial = float(np.interp(t0, times, yaws))
    delta = np.zeros(2)
    covered = 0.
    for i, (stamp, x, y) in enumerate(samples):
        next_stamp = samples[i + 1][0] if i + 1 < len(samples) else t1
        a, b = max(t0, stamp), min(t1, next_stamp, stamp + maximum_hold)
        if b <= a:
            continue
        theta = float(np.interp((a + b) / 2, times, yaws)) - initial
        c, s = math.cos(theta), math.sin(theta)
        delta += np.array([c*x-s*y, s*x+c*y]) * (b-a)
        covered += b-a
    coverage = covered / (t1-t0)
    return (delta.tolist() if coverage >= .75 else None), coverage


def read_window(bag, start, end):
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    database = sqlite3.connect(f'file:{bag}?mode=ro', uri=True)
    selected = {'/navigation/slip', '/navigation/traction_recovery', '/nav2/cmd_vel', '/cmd_vel', '/odom'}
    topics = {i: (name, get_message(kind)) for i, name, kind in
              database.execute('select id,name,type from topics') if name in selected}
    windows, phases, odom = {}, [], []
    commands = {'/nav2/cmd_vel': [], '/cmd_vel': []}
    for topic_id, stamp, data in database.execute(
            'select topic_id,timestamp,data from messages where timestamp between ? and ? order by timestamp',
            (int((start-2)*1e9), int((end+1)*1e9))):
        if topic_id not in topics:
            continue
        name, kind = topics[topic_id]
        message = deserialize_message(data, kind)
        when = stamp / 1e9
        if name == '/navigation/slip':
            row = json.loads(message.data)
            if row.get('t0', 0) >= start and row.get('t1', end+1) <= end:
                windows.setdefault((row['t0'], row['t1']), dict(row, published_at=when))
        elif name == '/navigation/traction_recovery':
            row = dict(json.loads(message.data), at=when)
            if start <= when <= end and (not phases or
                    (phases[-1]['phase'], phases[-1]['reason']) != (row['phase'], row['reason'])):
                phases.append(row)
        elif name in commands:
            commands[name].append((when, message.linear.x, message.linear.y))
        elif name == '/odom':
            q = message.pose.pose.orientation
            yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
            odom.append((message.header.stamp.sec + message.header.stamp.nanosec/1e9, yaw))
    database.close()
    rows = sorted(windows.values(), key=lambda row: row['t1'])
    for row in rows:
        delta, coverage = command_displacement(commands['/nav2/cmd_vel'], odom, row['t0'], row['t1'])
        row['command_delta'] = delta
        row['command_coverage'] = coverage
        row['command_angle_deg'] = direction(delta) if delta is not None else None
        row['wheel_angle_deg'] = direction(row.get('wheel_delta', [0, 0]))
        row['lidar_angle_deg'] = (direction(row.get('lidar_delta', [0, 0]))
                                  if row['state'] != 'unknown' else None)
    return rows, phases, commands


def write_figures(output, rows, phases, start, end):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    plt.rcParams.update({'font.size': 10, 'axes.spines.top': False, 'axes.spines.right': False,
                         'savefig.facecolor': 'white'})
    colors = dict(clear='#328477', suspect='#f59e0b', confirmed='#dd493d', unknown='#8b95a5')
    fig, axes = plt.subplots(3, 1, figsize=(12.5, 10), sharex=True, constrained_layout=True)
    times = [r['t1']-start for r in rows]
    def numeric(values):
        return [float('nan') if v is None else v for v in values]
    for key, label, color, marker in [('command_angle_deg', 'Requested direction (window integrated)', '#326ce5', 'o'),
            ('wheel_angle_deg', 'Wheel odometry direction', '#c98019', '.'),
            ('lidar_angle_deg', 'LiDAR-estimated actual direction', '#17866c', 's')]:
        axes[0].plot(times, numeric([r[key] for r in rows]), marker=marker, lw=1.5, color=color, label=label)
    for a in (-135, -45, 45, 135):
        axes[0].axhspan(a-10, a+10, color='#ffbf57', alpha=.10)
    all_angles = [r[k] for r in rows for k in ('command_angle_deg', 'wheel_angle_deg', 'lidar_angle_deg') if r[k] is not None]
    axes[0].set_ylim((min(all_angles)-12, max(all_angles)+12) if all_angles else (-180, 180))
    axes[0].set_ylabel('Body-frame direction (degrees)')
    axes[0].set_title('0 = forward, +90 = left; orange bands = near diagonals', loc='left', fontsize=11)
    axes[0].legend(loc='upper left', fontsize=8)
    for i, (key, label, color) in enumerate([('wheel_delta', 'Wheel odometry', '#c98019'),
                                            ('lidar_delta', 'LiDAR estimate', '#17866c')]):
        valid = lambda r: key != 'lidar_delta' or r['state'] != 'unknown'
        axes[1].plot(times, [r.get(key, [math.nan, math.nan])[1]*100 if valid(r) else math.nan for r in rows],
                     marker='o', color=color, label=f'{label}: lateral displacement')
    axes[1].set_ylabel('Leftward displacement (cm/window)')
    axes[1].legend(loc='upper left', fontsize=8)
    errors = [r.get('translation_error_m', math.nan)*100 for r in rows]
    axes[2].plot(times, errors, color='#dd493d', lw=1.6, label='Translation disagreement')
    axes[2].axhline(3, color='#dd493d', ls=':', alpha=.7, label='3 cm threshold (also requires >=35%)')
    axes[2].scatter(times, errors, c=[colors[r['state']] for r in rows], s=42, zorder=5)
    for state, color in colors.items():
        axes[2].scatter([], [], c=color, label=state)
    axes[2].set_ylabel('Wheel-LiDAR disagreement (cm)')
    axes[2].legend(loc='upper left', fontsize=8, ncol=2)
    axes[2].set_xlabel('Seconds since ' + datetime.fromtimestamp(start, KST).strftime('%H:%M:%S.%f')[:-3]+' KST')
    event_colors = {'braking':'#f59e0b', 'longitudinal_retry':'#326ce5', 'failed':'#dd493d'}
    for index, phase in enumerate(phases):
        if phase['phase'] not in event_colors:
            continue
        a = phase['at']-start
        b = min(end, phases[index+1]['at'] if index+1 < len(phases) else end)-start
        for ax in axes:
            ax.axvspan(a, b, color=event_colors[phase['phase']], alpha=.08)
        axes[0].axvline(a, color=event_colors[phase['phase']], ls='--', alpha=.7)
        label = {'braking':'Brake', 'longitudinal_retry':'Retry', 'failed':'Failed'}[phase['phase']]
        axes[0].text(a+.025, .99, label, transform=axes[0].get_xaxis_transform(),
                     va='top', fontsize=8, rotation=90, color=event_colors[phase['phase']])
    for ax in axes:
        ax.grid(alpha=.15); ax.set_xlim(0, end-start)
    fig.suptitle('Recorded slip event: command, wheel odometry and LiDAR motion', fontsize=15, fontweight='bold')
    fig.savefig(output/'slip-direction.png', dpi=180)
    fig.savefig(output/'slip-direction.svg')
    plt.close(fig)

    confirmed = next(r for r in rows if r['state'] == 'confirmed')
    fig, ax = plt.subplots(figsize=(6.8, 5), constrained_layout=True)
    for key, label, color in [('command_delta','Command integral','#326ce5'),
                              ('wheel_delta','Wheel odometry','#c98019'),
                              ('lidar_delta','LiDAR estimate','#17866c')]:
        v = confirmed.get(key)
        if v is None: continue
        x, y = np.array(v[:2])*100
        ax.quiver(0, 0, x, y, angles='xy', scale_units='xy', scale=1, color=color, label=label, width=.009)
        ax.annotate(f'{label}\n({x:.1f}, {y:.1f}) cm', (x, y), xytext=(5, 6), textcoords='offset points', color=color, fontsize=9)
    ax.set(xlabel='Forward (cm)', ylabel='Left (cm)', xlim=(-1, 15), ylim=(-2, 9), aspect='equal')
    ax.grid(alpha=.2)
    ax.set_title('First confirmed window: '+datetime.fromtimestamp(confirmed['t0'], KST).strftime('%H:%M:%S.%f')[:-3]+' - '+datetime.fromtimestamp(confirmed['t1'], KST).strftime('%H:%M:%S.%f')[:-3])
    fig.savefig(output/'confirmed-window.png', dpi=180);fig.savefig(output/'confirmed-window.svg');plt.close(fig)

    theta = np.linspace(-180, 180, 721)
    angle = np.radians(theta)
    fig, ax = plt.subplots(figsize=(10.5, 3.7), constrained_layout=True)
    ax.plot(theta, .1*np.sin(angle)**2+.45*np.sin(2*angle)**2, label='Recorded run: lateral 0.1 / diagonal 0.45', color='#8b95a5')
    ax.plot(theta, np.sin(angle)**2+20*np.sin(2*angle)**2, label='New setting: lateral 1 / diagonal 20', color='#326ce5')
    ax.set(xlabel='Body-frame travel direction (degrees)', ylabel='Added cost per metre', xticks=np.arange(-180, 181, 45))
    ax.grid(alpha=.2);ax.legend(fontsize=9)
    fig.savefig(output/'direction-cost.png', dpi=160);fig.savefig(output/'direction-cost.svg');plt.close(fig)
    return confirmed


def write_report(output, bag, rows, phases, start, end, confirmed):
    def n(value): return '—' if value is None else f'{value:.1f}'
    def clock(value): return datetime.fromtimestamp(value, KST).strftime('%H:%M:%S.%f')[:-3]
    table = ''.join(f'<tr><td>{clock(r["t1"])}</td><td>{n(r["command_angle_deg"])}</td><td>{n(r["wheel_angle_deg"])}</td><td>{n(r["lidar_angle_deg"])}</td><td>{n(r.get("translation_error_m",0)*100)}</td><td>{html.escape(r["state"])}</td></tr>' for r in rows)
    timeline = ''.join(f'<li><time>{clock(r["at"])}</time><b>{html.escape(r["phase"])}</b><code>{html.escape(r["reason"])}</code></li>' for r in phases)
    payload = {'bag':str(bag),'start':start,'end':end,'frame':'base_link at each window start',
               'direction_minimum_displacement_m':.015, 'command_maximum_hold_s':.3,
               'command_minimum_coverage':.75,'windows':rows,'phases':phases}
    (output/'analysis.json').write_text(json.dumps(payload, ensure_ascii=False, indent=2))
    doc = f'''<!doctype html><html lang="ko"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>대각선 명령과 슬립 — 2026-11-08</title><style>
*{{box-sizing:border-box}}body{{margin:0;background:#edf2f8;color:#17263b;font:16px/1.65 system-ui,sans-serif}}main{{max-width:1180px;margin:auto;padding:40px 24px}}header{{padding:32px;background:#122841;color:white;border-radius:20px}}h1{{font-size:34px;margin:8px 0}}h2{{font-size:24px;margin:0 0 16px}}h3{{margin-top:0}}.eyebrow{{letter-spacing:.12em;color:#9dd9ec;font-size:12px}}section{{background:white;padding:28px;margin-top:22px;border-radius:18px}}.cards{{display:grid;grid-template-columns:repeat(3,1fr);gap:16px;margin-top:22px}}.card{{padding:22px;border-radius:16px;background:white}}.big{{font-size:30px;font-weight:750}}.muted{{color:#61758c}}.note{{border-left:4px solid #e5a53c;padding:12px 18px;background:#fff7e6}}img{{width:100%;height:auto}}.two{{display:grid;grid-template-columns:1fr 1fr;gap:24px;align-items:center}}table{{border-collapse:collapse;width:100%;font-size:14px}}td,th{{padding:10px;text-align:right;border-bottom:1px solid #e6ecf4}}td:first-child,th:first-child{{text-align:left}}.scroll{{overflow:auto}}time{{font-variant-numeric:tabular-nums;color:#61758c}}li time,li b{{margin-right:14px}}code{{font-size:.87em;background:#eaf0f7;padding:3px 5px;border-radius:5px}}a{{color:#205cca}}footer{{font-size:13px;padding:24px 0;color:#61758c}}@media(max-width:750px){{main{{padding:18px 12px}}h1{{font-size:27px}}.cards,.two{{grid-template-columns:1fr}}section,header{{padding:20px}}}}
</style><main><header><div class="eyebrow">CLEANY · RECORDED PHYSICAL RUN</div><h1>대각선으로 명령했을 때<br>차체는 얼마나 따라갔나?</h1><p>2026-11-08 · {clock(start)}–{clock(end)} KST<br>슬립 직전 명령, 휠 odometry, LiDAR 이동 추정의 시간 비교</p></header>
<div class="cards"><div class="card"><div class="muted">명령 방향 · 첫 확정 구간</div><div class="big">{n(confirmed['command_angle_deg'])}°</div><div>전방 0° / 왼쪽 90°</div></div><div class="card"><div class="muted">LiDAR 추정 실제 이동 방향</div><div class="big">{n(confirmed['lidar_angle_deg'])}°</div><div>같은 약 1초 구간</div></div><div class="card"><div class="muted">odom–LiDAR 이동 불일치</div><div class="big">{confirmed['translation_error_m']*100:.2f} cm</div><div>상대 차이 {confirmed['translation_error_fraction']*100:.1f}%</div></div></div>
<section><h2>판독 결과</h2><p><b>전방과 좌측을 함께 명령한 구간에서 측방 이동이 크게 줄었고, 슬립 판정이 발생했다.</b> 첫 확정 구간에서 wheel odometry는 왼쪽으로 {confirmed['wheel_delta'][1]*100:.2f}cm를 예상했지만 LiDAR 추정은 {confirmed['lidar_delta'][1]*100:.2f}cm였다.</p><p>초반에는 비슷한 대각 명령도 실제 이동이 잘 따라갔다. 따라서 대각 명령 중 슬립이 일어난 것은 확인되지만, 이동 각도만으로 바닥 위치·접지·물리적 걸림 등의 원인을 구분할 수는 없다.</p><p class="note">이 그래프는 <b>변경 전 횡 0.1 / 대각 0.45</b> 설정의 실제 기록이다. 새 <b>횡 1 / 대각 20</b> 설정의 실물 효과를 보여 주는 결과가 아니다. LiDAR 정합은 독립적인 이동 추정이며 외부 측정 참값은 아니다.</p></section>
<section><h2>시간축: 방향과 슬립의 관계</h2><img src="slip-direction.svg" alt="명령 방향과 odometry 및 LiDAR 이동 방향, 측방 이동량, 슬립 오차의 시간 그래프"><p class="muted">각 점은 표시된 끝시각 이전 약 1초를 나타낸다. 명령은 같은 구간에서 적분하며 yaw 변화로 시작 차체 좌표에 맞췄다. 회색/누락은 정합 불확실 또는 이동 1.5cm 미만이며 각도를 계산하지 않았다. 구간은 약 0.5초씩 겹치므로 두 확정 창은 독립 표본이 아니다.</p><a href="slip-direction.png">고해상도 PNG</a> · <a href="analysis.json">계산 원본 JSON</a></section>
<section class="two"><div><h2>첫 슬립 확정 구간의 이동 벡터</h2><p>방향은 지도 기준이 아니라 <b>각 창 시작 시점의 차체 기준</b>이다. wheel odometry와 LiDAR 추정이 같은 좌표계를 사용한다.</p><p>명령: 파랑<br>휠이 계산한 이동: 주황<br>LiDAR가 추정한 차체 이동: 초록</p><p>대각 명령과 슬립 판정의 동시 발생은 확인된다. 이 한 구간만으로 바닥 마찰, 물리적 걸림 등 원인 전체를 확정하지 않는다.</p></div><img src="confirmed-window.svg" alt="첫 확정 구간의 명령 적분, 휠 이동 및 LiDAR 이동 벡터"></section>
<section><h2>복구는 왜 중단됐나?</h2><ol>{timeline}</ol><p>전후진·회전 전용 플래너가 최종 목표 <b>(−7.18, −11.87)m</b>까지 경로를 생성하지 못했다. <b>186,894회 탐색, 0.37초</b> 후 <code>no_valid_lattice_path</code>를 반환해 <code>longitudinal_retry_failed</code>로 종료했다. 2초 탐색 제한에 도달한 실패가 아니며, 재시도 주행 중 두 번째 슬립으로 끝난 것도 아니다.</p><p class="muted">그 시점의 복구용 costmap snapshot은 기록되지 않았다. 물리적으로 통과가 불가능했는지, 격자/footprint/시작 방향 제약 때문인지는 현재 기록만으로 구분할 수 없다.</p></section>
<section><h2>새 방향 비용: sin 기반 1 / 20</h2><p><code>cost / m = 1 × sin²(θ) + 20 × sin²(2θ)</code></p><img src="direction-cost.svg" alt="기존 및 새 sin 기반 방향 비용 곡선"><p>전후진 0, 횡이동 1, 45° 대각 20.5 cost/m. 대각 후보를 삭제하지 않고, 장애물·경로·목표 비용과 함께 평가한다. 순수 회전에는 이 이동 비용을 추가하지 않는다.</p></section>
<section><h2>복구 경로 개선안</h2><ol><li><b>실패 당시 입력부터 보존.</b> 복구용 costmap, footprint, 시작·목표 자세, primitive와 파라미터를 한 묶음으로 저장해 같은 입력에서 재현한다.</li><li><b>짧은 안전 복구 구간을 먼저 탐색.</b> 최종 목적지까지의 전후진 경로만 요구하지 않고, 현재 주변에서 충돌 없이 전후진·회전 가능한 중간 자세 후보를 찾는다. 현재 위치부터 전체 차체가 지나가는 구간의 충돌을 검사한다.</li><li><b>안전한 중간 자세에서 다시 판단.</b> 후보를 골랐다고 바로 원래 대각 주행을 재개하지 않는다. 이동 일치가 회복됐는지 확인하고, 종방향 우선 경로로 원래 목표를 이어 간다. 시작 방향과 목표 격자 정렬이 실패 원인이면 충돌 검사 안에서 대안을 탐색한다.</li><li><b>횟수·거리·시간을 제한.</b> 재차 슬립, 측정 불가, 안전 후보 없음이면 멈춘다. STOP·충돌 감시·Supervisor 권한은 유지한다. 먼저 기록 입력과 격리된 ROS 환경에서 검증한다.</li></ol><p class="note">이 항목은 개선 계획이다. 이번 작업의 운용 변경은 방향 비용 1 / 20이며, 위 복구 알고리즘을 구현했다고 표시하지 않는다.</p></section>
<section><h2>구간별 수치</h2><div class="scroll"><table><thead><tr><th>창 끝시각 KST</th><th>명령 °</th><th>wheel °</th><th>LiDAR °</th><th>차이 cm</th><th>상태</th></tr></thead><tbody>{table}</tbody></table></div></section>
<footer>원본 bag: {html.escape(str(bag))}<br>명령은 rosbag 수신 시각, 센서 구간은 header 시각을 사용한다. 명령 기록을 최대 0.3초만 유지하고 구간 커버리지 75% 미만은 제외했다. 겹치는 창을 중복 제거했으며, unknown LiDAR 값을 유효한 실제 방향으로 그리지 않았다.</footer></main></html>'''
    (output/'report.html').write_text(doc)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bag', type=Path, required=True)
    parser.add_argument('--start', type=float, required=True)
    parser.add_argument('--end', type=float, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    rows, phases, commands = read_window(args.bag, args.start, args.end)
    if not rows or not any(r['state'] == 'confirmed' for r in rows):
        parser.error('Selected interval contains no confirmed slip window')
    confirmed = write_figures(args.output, rows, phases, args.start, args.end)
    write_report(args.output, args.bag, rows, phases, args.start, args.end, confirmed)
    print(json.dumps({'windows':len(rows),'first_confirmed':confirmed,'report':str(args.output/'report.html')}, ensure_ascii=False))


if __name__ == '__main__':
    main()
