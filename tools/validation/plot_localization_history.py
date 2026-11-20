#!/usr/bin/env python3
"""Plot recorded alignment scores and command activity without replaying ROS topics."""
from __future__ import annotations

import argparse
import csv
from datetime import datetime, timedelta, timezone
import hashlib
import json
import math
from pathlib import Path
import sqlite3

import matplotlib
matplotlib.use('Agg')
import matplotlib.dates as mdates
import matplotlib.pyplot as plt
import numpy as np
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

KST = timezone(timedelta(hours=9))


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bag', type=Path, action='append', required=True)
    parser.add_argument('--end', required=True, help='ISO timestamp with timezone')
    parser.add_argument('--minutes', type=float, default=10.)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    end = datetime.fromisoformat(args.end).timestamp()
    start = end-args.minutes*60
    records, sources = [], []
    for bag in args.bag:
        for path in sorted(bag.glob('*.db3')):
            # One read transaction keeps all selected topics consistent even in an active bag.
            with sqlite3.connect(f'file:{path.resolve()}?mode=ro', uri=True) as db:
                db.execute('PRAGMA query_only=ON')
                db.execute('BEGIN')
                topics = {i: (name, get_message(kind)) for i, name, kind in
                          db.execute('select id,name,type from topics')
                          if name in ('/localization/status', '/cmd_vel', '/rosout')}
                ids = ','.join(map(str, topics))
                if not ids:
                    continue
                digest = hashlib.sha256()
                query = f'select topic_id,timestamp,data from messages where topic_id in ({ids}) and timestamp between ? and ? order by timestamp,id'
                for topic_id, stamp, payload in db.execute(query, (int(start*1e9), int(end*1e9))):
                    name, cls = topics[topic_id]
                    digest.update(f'{name}:{stamp}:'.encode())
                    digest.update(payload)
                    message = deserialize_message(payload, cls)
                    row = {'received': stamp/1e9, 'topic': name}
                    if name == '/localization/status':
                        row['data'] = json.loads(message.data)
                    elif name == '/cmd_vel':
                        row['data'] = {'vx': message.linear.x, 'vy': message.linear.y,
                                       'wz': message.angular.z}
                    else:
                        if not any(s in message.msg for s in
                                   ('Goal canceled', 'initialPoseReceived')):
                            continue
                        row['data'] = {'node': message.name, 'message': message.msg}
                    records.append(row)
                sources.append({'path': str(path.resolve()), 'selected_messages_sha256': digest.hexdigest()})
    records.sort(key=lambda r: r['received'])
    args.output.mkdir(parents=True, exist_ok=True)
    (args.output/'recorded-inputs.json').write_text(json.dumps(records, indent=2))
    rows = [r for r in records if r['topic'] == '/localization/status']
    if not rows:
        raise RuntimeError('No recorded localization status in requested interval')
    # A repeated status can retain an old score. Do not draw stale evidence as fresh.
    valid = lambda r: r['data'].get('score') and 0 <= r['received']-r['data'].get('stamp', 0) <= 1.
    with (args.output/'localization.csv').open('w') as stream:
        writer = csv.writer(stream)
        writer.writerow(['received_kst', 'scan_stamp', 'state', 'inlier_percent', 'trimmed_mean_cm',
                         'score_fresh', 'speed_limit_percent', 'reason', 'phase', 'recovery_reason'])
        for r in rows:
            d = r['data']; score = d.get('score', {})
            writer.writerow([datetime.fromtimestamp(r['received'], KST).isoformat(), d.get('stamp'),
                d.get('state'), score.get('inlier_ratio', float('nan'))*100,
                score.get('mean_distance', float('nan'))*100, bool(valid(r)),
                d.get('speed_limit_percentage'), d.get('reconvergence_reason'), d.get('phase'),
                d.get('recovery_reason')])
    x, ratio, error = [], [], []
    previous = None
    for r in rows:
        if previous is not None and r['received']-previous > 1.:
            x.append(datetime.fromtimestamp(previous+.01, KST));ratio.append(np.nan);error.append(np.nan)
        x.append(datetime.fromtimestamp(r['received'], KST))
        score = r['data'].get('score', {})
        ratio.append(score['inlier_ratio']*100 if valid(r) else np.nan)
        error.append(score['mean_distance']*100 if valid(r) else np.nan)
        previous = r['received']
    plt.rcParams.update({'font.size': 11, 'axes.spines.top': False, 'axes.spines.right': False})
    fig, axes = plt.subplots(3, 1, figsize=(17, 9), sharex=True,
                             gridspec_kw={'height_ratios': [2.2, 1.6, 1]}, layout='constrained')
    axes[0].plot(x, ratio, color='#146ea8', lw=1.25, label='Recorded scan/map inliers')
    axes[0].axhline(30, color='#c66d21', ls='--', label='Normal minimum: 30%')
    axes[0].set(ylabel='Alignment ratio (%)', ylim=(0, 100))
    axes[0].legend(loc='upper left', ncol=2)
    axes[1].plot(x, error, color='#405e7d', lw=1., label='Trimmed mean endpoint distance')
    axes[1].axhline(18, color='#c66d21', ls='--', label='Normal maximum: 18 cm')
    axes[1].set(ylabel='Mean error (cm)', ylim=(0, max(30, np.nanmax(error)*1.12)))
    axes[1].legend(loc='upper left', ncol=2)
    colors = {'degraded': '#f4c56b', 'unavailable': '#e87676', 'unknown': '#b6bac1'}
    for i, r in enumerate(rows):
        right = min(rows[i+1]['received'] if i+1<len(rows) else end, r['received']+1.)
        color = colors.get(r['data'].get('state'))
        if color:
            for ax in axes[:2]:
                ax.axvspan(datetime.fromtimestamp(r['received'], KST),
                           datetime.fromtimestamp(right, KST), color=color, alpha=.25, lw=0)
    commands = [r for r in records if r['topic'] == '/cmd_vel']
    cx, cv = [], []
    last = None
    for r in commands:
        if last is not None and r['received']-last > .3:
            cx.append(datetime.fromtimestamp(last+.01, KST));cv.append(np.nan)
        cx.append(datetime.fromtimestamp(r['received'], KST))
        cv.append(math.hypot(r['data']['vx'], r['data']['vy']))
        last = r['received']
    axes[2].plot(cx, cv, color='#008578', lw=1., label='Output translation command (not measured motion)')
    axes[2].set(ylabel='Command (m/s)', ylim=(0, max(.55, max(cv, default=0)*1.1)))
    axes[2].legend(loc='upper left')
    cancellations = [r for r in records if r['topic']=='/rosout' and
                     r['data']['message']=='Goal canceled']
    for j, r in enumerate(cancellations):
        t = datetime.fromtimestamp(r['received'], KST)
        for ax in axes:
            ax.axvline(t, color='#b02e3c', ls=':', lw=1.2)
        axes[0].annotate('Goal canceled '+t.strftime('%H:%M:%S'), xy=(t, 72-j*9),
                         xytext=(-10, 0), textcoords='offset points', ha='right',
                         fontsize=9, color='#a12433')
    for ax in axes:
        ax.grid(alpha=.15)
        ax.set_xlim(datetime.fromtimestamp(start,KST), datetime.fromtimestamp(end,KST))
    axes[2].xaxis.set_major_locator(mdates.MinuteLocator(interval=1, tz=KST))
    axes[2].xaxis.set_major_formatter(mdates.DateFormatter('%H:%M:%S', tz=KST))
    axes[2].set_xlabel('Recorded receipt time (KST); blank lines indicate missing or stale samples')
    period = datetime.fromtimestamp(start,KST).strftime('%H:%M:%S')+' – '+datetime.fromtimestamp(end,KST).strftime('%H:%M:%S')
    fig.suptitle('Cleany localization history | 2026-11-08 | '+period+
                 '\nBackground: yellow = degraded, red = unavailable, grey = unknown', fontsize=16)
    fig.savefig(args.output/'localization-10min.png', dpi=170)
    fig.savefig(args.output/'localization-10min.pdf')
    plt.close(fig)
    manifest = {'start': datetime.fromtimestamp(start,KST).isoformat(),
                'end': datetime.fromtimestamp(end,KST).isoformat(), 'sources': sources,
                'status_samples': len(rows), 'fresh_score_samples': sum(bool(valid(r)) for r in rows),
                'script_sha256': hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                'boundary': 'Recorded status scores, not recomputed localization; no ROS publication',
                'cancel_events': cancellations}
    (args.output/'manifest.json').write_text(json.dumps(manifest,indent=2))
    (args.output/'index.html').write_text(
        '<!doctype html><html lang="ko"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">'
        '<title>위치 정합 10분 기록</title><style>body{font:16px/1.7 system-ui;background:#eef2f7;color:#172033;margin:24px auto;max-width:1500px;padding:20px}img{width:100%;background:white}a{color:#007b79}</style>'
        '<h1>위치 정합 10분 기록</h1><p>2026-11-08 '+period+' KST. 이전 주행과 마지막 정지 구간을 포함한다.</p>'
        '<p>위: 지도–스캔 일치율과 정상 하한 30%. 가운데: 절사 평균 오차와 정상 상한 18cm. '
        '아래: 출력된 병진 속도 명령이며 실제 차체 속도는 아니다. 배경 노랑은 감속, 빨강은 주행 불가, 회색은 미검증이다. '
        '붉은 세로 점선은 목표 취소이며 사용자 취소도 포함한다. 기록 공백과 오래된 점수는 연결하지 않았다.</p>'
        '<p>65%는 새 복구 후보의 별도 검증 빔에 적용하는 기준이라 현재 위치의 정상 기준선으로 표시하지 않았다.</p>'
        '<a href="localization-10min.png">원본 PNG</a> · <a href="localization-10min.pdf">PDF</a> · '
        '<a href="localization.csv">CSV</a> · <a href="manifest.json">출처</a>'
        '<p><img src="localization-10min.png" alt="10분간 위치 정합 비율, 평균 오차, 출력 속도 명령"></p></html>')
    print(json.dumps(manifest, ensure_ascii=False))


if __name__ == '__main__':
    main()
