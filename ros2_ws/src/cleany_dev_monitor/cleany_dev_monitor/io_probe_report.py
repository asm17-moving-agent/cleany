"""Render passive IO evidence with execution stages, gaps and resource timelines."""
from __future__ import annotations

import argparse
from collections import defaultdict
import csv
import json
from pathlib import Path


def read_recording(directory: Path) -> tuple[dict, dict]:
    manifest = json.loads((directory/'manifest.json').read_text())
    records = defaultdict(list)
    with (directory/'events.jsonl').open() as stream:
        for line in stream:
            record = json.loads(line)
            records[record['kind']].append(record)
    for rows in records.values():
        rows.sort(key=lambda row: row['t'])
    return manifest, records


def stage_segments(records: dict, end: float) -> list[dict]:
    segments = []
    for row in records['window']:
        identity = tuple(row.get(k, '') for k in ('execution_id', 'stage', 'substage'))
        if segments and segments[-1]['identity'] == identity:
            segments[-1]['end_s'] = row['t']
        else:
            if segments:
                segments[-1]['end_s'] = row['t']
            segments.append(dict(identity=identity, start_s=row['t'], end_s=row['t']))
    if segments:
        segments[-1]['end_s'] = end
    return segments


def render(directory: Path) -> dict:
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib import font_manager
    from matplotlib.backends.backend_pdf import PdfPages
    import numpy as np

    manifest, records = read_recording(directory)
    config = manifest['config']
    duration = manifest.get('duration_s', max((r['t'] for rows in records.values() for r in rows), default=0))
    segments = stage_segments(records, duration)
    output = directory/'report'
    output.mkdir(exist_ok=True)
    font = Path('/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc')
    if font.exists():
        plt.rcParams['font.family'] = font_manager.FontProperties(fname=str(font)).get_name()
    plt.rcParams.update({'axes.unicode_minus': False, 'axes.spines.top': False, 'axes.spines.right': False})
    arrivals, windows = defaultdict(list), defaultdict(list)
    for row in records['arrival']:
        arrivals[row['topic']].append(row)
    for row in records['window']:
        for stream in row['streams']:
            windows[stream['topic']].append(dict(t=row['t'], **stream))
    event_fields = ['topic', 't', 'sequence', 'payload_bytes', 'gap_ms', 'active_gap_ms', 'capture_state',
                    'stamp_ns', 'stamp_age_ms', 'duplicate_stamp', 'feedback_max_age_ms', 'feedback_changed',
                    'protocol_error_max', 'motor_status_max', 'recovery_count_max', 'decode_error']
    with (output/'message-arrivals.csv').open('w') as stream:
        writer = csv.DictWriter(stream, fieldnames=event_fields, extrasaction='ignore')
        writer.writeheader(); writer.writerows(records['arrival'])
    with (output/'windows.csv').open('w') as stream:
        writer = csv.writer(stream)
        writer.writerow(['elapsed_s', 'execution_id', 'stage', 'substage', 'topic', 'hz', 'status',
                         'last_receipt_age_ms', 'current_source_age_ms', 'timer_lag_ms'])
        for row in records['window']:
            for sample in row['streams']:
                writer.writerow([row['t'], row.get('execution_id'), row['stage'], row.get('substage'),
                                 sample['topic'], sample['hz'], sample['status'], sample['last_receipt_age_ms'],
                                 sample['current_source_age_ms'], row['timer_lag_ms']])
    # Retain exact phase events, including pre-Action command history and terminal results.
    (output/'phase-events.json').write_text(json.dumps(
        {key: records[key] for key in ('execution', 'command', 'bt', 'wrist')}, ensure_ascii=False, indent=2))
    live_executions = [row['record'] for row in records['execution'] if not row.get('historical', False)]
    terminal = {row['execution_id']: row for row in live_executions if row.get('has_result')}
    evidence_complete = bool(manifest.get('finished') and manifest.get('writer', {}).get('complete'))
    status = ('SYNTHETIC_TEST' if manifest.get('synthetic') else
              'INCOMPLETE_RECORDING' if not evidence_complete else
              'NO_EXECUTION_RECORDED' if not live_executions else
              'NO_TERMINAL_RESULT' if not terminal else 'REVIEW_REQUIRED')
    metrics = {}
    for spec in config['streams']:
        topic = spec['topic']
        values = arrivals[topic]
        gaps = [row['active_gap_ms'] for row in values if row.get('active_gap_ms') is not None]
        active = [row['hz'] for row in windows[topic] if row['status'] in ('OBSERVED', 'NO_DATA') and row['hz'] is not None]
        metrics[topic] = dict(count=len(values), active_window_min_hz=min(active) if active else None,
                             active_max_gap_ms=max(gaps) if gaps else None,
                             active_zero_windows=sum(value == 0 for value in active),
                             unknown_windows=sum(row['status'] == 'UNKNOWN' for row in windows[topic]),
                             off_windows=sum(row['status'] == 'OFF' for row in windows[topic]),
                             missing_source_stamps=sum(row.get('stamp_ns') is None for row in values),
                             decode_errors=sum('decode_error' in row for row in values))
    summary = dict(status=status, evidence_complete=evidence_complete, metrics=metrics,
                   execution_ids=sorted({r['execution_id'] for r in live_executions}), terminal_results=terminal,
                   udp_rcvbuf_errors_delta=manifest.get('udp_after', {}).get('RcvbufErrors', 0)-manifest.get('udp_before', {}).get('RcvbufErrors', 0),
                   max_probe_tick_lag_ms=max((r['timer_lag_ms'] for r in records['window']), default=None),
                   tegrastats_samples=len(records['tegrastats']),
                   scope='Receiver observations only; physical grasp/placement and unattended safety are not certified.')
    (output/'summary.json').write_text(json.dumps(summary, ensure_ascii=False, indent=2))
    with (output/'phase-metrics.csv').open('w') as stream:
        writer = csv.writer(stream)
        writer.writerow(['execution_id', 'stage', 'substage', 'start_s', 'end_s', 'topic',
                         'active_windows', 'minimum_hz', 'maximum_active_gap_ms'])
        for segment in segments:
            for spec in config['streams']:
                topic = spec['topic']
                selected = [r['hz'] for r in windows[topic] if segment['start_s'] <= r['t'] < segment['end_s']
                            and r['status'] in ('OBSERVED', 'NO_DATA') and r['hz'] is not None]
                gaps = [r['active_gap_ms'] for r in arrivals[topic] if segment['start_s'] <= r['t'] < segment['end_s']
                        and r.get('active_gap_ms') is not None]
                writer.writerow([*segment['identity'], segment['start_s'], segment['end_s'], topic,
                                 len(selected), min(selected) if selected else None, max(gaps) if gaps else None])

    def heading(fig, title, note):
        fig.suptitle(title, x=.07, ha='left', fontsize=20, weight='bold')
        when = manifest.get('start', '')[:19].replace('T', ' ')
        fig.text(.07, .925, f"{manifest['label']} · {when} · {status} · {duration:.1f}초", size=10)
        fig.text(.07, .025, note, fontsize=9, linespacing=1.6)

    def phase_axis(ax):
        colors = plt.get_cmap('tab20')
        visible = [s for s in segments if s['identity'][1] not in ('NO_EXECUTION', 'TERMINAL')]
        for i, segment in enumerate(visible):
            x, end = segment['start_s'], segment['end_s']
            ax.barh(0, end-x, left=x, height=.7, color=colors(i % 20))
            ax.text(x, .6+(i%2)*.55, segment['identity'][2] or segment['identity'][1],
                    rotation=30, fontsize=7, ha='left', va='bottom')
        ax.set_xlim(0, max(duration, 1)); ax.set_ylim(-.6, 3.4)
        ax.set_yticks([]); ax.set_xlabel('기록 시작 후 경과 (초)')
        if not visible:
            ax.text(.5, .3, '기록된 활성 작업 단계 없음', transform=ax.transAxes, ha='center', color='#64748b')

    with PdfPages(output/'stability-timeseries.pdf') as pdf:
        main = [s for s in config['streams'] if s['kind'] != 'info']
        for specs, filename, title in ((main, 'reception.png', '파지 작업 중 수신 주파수와 간격'),
                                       ([s for s in config['streams'] if s['kind'] == 'info'], 'camera-info.png', '카메라 정보 토픽 수신')):
            if not specs:
                continue
            fig, axes = plt.subplots(len(specs)+1, 2, figsize=(16, 2*len(specs)+3), squeeze=False,
                                      gridspec_kw={'height_ratios': [1]*len(specs)+[.9]})
            fig.subplots_adjust(top=.87, bottom=.11, hspace=.7, wspace=.24)
            heading(fig, title,
                    f"왼쪽: 직전 {config['window_s']:g}초 수신 빈도, {config['sample_interval_s']*1000:g}ms 갱신 / 오른쪽: 활성 상태가 이어진 메시지별 간격, 평활화 없음.\n"
                    '회색 점: 확인된 손목 OFF / 주황 점: 활성 상태 불명. OFF 전후 간격은 성능 저하로 합산하지 않습니다.\n'
                    '단계는 수신 시점 기준이며 기록기 추가 구독의 부하를 포함합니다. 구독자 결과를 센서 촬영·모터 버스 주기로 해석하지 않습니다.')
            for i, spec in enumerate(specs):
                topic = spec['topic']; a, b = axes[i]
                rows = windows[topic]
                a.step([r['t'] for r in rows], [r['hz'] if r['hz'] is not None else np.nan for r in rows],
                       where='post', lw=1, color='#0d9488')
                for mode, color in (('OFF', '#94a3b8'), ('UNKNOWN', '#ea580c')):
                    selected = [r for r in rows if r['status'] == mode]
                    a.scatter([r['t'] for r in selected], [0]*len(selected), s=4, c=color)
                points = [r for r in arrivals[topic] if r.get('active_gap_ms') is not None]
                b.plot([r['t'] for r in points], [r['active_gap_ms'] for r in points], '.', ms=2, color='#0d9488')
                for ax, target, units in ((a, spec['hz'], 'Hz'), (b, 1000/spec['hz'], 'ms')):
                    ax.axhline(target, color='#64748b', ls='--', lw=.8)
                    ax.set_title(spec['label'], loc='left', fontsize=11)
                    ax.set_ylabel(units); ax.set_xlim(0, max(duration, 1)); ax.set_ylim(bottom=0)
                    ax.grid(alpha=.18)
            phase_axis(axes[-1, 0]); phase_axis(axes[-1, 1])
            fig.savefig(output/filename, dpi=140); pdf.savefig(fig); plt.close(fig)

        fig, axes = plt.subplots(6, 1, figsize=(15, 14))
        fig.subplots_adjust(top=.87, bottom=.11, hspace=.65)
        heading(fig, '신선도·지연과 시스템 자원',
                '영상: 마지막 수신 원본의 현재 나이 / 모터: 수신 시 실제 피드백 나이. 손목 stamp는 host acquisition 시각입니다.\n'
                'UDP 오류는 호스트 전체입니다. 기록기 timer 지연·writer 누락은 계측 신뢰도를 평가하는 별도 증거입니다.\n'
                'GPU 표본이 없으면 측정 불가로 남깁니다. 작업 완료나 ASSUMED 안착을 실제 파지·안착 성공으로 바꾸지 않습니다.')
        for spec in main:
            if spec['kind'] == 'joint': continue
            if spec['kind'] == 'motor':
                key = 'feedback_max_age_ms'
                points = [r for r in arrivals[spec['topic']] if r.get(key) is not None]
            else:
                key = 'current_source_age_ms'
                points = [r for r in windows[spec['topic']] if r.get(key) is not None and r['status'] != 'OFF']
            axes[0].plot([r['t'] for r in points], [r[key] for r in points], '.', ms=1.5, label=spec['label'])
        axes[0].set_ylabel('source age ms'); axes[0].legend(ncol=3, fontsize=8)
        res = records['resources']; xs = [r['t'] for r in res]
        process_label = '합성 발행기+기록기' if manifest.get('shared_process_fixture') else '기록기'
        axes[1].plot(xs, [r['cpu_percent'] for r in res], label='호스트 CPU')
        axes[1].plot(xs, [r['probe_cpu_percent_one_core'] for r in res], label=process_label+' CPU / 1 core')
        gpu = [r for r in records['tegrastats'] if r['gpu_percent'] is not None]
        if gpu: axes[1].plot([r['t'] for r in gpu], [r['gpu_percent'] for r in gpu], label='GPU GR3D')
        axes[1].legend(ncol=3, fontsize=8); axes[1].set_ylabel('%')
        axes[2].plot(xs, [r['memory_used_bytes']/2**30 for r in res], label='호스트 사용 RAM')
        axes[2].plot(xs, [r['probe_rss_bytes']/2**30 for r in res], label=process_label+' RSS')
        axes[2].set_ylabel('GiB'); axes[2].legend(fontsize=8)
        base = manifest.get('udp_before', {}).get('RcvbufErrors', 0)
        axes[3].step(xs, [r['udp'].get('RcvbufErrors', base)-base for r in res], where='post', label='UDP RcvbufErrors 증가')
        axes[3].set_ylabel('건'); axes[3].legend(fontsize=8)
        axes[4].plot([r['t'] for r in records['window']], [r['timer_lag_ms'] for r in records['window']])
        axes[4].set_ylabel('계측 timer 지연 ms')
        for ax in axes[:5]: ax.set_xlim(0, max(duration, 1)); ax.grid(alpha=.2)
        phase_axis(axes[5])
        fig.savefig(output/'resources-and-age.png', dpi=140); pdf.savefig(fig); plt.close(fig)
    print(json.dumps(dict(output=str(output), **summary), ensure_ascii=False, indent=2))
    return summary


def main(args=None) -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory', type=Path)
    options = parser.parse_args(args)
    render(options.directory)


if __name__ == '__main__':
    main()
