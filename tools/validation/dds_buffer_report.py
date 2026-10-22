#!/usr/bin/env python3
"""Render timed DDS buffer comparisons, retaining startup and observer caveats."""
from __future__ import annotations

import argparse
from collections import Counter, defaultdict
import csv
import html
import json
from pathlib import Path
import statistics


def percentile(values: list[float], fraction: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    position = (len(ordered)-1)*fraction
    lower = int(position)
    upper = min(lower+1, len(ordered)-1)
    return ordered[lower]+(ordered[upper]-ordered[lower])*(position-lower)


def summarize(manifest: dict, records: dict, warmup: float, end: float) -> dict:
    """Exclude warmup from arrival metrics and entire rolling windows."""
    duration = end-warmup
    metrics = {}
    for spec in manifest['config']['streams']:
        arrivals = [r for r in records['arrival'] if r['topic'] == spec['topic'] and warmup <= r['t'] <= end]
        windows = [s for r in records['window'] if warmup+manifest['config']['window_s'] <= r['t'] <= end
                   for s in r['streams'] if s['topic'] == spec['topic']]
        states = Counter(r['status'] for r in windows)
        hz = [s['hz'] for s in windows if s['status'] in ('OBSERVED', 'NO_DATA') and s['hz'] is not None]
        gaps = [r['gap_ms'] for r in arrivals if r.get('gap_ms') is not None and r['t']-r['gap_ms']/1000 >= warmup]
        ages = [r['stamp_age_ms'] for r in arrivals if r.get('stamp_age_ms') is not None]
        boundaries = [warmup]+[r['t'] for r in arrivals]+[end]
        idle_off = bool(windows) and set(states) == {'OFF'}
        metrics[spec['topic']] = dict(label=spec['label'], target_hz=spec['hz'], count=len(arrivals),
            status='OFF' if idle_off else ('OBSERVED' if hz else 'UNKNOWN'), states=dict(states),
            mean_hz=None if idle_off else len(arrivals)/duration,
            minimum_rolling_hz=min(hz) if hz else None, maximum_rolling_hz=max(hz) if hz else None,
            p05_rolling_hz=percentile(hz, .05), max_gap_ms=max(gaps) if gaps else None,
            max_silence_ms=None if idle_off else max((b-a)*1000 for a, b in zip(boundaries, boundaries[1:])),
            p99_stamp_age_ms=percentile(ages, .99), max_stamp_age_ms=max(ages) if ages else None,
            decode_errors=sum('decode_error' in r for r in arrivals))
    socket_samples = records['udp_sockets']
    tail = socket_samples[-1] if socket_samples else None
    baseline = next((r for r in socket_samples if r['t'] >= warmup), None)
    drops_full = sum(s['drops'] for s in tail['sockets']) if tail else None
    drops_measured = None
    if baseline and tail and {s['inode'] for s in baseline['sockets']} == {s['inode'] for s in tail['sockets']}:
        drops_measured = drops_full-sum(s['drops'] for s in baseline['sockets'])
    resources = [r for r in records['resources'] if warmup <= r['t'] <= end]
    cpu = [r['cpu_percent'] for r in resources]
    probe_cpu = [r['probe_cpu_percent_one_core'] for r in resources]
    gpu = [r['gpu_percent'] for r in records['tegrastats']
           if warmup <= r['t'] <= end and r.get('gpu_percent') is not None]
    active_events = [r for r in records['execution'] if not r.get('historical') and
                     r.get('record', {}).get('record_state') == 'ACTIVE']
    active_commands = [r for r in records['command'] if r.get('record', {}).get('owner_alive') and
                       r.get('record', {}).get('state') == 'ACTIVE']
    wrist_active = any(r['enabled'] for r in records['wrist'])
    joint_sets = [set(r.get('joints', [])) for r in records['arrival']
                  if r['topic'] == '/joint_states' and warmup <= r['t'] <= end]
    common_joints = sorted(set.intersection(*joint_sets)) if joint_sets else []
    return dict(metrics=metrics, measured_duration_s=duration, socket_drops_full=drops_full,
                socket_drops_after_warmup=drops_measured,
                socket_drop_interval_s=[baseline['t'], tail['t']] if baseline and tail else None,
                host_udp_drops_full=manifest['udp_after']['RcvbufErrors']-manifest['udp_before']['RcvbufErrors'],
                socket_actual_bytes=sorted({s['rcvbuf_bytes'] for s in tail['sockets']}) if tail else [],
                cpu_mean_percent=statistics.mean(cpu) if cpu else None, cpu_max_percent=max(cpu) if cpu else None,
                observer_cpu_mean_one_core_percent=statistics.mean(probe_cpu) if probe_cpu else None,
                gpu_mean_percent=statistics.mean(gpu) if gpu else None, gpu_max_percent=max(gpu) if gpu else None,
                observer_rss_max_bytes=max((r['probe_rss_bytes'] for r in resources), default=None),
                memory_used_max_bytes=max((r['memory_used_bytes'] for r in resources), default=None),
                joint_names_in_every_position_message=common_joints,
                idle_observed=not active_events and not active_commands and not wrist_active,
                complete=bool(manifest.get('finished') and manifest.get('writer', {}).get('complete') and
                              manifest.get('duration_s', 0) >= end),
                writer=manifest.get('writer'))


def load(directory: Path) -> tuple[dict, dict]:
    manifest = json.loads((directory/'manifest.json').read_text())
    records = defaultdict(list)
    for line in (directory/'events.jsonl').read_text().splitlines():
        row = json.loads(line)
        records[row['kind']].append(row)
    for rows in records.values():
        rows.sort(key=lambda r: r['t'])
    return manifest, records


def size_label(kib: int) -> str:
    return f'{kib//1024} MiB' if kib % 1024 == 0 else f'{kib} KiB'


def render(directory: Path) -> dict:
    import matplotlib
    matplotlib.use('Agg')
    from matplotlib import pyplot as plt, font_manager
    from matplotlib.backends.backend_pdf import PdfPages

    font = Path('/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc')
    if font.exists():
        font_manager.fontManager.addfont(str(font))
        plt.rcParams['font.family'] = font_manager.FontProperties(fname=str(font)).get_name()
    plt.rcParams.update({'axes.unicode_minus': False, 'font.size': 10, 'axes.spines.top': False,
                         'axes.spines.right': False, 'axes.grid': True, 'grid.alpha': .2})
    plan = json.loads((directory/'sweep.json').read_text())
    output = directory/'report'
    output.mkdir(exist_ok=True)
    runs, data = [], {}
    for run in plan['runs']:
        if run['returncode']:
            continue
        manifest, records = load(directory/run['label'])
        item = dict(**run, **summarize(manifest, records, plan['warmup_s'], plan['seconds']),
                    start=manifest['start'], observation_start_wall_ns=manifest['observation_start_wall_ns'])
        runs.append(item)
        data[run['label']] = records
    zero_drop_sizes = [size for size in plan['sizes_kib']
                       if len([r for r in runs if r['size_kib'] == size]) == plan['rounds']
                       and all(r['socket_drops_full'] == 0 and r['complete'] and r['idle_observed']
                               for r in runs if r['size_kib'] == size)]
    summary = dict(plan=plan, runs=runs,
                   minimum_observed_zero_drop_size_kib=min(zero_drop_sizes) if zero_drop_sizes else None,
                   scope='Additional host Best Effort receiver. Production perception Reliable receiver unchanged. No grasp validation.',
                   rolling_hz='1-second window sampled every 0.1s; windows entirely after warmup',
                   socket_drops='Per-observer UDP datagrams, not image frames; full count includes discovery/warmup',
                   limitations=['Sequential idle tests only; no inference workload or enabled wrists',
                                'No publication-side frame sequence; not an exact delivery percentage',
                                'Source age includes capture, processing, scheduling and transport',
                                'Kernel rmem_max unchanged; only observer per-socket buffer requests vary'])
    (output/'summary.json').write_text(json.dumps(summary, ensure_ascii=False, indent=2))
    with (output/'metrics.csv').open('w') as file:
        fields = ['label', 'round', 'size_kib', 'topic', 'status', 'mean_hz', 'minimum_rolling_hz',
                  'maximum_rolling_hz', 'max_gap_ms', 'max_silence_ms', 'p99_stamp_age_ms', 'count']
        writer = csv.DictWriter(file, fieldnames=fields, extrasaction='ignore')
        writer.writeheader()
        for run in runs:
            for topic, metric in run['metrics'].items():
                writer.writerow(dict(metric, label=run['label'], round=run['round'], size_kib=run['size_kib'], topic=topic))
    with (output/'timeseries.csv').open('w') as file:
        writer = csv.writer(file)
        writer.writerow(['run', 'size_kib', 'round', 'elapsed_s', 'wall_ns', 'topic', 'hz_1s_window', 'status', 'source_age_ms'])
        for run in runs:
            for row in data[run['label']]['window']:
                for stream in row['streams']:
                    writer.writerow([run['label'], run['size_kib'], run['round'], row['t'],
                                     run['observation_start_wall_ns']+int(row['t']*1e9), stream['topic'],
                                     stream['hz'], stream['status'], stream['current_source_age_ms']])
    figures = []
    groups = [
        ('head-hz', '헤드캠 실시간 수신 주파수', ['/camera/color/image_raw', '/camera/depth/image_raw'], ['컬러', '정렬 깊이'], 15),
        ('motor-hz', '모터 1–14 토픽 실시간 수신 주파수', ['/joint_states', '/joint_state_broadcaster/dynamic_joint_states'], ['위치', '원시 피드백'], 50),
        ('camera-info-hz', '헤드캠 CameraInfo 수신 주파수', ['/camera/color/camera_info', '/camera/depth/camera_info'], ['컬러 정보', '깊이 정보'], 15),
    ]
    sizes = plan['sizes_kib']
    for filename, title, topics, labels, target in groups:
        fig, axes = plt.subplots((len(sizes)+2)//3, 3, figsize=(16, 8), squeeze=False, sharex=True, sharey=True)
        for ax, size in zip(axes.flat, sizes):
            for run in (r for r in runs if r['size_kib'] == size):
                records = data[run['label']]
                for topic, label, color in zip(topics, labels, ('#1673b8', '#d17719')):
                    points = [(r['t']-plan['warmup_s'], s['hz']) for r in records['window']
                              if plan['warmup_s']+1 <= r['t'] <= plan['seconds']
                              for s in r['streams'] if s['topic'] == topic and s['hz'] is not None]
                    if points:
                        ax.plot(*zip(*points), color=color, linestyle='-' if run['round'] % 2 else '--',
                                linewidth=1.2, alpha=.85, label=f'{label} / {run["round"]}회')
            ax.axhline(target, color='#444444', linestyle=':', linewidth=1)
            ax.set_title(size_label(size), fontweight='bold')
            ax.set_ylim(0, target*1.12)
            ax.set_xlim(0, plan['seconds']-plan['warmup_s'])
            ax.set_ylabel('수신 Hz (최근 1초)')
            ax.set_xlabel('측정 경과 시간 (초)')
        for ax in list(axes.flat)[len(sizes):]:
            ax.set_visible(False)
        handles, labels = axes.flat[0].get_legend_handles_labels()
        fig.legend(handles, labels, loc='lower center', ncol=4, frameon=False)
        fig.suptitle(title+' · 순방향 / 역방향 반복', fontsize=18, fontweight='bold')
        fig.text(.5, .93, f'초기 {plan["warmup_s"]:g}초 제외 · 1초 이동 구간, 0.1초 간격 · 수신 계측기만 버퍼 변경', ha='center', fontsize=11)
        fig.tight_layout(rect=(0, .06, 1, .91))
        fig.savefig(output/f'{filename}.png', dpi=160)
        figures.append(fig)
    fig, axes = plt.subplots(2, 2, figsize=(16, 9))
    colors = {s: plt.get_cmap('tab10')(i) for i, s in enumerate(sizes)}
    for run in runs:
        records = data[run['label']]
        style = '-' if run['round'] % 2 else '--'
        color = colors[run['size_kib']]
        sockets = records['udp_sockets']
        axes[0, 0].plot([r['t'] for r in sockets], [sum(s['drops'] for s in r['sockets']) for r in sockets],
                        color=color, linestyle=style, label=f'{size_label(run["size_kib"])} / {run["round"]}회')
        resources = records['resources']
        axes[0, 1].plot([r['t'] for r in resources], [r['cpu_percent'] for r in resources], color=color, linestyle=style)
        for idx, topic in enumerate(('/camera/color/image_raw', '/camera/depth/image_raw')):
            rows = [r for r in records['arrival'] if r['topic'] == topic and plan['warmup_s'] <= r['t'] <= plan['seconds']]
            axes[1, idx].plot([r['t'] for r in rows], [r['stamp_age_ms'] for r in rows], color=color, linestyle=style, linewidth=.8, alpha=.65)
    for ax, title, ylabel in zip(axes.flat, ['계측기 소켓 누적 UDP 유실 (초기 연결 포함)', '전체 CPU 사용률', '헤드 컬러 메시지 도착 시 age', '헤드 깊이 메시지 도착 시 age'], ['유실 UDP datagrams', '%', 'ms', 'ms']):
        ax.set_title(title)
        ax.set_xlabel('프로세스 시작 후 경과 시간 (초)')
        ax.set_ylabel(ylabel)
        ax.axvspan(0, plan['warmup_s'], color='gray', alpha=.08)
    handles, labels = axes[0, 0].get_legend_handles_labels()
    fig.legend(handles, labels, loc='lower center', ncol=6, frameon=False, fontsize=9)
    fig.suptitle('버퍼 유실과 시스템 부하 · 실시간 원본', fontsize=18, fontweight='bold')
    fig.tight_layout(rect=(0, .09, 1, .94))
    fig.savefig(output/'drops-resources-age.png', dpi=160)
    figures.append(fig)
    with PdfPages(output/'comparison.pdf') as pdf:
        for fig in figures:
            pdf.savefig(fig)
            plt.close(fig)
    rows = []
    def metric_range(run, topic):
        m = run['metrics'][topic]
        lo, hi = m['minimum_rolling_hz'], m['maximum_rolling_hz']
        return '—' if lo is None else f'{lo:.0f}–{hi:.0f}'
    for run in sorted(runs, key=lambda r: (r['size_kib'], r['round'])):
        gap = run['metrics']['/camera/color/image_raw']['max_silence_ms']
        rows.append(f'<tr><td>{size_label(run["size_kib"])}</td><td>{run["round"]}</td>'+
                    ''.join(f'<td>{metric_range(run, t)}</td>' for t in [groups[1][2][0], groups[1][2][1], *groups[0][2]])+
                    f'<td>{gap:.1f}</td><td>{run["socket_drops_full"]:,}</td><td>{run["cpu_mean_percent"]:.1f}</td></tr>')
    valid = all(r['complete'] and r['idle_observed'] for r in runs) and plan['finished']
    candidate = summary['minimum_observed_zero_drop_size_kib']
    finding = (f'이번 대기 시험에서 두 회차 모두 UDP 유실이 0인 가장 작은 요청값은 <b>{size_label(candidate)}</b>입니다. '
               '실제 파지 시험의 후보이며 운영 최소값 확정은 아닙니다.') if candidate else '두 회차 모두 유실 0인 후보가 확인되지 않았습니다.'
    wrist_rows = []
    for size in sizes:
        subset = [r for r in runs if r['size_kib'] == size]
        values = []
        for arm in ('left', 'right'):
            for kind in ('image_raw', 'camera_info'):
                topic = f'/{arm}_wrist_camera/{kind}'
                states = sorted({r['metrics'][topic]['status'] for r in subset})
                count = sum(r['metrics'][topic]['count'] for r in subset)
                values.append(f'{" / ".join(states)} ({count}개)')
        wrist_rows.append(f'<tr><td>{size_label(size)}</td>'+''.join(f'<td>{v}</td>' for v in values)+'</tr>')
    body = f'''<!doctype html><html lang="ko"><meta charset="utf-8"><title>DDS 버퍼 단계별 실측</title>
<style>body{{font:16px system-ui,sans-serif;max-width:1350px;margin:40px auto;padding:0 24px;color:#213547;background:#fafbfd}}h1{{font-size:32px}}p{{line-height:1.65}}img{{width:100%;background:white;margin:12px 0}}table{{border-collapse:collapse;width:100%;background:white}}th,td{{padding:10px;border-bottom:1px solid #dae1e8;text-align:right}}th{{background:#eaf0f6}}.note{{padding:18px;background:#eaf0f6;border-radius:8px}}a{{color:#0969a2}}</style>
<h1>DDS 수신 버퍼 단계별 대기 상태 비교</h1>
<p>{html.escape(plan['started'])} · {len(runs)}회 측정 · 기록 완전성 및 대기 조건: {'확인' if valid else '검토 필요'}</p>
<p>{finding}</p>
<p class="note">각 버퍼를 순방향과 역방향으로 측정했습니다. 초기 {plan['warmup_s']:g}초를 제외한 {plan['seconds']-plan['warmup_s']:g}초씩 비교합니다.
아래 Hz는 평균이 아닌 <b>1초 이동 구간 수신 주파수의 최솟값–최댓값</b>입니다. 0.1초 간격 기록입니다.<br>
기존 운영 인식 프로세스와 OS 상한은 8 MiB를 유지하고, 별도 호스트 계측기의 Best Effort 소켓만 변경했습니다.
운영 인식의 Reliable 수신 및 실제 파지 부하에 대한 검증은 후속 시험입니다.</p>
<table><thead><tr><th>요청 버퍼</th><th>회차</th><th>모터 위치 Hz</th><th>원시 피드백 Hz</th><th>헤드 컬러 Hz</th><th>헤드 깊이 Hz</th><th>컬러 최대 공백 ms</th><th>소켓 유실*</th><th>평균 CPU %</th></tr></thead><tbody>{''.join(rows)}</tbody></table>
<p>* 계측기 소켓의 UDP 패킷 누적 유실이며 초기 연결 포함입니다. 영상 프레임 수와 다릅니다.
Linux SO_RCVBUF 실효 표기는 요청값의 2배로 확인합니다. 예: 208 KiB → 416 KiB, 8 MiB → 16 MiB. 메모리 선할당량은 아닙니다.</p>
<h2>헤드캠</h2><img src="head-hz.png"><h2>모터 1–14</h2><img src="motor-hz.png">
<p>집계 토픽 수신 주파수입니다. 개별 모터 하드웨어의 통신 주파수와 동일하지 않습니다. 헤드 모터 13·14는 feedback_stamp_ns가 없어 개별 피드백 갱신 나이 검증 범위에서 제외됩니다.</p>
<h2>CameraInfo</h2><img src="camera-info-hz.png"><h2>패킷 유실, CPU, 메시지 age</h2><img src="drops-resources-age.png">
<p>age는 메시지의 원본 타임스탬프부터 콜백 수신까지의 시간이며 순수 네트워크 지연이 아닙니다.</p>
<h2>양손목 및 전체 원본</h2><p>손목 OFF는 정상 비활성 상태입니다. OFF 구간의 0 Hz를 실패로 집계하지 않습니다. 활성 상태 성능은 실제 파지에서 별도 확인해야 합니다.</p>
<table><thead><tr><th>요청 버퍼</th><th>왼손목 영상</th><th>왼손목 정보</th><th>오른손목 영상</th><th>오른손목 정보</th></tr></thead><tbody>{''.join(wrist_rows)}</tbody></table>
<h2>208 KiB 최적화와 후속 파지 시험</h2>
<p>현재 raw 영상 경로에서 관측된 작은 버퍼의 유실을 줄이려면 전송량이나 순간 패킷 집중을 줄이는 별도 변경이 필요합니다.
우선 검토할 방법은 ROS 컬러 영상의 JPEG 전송과 깊이값을 보존하는 압축입니다.
<a href="https://github.com/ros-perception/image_transport_plugins/blob/humble/compressed_image_transport/package.xml">ROS compressed 이미지 전송</a>은 JPEG/PNG를 지원합니다.
<a href="https://github.com/ros-perception/image_transport_plugins/blob/humble/compressed_depth_image_transport/src/codec.cpp">compressedDepth 구현</a>은 깊이 최대값 필터도 적용하므로, 원본 보존 조건을 확인해야 합니다.
압축 및 복원 시간, CPU, 영상 품질, 깊이값을 함께 측정해야 하며 208 KiB에서의 안정성은 아직 입증되지 않았습니다.
Gemini 요청용 JPEG 변경은 ROS raw 영상의 UDP 전송량을 줄이는 변경과 별개입니다.</p>
<p>후속 시험은 운영 인식 수신기에 후보값을 적용하고 실제 소켓 크기를 확인한 뒤, 같은 물체와 파이프라인에서
사용자가 시작하는 파지를 기록하는 방식입니다. 인식, 계획, 팔 이동, 선택 손목 ON 구간의 Hz, 공백, age와 유실을 비교합니다.
이 대기 시험 중 운영 설정은 유지했으며 실제 파지는 실행하지 않았습니다.</p>
<p><a href="comparison.pdf">그래프 PDF</a> · <a href="metrics.csv">모든 10개 토픽 수치 CSV</a> · <a href="timeseries.csv">실시간 Hz CSV</a> · <a href="summary.json">소켓 실효 크기, 손목 상태, CPU 및 완전성 JSON</a></p>
<p>소스 commit: {html.escape(plan['commit'])} + 작업 트리 변경. 원본: 각 회차 events.jsonl / manifest.json / socket-limits.json.</p></html>'''
    (output/'index.html').write_text(body)
    return summary


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory', type=Path)
    render(parser.parse_args().directory)
