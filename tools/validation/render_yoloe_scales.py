#!/usr/bin/env python3
"""Compare measured n/s/m runs without interpreting detection counts as accuracy."""
from __future__ import annotations

import argparse
from collections import Counter
import csv
from datetime import datetime
import html
import json
from pathlib import Path
import re
import statistics

from benchmark_yoloe_dataset import distribution, write_json


MODELS = ('n', 's', 'm')
DATASETS = ('ocid-cleany-100', 'cleany-real34-saved-20261005')
LABELS = {'all': '전체', DATASETS[0]: 'OCID 100장', DATASETS[1]: '로봇 영상 16장'}


def read(path: Path):
    return json.loads(path.read_text())


def collect(root: Path) -> dict:
    suite = read(root / 'suite.json')
    if suite['status'] != 'complete' or len(suite['runs']) != 18:
        raise ValueError('Expected nine completed adapter and nine native runs')
    models = {}
    for model in MODELS:
        rows, blocks, events, native, native_events = [], [], [], [], []
        for run in range(1, 4):
            directory = root / f'round-{run}' / model
            block = read(directory / 'adapter/blocks.json')
            blocks.extend(dict(item, suite_round=run) for item in block)
            for row in csv.DictReader((directory / 'adapter/timings.csv').open()):
                rows.append(dict(row, suite_round=run, **{key: float(row[key]) for key in
                    ('latency_ms', 'preprocess_ms', 'inference_ms', 'postprocess_ms')}))
            for line in (directory / 'telemetry/telemetry.jsonl').read_text().splitlines():
                event = json.loads(line)
                epoch = datetime.fromisoformat(event['timestamp']).timestamp()
                if any(item['start_epoch'] <= epoch <= item['end_epoch'] for item in block):
                    events.append(event)
            directory = root / 'trtexec' / f'round-{run}' / model
            log = (directory / 'run.log').read_text()
            if '&&&& PASSED TensorRT.trtexec' not in log:
                raise ValueError(f'Native run did not pass: {directory}')
            native.append(dict(round=run, timings=read(directory / 'timings.json'),
                wall_seconds=float(re.search(r'Total Host Walltime: ([\d.]+) s', log).group(1)),
                reported_fps=float(re.search(r'Throughput: ([\d.]+) qps', log).group(1))))
            native_events.extend(json.loads(line) for line in
                (directory / 'telemetry/telemetry.jsonl').read_text().splitlines())
        groups = {}
        for dataset in ('all', *DATASETS):
            selected = [row for row in rows if dataset == 'all' or row['dataset'] == dataset]
            selected_blocks = [block for block in blocks if dataset == 'all' or block['dataset'] == dataset]
            groups[dataset] = dict(requests=len(selected), fps=len(selected)/sum(b['wall_seconds'] for b in selected_blocks),
                latency_ms=distribution([row['latency_ms'] for row in selected]),
                stages_ms={key: distribution([row[key + '_ms'] for row in selected])
                           for key in ('preprocess', 'inference', 'postprocess')})
        resource_keys = ('power_mw', 'gpu_temperature_c', 'cpu_temperature_c', 'ram_used_mb', 'gpu_clock_hz')
        resources = {key: distribution([event[key] for event in events if event.get(key) is not None])
                     for key in resource_keys}
        native_rows = [row for item in native for row in item['timings']]
        native_summary = dict(requests=len(native_rows),
            fps=len(native_rows) / sum(item['wall_seconds'] for item in native),
            gpu_ms=distribution([row['computeMs'] for row in native_rows]),
            host_ms=distribution([row['latencyMs'] for row in native_rows]),
            whole_process_gpu_mhz=distribution([e['gpu_clock_hz']/1e6 for e in native_events
                                                if e.get('gpu_clock_hz') is not None]),
            runs=[{k: v for k, v in item.items() if k != 'timings'} for item in native])
        predictions = read(root / 'round-1' / model / 'adapter/predictions.json')
        environment = read(root / 'round-1' / model / 'adapter/environment.json')
        counts = Counter((row['dataset'], row['id']) for row in rows)
        if len(counts) != 116 or set(counts.values()) != {suite['passes'] * 3}:
            raise ValueError(f'Incomplete input coverage for {model}')
        models[model] = dict(groups=groups, resources=resources, native=native_summary,
            environment=environment, blocks=blocks, predictions=predictions,
            output_counts=dict(images=116, detections=sum(len(p['predictions']) for p in predictions),
                zero_images=sum(not p['predictions'] for p in predictions),
                capped_images=sum(len(p['predictions'])==10 for p in predictions),
                empty_masks=sum(d['mask_pixels']==0 for p in predictions for d in p['predictions'])))
    identities = [read(root / 'round-1' / model / 'adapter/inputs.json')['samples'] for model in MODELS]
    if not identities[0] == identities[1] == identities[2]:
        raise ValueError('Models did not use identical input images')
    return dict(models=models, orders=suite['orders'],
                total_requests=sum(m['groups']['all']['requests'] for m in models.values()))


def charts(root: Path, result: dict) -> None:
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    import numpy as np
    models = result['models']
    colors = ['#008e9b', '#76b900', '#f0aa3e']
    plt.rcParams.update({'font.family': 'DejaVu Sans', 'font.size': 11,
                        'axes.spines.top': False, 'axes.spines.right': False})
    fig, axes = plt.subplots(2, 2, figsize=(13, 8.3), constrained_layout=True)
    fig.suptitle('YOLOE n / s / m | Orin NX 16GB | FP16, batch 1, 640', fontsize=19, fontweight='bold')
    x = np.arange(3)
    for delta,key,label,color in [(-.18,'p50','p50','#76b900'),(.18,'p95','p95','#008e9b')]:
        values = [models[m]['groups']['all']['latency_ms'][key] for m in MODELS]
        bars = axes[0,0].bar(x+delta, values, .35, label=label, color=color)
        axes[0,0].bar_label(bars, fmt='%.1f', padding=3, fontsize=9)
    axes[0,0].set(title='Full adapter latency (lower is faster)', ylabel='Milliseconds', xticks=x, xticklabels=MODELS)
    axes[0,0].legend()
    for delta,key,label,color in [(-.18,'adapter','Full adapter','#76b900'),(.18,'native','Native trtexec','#008e9b')]:
        values = [models[m]['groups']['all']['fps'] if key=='adapter' else models[m]['native']['fps'] for m in MODELS]
        bars = axes[0,1].bar(x+delta,values,.35,label=label,color=color)
        axes[0,1].bar_label(bars,fmt='%.1f',padding=3,fontsize=9)
    axes[0,1].set(title='Throughput (different timing boundaries)', ylabel='Images / second', xticks=x, xticklabels=MODELS)
    axes[0,1].legend()
    bottom = np.zeros(3)
    for key,label,color in [('preprocess','Preprocess','#008e9b'),('inference','TRT call','#76b900'),
                          ('postprocess','Postprocess','#f0aa3e'),('other','Adapter / CPU results','#91a0aa')]:
        values=[]
        for m in MODELS:
            entry=models[m]['groups']['all']; stages=entry['stages_ms']
            values.append(entry['latency_ms']['mean']-sum(s['mean'] for s in stages.values())
                          if key=='other' else stages[key]['mean'])
        axes[1,0].bar(MODELS, values, bottom=bottom, color=color,label=label)
        bottom+=values
    axes[1,0].set(title='Mean time per image',ylabel='Milliseconds')
    axes[1,0].legend(fontsize=8,loc='upper left')
    for m,color in zip(MODELS,colors):
        values=[]
        for r in range(1,4):
            blocks=[b for b in models[m]['blocks'] if b['suite_round']==r]
            values.append(sum(b['requests'] for b in blocks)/sum(b['wall_seconds'] for b in blocks))
        axes[1,1].plot([1,2,3],values,'o-',label=m,color=color,linewidth=2)
    axes[1,1].set(title='Adapter throughput by balanced round',xlabel='Round',ylabel='Images / second',xticks=[1,2,3],ylim=(0,None))
    axes[1,1].legend()
    fig.savefig(root/'comparison.png',dpi=170)
    fig.savefig(root/'comparison.svg')
    plt.close(fig)


def render(root: Path) -> None:
    result = collect(root)
    models = result['models']
    deletion = read(root / 'cleanup.json')
    legacy = read(root / 'legacy-deletion.json')
    smoke = read(root / 'post-cleanup-smoke.json')
    validation = read(root / 'n-validation.json')
    if not (deletion['verified_absent'] and deletion['retained_engines_unchanged']
            and legacy['verified_absent'] and smoke['passed'] and validation['passed']):
        raise ValueError('Conversion, cleanup, and post-cleanup checks must pass')
    result.update(cleanup=deletion, legacy_deletion=legacy, post_cleanup=smoke,
                  nano_validation=validation)
    write_json(root / 'results.json', result)
    charts(root, result)
    freed = (deletion['deleted_bytes'] + legacy['deleted_bytes']) / 1024**2
    escape = html.escape

    def table(headers: list[str], rows: list[list[str]]) -> str:
        return '<div class="scroll"><table><thead><tr>'+''.join(f'<th>{x}</th>' for x in headers)+'</tr></thead><tbody>'+''.join('<tr>'+''.join(f'<td>{x}</td>' for x in row)+'</tr>' for row in rows)+'</tbody></table></div>'

    parts = ['<!doctype html><html lang="ko"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>YOLOE n · s · m 비교</title>',
        '''<style>*{box-sizing:border-box}body{margin:0;background:#f3f5f6;color:#172026;font:15px/1.7 -apple-system,BlinkMacSystemFont,"Noto Sans KR",sans-serif}header{background:#121d23;color:white;border-top:6px solid #76b900}header>div,main{max-width:1280px;margin:auto;padding:36px 32px}header p{color:#bdcbd1}.eyebrow{color:#a4d653;font-size:12px;letter-spacing:.16em;font-weight:700}h1{font-size:38px;letter-spacing:-.04em;margin:14px 0}h2{font-size:23px;letter-spacing:-.03em;margin:0 0 16px}h3{font-size:17px}.cards{display:grid;grid-template-columns:repeat(3,1fr);gap:16px;margin-bottom:28px}.card{padding:24px;border-radius:10px;background:white;border:1px solid #dbe2e6;border-top:4px solid #76b900}.card b{font-size:33px;display:block;font-variant-numeric:tabular-nums}.card span,.muted{color:#617078;font-size:13px}section{background:white;border:1px solid #dbe2e6;border-radius:10px;padding:28px;margin-bottom:28px}.scroll{overflow:auto}table{width:100%;border-collapse:collapse;font-variant-numeric:tabular-nums;font-size:14px}th{background:#edf2f4;text-align:left;white-space:nowrap}td,th{padding:11px 14px;border-bottom:1px solid #dfe5e8;vertical-align:top}td{white-space:nowrap}tbody tr:hover{background:#f8fafb}.note{background:#fff6e1;border-left:3px solid #d2a13a;padding:16px 20px}.info{background:#edf5e4;border-left:4px solid #76b900;padding:18px 22px;margin-bottom:24px}.chart{display:block;width:100%}a{color:#006e8b;text-decoration:none}a:hover{text-decoration:underline}header a{color:#abd9e5}code{font-size:12px;overflow-wrap:anywhere}.triplet{display:grid;grid-template-columns:repeat(3,1fr);gap:12px}.triplet figure{margin:0;background:#f5f7f8;border:1px solid #dbe2e6}.triplet img{display:block;width:100%}.triplet figcaption{padding:12px;font-size:12px;overflow-wrap:anywhere}.case[hidden]{display:none}select,button{font:inherit;padding:9px 12px;border:1px solid #b9c5cc;border-radius:6px;background:white}select{max-width:100%;width:680px}button{cursor:pointer;margin-right:8px}.controls{display:flex;gap:8px;flex-wrap:wrap;margin-bottom:18px}details{margin-top:18px}summary{cursor:pointer;font-weight:600}footer{max-width:1280px;margin:auto;padding:0 32px 36px;font-size:12px;color:#617078}@media(max-width:800px){main,header>div{padding:24px 18px}h1{font-size:30px}section{padding:18px}.cards{grid-template-columns:1fr}.triplet{grid-template-columns:1fr}}@media print{.controls,details{display:none}section{break-inside:avoid}header{print-color-adjust:exact}}</style></head><body>''',
        '<header><div><div class="eyebrow">CLEANY / JETSON INFERENCE BENCHMARK</div><h1>YOLOE n · s · m 비교</h1><p>같은 116장, 같은 32종 프롬프트, 같은 TensorRT FP16 설정</p><p>Jetson Orin NX 16GB · 40W / 동적 클럭 · 입력 640×640 · 배치 1 · 2026-10-28</p><p><a href="results.json">전체 결과 JSON</a> / <a href="comparison.svg">차트 SVG</a> / <a href="cleanup.json">파일 정리 내역</a></p></div></header><main><div class="cards">']
    for m in MODELS:
        item=models[m]; group=item['groups']['all']
        parts.append(f'<div class="card"><span>YOLOE-26{m}-seg · {item["environment"]["engine_size_bytes"]/1024**2:.2f} MiB</span><b>{group["latency_ms"]["mean"]:.2f} ms / 장</b><span>{group["fps"]:.2f} FPS · p95 {group["latency_ms"]["p95"]:.2f}ms · 3,480회</span></div>')
    parts += ['</div>',
        '<div class="info"><strong>사진 한 장을 넣어 CPU 검출 박스와 마스크를 받기까지의 시간을 비교했습니다.</strong> 카메라 스트림, ROS, Gemini와 로봇 동작은 포함하지 않습니다. 운영 기본 모델은 m으로 유지했습니다.</div>',
        '<section><h2>01 &nbsp; 운영 검출 코드 성능</h2>',
        table(['모델','측정 횟수','평균','p50','p95','p99','처리율','m 대비 처리율'], [[m.upper(),f'{models[m]["groups"]["all"]["requests"]:,}',*[f'{models[m]["groups"]["all"]["latency_ms"][k]:.2f} ms' for k in ('mean','p50','p95','p99')],f'{models[m]["groups"]["all"]["fps"]:.2f} FPS',f'{models[m]["groups"]["all"]["fps"]/models["m"]["groups"]["all"]["fps"]:.2f}×'] for m in MODELS]),
        '<p class="muted">FPS = 측정 호출 수 ÷ 연속 실행 블록의 실제 경과 시간. p95는 호출의 95%가 해당 시간 이내에 끝났다는 뜻입니다. 30 FPS 같은 외부 목표를 합격 기준으로 적용하지 않았습니다.</p>',
        '<img class="chart" src="comparison.png" alt="n s m의 지연, 처리율, 처리 단계와 라운드별 비교 그래프">',
        table(['데이터셋','모델','평균','p50 / p95','처리율'], [[LABELS[d],m.upper(),f'{models[m]["groups"][d]["latency_ms"]["mean"]:.2f} ms',f'{models[m]["groups"][d]["latency_ms"]["p50"]:.2f} / {models[m]["groups"][d]["latency_ms"]["p95"]:.2f} ms',f'{models[m]["groups"][d]["fps"]:.2f} FPS'] for d in DATASETS for m in MODELS]),
        '</section><section><h2>02 &nbsp; TensorRT 엔진 단독 성능</h2><p>세 모델 모두 같은 첫 OCID 영상의 실제 전처리 텐서로 NVIDIA trtexec을 실행했습니다. 모델별 3회, 회당 워밍업 5초와 측정 30초입니다.</p>',
        table(['모델','GPU 평균','GPU p50 / p95','Host 평균','엔진 처리율','GPU 클럭 중앙값'], [[m.upper(),f'{models[m]["native"]["gpu_ms"]["mean"]:.2f} ms',f'{models[m]["native"]["gpu_ms"]["p50"]:.2f} / {models[m]["native"]["gpu_ms"]["p95"]:.2f} ms',f'{models[m]["native"]["host_ms"]["mean"]:.2f} ms',f'{models[m]["native"]["fps"]:.2f} FPS',f'{models[m]["native"]["whole_process_gpu_mhz"]["p50"]:.0f} MHz'] for m in MODELS]),
        '<div class="note">엔진 단독 수치는 전처리, 마스크 복원과 CPU 결과 변환을 제외합니다. trtexec은 enqueue를 겹쳐 실행하며 GPU 부하와 동적 클럭도 달라집니다. 따라서 엔진 FPS를 실제 운영 FPS로 해석하거나, 두 시간의 차이 전체를 전후처리 비용으로 해석하면 안 됩니다.</div>',
        '<p class="muted">Host 시간은 H2D + GPU compute + D2H입니다. inference stream 1, 데이터 전송 활성, CUDA Graph와 spin wait 비활성. 클럭 중앙값은 로딩과 워밍업을 포함한 각 trtexec 프로세스 구간입니다.</p></section>',
        '<section><h2>03 &nbsp; 전력, 메모리와 단계별 시간</h2>',
        table(['모델','보드 입력 전력 평균 / 최고','GPU 최고 온도','시스템 RAM 최고','GPU 클럭 중앙값'], [[m.upper(),f'{models[m]["resources"]["power_mw"]["mean"]/1000:.2f} / {models[m]["resources"]["power_mw"]["max"]/1000:.2f} W',f'{models[m]["resources"]["gpu_temperature_c"]["max"]:.1f}°C',f'{models[m]["resources"]["ram_used_mb"]["max"]:,.0f} MB',f'{models[m]["resources"]["gpu_clock_hz"]["p50"]/1e6:.0f} MHz'] for m in MODELS]),
        '<p class="muted">운영 코드의 timed block 안에서 tegrastats를 200ms마다 수집했습니다. RAM은 시스템 전체 사용량이며 모델 전용 VRAM이 아닙니다. 40W는 선택된 전력 모드이고 실제 소비 전력은 VDD_IN 열입니다. 장시간 열 안정성이나 외부 전력계 측정은 아닙니다.</p>',
        table(['평균 구간','N','S','M'], [[label,*[f'{models[m]["groups"]["all"]["stages_ms"][k]["mean"]:.2f} ms' for m in MODELS]] for k,label in [('preprocess','전처리'),('inference','TensorRT 호출'),('postprocess','Ultralytics 후처리')]]+[['어댑터와 CPU 결과 반환 등',*[f'{models[m]["groups"]["all"]["latency_ms"]["mean"]-sum(s["mean"] for s in models[m]["groups"]["all"]["stages_ms"].values()):.2f} ms' for m in MODELS]]]),
        '<p class="muted">세 내부 단계는 Ultralytics의 동기화된 계측값입니다. 마지막 행은 전체 호출 시간에서 내부 단계의 합을 뺀 잔여 시간입니다.</p></section>',
        '<section><h2>04 &nbsp; 검출 결과 비교</h2><div class="note"><strong>검출 개수가 많다고 정확도가 높은 것은 아닙니다.</strong> 현재 32종과 대응하는 완전한 박스·마스크 정답이 없어 모델 간 mAP, precision, recall은 산출하지 않았습니다. 아래 수치는 출력 특성이며, 동일 영상의 결과를 직접 비교할 수 있도록 갤러리를 제공합니다.</div>',
        table(['모델','전체 검출 수','검출 0개 영상','최대 10개 도달 영상','빈 마스크'], [[m.upper(),str(models[m]['output_counts']['detections']),str(models[m]['output_counts']['zero_images'])+'/116',str(models[m]['output_counts']['capped_images'])+'/116',str(models[m]['output_counts']['empty_masks'])] for m in MODELS]),
        '<details><summary>같은 사진의 n / s / m 검출 결과 나란히 보기</summary><p class="muted">첫 비교 라운드의 별도 출력 재생 결과입니다. 같은 물체의 여러 시점이 포함되어 있으며 사람이 정답을 검수한 점수가 아닙니다.</p><div class="controls"><button id="previous">← 이전</button><button id="next">다음 →</button><select id="sample" aria-label="비교할 영상">']
    cases=sorted(models['n']['predictions'],key=lambda p:(p['dataset']!=DATASETS[1],p['id']))
    for i,case in enumerate(cases):
        parts.append(f'<option value="{i}">{i+1:03d} / {LABELS[case["dataset"]]} / {escape(case["id"])}</option>')
    parts.append('</select></div>')
    for i,case in enumerate(cases):
        parts.append(f'<div class="case triplet" data-index="{i}" '+('hidden' if i else '')+'>')
        for m in MODELS:
            item=next(p for p in models[m]['predictions'] if (p['dataset'],p['id'])==(case['dataset'],case['id']))
            names=', '.join(f'{k} × {v}' for k,v in Counter(p['label'] for p in item['predictions']).items()) or '검출 없음'
            src=f'round-1/{m}/adapter/{item["image"]}'
            parts.append(f'<figure><a href="{src}"><img loading="lazy" src="{src}" alt="{m.upper()} {escape(case["id"])}"></a><figcaption><strong>{m.upper()} · {len(item["predictions"])}개</strong><br>{escape(names)}</figcaption></figure>')
        parts.append('</div>')
    parts += ['</details></section><section><h2>05 &nbsp; n 모델 변환 검증과 용량 정리</h2>',
        f'<p>공식 YOLOE-26n-seg 가중치를 배포 SHA-256과 대조한 후 기존 32종 프로파일로 변환했습니다. 원본 PyTorch의 confidence ≥ 0.30 검출에 대해 같은 클래스와 박스 IoU ≥ 0.5로 대응한 일치율은 <strong>{validation["match_rate"]:.2%}</strong>, 대응 마스크 평균 IoU는 <strong>{validation["mean_matched_mask_iou"]:.4f}</strong>였습니다. 기존 활성화 검사를 통과했으며, 이 수치는 변환 일치도이고 정답 대비 정확도는 아닙니다.</p>',
        f'<div class="info"><strong>불필요한 바이너리 {freed:,.1f} MiB 정리</strong><br>이전 real34 s 모델, 원본·준비 PT, ONNX, 텍스트 인코더와 측정용 엔진 사본을 삭제했습니다. n·s·m의 검증된 엔진과 실행에 필요한 metadata는 보존했습니다.</div>',
        f'<p>정리 후 세 엔진을 다시 로드해 저장 RGB <strong>{smoke["total_images"]}회</strong> 재생을 완료했습니다. 기본 m 엔진 선택은 유지했습니다. 재변환하거나 PT와 다시 비교하려면 공식 원본과 인코더를 다시 받아야 합니다.</p>',
        '<p><a href="n-source.json">공식 n 원본과 hash</a> / <a href="n-validation.json">n 변환 검증</a> / <a href="legacy-deletion.json">real34 삭제 내역</a> / <a href="cleanup.json">중간 파일 삭제 내역</a> / <a href="post-cleanup-smoke.json">정리 후 실행 결과</a></p></section>',
        '<section><h2>06 &nbsp; 측정 조건과 원본</h2>',
        table(['구분','조건'],[['장치와 런타임','Orin NX 16GB / JetPack 6.2.1 / CUDA 12.6 / TensorRT 10.3.0 / Ultralytics 8.4.104'],['모델 설정','동일 32종 / FP16 최적화 / FP32 입력 텐서 / batch 1 / static 640×640'],['검출 설정','confidence 0.25 / IoU 설정 0.5 / max_det 10 / retina_masks=True / rect=False'],['데이터','Jetson OCID 100장 + 저장 로봇 영상 16장 / 모두 640×480'],['반복',f'모델별 116장 × 10회 × 3라운드 = 3,480회 / 전체 {result["total_requests"]:,}회'],['실행 순서','1라운드 n→s→m / 2라운드 s→m→n / 3라운드 m→n→s'],['워밍업','프로세스마다 100회 + 데이터셋 블록마다 20회 / 통계에서 제외'],['환경 통제','모델마다 별도 프로세스 / 단일 GPU 작업 순차 실행 / 40W와 동적 클럭 유지'],['입력 준비','디스크에서 RGB를 미리 메모리에 읽음 / 같은 seed로 영상 순서 섞음'],['측정 제외','파일 해독, 모델 로딩, 준비 호출, 그림 저장, ROS, Gemini, 깊이 복원, 로봇 동작']]),
        '<p class="muted">모든 모델은 이번 실행에서 새로 측정했습니다. 이전 m 보고서의 숫자를 재사용하지 않았습니다. 같은 원본 영상을 반복한 것이므로 10,440개 독립적인 정확도 표본이 아닙니다. GPU 클럭을 고정한 최대 성능 비교는 아닙니다.</p>',
        table(['모델','엔진 SHA-256','첫 라운드 원본'],[[m.upper(),f'<code>{models[m]["environment"]["engine_sha256"]}</code>',f'<a href="round-1/{m}/adapter/timings.csv">CSV</a> / <a href="round-1/{m}/adapter/inputs.json">입력 hash</a> / <a href="round-1/{m}/adapter/environment.json">환경</a>'] for m in MODELS]),
        '<p><a href="suite.json">전체 실행 명령·상태</a> / <a href="host-environment.json">호스트 환경·소스 hash</a> / <a href="results.json">전체 집계</a> / <a href="export-tests.log">변환 도구 검사</a> / <a href="suite-tests.log">실행 순서와 삭제 경계 검사</a></p>',
        '<p><a href="https://docs.ultralytics.com/models/yoloe/">Ultralytics YOLOE 공식 문서</a>의 고정 프롬프트 export 절차와 <a href="https://docs.nvidia.com/deeplearning/tensorrt/latest/performance/benchmarking.html">NVIDIA TensorRT 벤치마크 문서</a>의 지표 구분을 참고했습니다. CLI 옵션은 설치된 TensorRT 10.3 기준입니다.</p></section></main><footer>Cleany 내부 실측 보고서. NVIDIA 공식 성능 결과나 MLPerf 인증 점수가 아닙니다. 전체 시스템 속도와 모델 정확도는 이 측정만으로 확정하지 않습니다.</footer>',
        '<script>const select=document.querySelector("#sample");function show(){document.querySelectorAll(".case").forEach(x=>x.hidden=x.dataset.index!==select.value)}select.addEventListener("change",show);for(const [id,direction] of [["previous",-1],["next",1]])document.getElementById(id).addEventListener("click",()=>{select.selectedIndex=(select.selectedIndex+direction+select.options.length)%select.options.length;show()});</script></body></html>']
    (root/'report.html').write_text('\n'.join(parts))


if __name__ == '__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root',type=Path,required=True)
    render(parser.parse_args().root)
