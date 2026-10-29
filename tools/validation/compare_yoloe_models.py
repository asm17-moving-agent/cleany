#!/usr/bin/env python3
"""Compare saved images with two prompted TensorRT models, without robot/API calls."""
from __future__ import annotations

import argparse
from collections import Counter
import hashlib
import html
import json
from pathlib import Path
import platform
import statistics
import time


def bbox_iou(a: list[float], b: list[float]) -> float:
    intersection = max(0., min(a[2], b[2]) - max(a[0], b[0])) * max(
        0., min(a[3], b[3]) - max(a[1], b[1]))
    union = (a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - intersection
    return intersection / union if union > 0 else 0.


def match_targets(targets: list[dict], predictions: list[dict], *, classified: bool) -> list[dict]:
    """Match each reviewed target and prediction at most once at box IoU >= 0.5."""
    edges = []
    for target in targets:
        matches = [(bbox_iou(target['bbox'], row['bbox']), index)
                   for index, row in enumerate(predictions)
                   if not classified or row['label'] in target['labels']]
        edges.append([index for overlap, index in sorted(matches, reverse=True) if overlap >= .5])
    assigned: dict[int, int] = {}

    def assign(left: int, seen: set[int]) -> bool:
        for right in edges[left]:
            if right in seen:
                continue
            seen.add(right)
            if right not in assigned or assign(assigned[right], seen):
                assigned[right] = left
                return True
        return False

    for left in range(len(targets)):
        assign(left, set())
    return [dict(target=left, prediction=right,
                 iou=bbox_iou(targets[left]['bbox'], predictions[right]['bbox']))
            for right, left in sorted(assigned.items(), key=lambda pair: pair[1])]


def unique_boxes(rows: list[dict]) -> int:
    kept: list[dict] = []
    for row in sorted(rows, key=lambda row: row['confidence'], reverse=True):
        if all(bbox_iou(row['bbox'], previous['bbox']) < .98 for previous in kept):
            kept.append(row)
    return len(kept)


def latency(values: list[float]) -> dict:
    values = sorted(values)
    at = (len(values) - 1) * .95
    lower = int(at)
    return dict(count=len(values), p50_ms=statistics.median(values),
                p95_ms=values[lower] + (values[min(lower + 1, len(values) - 1)] - values[lower]) * (at - lower))


def predict(model, rgb, maximum: int):
    import numpy as np
    import torch
    torch.cuda.synchronize()
    began = time.perf_counter()
    result = model.predict(np.ascontiguousarray(rgb[..., ::-1]), device='cuda:0',
        imgsz=640, rect=False, conf=.25, iou=.5, max_det=maximum,
        agnostic_nms=False, retina_masks=True, verbose=False)[0]
    rows = []
    masks = result.masks.data.cpu().numpy() if result.masks is not None else []
    for index, box in enumerate(result.boxes):
        rows.append(dict(label=result.names[int(box.cls.item())],
            confidence=float(box.conf.item()), bbox=box.xyxy[0].cpu().numpy().astype(float).tolist(),
            mask_pixels=int(np.count_nonzero(masks[index]))))
    torch.cuda.synchronize()
    elapsed = (time.perf_counter() - began) * 1000
    return rows, elapsed, dict(result.speed)


def panel(rgb, rows: list[dict], targets: list[dict], title: str, path: Path) -> None:
    from PIL import Image, ImageDraw
    height, width = rgb.shape[:2]
    scale = min(1., 1200 / width)
    image = Image.fromarray(rgb).resize((round(width * scale), round(height * scale)))
    pen = ImageDraw.Draw(image)
    for target in targets:
        pen.rectangle([value * scale for value in target['bbox']], outline='#43db91', width=2)
    for index, row in enumerate(rows, 1):
        box = [value * scale for value in row['bbox']]
        pen.rectangle(box, outline='#ffcc33', width=2)
        text = f"{index} {row['label']} {row['confidence']:.2f}"
        pen.text((box[0], max(0, box[1] - 12)), text, fill='#ffcc33', stroke_width=1, stroke_fill='black')
    canvas = Image.new('RGB', (image.width, image.height + 24), '#15222d')
    canvas.paste(image, (0, 24))
    ImageDraw.Draw(canvas).text((6, 6), title, fill='white')
    canvas.save(path)


def write_report(output: Path, result: dict) -> None:
    parts = ['<!doctype html><html lang="ko"><meta charset="utf-8">',
        '<meta name="viewport" content="width=device-width, initial-scale=1"><title>YOLOE s / m 비교</title>',
        '<style>body{font:16px system-ui;margin:24px;background:#f4f6f8;color:#15222d}table{border-collapse:collapse;background:white}td,th{padding:8px;border:1px solid #ccd4dc;text-align:left}.pair{display:grid;grid-template-columns:1fr 1fr;gap:12px}.pair img{width:100%}section{margin:28px 0;padding:16px;background:white}code{overflow-wrap:anywhere}@media(max-width:750px){.pair{grid-template-columns:1fr}}</style>',
        '<h1>YOLOE-26s / 26m · 32종 · TensorRT FP16</h1>',
        '<p>동일한 640 입력, confidence 0.25, IoU 설정 0.5. 기본 최대 10개와 진단용 40개를 구분합니다. 녹색은 수동 검토 박스, 노란색은 원본 검출입니다.</p>',
        '<p>수동 검토 대상의 회수율은 제한된 표본의 박스 IoU 0.5 결과입니다. 전체 정답 세트의 mAP나 마스크 정확도가 아닙니다. 로컬 4장은 같은 물체의 반복 촬영이며 독립 표본이 아닙니다.</p>',
        '<p>검토 표본은 주요 물체만 포함하며 컵 뚜껑과 빨대는 별도 정답으로 세지 않습니다. 담뱃갑은 cigarette 또는 box를 허용하고 원본 예측 이름도 함께 표시합니다. Jetson 영상에도 같은 상자의 반복 촬영이 포함됩니다.</p>',
        '<h2>요약</h2><table><tr><th>입력</th><th>최대 후보</th><th>모델</th><th>p50 / p95</th><th>검토 대상 위치 일치</th><th>종류까지 일치</th></tr>']
    for row in result['summary']:
        parts.append(f"<tr><td>{html.escape(row['group'])}</td><td>{row['maximum']}</td><td>{row['model']}</td><td>{row['p50_ms']:.1f} / {row['p95_ms']:.1f} ms</td><td>{row['localized']}/{row['targets']}</td><td>{row['classified']}/{row['targets']}</td></tr>")
    parts.append('</table>')
    for case in result['cases']:
        parts.append(f"<section><h2>{html.escape(case['id'])} · {case['mode']} · {case['shape'][1]}×{case['shape'][0]}</h2>")
        for maximum in (10, 40):
            if str(maximum) not in case['predictions']:
                continue
            parts.append(f'<h3>최대 후보 / max_det={maximum}</h3><div class="pair">')
            for model, row in case['predictions'][str(maximum)].items():
                parts.append(f"<div><b>{model}: {row['latency']['p50_ms']:.1f} ms · raw {len(row['rows'])} / unique {row['unique_boxes']}</b><img loading='lazy' src='{html.escape(row['image'])}'><p>{html.escape(', '.join(f'{key} × {value}' for key,value in Counter(r['label'] for r in row['rows']).items()))}</p></div>")
            parts.append('</div>')
            targets = [target for target in case['targets'] if target.get('scope', 'main') == 'main']
            if targets:
                parts.append('<table><tr><th>검토 대상</th><th>s</th><th>m</th></tr>')
                for index, target in enumerate(targets):
                    parts.append('<tr><td>' + html.escape('/'.join(target['labels'])) + '</td>')
                    for name in ('s', 'm'):
                        entry = case['predictions'][str(maximum)][name]
                        found = next((pair for pair in entry['classified'] if pair['target'] == index), None)
                        correct = found is not None
                        if found is None:
                            found = next((pair for pair in entry['localized'] if pair['target'] == index), None)
                        if found is None:
                            text = '위치 미일치 / 미검출'
                        else:
                            prediction = entry['rows'][found['prediction']]
                            text = f"{prediction['label']} {prediction['confidence']:.2f}, IoU {found['iou']:.2f}"
                            if not correct:
                                text += ' (종류 불일치)'
                        parts.append('<td>' + html.escape(text) + '</td>')
                    parts.append('</tr>')
                parts.append('</table>')
        parts.append('</section>')
    parts.append('<p><a href="results.json">원본 결과 JSON</a> · <a href="../annotations.json">수동 검토 기준</a></p></html>')
    (output / 'report.html').write_text('\n'.join(parts), encoding='utf-8')


def main() -> None:
    import cv2
    import numpy as np
    from PIL import Image, ImageOps
    import torch
    import ultralytics
    from ultralytics import YOLO
    import yaml
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input-root', type=Path, required=True)
    parser.add_argument('--annotations', type=Path, required=True)
    parser.add_argument('--s-engine', type=Path, required=True)
    parser.add_argument('--m-engine', type=Path, required=True)
    parser.add_argument('--profile', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--repeats', type=int, default=3)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    profile = yaml.safe_load(args.profile.read_text())['perception_inspector']['ros__parameters']
    classes = profile['yoloe_classes']
    reviewed = json.loads(args.annotations.read_text())
    models = {key: YOLO(str(path), task='segment') for key, path in [('s', args.s_engine), ('m', args.m_engine)]}
    cases = []
    for path in sorted(args.input_root.rglob('*')):
        if not path.is_file() or path.suffix.lower() not in ('.jpg', '.jpeg', '.png') or path.name.startswith('preview-'):
            continue
        with Image.open(path) as image:
            native = np.asarray(ImageOps.exif_transpose(image).convert('RGB')).copy()
        height, width = native.shape[:2]
        ratio = min(1., 640 / max(height, width))
        camera = cv2.resize(native, (round(width * ratio), round(height * ratio)), interpolation=cv2.INTER_LINEAR)
        modes = [('camera640', camera)]
        if path.parent.name == 'local':
            modes.append(('native', native))
        for mode, rgb in modes:
            targets = []
            annotation = reviewed.get(path.stem, {})
            for item in annotation.get('targets', []):
                sx = rgb.shape[1] / annotation['reference_width']
                sy = rgb.shape[0] / annotation['reference_height']
                target = dict(item)
                target['bbox'] = [item['bbox'][0] * sx, item['bbox'][1] * sy,
                                  item['bbox'][2] * sx, item['bbox'][3] * sy]
                targets.append(target)
            cases.append(dict(id=path.stem, mode=mode, rgb=rgb, targets=targets,
                path=str(path), sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
                group=path.parent.name, predictions={}))
    for name, model in models.items():
        for _ in range(5):
            predict(model, cases[0]['rgb'], 10)
        if [model.names[index] for index in range(len(model.names))] != classes:
            raise ValueError(f'{name}: engine classes differ from the requested profile')
    for case_index, case in enumerate(cases):
        for maximum in ((10, 40) if case['mode'] == 'camera640' else (10,)):
            entries = {name: dict(rows=[], times=[], speeds=[]) for name in models}
            for repeat in range(args.repeats):
                order = list(models) if (repeat + case_index) % 2 == 0 else list(reversed(models))
                for name in order:
                    rows, elapsed, speed = predict(models[name], case['rgb'], maximum)
                    entries[name]['rows'] = rows
                    entries[name]['times'].append(elapsed)
                    entries[name]['speeds'].append(speed)
            for name, entry in entries.items():
                main_targets = [target for target in case['targets'] if target.get('scope', 'main') == 'main']
                entry.update(latency=latency(entry.pop('times')), unique_boxes=unique_boxes(entry['rows']),
                    localized=match_targets(main_targets, entry['rows'], classified=False),
                    classified=match_targets(main_targets, entry['rows'], classified=True),
                    target_count=len(main_targets))
                entry['image'] = f"{case['id']}-{case['mode']}-{name}-{maximum}.jpg"
                panel(case['rgb'], entry['rows'], case['targets'],
                    f"{name} / max_det={maximum} / {entry['latency']['p50_ms']:.1f} ms", args.output / entry['image'])
            case['predictions'][str(maximum)] = entries
        print(json.dumps(dict(image=case['id'], mode=case['mode'], counts={cap: {name: len(entry['rows'])
            for name, entry in items.items()} for cap, items in case['predictions'].items()})), flush=True)
        case['shape'] = list(case.pop('rgb').shape)
    summary = []
    for group in ('local/camera640', 'local/native', 'jetson/camera640'):
        selected = [case for case in cases if f"{case['group']}/{case['mode']}" == group]
        for maximum in (10, 40):
            for model in models:
                entries = [case['predictions'][str(maximum)][model] for case in selected
                           if str(maximum) in case['predictions']]
                if not entries:
                    continue
                summary.append(dict(group=group, maximum=maximum, model=model,
                    **latency([entry['latency']['p50_ms'] for entry in entries]),
                    localized=sum(len(entry['localized']) for entry in entries),
                    classified=sum(len(entry['classified']) for entry in entries),
                    targets=sum(entry['target_count'] for entry in entries)))
    result = dict(classes=classes, profile_sha256=hashlib.sha256(args.profile.read_bytes()).hexdigest(),
        reviewed_class_coverage=sorted({label for key, item in reviewed.items() if key != '_notes'
                                       for target in item['targets'] for label in target['labels']}),
        environment=dict(python=platform.python_version(), torch=torch.__version__, ultralytics=ultralytics.__version__,
                         gpu=torch.cuda.get_device_name(0)),
        model_sha256={name: hashlib.sha256(path.read_bytes()).hexdigest() for name,path in [('s',args.s_engine),('m',args.m_engine)]},
        conditions=dict(imgsz=640, confidence=.25, iou=.5, agnostic_nms=False, retina_masks=True,
                        repeats=args.repeats, warmup=5, timing='synchronized predict and CPU mask materialization; excludes image loading and rendering'),
        summary=summary, cases=cases,
        limitations=['Manual illustrative boxes, not an independent complete detection benchmark.',
                     'No mask ground truth or measured segmentation accuracy.',
                     'Cigarette samples are packs; no exposed cigarette sticks are annotated.',
                     'Native local photos and resized camera-like images have different postprocessing costs.',
                     'Summary latency percentiles summarize per-image medians, not video throughput.'])
    (args.output / 'results.json').write_text(json.dumps(result,indent=2)+'\n')
    write_report(args.output, result)


if __name__ == '__main__':
    main()
