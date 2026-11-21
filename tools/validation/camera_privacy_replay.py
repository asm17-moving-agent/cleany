#!/usr/bin/env python3
"""Run the ROS camera redactor on a real clip; audit independently labeled regions."""
from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import time

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'ros2_ws/src/cleany_perception'))
from cleany_perception.privacy import CameraRedactor


def evaluate(mask, regions, t):
    rows = []
    h, w = mask.shape
    for r in regions:
        if not r['start'] <= t < r['end']:
            continue
        if 'keyframes' in r:
            times, polygons = zip(*r['keyframes'])
            polygons = np.asarray(polygons)
            right = min(max(1, int(np.searchsorted(times, t))), len(times)-1)
            left = right - 1
            weight = np.clip((t - times[left]) / (times[right] - times[left]), 0., 1.)
            xy = polygons[left] * (1. - weight) + polygons[right] * weight
        else:
            xy = np.asarray(r['polygon'])
        xy = xy * (w - 1, h - 1)
        area = np.zeros_like(mask)
        cv2.fillPoly(area, [np.rint(xy).astype(np.int32)], 1)
        region = mask[area != 0]
        rows.append({'label': r['label'], 'pixels': len(region),
                     'covered': int(np.count_nonzero(region))})
    return rows


def resized(bgr, max_side):
    if max_side and max(bgr.shape[:2]) > max_side:
        ratio = max_side / max(bgr.shape[:2])
        return cv2.resize(bgr, None, fx=ratio, fy=ratio, interpolation=cv2.INTER_AREA)
    return bgr


def replay(args):
    import torch
    cv2.setNumThreads(2)
    if args.device == 'cpu':
        torch.backends.mkldnn.enabled = False
        torch.set_num_threads(3)
    redactor = CameraRedactor(args.weights, args.face, args.text, args.profile, device=args.device)
    regions = json.loads(args.regions.read_text()) if args.regions else []
    still = args.input.suffix.lower() in {'.png', '.jpg', '.jpeg'}
    capture = cv2.VideoCapture(str(args.input)) if not still else None
    if capture is not None and not capture.isOpened():
        raise ValueError(f'Cannot decode {args.input}')
    source_fps = 1. if still else capture.get(cv2.CAP_PROP_FPS)
    if source_fps <= 0 or (not still and args.fps > source_fps + .01):
        raise ValueError('Replay rate must fit the source rate')
    start = round(args.start * source_fps)
    stop = start + max(1, round(args.duration * source_fps))
    if capture is not None:
        if start > 0:
            primer = max(0, start - max(1, round(source_fps / args.fps)))
            capture.set(cv2.CAP_PROP_POS_FRAMES, primer)
            ok, bgr = capture.read()
            if not ok:
                raise ValueError('Cannot read pre-roll for the requested cut')
            redactor.redact(resized(bgr, args.max_side), primer / source_fps)
        capture.set(cv2.CAP_PROP_POS_FRAMES, start)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    rows = []
    writer = None
    with tempfile.TemporaryDirectory(dir=args.output.parent) as temp:
        target = Path(temp) / args.output.name
        try:
            index = start
            while index < stop:
                if still:
                    bgr = cv2.imread(str(args.input))
                else:
                    if not capture.grab():
                        break
                    wanted = start + round(len(rows) * source_fps / args.fps)
                    if index < wanted:
                        index += 1
                        continue
                    ok, bgr = capture.retrieve()
                    if not ok:
                        raise ValueError(f'Cannot decode frame {index}')
                if bgr is None:
                    raise ValueError('Invalid input image')
                bgr = resized(bgr, args.max_side)
                before = time.perf_counter()
                redacted = redactor.redact(bgr, None if still else index / source_fps)
                elapsed = 1000 * (time.perf_counter() - before)
                if redacted is None:
                    rows.append({'source_frame': index, 'source_s': index / source_fps,
                                 'latency_ms': round(elapsed, 2), 'withheld': True, 'regions': []})
                    index += 1
                    continue
                frame = redacted.image
                rows.append({'source_frame': index, 'source_s': index / source_fps,
                             'latency_ms': round(elapsed, 2), 'counts': redacted.counts,
                             'mask_fraction': round(float(np.mean(redacted.mask != 0)), 5),
                             'regions': evaluate(redacted.mask, regions, index / source_fps)})
                if still:
                    if not cv2.imwrite(str(target), frame):
                        raise RuntimeError('Image export failed')
                    break
                if writer is None:
                    writer = subprocess.Popen([args.ffmpeg, '-nostdin', '-loglevel', 'error',
                        '-f', 'rawvideo', '-pixel_format', 'bgr24', '-video_size',
                        f'{frame.shape[1]}x{frame.shape[0]}', '-framerate', str(args.fps),
                        '-i', '-', '-an', '-vf', 'pad=ceil(iw/2)*2:ceil(ih/2)*2',
                        '-c:v', 'libx264', '-preset', 'fast', '-crf', '17', '-pix_fmt', 'yuv420p',
                        '-movflags', '+faststart', str(target)], stdin=subprocess.PIPE)
                writer.stdin.write(frame.tobytes())
                index += 1
        finally:
            if capture is not None:
                capture.release()
            if writer is not None:
                writer.stdin.close()
                if writer.wait() != 0:
                    raise RuntimeError('Video export failed')
        if not target.is_file():
            raise ValueError('No publishable frames in the requested interval')
        if not still and len(rows) < round(args.duration * args.fps):
            raise ValueError('Source ended before requested interval')
        os.replace(target, args.output)
    samples = [r['latency_ms'] for r in rows[1:]]
    digest = hashlib.sha256()
    with args.input.open('rb') as source:
        for chunk in iter(lambda: source.read(1024 * 1024), b''):
            digest.update(chunk)
    report = dict(input=str(args.input.resolve()), input_sha256=digest.hexdigest(),
        output=str(args.output), source_fps=source_fps, fps=None if still else args.fps,
        size=[frame.shape[1], frame.shape[0]], first_ms=rows[0]['latency_ms'],
        warm_p50_ms=float(np.median(samples)) if samples else None,
        warm_p95_ms=float(np.percentile(samples, 95)) if samples else None,
        withheld_frames=sum(bool(r.get('withheld')) for r in rows),
        emitted_frames=sum(not r.get('withheld') for r in rows), frames=rows)
    args.output.with_suffix('.json').write_text(json.dumps(report, indent=2) + '\n')


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    for name in ('input', 'output', 'weights', 'face', 'text'):
        p.add_argument('--' + name, type=Path, required=True)
    p.add_argument('--profile', type=Path, default=ROOT / 'ros2_ws/src/cleany_perception/config/yoloe_privacy.yaml')
    p.add_argument('--device', default='cpu')
    p.add_argument('--start', type=float, default=0.)
    p.add_argument('--duration', type=float, default=5.)
    p.add_argument('--fps', type=float, default=12.)
    p.add_argument('--max-side', type=int, default=960)
    p.add_argument('--ffmpeg', default='ffmpeg')
    p.add_argument('--regions', type=Path)
    args = p.parse_args()
    if args.start < 0 or args.duration <= 0 or args.fps <= 0 or args.max_side < 0:
        p.error('Invalid time, rate or size')
    replay(args)
