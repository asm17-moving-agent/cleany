#!/usr/bin/env python3
"""Render a map-only README GIF from recorded occupancy-grid snapshots.

Requires NumPy and Pillow. The first phase uses recorded maps; the extension
uses previously captured Cartographer sensor-replay maps. No SLAM is run here.
"""

import argparse
from contextlib import ExitStack
import hashlib
import json
from pathlib import Path

import numpy as np
from PIL import Image


ROOT = Path(__file__).resolve().parents[2]
SOURCES = (
    "runs/mapping/2026-11-05_221524_slam-animation/analysis",
    "runs/mapping/2026-11-06_map-growth/analysis/full4",
)
UNKNOWN = (233, 239, 235)


def digest(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--data-root", type=Path, required=True)
    parser.add_argument("--output", type=Path, default=ROOT / "docs/assets/readme/map-growth.gif")
    args = parser.parse_args()
    size, padding, stride, frame_ms, hold_ms = 512, 16, 2, 100, 2000
    bounds, selected, sources = [], [], []
    with ExitStack() as stack:
        for phase, relative in enumerate(SOURCES):
            source = args.data_root / relative
            timeline = json.loads((source / "timeline.json").read_text())
            arrays = stack.enter_context(np.load(source / "snapshots.npz", allow_pickle=False))
            sources.append({"path": relative, "timeline_sha256": digest(source / "timeline.json"),
                            "snapshots_sha256": digest(source / "snapshots.npz")})
            indices = sorted(set(range(0, len(timeline["frames"]), stride)) | {len(timeline["frames"]) - 1})
            for index in indices:
                frame = timeline["frames"][index]
                key = frame["map"]
                if key is None:
                    continue
                meta = timeline["metadata"][key]
                grid = arrays[key]
                if grid.ndim != 2 or np.any((grid < -1) | (grid > 100)):
                    raise ValueError(f"Invalid occupancy grid: {relative}/{key}")
                if meta.get("received_ns", 0) > frame["time"] * 1e9:
                    raise ValueError(f"Map arrives after its display time: {key}")
                iy, ix = np.where(grid >= 0)
                if not len(ix):
                    continue
                x, y = meta["origin"]
                resolution = meta["resolution"]
                bounds.append((x + ix.min() * resolution, x + (ix.max() + 1) * resolution,
                               y + iy.min() * resolution, y + (iy.max() + 1) * resolution))
                selected.append((phase, index, frame, meta, grid))
        x0, x1 = min(b[0] for b in bounds), max(b[1] for b in bounds)
        y0, y1 = min(b[2] for b in bounds), max(b[3] for b in bounds)
        scale = min((size - 2 * padding) / (x1 - x0), (size - 2 * padding) / (y1 - y0))
        left, top = (x0 + x1) / 2 - size / scale / 2, (y0 + y1) / 2 + size / scale / 2
        palette = [channel for value in range(101) for channel in [round(250 - 2.25 * value)] * 3]
        palette.extend(UNKNOWN)
        palette.extend([0] * (768 - len(palette)))
        images, trace = [], []
        for phase, index, frame, meta, grid in selected:
            pixels = np.flipud(np.where(grid < 0, 101, grid).astype(np.uint8))
            image = Image.fromarray(pixels).convert("P")
            image.putpalette(palette)
            resolution = meta["resolution"]
            ox, oy = meta["origin"]
            images.append(image.transform(
                (size, size), Image.Transform.AFFINE,
                (1 / (scale * resolution), 0, (left - ox) / resolution,
                 0, 1 / (scale * resolution), (oy + grid.shape[0] * resolution - top) / resolution),
                resample=Image.Resampling.NEAREST, fillcolor=101,
            ))
            trace.append({"phase": phase, "source_frame": index, "time": frame["time"],
                          "map": frame["map"], "known_cells": int(np.sum(grid >= 0))})
    args.output.parent.mkdir(parents=True, exist_ok=True)
    durations = [frame_ms] * len(images)
    durations[-1] += hold_ms
    images[0].save(args.output, save_all=True, append_images=images[1:],
                   duration=durations, loop=0, optimize=False, disposal=1)
    with Image.open(args.output) as gif:
        frames, duration = gif.n_frames, 0
        for index in range(frames):
            gif.seek(index)
            gif.load()
            duration += gif.info["duration"]
    provenance = {
        "scope": "Recorded initial maps followed by existing Cartographer sensor-replay extension snapshots.",
        "layers": ["occupancy_grid"], "size": size, "sources": sources,
        "method": "Fixed world bounds, nearest-neighbor grid sampling; no trajectories, text, scan points or UI.",
        "camera_bounds": [left, left + size / scale, top - size / scale, top],
        "source_frame_stride": stride, "nominal_frame_duration_ms": frame_ms,
        "final_hold_ms": hold_ms, "decoded_frames": frames, "duration_ms": duration,
        "sha256": digest(args.output), "frames": trace,
    }
    args.output.with_suffix(".json").write_text(json.dumps(provenance, indent=2) + "\n")
    print(f"Rendered {frames} GIF frames, {duration / 1000:.1f}s, {args.output.stat().st_size / 1024:.0f} KiB")


if __name__ == "__main__":
    main()
