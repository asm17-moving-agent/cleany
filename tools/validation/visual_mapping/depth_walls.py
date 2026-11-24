"""Compare vertical RGB-D surfaces on repeat passes, independently of feature matches."""

import argparse
import json
from collections import defaultdict
from itertools import combinations
from pathlib import Path

import cv2
import numpy as np
from PIL import Image, ImageDraw
from fuse_map import wall_eval
from geometry import camera, load_track, sample, transform


def inspect(root):
    folder = root / "results/fusion"
    field = load_track(root / "input/lidar/field.csv")
    lidar = load_track(root / "input/lidar/training.csv")
    fused = np.loadtxt(folder / "trajectory.csv", delimiter=",", skiprows=1)
    maps = {"field": field, "lidar": lidar, "fused": fused}
    clock = np.linspace(max(field[0, 0], lidar[0, 0]), min(field[-1, 0], lidar[-1, 0]), 600)
    rot, offset = wall_eval.rigid(sample(lidar, clock)[:, :2], sample(field, clock)[:, :2])
    k, mount = camera(root)
    geom = wall_eval.Map.read(root / "input/lidar")
    walls = {name: (geom.from_pixel(*a), geom.from_pixel(*b)) for name, a, b in wall_eval.WALLS}
    xx, yy = np.meshgrid(np.arange(2, 318, 4), np.arange(2, 238, 4))
    rays = np.stack(((2 * xx - k[0, 2]) / k[0, 0], (2 * yy - k[1, 2]) / k[1, 1], np.ones_like(xx)), axis=2)
    observations = {wall: defaultdict(list) for wall in walls}
    last = 0
    for path in sorted((root / "input/depth").glob("*.png")):
        ns = int(path.stem)
        t = ns * 1e-9
        if t - last < 0.85 or t < field[0, 0] or t > field[-1, 0]:
            continue
        last = t
        depth = cv2.imread(str(path), cv2.IMREAD_UNCHANGED)[yy, xx] * 0.001
        pts = rays * depth[:, :, None]
        dx = pts[1:-1, 2:] - pts[1:-1, :-2]
        dy = pts[2:, 1:-1] - pts[:-2, 1:-1]
        normals = np.cross(dx, dy)
        length = np.linalg.norm(normals, axis=2)
        normals = (normals / np.maximum(length[:, :, None], 1e-8)) @ mount[:3, :3].T
        keep = (depth[1:-1, 1:-1] > 0.4) & (depth[1:-1, 1:-1] < 5.0) & (length > 1e-5)
        keep &= (
            (np.linalg.norm(dx, axis=2) < 0.45)
            & (np.linalg.norm(dy, axis=2) < 0.45)
            & (abs(normals[:, :, 2]) < 0.28)
        )
        center = pts[1:-1, 1:-1] @ mount[:3, :3].T + mount[:3, 3]
        keep &= (center[:, :, 2] > 0.25) & (center[:, :, 2] < 2.5)
        body = center[keep, :2]
        if not len(body):
            continue
        pixels = np.stack((2 * xx[1:-1, 1:-1][keep], 2 * yy[1:-1, 1:-1][keep]), 1)
        poses = {key: sample(track, t) for key, track in maps.items()}
        hits = {key: transform(q, body) for key, q in poses.items()}
        for key in ("lidar", "fused"):
            hits[key] = hits[key] @ rot + offset
        q = poses["field"]
        normal = transform(np.r_[0, 0, q[2]], normals[keep, :2])
        for wall, (a, b) in walls.items():
            length = np.linalg.norm(b - a)
            tangent = (b - a) / length
            n = np.array([-tangent[1], tangent[0]])
            along = (hits["field"] - a) @ tangent
            dist = (hits["field"] - a) @ n
            ok = (along >= 0) & (along < length) & (abs(dist) < 0.45) & (abs(normal @ n) > 0.84)
            if ok.sum() < 16:
                continue
            side = int(np.sign((q[:2] - a) @ n))
            bins = np.floor(along / 0.75).astype(int)
            for cell in np.unique(bins[ok]):
                ids = np.flatnonzero(ok & (bins == cell))
                if len(ids) < 14:
                    continue
                values = [float(np.median((hits[key][ids] - a) @ n)) for key in maps]
                observations[wall][(int(cell), side)].append(
                    (t, ns, values, pixels[ids][:: max(1, len(ids) // 55)])
                )
    travel = np.r_[0, np.cumsum(np.linalg.norm(np.diff(field[:, 1:3], axis=0), axis=1))]
    results = {}
    examples = []
    for wall, bins in observations.items():
        grouped = defaultdict(list)
        for key, rows in bins.items():
            visits = defaultdict(list)
            for row in rows:
                visits[int((row[0] - field[0, 0]) / 4)].append(row)
            for u, v in combinations(visits.values(), 2):
                a = u[len(u) // 2]
                b = v[len(v) // 2]
                if b[0] - a[0] < 45 or np.diff(np.interp([a[0], b[0]], field[:, 0], travel))[0] < 5:
                    continue
                m1 = np.median([row[2] for row in u], axis=0)
                m2 = np.median([row[2] for row in v], axis=0)
                gaps = 100 * np.abs(m1 - m2)
                grouped[key].append((gaps, a, b))
        if not grouped:
            results[wall] = dict(pair_count=0, bins=0)
            continue
        med = np.median([np.median([v[0] for v in pairs], axis=0) for pairs in grouped.values()], axis=0)
        samples = [v for pairs in grouped.values() for v in pairs]
        g, a, b = min(samples, key=lambda r: abs(r[0][0] - med[0]))
        results[wall] = dict(
            pair_count=len(samples),
            bins=len(grouped),
            normal_separation_cm=dict(zip(maps, med.tolist())),
            example_time_ns=[a[1], b[1]],
            example_gaps_cm=g.tolist(),
        )
        if wall in ("A", "B"):
            examples.append((wall, a, b))
    if examples:
        canvas = Image.new("RGB", (960, len(examples) * 382), "white")
        draw = ImageDraw.Draw(canvas)
        for row, (key, a, b) in enumerate(examples):
            for col, obs in enumerate((a, b)):
                _, ns, _, uv = obs
                im = Image.open(root / "input/rgb" / f"{ns}.jpg").convert("RGB")
                dot = ImageDraw.Draw(im)
                for x, y in uv:
                    dot.ellipse((x - 3, y - 3, x + 3, y + 3), outline="#00ffb0", width=2)
                im.thumbnail((480, 360))
                canvas.paste(im, (col * 480, row * 382))
                draw.text(
                    (col * 480 + 6, row * 382 + 360),
                    f"Wall {key} | pass {col+1} | {int(obs[0]-field[0,0])} s",
                    fill="#183c39",
                )
        canvas.save(folder / "depth-wall-passes.jpg", quality=92)
    (folder / "depth-walls.json").write_text(json.dumps(results, indent=2))
    print({k: (v["pair_count"], v.get("normal_separation_cm")) for k, v in results.items()})


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("root", type=Path)
    inspect(parser.parse_args().root)
