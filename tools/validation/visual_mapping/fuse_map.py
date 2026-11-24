"""Fuse verified RGB-D revisit edges with LiDAR poses and evaluate on withheld scans."""

import argparse
import csv
import json
import sys
from pathlib import Path

import cv2
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import yaml
from PIL import Image
from scipy.optimize import least_squares
from scipy.sparse import lil_matrix

from geometry import load_track, relative, sample, transform, wrap

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "offline_mapping"))
import compare as wall_eval


def optimize(track, observations):
    # Limit repeated constraints from a stationary revisit to one vote per 8 s window pair.
    chosen = {}
    for edge in observations:
        key = (edge["a"] // 8_000_000_000, edge["b"] // 8_000_000_000)
        score = edge["inliers"] * edge["scan_overlap"] / (1 + 8 * edge["depth_residual_m"])
        if key not in chosen or score > chosen[key][0]:
            chosen[key] = score, edge
    edges = [v[1] for v in chosen.values()]
    if not edges:
        raise ValueError("No independently scan-verified visual constraints")
    times = np.r_[np.arange(track[0, 0], track[-1, 0], 3.0), track[-1, 0]]
    baseline = sample(track, times)
    delta = relative(baseline[:-1], baseline[1:])
    a = np.array([e["a"] for e in edges]) * 1e-9
    b = np.array([e["b"] for e in edges]) * 1e-9
    ai = np.clip(np.searchsorted(times, a, side="right") - 1, 0, len(times) - 2)
    bi = np.clip(np.searchsorted(times, b, side="right") - 1, 0, len(times) - 2)
    af = ((a - times[ai]) / (times[ai + 1] - times[ai]))[:, None]
    bf = ((b - times[bi]) / (times[bi + 1] - times[bi]))[:, None]
    ap, bp = sample(track, a), sample(track, b)
    obs = np.array([e["rel"] for e in edges])
    dist = np.linalg.norm(delta[:, :2], axis=1)
    link_noise = np.column_stack((0.025 + 0.025 * dist, 0.025 + 0.025 * dist, 0.009 + 0.013 * dist))
    anchors = np.unique(np.r_[0, np.arange(6, len(times), 6), len(times) - 1])
    prior_noise = np.tile([0.45, 0.45, 0.12], (len(anchors), 1))
    prior_noise[0] = [0.002, 0.002, 0.001]

    def residual(params):
        correction = params.reshape(-1, 3)
        nodes = baseline + correction
        local = relative(nodes[:-1], nodes[1:]) - delta
        left = ap + correction[ai] * (1 - af) + correction[ai + 1] * af
        right = bp + correction[bi] * (1 - bf) + correction[bi + 1] * bf
        loop = relative(left, right) - obs
        local[:, 2] = wrap(local[:, 2])
        loop[:, 2] = wrap(loop[:, 2])
        return np.concatenate(
            [
                (local / link_noise).ravel(),
                (loop / [0.09, 0.09, 0.038]).ravel(),
                (correction[anchors] / prior_noise).ravel(),
            ]
        )

    n = len(times)
    rows = n - 1 + len(edges) + len(anchors)
    jac = lil_matrix((rows * 3, n * 3), dtype="i1")
    dependencies = (
        [[i, i + 1] for i in range(n - 1)]
        + [[i, i + 1, j, j + 1] for i, j in zip(ai, bi)]
        + [[i] for i in anchors]
    )
    for k, cols in enumerate(dependencies):
        for col in cols:
            jac[3 * k : 3 * k + 3, 3 * col : 3 * col + 3] = 1
    fit = least_squares(
        residual,
        np.zeros(n * 3),
        jac_sparsity=jac.tocsr(),
        loss="huber",
        f_scale=1.5,
        max_nfev=400,
        ftol=1e-6,
        x_scale="jac",
        tr_options={"atol": 1e-8, "btol": 1e-8, "maxiter": 1500},
    )
    corr = fit.x.reshape(-1, 3)
    corrected = track.copy()
    for k in range(3):
        corrected[:, k + 1] += np.interp(track[:, 0], times, corr[:, k])
    for edge in edges:
        p, q = sample(corrected, np.array([edge["a"], edge["b"]]) * 1e-9)
        edge["fused"] = relative(p, q).tolist()
    return (
        corrected,
        edges,
        dict(
            status=int(fit.status),
            cost=float(fit.cost),
            function_evaluations=fit.nfev,
            message=fit.message,
            maximum_shift_m=float(np.linalg.norm(corr[:, :2], axis=1).max()),
        ),
    )


def world_scans(track, rec):
    t, xy = rec["times"], rec["xy"]
    bt = t[:, None] + rec["increments"][:, None] * np.arange(xy.shape[1])
    base = sample(track, bt)
    return transform(base, xy), transform(sample(track, t), rec["origin"]), bt


def grid(out, world, origins, train, bounds):
    xmin, ymin, xmax, ymax = bounds
    res = 0.05
    width, height = np.ceil((np.array([xmax, ymax]) - np.array([xmin, ymin])) / res).astype(int)
    used = np.zeros((height, width), np.uint16)
    hits = np.zeros_like(used)

    def pixel(xy):
        px = np.floor((xy - np.array([xmin, ymin])) / res).astype(np.int32)
        px[..., 1] = height - 1 - px[..., 1]
        return px

    origin_pix = pixel(origins)
    for i in np.flatnonzero(train):
        cloud = world[i]
        targets = pixel(cloud[np.isfinite(cloud[:, 0])])
        if len(targets) == 0:
            continue
        o = origin_pix[i]
        block = np.vstack([targets, o])
        lo = np.maximum(block.min(0), 0)
        hi = np.minimum(block.max(0) + 1, [width, height])
        if np.any(hi <= lo):
            continue
        w, h = hi - lo
        mask = np.zeros((h, w), np.uint8)
        lines = np.empty((len(targets), 2, 2), dtype="i4")
        lines[:, 0, :] = o - lo
        lines[:, 1, :] = targets - lo
        cv2.polylines(mask, lines, False, 1, 1)
        hit = np.zeros_like(mask)
        pts = targets - lo
        valid = (pts[:, 0] >= 0) & (pts[:, 1] >= 0) & (pts[:, 0] < w) & (pts[:, 1] < h)
        hit[pts[valid, 1], pts[valid, 0]] = 1
        sl = np.s_[lo[1] : hi[1], lo[0] : hi[0]]
        used[sl] += mask
        hits[sl] += hit
    pixels = np.full(used.shape, 205, dtype="u1")
    observed = used > 1
    pixels[observed] = 254
    pixels[observed & (hits.astype(float) / np.maximum(1, used) >= 0.18) & (hits >= 2)] = 0
    out.mkdir(parents=True, exist_ok=True)
    Image.fromarray(pixels).save(out / "map.pgm")
    spec = dict(
        image="map.pgm",
        resolution=res,
        origin=[float(xmin), float(ymin), 0.0],
        negate=0,
        occupied_thresh=0.65,
        free_thresh=0.196,
        mode="trinary",
    )
    (out / "map.yaml").write_text(yaml.safe_dump(spec, sort_keys=False))
    return wall_eval.Map.read(out)


def diagnostic(root, edges, train, fused, scans, field, align):
    out = root / "results/fusion"
    field_hits, field_orig, bt = world_scans(field, scans)
    table = wall_eval.Map.read(root / "input/lidar")
    withheld = np.arange(len(scans["times"])) % 8 == 0
    valid = np.isfinite(field_hits[withheld, :, 0])
    times = bt[withheld][valid]
    body = scans["xy"][withheld][valid]
    direction = np.arctan2(body[:, 1] - scans["origin"][1], body[:, 0] - scans["origin"][0])
    rays = np.column_stack([times, body, direction])
    # Use only the common timestamp span; the field checkpoint ended before the recorder.
    common = (times >= max(field[0, 0], train[0, 0])) & (times <= min(field[-1, 0], train[-1, 0]))
    rays = rays[common]
    hits, robot = wall_eval.project(field, rays)
    walls = wall_eval.revisits(table, field, rays, hits, robot, rays[0, 0])
    rot, shift = align
    wall_scores = {}
    for name, track in (("lidar", train), ("fused", fused)):
        xy, _ = wall_eval.project(track, rays)
        wall_scores[name], q = wall_eval.score_pairs(xy @ rot + shift, walls)
    # Select a real observed wall with reproducible split on the field map.
    wall_rows = {}
    for name, (a, b, n, visits) in walls.items():
        if not visits:
            continue
        nearby = []
        for edge in edges:
            pos = sample(field, np.array([edge["a"], edge["b"]]) * 1e-9)
            v = b - a
            fraction = np.clip(((pos[:, :2] - a) @ v) / (v @ v), 0, 1)
            dist = np.linalg.norm(pos[:, :2] - a - fraction[:, None] * v, axis=1)
            if max(dist) < 6:
                nearby.append(edge)
        wall_rows[name] = dict(camera_verified_nearby=len(nearby), paired_epochs=len(visits))
    content = dict(heldout_wall_beams=len(rays), scores=wall_scores, support=wall_rows)
    (out / "walls.json").write_text(json.dumps(content, indent=2))
    return walls, rays, content


def run(root):
    out = root / "results/fusion"
    report = json.loads((out / "visit-constraints.json").read_text())
    train = load_track(root / "input/lidar/training.csv")
    field = load_track(root / "input/lidar/field.csv")
    fused, used, solver = optimize(train, report["edges"])
    np.savetxt(
        out / "trajectory.csv",
        fused,
        delimiter=",",
        fmt=["%.9f", "%.8f", "%.8f", "%.8f"],
        header="timestamp_s,x_m,y_m,yaw_rad",
        comments="",
    )
    (out / "used-edges.json").write_text(json.dumps(used))
    rec = np.load(root / "input/lidar/scans.npz")
    within = (rec["times"] >= train[0, 0]) & (
        rec["times"] + rec["increments"] * (rec["xy"].shape[1] - 1) <= train[-1, 0]
    )
    held = (np.arange(len(within)) % 8 == 0) & within
    training = (np.arange(len(within)) % 8 != 0) & within
    w0, o0, _ = world_scans(train, rec)
    w1, o1, _ = world_scans(fused, rec)
    cloud = w0[training]
    bounds = np.r_[np.floor(np.nanmin(cloud, axis=(0, 1)) - 2), np.ceil(np.nanmax(cloud, axis=(0, 1)) + 2)]
    qa = {}
    for name, xy, origin in (("lidar", w0, o0), ("fused", w1, o1)):
        g = grid(out / name, xy, origin, training, bounds)
        h = xy[held]
        h = h[np.isfinite(h[:, 0])] if h.ndim == 2 else h[np.isfinite(h[:, :, 0])]
        qa[name] = wall_eval.fit(h, g)
        qa[name]["beams"] = len(h)
        qa[name]["route"] = wall_eval.route_check(
            g, sample(fused if name == "fused" else train, rec["times"][held])[:, :2]
        )
    times = np.linspace(max(train[0, 0], field[0, 0]), min(train[-1, 0], field[-1, 0]), 600)
    align = wall_eval.rigid(sample(train, times)[:, :2], sample(field, times)[:, :2])
    walls, rays, details = diagnostic(root, used, train, fused, rec, field, align)
    metrics = dict(
        training_scans=int(training.sum()),
        heldout_scans=int(held.sum()),
        solver=solver,
        accepted_edges=len(used),
        test=qa,
        walls=details,
    )
    (out / "metrics.json").write_text(json.dumps(metrics, indent=2))
    print(json.dumps({k: v for k, v in metrics.items() if k != "walls"}))


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("root", type=Path)
    run(p.parse_args().root)
