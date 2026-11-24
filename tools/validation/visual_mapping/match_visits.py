"""Measure RGB-D revisit constraints and check each transform on independent training LiDAR returns."""

import argparse
import json
from functools import lru_cache
from pathlib import Path

import cv2
import numpy as np
from scipy.spatial import cKDTree

from geometry import camera, load_track, relative, sample, transform, wrap


def features(root, cache):
    cache.mkdir(exist_ok=True)
    detector = cv2.SIFT_create(nfeatures=850, contrastThreshold=0.026)
    seen = set()
    for img in sorted((root / "input/rgb").glob("*.jpg")):
        ns = int(img.stem)
        if ns // 10**9 in seen:
            continue
        seen.add(ns // 10**9)
        output = cache / (img.stem + ".npz")
        if output.is_file():
            continue
        gray = cv2.imread(str(img), cv2.IMREAD_GRAYSCALE)
        depth = cv2.imread(str(root / "input/depth" / (img.stem + ".png")), cv2.IMREAD_UNCHANGED)
        if gray is None or depth is None:
            continue
        kp, desc = detector.detectAndCompute(gray, None)
        if desc is None:
            np.savez(output, uv=np.empty((0, 2), "f4"), z=np.empty(0, "f4"), desc=np.empty((0, 128), "u1"))
            continue
        uv = np.asarray([q.pt for q in kp], "f4")
        pix = np.rint(uv / 2).astype(int)
        z = (
            depth[np.clip(pix[:, 1], 0, depth.shape[0] - 1), np.clip(pix[:, 0], 0, depth.shape[1] - 1)]
            * 0.001
        )
        np.savez(output, uv=uv, z=z.astype("f4"), desc=np.rint(desc).clip(0, 255).astype("u1"))


@lru_cache(maxsize=48)
def read_feature(path):
    with np.load(path) as d:
        desc = d["desc"].astype("f4")
        uv, z = d["uv"], d["z"]
    kd = cv2.FlannBasedMatcher(dict(algorithm=1, trees=3), dict(checks=48))
    if len(desc) >= 2:
        kd.add([desc])
        kd.train()
    return uv, z, desc, kd


def estimate(a, b, k, mount):
    ua, za, da, matcher = read_feature(a)
    ub, zb, db, _ = read_feature(b)
    if min(len(da), len(db)) < 20:
        return
    pairs = [
        (m.trainIdx, m.queryIdx, m.distance)
        for m, n in matcher.knnMatch(db, k=2)
        if m.distance < 0.73 * n.distance
    ]
    # A repetitive door or chair should not give many votes to a single feature.
    pairs.sort(key=lambda pair: pair[2])
    used, indices = set(), []
    for i, j, _ in pairs:
        if i not in used and 0.28 < za[i] < 6.0 and 0.28 < zb[j] < 6.0:
            used.add(i)
            indices.append((i, j))
    if len(indices) < 17:
        return
    ia, ib = np.array(indices).T

    def backproject(uv, z):
        return np.column_stack(
            ((uv[:, 0] - k[0, 2]) * z / k[0, 0], (uv[:, 1] - k[1, 2]) * z / k[1, 1], z)
        ).astype("f8")

    xa, xb = backproject(ua[ia], za[ia]), backproject(ub[ib], zb[ib])
    ok, rv, tv, inliers = cv2.solvePnPRansac(
        xb,
        ua[ia],
        k,
        None,
        iterationsCount=150,
        reprojectionError=3,
        confidence=0.998,
        flags=cv2.SOLVEPNP_EPNP,
    )
    if not ok or inliers is None or len(inliers) < 15:
        return
    ids = inliers.ravel()
    rot, trans = cv2.Rodrigues(rv)[0], tv.ravel()
    ids = ids[np.linalg.norm(xb[ids] @ rot.T + trans - xa[ids], axis=1) < 0.09 + 0.015 * za[ia[ids]]]
    if len(ids) < 15 or len(ids) / len(indices) < 0.32:
        return
    pts = ua[ia[ids]]
    if cv2.contourArea(cv2.convexHull(pts)) < 0.024 * 640 * 480:
        return
    rv, tv = cv2.solvePnPRefineLM(xb[ids], pts, k, None, rv, tv)
    mat = np.eye(4)
    mat[:3, :3], mat[:3, 3] = cv2.Rodrigues(rv)[0], tv.ravel()
    planar = mount @ mat @ np.linalg.inv(mount)
    if abs(planar[2, 3]) > 0.20 or planar[2, 2] < np.cos(np.deg2rad(12)):
        return
    rel = np.r_[planar[:2, 3], np.arctan2(planar[1, 0], planar[0, 0])]
    errors = np.linalg.norm(xb[ids] @ mat[:3, :3].T + mat[:3, 3] - xa[ids], axis=1)
    return rel, dict(
        inliers=len(ids),
        correspondences=len(indices),
        depth_residual_m=float(np.median(errors)),
        matches=np.column_stack((pts, ub[ib[ids]])).round(1).tolist(),
    )


def scan_check(points_a, points_b, initial):
    a, b = points_a[np.isfinite(points_a[:, 0])], points_b[np.isfinite(points_b[:, 0])]
    if min(len(a), len(b)) < 30:
        return
    tree = cKDTree(a)
    current = transform(initial, b)
    for _ in range(8):
        dist, ix = tree.query(current)
        mask = dist < min(0.28, np.quantile(dist, 0.75))
        if mask.sum() < 24:
            return
        x, y = current[mask], a[ix[mask]]
        u, _, vt = np.linalg.svd((x - x.mean(0)).T @ (y - y.mean(0)))
        rot = u @ np.diag([1, np.linalg.det(u @ vt)]) @ vt
        current = current @ rot + y.mean(0) - x.mean(0) @ rot
    u, _, vt = np.linalg.svd((b - b.mean(0)).T @ (current - current.mean(0)))
    r = u @ np.diag([1, np.linalg.det(u @ vt)]) @ vt
    refined = np.r_[current.mean(0) - b.mean(0) @ r, np.arctan2(r[0, 1], r[0, 0])]
    d1 = tree.query(current)[0]
    d2 = cKDTree(current).query(a)[0]
    overlap = min(float(np.mean(d1 < 0.16)), float(np.mean(d2 < 0.16)))
    return refined, overlap


def process(root, cache):
    # A partially transferred split may have populated extra cached features during a pilot.
    selected = {}
    for image in sorted((root / "input/rgb").glob("*.jpg")):
        selected.setdefault(int(image.stem) // 10**9, cache / (image.stem + ".npz"))
    files = list(selected.values())
    cv2.setRNGSeed(19)
    times = np.array([int(f.stem) / 1e9 for f in files])
    track = load_track(root / "input/lidar/training.csv")
    pose = sample(track, times)
    traveled = np.r_[0, np.cumsum(np.linalg.norm(np.diff(pose[:, :2], axis=0), axis=1))]
    k, mount = camera(root)
    record = np.load(root / "input/lidar/scans.npz")
    st, xy = record["times"], record["xy"]
    train_idx = np.flatnonzero(np.arange(len(st)) % 8 != 0)
    train_clock = cKDTree(st[train_idx, None])
    candidates, visual, edges = 0, 0, []
    for j in range(0, len(files), 2):
        idx = np.flatnonzero(
            (times[j] - times > 40)
            & (traveled[j] - traveled > 5)
            & (np.linalg.norm(pose[:, :2] - pose[j, :2], axis=1) < 2.3)
            & (np.abs(wrap(pose[:, 2] - pose[j, 2])) < 1.12)
        )
        cost = np.linalg.norm(pose[idx, :2] - pose[j, :2], axis=1) + 0.6 * np.abs(
            wrap(pose[idx, 2] - pose[j, 2])
        )
        proposals = []
        for pos in np.argsort(cost):
            i = idx[pos]
            if all(abs(times[i] - times[old]) > 8 for old in proposals):
                proposals.append(i)
            if len(proposals) == 4:
                break
        for i in proposals:
            candidates += 1
            measured = estimate(files[i], files[j], k, mount)
            if measured is None:
                continue
            visual += 1
            rel, quality = measured
            prior = relative(pose[i], pose[j])
            if np.linalg.norm(rel[:2] - prior[:2]) > 1.05 or abs(wrap(rel[2] - prior[2])) > 0.38:
                continue
            si, sj = train_idx[train_clock.query(np.array([[times[i]], [times[j]]]))[1]]
            if max(abs(st[si] - times[i]), abs(st[sj] - times[j])) > 0.28:
                continue
            # Compare at the scan times, compensating the small camera / LiDAR time offset.
            ti, tj = sample(track, st[[si, sj]])

            def se2(p):
                c, s = np.cos(p[2]), np.sin(p[2])
                return np.array([[c, -s, p[0]], [s, c, p[1]], [0, 0, 1]])

            visual_scan = (
                np.linalg.inv(se2(ti)) @ se2(pose[i]) @ se2(rel) @ np.linalg.inv(se2(pose[j])) @ se2(tj)
            )
            init = np.r_[visual_scan[:2, 2], np.arctan2(visual_scan[1, 0], visual_scan[0, 0])]
            check = scan_check(xy[si], xy[sj], init)
            if check is None:
                continue
            refine, overlap = check
            diff = float(np.linalg.norm(refine[:2] - init[:2]))
            angle = abs(float(wrap(refine[2] - init[2])))
            if overlap < 0.52 or diff > 0.16 or angle > 0.09:
                continue
            edges.append(
                dict(
                    a=int(files[i].stem),
                    b=int(files[j].stem),
                    rel=rel.tolist(),
                    prior=prior.tolist(),
                    scan_overlap=overlap,
                    icp_delta_m=diff,
                    icp_delta_rad=angle,
                    **quality
                )
            )
    return dict(proposed=candidates, visual_verified=visual, scan_verified=len(edges), edges=edges)


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("root", type=Path)
    p.add_argument("cache", type=Path)
    p.add_argument("--features-only", action="store_true")
    args = p.parse_args()
    cv2.setNumThreads(1)
    cv2.setRNGSeed(19)
    features(args.root, args.cache)
    if not args.features_only:
        report = process(args.root, args.cache)
        dest = args.root / "results/fusion/visit-constraints.json"
        dest.write_text(json.dumps(report))
        print({k: v for k, v in report.items() if k != "edges"}, flush=True)
