"""Export visible RGB-D surfaces, visual graph components and quantitative tracking summaries."""

import argparse
import json
import sqlite3
from collections import defaultdict
from pathlib import Path

import cv2
import numpy as np
from scipy.spatial.transform import Rotation

from geometry import camera, load_track, sample, relative, wrap

OPTICAL = np.array([[0, 0, 1, 0], [-1, 0, 0, 0], [0, -1, 0, 0], [0, 0, 0, 1.0]], dtype=float)


def read_graph(out):
    rows = np.atleast_1d(
        np.genfromtxt(out / "all-components.csv", names=True, delimiter=",", dtype=None, encoding=None)
    )
    times = {
        int(r[0]): (int(r[1]), int(r[2]))
        for r in np.atleast_2d(np.loadtxt(out / "keyframes.csv", delimiter=",", skiprows=1, dtype="i8"))
    }
    graph = {}
    for row in rows:
        node = int(row["id"])
        if node > 0 and node in times:
            m = np.eye(4)
            m[:3, :] = np.array(tuple(row)[1:], float).reshape(3, 4)
            graph[node] = (times[node][0], m)
    # Component labels come from graph connectivity, not odometry reset numbers.
    optimized_count = len(graph)
    parent = {i: i for i in graph}

    def root(i):
        while parent[i] != i:
            parent[i] = parent[parent[i]]
            i = parent[i]
        return i

    unique = {}
    with sqlite3.connect((out / "map.db").resolve().as_uri() + "?mode=ro", uri=True) as con:
        weights = dict(con.execute("select id,weight from Node"))
        recorded = {}
        for uid, seg, blob in con.execute("select id,map_id,pose from Node"):
            if len(blob or b"") == 48:
                pose = np.eye(4)
                pose[:3] = np.frombuffer(blob, dtype="<f4").reshape(3, 4)
                recorded[uid] = (seg, pose)
        links = list(con.execute("select from_id,to_id,type,transform from Link"))
        # RTAB-Map stores rehearsed frames (-9) as a relative transform to their retained keyframe.
        for _ in range(len(weights)):
            before = len(graph)
            for x, y, typ, data in links:
                if (
                    weights.get(x) == -9
                    and x in times
                    and x not in graph
                    and y in graph
                    and len(data or b"") == 48
                ):
                    child_T_parent = np.eye(4)
                    child_T_parent[:3] = np.frombuffer(data, dtype="<f4").reshape(3, 4)
                    if abs(np.linalg.det(child_T_parent[:3, :3]) - 1) < 0.002:
                        relative_pose = np.linalg.inv(child_T_parent)
                    elif x in recorded and y in recorded and recorded[x][0] == recorded[y][0]:
                        # A rehearsal link can be null. Use the stored odometry within the same map session.
                        relative_pose = np.linalg.inv(recorded[y][1]) @ recorded[x][1]
                    else:
                        continue
                    graph[x] = (times[x][0], graph[y][1] @ relative_pose)
                    parent[x] = root(y)
            if len(graph) == before:
                break
        for x, y, t, data in links:
            if x in graph and y in graph and x != y and t in (0, 1, 2, 3, 4, 6):
                parent[root(x)] = root(y)
                if weights.get(x) != -9 and weights.get(y) != -9:
                    unique[(min(x, y), max(x, y), t)] = 1
        db_nodes = con.execute("select count(*) from Node").fetchone()[0]
    groups = defaultdict(list)
    for i in graph:
        groups[root(i)].append(i)
    groups = sorted(groups.values(), key=len, reverse=True)
    if not groups:
        raise ValueError(f"No optimized graph in {out}")
    poses = sorted((graph[i] for i in groups[0]), key=lambda g: g[0])
    with (out / "trajectory.tum").open("w") as f:
        for ns, m in poses:
            q = Rotation.from_matrix(m[:3, :3]).as_quat()
            f.write(f"{ns//10**9}.{ns%10**9:09} " + " ".join(f"{v:.8f}" for v in np.r_[m[:3, 3], q]) + "\n")
    tracking = np.genfromtxt(out / "tracking.csv", delimiter=",", names=True, dtype=None, encoding=None)
    successes = tracking["tracked"] == 1
    metric = dict(
        total_frames=len(tracking),
        tracked=int(successes.sum()),
        tracking_pct=round(100 * successes.mean(), 2),
        odometry_segments=int(len(np.unique(tracking["segment"]))),
        database_nodes=db_nodes,
        optimized_nodes=optimized_count,
        located_nodes_including_rehearsals=len(graph),
        component_nodes=[len(g) for g in groups],
        main_start_ns=int(poses[0][0]),
        main_end_ns=int(poses[-1][0]),
        global_loops=sum(t == 1 for _, _, t in unique),
        long_visual_links=sum(t in (1, 2, 3) and abs(graph[x][0] - graph[y][0]) > 40e9 for x, y, t in unique),
    )
    return poses, metric


def compare_track(poses, lidar, rig_T_base):
    ns = np.array([t for t, _ in poses], dtype="i8")
    visual = np.stack([m @ rig_T_base for _, m in poses])
    target = sample(lidar, ns / 1e9)
    source = visual[:, :2, 3]
    # Rigid XY alignment only. Distances and stereo scale are not rescaled.
    x = source - source.mean(0)
    y = target[:, :2] - target[:, :2].mean(0)
    u, _, vt = np.linalg.svd(x.T @ y)
    r = u @ np.diag([1.0, np.linalg.det(u @ vt)]) @ vt
    aligned = x @ r + target[:, :2].mean(0)
    residual = np.linalg.norm(aligned - target[:, :2], axis=1)
    return np.column_stack((ns / 1e9, aligned)), dict(
        comparison_nodes=len(ns),
        visual_span_m=float(np.linalg.norm(np.ptp(source, axis=0))),
        reference_span_m=float(np.linalg.norm(np.ptp(target[:, :2], axis=0))),
        visual_path_m=float(np.linalg.norm(np.diff(source, axis=0), axis=1)[np.diff(ns) < 5e9].sum()),
        reference_path_same_intervals_m=float(
            np.linalg.norm(np.diff(target[:, :2], axis=0), axis=1)[np.diff(ns) < 5e9].sum()
        ),
        p50_m=float(np.median(residual)),
        p90_m=float(np.percentile(residual, 90)),
        max_m=float(np.max(residual)),
    )


def moving_windows(out, lidar, rig_T_base):
    """Compare 5 s base-frame increments during reference motion, including lost windows in the denominator."""
    with sqlite3.connect((out / "map.db").as_uri() + "?mode=ro", uri=True) as con:
        records = list(con.execute("select stamp,map_id,pose from Node where id>0 order by stamp"))
    clock = np.array([row[0] for row in records])
    start = np.arange(np.ceil(lidar[0, 0]), lidar[-1, 0] - 5.0, 5.0)
    ref = relative(sample(lidar, start), sample(lidar, start + 5.0))
    moving = np.linalg.norm(ref[:, :2], axis=1) >= 0.40
    errors = []
    for t, truth in zip(start[moving], ref[moving]):
        i = int(np.argmin(abs(clock - t)))
        j = int(np.argmin(abs(clock - t - 5.0)))
        if (
            j <= i
            or max(abs(clock[i] - t), abs(clock[j] - t - 5)) > 0.65
            or np.any(np.diff(clock[i : j + 1]) > 2.15)
        ):
            continue
        if any(row[1] != records[i][1] for row in records[i : j + 1]):
            continue

        def pose(blob):
            mat = np.eye(4)
            mat[:3] = np.frombuffer(blob, dtype="<f4").reshape(3, 4)
            return mat @ rig_T_base

        delta = np.linalg.inv(pose(records[i][2])) @ pose(records[j][2])
        # Account for sub-keyframe time differences by querying the reference at the actual image stamps.
        truth = relative(*sample(lidar, clock[[i, j]]))
        errors.append(
            (
                float(np.linalg.norm(delta[:2, 3] - truth[:2])),
                float(abs(np.rad2deg(wrap(np.arctan2(delta[1, 0], delta[0, 0]) - truth[2])))),
            )
        )
    e = np.array(errors).reshape(-1, 2)
    return dict(
        eligible=int(moving.sum()),
        available=len(e),
        within_25cm_and_7deg=int(((e[:, 0] <= 0.25) & (e[:, 1] <= 7)).sum()),
        translation_p50_m=float(np.median(e[:, 0])) if len(e) else None,
        rotation_p50_deg=float(np.median(e[:, 1])) if len(e) else None,
    )


def write_cloud(path, xyz, colors):
    fields = np.dtype([("xyz", "<f4", 3), ("rgb", "u1", 3)])
    records = np.empty(len(xyz), dtype=fields)
    records["xyz"] = xyz
    records["rgb"] = colors
    header = (
        "ply\nformat binary_little_endian 1.0\n"
        + "element vertex "
        + str(len(xyz))
        + "\n"
        + "property float x\nproperty float y\nproperty float z\nproperty uchar red\n"
        + "property uchar green\nproperty uchar blue\nend_header\n"
    )
    with path.open("wb") as f:
        f.write(header.encode("ascii"))
        records.tofile(f)


def assemble(root, frames, name, k):
    # Each frame contributes one sample per 4 cm voxel; retain voxels observed in multiple frames.
    voxel = 0.04
    occupied = []
    colors = []
    xx, yy = np.meshgrid(np.arange(0, 320, 3), np.arange(0, 240, 3))
    rays = np.stack(((2 * xx - k[0, 2]) / k[0, 0], (2 * yy - k[1, 2]) / k[1, 1], np.ones_like(xx)), axis=-1)
    for stamp, pose in frames:
        d = cv2.imread(str(root / "input/depth" / f"{stamp}.png"), cv2.IMREAD_UNCHANGED)
        color = cv2.imread(str(root / "input/rgb" / f"{stamp}.jpg"))
        if d is None or color is None:
            raise FileNotFoundError(stamp)
        z = d[yy, xx] * 0.001
        valid = (z > 0.35) & (z < 5.0)
        pts = (rays[valid] * z[valid, None]) @ pose[:3, :3].T + pose[:3, 3]
        bgr = color[2 * yy[valid], 2 * xx[valid]]
        cell = np.floor(pts / voxel).astype("i8")
        # Three signed 20-bit coordinates packed for fast per-frame deduplication.
        cell += 2**19
        if np.any((cell < 0) | (cell >= 2**20)):
            raise ValueError("Voxel coordinate overflow")
        key = (cell[:, 0] << 40) | (cell[:, 1] << 20) | cell[:, 2]
        uniq, idx = np.unique(key, return_index=True)
        occupied.append(uniq)
        colors.append(bgr[idx, ::-1])
    keys = np.concatenate(occupied)
    rgb = np.concatenate(colors)
    unique, inverse, count = np.unique(keys, return_inverse=True, return_counts=True)
    mask = count >= 2
    rgb = (
        np.stack(
            [
                np.bincount(inverse, weights=rgb[:, j], minlength=len(count))[mask] / count[mask]
                for j in range(3)
            ],
            axis=1,
        )
        .round()
        .astype("u1")
    )
    key = unique[mask]
    scale = 2**20 - 1
    xyz = (np.column_stack((key >> 40, (key >> 20) & scale, key & scale)).astype(float) - 2**19 + 0.5) * voxel
    out = root / "results/rgbd"
    write_cloud(out / (name + ".ply"), xyz, rgb)
    return (
        xyz,
        rgb,
        dict(frames=len(frames), vertices=len(xyz), voxel_m=voxel, range_m=[0.35, 5.0], min_views=2),
    )


def matrices(root):
    k, mount = camera(root)
    # Fixed driver-to-CAD RGB optical frame is the mount convention used for global registration.
    model = json.loads((root / "input/manifest/00.json").read_text())["transforms"]

    def path(frame):
        mat = np.eye(4)
        while frame in model:
            v = model[frame]
            m = np.eye(4)
            m[:3, :3] = Rotation.from_quat(v["quat"]).as_matrix()
            m[:3, 3] = v["xyz"]
            mat = m @ mat
            frame = v["parent"]
        return frame, mat

    parent, color = path("camera_color_optical_frame")
    parent2, left = path("camera_infra1_optical_frame")
    if parent != parent2:
        raise ValueError("RealSense optical frames do not share a root")
    return k, mount, mount @ np.linalg.inv(color) @ left


def run(root):
    k, mount, base_T_left = matrices(root)
    lidar = load_track(root / "input/lidar/training.csv")
    summary = {}
    for mode, base_T_optical in (
        ("rgbd", mount),
        ("stereo", base_T_left),
        ("stereo-filtered", base_T_left),
        ("stereo-depth", base_T_left),
    ):
        out = root / "results" / mode
        poses, metrics = read_graph(out)
        rig_T_base = np.eye(4) if mode == "stereo-depth" else OPTICAL @ np.linalg.inv(base_T_optical)
        metrics["moving_windows"] = moving_windows(out, lidar, rig_T_base)
        aligned, metrics["relative_to_lidar"] = compare_track(poses, lidar, rig_T_base)
        np.savetxt(out / "aligned-to-lidar.csv", aligned, delimiter=",", header="time_s,x_m,y_m", comments="")
        if mode == "rgbd":
            _, _, metrics["independent_cloud"] = assemble(
                root, [(t, m @ OPTICAL) for t, m in poses], "visual-component", k
            )
        summary[mode] = metrics
    summary["primary_stereo"] = max(
        ("stereo", "stereo-filtered", "stereo-depth"),
        key=lambda mode: (
            summary[mode]["moving_windows"]["within_25cm_and_7deg"],
            summary[mode]["moving_windows"]["available"],
        ),
    )
    track = np.loadtxt(root / "results/fusion/trajectory.csv", delimiter=",", skiprows=1)
    frames = []
    last = -np.inf
    for path in sorted((root / "input/rgb").glob("*.jpg")):
        ns = int(path.stem)
        t = ns / 1e9
        if t < track[0, 0] or t > track[-1, 0] or t - last < 0.8:
            continue
        x, y, angle = sample(track, t)
        m = np.eye(4)
        m[:3, :3] = Rotation.from_euler("z", angle).as_matrix()
        m[:2, 3] = [x, y]
        frames.append((ns, m @ mount))
        last = t
    xyz, rgb, summary["global_cloud"] = assemble(root, frames, "floor19-color", k)
    # A uniform index sample is taken after voxelization for the lightweight local viewer.
    rng = np.random.default_rng(19)
    keep = rng.choice(len(xyz), size=min(100000, len(xyz)), replace=False)
    np.savez_compressed(root / "results/rgbd/preview.npz", xyz=xyz[keep].astype("f4"), rgb=rgb[keep])
    (root / "results/visual-metrics.json").write_text(json.dumps(summary, indent=2))
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("root", type=Path)
    run(p.parse_args().root)
