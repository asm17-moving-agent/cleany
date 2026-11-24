"""Compare final Cartographer poses on identical lidar beams and repeat visits."""

import argparse
import json
import math
import sqlite3
from collections import defaultdict
from dataclasses import dataclass
from itertools import combinations
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import yaml
from PIL import Image
from rclpy.serialization import deserialize_message
from scipy.ndimage import distance_transform_edt, label as connected_components, map_coordinates
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage


@dataclass
class Map:
    pixels: np.ndarray
    x: float
    y: float
    res: float
    occupied_threshold: float
    free_threshold: float

    @classmethod
    def read(cls, folder):
        info = yaml.safe_load((folder / "map.yaml").read_text())
        assert info["origin"][2] == 0 and not info["negate"]
        return cls(
            np.asarray(Image.open(folder / info["image"])),
            *info["origin"][:2],
            info["resolution"],
            info["occupied_thresh"],
            info["free_thresh"],
        )

    @property
    def extent(self):
        h, w = self.pixels.shape
        return [self.x, self.x + w * self.res, self.y, self.y + h * self.res]

    def from_pixel(self, x, y):
        return np.array([self.x + (x + 0.5) * self.res, self.extent[3] - (y + 0.5) * self.res])


# Fixed before scoring candidates. Coordinates identify the field-checkpoint PGM.
WALLS = [
    ("A", (390, 322), (463, 322)),  # middle corridor, upper edge
    ("B", (496, 96), (497, 289)),  # large room, west wall
    ("C", (510, 349), (680, 345)),  # long lower corridor wall
    ("D", (615, 103), (617, 289)),  # room divider
    ("E", (381, 120), (383, 293)),  # west wing, east wall
]


def records(db, topic):
    return db.execute(
        "SELECT data FROM messages WHERE topic_id=(SELECT id FROM topics WHERE name=?) "
        "ORDER BY timestamp",
        (topic,),
    )


def beams(database, stride):
    with sqlite3.connect(database.as_uri() + "?mode=ro", uri=True) as db:
        transforms = deserialize_message(next(records(db, "/tf_static"))[0], TFMessage).transforms
        mount = next(
            t.transform
            for t in transforms
            if (t.header.frame_id, t.child_frame_id) == ("base_link", "lidar_link")
        )
        q = mount.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        blocks = []
        for i, (blob,) in enumerate(records(db, "/scan")):
            if i % stride:
                continue
            scan = deserialize_message(blob, LaserScan)
            indices = np.arange(0, len(scan.ranges), 3)
            rng = np.asarray(scan.ranges)[indices]
            valid = np.isfinite(rng) & (rng >= 0.30) & (rng <= 12.0)
            indices, rng = indices[valid], rng[valid]
            stamp = scan.header.stamp.sec + scan.header.stamp.nanosec / 1e9
            angle = scan.angle_min + indices * scan.angle_increment + yaw
            blocks.append(
                np.column_stack(
                    (
                        stamp + indices * scan.time_increment,
                        mount.translation.x + rng * np.cos(angle),
                        mount.translation.y + rng * np.sin(angle),
                        angle,
                    )
                )
            )
    return np.concatenate(blocks)


def poses(path):
    rows = np.genfromtxt(path, delimiter=",", names=True)
    assert len(np.unique(rows["trajectory"])) == 1 and np.all(np.diff(rows["time_ns"]) > 0)
    return np.column_stack((rows["time_ns"] / 1e9, rows["x"], rows["y"], np.unwrap(rows["yaw"])))


def at(track, times):
    return np.column_stack([np.interp(times, track[:, 0], track[:, i]) for i in (1, 2, 3)])


def project(track, rays):
    robot = at(track, rays[:, 0])
    cs, sn = np.cos(robot[:, 2]), np.sin(robot[:, 2])
    hit = np.column_stack(
        (
            robot[:, 0] + cs * rays[:, 1] - sn * rays[:, 2],
            robot[:, 1] + sn * rays[:, 1] + cs * rays[:, 2],
        )
    )
    return hit, robot


def rigid(source, target):
    a, b = source.mean(axis=0), target.mean(axis=0)
    u, _, vt = np.linalg.svd((source - a).T @ (target - b))
    fix = np.diag([1.0, np.linalg.det(u @ vt)])
    rotation = u @ fix @ vt
    return rotation, b - a @ rotation


def quant(values, p):
    return round(float(np.percentile(values, p)), 3)


def fit(hits, grid):
    dt = distance_transform_edt(grid.pixels > 255 * (1 - grid.occupied_threshold)) * grid.res
    rows = (grid.extent[3] - hits[:, 1]) / grid.res - 0.5
    cols = (hits[:, 0] - grid.x) / grid.res - 0.5
    distance = map_coordinates(dt, (rows, cols), order=1, mode="constant", cval=1000.0)
    return dict(
        p50_cm=quant(distance * 100, 50),
        p90_cm=quant(distance * 100, 90),
        p95_cm=quant(distance * 100, 95),
        within_10cm_pct=round(100 * float(np.mean(distance <= 0.10)), 2),
    )


def route_check(grid, points):
    """Check observed robot centers on the trinary Nav2 grid, without footprint inflation."""
    labels, _ = connected_components(grid.pixels > 255 * (1 - grid.free_threshold))
    row = np.floor((grid.extent[3] - points[:, 1]) / grid.res).astype(int)
    col = np.floor((points[:, 0] - grid.x) / grid.res).astype(int)
    inside = (row >= 0) & (col >= 0) & (row < labels.shape[0]) & (col < labels.shape[1])
    visited = np.zeros(len(row), dtype=int)
    visited[inside] = labels[row[inside], col[inside]]
    counts = np.bincount(visited)
    return dict(
        center_samples=len(row),
        known_free=int((visited != 0).sum()),
        in_largest_visited_free_component=int(max(counts[1:], default=0)),
    )


def revisits(refmap, reftrack, rays, field_hit, field_robot, start):
    """Match longitudinal wall bins seen on the same side after a 5 m excursion."""
    traveled = np.r_[0, np.cumsum(np.linalg.norm(np.diff(reftrack[:, 1:3], axis=0), axis=1))]
    result = {}
    for key, pixel_a, pixel_b in WALLS:
        a, b = refmap.from_pixel(*pixel_a), refmap.from_pixel(*pixel_b)
        tang = (b - a) / np.linalg.norm(b - a)
        norm = np.array([-tang[1], tang[0]])
        along, offset = (field_hit - a) @ tang, (field_hit - a) @ norm
        angle = rays[:, 3] + field_robot[:, 2]
        incidence = np.cos(angle) * norm[0] + np.sin(angle) * norm[1]
        side = np.sign((field_robot[:, :2] - a) @ norm)
        chosen = np.flatnonzero(
            (along >= 0)
            & (along <= np.linalg.norm(b - a))
            & (np.abs(offset) < 0.45)
            & (np.abs(incidence) > 0.40)
        )
        groups = defaultdict(list)
        for i in chosen:
            label = (int(along[i] / 0.75), int(side[i]), int((rays[i, 0] - start) / 4))
            groups[label].append(i)
        bins = defaultdict(list)
        for (segment, side_id, epoch), ids in groups.items():
            if len(ids) >= 5:
                bins[segment, side_id].append((epoch, np.asarray(ids)))
        pairs = []
        for keybin, visits in bins.items():
            for (u, iu), (v, iv) in combinations(sorted(visits, key=lambda x: x[0]), 2):
                t1, t2 = start + (u + 0.5) * 4, start + (v + 0.5) * 4
                d1, d2 = np.interp([t1, t2], reftrack[:, 0], traveled)
                if t2 - t1 >= 45 and d2 - d1 >= 5:
                    pairs.append((keybin, u, v, iu, iv))
        result[key] = (a, b, norm, pairs)
    return result


def score_pairs(hits, walls):
    metrics = {}
    all_medians = []
    for key, (a, _, normal, pairs) in walls.items():
        groups = defaultdict(list)
        cache = {}
        for band, u, v, iu, iv in pairs:
            for epoch, idx in ((u, iu), (v, iv)):
                if (band, epoch) not in cache:
                    cache[band, epoch] = float(np.median((hits[idx] - a) @ normal))
            groups[band].append(abs(cache[band, u] - cache[band, v]) * 100)
        per_bin = [np.median(values) for values in groups.values()]
        if per_bin:
            all_medians.extend(per_bin)
            metrics[key] = dict(
                bins=len(groups),
                pairs=len(pairs),
                median_cm=quant(per_bin, 50),
                p90_cm=quant(per_bin, 90),
            )
    return metrics, quant(all_medians, 50) if all_medians else None


def wall_sections(grid, rotation, shift, a, b, normal):
    """Count field-aligned cross sections with two occupied bands at least 15 cm apart."""
    along = np.arange(0.05, np.linalg.norm(b - a), 0.05)
    center = a + np.outer(along, (b - a) / np.linalg.norm(b - a))
    offsets = np.arange(-0.46, 0.461, 0.025)
    local = (center[:, None, :] + offsets[None, :, None] * normal - shift) @ rotation.T
    x = (local[:, :, 0] - grid.x) / grid.res - 0.5
    y = (grid.extent[3] - local[:, :, 1]) / grid.res - 0.5
    values = map_coordinates(grid.pixels.astype(float), (y, x), order=1, mode="constant", cval=128)
    splits = covered = 0
    for row in values:
        idx = np.flatnonzero(row <= 255 * (1 - grid.occupied_threshold))
        if len(idx):
            covered += 1
            splits += int(np.any(np.diff(idx) * 0.025 >= 0.15))
    return dict(profiles=len(along), occupied=covered, split=splits)


def main(args):
    root = args.run.resolve()
    out = args.out
    out.mkdir(parents=True, exist_ok=True)
    first = next((root / "input/checkpoints").iterdir())
    maps = {"field": Map.read(first)}
    tracks = {"field": poses(root / "results/field/trajectory.csv")}
    for name in args.names:
        folder = root / "results" / name
        maps[name], tracks[name] = Map.read(folder), poses(folder / "trajectory.csv")
    rays = beams(
        next((root / "input/complete/navigation").glob("*.db3")), 8 if args.held_out else 3
    )
    start, end = max(t[0, 0] for t in tracks.values()), min(t[-1, 0] for t in tracks.values())
    rays = rays[(rays[:, 0] >= start) & (rays[:, 0] <= end)]
    field = tracks["field"]
    ref, robot = project(field, rays)
    walls = revisits(maps["field"], field, rays, ref, robot, start)
    sample = np.arange(start, end, 1.0)
    target = at(field, sample)[:, :2]
    result = {
        "common_start_unix_s": start,
        "common_duration_s": round(end - start, 2),
        "held_out_only": args.held_out,
        "beams": len(rays),
        "cases": {},
    }
    fig, axes = plt.subplots(1, len(maps), figsize=(6 * len(maps), 4.4), squeeze=False)
    detail, closeups = plt.subplots(2, len(maps), figsize=(4.3 * len(maps), 6.5), squeeze=False)
    for ax, (name, grid) in zip(axes[0], maps.items()):
        track = tracks[name]
        hit, _ = project(track, rays)
        src = at(track, sample)[:, :2]
        R, shift = rigid(src, target)
        wall_metrics, combined = score_pairs(hit @ R + shift, walls)
        result["cases"][name] = dict(
            fit=fit(hit, grid),
            wall_repeat=wall_metrics,
            median_wall_bin_cm=combined,
            nodes=len(track),
            wall_sections={
                key: wall_sections(grid, R, shift, a, b, n) for key, (a, b, n, _) in walls.items()
            },
            track_span_s=round(track[-1, 0] - track[0, 0], 2),
            p95_field_path_shift_m=quant(np.linalg.norm(src @ R + shift - target, axis=1), 95),
            route_check=route_check(grid, src),
            registration_to_field={"R": R.tolist(), "shift": shift.tolist()},
        )
        ax.imshow(
            grid.pixels, cmap="gray", vmin=0, vmax=255, extent=grid.extent, interpolation="nearest"
        )
        ax.plot(track[:, 1], track[:, 2], color="#087e88", lw=0.7, alpha=0.65)
        for label, (a, b, _, _) in walls.items():
            seg = (np.stack((a, b)) - shift) @ R.T
            ax.plot(seg[:, 0], seg[:, 1], color="#f29932", lw=1.6)
            ax.text(
                seg[0, 0],
                seg[0, 1],
                label,
                color="#623200",
                fontsize=8,
                bbox=dict(facecolor="white", edgecolor="none", alpha=0.8, pad=0.7),
            )
        caption = {
            "field": "Field checkpoint",
            "6m-rotation": "6 m search, lower odometry yaw weight",
        }.get(name, name)
        ax.set_title(caption)
        ax.set_aspect("equal")
        ax.set_xlabel("m")
        for j, label in enumerate(("A", "B")):
            panel = closeups[j, list(maps).index(name)]
            a, b, normal, pairs = walls[label]
            tang = (b - a) / np.linalg.norm(b - a)
            along = np.linspace(0, np.linalg.norm(b - a), 400)
            across = np.linspace(-0.75, 0.75, 140)
            positions = a + along[None, :, None] * tang + across[:, None, None] * normal
            local = (positions - shift) @ R.T
            cols = (local[:, :, 0] - grid.x) / grid.res - 0.5
            rows = (grid.extent[3] - local[:, :, 1]) / grid.res - 0.5
            pixels = map_coordinates(
                grid.pixels.astype(float), (rows, cols), order=1, mode="constant", cval=128
            )
            panel.imshow(
                pixels,
                extent=[0, along[-1], -0.75, 0.75],
                origin="lower",
                aspect="auto",
                cmap="gray",
                vmin=0,
                vmax=255,
            )
            if pairs:
                # Show a pair near the field's measured median separation for this wall.
                support = np.array([min(len(p[3]), len(p[4])) for p in pairs])
                eligible = [p for p, s in zip(pairs, support) if s >= np.median(support)]
                field_gaps = np.array(
                    [
                        abs(np.median(ref[p[3]] @ normal) - np.median(ref[p[4]] @ normal))
                        for p in eligible
                    ]
                )
                target_gap = result["cases"]["field"]["wall_repeat"][label]["median_cm"] / 100
                _, u, v, iu, iv = eligible[np.argmin(abs(field_gaps - target_gap))]
                gap_cm = 100 * abs(
                    np.median((hit[iu] @ R + shift) @ normal)
                    - np.median((hit[iv] @ R + shift) @ normal)
                )
                result["cases"][name].setdefault("shown_visits", {})[label] = dict(
                    seconds_from_common_start=[u * 4, v * 4],
                    gap_cm=round(float(gap_cm), 2),
                    beam_counts=[len(iu), len(iv)],
                )
                for epoch, idx, color in ((u, iu, "#0077c8"), (v, iv, "#e65d00")):
                    loc = hit[idx] @ R + shift - a
                    panel.scatter(
                        loc @ tang,
                        loc @ normal,
                        s=4,
                        color=color,
                        alpha=0.65,
                        label=f"{epoch*4//60:02d}:{epoch*4%60:02d}",
                    )
                panel.legend(loc="upper right", fontsize=8, title="replay (mm:ss)")
            panel.set_xlim(0, along[-1])
            panel.set_ylim(across[0], across[-1])
            panel.set_title(f"{caption}  /  wall {label}", fontsize=10)
            panel.set_xlabel("Along wall (m)")
            if name == "field":
                panel.set_ylabel("Across wall (m)")
    fig.tight_layout()
    fig.savefig(out / "overview.png", dpi=160)
    plt.close(fig)
    detail.tight_layout()
    detail.savefig(out / "wall-comparison.png", dpi=160)
    plt.close(detail)
    (out / "metrics.json").write_text(json.dumps(result, indent=2) + "\n")


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("run", type=Path)
    p.add_argument("out", type=Path)
    p.add_argument(
        "--held-out", action="store_true", help="score only scans withheld from training"
    )
    p.add_argument("names", nargs="+")
    main(p.parse_args())
