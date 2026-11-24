"""Locate RGB-D landmark matches on the walls marked in the field checkpoint."""

import cv2
import numpy as np

from geometry import camera, sample, transform


def identify(root, edges, walls, field, lidar, fused, align):
    k, mount = camera(root)

    def body_points(stamp, uv):
        d = cv2.imread(str(root / "input/depth" / f"{stamp}.png"), cv2.IMREAD_UNCHANGED)
        coords = np.rint(uv * 0.5).astype(int)
        z = d[np.clip(coords[:, 1], 0, 239), np.clip(coords[:, 0], 0, 319)] * 0.001
        eye = np.column_stack(((uv[:, 0] - k[0, 2]) * z / k[0, 0], (uv[:, 1] - k[1, 2]) * z / k[1, 1], z))
        return eye @ mount[:3, :3].T + mount[:3, 3], z

    samples = {w: [] for w in walls}
    for ed in edges:
        stamps = ed["a"], ed["b"]
        uv = np.array(ed["matches"]).reshape(-1, 2, 2)
        body_a, z_a = body_points(stamps[0], uv[:, 0])
        body_b, z_b = body_points(stamps[1], uv[:, 1])
        valid = (z_a > 0.28) & (z_b > 0.28) & (body_a[:, 2] > 0.15) & (body_a[:, 2] < 2.8)
        ts = np.array(stamps) * 1e-9
        lands = {}
        for name, track in (("field", field), ("lidar", lidar), ("fused", fused)):
            pose = sample(track, ts)
            x = transform(pose[0], body_a[:, :2])
            y = transform(pose[1], body_b[:, :2])
            if name != "field":
                x = x @ align[0] + align[1]
                y = y @ align[0] + align[1]
            lands[name] = x, y
        for wall, (a, b, n, _) in walls.items():
            v = b - a
            t = v / np.linalg.norm(v)
            x, y = lands["field"]
            mask = valid & (np.abs((x - a) @ n) < 0.45) & (np.abs((y - a) @ n) < 0.65)
            mask &= ((x - a) @ t >= 0) & ((x - b) @ t <= 0) & ((y - a) @ t >= -0.5) & ((y - b) @ t <= 0.5)
            if mask.sum() < 3:
                continue
            spread = np.ptp((x[mask] - a) @ t)
            if spread < 0.18:
                continue
            gaps = {
                name: float(np.median(np.abs((a0[mask] - a1[mask]) @ n)) * 100)
                for name, (a0, a1) in lands.items()
            }
            samples[wall].append(
                dict(
                    a=ed["a"],
                    b=ed["b"],
                    wall=wall,
                    matches_on_wall=int(mask.sum()),
                    landmark_spread_m=float(spread),
                    normal_gap_cm=gaps,
                    indices=np.flatnonzero(mask).tolist(),
                )
            )
    return samples
