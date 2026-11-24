"""Compare the stored collision geometries in link space, in millimetres."""

import argparse
from collections import Counter
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import igl
import numpy as np
from scipy.spatial.transform import Rotation
import trimesh

REPO = Path(__file__).resolve().parents[2]
from optimize_cad_collision import Mesh, read_mesh
from verify_cad_collision_refit import solid_hull

PK = Path("ros2_ws/src/cleany_description")


def model(path: Path) -> ET.Element:
    return ET.parse(path / PK / "urdf/cleany.urdf").getroot()


def get_mesh(c: ET.Element, package: Path) -> Mesh:
    g = c.find("geometry")[0]
    t = c.find("origin")
    move = np.fromstring(t.get("xyz"), sep=" ") * 1000
    rot = Rotation.from_euler("xyz", np.fromstring(t.get("rpy"), sep=" "))
    if g.tag == "mesh":
        uri = g.get("filename").removeprefix("package://cleany_description/")
        p = package / uri
        if not p.exists():
            p = REPO / PK / uri
        q = read_mesh(p)
        v = q.vertices * np.fromstring(g.get("scale", "1 1 1"), sep=" ") * 1000
        f = q.faces
    elif g.tag == "box":
        q = trimesh.creation.box(extents=np.fromstring(g.get("size"), sep=" ") * 1000)
        v, f = q.vertices, q.faces
    elif g.tag == "cylinder":
        q = trimesh.creation.cylinder(
            radius=float(g.get("radius")) * 1000,
            height=float(g.get("length")) * 1000,
            sections=96,
        )
        v, f = q.vertices, q.faces
    else:
        raise ValueError(g.tag)
    return Mesh(rot.apply(v) + move, f)


def split_centers(tri, spacing):
    """Bisect long edges to sample broad faces as well as their vertices."""
    allpoints = []
    for _ in range(24):
        e = np.linalg.norm(tri[:, [1, 2, 0]] - tri, axis=2)
        long = e.max(1) > spacing
        allpoints.append(tri[~long].mean(1))
        if not long.any():
            break
        t = tri[long]
        i = e[long].argmax(1)
        a, b, c = np.take_along_axis(
            t, ((i[:, None] + np.arange(3)) % 3)[:, :, None], axis=1
        ).transpose(1, 0, 2)
        mid = (a + b) / 2
        tri = np.concatenate((np.stack((a, mid, c), 1), np.stack((b, c, mid), 1)))
    return np.concatenate(allpoints)


def points(ms, spacing):
    return np.vstack(
        [
            v
            for m in ms
            for v in (m.vertices, split_centers(m.vertices[m.faces], spacing))
        ]
    )


def material(m, pts):
    """Measure signed distance with negative values inside a closed mesh."""
    t = igl.AABB()
    t.init(m.vertices, m.faces)
    lo = m.vertices.min(0)
    hi = m.vertices.max(0)
    dist = np.sqrt(
        t.squared_distance(m.vertices, m.faces, np.ascontiguousarray(pts))[0]
    )
    ix = np.where(
        ((pts >= lo - 0.00001) & (pts <= hi + 0.00001)).all(1) & (dist > 0.00001)
    )[0]
    if len(ix):
        dist[ix] *= np.where(
            np.abs(
                igl.fast_winding_number(
                    m.vertices, m.faces, np.ascontiguousarray(pts[ix])
                )
            )
            > 0.5,
            -1.0,
            1.0,
        )
    return dist


def assembly(ms, pts):
    return np.minimum.reduce([material(m, pts) for m in ms])


def gap(link, ms, balls):
    if link == "base_link" or not balls.get(link, {}).get("spheres"):
        return None
    p = np.vstack([v for m in ms for v in (m.vertices, m.vertices[m.faces].mean(1))])
    sb = balls[link]["spheres"]
    c = np.array([b["center"] for b in sb]) * 1000
    r = np.array([b["radius"] for b in sb]) * 1000
    return max(0.0, float((np.linalg.norm(p[:, None] - c, axis=2) - r).min(1).max()))


def canonical(e):
    return ET.canonicalize(ET.tostring(e), strip_text=True)


def count(tree, pkg):
    stat = {s: Counter() for s in ("world", "self", "all", "wrist", "gripper")}
    for l in tree.findall("link"):
        ln = l.get("name")
        for c in l.findall("collision"):
            nm = c.get("name")
            g = c.find("geometry")[0]
            ism = g.tag == "mesh"
            tags = ["all"] + ([] if nm.startswith("cad_world__") else ["self"])
            if not nm.startswith("cad_self__"):
                tags.append("world")
                if "wrist_" in ln:
                    tags.append("wrist")
                if "_gripper_" in ln:
                    tags.append("gripper")
            if ism:
                m = get_mesh(c, pkg)
                if "cad_enclosure__" in nm:
                    solid_hull(Mesh((m.vertices - m.vertices.mean(0)) / 1000, m.faces))
            for tag in tags:
                row = stat[tag]
                row[g.tag] += 1
                if ism:
                    row["vertices"] += len(np.unique(m.vertices, axis=0))
                    row["triangles"] += len(m.faces)
                    if "cad_enclosure__" in nm:
                        row["enclosure_vertices"] += len(m.vertices)
                        row["enclosure_triangles"] += len(m.faces)
    return stat


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--previous",
        type=Path,
        required=True,
        help="Previously qualified repository snapshot",
    )
    parser.add_argument(
        "--candidate", type=Path, required=True, help="Fitted repository snapshot"
    )
    parser.add_argument(
        "--installed",
        type=Path,
        required=True,
        help="Original robot.urdf and cleany_description package",
    )
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    previous, candidate = args.previous, args.candidate
    base, final = model(previous), model(candidate)
    profile = json.loads(
        (candidate / "configs/robot/cad_collision_optimized.json").read_text()
    )
    raw_tree = ET.parse(args.installed / "robot.urdf").getroot()
    balls = json.loads(
        (REPO / PK / "config/geometry/cad_runtime_gpu_primitives.json").read_text()
    )["links"]
    report = {
        "before": count(base, previous / PK),
        "after": count(final, candidate / PK),
        "groups": [],
    }
    for old, new in zip(base.findall("link"), final.findall("link"), strict=True):
        assert old.get("name") == new.get("name")
        for k in ("visual", "inertial"):
            assert [canonical(x) for x in old.findall(k)] == [
                canonical(x) for x in new.findall(k)
            ]
        if "wheel" in old.get("name") or "_tip_" in old.get("name"):
            assert canonical(old) == canonical(new)
        for c in old.findall("collision"):
            if "Camera_FixedMount_collision" in c.get("name"):
                assert canonical(c) == canonical(
                    new.find(f'collision[@name="{c.get("name")}"]')
                )
    assert base.attrib == final.attrib and [
        canonical(x) for x in base.findall("joint")
    ] == [canonical(x) for x in final.findall("joint")]
    previous_shapes_by_scope = {}
    for g in profile["rigid_collision_groups"]:
        link_name = g["link"]
        previous_link = base.find(f'link[@name="{link_name}"]')
        source_link = raw_tree.find(f'link[@name="{link_name}"]')
        fitted_link = final.find(f'link[@name="{link_name}"]')
        names = g["members"]
        previous_shapes = [
            get_mesh(previous_link.find(f'collision[@name="{n}"]'), previous / PK)
            for n in names
        ]
        source_shapes = [
            get_mesh(
                source_link.find(f'collision[@name="{n}"]'),
                args.installed / "cleany_description",
            )
            for n in names
        ]
        fitted_collisions = [
            c
            for c in fitted_link.findall("collision")
            if c.get("name").split("rigid_fit_")[-1].rsplit("_", 1)[0] == g["name"]
        ]
        assert len(fitted_collisions) == len(g["shapes"])
        fitted_shapes = [get_mesh(c, candidate / PK) for c in fitted_collisions]
        spacing = 0.65 if "gripper_" in link_name else 1.0
        source_points = points(source_shapes, spacing)
        fitted_points = points(fitted_shapes, spacing)
        miss = float(max(0, assembly(fitted_shapes, source_points).max()))
        extra = float(max(0, assembly(source_shapes, fitted_points).max()))
        key = (link_name, names[0].startswith("cad_self__"))
        if key not in previous_shapes_by_scope:
            selected = [
                c
                for c in previous_link.findall("collision")
                if not c.get("name").startswith(
                    "cad_world__" if key[1] else "cad_self__"
                )
            ]
            previous_shapes_by_scope[key] = [
                get_mesh(c, previous / PK) for c in selected
            ]
        local_extra = assembly(previous_shapes, fitted_points)
        exposed = fitted_points[local_extra > 1.2]
        total_excess = (
            float(max(0, local_extra.max()))
            if not len(exposed)
            else max(
                1.2,
                float(max(0, assembly(previous_shapes_by_scope[key], exposed).max())),
            )
        )
        previous_gap, fitted_gap = (
            gap(link_name, previous_shapes, balls),
            gap(link_name, fitted_shapes, balls),
        )
        if "cad_self__" not in names[0] and previous_gap is not None:
            assert fitted_gap <= max(0.01015, previous_gap + 0.00015), (
                link_name,
                fitted_gap,
                previous_gap,
            )
        report["groups"].append(
            dict(
                link=link_name,
                members=names,
                spacing_mm=spacing,
                previous_assembly_excess_upper_sample_mm=total_excess,
                max_source_outside_proxy_sample_mm=miss,
                max_proxy_outside_source_sample_mm=extra,
                previous_gpu_gap_mm=previous_gap,
                new_gpu_gap_mm=fitted_gap,
            )
        )
    # Free space bracketed by material on either side of each U-shaped camera support, including co-located parts.
    for side in ["left", "right"]:
        ln = "cad_" + side + "_wrist_roll"
        bl = base.find(f'link[@name="{ln}"]')
        fl = final.find(f'link[@name="{ln}"]')
        c = next(
            c
            for c in bl.findall("collision")
            if "Camera_FixedMount_collision" in c.get("name")
        )
        m = get_mesh(c, previous / PK)
        axes = [
            np.arange(np.floor(x) - 1, np.ceil(y) + 2, 1.0)
            for x, y in zip(m.vertices.min(0), m.vertices.max(0))
        ]
        shape = tuple(len(x) for x in axes)
        p = np.array(np.meshgrid(*axes, indexing="ij")).reshape(3, -1).T
        ds = material(m, p).reshape(shape)
        within = ds <= 0.0
        across = np.zeros(shape, dtype=bool)
        for i in (0, 1):
            across |= np.maximum.accumulate(within, axis=i) & np.flip(
                np.maximum.accumulate(np.flip(within, axis=i), axis=i), axis=i
            )
        pp = p[(across & (ds > 0.5)).ravel()]
        orig = [get_mesh(e, previous / PK) for e in bl.findall("collision")]
        new = [get_mesh(e, candidate / PK) for e in fl.findall("collision")]
        d = assembly(orig, pp)
        dp = assembly(new, pp)
        report[side + "_mount_opening"] = {
            "grid_mm": 1.0,
            "free_points_half_mm_margin": int((d > 0.5).sum()),
            "free_points_occupied_after": int(((d > 0.5) & (dp <= 0)).sum()),
            "minimum_margin_for_previously_free_1mm_points": float(dp[d > 1].min()),
        }
    p = args.output
    p.write_text(json.dumps(report, indent=2) + "\n")
    for side in ("left", "right"):
        if report[side + "_mount_opening"]["free_points_occupied_after"]:
            raise ValueError(f"Fitted geometry occupies the {side} wrist mount opening")
    print(json.dumps({k: v for k, v in report.items() if k != "groups"}))
    print(
        "largest geometric changes",
        sorted(
            [
                (
                    r["max_source_outside_proxy_sample_mm"],
                    r["max_proxy_outside_source_sample_mm"],
                    r["link"],
                )
                for r in report["groups"]
            ],
            reverse=True,
        )[:5],
    )


if __name__ == "__main__":
    main()
