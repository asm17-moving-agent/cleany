#!/usr/bin/env python3
"""Render documentation figures from repository settings and existing test fixtures.

Requires matplotlib, numpy, scipy, PyYAML, pytest and opencv-python-headless
(the last two support existing calibration fixture imports).
Run from any directory; --check regenerates in memory and compares bytes and inputs.
These figures describe configuration or synthetic fixtures, never physical results.
"""

from __future__ import annotations

import argparse
import ast
import hashlib
import io
from importlib.metadata import version
import json
import re
import runpy
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon, Rectangle
import numpy as np
import yaml
from mpl_toolkits.mplot3d.art3d import Poly3DCollection


ROOT = Path(__file__).resolve().parents[2]
PKG = "ros2_ws/src/cleany_"
NAV = PKG + "navigation/"
MANIFEST = ROOT / "docs/assets/visuals/generated.json"
GREEN, ORANGE, BLUE, RED = "#19776d", "#c47525", "#386ca0", "#b94b49"
plt.rcParams.update({"font.size": 12, "axes.spines.top": False,
                     "axes.spines.right": False, "figure.facecolor": "white",
                     "axes.titleweight": "bold", "savefig.facecolor": "white"})


def read(path: str):
    text = (ROOT / path).read_text()
    return json.loads(text) if path.endswith(".json") else yaml.safe_load(text)


def digest(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def axes(title: str, size=(6.4, 5.8)):
    fig, ax = plt.subplots(figsize=size, layout="constrained")
    ax.set_title(title, pad=16)
    ax.grid(alpha=.16)
    return fig, ax


def metric(ax, x="base_link X / forward (m)", y="base_link Y / left (m)"):
    ax.set(xlabel=x, ylabel=y, aspect="equal")


def fixture_namespace(path: str, function: str | None = None) -> dict:
    """Reuse fixture construction and assertions without running the test suite."""
    namespace = runpy.run_path(str(ROOT / path))
    if function is None:
        return namespace
    tree = ast.parse((ROOT / path).read_text())
    node = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == function)
    node.body.append(ast.Return(ast.Call(ast.Name("locals", ast.Load()), [], [])))
    compiled = ast.fix_missing_locations(ast.Module(body=[node], type_ignores=[]))
    exec(compile(compiled, path, "exec"), namespace)
    return namespace[function]()


def footprint():
    robot = NAV + "config/phone_lidar_robot.yaml"
    safety = NAV + "config/phone_lidar_safety.yaml"
    r = read(robot)["local_costmap"]["local_costmap"]["ros__parameters"]
    s = read(safety)["collision_monitor"]["ros__parameters"]
    wheel = np.array(json.loads(r["footprint"]))
    stop = np.array(json.loads(s["StopZone"]["points"]))
    slow = np.array(json.loads(s["SlowZone"]["points"]))
    fig, ax = axes("Phone: wheel envelope and 2D zones")
    for points, color, label in [(slow, ORANGE, "Slow boundary"),
                                  (stop, RED, "Stop / padded footprint"),
                                  (wheel, GREEN, "CAD wheel envelope")]:
        ax.add_patch(Polygon(points, facecolor=color, edgecolor=color, alpha=.18))
        loop = np.vstack([points, points[0]])
        ax.plot(*loop.T, color=color, lw=2, label=label)
    assert np.allclose(np.sort(np.abs(stop), axis=0)[0],
                       np.max(np.abs(wheel), axis=0) + r["footprint_padding"])
    ax.scatter([0], [0], color="black", s=20)
    ax.annotate("+X", (.18, 0), (0, 0), arrowprops={"arrowstyle": "->"})
    ax.set(xlim=(-.55, .55), ylim=(-.60, .60))
    metric(ax)
    ax.legend(loc="upper center", bbox_to_anchor=(.5, -.18), fontsize=10)
    return fig, [robot, safety], "Equal-scale configured polygons; no braking-distance claim."


def direction_cost():
    config = NAV + "config/nav2_phone_omni.yaml"
    code = NAV + "mppi/src/critics/directional_motion_critic.cpp"
    params = read(config)["controller_server"]["ros__parameters"]["FollowPath"]["DirectionalMotionCritic"]
    # Keep this mathematical illustration tied to the documented implementation.
    assert "lateral*sin(theta)^2 + diagonal*sin(2*theta)^2" in (ROOT / code).read_text()
    theta = np.linspace(-np.pi, np.pi, 1441)
    cost = params["lateral_weight"] * np.sin(theta)**2 + params["diagonal_weight"] * np.sin(2*theta)**2
    fig, ax = axes("Body-frame direction preference", (6.4, 5.2))
    ax.plot(np.degrees(theta), cost, color=GREEN, lw=2.4)
    ax.set(xlabel="Velocity direction relative to +X (degrees)", ylabel="Additional cost / metre",
           xticks=[-180, -90, 0, 90, 180], ylim=(-.6, 23))
    for x, label in [(0, "forward"), (90, "left"), (-90, "right")]:
        ax.annotate(label, (x, 0), (x, 4), ha="center", fontsize=10,
                    arrowprops={"arrowstyle": "->", "color": "gray"})
    return fig, [config, code], "Analytic critic cost using configured weights; not a motion result."


def lattice():
    source = NAV + "config/lattice/omni-r10cm-balanced.json"
    data = read(source)
    paths = [p for p in data["primitives"] if p["start_angle_index"] == 0]
    fig, ax = axes("Balanced lattice: start heading 0")
    rotations = 0
    for i, p in enumerate(paths):
        points = np.vstack([[0, 0, 0], p["poses"]])
        color = plt.get_cmap("tab10")(i % 10)
        if np.max(np.abs(points[:, :2])) < 1e-8:
            rotations += 1
            angle = points[-1, 2]
            ax.annotate("", (.025*np.cos(angle), .025*np.sin(angle)), (0, 0),
                        arrowprops={"arrowstyle": "->", "color": color, "lw": 2})
        else:
            ax.plot(points[:, 0], points[:, 1], "o-", ms=3, lw=2, color=color)
            end = points[-1]
            ax.arrow(end[0], end[1], .013*np.cos(end[2]), .013*np.sin(end[2]),
                     width=.0008, head_width=.004, color=color)
    metric(ax, "Local X / forward (m)", "Local Y / left (m)")
    ax.set_title(f"Balanced lattice: heading 0\n{len(paths)} primitives; {rotations} rotate in place")
    ax.text(.02, .02, "Dots: stored poses\nArrows: final yaw", transform=ax.transAxes, fontsize=10,
            bbox={"facecolor": "white", "alpha": .9, "edgecolor": "none"})
    ax.margins(.25)
    return fig, [source], "Stored primitive poses at heading index 0; straight segments join samples."


def wrist_axes():
    source = PKG + "description/config/geometry/cad_final.json"
    transform = np.array(read(source)["wrist_cameras"]["left"])
    fig = plt.figure(figsize=(6.4, 6.4), layout="constrained")
    ax = fig.add_subplot(projection="3d")
    origin = transform[:3, 3] * 1000
    for index, color in enumerate([RED, GREEN, BLUE]):
        v = transform[:3, index] * 25
        ax.quiver(*origin, *v, color=color, arrow_length_ratio=.15, lw=2)
        ax.text(*(origin+v*1.15), "optical " + "XYZ"[index], color=color, fontsize=11)
    ax.scatter(*origin, color="black", s=30)
    ax.scatter(0, 0, 0, color=ORANGE, s=30)
    ax.plot([0, origin[0]], [0, origin[1]], [0, origin[2]], "--", color="gray")
    ax.text2D(.03, .03, "Orange point: jaw origin", transform=ax.transAxes, fontsize=10)
    ax.set(xlabel="Jaw X (mm)", ylabel="Jaw Y (mm)", zlabel="Jaw Z (mm)",
           xlim=(-110, 20), ylim=(-100, 25), zlim=(-45, 55),
           title="CAD-final left wrist optical frame")
    ax.set_box_aspect((1, 1, 1)); ax.view_init(23, 35)
    return fig, [source], "Approximate geometry transform, not measured hand-eye calibration."


def bins():
    source = PKG + "mujoco_sim/config/robot_top_bins.yaml"
    data = read(source)
    fig, ax = axes("Simulation: rear tray openings")
    tray = data["internal_tray"]
    size = np.array(tray["size_xy_m"]); center = np.array(tray["center_xy_m"])
    ax.add_patch(Rectangle(center-size/2, *size, facecolor="#e7ebea", edgecolor="gray"))
    for item, color, label in zip(data["bins"], [GREEN, ORANGE], ["Lost items / left", "Trash / right"]):
        center = np.array(item["center_xy_m"]); outer = np.array(item["outside_size_m"][:2])
        inner = outer - 2*item["wall_thickness_m"]
        ax.add_patch(Rectangle(center-outer/2, *outer, color=color, alpha=.75))
        ax.add_patch(Rectangle(center-inner/2, *inner, facecolor="white", edgecolor=color))
        ax.text(*center, f"{label}\n{inner[0]*1000:.0f} × {inner[1]*1000:.0f} mm", ha="center", va="center", fontsize=10)
    metric(ax); ax.set(xlim=(-.24, .12), ylim=(-.28, .28))
    ax.scatter(0, 0, color="black", s=20)
    ax.annotate("", (.07, 0), (0, 0), arrowprops={"arrowstyle": "->"})
    ax.text(.005, .025, "+X forward", fontsize=10)
    return fig, [source], "Plan view from YAML; opening dimensions exclude walls; simulation concept only."


def depth_mask():
    source = NAV + "config/d435_fixed_body_boxes.json"
    boxes = read(source)["boxes"]
    fig, axs = plt.subplots(2, 1, figsize=(6.4, 10), layout="constrained")
    for ax, horizontal, label in zip(axs, [0, 1], ["X / forward", "Y / left"]):
        for box in boxes:
            c, h = np.array(box["center"]), np.array(box["half_size"])
            ax.add_patch(Rectangle((c[horizontal]-h[horizontal], c[2]-h[2]),
                                  2*h[horizontal], 2*h[2], facecolor=GREEN,
                                  edgecolor=GREEN, alpha=.18))
        ax.autoscale_view()
        metric(ax, f"base_link {label} (m)", "base_link Z / up (m)")
        ax.set_title("Fixed AABBs: " + ("side (XZ)" if horizontal == 0 else "front (YZ)"))
        ax.margins(.08)
    return fig, [source], "XZ and YZ projections of fixed AABBs, excluding moving arms; not the full runtime 3D safety model."


def alignment():
    test = NAV + "test/test_localization_alignment.py"
    core = NAV + "cleany_navigation/localization_alignment.py"
    ns = fixture_namespace(test)
    model, points, pose = ns["scene"]()
    fig, axs = plt.subplots(2, 1, figsize=(6.4, 10), layout="constrained")
    for ax, current, title in zip(axs, [pose, pose+[.5, -.4, .1]], ["Fixture pose", "Same scan; wrong pose (+0.5, -0.4 m, +0.1 rad)"]):
        world = ns["transform"](points, current)
        ax.imshow(model.cells, origin="lower", extent=(0, 10, 0, 10), cmap="Greys", vmin=0, vmax=100)
        ax.scatter(*world.T, s=7, color=ORANGE, label="Projected scan endpoints")
        ax.scatter(*current[:2], marker="^", s=50, color=BLUE, label="Pose")
        ax.set_title(title, fontsize=12); metric(ax, "Map X (m)", "Map Y (m)")
        ax.legend(loc="upper left", fontsize=9)
    return fig, [test, core], "Exact scene() fixture and core transform; synthetic scan, not a recorded run."


def joint_sync():
    test = PKG + "handeye_calibration/test/test_joint_state_sync.py"
    core = PKG + "handeye_calibration/cleany_handeye_calibration/joint_state_sync.py"
    ns = fixture_namespace(test, "test_interpolation_uses_names_not_message_order_and_keeps_extras")
    before, after, sample = ns["before"], ns["after"], ns["interpolation"]
    name = before.joint_names[0]
    t = [before.stamp_ns, after.stamp_ns]
    q = [before.positions_rad[0], after.positions_rad[after.joint_names.index(name)]]
    fig, ax = axes("Joint feedback at image capture time", (6.4, 5.2))
    ax.plot(t, q, "o-", lw=2, color=GREEN, label="Feedback samples / linear interpolation")
    image_time = sample.image_stamp_ns
    image_q = sample.sample.positions_rad[0]
    ax.scatter([image_time], [image_q], color=ORANGE, s=70, zorder=5)
    ax.axvline(image_time, color=ORANGE, linestyle="--")
    ax.annotate(f"Image: {image_time} ns\nq = {image_q:g} rad; ratio = {sample.ratio:g}",
                (image_time, image_q), (image_time+15, image_q-1.5), fontsize=11)
    ax.set(xlabel="Fixture timestamp (ns)", ylabel="First joint position (rad)", xticks=[t[0], image_time, t[1]], ylim=(-2, 11))
    ax.legend(loc="upper left", fontsize=9)
    return fig, [test, core], "Existing unit-test values and core interpolation; nanosecond spacing is not a hardware sample rate."


def octomap_cells():
    source = PKG + "scene_mapping/test/test_known_geometry.cpp"
    text = (ROOT / source).read_text()
    partial = text.split("TEST(KnownGeometry, PreservesCellWhoseCenterButNotCornersIsInside)")[1].split("TEST(")[0]
    size, center = [np.array([float(v) for v in values.split(",")]) for values in
                    re.search(r"box\(\{([^}]+)\}, \{([^}]+)\}\)", partial).groups()]
    cell_size = float(re.search(r"OcTree tree\(([^)]+)\)", partial)[1])
    fig, ax = axes("Observed leaf crossing a shape boundary", (6.4, 5.6))
    scale = 1000
    ax.add_patch(Rectangle((center[:2]-size[:2]/2)*scale, *(size[:2]*scale), facecolor=GREEN, alpha=.18, label="Known box (9 mm wide)"))
    ax.add_patch(Rectangle((center[:2]-cell_size/2)*scale, cell_size*scale, cell_size*scale,
                          fill=False, edgecolor=RED, linewidth=2.4, label="Occupied leaf (10 mm wide): retained"))
    for x in [0, 10]:
        for y in [0, 10]: ax.scatter(x, y, color=RED, s=45)
    ax.scatter(*(center[:2]*scale), color=BLUE, s=35, label="Centre is inside")
    metric(ax, "Fixture X (mm)", "Fixture Y (mm)")
    ax.set(xlim=(-5, 15), ylim=(-12, 22)); ax.legend(loc="upper center", bbox_to_anchor=(.5, -.2), fontsize=10)
    return fig, [source], "XY section at z=5 mm of C++ fixture; actual containment checks all eight 3D corners."


def collision_shape():
    settings = "configs/robot/arm_collision_meshes.json"
    reader = "tools/optimize_arm_collision_meshes.py"
    package = read(settings)["description_package"]
    filename = "Aligned_AssembledElbowBSocket.stl"
    inputs = [f"{package}/meshes/cad_final/{filename}", f"{package}/meshes/cad_final/collision/{filename}"]
    loader = runpy.run_path(str(ROOT / reader))["read_stl"]
    meshes = [loader(ROOT / path) for path in inputs]
    scale = read(settings).get("mesh_scales", {}).get(filename, 1.) * 1000
    lo = np.min(np.vstack([m.vertices for m in meshes]), axis=0) * scale
    hi = np.max(np.vstack([m.vertices for m in meshes]), axis=0) * scale
    fig = plt.figure(figsize=(6.4, 10), layout="constrained")
    for i, (mesh, title) in enumerate(zip(meshes, ["CAD visual", "Optimized collision"])):
        ax = fig.add_subplot(2, 1, i+1, projection="3d")
        triangles = mesh.vertices[mesh.faces] * scale
        ax.add_collection3d(Poly3DCollection(triangles, facecolors=GREEN if i == 0 else ORANGE,
                                            edgecolor="none", shade=True))
        ax.set(xlim=(lo[0], hi[0]), ylim=(lo[1], hi[1]), zlim=(lo[2], hi[2]),
               xlabel="Mesh X (mm)", ylabel="Mesh Y (mm)", zlabel="Mesh Z (mm)",
               title=f"{title}: {len(mesh.faces):,} triangles")
        ax.set_box_aspect(hi-lo); ax.view_init(25, -55); ax.set_proj_type("ortho")
        ax.locator_params(nbins=4)
        ax.tick_params(labelsize=9, pad=1)
        for axis in [ax.xaxis, ax.yaxis, ax.zaxis]:
            axis.label.set_size(10)
    return fig, inputs + [settings, reader], "Current CAD elbow socket and its convex collision envelope, identical view; not a whole-robot collision result."


FIGURES = [
    ("footprint", NAV + "docs/assets/footprint.png", footprint),
    ("direction-cost", NAV + "docs/assets/direction-cost.png", direction_cost),
    ("lattice", NAV + "docs/assets/lattice.png", lattice),
    ("alignment", NAV + "docs/assets/alignment.png", alignment),
    ("depth-mask", NAV + "docs/assets/depth-mask.png", depth_mask),
    ("wrist-axes", PKG + "description/docs/assets/wrist-axes.png", wrist_axes),
    ("bins", PKG + "mujoco_sim/docs/assets/bins.png", bins),
    ("joint-sync", PKG + "handeye_calibration/docs/assets/joint-sync.png", joint_sync),
    ("octomap-cells", PKG + "scene_mapping/docs/assets/octomap-cells.png", octomap_cells),
    ("collision-shape", PKG + "description/docs/assets/collision-shape.png", collision_shape),
]


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args()
    for package in ["navigation", "handeye_calibration"]:
        sys.path.insert(0, str(ROOT / (PKG + package)))
    records = []
    for name, path, renderer in FIGURES:
        fig, inputs, scope = renderer()
        output = io.BytesIO()
        fig.savefig(output, format="png", dpi=160, metadata={"Software": "Cleany contract figures"})
        plt.close(fig)
        content = output.getvalue()
        target = ROOT / path
        if args.check:
            if not target.is_file() or target.read_bytes() != content:
                raise SystemExit(f"Stale figure: {path}")
        else:
            target.parent.mkdir(parents=True, exist_ok=True)
            target.write_bytes(content)
        inputs.append(str(Path(__file__).resolve().relative_to(ROOT)))
        records.append({"id": name, "asset": path, "scope": scope,
                        "sha256": digest(content), "inputs": [
                            {"path": p, "sha256": digest((ROOT / p).read_bytes())} for p in inputs]})
    manifest = json.dumps({"command": "python3 tools/docs/render_contract_figures.py",
                           "renderer_versions": {name: version(name) for name in
                               ["matplotlib", "numpy", "scipy", "PyYAML", "pytest", "opencv-python-headless"]},
                           "assets": records}, ensure_ascii=False, indent=2) + "\n"
    if args.check:
        if not MANIFEST.is_file() or MANIFEST.read_text() != manifest:
            raise SystemExit("Stale generated figure manifest")
    else:
        MANIFEST.parent.mkdir(parents=True, exist_ok=True)
        MANIFEST.write_text(manifest)
    print(f"{'Verified' if args.check else 'Rendered'} {len(records)} figures and input hashes")


if __name__ == "__main__":
    main()
