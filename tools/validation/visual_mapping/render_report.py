"""Render offline review figures and a self-contained interactive colored point cloud."""

import argparse
import base64
import json
from pathlib import Path

import cv2
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from PIL import Image
from scipy.spatial.distance import cdist

from fuse_map import diagnostic, wall_eval, world_scans
from geometry import load_track, sample, wrap, relative
from wall_evidence import identify


def cloud_view(data, dest):
    xyz, rgb = data["xyz"], data["rgb"]
    center = (np.percentile(xyz, 1, axis=0) + np.percentile(xyz, 99, axis=0)) / 2
    coords = np.rint((xyz - center) * 100).astype("<i2")
    encode = lambda array: base64.b64encode(array.tobytes()).decode()
    page = """<!doctype html><html lang="ko"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>19층 RGB-D 컬러 점군</title><style>*{box-sizing:border-box}body{margin:0;background:#111a22;color:#eff4f2;font:16px system-ui,sans-serif}canvas{width:100vw;height:100vh;display:block;touch-action:none;cursor:grab}aside{position:fixed;left:22px;top:18px;max-width:86vw;background:#15212ee8;padding:14px 20px;border-radius:10px;line-height:1.6}button{background:#314754;color:white;border:0;border-radius:4px;padding:6px 12px;cursor:pointer}</style><canvas id="cloud"></canvas><aside><strong>19층 RGB-D 컬러 점군</strong><br>드래그로 회전 · 휠로 확대 · Shift+드래그로 이동 &nbsp; <button id="reset">시점 복원</button> <button id="height">높이로 보기</button><br><small>표시 __VERTEX_COUNT__개 · 전체 해상도는 PLY 파일</small></aside><script>
const canvas=document.getElementById('cloud'),gl=canvas.getContext('webgl',{antialias:true});
if(!gl){document.querySelector('aside').append(' · WebGL을 켜거나 PLY 파일을 열어주세요.');throw Error('WebGL');}
function bytes(b){return Uint8Array.from(atob(b),c=>c.charCodeAt(0))}
function shader(type,source){const s=gl.createShader(type);gl.shaderSource(s,source);gl.compileShader(s);if(!gl.getShaderParameter(s,gl.COMPILE_STATUS))throw Error(gl.getShaderInfoLog(s));return s}
const v=shader(gl.VERTEX_SHADER,'attribute vec3 xyz;attribute vec3 rgb;varying vec3 color;uniform vec4 view;uniform vec2 offset;uniform float size;uniform float heightMode;void main(){vec3 p=xyz*.01;float a=view.x,b=view.y;float x=cos(a)*p.x-sin(a)*p.y;float y=sin(a)*p.x+cos(a)*p.y;float h=cos(b)*y+sin(b)*p.z;float d=-sin(b)*y+cos(b)*p.z;gl_Position=vec4((x*view.z+offset.x)*view.w,h*view.z+offset.y,-d*.012,1.);gl_PointSize=size;float t=clamp(.5+.22*p.z,0.,1.);vec3 palette=mix(vec3(.13,.49,.64),vec3(1.,.79,.34),t);color=mix(rgb,palette,heightMode);}');
const f=shader(gl.FRAGMENT_SHADER,'precision mediump float;varying vec3 color;void main(){gl_FragColor=vec4(color,1.);}');
const program=gl.createProgram();gl.attachShader(program,v);gl.attachShader(program,f);gl.linkProgram(program);if(!gl.getProgramParameter(program,gl.LINK_STATUS))throw Error('link');gl.useProgram(program);
function buffer(key,data,type,norm){const b=gl.createBuffer();gl.bindBuffer(gl.ARRAY_BUFFER,b);gl.bufferData(gl.ARRAY_BUFFER,data,gl.STATIC_DRAW);const i=gl.getAttribLocation(program,key);gl.enableVertexAttribArray(i);gl.vertexAttribPointer(i,3,type,norm,0,0)}
buffer('xyz',bytes('COORDS'),gl.SHORT,false);buffer('rgb',bytes('COLORS'),gl.UNSIGNED_BYTE,true);
let yaw,pitch,zoom,ox,oy,heightMode=0;function restore(){yaw=-.42;pitch=.85;zoom=.06;ox=oy=0;draw()}function draw(){const d=Math.min(devicePixelRatio,2);canvas.width=canvas.clientWidth*d;canvas.height=canvas.clientHeight*d;gl.viewport(0,0,canvas.width,canvas.height);gl.clearColor(.065,.1,.13,1);gl.clear(gl.COLOR_BUFFER_BIT|gl.DEPTH_BUFFER_BIT);gl.enable(gl.DEPTH_TEST);gl.uniform4f(gl.getUniformLocation(program,'view'),yaw,pitch,zoom,canvas.height/canvas.width);gl.uniform2f(gl.getUniformLocation(program,'offset'),ox,oy);gl.uniform1f(gl.getUniformLocation(program,'size'),Math.max(1.5,Math.min(7,48*zoom))*d);gl.uniform1f(gl.getUniformLocation(program,'heightMode'),heightMode);gl.drawArrays(gl.POINTS,0,__VERTEX_COUNT__)}
let last=null;canvas.onpointerdown=e=>{canvas.setPointerCapture(e.pointerId);last=[e.clientX,e.clientY]};canvas.onpointerup=canvas.onpointercancel=()=>last=null;canvas.onpointermove=e=>{if(!last)return;const dx=e.clientX-last[0],dy=e.clientY-last[1];last=[e.clientX,e.clientY];if(e.shiftKey){ox+=dx*2/canvas.clientHeight;oy-=dy*2/canvas.clientHeight}else{yaw+=dx*.008;pitch=Math.max(-1.52,Math.min(1.52,pitch+dy*.008))}draw()};canvas.onwheel=e=>{e.preventDefault();zoom=Math.max(.014,Math.min(3,zoom*Math.exp(-e.deltaY*.0015)));draw()};document.getElementById('reset').onclick=restore;document.getElementById('height').onclick=e=>{heightMode=1-heightMode;e.target.textContent=heightMode?'원본 색상':'높이로 보기';draw()};window.onresize=draw;restore();</script></html>"""
    dest.write_text(
        page.replace("__VERTEX_COUNT__", str(len(xyz)))
        .replace("COORDS", encode(coords))
        .replace("COLORS", encode(rgb))
    )


def figures(root, summary):
    folder = root / "results"
    fusion = folder / "fusion"
    vol = np.load(folder / "rgbd/preview.npz")
    cloud_view(vol, folder / "rgbd/viewer.html")
    x, y, z = vol["xyz"].T
    rgb = vol["rgb"] / 255
    fig, ax = plt.subplots(figsize=(11, 7), facecolor="#14202a")
    ax.set_facecolor("#14202a")
    # Same near-orthographic oblique projection used in the offline viewer.
    u = 0.91 * x + 0.415 * y
    v = (-0.415 * x + 0.91 * y) * 0.66 + z * 0.75
    order = np.argsort((-0.415 * x + 0.91 * y) * -0.75 + z * 0.66)
    ax.scatter(u[order], v[order], s=1.4, c=rgb[order], linewidths=0)
    ax.set_aspect("equal")
    ax.axis("off")
    fig.savefig(folder / "rgbd/overview.png", dpi=145, bbox_inches="tight", pad_inches=0.15)
    plt.close(fig)
    fused = np.loadtxt(fusion / "trajectory.csv", delimiter=",", skiprows=1)
    track = load_track(root / "input/lidar/training.csv")
    field = load_track(root / "input/lidar/field.csv")
    m = wall_eval.Map.read(fusion / "fused")
    fig, axes = plt.subplots(1, 2, figsize=(12, 6.2), constrained_layout=True)
    for ax, mode in zip(axes, ("rgbd", summary["primary_stereo"])):
        xy = np.loadtxt(folder / mode / "aligned-to-lidar.csv", delimiter=",", skiprows=1)
        ax.imshow(m.pixels, cmap="gray", extent=m.extent, vmin=0, vmax=255)
        ax.plot(track[:, 1], track[:, 2], color="#165e86", lw=1, alpha=0.8, label="LiDAR reference")
        # Do not draw an imaginary line across long tracking gaps.
        for j, part in enumerate(np.split(xy, np.flatnonzero(np.diff(xy[:, 0]) > 5) + 1)):
            ax.plot(
                part[:, 1],
                part[:, 2],
                color="#d0502a",
                lw=1.2,
                label="Visual graph (rigid XY alignment)" if not j else None,
            )
        ax.set_title(("RGB-D" if mode == "rgbd" else "IR stereo") + " camera-only SLAM")
        ax.set_xlabel("x (m)")
        ax.set_ylabel("y (m)")
        ax.set_aspect("equal")
        ax.legend(fontsize=7)
    fig.savefig(folder / "trajectories.png", dpi=140)
    plt.close(fig)
    metrics = json.loads((fusion / "metrics.json").read_text())
    used = json.loads((fusion / "used-edges.json").read_text())
    rec = np.load(root / "input/lidar/scans.npz")
    ts = np.linspace(max(field[0, 0], track[0, 0]), min(field[-1, 0], track[-1, 0]), 600)
    align = wall_eval.rigid(sample(track, ts)[:, :2], sample(field, ts)[:, :2])
    walls, rays, _ = diagnostic(root, used, track, fused, rec, field, align)
    fig, axes = plt.subplots(2, 3, figsize=(13, 6.4), constrained_layout=True)
    xy0, _ = wall_eval.project(field, rays)
    xy1 = wall_eval.project(track, rays)[0] @ align[0] + align[1]
    xy2 = wall_eval.project(fused, rays)[0] @ align[0] + align[1]
    examples = []
    for row, key in enumerate(("A", "B")):
        a, b, n, pairs = walls[key]
        tangent = (b - a) / np.linalg.norm(b - a)
        if not pairs:
            for ax in axes[row]:
                ax.text(0.5, 0.5, "No paired passes", ha="center")
            continue
        offsets = np.array(
            [abs(np.median((xy0[ia] - a) @ n) - np.median((xy0[ib] - a) @ n)) for _, _, _, ia, ib in pairs]
        )
        pick = int(np.argmin(abs(offsets - np.median(offsets))))
        _, _, _, ia, ib = pairs[pick]
        t1, t2 = np.median(rays[ia, 0]), np.median(rays[ib, 0])
        example = dict(
            wall=key, pass_s=[float(t1 - track[0, 0]), float(t2 - track[0, 0])], median_separation_m=[]
        )
        for ax, xy, title in zip(
            axes[row], [xy0, xy1, xy2], ["Field checkpoint", "LiDAR replay", "Camera + LiDAR"]
        ):
            for ids, col, label in ((ia, "#176486", "Pass 1"), (ib, "#d45932", "Pass 2")):
                ax.scatter((xy[ids] - a) @ tangent, (xy[ids] - a) @ n, s=6, c=col, alpha=0.65, label=label)
            gap = abs(np.median((xy[ia] - a) @ n) - np.median((xy[ib] - a) @ n))
            example["median_separation_m"].append(float(gap))
            ax.set_title(f"{title} · {gap*100:.1f} cm")
            ax.grid(alpha=0.15)
            ax.set_xlabel("Along wall (m)")
            ax.set_ylabel("Normal offset (m)")
        stacked = np.vstack([xy[ids] for xy in (xy0, xy1, xy2) for ids in (ia, ib)])
        xx, yy = (stacked - a) @ tangent, (stacked - a) @ n
        for ax in axes[row]:
            ax.set_xlim(xx.min() - 0.09, xx.max() + 0.09)
            ax.set_ylim(yy.min() - 0.10, yy.max() + 0.10)
        axes[row, 0].set_ylabel(f"Wall {key} · offset (m)")
        axes[row, 0].legend(fontsize=7)
        examples.append(example)
    fig.savefig(fusion / "wall-passes.png", dpi=145)
    plt.close(fig)
    table = wall_eval.Map.read(root / "input/lidar")
    fig, axes = plt.subplots(1, 2, figsize=(13, 6), constrained_layout=True)
    for ax, sub, title in zip(axes, ("lidar", "fused"), ("LiDAR only", "RGB-D revisits + LiDAR")):
        g = wall_eval.Map.read(fusion / sub)
        ax.imshow(g.pixels, cmap="gray", vmin=0, vmax=255, extent=g.extent)
        t = track if sub == "lidar" else fused
        ax.plot(t[:, 1], t[:, 2], lw=0.6, c="#147b82", alpha=0.72)
        if sub == "fused":
            for ed in used:
                points = sample(fused, np.array([ed["a"], ed["b"]]) * 1e-9)
                ax.plot(points[:, 0], points[:, 1], lw=0.65, c="#d97619", alpha=0.6)
        for key in ("A", "B"):
            a, b, _, _ = walls[key]
            ab = (np.stack((a, b)) - align[1]) @ align[0].T
            ax.plot(ab[:, 0], ab[:, 1], c="#b32162", lw=2)
            ax.text(ab[0, 0], ab[0, 1], key, color="#99144a", weight="bold")
        ax.set_title(title)
        ax.set_aspect("equal")
        ax.set_xlabel("x (m)")
        ax.set_ylabel("y (m)")
    fig.savefig(fusion / "comparison.png", dpi=135)
    plt.close(fig)
    # Representative high-support pair near either surveyed wall; report the distance explicitly.
    measures = []
    for ed in used:
        pos = sample(field, np.array([ed["a"], ed["b"]]) * 1e-9)[:, :2]
        wall_distance = []
        for key in ("A", "B"):
            a, b, _, _ = walls[key]
            v = b - a
            proj = a + np.clip(((pos - a) @ v) / (v @ v), 0, 1)[:, None] * v
            wall_distance.append((float(np.linalg.norm(pos - proj, axis=1).max()), key))
        dist, key = min(wall_distance)
        measures.append((dist, key, ed))
    verified = json.loads((fusion / "visit-constraints.json").read_text())["edges"]
    direct = identify(root, verified, walls, field, track, fused, align)
    linkage = {
        k: [{n: v for n, v in row.items() if n != "indices"} for row in vals] for k, vals in direct.items()
    }
    (fusion / "wall-landmarks.json").write_text(json.dumps(linkage, indent=2))
    direct_examples = [v for wall in ("A", "B") for v in direct.get(wall, [])]
    landmark = None
    if direct_examples:
        landmark = max(direct_examples, key=lambda row: row["matches_on_wall"])
        ed = next(e for e in verified if (e["a"], e["b"]) == (landmark["a"], landmark["b"]))
        a, b, _, _ = walls[landmark["wall"]]
        pos = sample(field, np.array([ed["a"], ed["b"]]) * 1e-9)[:, :2]
        v = b - a
        dist = float(
            np.linalg.norm(pos - a - np.clip(((pos - a) @ v) / (v @ v), 0, 1)[:, None] * v, axis=1).max()
        )
        best = dist, landmark["wall"], ed
    else:
        best = min(measures, key=lambda x: x[0] + 3 / (1 + x[2]["inliers"] / 18))
    dist, key, ed = best
    paths = [root / "input/rgb" / f"{ed[k]}.jpg" for k in ("a", "b")]
    left, right = [np.asarray(Image.open(p).convert("RGB")) for p in paths]
    fig, ax = plt.subplots(figsize=(11, 4.4))
    ax.imshow(np.concatenate([left, right], axis=1))
    lines = np.array(ed["matches"])
    if landmark:
        lines = lines[landmark["indices"]]
    lines = lines[:: max(1, len(lines) // 22)]
    for u, v, x, y in lines:
        ax.plot([u, x + left.shape[1]], [v, y], lw=0.6, color="#ffd636", alpha=0.7)
    ax.axis("off")
    ax.set_title(
        f"Repeated view near wall {key} · {ed['inliers']} RGB-D inliers · LiDAR overlap {ed['scan_overlap']:.0%}"
    )
    fig.savefig(fusion / "revisit.jpg", dpi=145, bbox_inches="tight", pad_inches=0.08)
    plt.close(fig)
    (fusion / "illustrated-passes.json").write_text(
        json.dumps(
            dict(
                examples=examples,
                match={k: v for k, v in ed.items() if k != "matches"},
                wall=key,
                distance_m=dist,
                landmarks=landmark,
            ),
            indent=2,
        )
    )


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("root", type=Path)
    r = p.parse_args().root
    figures(r, json.loads((r / "results/visual-metrics.json").read_text()))
