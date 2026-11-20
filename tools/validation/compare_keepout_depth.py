"""저장된 정지 Depth 관측과 수동 Keepout 마스크의 공간 중첩을 비교합니다.

원본 캡처의 마지막 점군과 최신 map_to_base 변환을 사용합니다. 촬영 시각에
정확히 동기화한 TF나 의미 분할 정답이 아니므로 검출 정확도를 산정하지 않습니다.
"""

from __future__ import annotations

import argparse
from datetime import datetime, timezone, timedelta
import hashlib
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.font_manager import FontProperties
from matplotlib.patches import Patch, Rectangle
from matplotlib.lines import Line2D
import numpy as np
from PIL import Image
from scipy.ndimage import label, distance_transform_edt
from scipy.spatial.transform import Rotation
import yaml


def transform(points: np.ndarray, pose: dict) -> np.ndarray:
    rotation = Rotation.from_quat([pose["rotation"][k] for k in ("x", "y", "z", "w")])
    return rotation.apply(points) + [pose["translation"][k] for k in ("x", "y", "z")]


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run", type=Path, required=True)
    parser.add_argument("--mask-yaml", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--font", type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    manifest = json.loads((args.run / "manifest.json").read_text())
    map_yaml = Path(manifest["map"])
    original = yaml.safe_load(map_yaml.read_text())
    config = yaml.safe_load(args.mask_yaml.read_text())
    assert config["origin"] == original["origin"] and config["resolution"] == original["resolution"]
    assert config.get("negate", 0) == 0 and config["mode"] == "trinary"
    assert config["origin"][2] == 0, "회전 원점 미지원"
    image_path = map_yaml.parent / original["image"]
    mask_path = args.mask_yaml.parent / config["image"]
    base = np.array(Image.open(image_path).convert("L"))
    mask = np.array(Image.open(mask_path).convert("L"))
    assert base.shape == mask.shape
    blocked = np.flipud((255.0 - mask) / 255 > config["occupied_thresh"])
    h, w = mask.shape
    resolution = config["resolution"]
    ox, oy, _ = config["origin"]
    extent = (ox, ox + w * resolution, oy, oy + h * resolution)
    labels, count = label(blocked, structure=np.ones((3, 3)))
    # 손으로 채운 큰 연결 영역만 책상 후보로 구분하고 가느다란 경계선은 제외합니다.
    tables = [i for i in range(1, count + 1) if np.sum(labels == i) >= 1000]
    assert len(tables) == 9, "수동 마스크의 책상 영역 9개를 확인해야 합니다."
    distances = distance_transform_edt(~blocked) * resolution
    xx = ox + (np.arange(w) + .5) * resolution
    yy = oy + (np.arange(h) + .5) * resolution
    samples, metrics = {}, {}
    sources = [map_yaml, image_path, args.mask_yaml, mask_path, args.run / "manifest.json"]
    for name in ("chair-present", "chair-removed", "final-state", "pink-diagnostic"):
        metadata_path, points_path = args.run / f"{name}.json", args.run / f"{name}.npz"
        meta = json.loads(metadata_path.read_text())
        raw = np.load(points_path)["marking"]
        stamp = meta["clouds"]["marking"][-1]
        assert stamp["frame"] == "base_link" and stamp["count"] == len(raw)
        points = transform(raw, meta["map_to_base"])
        columns = np.floor((points[:, 0] - ox) / resolution).astype(int)
        rows = np.floor((points[:, 1] - oy) / resolution).astype(int)
        valid = (rows >= 0) & (rows < h) & (columns >= 0) & (columns < w)
        cells = np.unique(np.column_stack((rows[valid], columns[valid])), axis=0)
        rr, cc = cells.T
        inside = blocked[rr, cc]
        point_inside = blocked[rows[valid], columns[valid]]
        outside_distance = distances[rr[~inside], cc[~inside]]
        per_table = []
        for ident in tables:
            n = int(np.sum(labels[rr, cc] == ident))
            if n:
                tr, tc = np.where(labels == ident)
                per_table.append(dict(component=ident, mask_cells=len(tr), observed_cells=n,
                                      center_map_xy=[float(xx[tc].mean()), float(yy[tr].mean())]))
        metrics[name] = dict(
            time_kst=datetime.fromtimestamp(stamp["stamp"], timezone(timedelta(hours=9))).isoformat(),
            points=len(raw), points_inside_mask=int(point_inside.sum()), points_outside_map=int((~valid).sum()),
            observed_unique_cells=len(cells), cells_inside_mask=int(inside.sum()),
            cells_outside_mask=int((~inside).sum()), intersected_tables=per_table,
            outside_cell_center_distance_median=float(np.median(outside_distance)),
            outside_cell_center_distance_p95=float(np.percentile(outside_distance, 95)),
            outside_cells_within_15cm=int(np.sum(outside_distance <= .15 + 1e-8)),
            robot_map_pose=meta["map_to_base"],
        )
        samples[name] = (points, meta)
        sources.extend([metadata_path, points_path])
    font = FontProperties(fname=str(args.font))
    plt.rcParams.update({"axes.unicode_minus": False})
    fig, axes = plt.subplots(1, 3, figsize=(18, 8), dpi=150)
    fig.patch.set_facecolor("#f3f7f5")
    overlay = np.zeros((h, w, 4)); overlay[blocked] = (.90, .12, .12, .32)
    limits_points = np.vstack(
        [samples[n][0] for n in ("chair-present", "chair-removed")]
        + [np.array([[samples[n][1]["map_to_base"]["translation"][k]
                      for k in ("x", "y", "z")]]) for n in ("chair-present", "chair-removed")]
    )
    low, high = limits_points[:, :2].min(axis=0)-.35, limits_points[:, :2].max(axis=0)+.35
    for ax in axes:
        ax.imshow(np.flipud(base), origin="lower", extent=extent, cmap="gray", vmin=0, vmax=255, interpolation="nearest")
        ax.imshow(overlay, origin="lower", extent=extent, interpolation="nearest")
        ax.set_aspect("equal"); ax.set_xlabel("map X (m)"); ax.set_ylabel("map Y (m)")
    axes[0].set_title("전체 지도와 비교 위치", fontproperties=font, fontsize=14)
    axes[0].add_patch(Rectangle(low, *(high-low), fill=False, ec="#008b8b", lw=2))
    for ax, name, title in zip(axes[1:], ("chair-present", "chair-removed"), ("의자 배치 상태", "의자 제거 상태")):
        p, meta = samples[name]
        sc = ax.scatter(p[:, 0], p[:, 1], c=p[:, 2], cmap="viridis", s=3, vmin=.19, vmax=1.4, alpha=.75)
        ax.contour(xx, yy, blocked.astype(float), levels=[.5], colors=["#c81717"], linewidths=.9)
        pose = meta["map_to_base"]["translation"]
        ax.scatter(pose["x"], pose["y"], c="#f08c00", marker="^", s=65)
        ax.set(xlim=(low[0],high[0]), ylim=(low[1],high[1]))
        m = metrics[name]
        ax.set_title(title+" · "+m["time_kst"][11:19], fontproperties=font, fontsize=14)
        ax.text(.02,.02,f"관측 격자 {m['observed_unique_cells']}개 중\n마스크 내부 {m['cells_inside_mask']} / 외부 {m['cells_outside_mask']}",
                transform=ax.transAxes, fontproperties=font, fontsize=11,
                bbox=dict(facecolor="white", alpha=.9, edgecolor="none"))
    fig.suptitle("수동 Keepout 영역과 기록된 Depth 관측 비교", fontproperties=font, fontsize=22, y=.97)
    fig.text(.5,.905,"2026-11-08 정지 캡처 / 기록된 map_to_base 사용 / 추가 위치 맞춤 없음", ha="center", fontproperties=font, fontsize=12)
    fig.subplots_adjust(left=.045,right=.93,top=.84,bottom=.23,wspace=.25)
    cbax=fig.add_axes([.945,.31,.012,.40]); cb=fig.colorbar(sc,cax=cbax)
    cb.set_label("Depth 높이 (m)", fontproperties=font)
    fig.legend(handles=[Patch(fc="#e62222",alpha=.45,label="수동 Keepout"),
                        Line2D([],[],marker="o",ls="",color="#238b8d",label="Depth 장애물 관측점 (책상 의미 분할 아님)"),
                        Line2D([],[],marker="^",ls="",color="#f08c00",label="기록된 로봇 위치")],
               prop=font,loc="lower center",bbox_to_anchor=(.5,.13),ncol=3,frameon=False)
    fig.text(.5,.075,"마스크 내부의 빈 곳: 미관측 또는 가림 가능. 마스크 외부의 점: 의자·주변 물체·정합 오차 가능.\n"
             "정지 캡처 최신 TF 기준의 근사 중첩이며, 책상 전체 검출률·충돌 회피 성능 비교가 아닙니다.",
             ha="center",fontproperties=font,fontsize=11,color="#475569")
    fig.savefig(args.output/"keepout-depth-comparison.png"); plt.close(fig)
    # 같은 기록을 처음 보는 사람이 영역과 관측점의 차이를 바로 읽도록 단순화합니다.
    fig, axes = plt.subplots(1, 2, figsize=(13, 8), dpi=150)
    fig.patch.set_facecolor("#f3f7f5")
    p, meta = samples["chair-removed"]
    for ax in axes:
        ax.imshow(np.flipud(base), origin="lower", extent=extent, cmap="gray",
                  vmin=0, vmax=255, interpolation="nearest", alpha=.23)
        ax.set(xlim=(-3.1, 1.2), ylim=(-12.7, -8.1), aspect="equal")
        ax.set_axis_off()
    axes[0].imshow(overlay, origin="lower", extent=extent, interpolation="nearest")
    for ax in axes:
        ax.contour(xx, yy, blocked.astype(float), levels=[.5], colors=["#df3030"], linewidths=2)
    axes[1].scatter(p[:, 0], p[:, 1], color="#087f8c", s=5, alpha=.55)
    axes[0].set_title("① 네가 칠한 Keepout", fontproperties=font, fontsize=21, pad=18)
    axes[1].set_title("② 같은 곳의 Depth 기록", fontproperties=font, fontsize=21, pad=18)
    def note(ax, text, target, location, color):
        ax.annotate(text, xy=target, xytext=location, fontproperties=font,
                    fontsize=14, ha="center", va="center", color=color,
                    bbox=dict(boxstyle="round,pad=.5", fc="white", ec=color, alpha=.97),
                    arrowprops=dict(arrowstyle="->", color=color, lw=2))
    note(axes[0], "빨간 면적 전체를\n진입 금지로 지정", (-1.9,-10.5), (-.1,-11.7), "#b92525")
    note(axes[1], "마스크 밖에도\n관측점이 있음", (.15,-8.9), (.15,-10.1), "#086e79")
    note(axes[1], "마스크 안이지만\n이 기록에는 관측점 없음", (-2.1,-11.5), (-.9,-12.25), "#475569")
    fig.suptitle("책상 한 곳을 확대해서 비교", fontproperties=font, fontsize=25, y=.97)
    fig.text(.5,.9,"두 그림은 같은 위치와 축척입니다.  |  맨 윗줄 가운데 책상",ha="center",fontproperties=font,fontsize=14)
    fig.subplots_adjust(top=.82,bottom=.17,left=.04,right=.96,wspace=.12)
    fig.legend(handles=[Patch(fc="#e62222", alpha=.32, label="수동 지정 영역 / 빨간 윤곽"),
                        Line2D([],[],marker="o",ls="",color="#087f8c",label="Depth가 실제 관측한 장애물 점")],
               prop=font,loc="lower center",bbox_to_anchor=(.5,.09),ncol=2,frameon=False,fontsize=14)
    fig.text(.5,.035,"2026-11-08 11:14:51 정지 기록 · 책상만 분류한 점군 아님 · 기록된 위치 추정 기준 · Nav2 적용 전",
             ha="center",fontproperties=font,fontsize=11,color="#475569")
    fig.savefig(args.output/"keepout-depth-simple.png"); plt.close(fig)
    result=dict(metrics=metrics, sources={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in sources},
                method="base_link marking points transformed by snapshot latest map_to_base; 5cm grid, no registration optimization",
                limitations=["Latest TF is not exactly synchronized to exposure", "No semantic desk labels or visibility ground truth",
                             "Stationary views cover only part of the scene; occupancy hits do not imply full tabletop coverage",
                             "Point density and occlusion preclude interpreting overlap as detector accuracy"],
                map_cells=[w,h],resolution=resolution,table_components=len(tables))
    (args.output/"comparison.json").write_text(json.dumps(result,ensure_ascii=False,indent=2)+"\n")
    print(json.dumps(metrics,ensure_ascii=False,indent=2))


if __name__ == "__main__":
    main()
