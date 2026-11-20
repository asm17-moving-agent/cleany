"""원본 지도와 Keepout 마스크를 같은 픽셀 좌표로 겹쳐 검토 이미지를 만듭니다."""

from argparse import ArgumentParser
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.font_manager import FontProperties
from matplotlib.patches import Patch
import numpy as np
from PIL import Image
import yaml


def main() -> None:
    parser = ArgumentParser(description=__doc__)
    parser.add_argument("--map-yaml", type=Path, required=True)
    parser.add_argument("--mask-yaml", type=Path, required=True)
    parser.add_argument("--font", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    original = yaml.safe_load(args.map_yaml.read_text())
    mask_config = yaml.safe_load(args.mask_yaml.read_text())
    for key in ("resolution", "origin"):
        if original[key] != mask_config[key]:
            raise ValueError(f"지도와 마스크의 {key} 불일치")
    if mask_config.get("negate", 0) != 0 or mask_config.get("mode") != "trinary":
        raise ValueError("negate=0, mode=trinary 마스크만 지원합니다.")
    base = np.array(Image.open(args.map_yaml.parent / original["image"]).convert("L"))
    mask = np.array(Image.open(args.mask_yaml.parent / mask_config["image"]).convert("L"))
    if base.shape != mask.shape:
        raise ValueError("지도와 마스크 크기 불일치")
    blocked = (255.0 - mask) / 255.0 > mask_config["occupied_thresh"]
    overlay = np.zeros((*mask.shape, 4))
    overlay[blocked] = (0.9, 0.15, 0.15, 0.65)
    font = FontProperties(fname=str(args.font))
    fig, ax = plt.subplots(figsize=(8, 11), dpi=180)
    fig.patch.set_facecolor("#f5f7f8")
    ax.imshow(base, cmap="gray", vmin=0, vmax=255, interpolation="nearest")
    ax.imshow(overlay, interpolation="nearest")
    ax.set_axis_off()
    fig.suptitle("책상 상판 및 지정 경계 Keepout 마스크", fontproperties=font, fontsize=18, y=0.97)
    ax.set_title("수동 지정 영역 — Nav2 적용 전", fontproperties=font, fontsize=13, pad=16)
    ax.legend(handles=[Patch(facecolor="#e62626", alpha=0.65, label="빨강: 수동 지정 진입 금지 영역")],
              prop=font, loc="lower center", bbox_to_anchor=(0.5, -0.05), frameon=False)
    fig.text(0.5, 0.035,
             f"원본 지도: 2026-11-08 수동 편집본 | {mask.shape[1]}×{mask.shape[0]} px | 약 {original['resolution']:.2f} m/px\n"
             "책상 9개 및 지정 경계 표시 · 실제 주행 및 충돌 회피 효과 미검증",
             ha="center", fontproperties=font, fontsize=10, color="#475569")
    fig.subplots_adjust(top=0.9, bottom=0.12, left=0.04, right=0.96)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.output, facecolor=fig.get_facecolor())
    plt.close(fig)
    print(f"{args.output}: {blocked.sum()}개 금지 픽셀, 원본 좌표 유지")


if __name__ == "__main__":
    main()
