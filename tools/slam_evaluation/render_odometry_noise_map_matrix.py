#!/usr/bin/env python3
"""Render a two-algorithm by four-odometry-level map matrix."""

from __future__ import annotations

import argparse
import json
import math
from dataclasses import dataclass
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont
import yaml


LEVELS = (
    ("level0", "Level 0\nNo added noise"),
    ("level1", "Level 1\n1/3 stress"),
    ("level2", "Level 2\n2/3 stress"),
    ("level3", "Level 3\nStress"),
)
ALGORITHMS = (
    ("slam_toolbox", "slam_toolbox"),
    ("cartographer", "Cartographer"),
)
FONT_PATHS = {
    "regular": (
        Path("/usr/share/fonts/google-noto/NotoSans-Regular.ttf"),
        Path("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"),
    ),
    "bold": (
        Path("/usr/share/fonts/google-noto/NotoSans-Bold.ttf"),
        Path("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf"),
    ),
}


@dataclass(frozen=True)
class MapImage:
    """One occupancy map and its placement in the odom-aligned plane."""

    pixels: Image.Image
    resolution: float
    origin_x: float
    origin_y: float

    @property
    def max_x(self) -> float:
        return self.origin_x + self.pixels.width * self.resolution

    @property
    def max_y(self) -> float:
        return self.origin_y + self.pixels.height * self.resolution


def load_map(run: Path) -> MapImage:
    """Load a ROS occupancy map and its YAML placement metadata."""
    metadata = yaml.safe_load(
        (run / "map_final.yaml").read_text(encoding="utf-8")
    )
    pixels = Image.open(run / "map_final.pgm").convert("L")
    return MapImage(
        pixels=pixels,
        resolution=float(metadata["resolution"]),
        origin_x=float(metadata["origin"][0]),
        origin_y=float(metadata["origin"][1]),
    )


def load_font(style: str, size: int) -> ImageFont.FreeTypeFont:
    """Load a portable Noto or DejaVu font."""
    for path in FONT_PATHS[style]:
        if path.is_file():
            return ImageFont.truetype(str(path), size)
    raise FileNotFoundError(f"no supported {style} font found")


def centered(
    draw: ImageDraw.ImageDraw,
    center_x: int,
    y: int,
    text: str,
    font: ImageFont.FreeTypeFont,
    fill: str,
) -> None:
    """Draw possibly multiline text centered on an X coordinate."""
    bounds = draw.multiline_textbbox((0, 0), text, font=font, align="center")
    width = bounds[2] - bounds[0]
    draw.multiline_text(
        (center_x - width / 2, y),
        text,
        font=font,
        fill=fill,
        align="center",
        spacing=2,
    )


def main() -> None:
    """Render the complete two-by-four comparison."""
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--results-root",
        type=Path,
        default=Path("ros2_ws/slam_results"),
    )
    parser.add_argument(
        "--experiment-name", default="odometry_noise"
    )
    args = parser.parse_args()
    experiment = args.results_root / args.experiment_name
    output = experiment / "comparison"
    metrics = json.loads(
        (output / "metrics.json").read_text(encoding="utf-8")
    )
    metric_lookup = {
        (row["algorithm"], row["odometry_level"]): row for row in metrics
    }
    maps = {
        (algorithm, level): load_map(
            experiment / "runs" / algorithm / level
        )
        for algorithm, _ in ALGORITHMS
        for level, _ in LEVELS
    }
    resolutions = {round(item.resolution, 6) for item in maps.values()}
    if len(resolutions) != 1:
        raise ValueError(f"map resolutions differ: {sorted(resolutions)}")
    resolution = next(iter(resolutions))
    min_x = min(item.origin_x for item in maps.values())
    min_y = min(item.origin_y for item in maps.values())
    max_x = max(item.max_x for item in maps.values())
    max_y = max(item.max_y for item in maps.values())
    world_width = math.ceil((max_x - min_x) / resolution)
    world_height = math.ceil((max_y - min_y) / resolution)
    panel_scale = 2
    panel_width = world_width * panel_scale
    panel_height = world_height * panel_scale
    left = 190
    top = 185
    gap_x = 18
    gap_y = 70
    caption = 42
    total_width = left + 4 * panel_width + 3 * gap_x + 30
    total_height = top + 2 * (panel_height + caption) + gap_y + 55
    canvas = Image.new("RGB", (total_width, total_height), "white")
    draw = ImageDraw.Draw(canvas)
    title_font = load_font("bold", 38)
    header_font = load_font("bold", 24)
    row_font = load_font("bold", 25)
    metric_font = load_font("regular", 19)
    footer_font = load_font("regular", 18)
    centered(
        draw,
        total_width // 2,
        18,
        "StudyCafe 30 cm SLAM - odometry noise levels",
        title_font,
        "#172033",
    )
    for column, (_, label) in enumerate(LEVELS):
        x = left + column * (panel_width + gap_x)
        input_metric = metric_lookup["slam_toolbox", LEVELS[column][0]]
        label = (
            f"{label}\nInput ATE "
            f"{100 * input_metric['input_odom_ate_translation_rmse_m']:.2f} cm"
        )
        centered(
            draw,
            x + panel_width // 2,
            75,
            label,
            header_font,
            "#172033",
        )

    nearest = getattr(Image, "Resampling", Image).NEAREST
    for row, (algorithm, label) in enumerate(ALGORITHMS):
        y = top + row * (panel_height + caption + gap_y)
        centered(
            draw,
            left // 2,
            y + panel_height // 2 - 14,
            label,
            row_font,
            "#172033",
        )
        for column, (level, _) in enumerate(LEVELS):
            x = left + column * (panel_width + gap_x)
            item = maps[algorithm, level]
            panel = Image.new("L", (world_width, world_height), 205)
            offset_x = round((item.origin_x - min_x) / resolution)
            offset_y = round((max_y - item.max_y) / resolution)
            panel.paste(item.pixels, (offset_x, offset_y))
            panel = panel.resize((panel_width, panel_height), nearest)
            canvas.paste(panel.convert("RGB"), (x, y))
            draw.rectangle(
                (x, y, x + panel_width - 1, y + panel_height - 1),
                outline="#9aa5b4",
                width=2,
            )
            metric = metric_lookup[algorithm, level]
            text = (
                f"ATE {100 * metric['ate_translation_rmse_m']:.2f} cm  |  "
                f"RPE {100 * metric['rpe_1s_translation_rmse_m']:.2f} cm"
            )
            centered(
                draw,
                x + panel_width // 2,
                y + panel_height + 8,
                text,
                metric_font,
                "#596579",
            )
    centered(
        draw,
        total_width // 2,
        total_height - 35,
        (
            "Same 30 cm measured-LiDAR bag; synthetic odometry only. "
            "Black occupied, white free, gray unknown."
        ),
        footer_font,
        "#596579",
    )
    destination = output / "map_matrix_2x4.png"
    canvas.save(destination)
    print(destination)


if __name__ == "__main__":
    main()
