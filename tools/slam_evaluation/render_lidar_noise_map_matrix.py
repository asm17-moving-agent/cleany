#!/usr/bin/env python3
"""Render the LiDAR-noise SLAM maps on one world-aligned canvas."""

from __future__ import annotations

import argparse
import json
import math
from dataclasses import dataclass
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont
import yaml


NOISE_PROFILES = (
    ('measured', 'Measured noise  sigma = 2.5 mm', '#2563eb'),
    ('stress', 'Stress noise  sigma = 10 mm', '#ea580c'),
)
ALGORITHMS = (
    ('slam_toolbox', 'slam_toolbox'),
    ('cartographer', 'Cartographer'),
)
HEIGHTS = ((16.5, '16p5'), (30.0, '30'), (45.0, '45'))
FONT_PATHS = {
    'regular': (
        Path('/usr/share/fonts/google-noto/NotoSans-Regular.ttf'),
        Path('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf'),
    ),
    'bold': (
        Path('/usr/share/fonts/google-noto/NotoSans-Bold.ttf'),
        Path('/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf'),
    ),
}


@dataclass(frozen=True)
class MapImage:
    """One occupancy map with its planar world placement."""

    pixels: Image.Image
    resolution: float
    origin_x: float
    origin_y: float

    @property
    def max_x(self) -> float:
        """Return the map's upper X bound."""
        return self.origin_x + self.pixels.width * self.resolution

    @property
    def max_y(self) -> float:
        """Return the map's upper Y bound."""
        return self.origin_y + self.pixels.height * self.resolution


def load_map(run: Path) -> MapImage:
    """Load an occupancy image and placement metadata from one run."""
    with (run / 'map_final.yaml').open(encoding='utf-8') as stream:
        metadata = yaml.safe_load(stream)
    pixels = Image.open(run / 'map_final.pgm').convert('L')
    origin_x, origin_y = (float(value) for value in metadata['origin'][:2])
    return MapImage(
        pixels=pixels,
        resolution=float(metadata['resolution']),
        origin_x=origin_x,
        origin_y=origin_y,
    )


def load_font(style: str, size: int) -> ImageFont.FreeTypeFont:
    """Load a portable Noto or DejaVu font."""
    for path in FONT_PATHS[style]:
        if path.is_file():
            return ImageFont.truetype(str(path), size)
    raise FileNotFoundError(f'no supported {style} font found')


def text_width(
    draw: ImageDraw.ImageDraw,
    value: str,
    font: ImageFont.FreeTypeFont,
) -> int:
    """Measure rendered text width."""
    bounds = draw.textbbox((0, 0), value, font=font)
    return bounds[2] - bounds[0]


def centered_text(
    draw: ImageDraw.ImageDraw,
    center_x: int,
    y: int,
    value: str,
    font: ImageFont.FreeTypeFont,
    fill: str,
) -> None:
    """Draw text centered on the requested X coordinate."""
    draw.text(
        (center_x - text_width(draw, value, font) / 2, y),
        value,
        font=font,
        fill=fill,
    )


def main() -> None:
    """Render the complete two-noise, two-algorithm, three-height matrix."""
    parser = argparse.ArgumentParser()
    parser.add_argument(
        '--results-root',
        type=Path,
        default=Path('ros2_ws/slam_results'),
    )
    args = parser.parse_args()
    output = args.results_root / 'algorithm_comparison' / 'lidar_noise'
    metrics = json.loads((output / 'metrics.json').read_text(encoding='utf-8'))
    metric_lookup = {
        (row['noise_profile'], row['algorithm'], float(row['height_cm'])): row
        for row in metrics
    }

    maps: dict[tuple[str, str, float], MapImage] = {}
    for noise, _, _ in NOISE_PROFILES:
        for algorithm, _ in ALGORITHMS:
            for height, token in HEIGHTS:
                run = args.results_root.joinpath(
                    'algorithm_compare_runs',
                    noise,
                    algorithm,
                    f'{token}cm',
                )
                maps[noise, algorithm, height] = load_map(run)

    resolutions = {round(item.resolution, 9) for item in maps.values()}
    if len(resolutions) != 1:
        raise ValueError(
            f'map resolutions do not match: {sorted(resolutions)}'
        )
    resolution = next(iter(resolutions))
    min_x = min(item.origin_x for item in maps.values())
    min_y = min(item.origin_y for item in maps.values())
    max_x = max(item.max_x for item in maps.values())
    max_y = max(item.max_y for item in maps.values())
    world_width = math.ceil((max_x - min_x) / resolution)
    world_height = math.ceil((max_y - min_y) / resolution)
    nearest = getattr(Image, 'Resampling', Image).NEAREST
    panel_scale = 2
    panel_width = world_width * panel_scale
    panel_height = world_height * panel_scale

    left_margin = 205
    right_margin = 32
    column_gap = 18
    group_gap = 34
    top_margin = 174
    caption_height = 48
    row_gap = 28
    footer_height = 64
    total_width = sum(
        (
            left_margin,
            6 * panel_width,
            4 * column_gap,
            group_gap,
            right_margin,
        )
    )
    total_height = sum(
        (
            top_margin,
            2 * (panel_height + caption_height),
            row_gap,
            footer_height,
        )
    )
    image = Image.new('RGB', (total_width, total_height), '#ffffff')
    draw = ImageDraw.Draw(image)
    title_font = load_font('bold', 42)
    group_font = load_font('bold', 30)
    label_font = load_font('regular', 25)
    row_font = load_font('bold', 27)
    metric_font = load_font('regular', 21)
    footer_font = load_font('regular', 20)
    foreground = '#172033'
    muted = '#596579'
    frame = '#9aa5b4'

    centered_text(
        draw,
        total_width // 2,
        22,
        'StudyCafe SLAM map comparison - 2 x 2 x 3',
        title_font,
        foreground,
    )

    column_x: list[int] = []
    for column in range(6):
        extra = group_gap if column >= 3 else 0
        column_x.append(
            left_margin + column * (panel_width + column_gap) + extra
        )

    for noise_index, (_, group_label, group_color) in enumerate(
        NOISE_PROFILES
    ):
        first = column_x[noise_index * 3]
        last = column_x[noise_index * 3 + 2] + panel_width
        center = (first + last) // 2
        centered_text(draw, center, 78, group_label, group_font, group_color)
        draw.line((first, 119, last, 119), fill=group_color, width=5)
        for height_index, (height, _) in enumerate(HEIGHTS):
            x = column_x[noise_index * 3 + height_index]
            centered_text(
                draw,
                x + panel_width // 2,
                130,
                f'LiDAR {height:g} cm',
                label_font,
                foreground,
            )

    for row_index, (algorithm, algorithm_label) in enumerate(ALGORITHMS):
        map_y = top_margin + row_index * (
            panel_height + caption_height + row_gap
        )
        centered_text(
            draw,
            left_margin // 2,
            map_y + panel_height // 2 - 16,
            algorithm_label,
            row_font,
            foreground,
        )
        for noise_index, (noise, _, _) in enumerate(NOISE_PROFILES):
            for height_index, (height, _) in enumerate(HEIGHTS):
                x = column_x[noise_index * 3 + height_index]
                item = maps[noise, algorithm, height]
                panel = Image.new('L', (world_width, world_height), 205)
                offset_x = round((item.origin_x - min_x) / resolution)
                offset_y = round((max_y - item.max_y) / resolution)
                panel.paste(item.pixels, (offset_x, offset_y))
                panel = panel.resize((panel_width, panel_height), nearest)
                image.paste(panel.convert('RGB'), (x, map_y))
                draw.rectangle(
                    (x, map_y, x + panel_width - 1, map_y + panel_height - 1),
                    outline=frame,
                    width=2,
                )
                metric = metric_lookup[noise, algorithm, height]
                ate_cm = 100 * metric['ate_translation_rmse_m']
                rpe_cm = 100 * metric['rpe_1s_translation_rmse_m']
                caption = (
                    f'ATE {ate_cm:.2f} cm  |  RPE 1s {rpe_cm:.2f} cm'
                )
                centered_text(
                    draw,
                    x + panel_width // 2,
                    map_y + panel_height + 9,
                    caption,
                    metric_font,
                    muted,
                )

    legend_y = total_height - 40
    centered_text(
        draw,
        total_width // 2,
        legend_y,
        (
            'Black: occupied   |   White: free   |   Gray: unknown   |   '
            f'World-aligned canvas, {resolution:g} m/pixel'
        ),
        footer_font,
        muted,
    )
    destination = output / 'map_matrix_2x2x3.png'
    image.save(destination)
    print(destination)


if __name__ == '__main__':
    main()
