#!/usr/bin/env python3
"""Create a deterministic seamless grey felt approximation (not a measured scan)."""

from pathlib import Path
import argparse
import random
from PIL import Image, ImageDraw


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True, help="Output felt PNG path")
    args = parser.parse_args()
    rng = random.Random(18)
    size = 512
    image = Image.new('RGB', (size, size))
    image.putdata(
        [
            (v, v, v)
            for v in (max(75, min(165, int(rng.gauss(120, 9)))) for _ in range(size * size))
        ]
    )
    draw = ImageDraw.Draw(image)
    for _ in range(22000):
        x, y = rng.randrange(size), rng.randrange(size)
        dx, dy = rng.randrange(-3, 4), rng.randrange(-3, 4)
        value = rng.randrange(100, 143)
        for ox in (-size, 0, size):
            for oy in (-size, 0, size):
                draw.line((x + ox, y + oy, x + dx + ox, y + dy + oy), fill=(value,) * 3)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    image.save(args.output)
    print(args.output)


if __name__ == "__main__":
    main()
