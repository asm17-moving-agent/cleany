#!/usr/bin/env python3
"""Original seamless weave approximations, not copied product photographs."""

from pathlib import Path
import math
import random
from PIL import Image

root = (
    Path(__file__).resolve().parents[2] / 'ros2_ws/src/cleany_gazebo_sim/materials/roly'
)
rng = random.Random(210)
for name in ('seat', 'back'):
    image = Image.new('RGB', (512, 512))
    pixels = []
    for y in range(512):
        for x in range(512):
            if name == 'seat':
                warp = math.sin(x * math.pi / 3) * math.cos(y * math.pi / 3)
                value = int(211 + 23 * warp + rng.uniform(-12, 12))
            else:
                vertical = abs(math.sin(x * math.pi / 8))
                horizontal = abs(math.sin(y * math.pi / 4))
                value = int(
                    125 + 90 * max(vertical**6, horizontal**4) + rng.uniform(-7, 7)
                )
            pixels.append((max(0, min(255, value)),) * 3)
    image.putdata(pixels)
    image.save(root / f'{name}.png')
