#!/usr/bin/env python3
"""Extract viewport-only clips from the project's recorded demo videos."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import subprocess


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--mujoco', type=Path, required=True)
    parser.add_argument('--rviz', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    entries = []
    for name, source, start, duration, crop in [
        ('mujoco-work', args.mujoco, 6, 18, '1240:894:24:144'),
        ('rviz-motion', args.rviz, 4, 10, '1872:972:24:72'),
    ]:
        source = source.resolve(strict=True)
        mp4 = args.output / f'{name}.mp4'
        filters = f'crop={crop},scale=720:-2:flags=lanczos,setsar=1'
        command = ['ffmpeg', '-hide_banner', '-loglevel', 'error', '-y',
                   '-ss', str(start), '-i', str(source), '-t', str(duration),
                   '-vf', filters, '-an', '-c:v', 'libopenh264', '-b:v', '3M',
                   '-pix_fmt', 'yuv420p', '-movflags', '+faststart', str(mp4)]
        subprocess.run(command, check=True)
        gif_filter = ('fps=10,scale=600:-1:flags=lanczos,split[a][b];'
                      '[a]palettegen=max_colors=128:stats_mode=diff[p];'
                      '[b][p]paletteuse=dither=bayer:bayer_scale=3')
        subprocess.run(['ffmpeg', '-hide_banner', '-loglevel', 'error', '-y',
                        '-i', str(mp4), '-filter_complex', gif_filter,
                        '-loop', '0', str(args.output / f'{name}.gif')], check=True)
        subprocess.run(['ffmpeg', '-hide_banner', '-loglevel', 'error', '-y',
                        '-ss', '5', '-i', str(mp4), '-frames:v', '1',
                        str(args.output / f'{name}.png')], check=True)
        entries.append({'id': name, 'source': str(source),
                        'source_sha256': hashlib.sha256(source.read_bytes()).hexdigest(),
                        'start_s': start, 'duration_s': duration, 'speed': 1,
                        'crop_xywh': [int(n) for n in crop.split(':')[2:]]
                        + [int(n) for n in crop.split(':')[:2]],
                        'command': command, 'kind': 'recorded simulation viewport'})
    (args.output / 'clips.json').write_text(json.dumps(entries, indent=2) + '\n')


if __name__ == '__main__':
    main()
