"""Combine camera-specific YOLO datasets without copying RGB or labels."""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import yaml


def combine(output: Path, sources: dict[str, Path]) -> dict:
    if not sources:
        raise ValueError('At least one source dataset is required')
    if output.exists() and any(output.iterdir()):
        raise FileExistsError(f'Dataset output must be empty: {output}')
    names = None
    counts = {}
    entries = []
    for prefix, config_path in sources.items():
        if not prefix or not prefix.replace('_', '').isalnum():
            raise ValueError(f'Invalid source prefix: {prefix}')
        config = yaml.safe_load(config_path.read_text(encoding='utf-8'))
        root = Path(config['path']).expanduser().resolve()
        if names is None:
            names = config['names']
        elif config['names'] != names:
            raise ValueError(f'Class names/order mismatch: {config_path}')
        for split in ('train', 'val'):
            images = sorted((root / config[split]).glob('*.png'))
            if not images:
                raise ValueError(f'No {split} images: {config_path}')
            counts[f'{split}_{prefix}'] = len(images)
            for image in images:
                label = root / 'labels' / split / f'{image.stem}.txt'
                if not label.is_file():
                    raise FileNotFoundError(label)
                entries.append((split, f'{prefix}_{image.stem}', image, label))
    for split in ('train', 'val'):
        for kind in ('images', 'labels'):
            (output / kind / split).mkdir(parents=True, exist_ok=True)
    for split, stem, image, label in entries:
        (output / 'images' / split / f'{stem}.png').symlink_to(image.resolve())
        (output / 'labels' / split / f'{stem}.txt').symlink_to(label.resolve())
    config = {'path': str(output.resolve()), 'train': 'images/train',
              'val': 'images/val', 'names': names}
    (output / 'dataset.yaml').write_text(
        yaml.safe_dump(config, sort_keys=False), encoding='utf-8')
    manifest = {'sources': {name: str(path.resolve()) for name, path in sources.items()},
                'counts': counts}
    (output / 'manifest.json').write_text(
        json.dumps(manifest, indent=2) + '\n', encoding='utf-8')
    return manifest


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--source', action='append', required=True,
                        help='prefix=/absolute/path/to/dataset.yaml')
    args = parser.parse_args()
    sources = {}
    for item in args.source:
        if '=' not in item:
            parser.error('Each --source must be prefix=dataset.yaml')
        prefix, path = item.split('=', 1)
        if prefix in sources:
            parser.error(f'Duplicate source prefix: {prefix}')
        sources[prefix] = Path(path).expanduser().resolve()
    print(json.dumps(combine(args.output.expanduser(), sources), indent=2))


if __name__ == '__main__':
    main()
