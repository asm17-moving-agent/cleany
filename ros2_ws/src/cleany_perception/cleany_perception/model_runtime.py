"""Local assets and explicit device selection for learned models."""
from __future__ import annotations

from pathlib import Path
from typing import Mapping

import yaml


def load_model_profile(path: str | Path) -> dict:
    with Path(path).open(encoding='utf-8') as stream:
        data = yaml.safe_load(stream)
    return data['perception_inspector']['ros__parameters']


def resolve_device(requested: str, cuda_available: bool) -> str:
    if requested == 'auto':
        return 'cuda:0' if cuda_available else 'cpu'
    if requested == 'cpu':
        return requested
    if requested == 'cuda' or (
        requested.startswith('cuda:') and requested[5:].isdigit()
    ):
        if not cuda_available:
            raise ValueError('CUDA unavailable; explicitly select cpu or auto')
        return requested
    raise ValueError(f'Unsupported perception device: {requested!r}')


def resolve_model_assets(
    values: Mapping[str, str], model_directory: str, kind: str,
) -> dict[str, str]:
    root = Path(model_directory).expanduser().resolve()

    def asset(key: str, directory: bool = False) -> str:
        raw = values[key].strip()
        if not raw:
            raise ValueError(f'{key} must name a local asset')
        path = Path(raw).expanduser()
        path = (path if path.is_absolute() else root / path).resolve()
        if not (path.is_dir() if directory else path.is_file()):
            raise ValueError(f'{key} not found: {path}')
        return str(path)

    if kind == 'yoloe':
        result = {
            'yoloe_model_path': asset('yoloe_model_path'),
            'yoloe_text_encoder_directory': asset(
                'yoloe_text_encoder_directory', directory=True
            ),
        }
        encoder = (
            Path(result['yoloe_text_encoder_directory']) / 'mobileclip2_b.ts'
        )
        if not encoder.is_file():
            raise ValueError(f'YOLOE text encoder not found: {encoder}')
        return result
    if kind == 'sam2':
        if not values['sam2_model_config'].strip():
            raise ValueError('sam2_model_config must not be empty')
        return {'sam2_checkpoint': asset('sam2_checkpoint')}
    raise ValueError(f'Unsupported local model kind: {kind}')
