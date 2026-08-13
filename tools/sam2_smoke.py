#!/usr/bin/env python3
"""Load a SAM2 checkpoint and validate one bbox-prompted CUDA mask."""

from __future__ import annotations

import argparse
from contextlib import nullcontext
import json
from pathlib import Path
import time
from typing import Any, Sequence

import numpy as np


DEFAULT_MODEL_CONFIG = 'configs/sam2.1/sam2.1_hiera_s.yaml'


def synthetic_image(width: int = 640, height: int = 480) -> np.ndarray:
    """Create a deterministic high-contrast RGB image for a smoke test."""
    image = np.full((height, width, 3), 32, dtype=np.uint8)
    x_min, y_min, x_max, y_max = central_box(width, height)
    image[y_min:y_max, x_min:x_max] = (220, 80, 40)
    return image


def central_box(width: int, height: int) -> tuple[int, int, int, int]:
    if width < 4 or height < 4:
        raise ValueError('image dimensions must each be at least 4 pixels')
    return (width // 4, height // 4, width * 3 // 4, height * 3 // 4)


def select_mask(
    predicted_masks: Any,
    scores: Any,
    image_shape: tuple[int, int],
) -> tuple[np.ndarray, float]:
    masks = np.asarray(predicted_masks)
    score_values = np.asarray(scores, dtype=np.float64).reshape(-1)
    if masks.ndim == 2:
        mask = masks
    elif masks.ndim == 3 and masks.shape[0] >= 1:
        mask = masks[0]
    elif masks.ndim == 4 and masks.shape[:2] == (1, 1):
        mask = masks[0, 0]
    else:
        raise ValueError(f'unexpected SAM2 mask shape: {masks.shape}')
    if mask.shape != image_shape:
        raise ValueError(
            f'mask shape {mask.shape} does not match image {image_shape}'
        )
    score = float(score_values[0]) if score_values.size else 0.0
    return mask.astype(np.bool_), score


def run_smoke(
    checkpoint: Path,
    model_config: str,
    device: str,
) -> dict[str, Any]:
    if not checkpoint.is_file():
        raise FileNotFoundError(f'SAM2 checkpoint not found: {checkpoint}')

    import torch
    from sam2.build_sam import build_sam2
    from sam2.sam2_image_predictor import SAM2ImagePredictor

    if device == 'cuda' and not torch.cuda.is_available():
        raise RuntimeError('CUDA was requested but torch.cuda is unavailable')

    image = synthetic_image()
    box = np.asarray(central_box(image.shape[1], image.shape[0]), np.float32)
    if device == 'cuda':
        torch.cuda.empty_cache()
        torch.cuda.reset_peak_memory_stats()
        torch.cuda.synchronize()

    load_started = time.monotonic()
    predictor = SAM2ImagePredictor(
        build_sam2(model_config, str(checkpoint), device=device)
    )
    if device == 'cuda':
        torch.cuda.synchronize()
    load_seconds = time.monotonic() - load_started

    inference_started = time.monotonic()
    inference_context = torch.inference_mode()
    autocast_context = (
        torch.autocast('cuda', dtype=torch.bfloat16)
        if device == 'cuda'
        else nullcontext()
    )
    with inference_context, autocast_context:
        predictor.set_image(image)
        masks, scores, _logits = predictor.predict(
            box=box,
            multimask_output=False,
        )
    if device == 'cuda':
        torch.cuda.synchronize()
    inference_seconds = time.monotonic() - inference_started

    mask, score = select_mask(masks, scores, image.shape[:2])
    mask_pixels = int(np.count_nonzero(mask))
    if mask_pixels == 0:
        raise RuntimeError('SAM2 returned an empty mask')

    return {
        'success': True,
        'torch_version': torch.__version__,
        'torch_cuda_version': torch.version.cuda,
        'device': device,
        'device_name': (
            torch.cuda.get_device_name(0) if device == 'cuda' else None
        ),
        'model_config': model_config,
        'checkpoint': str(checkpoint.resolve()),
        'image_shape': list(image.shape),
        'bbox_xyxy': box.tolist(),
        'mask_shape': list(mask.shape),
        'mask_pixels': mask_pixels,
        'mask_fraction': mask_pixels / mask.size,
        'score': score,
        'load_seconds': load_seconds,
        'inference_seconds': inference_seconds,
        'peak_cuda_memory_bytes': (
            int(torch.cuda.max_memory_allocated())
            if device == 'cuda'
            else None
        ),
    }


def _parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--checkpoint', type=Path, required=True)
    parser.add_argument('--model-config', default=DEFAULT_MODEL_CONFIG)
    parser.add_argument('--device', choices=('cuda', 'cpu'), default='cuda')
    parser.add_argument('--output', type=Path)
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    options = _parse_args(argv)
    try:
        report = run_smoke(
            options.checkpoint,
            options.model_config,
            options.device,
        )
        status = 0
    except Exception as error:
        report = {
            'success': False,
            'error_type': type(error).__name__,
            'error': str(error),
        }
        status = 2

    rendered = json.dumps(report, indent=2, ensure_ascii=False) + '\n'
    if options.output is None:
        print(rendered, end='')
    else:
        options.output.parent.mkdir(parents=True, exist_ok=True)
        options.output.write_text(rendered, encoding='utf-8')
        print(rendered, end='')
    return status


if __name__ == '__main__':
    raise SystemExit(main())
