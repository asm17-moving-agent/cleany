"""Prepare the exact RGB snapshot used for an external image request."""
from __future__ import annotations

import fcntl
from functools import lru_cache
import os
from pathlib import Path
from threading import Lock

import numpy as np
from PIL import Image

from .core.models import FailureKind, InspectionFailure, RgbArray


DEFAULT_MODELS = Path('/var/opt/cleany/models/camera-privacy/active')
DEFAULT_GPU_LOCK = '/var/opt/cleany/state/cleany/gpu-work.lock'
_inference_lock = Lock()


@lru_cache(maxsize=1)
def _redactor():
    from .privacy import CameraRedactor

    root = Path(os.environ.get('CLEANY_CAMERA_PRIVACY_MODELS', DEFAULT_MODELS))
    return CameraRedactor(root / 'privacy.engine', root / 'face.onnx', root / 'text.plan',
                          root / 'profile.yaml', device='0')


def for_gemini(rgb: RgbArray) -> Image.Image:
    """Prepare a private copy for annotation or upload, preserving pixel coordinates."""
    pixels = np.asarray(rgb)
    if pixels.dtype != np.uint8 or pixels.ndim != 3 or pixels.shape[2] != 3 or not pixels.size:
        raise InspectionFailure(FailureKind.DETECTOR_RESPONSE, 'Expected an HxWx3 uint8 RGB image')
    if os.environ.get('CLEANY_GEMINI_IMAGE_MODE', 'original') == 'original':
        return Image.fromarray(pixels.copy())
    try:
        lock = Path(os.environ.get('CLEANY_CAMERA_PRIVACY_GPU_LOCK', DEFAULT_GPU_LOCK))
        with _inference_lock, lock.open('a') as lease:
            fcntl.flock(lease, fcntl.LOCK_EX | fcntl.LOCK_NB)
            result = _redactor().redact(np.ascontiguousarray(pixels[:, :, ::-1]))
        if result is None or result.image.dtype != np.uint8 or result.image.shape != pixels.shape:
            raise ValueError('Invalid redacted snapshot')
        return Image.fromarray(np.ascontiguousarray(result.image[:, :, ::-1]))
    except Exception as error:
        raise InspectionFailure(FailureKind.DETECTOR_API,
                                f'Gemini image withheld during redaction ({type(error).__name__})') from error
