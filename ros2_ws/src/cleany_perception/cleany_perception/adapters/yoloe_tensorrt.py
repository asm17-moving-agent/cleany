"""Static TensorRT execution with Ultralytics preprocessing and retina masks."""
from __future__ import annotations

from typing import Any

import numpy as np


class CudaSynchronizationError(RuntimeError):
    """The CUDA stream could not be drained; retrying may return unsafe data."""


class TensorRTAsyncRunner:
    """Own a CUDA stream, execution context and output buffers for one model.

    The caller must serialize prediction and CPU result copies with the model's
    inference lock. The returned Results borrow reusable GPU output storage.
    """

    def __init__(self, model: Any) -> None:
        predictor = model.predictor
        backend = predictor.model.backend
        if backend.dynamic or not backend.is_trt10:
            raise ValueError('Async execution requires a static TensorRT 10 engine')
        if predictor.model.nhwc or not predictor.model.end2end:
            raise ValueError('Async execution requires the validated NCHW end-to-end engine')
        import torch

        self._torch = torch
        self._predictor = predictor
        self._input = backend.bindings['images'].data
        self._device = self._input.device
        self._stream = torch.cuda.Stream(device=self._device)
        self._context = backend.model.create_execution_context()
        if self._context is None:
            raise RuntimeError('Cannot create the async TensorRT execution context')
        self._outputs = {
            name: torch.empty_like(backend.bindings[name].data)
            for name in sorted(backend.output_names)
        }
        for name, tensor in self._outputs.items():
            if not self._context.set_tensor_address(name, tensor.data_ptr()):
                raise RuntimeError(f'TensorRT refused output binding: {name}')

    def predict(self, rgb: np.ndarray, *, confidence: float, iou: float,
                maximum_detections: int) -> Any:
        """Return completed full-resolution masks without per-stage profiling."""
        torch, predictor = self._torch, self._predictor
        with torch.cuda.device(self._device), torch.cuda.stream(self._stream), torch.inference_mode():
            try:
                predictor.args.conf = confidence
                predictor.args.iou = iou
                predictor.args.max_det = maximum_detections
                predictor.args.retina_masks = True
                predictor.args.rect = False
                bgr = np.ascontiguousarray(rgb[..., ::-1])
                predictor.batch = (['memory.png'], [bgr], [''])
                tensor = predictor.preprocess([bgr])
                if (tensor.shape != self._input.shape or tensor.dtype != self._input.dtype
                        or tensor.device != self._device or not tensor.is_contiguous()):
                    raise ValueError('Input differs from the validated TensorRT binding')
                if not self._context.set_tensor_address('images', tensor.data_ptr()):
                    raise RuntimeError('TensorRT refused input binding')
                if not self._context.execute_async_v3(self._stream.cuda_stream):
                    raise RuntimeError('TensorRT asynchronous execution failed')
                results = predictor.postprocess(list(self._outputs.values()), tensor, [bgr])
                predictor.results = results
                return results
            finally:
                # Also drain on failure before the caller can reuse the legacy
                # context. A failed drain must never trigger a CUDA retry.
                try:
                    self._stream.synchronize()
                except Exception as error:
                    raise CudaSynchronizationError(
                        f'Cannot synchronize YOLOE CUDA stream: {error}') from error
