"""TensorRT execution of the fixed 640-pixel DB text probability map on Jetson."""
from pathlib import Path

import numpy as np


class TextEngine:
    def __init__(self, path: Path, device: str):
        import tensorrt as trt
        import torch

        self.torch = torch
        self.logger = trt.Logger(trt.Logger.WARNING)
        self.runtime = trt.Runtime(self.logger)
        self.engine = self.runtime.deserialize_cuda_engine(path.read_bytes())
        if self.engine is None:
            raise ValueError('Cannot deserialize the text detection engine')
        names = [self.engine.get_tensor_name(i) for i in range(self.engine.num_io_tensors)]
        self.source, = [n for n in names if self.engine.get_tensor_mode(n) == trt.TensorIOMode.INPUT]
        dest, = [n for n in names if self.engine.get_tensor_mode(n) == trt.TensorIOMode.OUTPUT]
        if any(self.engine.get_tensor_dtype(n) != trt.float32 for n in names):
            raise ValueError('Text detection requires float32 I/O bindings')
        self.context = self.engine.create_execution_context()
        shape = (1, 3, 640, 640)
        if not self.context.set_input_shape(self.source, shape):
            raise ValueError('Text engine needs a 640x640 input profile')
        if tuple(self.context.get_tensor_shape(dest)) != (1, 1, 640, 640):
            raise ValueError('Unexpected text map dimensions')
        cuda = f'cuda:{device}' if device.isdigit() else device
        self.input = torch.empty(shape, device=cuda, dtype=torch.float32)
        self.output = torch.empty((1, 1, 640, 640), device=cuda, dtype=torch.float32)
        self.stream = torch.cuda.Stream(device=self.input.device)
        for name, tensor in ((self.source, self.input), (dest, self.output)):
            if not self.context.set_tensor_address(name, tensor.data_ptr()):
                raise RuntimeError('Text engine refused tensor binding')

    def predict(self, blob: np.ndarray) -> np.ndarray:
        h, w = blob.shape[-2:]
        with self.torch.cuda.stream(self.stream):
            self.input.zero_()
            self.input[:, :, :h, :w].copy_(self.torch.from_numpy(blob))
            if not self.context.execute_async_v3(self.stream.cuda_stream):
                raise RuntimeError('Text engine failed')
            return self.output[0, 0, :h, :w].cpu().numpy()
