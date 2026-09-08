"""Bounded SAM2 video inference, seeded by a detection or explicit prior box.

The public SAM2 video API reads JPEG frames. RGB inputs are encoded with
quality=100/subsampling=0 in an automatically removed private temporary folder.
No synthetic confidence, image-predictor IoU, or fresh semantic detection is
claimed for the resulting mask. Callers must validate current depth/geometry.
"""
from __future__ import annotations

from contextlib import nullcontext
from functools import wraps
from pathlib import Path
from tempfile import TemporaryDirectory
import threading
from typing import Any, Callable, Sequence

import numpy as np
from PIL import Image

from cleany_perception.core.models import (
    Detection2D, FailureKind, InspectionFailure, MaskArray, RgbArray,
)


def _serialized(method):
    @wraps(method)
    def call(self, *args, **kwargs):
        with self._lock:
            return method(self, *args, **kwargs)
    return call


class Sam2ReferenceTracker:
    def __init__(self, model_config: str, checkpoint_path: str,
                 device: str = 'cuda',
                 predictor_factory: Callable[[str, str, str], Any] | None = None):
        self._config = model_config
        self._checkpoint = checkpoint_path
        self._device = device
        self._factory = predictor_factory
        self._predictor = None
        self._lock = threading.RLock()
        self._streams = 0
        self._stream_threads = None
        self._previous_threads = None

    @_serialized
    def prepare(self) -> None:
        if self._predictor is not None:
            return
        if not self._config or not self._device or not Path(self._checkpoint).is_file():
            raise InspectionFailure(FailureKind.MASK, 'Invalid SAM2 reference model assets/device')
        try:
            if self._factory is not None:
                self._predictor = self._factory(self._config, self._checkpoint, self._device)
            else:
                from sam2.build_sam import build_sam2_video_predictor

                self._predictor = build_sam2_video_predictor(
                    self._config, self._checkpoint, device=self._device)
        except Exception as error:
            raise InspectionFailure(FailureKind.MASK, f'SAM2 reference model: {error}') from error

    def track(self, reference_rgb: RgbArray, detection: Detection2D,
              current_rgb: RgbArray) -> MaskArray:
        return self.track_sequence((reference_rgb, current_rgb), detection)

    @_serialized
    def track_sequence(self, frames: Sequence[RgbArray], detection: Detection2D,
                       *, reference_mask: MaskArray | None = None) -> MaskArray:
        """Return only the last mask; intermediate frames bridge appearance changes."""
        if not 2 <= len(frames) <= 16:
            raise InspectionFailure(FailureKind.MASK, 'SAM2 sequence requires 2..16 frames')
        reference_rgb = frames[0]
        if (reference_rgb.ndim != 3 or reference_rgb.shape[2] != 3
                or any(frame.shape != reference_rgb.shape or frame.dtype != np.uint8 for frame in frames)):
            raise InspectionFailure(FailureKind.MASK, 'Reference/current RGB dimensions or types differ')
        height, width = reference_rgb.shape[:2]
        if reference_mask is not None and (reference_mask.shape != (height, width)
                or reference_mask.dtype != np.bool_ or not reference_mask.any()):
            raise InspectionFailure(FailureKind.MASK, 'Invalid reference seed mask')
        box = detection.bbox
        if box.x_max > width or box.y_max > height:
            raise InspectionFailure(FailureKind.MASK, 'Reference box is outside its RGB image')
        self.prepare()
        try:
            context = nullcontext()
            if self._factory is None:
                import torch

                context = torch.inference_mode()
            with context, TemporaryDirectory(prefix='cleany-sam2-reference-') as directory:
                for index, rgb in enumerate(frames):
                    Image.fromarray(rgb).save(
                        Path(directory) / f'{index:05d}.jpg', quality=100, subsampling=0)
                state = self._predictor.init_state(
                    directory, offload_video_to_cpu=True, offload_state_to_cpu=True)
                if reference_mask is None:
                    self._predictor.add_new_points_or_box(
                        state, frame_idx=0, obj_id=1,
                        box=np.array((box.x_min, box.y_min, box.x_max, box.y_max), np.float32))
                else:
                    self._predictor.add_new_mask(state, frame_idx=0, obj_id=1, mask=reference_mask)
                current_mask = None
                for index, ids, logits in self._predictor.propagate_in_video(state):
                    if index != len(frames)-1:
                        continue
                    if list(ids) != [1] or tuple(logits.shape) != (1, 1, height, width):
                        raise ValueError('Unexpected SAM2 reference identity/mask shape')
                    values = logits[0, 0]
                    if hasattr(values, 'cpu'):
                        values = values.cpu().numpy()
                    values = np.asarray(values)
                    if not np.isfinite(values).all():
                        raise ValueError('Nonfinite SAM2 reference logits')
                    current_mask = np.array(values > 0., dtype=np.bool_, copy=True)
                if current_mask is None:
                    raise ValueError('SAM2 did not produce the current frame')
                return current_mask
        except InspectionFailure:
            raise
        except Exception as error:
            raise InspectionFailure(FailureKind.MASK, f'SAM2 reference inference: {error}') from error

    @_serialized
    def start_stream(self, rgb: RgbArray, mask: MaskArray, *, memory_frames: int = 32,
                     cpu_threads: int = 4):
        """Initialize once; each subsequent step encodes only its new RGB frame.

        SAM2's installed video predictor has no public append-frame method.
        Sam2Stream isolates the checked state-layout dependency; no vendor edits.
        """
        self.prepare()
        if not isinstance(cpu_threads, int) or not 1 <= cpu_threads <= 8:
            raise ValueError('SAM2 streaming CPU threads must be 1..8')
        if self._streams and self._stream_threads != cpu_threads:
            raise ValueError('Concurrent streams cannot use different process thread budgets')
        if not self._streams:
            import torch
            self._stream_threads = cpu_threads
            self._previous_threads = torch.get_num_threads()
            if self._device == 'cpu':
                # Ultralytics select_device can override OMP_NUM_THREADS to 7
                # on this VM. Reserve compute for physics/control while tracking.
                torch.set_num_threads(cpu_threads)
        self._streams += 1
        try:
            return Sam2Stream(self, rgb, mask, memory_frames)
        except Exception:
            self._release_stream()
            raise

    def _release_stream(self) -> None:
        self._streams -= 1
        if not self._streams:
            import torch
            if self._device == 'cpu':
                torch.set_num_threads(self._previous_threads)
            self._previous_threads = self._stream_threads = None


class Sam2Stream:
    def __init__(self, owner: Sam2ReferenceTracker, rgb: RgbArray,
                 mask: MaskArray, memory_frames: int):
        self.owner = owner
        self.shape = rgb.shape
        self.state = None
        self.index = 0
        predictor = owner._predictor
        required = max(predictor.max_obj_ptrs_in_encoder,
                       predictor.num_maskmem * predictor.memory_temporal_stride_for_eval)
        if not required <= memory_frames <= 64:
            raise ValueError('Streaming memory must cover SAM2 attention and stay <=64 frames')
        self.memory_frames = memory_frames
        if (rgb.ndim != 3 or rgb.shape[2] != 3 or rgb.dtype != np.uint8
                or mask.shape != rgb.shape[:2] or mask.dtype != np.bool_ or not mask.any()):
            raise ValueError('Invalid SAM2 streaming seed')
        import torch

        with torch.inference_mode(), TemporaryDirectory(prefix='cleany-sam2-seed-') as directory:
            Image.fromarray(rgb).save(Path(directory) / '00000.jpg', quality=100, subsampling=0)
            state = predictor.init_state(directory, offload_video_to_cpu=True, offload_state_to_cpu=True)
            for key in ('images', 'num_frames', 'output_dict_per_obj', 'frames_tracked_per_obj',
                        'cached_features', 'temp_output_dict_per_obj'):
                if key not in state:
                    raise RuntimeError(f'Unsupported SAM2 streaming state: missing {key}')
            # The predictor only indexes images by frame index. Retain seed and
            # current tensor, not an ever-growing video tensor/JPEG directory.
            state['images'] = {0: state['images'][0]}
            predictor.add_new_mask(state, frame_idx=0, obj_id=1, mask=mask)
            predictor.propagate_in_video_preflight(state)
            self.state = state

    def track(self, rgb: RgbArray) -> MaskArray:
        import torch

        with self.owner._lock, torch.inference_mode():
            if self.state is None or rgb.shape != self.shape or rgb.dtype != np.uint8:
                raise ValueError('Closed SAM2 stream or changed RGB shape/type')
            predictor, state = self.owner._predictor, self.state
            # Same RGB resize/normalization as SAM2's JPEG loader; new frames
            # stay in memory and do not undergo a JPEG encode/decode roundtrip.
            pixels = np.array(Image.fromarray(rgb).resize((predictor.image_size, predictor.image_size)))
            tensor = torch.from_numpy(pixels).permute(2, 0, 1).float() / 255.
            tensor.sub_(torch.tensor((.485, .456, .406))[:, None, None])
            tensor.div_(torch.tensor((.229, .224, .225))[:, None, None])
            self.index += 1
            state['images'] = {0: state['images'][0], self.index: tensor}
            state['num_frames'] = self.index + 1
            result = None
            for index, ids, logits in predictor.propagate_in_video(
                    state, start_frame_idx=self.index, max_frame_num_to_track=0):
                if index != self.index or list(ids) != [1] or tuple(logits.shape) != (1, 1, *self.shape[:2]):
                    raise ValueError('Unexpected streaming SAM2 frame, identity or mask shape')
                values = logits[0, 0].cpu().numpy()
                if not np.isfinite(values).all():
                    raise ValueError('Nonfinite streaming SAM2 logits')
                result = np.array(values > 0., dtype=np.bool_, copy=True)
            if result is None:
                raise ValueError('SAM2 stream produced no mask')
            cutoff = self.index - self.memory_frames + 1
            for output in state['output_dict_per_obj'].values():
                for index in list(output['non_cond_frame_outputs']):
                    if index < cutoff:
                        del output['non_cond_frame_outputs'][index]
            for tracked in state['frames_tracked_per_obj'].values():
                for index in list(tracked):
                    if index < cutoff:
                        del tracked[index]
            return result

    def close(self) -> None:
        with self.owner._lock:
            if self.state is not None:
                self.state = None
                self.owner._release_stream()
