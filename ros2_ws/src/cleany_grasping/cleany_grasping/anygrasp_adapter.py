from __future__ import annotations

import os
import gc
from contextlib import contextmanager, nullcontext
from pathlib import Path
from threading import Lock
from types import SimpleNamespace

import numpy as np

from cleany_grasping.core.models import PointCloud, RawGrasp


_CWD_LOCK = Lock()


def _torch_runtime():
    """Load torch only for the licensed GPU predictor path."""
    try:
        import torch
    except (ImportError, OSError):
        return None
    return torch


class ModelUnavailableError(RuntimeError):
    pass


@contextmanager
def _working_directory(path: Path):
    with _CWD_LOCK:
        previous = Path.cwd()
        os.chdir(path)
        try:
            yield
        finally:
            os.chdir(previous)


class AnyGraspPredictor:
    """Lazy boundary around the licensed AnyGrasp aarch64 SDK."""

    def __init__(
        self,
        checkpoint_path: str,
        license_path: str,
        maximum_gripper_width_m: float = 0.10,
        gripper_height_m: float = 0.03,
        cuda_cache_release_threshold_mb: float = 512.0,
        adaptive_sampling_enabled: bool = False,
        adaptive_maximum_points: int = 8000,
        adaptive_minimum_region_points: int = 2000,
    ) -> None:
        if cuda_cache_release_threshold_mb < 0.0:
            raise ValueError(
                'AnyGrasp CUDA cache release threshold must not be negative'
            )
        if adaptive_maximum_points < 3:
            raise ValueError(
                'AnyGrasp adaptive maximum points must be at least three'
            )
        if not 1 <= adaptive_minimum_region_points <= adaptive_maximum_points:
            raise ValueError(
                'AnyGrasp adaptive minimum region points must be between '
                'one and the maximum point count'
            )
        self._checkpoint_path = Path(checkpoint_path).expanduser()
        self._license_path = Path(license_path).expanduser()
        self._maximum_gripper_width_m = maximum_gripper_width_m
        self._gripper_height_m = gripper_height_m
        self._cuda_cache_release_threshold_bytes = int(
            cuda_cache_release_threshold_mb * 1024 * 1024
        )
        self._adaptive_sampling_enabled = adaptive_sampling_enabled
        self._adaptive_maximum_points = adaptive_maximum_points
        self._adaptive_minimum_region_points = adaptive_minimum_region_points
        self._model = None
        self._load_lock = Lock()

    def _load(self):
        if self._model is not None:
            return self._model
        with self._load_lock:
            if self._model is not None:
                return self._model
            if (
                not self._checkpoint_path.is_file()
                or not self._license_path.is_file()
            ):
                raise ModelUnavailableError(
                    'AnyGrasp checkpoint and SDK license files must be '
                    'configured'
                )
            license_dir = self._license_path.parent
            if license_dir.name != 'license':
                raise ModelUnavailableError(
                    'AnyGrasp license files must be in a directory named '
                    'license'
                )
            try:
                from gsnet import check_license, create_detector
            except (ImportError, OSError) as error:
                raise ModelUnavailableError(
                    f'AnyGrasp SDK is unavailable: {error}'
                ) from error
            if not check_license(str(license_dir)):
                raise ModelUnavailableError(
                    'AnyGrasp license validation failed'
                )
            configuration = SimpleNamespace(
                checkpoint_path=str(self._checkpoint_path.resolve()),
                max_gripper_width=self._maximum_gripper_width_m,
                gripper_height=self._gripper_height_m,
            )
            # The 2026 SDK resolves license/licenseCfg.json relative to cwd.
            with _working_directory(license_dir.parent):
                model = create_detector(configuration)
            if model is None:
                raise ModelUnavailableError(
                    'AnyGrasp detector initialization failed'
                )
            self._model = model
            return model

    def prepare(self) -> None:
        self._load()

    def predict(
        self,
        target_cloud: PointCloud,
        context_cloud: PointCloud,
        workspace_bounds: np.ndarray,
    ):
        del target_cloud
        points, region_mask = self._prepare_points(
            context_cloud,
            workspace_bounds,
        )
        if self.should_retry_full_resolution(context_cloud):
            points, region_mask = self._adaptive_sample(points, region_mask)
        return self._predict_points(points, region_mask)

    def should_retry_full_resolution(self, context_cloud: PointCloud) -> bool:
        """Return whether the fast path omitted context points."""
        return (
            self._adaptive_sampling_enabled
            and context_cloud.points.shape[0] > self._adaptive_maximum_points
        )

    def predict_full_resolution(
        self,
        target_cloud: PointCloud,
        context_cloud: PointCloud,
        workspace_bounds: np.ndarray,
    ) -> tuple[RawGrasp, ...]:
        """Retry with the exact baseline cloud after fast-path rejection."""
        del target_cloud
        points, region_mask = self._prepare_points(
            context_cloud,
            workspace_bounds,
        )
        return self._predict_points(points, region_mask)

    @staticmethod
    def _prepare_points(
        context_cloud: PointCloud,
        workspace_bounds: np.ndarray,
    ) -> tuple[np.ndarray, np.ndarray]:
        bounds = np.asarray(workspace_bounds, dtype=np.float32)
        if bounds.shape != (6,):
            raise ValueError('AnyGrasp workspace bounds must have six values')
        xyz_bounds = bounds.reshape((3, 2))
        points = context_cloud.points.astype(np.float32)
        region_mask = np.all(
            (points >= xyz_bounds[:, 0]) & (points <= xyz_bounds[:, 1]),
            axis=1,
        )
        return points, region_mask

    def _adaptive_sample(
        self,
        points: np.ndarray,
        region_mask: np.ndarray,
    ) -> tuple[np.ndarray, np.ndarray]:
        maximum = self._adaptive_maximum_points
        region_indices = np.flatnonzero(region_mask)
        context_indices = np.flatnonzero(~region_mask)
        proportional_region = round(
            maximum * region_indices.size / max(1, points.shape[0])
        )
        region_budget = min(
            region_indices.size,
            max(self._adaptive_minimum_region_points, proportional_region),
        )
        context_budget = min(context_indices.size, maximum - region_budget)
        region_budget = min(
            region_indices.size,
            maximum - context_budget,
        )

        def spaced(indices: np.ndarray, count: int) -> np.ndarray:
            if count >= indices.size:
                return indices
            positions = np.linspace(0, indices.size - 1, count, dtype=int)
            return indices[positions]

        selected = np.sort(
            np.concatenate(
                (
                    spaced(region_indices, region_budget),
                    spaced(context_indices, context_budget),
                )
            )
        )
        return points[selected], region_mask[selected]

    def _predict_points(
        self,
        points: np.ndarray,
        region_mask: np.ndarray,
    ) -> tuple[RawGrasp, ...]:
        model = self._load()
        torch = _torch_runtime()
        inference_context = (
            torch.inference_mode() if torch is not None else nullcontext()
        )
        group = None
        try:
            with inference_context:
                group = model.get_grasp(
                    points,
                    {
                        'dense_grasp': False,
                        'collision_detection': True,
                        'region_steering': region_mask,
                        'approach_steering': None,
                        'approach_thresh': np.pi,
                    },
                )
            if group is None:
                return ()
            return tuple(
                RawGrasp(
                    rotation,
                    translation,
                    float(width),
                    float(depth),
                    float(score),
                )
                for rotation, translation, width, depth, score in zip(
                    group.rotation_matrices,
                    group.translations,
                    group.widths,
                    group.depths,
                    group.scores,
                )
            )
        finally:
            group = None
            self._release_unused_cuda_cache(torch)

    def _release_unused_cuda_cache(self, torch) -> None:
        threshold = self._cuda_cache_release_threshold_bytes
        if torch is None or threshold == 0:
            return
        cuda = getattr(torch, 'cuda', None)
        try:
            if cuda is None or not cuda.is_available():
                return
            cuda.synchronize()
            unused = int(cuda.memory_reserved()) - int(
                cuda.memory_allocated()
            )
            if unused < threshold:
                return
            gc.collect()
            cuda.empty_cache()
        except (AttributeError, RuntimeError):
            # CUDA cache release is best-effort and must not hide a valid
            # grasp result or replace the SDK's original inference error.
            return
