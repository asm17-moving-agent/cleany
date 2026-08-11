from __future__ import annotations

from pathlib import Path

import numpy as np

from cleany_grasping.core.models import PointCloud, RawGrasp


class ModelUnavailableError(RuntimeError):
    pass


class AnyGraspPredictor:
    """Thin, lazy-loading boundary around the licensed AnyGrasp SDK."""

    def __init__(self, checkpoint_path: str, license_path: str) -> None:
        self._checkpoint_path = Path(checkpoint_path)
        self._license_path = Path(license_path)
        self._model = None

    def _load(self):
        if self._model is not None:
            return self._model
        if not self._checkpoint_path.is_file() or not self._license_path.is_file():
            raise ModelUnavailableError(
                'AnyGrasp checkpoint and SDK license files must be configured'
            )
        try:
            from gsnet import AnyGrasp
        except (ImportError, OSError) as error:
            raise ModelUnavailableError(f'AnyGrasp SDK is unavailable: {error}') from error
        configuration = {
            'checkpoint_path': str(self._checkpoint_path),
            'license_dir': str(self._license_path.parent),
        }
        model = AnyGrasp(configuration)
        model.load_net()
        self._model = model
        return model

    def predict(self, context_cloud: PointCloud, workspace_bounds: np.ndarray):
        model = self._load()
        result = model.get_grasp(
            context_cloud.points.astype(np.float32),
            context_cloud.colors.astype(np.float32),
            lims=np.asarray(workspace_bounds, dtype=np.float32).tolist(),
            apply_object_mask=False,
            dense_grasp=False,
            collision_detection=True,
        )
        group = result[0] if isinstance(result, tuple) else result
        if group is None:
            return ()
        # The SDK's GraspGroup exposes vectorized public arrays.
        return tuple(
            RawGrasp(rotation, translation, float(width), float(depth), float(score))
            for rotation, translation, width, depth, score in zip(
                group.rotation_matrices,
                group.translations,
                group.widths,
                group.depths,
                group.scores,
            )
        )
