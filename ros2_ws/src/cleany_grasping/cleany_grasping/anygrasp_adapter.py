from __future__ import annotations

import os
from contextlib import contextmanager
from pathlib import Path
from threading import Lock
from types import SimpleNamespace

import numpy as np

from cleany_grasping.core.models import PointCloud, RawGrasp


_CWD_LOCK = Lock()


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
    ) -> None:
        self._checkpoint_path = Path(checkpoint_path).expanduser()
        self._license_path = Path(license_path).expanduser()
        self._maximum_gripper_width_m = maximum_gripper_width_m
        self._gripper_height_m = gripper_height_m
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

    def predict(self, context_cloud: PointCloud, workspace_bounds: np.ndarray):
        model = self._load()
        bounds = np.asarray(workspace_bounds, dtype=np.float32)
        if bounds.shape != (6,):
            raise ValueError('AnyGrasp workspace bounds must have six values')
        xyz_bounds = bounds.reshape((3, 2))
        points = context_cloud.points.astype(np.float32)
        region_mask = np.all(
            (points >= xyz_bounds[:, 0]) & (points <= xyz_bounds[:, 1]),
            axis=1,
        )
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
