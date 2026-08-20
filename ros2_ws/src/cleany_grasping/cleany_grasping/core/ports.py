from __future__ import annotations

from typing import Protocol

import numpy as np
from numpy.typing import NDArray

from cleany_grasping.core.models import PointCloud, RawGrasp


class GraspPredictor(Protocol):
    def predict(
        self,
        target_cloud: PointCloud,
        context_cloud: PointCloud,
        workspace_bounds: NDArray[np.float64],
    ) -> tuple[RawGrasp, ...]: ...
