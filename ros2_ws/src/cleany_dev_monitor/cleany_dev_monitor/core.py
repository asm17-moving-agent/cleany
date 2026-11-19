"""Live browser cache, sharing the headless observation record contract."""
from cleany_logger.observation import (
    ObservationBuffer,
    grid_origin,
    preview,
    transform_point,
    yaw,
)

__all__ = ["Hub", "grid_origin", "preview", "transform_point", "yaw"]


class Hub(ObservationBuffer):
    """Live UI state; camera frames belong to the telemetry preview provider."""
