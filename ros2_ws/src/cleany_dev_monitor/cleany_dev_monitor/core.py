"""Live browser cache, sharing the headless observation record contract."""
from cleany_logger.observation import ObservationBuffer, grid_origin, preview, transform_point, yaw
from .camera import Cameras


class Hub(ObservationBuffer):
    def __init__(self, event_limit=2048):
        super().__init__(event_limit)
        self.cameras = Cameras()
