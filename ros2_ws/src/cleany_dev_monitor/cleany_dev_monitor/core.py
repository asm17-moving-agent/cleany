"""화면 없는 관측 기록 계약을 공유하는 실시간 브라우저 캐시입니다."""
from cleany_logger.observation import ObservationBuffer, grid_origin, preview, transform_point, yaw
from .camera import Cameras


class Hub(ObservationBuffer):
    def __init__(self, event_limit=2048):
        super().__init__(event_limit)
        self.cameras = Cameras()
