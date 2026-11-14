"""크기 제한 진단용 명령 이력이며 제어나 미끄럼 판정의 입력이 아닙니다."""
from collections import deque
import math
from typing import Any


class CommandHistory:
    """관측 종료 시각 이하에서 마지막으로 수신한 Twist를 선택합니다.

    Twist에는 header가 없으므로 ROS 수신 시각을 사용합니다. 과거 명령이 없을 때
    미래 표본으로 대체하지 않습니다.
    """

    def __init__(self, max_age_s: float = .3):
        if not math.isfinite(max_age_s) or max_age_s <= 0:
            raise ValueError('command context age must be positive and finite')
        self.max_age_s = max_age_s
        self.samples: deque = deque(maxlen=512)

    def add(self, stamp: float, velocity: tuple[float, float, float]) -> None:
        if not all(math.isfinite(v) for v in (stamp, *velocity)):
            return
        if self.samples and stamp < self.samples[-1][0]:
            self.samples.clear()
        self.samples.append((stamp, velocity))
        while self.samples and stamp-self.samples[0][0] > 4.:
            self.samples.popleft()

    def at(self, stamp: float) -> dict[str, Any]:
        for received, velocity in reversed(self.samples):
            if received <= stamp:
                age = stamp-received
                return dict(velocity=list(velocity) if age <= self.max_age_s else None,
                            received_at=received, age_s=age,
                            reason='' if age <= self.max_age_s else 'stale')
        return dict(velocity=None, received_at=None, age_s=None, reason='no_prior_sample')


def json_safe(value: Any) -> Any:
    """사용할 수 없는 숫자 진단값은 명시적인 null로 유지합니다."""
    if isinstance(value, dict):
        return {k: json_safe(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [json_safe(v) for v in value]
    return None if isinstance(value, float) and not math.isfinite(value) else value
