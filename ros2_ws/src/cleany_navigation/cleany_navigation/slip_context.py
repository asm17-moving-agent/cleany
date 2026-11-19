"""Bounded diagnostic command history; never feeds control or slip decisions."""
from collections import deque
import math
from typing import Any


class CommandHistory:
    """Select the last received Twist at or before an observation's end time.

    Times are ROS receive times because Twist has no header. Future samples
    are never substituted for a missing historical command.
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
    """Retain explicit nulls for unavailable numeric diagnostic values."""
    if isinstance(value, dict):
        return {k: json_safe(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [json_safe(v) for v in value]
    return None if isinstance(value, float) and not math.isfinite(value) else value
