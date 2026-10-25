"""Fresh, measured stowed-arm evidence for permission to move the mobile base."""
from collections import deque
import math


class DriveGate:
    def __init__(self, positions: dict[str, float], *, maximum_age: float = .8,
                 tolerance: float = .03, velocity_limit: float = .02,
                 duration: float = .5, samples: int = 5):
        if not positions or not all(math.isfinite(v) for v in positions.values()):
            raise ValueError('Explicit finite stowed joint positions are required')
        self.positions = positions
        self.maximum_age, self.tolerance = maximum_age, tolerance
        self.velocity_limit, self.duration, self.samples = velocity_limit, duration, samples
        self.history = deque(maxlen=1024)
        self.stamp = -1
        self.signature = None

    def update(self, names, positions, velocities, stamp_ns: int, now: float) -> None:
        signature = (tuple(names), tuple(positions), tuple(velocities), stamp_ns)
        if signature == self.signature:
            return  # DDS duplicate does not refresh evidence.
        self.signature = signature
        if (stamp_ns <= self.stamp or len(names) != len(set(names))
                or len(positions) != len(names) or len(velocities) != len(names)):
            self.history.clear()
            return
        self.stamp = stamp_ns
        state = dict(zip(names, zip(positions, velocities)))
        valid = all(name in state and all(math.isfinite(v) for v in state[name])
                    and abs(state[name][0] - target) <= self.tolerance
                    and abs(state[name][1]) <= self.velocity_limit
                    for name, target in self.positions.items())
        if (not valid or self.history and now - self.history[-1][1] >= self.maximum_age):
            self.history.clear()
        if valid:
            self.history.append((stamp_ns, now))

    def ready(self, now: float, clock_ns: int) -> bool:
        if not self.history:
            return False
        stamp, receipt = self.history[-1]
        if not 0 <= now - receipt < self.maximum_age or not 0 <= (clock_ns-stamp)/1e9 < self.maximum_age:
            return False
        recent = [item for item in self.history if stamp-item[0] <= (self.duration+.1)*1e9]
        return (len(recent) >= self.samples
                and (recent[-1][0]-recent[0][0])/1e9 >= self.duration)
