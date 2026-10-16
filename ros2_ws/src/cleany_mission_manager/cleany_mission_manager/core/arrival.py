"""Navigation arrival bounds; these do not certify manipulation reachability."""
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class ArrivalError:
    position_m: float
    yaw_rad: float

    def within(self, position_tolerance: float, yaw_tolerance: float) -> bool:
        if any(not math.isfinite(v) or v <= 0 for v in (position_tolerance, yaw_tolerance)):
            raise ValueError("arrival tolerances must be finite and positive")
        return self.position_m <= position_tolerance and self.yaw_rad <= yaw_tolerance


def arrival_error(expected: tuple[float, float, float], actual: tuple[float, float, float]) -> ArrivalError:
    if len(expected) != 3 or len(actual) != 3 or not all(math.isfinite(v) for v in (*expected, *actual)):
        raise ValueError("arrival poses must be finite x, y, yaw")
    delta = actual[2] - expected[2]
    return ArrivalError(math.hypot(actual[0]-expected[0], actual[1]-expected[1]),
                        abs(math.atan2(math.sin(delta), math.cos(delta))))
