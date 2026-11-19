from __future__ import annotations

from dataclasses import dataclass
from collections import deque
from math import atan2, cos, isfinite, sin


@dataclass(frozen=True)
class MecanumGeometry:
    """4륜 메카넘 기구학에 사용하는 물리 치수입니다."""

    wheel_radius_m: float
    wheelbase_m: float
    wheel_separation_m: float

    def __post_init__(self) -> None:
        values = (
            self.wheel_radius_m,
            self.wheelbase_m,
            self.wheel_separation_m,
        )
        if not all(isfinite(value) and value > 0.0 for value in values):
            raise ValueError('Mecanum geometry values must be positive and finite')

    @property
    def center_to_wheel_sum_m(self) -> float:
        """차체 중심에서 휠까지의 전후·좌우 거리 합을 반환합니다."""
        return (self.wheelbase_m + self.wheel_separation_m) / 2.0


@dataclass(frozen=True)
class WheelPositions:
    """누적 휠 회전량이며 단위는 rad입니다."""

    front_left: float
    front_right: float
    rear_left: float
    rear_right: float

    def __post_init__(self) -> None:
        if not all(isfinite(value) for value in self.as_tuple()):
            raise ValueError('Wheel positions must be finite')

    def as_tuple(self) -> tuple[float, float, float, float]:
        return (
            self.front_left,
            self.front_right,
            self.rear_left,
            self.rear_right,
        )


@dataclass(frozen=True)
class OdometryEstimate:
    """적분한 평면 위치와 차체 좌표계의 속도입니다."""

    x_m: float
    y_m: float
    yaw_rad: float
    linear_x_mps: float
    linear_y_mps: float
    angular_z_rps: float


class MecanumOdometry:
    """누적 휠 위치를 적분해 평면 odometry를 추정합니다."""

    def __init__(self, geometry: MecanumGeometry, *, velocity_window_s: float = 0.0,
                 yaw_scale: float = 1.0) -> None:
        if not isfinite(velocity_window_s) or velocity_window_s < 0.0:
            raise ValueError('Velocity window must be finite and non-negative')
        if not isfinite(yaw_scale) or yaw_scale <= 0.0:
            raise ValueError('Yaw scale must be positive and finite')
        self._geometry = geometry
        self._yaw_scale = yaw_scale
        self._velocity_window_s = velocity_window_s
        self._velocity_history: deque[tuple[float, WheelPositions]] = deque()
        self._previous_positions: WheelPositions | None = None
        self._previous_stamp_s: float | None = None
        self._x_m = 0.0
        self._y_m = 0.0
        self._yaw_rad = 0.0

    def update(
        self,
        positions: WheelPositions,
        stamp_s: float,
    ) -> OdometryEstimate | None:
        """중간 방향각을 사용하는 메카넘 odometry로 휠 변위를 적분합니다.

        시각이 증가하지 않으면 표본 기준만 초기화하고 위치는 유지합니다.
        속도 창을 사용하는 경우 속도 초기화 구간에도 위치 적분은 계속합니다.

        Args:
            positions: 누적 휠 회전량(rad).
            stamp_s: 한 시계에서 얻은 유한한 표본 시각(s).

        Returns:
            odom 좌표계의 위치와 차체 좌표계의 속도. 첫 표본, 시간 초기화 또는
            속도 창에 표본이 부족한 경우 None입니다.

        Raises:
            ValueError: 시각이 유한하지 않은 경우.
        """
        if not isfinite(stamp_s):
            raise ValueError('Sample timestamp must be finite')

        previous_positions = self._previous_positions
        previous_stamp_s = self._previous_stamp_s
        self._previous_positions = positions
        self._previous_stamp_s = stamp_s
        if (
            previous_positions is None
            or previous_stamp_s is None
            or stamp_s <= previous_stamp_s
        ):
            self._velocity_history.clear()
            self._velocity_history.append((stamp_s, positions))
            return None

        dt_s = stamp_s - previous_stamp_s
        forward_m, lateral_m, yaw_delta_rad = self._body_delta(
            positions, previous_positions
        )

        midpoint_yaw = self._yaw_rad + yaw_delta_rad / 2.0
        self._x_m += (
            cos(midpoint_yaw) * forward_m
            - sin(midpoint_yaw) * lateral_m
        )
        self._y_m += (
            sin(midpoint_yaw) * forward_m
            + cos(midpoint_yaw) * lateral_m
        )
        self._yaw_rad = atan2(
            sin(self._yaw_rad + yaw_delta_rad),
            cos(self._yaw_rad + yaw_delta_rad),
        )

        velocity_delta = (forward_m, lateral_m, yaw_delta_rad)
        velocity_dt = dt_s
        if self._velocity_window_s:
            # 위치는 받은 encoder 표본마다 적분합니다. 속도만 더 긴 관측 구간의 누적
            # count로 계산해 tick 양자화가 번갈아 튀는 순간 속도로 나타나지 않게 합니다.
            if dt_s > self._velocity_window_s:
                self._velocity_history.clear()
            self._velocity_history.append((stamp_s, positions))
            cutoff = stamp_s - self._velocity_window_s
            while len(self._velocity_history) > 1 and self._velocity_history[1][0] <= cutoff:
                self._velocity_history.popleft()
            old_stamp, old_positions = self._velocity_history[0]
            velocity_dt = stamp_s - old_stamp
            if velocity_dt + 1e-9 < self._velocity_window_s:
                # 시작 직후, 시계 초기화 또는 데이터 공백 뒤에는 관측할 수 없는 속도를
                # 최신 측정값 0으로 보고하지 않습니다.
                return None
            velocity_delta = self._body_delta(positions, old_positions)

        return OdometryEstimate(
            x_m=self._x_m,
            y_m=self._y_m,
            yaw_rad=self._yaw_rad,
            linear_x_mps=velocity_delta[0] / velocity_dt,
            linear_y_mps=velocity_delta[1] / velocity_dt,
            angular_z_rps=velocity_delta[2] / velocity_dt,
        )

    def _body_delta(
        self, current: WheelPositions, previous: WheelPositions
    ) -> tuple[float, float, float]:
        """두 휠 표본 사이의 전방·횡방향 변위와 yaw 변화량을 계산합니다."""
        front_left, front_right, rear_left, rear_right = (
            now - before
            for now, before in zip(current.as_tuple(), previous.as_tuple())
        )
        radius_scale = self._geometry.wheel_radius_m / 4.0
        return (
            radius_scale * (front_left + front_right + rear_left + rear_right),
            radius_scale * (-front_left + front_right + rear_left - rear_right),
            radius_scale
            * (-front_left + front_right - rear_left + rear_right)
            / self._geometry.center_to_wheel_sum_m * self._yaw_scale,
        )
