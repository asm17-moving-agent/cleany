"""An inline command ramp upstream of the existing collision monitor."""
from __future__ import annotations
from dataclasses import dataclass
import math


@dataclass
class BrakeRamp:
    linear_deceleration: float = .2
    angular_deceleration: float = .4
    wheel_acceleration: float = 3.
    wheel_radius: float = .0635
    wheel_arm: float = .47
    output: tuple[float, float, float] = (0., 0., 0.)

    def __post_init__(self):
        if not all(math.isfinite(v) and v > 0 for v in (self.linear_deceleration, self.angular_deceleration,
                    self.wheel_acceleration, self.wheel_radius, self.wheel_arm)):
            raise ValueError('Deceleration must be finite and positive')

    def step(self, target: tuple, brake: bool, dt: float, healthy: bool = True) -> tuple:
        if not healthy or not all(math.isfinite(v) for v in (*target, dt)) or not 0 < dt <= .1:
            self.output = (0., 0., 0.)
        elif not brake:
            self.output = tuple(target)
        else:
            x, y, yaw = self.output
            speed = math.hypot(x, y)
            rates = [(x-y-self.wheel_arm*yaw)/self.wheel_radius,
                     (x+y+self.wheel_arm*yaw)/self.wheel_radius,
                     (x+y-self.wheel_arm*yaw)/self.wheel_radius,
                     (x-y+self.wheel_arm*yaw)/self.wheel_radius]
            peak=max(map(abs,rates))
            drops=[1.]
            if speed:drops.append(self.linear_deceleration*dt/speed)
            if yaw:drops.append(self.angular_deceleration*dt/abs(yaw))
            if peak:drops.append(self.wheel_acceleration*dt/peak)
            scale=max(0.,1-min(drops))
            self.output = (x*scale, y*scale, yaw*scale)
        return self.output
