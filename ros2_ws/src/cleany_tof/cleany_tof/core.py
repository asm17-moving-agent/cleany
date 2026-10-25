"""Conservative ToF geometry and controllers, independent of ROS and serial IO."""
from dataclasses import dataclass
import math
import numpy as np


@dataclass(frozen=True)
class Settings:
    # These are commissioning candidates, not measured physical guarantees.
    calibrated: bool = False
    horizontal_fov: float = 1.0471975512
    vertical_fov: float = 1.0471975512
    flip_rows: bool = False
    flip_columns: bool = False
    sensor_x: float = 0.0
    sensor_y: float = 0.0
    sensor_z: float = 0.0
    sensor_yaw: float = 0.0
    sensor_pitch: float = 0.0
    sensor_roll: float = 0.0
    half_length: float = 0.48
    half_width: float = 0.40
    margin: float = 0.08
    deceleration: float = 0.3
    reaction_s: float = 0.35
    source_timeout_s: float = 0.20
    command_timeout_s: float = 0.20
    max_speed: float = 0.25
    max_yaw_rate: float = 0.3
    max_sigma_m: float = 0.03
    max_range_m: float = 4.0
    min_valid: int = 32
    required_zones: tuple[int, ...] = tuple(range(64))
    approach_rows: tuple[int, ...] = (2, 3, 4, 5)
    fit_residual_m: float = 0.025
    min_plane_span_m: float = 0.15
    max_plane_yaw_rad: float = 0.35
    approach_speed: float = 0.04
    approach_yaw_rate: float = 0.08
    distance_tolerance_m: float = 0.015
    yaw_tolerance_rad: float = 0.025
    settle_s: float = 0.6
    approach_timeout_s: float = 30.0
    progress_timeout_s: float = 5.0
    stopped_speed: float = 0.01
    stopped_yaw_rate: float = 0.02

    def __post_init__(self):
        for name, value in vars(self).items():
            if isinstance(value, float) and not math.isfinite(value):
                raise ValueError(f'{name} must be finite')
        positive = ('horizontal_fov', 'vertical_fov', 'half_length', 'half_width', 'margin',
                    'deceleration', 'reaction_s', 'source_timeout_s', 'command_timeout_s',
                    'max_speed', 'max_yaw_rate', 'max_sigma_m', 'max_range_m',
                    'fit_residual_m', 'min_plane_span_m', 'max_plane_yaw_rad',
                    'approach_speed', 'approach_yaw_rate', 'distance_tolerance_m',
                    'yaw_tolerance_rad', 'settle_s', 'approach_timeout_s', 'progress_timeout_s',
                    'stopped_speed', 'stopped_yaw_rate')
        if any(getattr(self, n) <= 0 for n in positive):
            raise ValueError('limits must be positive')
        if not 0 < self.horizontal_fov < math.pi or not 0 < self.vertical_fov < math.pi:
            raise ValueError('FOV must be below pi')
        if (not 1 <= self.min_valid <= 64 or not self.required_zones
                or any(type(i) is not int or not 0 <= i < 64 for i in self.required_zones)
                or len(set(self.required_zones)) != len(self.required_zones)
                or not self.approach_rows or any(type(i) is not int or not 0 <= i < 8 for i in self.approach_rows)):
            raise ValueError('invalid zone coverage')
        if self.approach_speed > self.max_speed or self.approach_yaw_rate > self.max_yaw_rate:
            raise ValueError('approach limits exceed guard limits')


def project(distance, sigma, status, count, valid, settings: Settings):
    """Return base-frame zone centres and a quality mask; ranges are radial metres."""
    arrays = [np.asarray(x) for x in (distance, sigma, status, count, valid)]
    if any(a.shape != (64,) for a in arrays):
        raise ValueError('expected six 64-zone arrays')
    d, s, st, n, v = arrays
    good = (v.astype(bool) & np.isin(st, (5, 9)) & (n > 0) & np.isfinite(d)
            & (d > 0) & (d <= settings.max_range_m) & np.isfinite(s)
            & (s >= 0) & (s <= settings.max_sigma_m))
    row, col = np.indices((8, 8))
    if settings.flip_rows:
        row = 7-row
    if settings.flip_columns:
        col = 7-col
    # Calibrated image columns progress towards robot right; rows down.
    y = np.tan((3.5-col.ravel()) / 8 * settings.horizontal_fov)
    z = np.tan((3.5-row.ravel()) / 8 * settings.vertical_fov)
    rays = np.stack((np.ones(64), y, z), axis=1)
    rays /= np.linalg.norm(rays, axis=1)[:, None]
    cy, sy = math.cos(settings.sensor_yaw), math.sin(settings.sensor_yaw)
    cp, sp = math.cos(settings.sensor_pitch), math.sin(settings.sensor_pitch)
    cr, sr = math.cos(settings.sensor_roll), math.sin(settings.sensor_roll)
    rotation = (np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
                @ np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
                @ np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]]))
    points = (rays * np.where(good, d, 0)[:, None]) @ rotation.T
    points += (settings.sensor_x, settings.sensor_y, settings.sensor_z)
    return points, good


def coverage(good, settings: Settings) -> bool:
    return bool(np.count_nonzero(good) >= settings.min_valid and np.all(good[list(settings.required_zones)]))


def limit_velocity(command, measured, points, good, age_s: float, settings: Settings):
    """Uniformly scale planar velocity against a conservative stopping envelope.

    Every good return contributes, including a lone obstacle outside the plane.
    A circumscribed radius bounds rotational speed. Existing LiDAR/depth guards
    retain responsibility for the ToF blind area; no ToF ray certifies that area.
    """
    zero = (0.0, 0.0, 0.0)
    if (not settings.calibrated or not coverage(good, settings) or not math.isfinite(age_s)
            or not 0 <= age_s <= settings.source_timeout_s
            or not all(math.isfinite(x) for x in (*command, *measured))):
        return zero, 'TOF_UNAVAILABLE'
    vx, vy, wz = command
    mx, my, mw = measured
    radius = math.hypot(settings.half_length, settings.half_width)
    scale = min(1., settings.max_speed/max(math.hypot(vx, vy), 1e-9),
                settings.max_yaw_rate/max(abs(wz), 1e-9))
    for x, y, _ in points[good]:
        clearance = math.hypot(max(abs(x)-settings.half_length, 0),
                               max(abs(y)-settings.half_width, 0)) - settings.margin - 3*settings.max_sigma_m
        if clearance <= 0:
            return zero, 'PROXIMITY_STOP'
        # Overbound corner/lateral closing, rather than projecting only on the
        # ray centre (which can underestimate a rectangular body's corner).
        closing = (math.hypot(vx, vy) if vx*x > 0 or vy*y > 0 else 0.) + abs(wz)*radius
        actual = (math.hypot(mx, my) if mx*x > 0 or my*y > 0 else 0.) + abs(mw)*radius
        latency = settings.reaction_s + age_s
        safe = math.sqrt((settings.deceleration*latency)**2 + 2*settings.deceleration*clearance) - settings.deceleration*latency
        if actual > safe:
            return zero, 'BRAKING_ENVELOPE'
        if closing > 0:
            scale = min(scale, safe/closing)
    return tuple(float(x*scale) for x in command), 'LIMITED' if scale < .999 else 'CLEAR'


@dataclass(frozen=True)
class Plane:
    distance: float
    yaw: float
    residual: float


def estimate_plane(points, good, settings: Settings) -> Plane:
    """Fit x = distance + slope*y using row medians in at least six columns.

    This estimates the front plane normal. A flat unmarked desk has no observable
    lateral centre; staging localization must supply lateral placement.
    """
    centres = []
    for col in range(8):
        indices = [row*8+col for row in settings.approach_rows if good[row*8+col]]
        if len(indices) >= 2:
            centres.append(np.median(points[indices, :2], axis=0))
    if len(centres) < 6:
        raise ValueError('insufficient plane columns')
    p = np.asarray(centres)
    if np.ptp(p[:, 1]) < settings.min_plane_span_m:
        raise ValueError('insufficient plane span')
    matrix = np.column_stack((np.ones(len(p)), p[:, 1]))
    distance, slope = np.linalg.lstsq(matrix, p[:, 0], rcond=None)[0]
    # Check all selected samples too: a foreground object must not disappear in medians.
    selected = [row*8+col for row in settings.approach_rows for col in range(8) if good[row*8+col]]
    residual = float(np.max(np.abs(points[selected, 0] - distance - slope*points[selected, 1])))
    yaw = math.atan(-slope)
    if residual > settings.fit_residual_m or abs(yaw) > settings.max_plane_yaw_rad or distance <= 0:
        raise ValueError('ambiguous or misaligned desk plane')
    return Plane(float(distance), yaw, residual)


class ApproachController:
    def __init__(self, target: float, now: float, settings: Settings):
        self.s = settings
        if not math.isfinite(target) or target < settings.half_length + settings.margin + 3*settings.max_sigma_m + 2*settings.distance_tolerance_m:
            raise ValueError('target enters proximity stop envelope')
        self.target, self.started = target, now
        self.settled_since = None
        self.best_error = math.inf
        self.progress_at = now

    def step(self, plane: Plane, measured, now: float):
        """Return velocity and state; success requires sustained fresh stopped data."""
        s = self.s
        if not all(math.isfinite(x) for x in (plane.distance, plane.yaw, *measured, now)):
            return (0., 0., 0.), 'INVALID'
        if now-self.started >= s.approach_timeout_s:
            return (0., 0., 0.), 'TIMEOUT'
        error = plane.distance-self.target
        if error < -s.distance_tolerance_m:
            return (0., 0., 0.), 'OVERSHOOT'
        score = abs(error) + abs(plane.yaw)*s.half_length
        if score < self.best_error - .003:
            self.best_error, self.progress_at = score, now
        aligned = abs(plane.yaw) <= s.yaw_tolerance_rad
        arrived = abs(error) <= s.distance_tolerance_m and aligned
        if arrived:
            stopped = math.hypot(*measured[:2]) <= s.stopped_speed and abs(measured[2]) <= s.stopped_yaw_rate
            if stopped:
                if self.settled_since is None:
                    self.settled_since = now
                if now-self.settled_since >= s.settle_s:
                    return (0., 0., 0.), 'SUCCEEDED'
            else:
                self.settled_since = None
            return (0., 0., 0.), 'SETTLING'
        self.settled_since = None
        if now-self.progress_at >= s.progress_timeout_s:
            return (0., 0., 0.), 'NO_PROGRESS'
        yaw = max(-s.approach_yaw_rate, min(s.approach_yaw_rate, plane.yaw*.6))
        speed = min(s.approach_speed, max(0., error*.4)) if aligned else 0.
        return (speed, 0., yaw), 'APPROACHING' if aligned else 'ALIGNING'
