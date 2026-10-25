"""Directional ToF coverage and independently observed no-return fallback."""
from dataclasses import asdict
import hashlib
import json
import math
import numpy as np
from .core import Settings, sensor_rotation
from .observation import Observation, ZoneState


def model_id(settings: Settings) -> str:
    return hashlib.sha256(json.dumps(asdict(settings), sort_keys=True).encode()).hexdigest()


def load_settings(path: str) -> Settings:
    """Load the shared wildcard profile used by both ToF consumers."""
    import yaml
    with open(path, encoding='utf-8') as stream:
        parameters = yaml.safe_load(stream)['/**']['ros__parameters']
    legacy = {'min_valid', 'required_zones'} & parameters.keys()
    if legacy:
        raise ValueError('remove obsolete global coverage parameters: '+', '.join(sorted(legacy)))
    return Settings(**{key: value for key, value in parameters.items() if key in Settings.__dataclass_fields__})


class TofRegion:
    """Conservative cube/frustum intersection in base_link, with no range claim."""
    def __init__(self, settings: Settings):
        self.s = settings
        self.rotation = sensor_rotation(settings)
        h, v = math.tan(settings.horizontal_fov/2), math.tan(settings.vertical_fov/2)
        self.normals = np.array([[-1, 0, 0], [-h, 1, 0], [-h, -1, 0], [-v, 0, 1], [-v, 0, -1]])

    def intersects(self, centers: np.ndarray, size: float) -> np.ndarray:
        p = (centers-[self.s.sensor_x, self.s.sensor_y, self.s.sensor_z]) @ self.rotation
        support = size/2*np.abs(self.normals @ self.rotation.T).sum(axis=1)
        # A certificate may be consumed up to source_timeout_s later. Include
        # movement of the sensing frustum at the configured velocity bounds.
        travel = self.s.max_speed*self.s.source_timeout_s
        turn = self.s.max_yaw_rate*self.s.source_timeout_s
        reach = np.linalg.norm(centers, axis=1)+math.sqrt(3)*size/2
        padding = (travel+reach*turn)[:, None]*np.linalg.norm(self.normals, axis=1)
        return (p @ self.normals.T <= support+padding+1e-9).all(axis=1)


def relevant_zones(command, measured, settings: Settings) -> np.ndarray:
    """Include a zone if any beam corner approaches under either velocity.

    Rotation considers every zone. Translation uses the whole angular zone,
    rather than its invalid distance slot or only the central ray.
    """
    if abs(command[2])+abs(measured[2]) > 1e-9:
        return np.ones(64, dtype=bool)
    row, col = np.indices((8, 8))
    if settings.flip_rows:
        row = 7-row
    if settings.flip_columns:
        col = 7-col
    relevant = np.zeros(64, dtype=bool)
    for dr in (-.5, .5):
        for dc in (-.5, .5):
            rays = np.column_stack((np.ones(64),
                np.tan((3.5-col.ravel()+dc)/8*settings.horizontal_fov),
                np.tan((3.5-row.ravel()+dr)/8*settings.vertical_fov))) @ sensor_rotation(settings).T
            for velocity in (command, measured):
                for axis, (origin, extent) in enumerate(((settings.sensor_x, settings.half_length),
                                                        (settings.sensor_y, settings.half_width))):
                    speed = velocity[axis]
                    if abs(speed) > 1e-9:
                        relevant |= rays[:, axis]*speed > 1e-9
                        if math.copysign(1., speed)*origin >= extent:
                            relevant[:] = True
    return relevant


def _within_sweep(velocity, assessed) -> bool:
    v, a = np.asarray(velocity, dtype=float), np.asarray(assessed, dtype=float)
    if v.shape != (3,) or a.shape != (3,) or not np.isfinite([v, a]).all():
        return False
    if not np.any(v):
        return True
    norm = float(a @ a)
    if norm <= 1e-12:
        return False
    scale = float(v @ a)/norm
    return 0 <= scale <= 1+1e-9 and np.allclose(v, a*scale, atol=1e-8, rtol=0)


def proof_covers(proof: dict | None, command, measured, settings: Settings) -> bool:
    if not isinstance(proof, dict) or proof.get('contract') != 'tof_sweep_v1' or proof.get('model_id') != model_id(settings):
        return False
    if proof.get('covered') is not True or proof.get('reason') not in ('COVERED', 'NO_TOF_SWEEP'):
        return False
    try:
        vectors = (proof['requested'], proof['measured'])
        return all(any(_within_sweep(v, assessed) for assessed in vectors) for v in (command, measured))
    except (KeyError, TypeError, ValueError):
        return False


def limit_velocity(command, measured, observation: Observation, age_s: float,
                   settings: Settings, proof: dict | None = None):
    """Keep NO_RETURN unknown; delegate only to matching, fresh depth evidence.

    The ROS shell owns proof freshness. Unreliable relevant zones never borrow
    coverage. Every measured return, including outside the plane ROI, can stop.
    """
    zero = (0., 0., 0.)
    if (not settings.calibrated or not observation.available or not math.isfinite(age_s)
            or not 0 <= age_s <= settings.source_timeout_s
            or not all(math.isfinite(v) for v in (*command, *measured))):
        return zero, 'TOF_UNAVAILABLE'
    relevant = relevant_zones(command, measured, settings)
    if np.any(relevant & np.isin(observation.states, (ZoneState.UNRELIABLE, ZoneState.NOT_UPDATED))):
        return zero, 'UNRELIABLE_COVERAGE'
    fallback = bool(np.any(relevant & (observation.states == ZoneState.NO_RETURN)))
    if fallback and (math.hypot(*measured[:2]) > settings.max_speed+1e-9
                     or abs(measured[2]) > settings.max_yaw_rate+1e-9):
        return zero, 'NO_RETURN_MOTION_OUT_OF_BOUNDS'
    speed_limit = settings.no_return_max_speed if fallback else settings.max_speed
    yaw_limit = settings.no_return_max_yaw_rate if fallback else settings.max_yaw_rate
    scale = min(1., speed_limit/max(math.hypot(*command[:2]), 1e-9),
                yaw_limit/max(abs(command[2]), 1e-9))
    candidate = tuple(float(v*scale) for v in command)
    if fallback and not proof_covers(proof, candidate, measured, settings):
        return zero, 'NO_RETURN_UNCOVERED'
    radius = math.hypot(settings.half_length, settings.half_width)
    for x, y, _ in observation.points[observation.ranges]:
        clearance = math.hypot(max(abs(x)-settings.half_length, 0),
                               max(abs(y)-settings.half_width, 0))-settings.margin-3*settings.max_sigma_m
        if clearance <= 0:
            return zero, 'PROXIMITY_STOP'
        vx, vy, wz = command
        mx, my, mw = measured
        closing = (math.hypot(vx, vy) if vx*x > 0 or vy*y > 0 else 0.)+abs(wz)*radius
        actual = (math.hypot(mx, my) if mx*x > 0 or my*y > 0 else 0.)+abs(mw)*radius
        latency = settings.reaction_s+age_s
        safe = math.sqrt((settings.deceleration*latency)**2+2*settings.deceleration*clearance)-settings.deceleration*latency
        if actual > safe:
            return zero, 'BRAKING_ENVELOPE'
        if closing > 0:
            scale = min(scale, safe/closing)
    return tuple(float(v*scale) for v in command), ('NO_RETURN_DEPTH_COVERED' if fallback else
                                                 'LIMITED' if scale < .999 else 'CLEAR')
