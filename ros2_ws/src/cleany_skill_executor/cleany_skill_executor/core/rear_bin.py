"""CAD opening geometry and route A transport stations in base_link."""
from dataclasses import dataclass
from hashlib import sha256
import json
import math
from pathlib import Path

import numpy as np

from .hardware_execution import ArmHold
from .recorded_trajectory import CommandPoint, RecordedTrajectory


REAR_BIN = 'rear_bin_a'
STATIONS = ('lift_complete', 'sweep_out', 'side_clearance', 'rear_sweep', 'rim_entry', 'release')
TRAJECTORY_STAGES = ('rear_bin_transport', 'rear_bin_retreat', 'rear_bin_park', 'rear_bin_standby')


def rear_bin_mode(config: dict) -> bool:
    return config.get('operation_mode') == 'pick_rear_bin_release'


def _vector(value, length):
    if (not isinstance(value, list) or len(value) != length
            or any(type(x) not in (int, float) or not math.isfinite(x) for x in value)):
        raise ValueError(f'Rear route requires {length} finite coordinates')
    return tuple(float(x) for x in value)


@dataclass(frozen=True)
class RearBinRoute:
    stations: tuple[tuple[float, ...], ...]
    park: tuple[float, ...]
    release_xyz: tuple[float, ...]
    opening_x: tuple[float, ...]
    opening_y: tuple[float, ...]
    rim_z: float
    tolerance: float
    return_side_clearance: tuple[float, ...] | None = None

    @classmethod
    def from_config(cls, config: dict, arm: str) -> 'RearBinRoute':
        geometry = config['rear_bin_a']
        route = geometry['arms'][arm]
        stations = tuple(_vector(route['stations_rad'][name], 4) for name in STATIONS)
        park = _vector(route['park_joint_rad'], 5)
        return_side = (_vector(route['return_side_clearance_rad'], 4)
                       if 'return_side_clearance_rad' in route else None)
        release = _vector(route['release_tcp_m'], 3)
        x = _vector(geometry['opening_x_m'], 2)
        y = _vector(geometry['opening_y_m'], 2)
        rim, tolerance = _vector([geometry['rim_z_m'], geometry['arrival_tolerance_m']], 2)
        if not (x[0] < release[0] < x[1] and y[0] < release[1] < y[1]
                and rim < release[2] and 0 < tolerance < release[2] - rim):
            raise ValueError('Rear release target must be above and inside the opening')
        return cls(stations, park, release, x, y, rim, tolerance, return_side)

    def outbound(self, roll: float) -> tuple[tuple[float, ...], ...]:
        """Preserve the selected grasp roll during transport and open-jaw retreat."""
        return tuple((*q, roll) for q in self.stations)

    def retreat(self, roll: float) -> tuple[tuple[float, ...], ...]:
        """Retreat from the release pose through the rim and side with the jaw open."""
        side, rear, rim = self.stations[2:-1]
        if self.return_side_clearance is not None:
            side = self.return_side_clearance
        return ((*rim, roll), (*rear, roll), (*side, roll))

    def check_payload(self, tcp, center, half_extents) -> None:
        """Check horizontal TCP alignment and that the payload clears the CAD rim."""
        if not np.isfinite([tcp, center, half_extents]).all() or not np.all(half_extents > 0):
            raise ValueError('Missing finite rear release geometry')
        if math.dist(tcp[:2], self.release_xyz[:2]) > self.tolerance:
            raise ValueError('Measured TCP is outside the rear release horizontal tolerance')
        low, high = center - half_extents, center + half_extents
        if not (self.opening_x[0] < low[0] and high[0] < self.opening_x[1]
                and self.opening_y[0] < low[1] and high[1] < self.opening_y[1]
                and low[2] > self.rim_z):
            raise ValueError('Attached payload does not clear the rear opening')


@dataclass(frozen=True)
class RearBinTrajectories:
    """Cached waypoint cubics. Dispatch still verifies current hardware and collisions."""

    model_sha256: str
    paths: dict[str, dict[str, RecordedTrajectory]]

    @classmethod
    def load(cls, filename: str) -> 'RearBinTrajectories':
        document = json.loads(Path(filename).read_text())
        if document['schema_version'] != 1:
            raise ValueError('Unsupported rear-bin trajectory schema')
        source = document['source_files']
        model_hash = source.get('robot_control_urdf', source['robot_urdf'])['sha256']
        if (not isinstance(model_hash, str) or len(model_hash) != 64
                or any(c not in '0123456789abcdef' for c in model_hash)):
            raise ValueError('Rear-bin trajectories require a robot model checksum')
        paths = {}
        for arm in ('left', 'right'):
            stages = document['arms'][arm]['stages']
            return_stages = ('rear_bin_return',) if 'rear_bin_return' in stages else TRAJECTORY_STAGES[1:]
            paths[arm] = {}
            for stage in (TRAJECTORY_STAGES[0], *return_stages):
                trajectory = stages[stage]['trajectory']
                path = RecordedTrajectory(tuple(trajectory['joint_names']), tuple(
                    CommandPoint(point['time_from_start']['sec'] * 10**9 + point['time_from_start']['nanosec'],
                                 tuple(point['positions']), tuple(point['velocities']))
                    for point in trajectory['points']))
                path.validate()
                if any(abs(v) > 1e-9 for point in (path.points[0], path.points[-1]) for v in point.velocities):
                    raise ValueError('Rear-bin trajectory endpoints must be stationary')
                paths[arm][stage] = path
        return cls(model_hash, paths)

    def for_stage(self, arm: str, stage: str, names: tuple[str, ...], targets: tuple,
                  measured: tuple[float, ...], hold: ArmHold, description: str, *,
                  start_tolerance: float, arrival_tolerance: float, maximum_bias: float) -> RecordedTrajectory | None:
        """Bind the held wrist roll and previous command to a compatible cached path."""
        path = self.paths.get(arm, {}).get(stage)
        if (path is None or path.joint_names != names or len(path.points) != len(targets) + 1
                or sha256(description.encode()).hexdigest() != self.model_sha256):
            return None
        points = path.points
        if stage in ('rear_bin_transport', 'rear_bin_retreat', 'rear_bin_return'):
            # Continuous return holds the grasp roll through the side waypoint.
            # The last two waypoints transition to the configured park and home rolls.
            held_points = points[:-2] if stage == 'rear_bin_return' else points
            saved_roll = points[0].positions[-1]
            if any(point.positions[-1] != saved_roll or point.velocities[-1] != 0 for point in held_points):
                return None
            roll = targets[0][-1]
            points = tuple(CommandPoint(point.time_ns, (*point.positions[:-1], roll), point.velocities)
                           for point in held_points) + points[len(held_points):]
        if tuple(point.positions for point in points[1:]) != targets:
            return None
        anchor = points[0].positions
        if stage == 'rear_bin_park':
            anchor = (*anchor[:-1], hold.nominal[-1])
        if (any(abs(a - b) > start_tolerance for a, b in zip(anchor, hold.nominal, strict=True))
                or any(abs(a - b) > maximum_bias for a, b in zip(hold.nominal, hold.command, strict=True))
                or any(abs(a - b) > arrival_tolerance for a, b in zip(measured, hold.command, strict=True))):
            return None
        # Keep the existing actuator setpoint at the transition. Hardware replay
        # retimes this cubic and validates the measured-to-held bridge as well.
        first = CommandPoint(0, hold.command, points[0].velocities)
        return RecordedTrajectory(names, (first, *points[1:]))
