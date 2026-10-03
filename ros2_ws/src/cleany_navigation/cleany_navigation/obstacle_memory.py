"""Bounded, sensor-separated voxel evidence with a persistent map-frame grid."""
from __future__ import annotations

from dataclasses import asdict, dataclass
import json
from pathlib import Path

import numpy as np


@dataclass(frozen=True)
class MemoryGeometry:
    width: int
    height: int
    resolution: float
    origin_x: float
    origin_y: float
    min_z: float
    max_z: float
    z_resolution: float
    map_id: str

    def __post_init__(self):
        if min(self.width, self.height, self.resolution, self.z_resolution) <= 0 or self.max_z <= self.min_z:
            raise ValueError('Invalid memory grid geometry')

    @property
    def shape(self) -> tuple[int, int, int]:
        return (int(np.ceil((self.max_z-self.min_z)/self.z_resolution)), self.height, self.width)


class ObstacleMemory:
    sources = ('lidar', 'depth')

    def __init__(self, geometry: MemoryGeometry):
        self.geometry = geometry
        self.evidence = np.zeros((2, *geometry.shape), dtype=np.int8)
        self.last_seen = np.zeros((2, geometry.height, geometry.width), dtype=np.float64)
        self.hit_count = np.zeros_like(self.last_seen, dtype=np.uint32)

    def indices(self, points: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        g = self.geometry
        ijk = np.floor((points-[g.origin_x, g.origin_y, g.min_z])/[g.resolution, g.resolution, g.z_resolution]).astype(np.int64)
        valid = ((ijk >= 0) & (ijk < [g.width, g.height, g.shape[0]])).all(axis=1)
        xyz = ijk[valid]
        return (xyz[:, 2]*g.height+xyz[:, 1])*g.width+xyz[:, 0], valid

    def observe(self, source: str, sensor: np.ndarray, endpoints: np.ndarray, hits: np.ndarray, timestamp: float,
                all_hit_endpoints: np.ndarray | None = None) -> None:
        """Ray sampling at half a voxel; hits win over free rays in one frame.

        Only this source's traversed 3-D voxels are cleared. No FOV-wide or
        projected-floor clearing, and no automatic deletion merely due to age.
        """
        channel = self.sources.index(source)
        endpoints = np.asarray(endpoints, dtype=float).reshape(-1, 3)
        hits = np.asarray(hits, dtype=bool)
        if len(endpoints) != len(hits) or not np.isfinite(sensor).all() or not np.isfinite(timestamp) or timestamp <= 0:
            raise ValueError('Invalid observation')
        valid = np.isfinite(endpoints).all(axis=1)
        endpoints, hits = endpoints[valid], hits[valid]
        if not len(endpoints):
            return
        g = self.geometry
        delta = endpoints-sensor
        steps = np.maximum(1, np.ceil(np.max(np.abs(delta)/[g.resolution, g.resolution, g.z_resolution], axis=1)*2).astype(int))
        # A malformed/unbounded ray must not cause unbounded allocation.
        if steps.max() > 4096:
            raise ValueError('Observation ray exceeds bounded workspace')
        samples = np.arange(int(steps.max())+1)
        fraction = samples[None, :]/steps[:, None]
        mask = samples[None, :] < (steps+~hits)[:, None]
        rays = (sensor+fraction[:, :, None]*delta[:, None, :])[mask]
        free, _ = self.indices(rays)
        mark_points = endpoints[hits] if all_hit_endpoints is None else np.asarray(all_hit_endpoints, dtype=float).reshape(-1, 3)
        mark_points = mark_points[np.isfinite(mark_points).all(axis=1)]
        occupied, _ = self.indices(mark_points)
        free, occupied = np.unique(free), np.unique(occupied)
        free = np.setdiff1d(free, occupied, assume_unique=True)
        scores = self.evidence[channel].reshape(-1)
        scores[free] = np.maximum(-3, scores[free]-1)
        scores[occupied] = np.minimum(5, np.maximum(scores[occupied], 0)+2)
        xy = np.unique(np.concatenate((free, occupied)) % (g.width*g.height))
        self.last_seen[channel].reshape(-1)[xy] = timestamp
        occupied_xy = np.unique(occupied % (g.width*g.height))
        counts = self.hit_count[channel].reshape(-1)
        counts[occupied_xy] = np.minimum(counts[occupied_xy].astype(np.uint64)+1, np.iinfo(np.uint32).max)

    def grid(self, source: str | None = None) -> np.ndarray:
        channels = [self.sources.index(source)] if source else [0, 1]
        seen = np.any(self.last_seen[channels] > 0, axis=0)
        occupied = np.any(self.evidence[channels] > 0, axis=(0, 1))
        return np.where(occupied, 100, np.where(seen, 0, -1)).astype(np.int8)

    def occupied_centers(self, source: str | None = None) -> np.ndarray:
        """Return occupied 3-D voxel centers; free evidence from another source cannot erase them."""
        channels = [self.sources.index(source)] if source else [0, 1]
        z, y, x = np.nonzero(np.any(self.evidence[channels] > 0, axis=0))
        g = self.geometry
        return np.column_stack((g.origin_x+(x+.5)*g.resolution,
                                g.origin_y+(y+.5)*g.resolution,
                                g.min_z+(z+.5)*g.z_resolution)).astype(np.float32)

    def save(self, path: Path) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        temporary = path.with_suffix(path.suffix+'.tmp')
        with temporary.open('wb') as stream:
            np.savez_compressed(stream, schema=np.array(1), geometry=np.array(json.dumps(asdict(self.geometry))),
                                evidence=self.evidence, last_seen=self.last_seen, hit_count=self.hit_count)
        temporary.replace(path)

    @classmethod
    def restore(cls, path: Path, geometry: MemoryGeometry) -> ObstacleMemory:
        with np.load(path, allow_pickle=False) as data:
            if int(data['schema']) != 1 or json.loads(str(data['geometry'])) != asdict(geometry):
                raise ValueError('Snapshot schema or map/geometry does not match')
            result = cls(geometry)
            for key in ('evidence', 'last_seen', 'hit_count'):
                expected, actual = getattr(result, key), data[key]
                if actual.shape != expected.shape or actual.dtype != expected.dtype:
                    raise ValueError(f'Invalid snapshot array: {key}')
                setattr(result, key, actual.copy())
            return result
