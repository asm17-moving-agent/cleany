"""지속적인 map 좌표 격자에 센서별로 분리해 보관하는 범위 제한 voxel 근거입니다."""
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
        # 성긴 타임스탬프와 미확인 인덱스로 두 번째 조밀한 3D float 격자를 피합니다.
        self.occupied_at: list[dict[int, float]] = [{}, {}]
        self.expired: list[set[int]] = [set(), set()]

    def rebind_map(self, geometry: MemoryGeometry) -> None:
        """관측 장애물을 잊지 않고 정적 지도 식별 정보를 갱신합니다.

        원점, 해상도나 범위가 바뀌면 기존 voxel 인덱스를 재사용할 수 없습니다.
        호출자는 근거를 조용히 재해석하지 말고 mapping을 멈춰야 합니다.
        """
        old, new = asdict(self.geometry), asdict(geometry)
        old.pop('map_id');new.pop('map_id')
        if old != new:
            raise ValueError('Map geometry changed; restart mapping with the new frame geometry')
        self.geometry = geometry

    def indices(self, points: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        g = self.geometry
        ijk = np.floor((points-[g.origin_x, g.origin_y, g.min_z])/[g.resolution, g.resolution, g.z_resolution]).astype(np.int64)
        valid = ((ijk >= 0) & (ijk < [g.width, g.height, g.shape[0]])).all(axis=1)
        xyz = ijk[valid]
        return (xyz[:, 2]*g.height+xyz[:, 1])*g.width+xyz[:, 0], valid

    def observe(self, source: str, sensor: np.ndarray, endpoints: np.ndarray, hits: np.ndarray, timestamp: float,
                all_hit_endpoints: np.ndarray | None = None) -> None:
        """voxel 반 간격으로 ray를 표본화하며 한 프레임에서는 hit가 빈 공간 ray보다 우선합니다.

        이 소스의 ray가 통과한 3D voxel만 지웁니다. FOV 전체나 투영 바닥을 일괄
        제거하지 않습니다. 만료된 근거는 같은 소스가 정확히 그 voxel을 재관측할 때까지
        미확인으로 남습니다.
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
        # 잘못되거나 범위 없는 ray가 무제한 할당을 유발해서는 안 됩니다.
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
        timestamps, expired = self.occupied_at[channel], self.expired[channel]
        expired.difference_update(map(int, free))
        expired.difference_update(map(int, occupied))
        for index in free[scores[free] <= 0]:
            timestamps.pop(int(index), None)
        timestamps.update((int(index), timestamp) for index in occupied)
        xy = np.unique(np.concatenate((free, occupied)) % (g.width*g.height))
        self.last_seen[channel].reshape(-1)[xy] = timestamp
        occupied_xy = np.unique(occupied % (g.width*g.height))
        counts = self.hit_count[channel].reshape(-1)
        counts[occupied_xy] = np.minimum(counts[occupied_xy].astype(np.uint64)+1, np.iinfo(np.uint32).max)

    def expire(self, now: float, ttl_s: float) -> int:
        """점유 voxel을 개별 만료시키며 다른 소스의 근거는 보존합니다.

        0이면 만료를 끕니다. 미확인 voxel은 원래 센서의 ray가 통과하거나 다시
        표시할 때까지 충돌 후보로 남습니다.
        """
        if not np.isfinite(now) or not np.isfinite(ttl_s) or ttl_s < 0:
            raise ValueError('Invalid memory expiry time')
        if ttl_s == 0:
            return 0
        count = 0
        for channel, timestamps in enumerate(self.occupied_at):
            old = [i for i, stamp in timestamps.items() if now-stamp >= ttl_s or stamp > now]
            self.expired[channel].update(old)
            self.evidence[channel].reshape(-1)[old] = 0
            for i in old:
                del timestamps[i]
            # 빈 공간 정보도 만료되지만 충돌 장애물은 아닙니다.
            old_xy = (self.last_seen[channel] > 0) & (
                (now-self.last_seen[channel] >= ttl_s) | (self.last_seen[channel] > now))
            self.last_seen[channel][old_xy] = 0
            count += len(old)
        return count

    def _height_slice(self, min_z: float | None, max_z: float | None) -> slice:
        g = self.geometry
        low = g.min_z if min_z is None else min_z
        high = g.max_z if max_z is None else max_z
        if not np.isfinite([low, high]).all() or low >= high:
            raise ValueError('Invalid projection height range')
        # 중심점뿐 아니라 교차하는 voxel 부피를 포함합니다.
        return slice(max(0, int(np.floor((low-g.min_z)/g.z_resolution))),
                     max(0, min(g.shape[0], int(np.ceil((high-g.min_z)/g.z_resolution)))))

    def grid(self, source: str | None = None, *, min_z: float | None = None,
             max_z: float | None = None, block_expired: bool = False) -> np.ndarray:
        channels = slice(None) if source is None else slice(self.sources.index(source), self.sources.index(source)+1)
        seen = np.any(self.last_seen[channels] > 0, axis=0)
        occupied = np.zeros((self.geometry.height, self.geometry.width), dtype=bool)
        unknown = np.zeros_like(occupied)
        selected = range(2) if source is None else [self.sources.index(source)]
        g = self.geometry
        zs = self._height_slice(min_z, max_z)
        plane = g.width*g.height
        for channel in selected:
            indices = np.fromiter(self.occupied_at[channel], dtype=np.int64)
            indices = indices[(indices//plane >= zs.start) & (indices//plane < zs.stop)]
            occupied.reshape(-1)[indices % plane] = True
            indices = np.fromiter(self.expired[channel], dtype=np.int64)
            indices = indices[(indices//plane >= zs.start) & (indices//plane < zs.stop)]
            unknown.reshape(-1)[indices % plane] = True
        result = np.where(occupied, 100, np.where(seen & ~unknown, 0, -1)).astype(np.int8)
        if block_expired:
            result[unknown] = 100
        return result

    def occupied_centers(self, source: str | None = None, *, center: np.ndarray | None = None,
                         radius: float | None = None, include_expired: bool = False) -> np.ndarray:
        """빠른 충돌 검사를 위해 전역 복사 없이 로컬 창만 추출합니다."""
        channels = slice(None) if source is None else slice(self.sources.index(source), self.sources.index(source)+1)
        g = self.geometry
        x0, y0, x1, y1 = 0, 0, g.width, g.height
        if center is not None:
            if radius is None or not np.isfinite(radius) or radius <= 0 or not np.isfinite(center[:2]).all():
                raise ValueError('Invalid local voxel window')
            x0 = max(0, min(g.width, int(np.floor((center[0]-radius-g.origin_x)/g.resolution))))
            y0 = max(0, min(g.height, int(np.floor((center[1]-radius-g.origin_y)/g.resolution))))
            x1 = max(x0, min(g.width, int(np.ceil((center[0]+radius-g.origin_x)/g.resolution))))
            y1 = max(y0, min(g.height, int(np.ceil((center[1]+radius-g.origin_y)/g.resolution))))
        # 성긴 점유 키를 재사용하며 추출 시 전체 부피를 검색하지 않습니다.
        indices = set()
        for channel in (range(2) if source is None else [self.sources.index(source)]):
            indices.update(self.occupied_at[channel])
            if include_expired:
                indices.update(self.expired[channel])
        flat = np.array(sorted(indices), dtype=np.int64)
        z, xy = np.divmod(flat, g.width*g.height)
        y, x = np.divmod(xy, g.width)
        inside = (x >= x0) & (x < x1) & (y >= y0) & (y < y1)
        z, y, x = z[inside], y[inside], x[inside]
        points = np.column_stack((g.origin_x+(x+.5)*g.resolution,
                                 g.origin_y+(y+.5)*g.resolution,
                                 g.min_z+(z+.5)*g.z_resolution)).astype(np.float32)
        if center is not None:
            points = points[np.linalg.norm(points[:, :2]-center[:2], axis=1) <= radius]
        return points

    def save(self, path: Path) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        temporary = path.with_suffix(path.suffix+'.tmp')
        with temporary.open('wb') as stream:
            np.savez_compressed(stream, schema=np.array(2), geometry=np.array(json.dumps(asdict(self.geometry))),
                                evidence=self.evidence, last_seen=self.last_seen, hit_count=self.hit_count,
                                **{f'expired_{c}': np.array(list(self.expired[c]), dtype=np.int64) for c in range(2)})
        temporary.replace(path)

    @classmethod
    def restore(cls, path: Path, geometry: MemoryGeometry) -> ObstacleMemory:
        with np.load(path, allow_pickle=False) as data:
            if int(data['schema']) not in (1, 2) or json.loads(str(data['geometry'])) != asdict(geometry):
                raise ValueError('Snapshot schema or map/geometry does not match')
            result = cls(geometry)
            for key in ('evidence', 'last_seen', 'hit_count'):
                expected, actual = getattr(result, key), data[key]
                if actual.shape != expected.shape or actual.dtype != expected.dtype:
                    raise ValueError(f'Invalid snapshot array: {key}')
                setattr(result, key, actual.copy())
            for c in range(2):
                # 복구한 관측은 이전 실행과 시계에 속합니다. 시작 시 최신으로 표시하지 않고
                # 재관측할 때까지 미확인으로 유지합니다.
                result.expired[c].update(map(int, np.flatnonzero(result.evidence[c] > 0)))
                if int(data['schema']) == 2:
                    indices = data[f'expired_{c}']
                    if np.any(indices < 0) or np.any(indices >= result.evidence[c].size):
                        raise ValueError('Invalid expired voxel index')
                    result.expired[c].update(map(int, indices))
            result.evidence[result.evidence > 0] = 0
            result.last_seen[:] = 0
            return result


def local_observation(sensor: np.ndarray, endpoints: np.ndarray, hits: np.ndarray,
                      all_hits: np.ndarray | None, center: np.ndarray, radius: float
                      ) -> tuple[np.ndarray, np.ndarray, np.ndarray | None]:
    """공간 제거 ray를 근처 원으로 자르며 모든 로컬 장애물 hit는 유지합니다.

    먼 끝점의 ray도 가까운 구간을 지웁니다. 통째로 버리면 사라진 가까운 장애물이
    계속 점유 상태로 남으므로 그렇게 처리하지 않습니다.
    """
    offset = sensor[:2]-center[:2]
    if np.linalg.norm(offset) >= radius:
        raise ValueError('Sensor outside local observation window')
    delta = endpoints-sensor
    a = np.sum(delta[:, :2]**2, axis=1)
    b = 2*(delta[:, :2]@offset)
    c = np.dot(offset, offset)-radius**2
    roots = np.ones(len(endpoints))
    active = a > 1e-12
    roots[active] = (-b[active]+np.sqrt(b[active]**2-4*a[active]*c))/(2*a[active])
    fraction = np.minimum(1., roots)
    local_hits = hits & (fraction >= 1.)
    clipped = sensor+fraction[:, None]*delta
    if all_hits is not None:
        all_hits = all_hits[np.linalg.norm(all_hits[:, :2]-center[:2], axis=1) <= radius]
    return clipped, local_hits, all_hits
