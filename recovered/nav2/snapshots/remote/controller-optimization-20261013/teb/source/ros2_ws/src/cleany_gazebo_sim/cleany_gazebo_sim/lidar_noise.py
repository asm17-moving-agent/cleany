from __future__ import annotations

from dataclasses import dataclass
from math import isfinite
from pathlib import Path

import yaml


@dataclass(frozen=True)
class LidarNoiseProfile:
    name: str
    description: str
    mean: float
    stddev: float


def load_lidar_noise_profiles(path: Path) -> dict[str, LidarNoiseProfile]:
    raw = yaml.safe_load(path.read_text(encoding='utf-8'))
    if not isinstance(raw, dict) or raw.get('schema_version') != 1:
        raise ValueError('unsupported LiDAR noise profile schema')
    entries = raw.get('profiles')
    if not isinstance(entries, dict) or not entries:
        raise ValueError('LiDAR noise profiles must be a non-empty mapping')

    profiles: dict[str, LidarNoiseProfile] = {}
    for name, values in entries.items():
        if not isinstance(name, str) or not isinstance(values, dict):
            raise ValueError('LiDAR noise profile entries must be mappings')
        description = values.get('description')
        mean = values.get('mean')
        stddev = values.get('stddev')
        if not isinstance(description, str) or not description.strip():
            raise ValueError(f'profile {name!r} needs a description')
        if not isinstance(mean, (int, float)) or not isfinite(mean):
            raise ValueError(f'profile {name!r} mean must be finite')
        if not isinstance(stddev, (int, float)) or not isfinite(stddev):
            raise ValueError(f'profile {name!r} stddev must be finite')
        if stddev < 0:
            raise ValueError(f'profile {name!r} stddev must be non-negative')
        profiles[name] = LidarNoiseProfile(
            name=name,
            description=description.strip(),
            mean=float(mean),
            stddev=float(stddev),
        )
    return profiles


def load_lidar_noise_profile(path: Path, name: str) -> LidarNoiseProfile:
    profiles = load_lidar_noise_profiles(path)
    try:
        return profiles[name]
    except KeyError as error:
        available = ', '.join(sorted(profiles))
        raise ValueError(
            f'unknown LiDAR noise profile {name!r}; available: {available}'
        ) from error
