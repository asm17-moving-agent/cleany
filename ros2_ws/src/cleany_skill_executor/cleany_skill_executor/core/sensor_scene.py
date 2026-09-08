"""Fail-closed prerequisites, not a guarantee of unseen-space safety."""
import math


def validate_sensor_scene(
    cloud_age_sec: float,
    maximum_age_sec: float,
    octomap_id: str,
    octomap_bytes: int,
) -> None:
    if not math.isfinite(maximum_age_sec) or maximum_age_sec <= 0:
        raise ValueError('Scene maximum age must be finite and positive')
    if (not math.isfinite(cloud_age_sec)
            or not 0 <= cloud_age_sec <= maximum_age_sec):
        raise RuntimeError('Scene has no fresh self-filtered depth cloud')
    if octomap_id != 'OcTree' or octomap_bytes <= 0:
        raise RuntimeError('MoveIt has no populated sensor OctoMap')
