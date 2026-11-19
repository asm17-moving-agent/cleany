"""조건이 없으면 거부하는 전제 검사이며 보이지 않는 공간의 안전 보장이 아닙니다."""
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
