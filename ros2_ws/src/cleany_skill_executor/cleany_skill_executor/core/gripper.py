"""순수 그리퍼 벌림과 물리 접촉 보조 함수입니다."""

from __future__ import annotations

import math
import re


def approach_offset_for_label(label: str, base: float, labels: list[str], extra: float | list[float]) -> float:
    """명시적으로 설정한 물체 유형별 삽입 보정을 적용합니다."""
    offsets = extra if isinstance(extra, list) else [extra] * len(labels)
    if (len(offsets) != len(labels) or not math.isfinite(base)
            or any(not math.isfinite(value) or not 0 <= value <= .02 for value in offsets)):
        raise ValueError('Approach correction must be finite and within 0..0.02 m')
    normalized = [value.strip().casefold() for value in labels]
    if len(set(normalized)) != len(normalized):
        raise ValueError('Approach correction labels must be unique')
    detailed_label = label.strip().casefold()
    corrections = dict(zip(normalized, offsets))
    if detailed_label in corrections:
        return base + corrections[detailed_label]
    # Gemini 상세 label에 설정된 YOLOE 분류 구절이 포함될 수 있습니다.
    # 가장 긴 전체 구절을 사용해 mouse보다 computer mouse를 우선합니다.
    matches = [name for name in normalized if name and re.search(
        rf'(?<!\w){re.escape(name)}(?!\w)', detailed_label)]
    if not matches:
        return base
    return base + corrections[max(matches, key=len)]


def aperture_centering_offset(required_opening_m: float, margin_m: float,
                              fixed_inner_x_m: float, clearance_m: float = 0.0) -> float:
    """고정 손가락 도구 보정으로 실제 폭을 양 손가락 사이에 놓습니다."""
    if (not all(math.isfinite(v) for v in (required_opening_m,margin_m,fixed_inner_x_m,clearance_m))
            or required_opening_m <= 0 or not 0 <= margin_m < required_opening_m
            or not 0 <= clearance_m <= margin_m/2):
        raise ValueError('Invalid aperture centering dimensions')
    return (required_opening_m-margin_m)/2-fixed_inner_x_m+clearance_m


def aligned_wrist_rolls(*, approach: tuple[float, float, float],
                       closing: tuple[float, float, float],
                       desired: tuple[float, float, float], current: float,
                       lower: float, upper: float, sign_invariant: bool) -> tuple[float, ...]:
    """roll과 TCP 축이 일직선인 도구를 회전하며 호출자가 FK와 충돌을 다시 검사해야 합니다."""
    if not all(math.isfinite(x) for x in (*approach, *closing, *desired, current, lower, upper)) or lower >= upper:
        raise ValueError('wrist alignment needs finite vectors and ordered limits')
    norm = math.sqrt(sum(x*x for x in approach))
    if norm < 1e-9:
        raise ValueError('wrist alignment axis is zero')
    axis = tuple(x/norm for x in approach)

    def projected(vector):
        dot = sum(x*y for x, y in zip(vector, axis, strict=True))
        result = tuple(x-dot*y for x, y in zip(vector, axis, strict=True))
        length = math.sqrt(sum(x*x for x in result))
        return tuple(x/length for x in result) if length > 1e-9 else None

    source, target = projected(closing), projected(desired)
    if source is None or target is None:
        return ()
    cross = (source[1]*target[2]-source[2]*target[1], source[2]*target[0]-source[0]*target[2],
             source[0]*target[1]-source[1]*target[0])
    angle = math.atan2(sum(x*y for x, y in zip(axis, cross, strict=True)),
                       sum(x*y for x, y in zip(source, target, strict=True)))
    candidates = []
    for reversal in ((0., math.pi) if sign_invariant else (0.,)):
        value = current+angle+reversal
        first, last = math.ceil((lower-value)/math.tau), math.floor((upper-value)/math.tau)
        candidates.extend(value+k*math.tau for k in range(first, last+1))
    return tuple(sorted(candidates, key=lambda x: abs(x-current)))


def opening_to_gripper_position(
    *,
    required_opening_m: float,
    opening_reduction_m: float,
    reference_aperture_m: float,
    reference_position_rad: float,
    aperture_m_per_rad: float,
    minimum_position_rad: float,
    maximum_position_rad: float,
) -> float:
    values = (
        required_opening_m,
        opening_reduction_m,
        reference_aperture_m,
        reference_position_rad,
        aperture_m_per_rad,
        minimum_position_rad,
        maximum_position_rad,
    )
    if not all(math.isfinite(value) for value in values):
        raise ValueError('gripper aperture calibration must be finite')
    if required_opening_m <= 0.0 or opening_reduction_m < 0.0:
        raise ValueError('gripper opening values must be positive')
    if reference_aperture_m <= 0.0 or aperture_m_per_rad <= 0.0:
        raise ValueError('gripper aperture calibration must be positive')
    if minimum_position_rad > maximum_position_rad:
        raise ValueError('gripper position limits are reversed')
    contact_aperture = max(0.0, required_opening_m - opening_reduction_m)
    position = reference_position_rad + (
        contact_aperture - reference_aperture_m
    ) / aperture_m_per_rad
    return min(maximum_position_rad, max(minimum_position_rad, position))


def is_gripper_contact_stall(
    *,
    start: float,
    actual: float,
    command: float,
    velocity: float,
    minimum_motion: float,
    minimum_residual: float,
    maximum_velocity: float,
    allow_closing_motion: bool = False,
) -> bool:
    """닫힘 움직임과 남은 위치 오차로 접촉을 분류합니다.

    위치와 이동 임계값은 라디안, 속도는 rad/s입니다. 닫히면 위치가 감소합니다.
    무효하거나 유한하지 않은 입력에는 False를 반환합니다. allow_closing_motion은
    이미 잡힌 물체의 정지 판정을 완화하므로 호출자가 피드백 최신성과 시간상
    지속성을 별도로 확인해야 합니다.
    """
    values = (
        start,
        actual,
        command,
        velocity,
        minimum_motion,
        minimum_residual,
        maximum_velocity,
    )
    if not all(math.isfinite(value) for value in values):
        return False
    if minimum_motion <= 0.0 or minimum_residual <= 0.0:
        return False
    if maximum_velocity < 0.0 or command >= start:
        return False
    motion = start - actual
    residual = actual - command
    return (
        motion >= minimum_motion
        and residual >= minimum_residual
        # 파지가 확인된 뒤에는 운반 중에도 계속 조일 수 있습니다. 남은 오차가 사라지면
        # 빈 채 닫힌 그리퍼로 거부하며 열리는 움직임도 계속 제한합니다.
        # 최초 파지에는 반드시 실제 정지가 필요합니다.
        and (velocity <= maximum_velocity if allow_closing_motion
             else abs(velocity) <= maximum_velocity)
    )
