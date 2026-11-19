"""임시 고정 베이스 장면 복사본에만 되돌릴 수 있는 최적화를 적용합니다."""
from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import yaml


PERFORMANCE_PROFILES = ('baseline', 'tabletop_collision', 'tabletop_fast')
_BACKGROUND_PREFIXES = ('demo_desk_', 'desk_monitor_', 'desk_partition_', 'wall_')


@dataclass(frozen=True)
class TabletopPerformance:
    collision_keep_radius_m: float
    shadow_map_size: int


def load_performance_profile(path: Path, name: str) -> TabletopPerformance | None:
    if name not in PERFORMANCE_PROFILES:
        raise ValueError(f'Unknown simulation performance profile: {name}')
    if name == 'baseline':
        return None
    config = yaml.safe_load(path.read_text(encoding='utf-8'))
    if not isinstance(config, dict) or config.get('schema_version') != 1:
        raise ValueError('Unsupported tabletop performance config')
    raw = config['profiles'][name]
    radius = raw['collision_keep_radius_m']
    shadow = raw['shadow_map_size']
    # 반경을 줄이려면 별도 도달 가능성 검토가 필요하며 임의로 조정하지 않습니다.
    if (isinstance(radius, bool) or not isinstance(radius, (int, float))
            or not math.isfinite(radius) or radius < 2.0):
        raise ValueError('Collision keep radius must be finite and at least 2 m')
    if type(shadow) is not int or shadow not in (2048, 4096):
        raise ValueError('Shadow map size must be 2048 or 4096')
    return TabletopPerformance(float(radius), shadow)


def _vector(element: ET.Element, key: str, default: str) -> tuple[float, ...]:
    values = tuple(float(v) for v in element.get(key, default).split())
    if not values or not all(math.isfinite(v) for v in values):
        raise ValueError(f'Invalid {key} in background geometry')
    return values


def _static_body_radius(body: ET.Element) -> float | None:
    """회전에 불변인 보수적 경계이며 알려지지 않았거나 중첩된 body는 건너뜁니다."""
    if body.get('mocap', 'false') != 'false' or any(
            body.find(tag) is not None for tag in ('joint', 'freejoint', 'body')):
        return None
    bounds = []
    for geom in body.findall('geom'):
        if geom.get('type') not in ('box', 'cylinder', 'sphere', 'capsule'):
            return None
        if geom.get('fromto') is not None:
            return None
        position = _vector(geom, 'pos', '0 0 0')
        size = _vector(geom, 'size', '')
        if len(position) != 3 or not 1 <= len(size) <= 3 or min(size) <= 0:
            raise ValueError('Invalid background primitive dimensions')
        # 합은 capsule의 반구 끝도 포함하며 의도적으로 여유 있게 잡습니다.
        bounds.append(math.sqrt(sum(v*v for v in position)) + sum(size))
    return max(bounds) if bounds else None


def apply_tabletop_performance(
    scene_text: str, model_text: str, profile: TabletopPerformance | None,
) -> tuple[str, str, tuple[str, ...]]:
    if profile is None:
        return scene_text, model_text, ()
    scene, robot = ET.fromstring(scene_text), ET.fromstring(model_text)
    chassis = robot.find("./worldbody/body[@name='chassis']")
    weld = scene.find("./equality/weld[@name='study_cafe_grasp_chassis_world_weld']")
    if (scene.get('model') != 'cleany_study_cafe_grasp_execution' or chassis is None
            or weld is None or weld.get('body1') != 'chassis'
            or weld.get('body2', 'world') != 'world'
            or weld.get('active', 'true') != 'true'):
        raise ValueError('Tabletop optimization requires the fixed-base study-cafe grasp scene')
    origin = _vector(chassis, 'pos', '0 0 0')
    if len(origin) != 3:
        raise ValueError('Chassis position requires three values')
    # 명시적 접촉 쌍은 mask를 우회하므로 해당 쌍에서 사용하는 모든 geom을 보존합니다.
    paired = {pair.get(key) for root in (scene, robot)
              for pair in root.findall('./contact/pair') for key in ('geom1', 'geom2')}
    disabled: list[str] = []
    for body in scene.findall('./worldbody/body'):
        if not body.get('name', '').startswith(_BACKGROUND_PREFIXES):
            continue
        bound = _static_body_radius(body)
        position = _vector(body, 'pos', '0 0 0')
        if len(position) != 3:
            raise ValueError('Background body position requires three values')
        separation = math.hypot(position[0]-origin[0], position[1]-origin[1])
        if bound is None or separation - bound <= profile.collision_keep_radius_m:
            continue
        for geom in body.findall('geom'):
            name = geom.get('name', '')
            if not name or name in paired:
                continue
            if geom.get('contype', '1') == geom.get('conaffinity', '1') == '0':
                continue
            geom.set('contype', '0')
            geom.set('conaffinity', '0')
            disabled.append(name)
    quality = scene.find('./visual/quality')
    if quality is None:
        raise ValueError('Study-cafe scene requires visual quality settings')
    quality.set('shadowsize', str(profile.shadow_map_size))
    # 로봇, 접촉 쌍, 질량, 마찰, solver와 카메라 계약은 변경하지 않습니다.
    return ET.tostring(scene, encoding='unicode'), model_text, tuple(disabled)
