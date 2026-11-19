"""GUI와 RGB-D 카메라 렌더링이 공유하는 스터디카페 한정 조명입니다."""
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import yaml


def _vector(raw: object, *, color: bool = False) -> str:
    if not isinstance(raw, (list, tuple)) or len(raw) != 3:
        raise ValueError('Lighting vectors require three numbers')
    try:
        values = tuple(float(v) for v in raw)
    except (TypeError, ValueError) as error:
        raise ValueError('Lighting vectors require numeric values') from error
    if not all(math.isfinite(v) and (not color or 0 <= v <= 1) for v in values):
        raise ValueError('Lighting vectors must be finite; colors must be in [0, 1]')
    return ' '.join(str(v) for v in values)


def apply_study_cafe_lighting(scene: ET.Element, robot: ET.Element, path: Path) -> None:
    config = yaml.safe_load(path.read_text())
    if not isinstance(config, dict) or config.get('schema_version') != 1:
        raise ValueError('Unsupported study-cafe lighting config')
    size = config['shadow_map_size']
    scale = float(config['shadow_scale'])
    if isinstance(size, bool) or not isinstance(size, int) or not 128 <= size <= 8192:
        raise ValueError('Shadow map size must be an integer in [128, 8192]')
    if not math.isfinite(scale) or not 0 < scale <= 1:
        raise ValueError('Shadow scale must be in (0, 1]')
    visual = scene.find('visual')
    world = scene.find('worldbody')
    robot_world = robot.find('worldbody')
    if visual is None or world is None or robot_world is None:
        raise ValueError('Study-cafe lighting requires visual and worldbody sections')
    headlight = visual.find('headlight')
    if headlight is None:
        headlight = ET.SubElement(visual, 'headlight')
    headlight.set('active', '1')
    for channel in ('ambient', 'diffuse', 'specular'):
        headlight.set(channel, _vector(config['headlight'][channel], color=True))
    quality = visual.find('quality')
    if quality is None:
        quality = ET.SubElement(visual, 'quality')
    quality.set('shadowsize', str(size))
    map_ = visual.find('map')
    if map_ is None:
        map_ = ET.SubElement(visual, 'map')
    map_.set('shadowscale', str(scale))
    statistic = scene.find('statistic')
    if statistic is None or 'extent' not in statistic.attrib:
        raise ValueError('Room lighting requires an explicit scene extent')
    extent = float(statistic.attrib['extent'])
    half_extent = float(config['shadow_clip_half_extent_m'])
    if not all(math.isfinite(v) and v > 0 for v in (extent, half_extent)):
        raise ValueError('Shadow coverage and scene extent must be positive and finite')
    # MuJoCo는 장면 범위로 방향성 그림자 절단면을 조정합니다.
    # 근접 조작 장면과 방 viewer에 같은 조명 범위가 필요합니다.
    map_.set('shadowclip', str(half_extent / extent))
    # 구체화된 로봇 복사본의 일반 월드 조명만 제거합니다.
    # 기준 로봇 MJCF와 다른 장면은 유지합니다.
    for light in list(robot_world.findall('light')):
        robot_world.remove(light)
    names = set()
    for raw in config['lights']:
        name = raw['name']
        if not isinstance(name, str) or not name.isidentifier() or name in names:
            raise ValueError('Lights require unique identifier names')
        names.add(name)
        direction = _vector(raw['direction'])
        if not any(float(v) for v in direction.split()):
            raise ValueError('Light direction must be nonzero')
        cutoff = float(raw['cutoff_degrees'])
        if not math.isfinite(cutoff) or not 0 < cutoff <= 80:
            raise ValueError('Light cutoff must be in (0, 80] degrees')
        exponent = float(raw.get('exponent', 10.0))
        if not math.isfinite(exponent) or not 0 <= exponent <= 128:
            raise ValueError('Light exponent must be in [0, 128]')
        if any(not isinstance(raw[key], bool) for key in ('directional', 'cast_shadow')):
            raise ValueError('Light flags must be booleans')
        attributes = dict(name=name, pos=_vector(raw['position_m']), dir=direction,
            directional=str(raw['directional']).lower(),
            castshadow=str(raw['cast_shadow']).lower(), cutoff=str(cutoff), exponent=str(exponent),
            attenuation='1 0 0')
        attributes.update({key: _vector(raw[key], color=True)
                           for key in ('ambient', 'diffuse', 'specular')})
        ET.SubElement(world, 'light', attributes)
    if not names:
        raise ValueError('Study-cafe lighting requires at least one light')
