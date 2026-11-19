"""네임스페이스 접두사를 문자열로 읽는 Gazebo 확장 속성을 직렬화합니다."""
from pathlib import Path
from math import isfinite
from tempfile import mkdtemp
from xml.etree import ElementTree as ET

NAMESPACES = {'ignition': 'http://ignitionrobotics.org/schema',
              'gz': 'http://gazebosim.org/schema'}


def write_world(tree: ET.ElementTree, target: Path) -> None:
    # DART는 HasAttribute("ignition:expressed_in") 또는 "gz:expressed_in"을 사용합니다.
    # XML상 동등한 ns0 접두사도 마찰 좌표계의 해석을 조용히 바꿀 수 있습니다.
    for prefix, uri in NAMESPACES.items():
        ET.register_namespace(prefix, uri)
    tree.write(target, encoding='unicode', xml_declaration=True)


def _set_physics(tree: ET.ElementTree, name: str, value: float) -> None:
    physics = tree.find('./world/physics')
    if physics is None:
        raise ValueError('physics override requires a world physics block')
    element = physics.find(name)
    if element is None:
        element = ET.SubElement(physics, name)
    element.text = str(value)


def materialize_friction_frames(
    source: Path, *, max_step_size: float | None = None,
    real_time_factor: float | None = None,
) -> Path:
    """제공된 원본을 수정하지 않고 외부 또는 생성된 월드를 보정합니다."""
    tree = ET.parse(source)
    key = '{'+NAMESPACES['ignition']+'}expressed_in'
    modified = False
    if real_time_factor is not None:
        if not isfinite(real_time_factor) or real_time_factor <= 0:
            raise ValueError('physics real_time_factor must be finite and positive')
        _set_physics(tree, 'real_time_factor', real_time_factor)
        modified = True
    if max_step_size is not None:
        if not isfinite(max_step_size) or not 0 < max_step_size <= .01:
            raise ValueError('physics max_step_size must be finite and in (0, 0.01]')
        _set_physics(tree, 'max_step_size', max_step_size)
        modified = True
    for direction in tree.findall('.//fdir1'):
        for candidate in list(direction.attrib):
            if candidate in {'{'+uri+'}expressed_in' for uri in NAMESPACES.values()}:
                direction.set(key, direction.attrib.pop(candidate))
                modified = True
    if not modified:
        return source
    target = Path(mkdtemp(prefix='cleany-friction-frames-')) / source.name
    write_world(tree, target)
    return target
