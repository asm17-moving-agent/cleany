"""Serialize Gazebo extensions whose readers match literal namespace prefixes."""
from pathlib import Path
from math import isfinite
from tempfile import mkdtemp
from xml.etree import ElementTree as ET

NAMESPACES = {'ignition': 'http://ignitionrobotics.org/schema',
              'gz': 'http://gazebosim.org/schema'}


def write_world(tree: ET.ElementTree, target: Path) -> None:
    # DART reads HasAttribute("ignition:expressed_in") / "gz:expressed_in".
    # XML-equivalent ns0 prefixes silently change friction frame semantics.
    for prefix, uri in NAMESPACES.items():
        ET.register_namespace(prefix, uri)
    tree.write(target, encoding='unicode', xml_declaration=True)


def materialize_friction_frames(
    source: Path, *, harmonic: bool = False, max_step_size: float | None = None,
    real_time_factor: float | None = None,
) -> Path:
    """Repair external/generated worlds without editing the supplied artifact."""
    tree = ET.parse(source)
    prefix = 'gz' if harmonic else 'ignition'
    key = '{'+NAMESPACES[prefix]+'}expressed_in'
    found = False
    if real_time_factor is not None:
        if not isfinite(real_time_factor) or real_time_factor <= 0:
            raise ValueError('physics real_time_factor must be finite and positive')
        physics = tree.find('./world/physics')
        if physics is None:
            raise ValueError('physics override requires a world physics block')
        factor = physics.find('real_time_factor')
        if factor is None:
            factor = ET.SubElement(physics, 'real_time_factor')
        factor.text = str(real_time_factor)
        found = True
    if max_step_size is not None:
        if not isfinite(max_step_size) or not 0 < max_step_size <= .01:
            raise ValueError('physics max_step_size must be finite and in (0, 0.01]')
        physics = tree.find('./world/physics')
        if physics is None:
            raise ValueError('physics override requires a world physics block')
        step = physics.find('max_step_size')
        if step is None:
            step = ET.SubElement(physics, 'max_step_size')
        step.text = str(max_step_size)
        found = True
    for direction in tree.findall('.//fdir1'):
        for candidate in list(direction.attrib):
            if candidate in {'{'+uri+'}expressed_in' for uri in NAMESPACES.values()}:
                direction.set(key, direction.attrib.pop(candidate))
                found = True
    if not found:
        return source
    target = Path(mkdtemp(prefix='cleany-friction-frames-')) / source.name
    write_world(tree, target)
    return target
