"""Serialize Gazebo extensions whose readers match literal namespace prefixes."""
from pathlib import Path
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


def materialize_friction_frames(source: Path, *, harmonic: bool = False) -> Path:
    """Repair external/generated worlds without editing the supplied artifact."""
    tree = ET.parse(source)
    prefix = 'gz' if harmonic else 'ignition'
    key = '{'+NAMESPACES[prefix]+'}expressed_in'
    found = False
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
