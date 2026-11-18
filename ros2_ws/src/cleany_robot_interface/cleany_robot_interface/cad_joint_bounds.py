"""Transport operational joint limits into the existing CAD display coordinates."""
import math
import xml.etree.ElementTree as ET


def apply_cad_joint_bounds(model: str, source_model: str, coordinates: dict) -> str:
    """Bound mapped CAD joints without changing their axes or display mapping.

    Coordinates use ``q_cad = scale * q_ros + offset_rad``. These are display
    coordinates, not a new encoder calibration or physical clearance estimate.
    Unmapped linkage joints retain their existing contracts.
    """
    root, source = ET.fromstring(model), ET.fromstring(source_model)
    for name, mapping in coordinates.items():
        scale, offset = mapping['scale'], mapping['offset_rad']
        if (not all(type(v) in (int, float) and math.isfinite(v) for v in (scale, offset))
                or scale == 0):
            raise ValueError('Invalid CAD joint coordinate mapping: ' + name)
        joint = root.find(f"joint[@name='{name}']")
        original = source.find(f"joint[@name='{mapping['source_joint']}']")
        limit = None if original is None else original.find('limit')
        if (joint is None or joint.get('type') not in ('continuous', 'revolute')
                or original is None or original.get('type') != 'revolute' or limit is None):
            raise ValueError('CAD mapping requires bounded operational joint: ' + name)
        lower, upper = (float(limit.get(key)) for key in ('lower', 'upper'))
        if not all(math.isfinite(v) for v in (lower, upper)) or lower >= upper:
            raise ValueError('Invalid operational joint limits: ' + name)
        values = sorted((scale * lower + offset, scale * upper + offset))
        target = joint.find('limit')
        if target is None:
            target = ET.SubElement(joint, 'limit')
        joint.set('type', 'revolute')
        target.attrib.update(lower=str(values[0]), upper=str(values[1]),
                             effort=limit.get('effort'),
                             velocity=str(abs(scale) * float(limit.get('velocity'))))
    return ET.tostring(root, encoding='unicode')
