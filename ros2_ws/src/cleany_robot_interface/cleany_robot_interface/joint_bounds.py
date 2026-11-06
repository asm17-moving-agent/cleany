"""Materialize explicit model bounds without changing observed joint angles."""
import math
import xml.etree.ElementTree as ET

from cleany_robot_interface.core import JointMapping


def calibrated_joint_bounds(
    joints: list[JointMapping], selected: list[str],
) -> dict[str, tuple[float, float]]:
    by_name = {joint.name: joint for joint in joints}
    if (not isinstance(selected, list) or not all(isinstance(name, str) for name in selected)
            or len(selected) != len(set(selected)) or not set(selected) <= set(by_name)):
        raise ValueError('calibrated_joint_position_bounds must name unique calibrated arm joints')
    return {name: tuple(sorted((by_name[name].radians(by_name[name].range_min),
                               by_name[name].radians(by_name[name].range_max))))
            for name in selected}


def apply_joint_bounds(model: str, bounds: dict) -> str:
    root = ET.fromstring(model)
    for name, values in bounds.items():
        if (not isinstance(values, (list, tuple)) or len(values) != 2
                or not all(isinstance(v, (int, float)) and math.isfinite(v) for v in values)
                or values[0] >= values[1]):
            raise ValueError('Invalid joint position bounds: ' + name)
        joint = root.find(f"joint[@name='{name}']")
        limit = None if joint is None else joint.find('limit')
        if joint is None or joint.get('type') not in ('revolute', 'prismatic') or limit is None:
            raise ValueError('Joint position bound lacks a bounded model joint: ' + name)
        limit.set('lower', str(values[0]))
        limit.set('upper', str(values[1]))
    return ET.tostring(root, encoding='unicode')
