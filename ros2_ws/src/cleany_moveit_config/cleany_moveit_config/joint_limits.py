"""Keep default planning overrides consistent with a materialized real model."""
import copy
import math
import xml.etree.ElementTree as ET


def limits_from_model(config: dict, model: str) -> dict:
    result = copy.deepcopy(config)
    joints = {joint.get('name'): joint for joint in ET.fromstring(model).findall('joint')}
    for name, values in result['robot_description_planning']['joint_limits'].items():
        joint = joints.get(name)
        limit = None if joint is None else joint.find('limit')
        if joint is None or joint.get('type') not in ('revolute', 'prismatic') or limit is None:
            raise ValueError('Planning limits lack a bounded model joint: ' + name)
        lower, upper = float(limit.get('lower')), float(limit.get('upper'))
        if not math.isfinite(lower) or not math.isfinite(upper) or lower >= upper:
            raise ValueError('Invalid model joint bounds: ' + name)
        values.update(has_position_limits=True, min_position=lower, max_position=upper)
    return result
