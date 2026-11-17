"""Resolve calibrated CAD linkage state while retaining actuator coordinates."""
from __future__ import annotations

from dataclasses import dataclass
from copy import deepcopy
import json
import math
import xml.etree.ElementTree as ET

from .gripper_linkage import solve_linkage

ATTRIBUTE = 'cleany_cad_calibration'
SCHEMA = 'cleany.calibrated_cad/v1'


def match_gripper_endpoints(config: dict, endpoints: dict) -> dict:
    """Map actuator endpoints onto nominal CAD endpoints without changing commands.

    This affine display/geometry correspondence is not a measured motor/cam
    transmission ratio. The existing nonlinear solver still closes each link.
    """
    result = deepcopy(config)
    for group in result['grippers']:
        pair = endpoints['grippers'][group['side']]
        values = {name: float(pair[name]) for name in (
            'motor_open_rad', 'motor_closed_rad', 'cad_open_rad', 'cad_closed_rad')}
        if (not all(math.isfinite(v) for v in values.values())
                or values['motor_open_rad'] <= values['motor_closed_rad']
                or values['cad_open_rad'] >= values['cad_closed_rad']):
            raise ValueError('Invalid CAD gripper endpoint correspondence')
        scale = ((values['cad_open_rad']-values['cad_closed_rad']) /
                 (values['motor_open_rad']-values['motor_closed_rad']))
        group.update(scale=scale,
                     offset_rad=values['cad_closed_rad']-scale*values['motor_closed_rad'],
                     endpoint_mapping=values,
                     basis='Operator-selected endpoint correspondence; intermediate CAD geometry is nominal, not a measured transmission ratio.')
    return result


def gripper_touch_links(description: str, side: str) -> list[str]:
    """Return hand bodies only; arm and support collision checks remain separate."""
    if side not in ('left', 'right'):
        raise ValueError('Unknown gripper side')
    defaults = [side+'_gripper_frame', side+'_moving_jaw_link']
    root = ET.fromstring(description)
    if root.get(ATTRIBUTE) is None:
        return defaults
    parents = {j.find('child').get('link'):j.find('parent').get('link') for j in root.findall('joint')}
    wrist = 'cad_'+side+'_wrist_roll'
    links = set(defaults)
    for link in root.findall('link'):
        if link.find('collision') is None:
            continue
        child = link.get('name')
        visited = set()
        while child and child not in visited:
            if child == wrist:
                links.add(link.get('name'))
                break
            visited.add(child)
            child = parents.get(child)
    if wrist not in links:
        raise ValueError('Calibrated CAD wrist collision geometry is missing')
    return sorted(links)


@dataclass(frozen=True)
class CadCoordinates:
    """Use the existing nominal pin solver; never interpret derived joints as motors."""

    config: dict

    def __post_init__(self) -> None:
        if self.config.get('schema') != SCHEMA:
            raise ValueError('Unknown calibrated CAD coordinate schema')
        groups = self.config.get('grippers', [])
        if len(groups) != 2 or {g['side'] for g in groups} != {'left', 'right'}:
            raise ValueError('Both CAD gripper coordinate bindings are required')
        joints = set()
        for group in groups:
            if group['source_joint'] != group['side']+'_gripper_joint':
                raise ValueError('CAD gripper binding must name its actuator coordinate')
            if (not math.isfinite(group['scale']+group['offset_rad']) or group['scale'] == 0):
                raise ValueError('Invalid CAD cam coordinate binding')
            endpoints = group.get('endpoint_mapping')
            if endpoints:
                for end in ('open', 'closed'):
                    mapped = group['scale']*endpoints['motor_'+end+'_rad']+group['offset_rad']
                    if not math.isclose(mapped, endpoints['cad_'+end+'_rad'], abs_tol=1e-12):
                        raise ValueError('CAD gripper endpoint mapping is stale')
            for spec in group['linkage']['fingers'].values():
                if joints.intersection(spec['joints'].values()):
                    raise ValueError('Dependent CAD joints must be unique')
                joints.update(spec['joints'].values())
            solve_linkage(group['linkage'], 0.)
        if len(joints) != 16:
            raise ValueError('Two closed CAD grippers require sixteen dependent coordinates')

    @classmethod
    def from_urdf(cls, description: str) -> CadCoordinates | None:
        value = ET.fromstring(description).get(ATTRIBUTE)
        return cls(json.loads(value)) if value else None

    @property
    def dependent_names(self) -> tuple[str, ...]:
        return tuple(name for group in self.config['grippers']
                     for spec in group['linkage']['fingers'].values()
                     for name in spec['joints'].values())

    def dependent_positions(self, positions: dict[str, float]) -> dict[str, float]:
        result = {}
        for group in self.config['grippers']:
            value = positions[group['source_joint']]
            if not math.isfinite(value):
                raise ValueError('Nonfinite actuator position for CAD linkage')
            delta = group['scale']*value+group['offset_rad']
            resolved, _ = solve_linkage(group['linkage'], delta)
            result.update(resolved)
        return result

    def expanded(self, positions: dict[str, float]) -> dict[str, float]:
        return {**positions, **self.dependent_positions(positions)}
