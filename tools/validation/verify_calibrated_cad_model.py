"""Compare an operational-coordinate adapter with its unchanged CAD source."""
from __future__ import annotations

import argparse
from copy import deepcopy
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np

from cleany_description.cad_coordinates import CadCoordinates
from cleany_description.gripper_linkage import solve_linkage
from cleany_skill_executor.core.urdf_fk import UrdfChain


def verify(source: Path, candidate: Path, inputs: Path, reference: Path,
           samples: int = 48) -> dict:
    raw, adapted = source.read_text(), candidate.read_text()
    bindings = json.loads(inputs.read_text())
    root, current = ET.fromstring(raw), ET.fromstring(adapted)
    closure = CadCoordinates.from_urdf(adapted)
    assert closure is not None
    limits = {j.get('name'): [float(j.find('limit').get(k)) for k in ('lower','upper')]
              for j in ET.parse(reference).getroot().findall('joint')
              if j.get('type') == 'revolute' and j.find('mimic') is None}
    for name, bound in limits.items():
        limit = current.find(f"joint[@name='{name}']/limit")
        assert [float(limit.get(k)) for k in ('lower','upper')] == bound, name
    for link in root.findall('link'):
        target = current.find(f"link[@name='{link.get('name')}']")
        assert target is not None
        for kind in ('visual','collision'):
            assert [ET.tostring(v) for v in link.findall(kind)] == [ET.tostring(v) for v in target.findall(kind)]
    links = [link.get('name') for link in root.findall('link') if link.get('name') != 'base_link']
    source_chains = {name: UrdfChain(raw, 'base_link', name) for name in links}
    adapted_chains = {name: UrdfChain(adapted, 'base_link', name) for name in links}
    source_names = [j.get('name') for j in root.findall('joint') if j.get('type') != 'fixed']
    rng = np.random.default_rng(42)
    position_error, rotation_error = 0., 0.
    for _ in range(samples):
        state = {name: float(rng.uniform(*bound)) for name,bound in limits.items()}
        cad = dict.fromkeys(source_names, 0.)
        for name, cfg in bindings['coordinates'].items():
            cad[name] = state[cfg['source_joint']]*cfg['scale']+cfg['offset_rad']
        for group in closure.config['grippers']:
            delta = state[group['source_joint']]*group['scale']+group['offset_rad']
            cad[group['linkage']['cam_joint']] = delta
            fingers, _ = solve_linkage(group['linkage'], delta)
            cad.update(fingers)
        measured = closure.expanded(state)
        for name in links:
            p0, r0 = source_chains[name].pose(cad)
            p1, r1 = adapted_chains[name].pose(measured)
            position_error = max(position_error,float(np.linalg.norm(p1-p0)))
            rotation_error = max(rotation_error,float(np.max(np.abs(r1-r0))))
    assert position_error < 1e-10 and rotation_error < 1e-10
    return {'success':True,'samples':samples,'source_links_compared':len(links),
            'fk_comparisons':samples*len(links), 'maximum_position_error_m':position_error,
            'maximum_rotation_matrix_error':rotation_error,
            'all_original_visual_and_collision_entries_unchanged':True,
            'actuator_joint_limits_unchanged':True, 'nonlinear_gripper_states_resolved':True,
            'source_sha256':hashlib.sha256(raw.encode()).hexdigest(),
            'candidate_sha256':hashlib.sha256(adapted.encode()).hexdigest(),
            'scope':'software_coordinate_equivalence_not_physical_accuracy',
            'hardware_drivers_started':0,'motor_commands_sent':0}


def main() -> None:
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ('source','candidate','inputs','reference','output'):
        parser.add_argument('--'+name,type=Path,required=True)
    args=parser.parse_args()
    result=verify(args.source,args.candidate,args.inputs,args.reference)
    args.output.write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result))


if __name__ == '__main__':
    main()
