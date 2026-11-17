"""Publish one calibrated, non-actuated CAD asset for every default runtime."""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

from cleany_robot_interface.state_profile import prepare_state_profile


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--repository', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    root = args.repository.resolve()
    profile = root/'configs/robot/cad_grasp.json'
    prepared = prepare_state_profile(profile, root, args.output)
    text = prepared.model.read_text()
    tree = ET.fromstring(text)
    if tree.find('ros2_control') is not None or 'cleany_cad_calibration' not in tree.attrib:
        raise ValueError('The default CAD must be calibrated and contain no hardware plugin')
    package = root/'ros2_ws/src/cleany_description'
    destination = package/'urdf/cleany.urdf'
    destination.write_text(text)
    binding = json.loads((root/'configs/robot/cad_grasp_binding.json').read_text())
    manifest = {'schema': 'cleany.runtime_cad/v1',
        'model': 'urdf/cleany.urdf',
        'sha256': hashlib.sha256(text.encode()).hexdigest(),
        'state_profile': 'configs/robot/cad_grasp.json',
        'source_model_sha256': binding['source_model_sha256'],
        'gripper_endpoint_sha256': binding['gripper_endpoint_sha256'],
        'actuator_names': [j.get('name') for j in tree.findall('joint')
                           if j.get('type') == 'revolute' and j.find('mimic') is None],
        'motor_commands_sent': 0}
    (package/'config/geometry/runtime_model.json').write_text(json.dumps(manifest, indent=2)+'\n')
    print(json.dumps(manifest))


if __name__ == '__main__':
    main()
