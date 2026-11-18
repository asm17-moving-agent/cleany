"""Materialize calibrated arm limits for control, planning and the CAD display."""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import yaml

from cleany_robot_interface.cad_joint_bounds import apply_cad_joint_bounds
from cleany_robot_interface.core import motor_calibration_directory
from cleany_robot_interface.control_profile import prepare_control_profile
from cleany_robot_interface.core import mappings


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('repository', 'profile', 'control', 'cad-model', 'pose-config', 'output'):
        parser.add_argument('--' + name, type=Path, required=True)
    args = parser.parse_args()
    profile = prepare_control_profile(args.profile.resolve(), args.repository.resolve(),
                                      args.output.resolve(), args.control.resolve(), True)
    state = profile.state.model.read_text()
    state_tree = ET.fromstring(state)
    control = ET.parse(profile.model).getroot()
    arm = yaml.safe_load(profile.state.arm_config.read_text())
    directory = motor_calibration_directory(profile.state.arm_config, arm)
    calibrations = {side: json.loads((directory / cfg['calibration']).read_text())
                    for side, cfg in arm['arms'].items()}
    coordinates = json.loads(args.pose_config.read_text())['joint_coordinates']
    cad = apply_cad_joint_bounds(args.cad_model.read_text(), state, coordinates)
    cad_file = args.output / 'cad.urdf'
    cad_file.write_text(cad)
    cad_tree = ET.fromstring(cad)
    cad_bounds = {}
    for name, mapping in coordinates.items():
        target = cad_tree.find(f"joint[@name='{name}']/limit")
        values = [float(target.get(k)) for k in ('lower', 'upper')]
        cad_bounds[name] = values
    rows = []
    for joint in mappings(arm, calibrations):
        limit = control.find(f"joint[@name='{joint.name}']/limit")
        effective = [float(limit.get(k)) for k in ('lower', 'upper')]
        physical = sorted((joint.radians(joint.range_min), joint.radians(joint.range_max)))
        state_limit = state_tree.find(f"joint[@name='{joint.name}']/limit")
        assert physical == [float(state_limit.get(k)) for k in ('lower', 'upper')], (
            'State profile must select every calibrated arm joint', joint.name)
        assert effective[0] >= physical[0] - 1e-12
        assert effective[1] <= physical[1] + 1e-12
        driver = control.find(f"ros2_control/joint[@name='{joint.name}']")
        assert effective == [float(driver.find(f"param[@name='{k}']").text)
                             for k in ('lower', 'upper')]
        rows.append({'motor': joint.motor_id, 'joint': joint.name,
                     'raw_ticks': [joint.range_min, joint.range_max],
                     'physical_rad': physical, 'control_rad': effective})
    report = {'hardware_writes': False, 'physical_motion_validated': False,
              'joints': rows, 'cad_bounds_rad': cad_bounds,
              'files_sha256': {str(p): hashlib.sha256(p.read_bytes()).hexdigest()
                               for p in (profile.state.model, profile.model, cad_file,
                                         args.profile, args.control, args.pose_config)}}
    (args.output / 'limits-verification.json').write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(report))


if __name__ == '__main__':
    main()
