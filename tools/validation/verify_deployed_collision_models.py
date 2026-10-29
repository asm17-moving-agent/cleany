"""Verify installed MoveIt resources and default CUDA models without motion."""
from __future__ import annotations

from dataclasses import replace
import argparse
import hashlib
import importlib.metadata
import json
from pathlib import Path
from types import SimpleNamespace
import xml.etree.ElementTree as ET

import numpy as np
import torch
import trimesh
import xacro
import yaml
from ament_index_python.packages import get_package_share_directory

from cleany_skill_executor.collision_sphere_model import MODEL_PATH, mesh_path
from cleany_skill_executor.core.physical_grasp_selection import ARM_JOINT_NAMES, JointSolution
from cleany_skill_executor.core.urdf_fk import UrdfChain
from cleany_skill_executor.curobo_batch import CuRoboBatchEngine
from cleany_skill_executor.curobo_model import CuRoboConfig, RobotModel
from cleany_skill_executor.hybrid_adapter import _CandidateRobotModel
from cleany_skill_executor.robot_geometry_cache import model_cache_path


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, help='Candidate config before installation; defaults to installed config')
    args = parser.parse_args()
    description = Path(get_package_share_directory('cleany_description'))
    semantic = Path(get_package_share_directory('cleany_moveit_config'))
    package = Path(get_package_share_directory('cleany_skill_executor'))
    parameters = yaml.safe_load((args.config or package/'config/hybrid_motion.yaml').read_text())[
        'grasp_selection_server']['ros__parameters']
    assert parameters['curobo_sphere_model'] == 'joint_simplified'
    xml = xacro.process_file(str(description/'urdf/cleany.urdf.xacro'), mappings={
        'include_head_camera': 'true', 'include_wheel_joints': 'false',
        'head_fixed': 'false', 'calibration_profile': 'jetson_20261009'}).toxml()
    srdf = xacro.process_file(str(semantic/'config/cleany.srdf.xacro'),
                              mappings={'include_head_camera': 'true'}).toxml()
    root = ET.fromstring(xml)
    resources = root.findall('./link/collision/geometry/mesh')
    manifest = json.loads((description/'meshes/collision/arm_meshes.json').read_text())
    for name, entry in manifest['meshes'].items():
        assert hashlib.sha256((description/'meshes/collision'/name).read_bytes()).hexdigest() == entry['output_sha256']
    triangles = sum(len(trimesh.load_mesh(mesh_path(m.get('filename')), process=False).faces)
                    for m in resources)
    optimized = sum('/meshes/collision/' in m.get('filename') for m in resources)
    assert len(resources) == 38 and optimized == 28 and triangles == 6838
    config = CuRoboConfig(num_seeds=16, ik_position_threshold_m=.010,
                         geometry_cache_directory=parameters['curobo_geometry_cache_directory'])
    model = _CandidateRobotModel(xml, srdf, config, exclude_opposite_arm_cost=True)
    assert model.sphere_reduction['method'] == 'joint_simplified'
    assert sum(map(len, model.spheres.values())) == 118
    assert len(model.spheres['head_pan_link']) == 1
    assert len(model.spheres['head_tilt_link']) == 2
    assert 'top_base_link' not in model.spheres
    assert len(model.mast_obstacles) == 5
    assert len(model.bodies) == 232
    assert sum(len(body.vertices) for body in model.bodies) == 21008
    assert model.with_config(replace(config, num_seeds=32)).spheres is model.spheres
    cached = RobotModel(xml, srdf, config)
    assert cached.full_geometry_cache_hit and cached.spheres == model.spheres
    cache_path = model_cache_path(xml, srdf, config)
    state = {}
    for arm, angles in [('left', [-1.6, 2.87, 3.13, -.4, 1.55]),
                        ('right', [1.87, 2.44, 3.08, -1.42, 1.55])]:
        state.update(zip(ARM_JOINT_NAMES[arm], angles))
        state[f'{arm}_gripper_joint'] = -.2 if arm == 'left' else -.26
    state.update(head_pan_joint=0., head_tilt_joint=0.)
    names = model.collision_links
    acm = SimpleNamespace(entry_names=names, entry_values=[SimpleNamespace(enabled=[
        a == b or b in model.ignore.get(a, ()) or a in model.ignore.get(b, ())
        for b in names]) for a in names], default_entry_names=[], default_entry_values=[])
    engine = CuRoboBatchEngine(model, config, ik_iterations=50, ik_batch_size=8)
    result = {'environment': {'torch': torch.__version__, 'cuda': torch.version.cuda,
        'curobo': importlib.metadata.version('nvidia_curobo'), 'device': torch.cuda.get_device_name()},
        'installed_description': str(description), 'installed_skill_executor': str(package),
        'sphere_asset': str(MODEL_PATH), 'sphere_asset_sha256': hashlib.sha256(MODEL_PATH.read_bytes()).hexdigest(),
        'full_model_cache': str(cache_path),
        'full_model_cache_sha256': hashlib.sha256(cache_path.read_bytes()).hexdigest(),
        'moveit': {'mesh_instances': len(resources), 'optimized_instances': optimized, 'triangles': triangles},
        'gpu': {'bodies': len(model.bodies), 'vertices': sum(len(b.vertices) for b in model.bodies)},
        'spheres': {'total': 118, 'wrist_mounts': 4, 'head': 3, 'full_cache_reused': cached.full_geometry_cache_hit,
                    'method': model.sphere_reduction['method']},
        'mast_boxes': model.mast_obstacles,
        'arms': {}, 'robot_motion': False}
    try:
        for arm, joints in ARM_JOINT_NAMES.items():
            tip = f'{arm}_grasp_tcp'
            chain = UrdfChain(xml, 'base_link', tip)
            target = chain.pose(state)[0]
            seed = JointSolution(joints, tuple(state[n] for n in joints))
            targets = [chain.pose({**state, joints[0]: state[joints[0]]+float(offset)})[0]
                       for offset in np.linspace(-.06, .06, 8)]
            groups = engine.solve_batch(arm, tip, [p.tolist() for p in targets], [[seed]]*8, state, {})
            assert len(groups) == 8 and all(groups)
            independent_errors = []
            accepted = []
            for group, target in zip(groups, targets):
                values = [s.positions for s in group]
                valid = engine.valid_states(joints, values, state, {}, acm)
                assert valid.any()
                accepted.append(int(valid.sum()))
                for solution in group:
                    actual = chain.pose({**state, **dict(zip(joints, solution.positions))})[0]
                    independent_errors.append(float(np.linalg.norm(actual-target)))
            assert max(independent_errors) <= .01001
            solver = engine._batch_ik_caches[(arm, tip, 8)][0][(arm, tip)]
            active = int(solver.kinematics.get_state(
                engine.tensor.to_device([seed.positions])).link_spheres_tensor.shape[-2])
            assert active == (60 if arm == 'left' else 61)
            head_errors = []
            cache = engine._batch_ik_caches[(arm, tip, 8)]
            for pan, tilt in [(0., 0.), (-1., -.4), (.8, .6), (0., 0.)]:
                moved_state = {**state, 'head_pan_joint': pan, 'head_tilt_joint': tilt}
                previous = engine._ik, engine._ik_locks, engine._world_keys
                engine._ik, engine._ik_locks, engine._world_keys = cache
                try:
                    assert engine._solver(arm, tip, moved_state, {}) is solver
                finally:
                    engine._ik, engine._ik_locks, engine._world_keys = previous
                actual = solver.kinematics.get_state(engine.tensor.to_device(
                    [seed.positions])).link_spheres_tensor[0].detach().cpu().numpy()
                matched = set()
                for owner in ('head_pan_link', 'head_tilt_link'):
                    p, r = UrdfChain(xml, 'base_link', owner).pose(moved_state)
                    for sphere in model.spheres[owner]:
                        expected = np.r_[r @ sphere['center'] + p, sphere['radius']]
                        distances = np.linalg.norm(actual-expected, axis=1)
                        index = int(distances.argmin())
                        assert index not in matched
                        matched.add(index)
                        head_errors.append(float(distances[index]))
            assert max(head_errors) < 1e-5
            elbow_errors = []
            for angle in (-.2, .7, 1.8, 3.1):
                moved_state = {**state, f'{arm}_elbow_pitch_joint': angle}
                q = [moved_state[n] for n in solver.joint_names]
                actual = solver.kinematics.get_state(engine.tensor.to_device(
                    [q])).link_spheres_tensor[0].detach().cpu().numpy()
                matched = set()
                for owner in (f'{arm}_upper_arm_link', f'{arm}_lower_arm_link'):
                    p, r = UrdfChain(xml, 'base_link', owner).pose(moved_state)
                    for sphere in model.spheres[owner][:2]:
                        expected = np.r_[r @ sphere['center'] + p, sphere['radius']]
                        distances = np.linalg.norm(actual-expected, axis=1)
                        index = int(distances.argmin())
                        assert index not in matched
                        matched.add(index)
                        elbow_errors.append(float(distances[index]))
            assert max(elbow_errors) < 1e-5
            shoulder_errors = []
            owner = f'{arm}_upper_arm_link'
            assert len(model.spheres[owner]) == 6
            for yaw, pitch in [(-1., .4), (.8, 1.5), (1.1, 3.), (0., 0.)]:
                moved_state = {**state, f'{arm}_shoulder_yaw_joint': yaw,
                               f'{arm}_shoulder_pitch_joint': pitch}
                q = [moved_state[n] for n in solver.joint_names]
                actual = solver.kinematics.get_state(engine.tensor.to_device(
                    [q])).link_spheres_tensor[0].detach().cpu().numpy()
                p, r = UrdfChain(xml, 'base_link', owner).pose(moved_state)
                matched = set()
                for sphere in model.spheres[owner][2:4]:
                    expected = np.r_[r @ sphere['center'] + p, sphere['radius']]
                    distances = np.linalg.norm(actual-expected, axis=1)
                    index = int(distances.argmin())
                    assert index not in matched
                    matched.add(index)
                    shoulder_errors.append(float(distances[index]))
            assert max(shoulder_errors) < 1e-5
            wrist_errors = []
            owner = f'{arm}_gripper_frame'
            covers = [c for c in json.loads(MODEL_PATH.read_text())['rigid_covers']
                      if c['link'] == owner and c['region'] == 'wrist_camera_mount']
            assert len(covers) == 2 and len(model.spheres[owner]) == 15
            wrist_spheres = [model.spheres[owner][c['sphere_index']] for c in covers]
            for pitch, roll in [(-1.4, -2.), (-.4, 1.55), (1.2, 2.), (0., 0.)]:
                moved_state = {**state, f'{arm}_wrist_pitch_joint': pitch,
                               f'{arm}_wrist_roll_joint': roll}
                q = [moved_state[n] for n in solver.joint_names]
                actual = solver.kinematics.get_state(engine.tensor.to_device(
                    [q])).link_spheres_tensor[0].detach().cpu().numpy()
                p, r = UrdfChain(xml, 'base_link', owner).pose(moved_state)
                matched = set()
                for sphere in wrist_spheres:
                    expected = np.r_[r @ sphere['center']+p, sphere['radius']]
                    distances = np.linalg.norm(actual-expected, axis=1)
                    index = int(distances.argmin())
                    assert index not in matched
                    matched.add(index)
                    wrist_errors.append(float(distances[index]))
            assert max(wrist_errors) < 1e-5
            result['arms'][arm] = {'targets': 8, 'active_proposal_spheres': active,
                'full_geometry_valid_solutions_per_target': accepted,
                'independent_fk_error_max_mm': max(independent_errors)*1000,
                'head_locked_pose_updates': 4,
                'head_sphere_fk_error_max_mm': max(head_errors)*1000,
                'elbow_poses': 4,
                'elbow_sphere_fk_error_max_mm': max(elbow_errors)*1000,
                'shoulder_poses': 4,
                'shoulder_upper_sphere_fk_error_max_mm': max(shoulder_errors)*1000,
                'wrist_mount_poses': 4,
                'wrist_mount_sphere_fk_error_max_mm': max(wrist_errors)*1000}
    finally:
        engine.close()
    print('DEPLOYMENT_RESULT ' + json.dumps(result, allow_nan=False), flush=True)


if __name__ == '__main__':
    main()
