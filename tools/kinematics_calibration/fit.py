#!/usr/bin/env python3
"""Fit review-only kinematic corrections from frozen wrist-camera captures."""

from __future__ import annotations

import argparse
from hashlib import sha256
import json
from pathlib import Path
import platform
import subprocess

import cv2
import numpy as np
import scipy
from scipy.spatial.transform import Rotation

from model import Chain, Observation, PARAMETER_COUNTS, Problem, pose, vector


NAMES = ('shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll')


def read(path: Path) -> dict:
    return json.loads(path.read_text())


def digest(path: Path) -> str:
    return sha256(path.read_bytes()).hexdigest()


def load(spec: dict, urdf: Path) -> tuple[Problem, np.ndarray, dict]:
    extrinsics_path, intrinsics_path = Path(spec['extrinsics']), Path(spec['intrinsics'])
    extrinsics, intrinsics = read(extrinsics_path), read(intrinsics_path)
    side = extrinsics['side']
    dataset = Path(extrinsics['input_dataset'])
    manifest_path = dataset / 'manifest.json'
    manifest = read(manifest_path)
    if side != intrinsics['side'] or side != manifest['side']:
        raise ValueError('Mismatched camera sides')
    if digest(urdf) != extrinsics['cad_urdf_sha256'] or digest(urdf) != manifest['cad_urdf_sha256']:
        raise ValueError('URDF differs from the capture model')
    if digest(manifest_path) != intrinsics['manifest_sha256'] or digest(intrinsics_path) != extrinsics['intrinsics_sha256']:
        raise ValueError('Changed capture manifest or intrinsics')
    split_path = Path(extrinsics['split_manifest'])
    if digest(split_path) != extrinsics['split_manifest_sha256']:
        raise ValueError('Changed frozen split')
    split = read(split_path)['cameras'][side]
    train, held = split['training_samples'], split['held_out_samples']
    if len(train) != 20 or len(held) != 5 or len(set(train + held)) != 25:
        raise ValueError('Expected disjoint frozen 20/5 split')
    cfg = manifest['config']
    columns, rows = cfg['squares_x'] - 1, cfg['squares_y'] - 1
    kin = extrinsics['kinematics']
    chain = Chain.read(urdf, extrinsics['parent_frame'])
    if chain.names != kin['joint_names']:
        raise ValueError('Joint order differs from capture model')
    observations = []
    for name in train + held:
        record_path = dataset / name / 'record.json'
        image_path = dataset / name / 'rgb.png'
        record = read(record_path)
        expected = next(row for row in intrinsics['input_files'] if row['sample'] == name)
        if digest(record_path) != expected['record_sha256'] or digest(image_path) != expected['rgb_sha256']:
            raise ValueError(f'Changed input: {name}')
        ids = np.asarray(record['corner_ids'], dtype=int)
        if np.any(ids < 0) or np.any(ids >= columns * rows) or len(set(ids.tolist())) != len(ids):
            raise ValueError('Invalid ChArUco corner IDs')
        points = np.c_[ids % columns + 1, ids // columns + 1, np.zeros(len(ids))] * cfg['square_length_m']
        nominal = []
        for joint in NAMES:
            feedback = record['joint_observation']['joints'][joint]
            if feedback['id'] != cfg['motor_ids'][joint]:
                raise ValueError('Feedback motor ID differs from capture configuration')
            mapping = kin['raw_mapping'][joint]
            nominal.append(mapping['offset_rad'] + mapping['direction'] * (feedback['raw_tick'] - mapping['zero_tick']) * 2 * np.pi / 4096)
        angles = np.array(nominal) * kin['signs'] + kin['offsets_rad']
        pixels = np.asarray(record['image_points_px'], dtype=float)
        if pixels.shape != (len(points), 2) or not np.isfinite(pixels).all() or not np.isfinite(angles).all():
            raise ValueError('Invalid observations')
        observations.append(Observation(name, angles, points, pixels))
    problem = Problem(chain, observations, np.array(intrinsics['K']), np.array(intrinsics['D']))
    seed = np.r_[vector(np.array(extrinsics['parent_T_camera'])), vector(np.array(extrinsics['base_T_board']))]
    provenance = {
        'side': side, 'dataset': str(dataset), 'urdf': str(urdf), 'urdf_sha256': digest(urdf),
        'intrinsics': str(intrinsics_path), 'intrinsics_sha256': digest(intrinsics_path),
        'source_extrinsics': str(extrinsics_path), 'source_extrinsics_sha256': digest(extrinsics_path),
        'split_manifest': str(split_path), 'split_manifest_sha256': digest(split_path),
        'raw_mapping': kin, 'reference_camera': extrinsics['reference'],
        'training_samples': train, 'held_out_samples': held,
    }
    return problem, seed, provenance


def calibrate(problem: Problem, seed: np.ndarray, *, folds: int, max_evaluations: int) -> dict:
    training, held_out = list(range(20)), list(range(20, 25))
    families = list(PARAMETER_COUNTS)
    if not 2 <= folds <= 10:
        raise ValueError('Training cross-validation requires 2 to 10 folds')
    # Model selection only sees the original 20 training views. Held-out views
    # are reserved for the selected model and its uncorrected baseline.
    partitions = [list(map(int, fold)) for fold in np.array_split(np.arange(20), folds)]
    cv = {family: [] for family in families}
    failures = []
    for fold, validation in enumerate(partitions):
        fit_indices = [index for index in training if index not in validation]
        warm = seed.copy()
        for family in families:
            fit = problem.fit(family, fit_indices, warm, max_evaluations=max_evaluations)
            warm = fit.x
            score = problem.evaluate(fit.x, validation)
            score.update({'optimizer_success': bool(fit.success), 'evaluations': fit.nfev})
            cv[family].append(score)
            if not fit.success or not score['all_points_in_front']:
                failures.append({'fold': fold, 'family': family, 'message': fit.message})
            print(f'fold={fold + 1} family={family} validation={score["rms_px"]:.3f}px converged={fit.success}', flush=True)
    means = {family: float(np.mean([score['rms_px'] for score in scores])) for family, scores in cv.items()}
    eligible = [family for family in families if all(s['optimizer_success'] and s['all_points_in_front'] for s in cv[family])]
    if not eligible:
        raise ValueError('No model family converged in every training fold')
    best = min(eligible, key=means.get)
    standard_error = float(np.std([s['rms_px'] for s in cv[best]], ddof=1) / np.sqrt(folds))
    selected = next(f for f in eligible if means[f] <= means[best] + standard_error)
    warm = seed.copy()
    baseline = None
    for family in families[:families.index(selected) + 1]:
        final = problem.fit(family, training, warm, max_evaluations=max_evaluations)
        if not final.success:
            raise ValueError(f'Final {family} optimization failed: {final.message}')
        warm = final.x
        if family == 'extrinsics':
            baseline = problem.evaluate(final.x, held_out)
    trained = problem.evaluate(final.x, training)
    validation = problem.evaluate(final.x, held_out)
    origins, axes = problem.chain.corrected(final.x)
    # Remove prior rows before reporting information supplied by measurements.
    measurement_rows = sum(2 * len(problem.observations[i].points) for i in training)
    jacobian = final.jac[:measurement_rows]
    scales = np.r_[np.full(3, .1), np.full(3, .01), np.full(3, .1), np.full(3, .01), problem.prior_sigma(len(final.x))]
    singular = np.linalg.svd(jacobian * scales, compute_uv=False)
    rank = int(np.sum(singular > singular[0] * 1e-6))
    sigma = problem.prior_sigma(len(final.x))
    return {
        'status': 'review_only', 'selected_family': selected,
        'selection': 'first, least complex family within one standard error of best training-only cross-validation',
        'cross_validation_mean_rms_px': means, 'cross_validation': cv, 'cross_validation_failures': failures,
        'training': trained, 'held_out': validation, 'baseline_held_out': baseline,
        'parameter_vector': final.x.tolist(), 'parameter_count': len(final.x),
        'scaled_measurement_jacobian_rank': rank, 'scaled_measurement_jacobian_singular_values': singular.tolist(),
        'parameters_within_1pct_of_bound': np.flatnonzero(np.abs(final.x[12:]) >= .99 * 5 * sigma).tolist(),
        'corrections': {
            'joint_names': problem.chain.names,
            'zero_correction_rad': ([0.] + final.x[12:15].tolist() + [0.]) if len(final.x) >= 15 else [0.] * 5,
            'joint_origin_matrices': origins.tolist(), 'joint_axes': axes.tolist(),
            'axis_line_offsets_m': problem.chain.axis_positions(final.x).tolist(),
            'joint_motion_convention': 'origin @ Translate(axis_line_offset) @ Rotate(joint_axis, angle + zero_correction) @ Translate(-axis_line_offset)',
            'wrist_T_camera': pose(final.x[:6]).tolist(), 'base_T_board': pose(final.x[6:12]).tolist(),
            'axis_direction_change_deg': np.degrees(np.arccos(np.clip(np.sum(axes * problem.chain.axes, axis=1), -1., 1.))).tolist(),
            'link_length_correction_m': final.x[23:25].tolist() if len(final.x) == 25 else None,
        },
        'gauge': 'First yaw and last roll zeros fixed to source mapping; camera mount and board pose fitted jointly. Link/axis corrections are regularized relative to the input CAD.',
        'physical_parameters_independently_verified': False,
        'source_raw_mapping_retained': True,
        'limits_modified': False, 'runtime_applied': False,
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    config = read(args.config)
    args.output.mkdir(parents=True, exist_ok=False)
    cv2.setNumThreads(1)
    environment = {'python': platform.python_version(), 'numpy': np.__version__, 'scipy': scipy.__version__, 'opencv': cv2.__version__}
    repository = Path(__file__).resolve().parents[2]
    implementation = {
        'files_sha256': {name: digest(Path(__file__).with_name(name)) for name in ('fit.py', 'model.py')},
        'repository_head': subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=repository, text=True).strip(),
        'worktree_dirty': bool(subprocess.check_output(['git', 'status', '--porcelain'], cwd=repository, text=True)),
    }
    for spec in config['datasets']:
        problem, seed, provenance = load(spec, Path(config['urdf']))
        priors = config['priors']
        problem.zero_sigma_rad = np.deg2rad(priors['zero_sigma_deg'])
        problem.axis_sigma_rad = np.deg2rad(priors['axis_sigma_deg'])
        problem.length_sigma_m = priors['length_sigma_m']
        problem.pixel_sigma_px = priors['pixel_sigma_px']
        scales = [problem.zero_sigma_rad, problem.axis_sigma_rad, problem.length_sigma_m, problem.pixel_sigma_px]
        if not np.isfinite(scales).all() or min(scales) <= 0:
            raise ValueError('Prior scales must be positive')
        print(f'Calibrating {provenance["side"]}', flush=True)
        result = calibrate(problem, seed, folds=config['training_folds'], max_evaluations=config['max_evaluations'])
        reference = provenance['reference_camera']
        mount = np.array(result['corrections']['wrist_T_camera'])
        result['camera_translation_change_from_reference_mm'] = float(np.linalg.norm(mount[:3, 3] - reference['xyz_m']) * 1000)
        reference_rotation = Rotation.from_euler('xyz', reference['rpy_rad']).as_matrix()
        result['camera_rotation_change_from_reference_deg'] = float(np.rad2deg(Rotation.from_matrix(reference_rotation.T @ mount[:3, :3]).magnitude()))
        result.update({'provenance': provenance, 'environment': environment, 'implementation': implementation, 'configuration': config, 'config_sha256': digest(args.config)})
        path = args.output / f'{provenance["side"]}.json'
        path.write_text(json.dumps(result, indent=2, allow_nan=False) + '\n')
        print(f'{provenance["side"]}: {result["selected_family"]}, held-out={result["held_out"]["rms_px"]:.3f}px -> {path}', flush=True)


if __name__ == '__main__':
    main()
