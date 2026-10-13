"""Atomic pose-manifest and matching runtime-profile materialization."""

from __future__ import annotations

import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
import cv2
import mujoco
import numpy as np
import xacro

from cleany_handeye_calibration.pose_generation_profile import (
    MujocoPoseGenerationProfile,
)
from cleany_handeye_calibration.pose_manifest import (
    PoseManifest,
    write_pose_manifest,
)


POSE_MANIFEST_NAME = 'materialized_poses.yaml'
RUNTIME_CONFIG_NAME = 'materialized_runtime.json'
URDF_NAME = 'cleany_handeye.urdf'


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _combined_sha256(paths: tuple[Path, ...]) -> str:
    digest = hashlib.sha256()
    for path in paths:
        payload = path.read_bytes()
        digest.update(len(payload).to_bytes(8, byteorder='big'))
        digest.update(payload)
    return digest.hexdigest()


def _package_version(package_name: str) -> str:
    package_xml = (
        Path(get_package_share_directory(package_name)) / 'package.xml'
    )
    root = ET.fromstring(package_xml.read_text(encoding='utf-8'))
    version = root.findtext('version')
    if version is None or not version.strip():
        raise ValueError(f'{package_name} package version is unavailable')
    return version.strip()


def _git_provenance(repository_root: Path) -> tuple[str, bool]:
    commit = subprocess.run(
        ['git', 'rev-parse', 'HEAD'],
        cwd=repository_root,
        check=True,
        capture_output=True,
        text=True,
    ).stdout.strip()
    status = subprocess.run(
        ['git', 'status', '--porcelain'],
        cwd=repository_root,
        check=True,
        capture_output=True,
        text=True,
    ).stdout
    return commit, bool(status.strip())


def _materialized_urdf() -> str:
    description_share = Path(
        get_package_share_directory('cleany_description')
    )
    return xacro.process_file(
        str(description_share / 'urdf' / 'cleany.urdf.xacro')
    ).toxml()


def _runtime_mapping(
    *,
    manifest: PoseManifest,
    profile: MujocoPoseGenerationProfile,
    artifact_root: Path,
    run_id: str,
    pose_manifest_sha256: str,
    urdf_sha256: str,
    mjcf_sha256: str,
    repository_root: Path,
) -> dict:
    first = manifest.poses[0]
    run = manifest.run_config.require_ready()
    limits = run.soft_joint_limits
    assert limits is not None
    timeout = run.stage_timeouts
    commit, dirty = _git_provenance(repository_root)
    sim_share = Path(get_package_share_directory('cleany_mujoco_sim'))
    svg = sim_share / 'assets' / 'charuco_7x5_30mm_15mm_dict_5x5_100.svg'
    pdf = sim_share / 'assets' / 'charuco_7x5_30mm_15mm_dict_5x5_100.pdf'
    motion = profile.motion
    return {
        'schema_version': 'cleany.single_pose_runtime/v1',
        'artifact_root': str(artifact_root),
        'sample': {
            'sample_id': first.pose_id,
            'pose_id': first.pose_id,
            'split': first.split.value,
            'target_position_m': list(first.target.position_m),
            'ik_seed_positions_rad': list(first.ik_seed.positions_rad),
        },
        'safety_profile': {
            'profile_id': (
                f'mujoco_random_seed_{manifest.generator.random_seed}'
            ),
            'soft_joint_limits_rad': {
                name: [lower, upper]
                for name, lower, upper in zip(
                    limits.joint_names,
                    limits.lower_rad,
                    limits.upper_rad,
                    strict=True,
                )
            },
            'collision_margin_m': run.collision_margin_m,
        },
        'expected_resolved_evidence': {
            'joint_positions_rad': list(
                first.resolved_joint_pose.positions_rad
            ),
            'match_tolerance_rad': (
                profile.expected_resolved_match_tolerance_rad
            ),
            'collision_clearance_m': (
                first.validation.minimum_collision_distance_m
            ),
        },
        'motion': {
            'current_state_max_age_sec': motion.current_state_max_age_sec,
            'right_park_position_tolerance_rad': (
                motion.right_park_position_tolerance_rad
            ),
            'stage_timeouts_sec': {
                'ik': motion.stage_timeouts.ik_sec,
                'state_validity': motion.stage_timeouts.state_validity_sec,
                'plan': motion.stage_timeouts.plan_sec,
                'execute': motion.stage_timeouts.execute_sec,
                'cancel': motion.stage_timeouts.cancel_sec,
                'settle': motion.stage_timeouts.settle_sec,
            },
            'max_velocity_scaling_factor': (
                motion.max_velocity_scaling_factor
            ),
            'max_acceleration_scaling_factor': (
                motion.max_acceleration_scaling_factor
            ),
            'controller_path_tolerance_rad': (
                motion.controller_path_tolerance_rad
            ),
            'controller_goal_tolerance_rad': (
                motion.controller_goal_tolerance_rad
            ),
            'settle_position_tolerance_rad': (
                motion.settle_position_tolerance_rad
            ),
            'settle_velocity_tolerance_rad_s': (
                motion.settle_velocity_tolerance_rad_s
            ),
            'settle_duration_sec': motion.settle_duration_sec,
            'planning_attempts': motion.planning_attempts,
        },
        'orchestration_timeouts_sec': {
            'resolve_position_ik': timeout.ik_sec,
            'validate_resolved_pose': timeout.state_validity_sec,
            'plan': timeout.plan_sec,
            'execute': timeout.execute_sec,
            'wait_settled': timeout.settle_sec,
            'acquire_image': timeout.image_acquisition_sec,
            'detect_target': timeout.target_detection_sec,
            'compute_feedback_fk': timeout.feedback_fk_sec,
            'record_sample': timeout.record_sample_sec,
        },
        'feedback_buffer': {
            'capacity': profile.feedback.capacity,
            'max_sample_distance_ns': (
                profile.feedback.max_sample_distance_ns
            ),
            'clock_reset_threshold_ns': (
                profile.feedback.clock_reset_threshold_ns
            ),
            'startup_state_timeout_sec': (
                profile.feedback.startup_state_timeout_sec
            ),
            'startup_planning_scene_timeout_sec': (
                profile.feedback.startup_planning_scene_timeout_sec
            ),
        },
        'dataset_manifest': {
            'run_id': run_id,
            'git': {'commit': commit, 'dirty': dirty},
            'source_hashes': {
                'urdf_sha256': urdf_sha256,
                'mjcf_sha256': mjcf_sha256,
                'pose_manifest_sha256': pose_manifest_sha256,
            },
            'software_versions': {
                'ros_distro': os.environ.get('ROS_DISTRO', 'humble'),
                'moveit': _package_version('moveit_ros_move_group'),
                'opencv': cv2.__version__,
                'mujoco': mujoco.__version__,
                'mujoco_ros2_control': _package_version(
                    'mujoco_ros2_control'
                ),
                'vendor': {'numpy': np.__version__},
            },
            'target': {
                'board_svg_sha256': _sha256(svg),
                'board_pdf_sha256': _sha256(pdf),
                'size_provenance': 'simulation_manifest_exact_geometry',
            },
            'timing': {
                'simulation_timestep_s': 0.002,
                'controller_update_rate_hz': 50.0,
                'image_rate_hz': 10.0,
                'joint_state_rate_hz': 50.0,
            },
            'calibration_parameters': {
                'pose_selection_strategy': manifest.selection.strategy,
                'candidate_pool_size': manifest.generator.candidate_pool_size,
                'maximum_axis_parallelism': (
                    manifest.selection.diversity.maximum_axis_parallelism
                ),
                'rotation_covariance_log_det': (
                    manifest.selection.diversity.rotation_covariance_log_det
                ),
                'rotation_covariance_rank': (
                    manifest.selection.diversity.rotation_covariance_rank
                ),
                'settle_position_tolerance_rad': (
                    motion.settle_position_tolerance_rad
                ),
                'mjcf_hash_scope': (
                    'canonical_robot_plus_handeye_scene_template'
                ),
            },
            'random_seed': manifest.generator.random_seed,
        },
    }


def materialize_pose_generation_artifacts(
    *,
    output_directory: str | Path,
    artifact_root: str | Path,
    run_id: str,
    manifest: PoseManifest,
    profile: MujocoPoseGenerationProfile,
    repository_root: str | Path,
    scene_template_path: str | Path,
) -> tuple[Path, Path, Path]:
    if manifest.run_config != profile.run_config:
        raise ValueError(
            'pose manifest and generation profile run configuration differ'
        )
    destination = Path(output_directory).expanduser()
    root = Path(artifact_root).expanduser()
    repository = Path(repository_root).expanduser().resolve(strict=True)
    scene_template = (
        Path(scene_template_path).expanduser().resolve(strict=True)
    )
    if not destination.is_absolute() or not root.is_absolute():
        raise ValueError('output_directory and artifact_root must be absolute')
    destination = destination.resolve()
    root = root.resolve()
    if destination.exists():
        raise FileExistsError(
            f'pose artifact directory already exists: {destination}'
        )
    destination.parent.mkdir(parents=True, exist_ok=True)
    temporary = Path(
        tempfile.mkdtemp(
            prefix=f'.{destination.name}.',
            dir=destination.parent,
        )
    )
    try:
        manifest_path = temporary / POSE_MANIFEST_NAME
        urdf_path = temporary / URDF_NAME
        runtime_path = temporary / RUNTIME_CONFIG_NAME
        write_pose_manifest(manifest_path, manifest)
        urdf_path.write_text(_materialized_urdf(), encoding='utf-8')

        description_share = Path(
            get_package_share_directory('cleany_description')
        )
        canonical_mjcf = description_share / 'mjcf' / 'cleany.xml'
        runtime = _runtime_mapping(
            manifest=manifest,
            profile=profile,
            artifact_root=root,
            run_id=run_id,
            pose_manifest_sha256=_sha256(manifest_path),
            urdf_sha256=_sha256(urdf_path),
            mjcf_sha256=_combined_sha256(
                (canonical_mjcf, scene_template)
            ),
            repository_root=repository,
        )
        runtime_path.write_text(
            json.dumps(runtime, indent=2, sort_keys=True, allow_nan=False)
            + '\n',
            encoding='utf-8',
        )
        for path in (manifest_path, urdf_path, runtime_path):
            with path.open('rb') as stream:
                os.fsync(stream.fileno())
        os.replace(temporary, destination)
        directory_fd = os.open(destination.parent, os.O_RDONLY)
        try:
            os.fsync(directory_fd)
        finally:
            os.close(directory_fd)
    except BaseException:
        shutil.rmtree(temporary, ignore_errors=True)
        raise
    return (
        destination / POSE_MANIFEST_NAME,
        destination / RUNTIME_CONFIG_NAME,
        destination / URDF_NAME,
    )


__all__ = [
    'POSE_MANIFEST_NAME',
    'RUNTIME_CONFIG_NAME',
    'URDF_NAME',
    'materialize_pose_generation_artifacts',
]
