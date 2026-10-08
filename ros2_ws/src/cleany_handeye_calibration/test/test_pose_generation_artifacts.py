from dataclasses import replace
import hashlib
import json
from pathlib import Path

import pytest

from cleany_handeye_calibration.pose_generation_artifacts import (
    materialize_pose_generation_artifacts,
)
from cleany_handeye_calibration.pose_generation_profile import (
    load_mujoco_pose_generation_profile,
)
from cleany_handeye_calibration.pose_manifest import load_pose_manifest
from cleany_handeye_calibration.single_pose_runtime_config import (
    load_single_pose_runtime_config,
)
from pose_test_support import materialized_manifest


PACKAGE_ROOT = Path(__file__).parents[1]
REPOSITORY_ROOT = PACKAGE_ROOT.parents[2]
SCENE_PATH = (
    PACKAGE_ROOT.parent / 'cleany_mujoco_sim' / 'scenes' / 'handeye.xml.in'
)


def test_materializes_matching_manifest_runtime_and_urdf_atomically(tmp_path):
    profile = load_mujoco_pose_generation_profile(
        PACKAGE_ROOT / 'config' / 'pose_generation.mujoco.yaml'
    )
    manifest = replace(
        materialized_manifest(),
        run_config=profile.run_config,
    )
    output = tmp_path / 'profile'
    manifest_path, runtime_path, urdf_path = (
        materialize_pose_generation_artifacts(
            output_directory=output,
            artifact_root=tmp_path / 'runs',
            run_id='test_seed_20260810',
            manifest=manifest,
            profile=profile,
            repository_root=REPOSITORY_ROOT,
            scene_template_path=SCENE_PATH,
        )
    )

    assert len(load_pose_manifest(manifest_path).poses) == 25
    runtime = load_single_pose_runtime_config(runtime_path)
    assert runtime.motion.settle_position_tolerance_rad == 0.015
    assert runtime.dataset_manifest.source_hashes.pose_manifest_sha256 == (
        hashlib.sha256(manifest_path.read_bytes()).hexdigest()
    )
    assert '<robot name="cleany"' in urdf_path.read_text(encoding='utf-8')
    mapping = json.loads(runtime_path.read_text(encoding='utf-8'))
    assert mapping['dataset_manifest']['calibration_parameters'][
        'rotation_covariance_rank'
    ] == 3

    with pytest.raises(FileExistsError):
        materialize_pose_generation_artifacts(
            output_directory=output,
            artifact_root=tmp_path / 'runs',
            run_id='test_seed_20260810',
            manifest=manifest,
            profile=profile,
            repository_root=REPOSITORY_ROOT,
            scene_template_path=SCENE_PATH,
        )
