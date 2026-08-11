from __future__ import annotations

from dataclasses import replace
import json

import cv2
import numpy as np
import pytest

from cleany_handeye_calibration.camera_acquisition import (
    CAMERA_D,
    CAMERA_DISTORTION_MODEL,
    CAMERA_FRAME_ID,
    CAMERA_HEIGHT,
    CAMERA_K,
    CAMERA_P,
    CAMERA_R,
    CAMERA_WIDTH,
    CameraFramePair,
    CameraInfoFrame,
    DEFAULT_CAMERA_CONTRACT,
    ImageFrame,
)
from cleany_handeye_calibration.dataset_writer import (
    CaptureTiming,
    DatasetCorruptionError,
    DatasetManifestV1,
    DatasetWriter,
    DuplicateSampleError,
    GitProvenance,
    SoftwareVersions,
    SourceArtifactHashes,
    TargetDatasetContract,
    manifest_to_mapping,
    mapping_sha256,
    sha256_file,
)
from cleany_handeye_calibration.joint_state_sync import (
    DEFAULT_DUAL_ARM_JOINT_CONTRACT,
    LEFT_ARM_JOINT_NAMES,
)
from cleany_handeye_calibration.models import (
    CalibrationSample,
    JointPose,
    PositionTarget,
    SampleSplit,
    TimedJointSample,
)
from cleany_handeye_calibration.schema import (
    CalibrationSampleRecord,
    sample_record_from_mapping,
)
from cleany_handeye_calibration.target_detector import (
    INNER_CORNER_COUNT,
    analyze_charuco_corners,
)
from cleany_handeye_calibration.transforms import RigidTransform


SHA_A = 'a' * 64
SHA_B = 'b' * 64
SHA_C = 'c' * 64
SHA_D = 'd' * 64
SHA_E = 'e' * 64
JOINT_NAMES = DEFAULT_DUAL_ARM_JOINT_CONTRACT.required_joint_names


def _manifest(run_id: str = 'run_001') -> DatasetManifestV1:
    return DatasetManifestV1(
        run_id=run_id,
        git=GitProvenance(
            commit='0123456789abcdef0123456789abcdef01234567',
            dirty=True,
        ),
        source_hashes=SourceArtifactHashes(
            urdf_sha256=SHA_A,
            mjcf_sha256=SHA_B,
            pose_manifest_sha256=SHA_C,
        ),
        software=SoftwareVersions(
            ros_distro='humble',
            moveit='2.5.9',
            opencv='4.5.4',
            mujoco='3.4.0',
            mujoco_ros2_control='0.0.3',
            vendor_versions={
                'mujoco_ros2_control_camera_core': '0.0.3',
                'opencv-contrib': '4.5.4',
            },
        ),
        camera=DEFAULT_CAMERA_CONTRACT,
        camera_vertical_fov_degrees=93.0,
        target=TargetDatasetContract(
            board_svg_sha256=SHA_D,
            board_pdf_sha256=SHA_E,
            size_provenance='simulation_manifest_exact_geometry',
        ),
        timing=CaptureTiming(
            simulation_timestep_s=0.002,
            controller_update_rate_hz=50.0,
            image_rate_hz=30.0,
            joint_state_rate_hz=50.0,
        ),
        calibration_parameters={
            'motion': {
                'max_velocity_scaling': 0.1,
                'max_acceleration_scaling': 0.1,
            },
            'settle': {
                'position_error_rad': 0.005,
                'velocity_rad_s': 0.01,
                'duration_sec': 1.0,
            },
            'timeouts_sec': {
                'image': 2.0,
                'fk': 1.0,
            },
        },
        random_seed=20260810,
    )


def _detection():
    return analyze_charuco_corners(
        tuple(range(INNER_CORNER_COUNT)),
        tuple(
            (110.0 + (index % 6) * 35.0, 90.0 + (index // 6) * 45.0)
            for index in range(INNER_CORNER_COUNT)
        ),
    )


def _record_and_pair(
    sample_id: str,
    stamp_ns: int,
    *,
    color_rgb: tuple[int, int, int],
) -> tuple[CalibrationSampleRecord, CameraFramePair]:
    camera_info = CameraInfoFrame(
        stamp_ns=stamp_ns,
        frame_id=CAMERA_FRAME_ID,
        width=CAMERA_WIDTH,
        height=CAMERA_HEIGHT,
        distortion_model=CAMERA_DISTORTION_MODEL,
        d=CAMERA_D,
        k=CAMERA_K,
        r=CAMERA_R,
        p=CAMERA_P,
    )
    image = ImageFrame(
        stamp_ns=stamp_ns,
        frame_id=CAMERA_FRAME_ID,
        width=CAMERA_WIDTH,
        height=CAMERA_HEIGHT,
        encoding='rgb8',
        is_bigendian=False,
        step=CAMERA_WIDTH * 3,
        data=bytes(color_rgb) * (CAMERA_WIDTH * CAMERA_HEIGHT),
    )
    pair = CameraFramePair(image=image, camera_info=camera_info)
    sample = CalibrationSample(
        sample_id=sample_id,
        pose_id=f'pose_{sample_id}',
        split=SampleSplit.CALIBRATION,
        base_T_gripper=RigidTransform.from_rodrigues(
            parent_frame='base_link',
            child_frame='left_gripper_frame',
            translation_m=(0.3, 0.2, 0.5),
            rodrigues_vector=(0.1, -0.2, 0.3),
        ),
        camera_T_target=RigidTransform.from_quaternion_xyzw(
            parent_frame=CAMERA_FRAME_ID,
            child_frame='charuco_target',
            translation_m=(0.01, -0.02, 0.4),
            quaternion_xyzw=(0.0, 0.0, 0.0, 1.0),
        ),
    )
    record = CalibrationSampleRecord(
        sample=sample,
        calibration_arm='left',
        planning_group='left_arm',
        position_target=PositionTarget('base_link', (0.3, 0.2, 0.5)),
        ik_seed=JointPose(LEFT_ARM_JOINT_NAMES, (0.0,) * 5),
        resolved_ik=JointPose(
            LEFT_ARM_JOINT_NAMES,
            (0.1, 0.2, 0.3, 0.4, 0.5),
        ),
        image_stamp_ns=stamp_ns,
        joint_state_before_stamp_ns=stamp_ns - 100,
        joint_state_after_stamp_ns=stamp_ns + 100,
        joint_interpolation_ratio=0.5,
        interpolated_joints=TimedJointSample(
            stamp_ns=stamp_ns,
            joint_names=JOINT_NAMES,
            positions_rad=tuple(index * 0.01 for index in range(12)),
            velocities_rad_s=(0.0,) * 12,
        ),
        camera_info=camera_info,
        target_detection=_detection(),
        pnp_method='SOLVEPNP_IPPE',
        pnp_reprojection_rmse_px=0.125,
        pnp_ambiguous=False,
        pnp_selected_candidate_index=0,
        image_path=f'images/{sample_id}.png',
    )
    return record, pair


def test_manifest_v1_records_all_reproducibility_inputs_and_own_hash():
    mapping = manifest_to_mapping(_manifest())

    assert mapping['schema_version'] == 1
    assert mapping['git']['dirty'] is True
    assert mapping['source_hashes'] == {
        'urdf_sha256': SHA_A,
        'mjcf_sha256': SHA_B,
        'pose_manifest_sha256': SHA_C,
    }
    assert set(mapping['software_versions']) == {
        'ros_distro',
        'moveit',
        'opencv',
        'mujoco',
        'mujoco_ros2_control',
        'vendor',
    }
    assert mapping['camera']['width'] == 640
    assert mapping['camera']['height'] == 480
    assert mapping['camera']['vertical_fov_degrees'] == 93.0
    assert mapping['camera']['computed_focal_length_px'] == pytest.approx(
        CAMERA_K[4],
        abs=1.0e-6,
    )
    assert 'vertical_fov_rad' in mapping['camera']['focal_length_formula']
    assert mapping['camera']['K'] == list(CAMERA_K)
    assert mapping['camera']['D'] == list(CAMERA_D)
    assert mapping['target']['point_ordering'] == (
        'charuco_corner_id_ascending'
    )
    assert mapping['target']['square_length_m_used'] == 0.03
    assert mapping['target']['marker_length_m_used'] == 0.015
    assert mapping['timing']['simulation_timestep_s'] == 0.002
    assert mapping['calibration']['random_seed'] == 20260810
    body = dict(mapping)
    stored_hash = body.pop('manifest_sha256')
    assert stored_hash == mapping_sha256(body)


def test_run_id_cannot_escape_the_configured_artifact_root():
    with pytest.raises(ValueError, match='run_id must contain only'):
        replace(_manifest(), run_id='../outside')


def test_writer_persists_lossless_image_and_complete_jsonl_row(tmp_path):
    writer = DatasetWriter(artifact_root=tmp_path, manifest=_manifest())
    record, pair = _record_and_pair(
        'sample_001',
        1_500,
        color_rgb=(10, 20, 30),
    )

    stored = writer.append_sample(record, pair)
    restored = writer.read_samples()

    assert len(restored) == 1
    assert restored[0].record.sample.sample_id == record.sample.sample_id
    assert restored[0].image_sha256 == stored.image_sha256
    assert restored[0].source_image_sha256 == stored.source_image_sha256
    assert restored[0].record_sha256 == stored.record_sha256
    assert writer.manifest_path.name == 'manifest.yaml'
    assert writer.samples_path.name == 'samples.jsonl'
    image_path = writer.run_directory / record.image_path
    assert image_path.is_file()
    assert sha256_file(image_path) == stored.image_sha256
    decoded = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
    np.testing.assert_array_equal(decoded[0, 0], (30, 20, 10))

    lines = writer.samples_path.read_text(encoding='ascii').splitlines()
    assert len(lines) == 1
    mapping = json.loads(lines[0])
    parsed_record = sample_record_from_mapping(mapping)
    assert parsed_record.sample.sample_id == record.sample.sample_id
    np.testing.assert_allclose(
        parsed_record.sample.base_T_gripper.as_homogeneous_matrix(),
        record.sample.base_T_gripper.as_homogeneous_matrix(),
        rtol=0.0,
        atol=1.0e-12,
    )
    assert parsed_record.target_detection == record.target_detection
    assert mapping['image_sha256'] == stored.image_sha256
    assert mapping['source_image_sha256'] == stored.source_image_sha256
    assert mapping['record_sha256'] == stored.record_sha256
    assert len(mapping['joint_names']) == 12
    assert mapping['camera_calibration']['K'] == list(CAMERA_K)
    assert mapping['camera_calibration']['D'] == list(CAMERA_D)
    assert len(mapping['target_detection']['corner_ids']) == 24
    assert len(mapping['target_detection']['object_points_m']) == 24
    assert len(mapping['target_detection']['image_points_px']) == 24


def test_writer_archives_every_attempt_image_before_sample_commit(tmp_path):
    manifest = _manifest()
    writer = DatasetWriter(artifact_root=tmp_path, manifest=manifest)
    _, pair = _record_and_pair(
        'sample_001',
        1_500,
        color_rgb=(10, 20, 30),
    )

    first = writer.archive_attempt_image(
        pose_id='calibration_001',
        attempt=1,
        pair=pair,
    )
    second = writer.archive_attempt_image(
        pose_id='calibration_001',
        attempt=2,
        pair=pair,
    )

    assert (first.sequence, second.sequence) == (1, 2)
    assert writer.read_samples() == ()
    images = sorted(writer.attempt_images_directory.glob('*.png'))
    metadata = sorted(writer.attempt_images_directory.glob('*.json'))
    assert len(images) == len(metadata) == 2
    decoded = cv2.imread(str(images[0]), cv2.IMREAD_COLOR)
    np.testing.assert_array_equal(decoded[0, 0], (30, 20, 10))
    first_metadata = json.loads(metadata[0].read_text(encoding='ascii'))
    assert first_metadata['pose_id'] == 'calibration_001'
    assert first_metadata['attempt'] == 1
    assert first_metadata['image_stamp_ns'] == 1_500
    assert first_metadata['png_sha256'] == sha256_file(images[0])

    reopened = DatasetWriter(artifact_root=tmp_path, manifest=manifest)
    assert len(tuple(reopened.attempt_images_directory.glob('*.png'))) == 2


def test_writer_refuses_duplicate_sample_ids(tmp_path):
    writer = DatasetWriter(artifact_root=tmp_path, manifest=_manifest())
    record, pair = _record_and_pair(
        'sample_001',
        1_500,
        color_rgb=(1, 2, 3),
    )
    writer.append_sample(record, pair)

    with pytest.raises(DuplicateSampleError):
        writer.append_sample(record, pair)


@pytest.mark.parametrize(
    ('failure_stage', 'recovered_ids'),
    [
        ('after_image_commit', ('sample_001',)),
        ('after_journal_commit', ('sample_001', 'sample_002')),
        ('after_samples_commit', ('sample_001', 'sample_002')),
    ],
)
def test_recovery_preserves_previous_rows_without_dangling_image_references(
    tmp_path,
    failure_stage,
    recovered_ids,
):
    manifest = _manifest()
    writer = DatasetWriter(artifact_root=tmp_path, manifest=manifest)
    first_record, first_pair = _record_and_pair(
        'sample_001',
        1_500,
        color_rgb=(1, 2, 3),
    )
    writer.append_sample(first_record, first_pair)

    def fail_at(stage: str) -> None:
        if stage == failure_stage:
            raise RuntimeError(f'injected failure at {stage}')

    interrupted = DatasetWriter(
        artifact_root=tmp_path,
        manifest=manifest,
        fault_hook=fail_at,
    )
    second_record, second_pair = _record_and_pair(
        'sample_002',
        2_500,
        color_rgb=(4, 5, 6),
    )
    with pytest.raises(RuntimeError, match='injected failure'):
        interrupted.append_sample(second_record, second_pair)

    recovered = DatasetWriter(
        artifact_root=tmp_path,
        manifest=manifest,
    )
    samples = recovered.read_samples()

    assert tuple(sample.record.sample.sample_id for sample in samples) == (
        recovered_ids
    )
    for sample in samples:
        image = recovered.run_directory / sample.record.image_path
        assert image.is_file()
        assert sha256_file(image) == sample.image_sha256
    assert not tuple((recovered.run_directory / '.journal').glob('*.json'))
    if failure_stage == 'after_image_commit':
        assert not (
            recovered.run_directory / second_record.image_path
        ).exists()


def test_reader_fails_closed_when_a_committed_image_is_missing(tmp_path):
    manifest = _manifest()
    writer = DatasetWriter(artifact_root=tmp_path, manifest=manifest)
    record, pair = _record_and_pair(
        'sample_001',
        1_500,
        color_rgb=(1, 2, 3),
    )
    writer.append_sample(record, pair)
    (writer.run_directory / record.image_path).unlink()

    with pytest.raises(DatasetCorruptionError, match='image is missing'):
        DatasetWriter(artifact_root=tmp_path, manifest=manifest)


def test_writer_rejects_camera_or_record_mismatch_before_writing(tmp_path):
    writer = DatasetWriter(artifact_root=tmp_path, manifest=_manifest())
    record, pair = _record_and_pair(
        'sample_001',
        1_500,
        color_rgb=(1, 2, 3),
    )
    other_info = replace(pair.camera_info, stamp_ns=1_501)
    mismatched_pair = CameraFramePair(
        image=replace(pair.image, stamp_ns=1_501),
        camera_info=other_info,
    )

    with pytest.raises(ValueError, match='pair stamp'):
        writer.append_sample(record, mismatched_pair)

    assert writer.read_samples() == ()
    assert not tuple((writer.run_directory / 'images').glob('*.png'))


def test_external_artifact_root_does_not_modify_source_inputs(tmp_path):
    source_dir = tmp_path / 'source'
    source_dir.mkdir()
    urdf = source_dir / 'robot.urdf'
    mjcf = source_dir / 'scene.xml'
    pose_manifest = source_dir / 'poses.yaml'
    urdf.write_text('<robot/>', encoding='utf-8')
    mjcf.write_text('<mujoco/>', encoding='utf-8')
    pose_manifest.write_text('poses: []\n', encoding='utf-8')
    before = {
        path: (path.read_bytes(), sha256_file(path))
        for path in (urdf, mjcf, pose_manifest)
    }
    manifest = replace(
        _manifest(),
        source_hashes=SourceArtifactHashes(
            urdf_sha256=before[urdf][1],
            mjcf_sha256=before[mjcf][1],
            pose_manifest_sha256=before[pose_manifest][1],
        ),
    )

    writer = DatasetWriter(
        artifact_root=tmp_path / 'external-artifacts',
        manifest=manifest,
    )
    record, pair = _record_and_pair(
        'sample_001',
        1_500,
        color_rgb=(1, 2, 3),
    )
    writer.append_sample(record, pair)

    assert {
        path: (path.read_bytes(), sha256_file(path))
        for path in before
    } == before
    assert writer.run_directory.is_relative_to(
        (tmp_path / 'external-artifacts').resolve()
    )
