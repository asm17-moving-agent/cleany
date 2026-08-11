from __future__ import annotations

import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time
from uuid import uuid4

import pytest


if os.environ.get('ROS_DISTRO') is None:
    pytest.skip('ROS 2 environment is not active', allow_module_level=True)


PACKAGE_ROOT = Path(__file__).parents[1]
WORKSPACE_ROOT = PACKAGE_ROOT.parents[1]
REPOSITORY_ROOT = WORKSPACE_ROOT.parent
LEFT_JOINTS = (
    'left_shoulder_yaw_joint',
    'left_shoulder_pitch_joint',
    'left_elbow_pitch_joint',
    'left_wrist_pitch_joint',
    'left_wrist_roll_joint',
)
RIGHT_JOINTS = tuple(name.replace('left_', 'right_') for name in LEFT_JOINTS)
EXPECTED_RESOLVED = (
    -1.5767935884419453,
    0.7221886746271129,
    0.35912286260054327,
    0.9498367845587643,
    -1.1053229792363208,
)
TARGET_POSITION_M = (
    0.48213565748783654,
    0.15284906255858033,
    0.6377038732895058,
)
OBSERVED_TARGET_CLEARANCE_M = 0.1286505425794566


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _config(artifact_root: Path, run_id: str) -> dict:
    pose_payload = json.dumps(
        {
            'target_position_m': TARGET_POSITION_M,
            'ik_seed_positions_rad': EXPECTED_RESOLVED,
            'expected_resolved_positions_rad': EXPECTED_RESOLVED,
        },
        separators=(',', ':'),
        sort_keys=True,
    ).encode('ascii')
    commit = subprocess.run(
        ['git', '-C', str(REPOSITORY_ROOT), 'rev-parse', 'HEAD'],
        check=True,
        capture_output=True,
        text=True,
    ).stdout.strip()
    orchestration_timeouts = {
        'resolve_position_ik': 8.0,
        'validate_resolved_pose': 6.0,
        'plan': 12.0,
        'execute': 18.0,
        'wait_settled': 6.0,
        'acquire_image': 5.0,
        'detect_target': 2.0,
        'compute_feedback_fk': 6.0,
        'record_sample': 3.0,
    }
    return {
        'schema_version': 'cleany.single_pose_runtime/v1',
        'artifact_root': str(artifact_root),
        'sample': {
            'sample_id': 'sample_001',
            'pose_id': 'calibration_001',
            'split': 'calibration',
            'target_position_m': list(TARGET_POSITION_M),
            'ik_seed_positions_rad': list(EXPECTED_RESOLVED),
        },
        'safety_profile': {
            'profile_id': 'mujoco_e2e_measured_clearance_v1',
            'soft_joint_limits_rad': {
                'left_shoulder_yaw_joint': [-2.11, 2.11],
                'left_shoulder_pitch_joint': [-0.17, 3.32],
                'left_elbow_pitch_joint': [-0.17, 3.09],
                'left_wrist_pitch_joint': [-0.3245, 1.608],
                'left_wrist_roll_joint': [-2.693, 2.791],
            },
            'collision_margin_m': 0.10,
        },
        'expected_resolved_evidence': {
            'joint_positions_rad': list(EXPECTED_RESOLVED),
            'match_tolerance_rad': 1.0e-4,
            'collision_clearance_m': OBSERVED_TARGET_CLEARANCE_M,
        },
        'motion': {
            'current_state_max_age_sec': 0.5,
            'right_park_position_tolerance_rad': 0.01,
            'stage_timeouts_sec': {
                'ik': 6.0,
                'state_validity': 4.0,
                'plan': 10.0,
                'execute': 15.0,
                'cancel': 2.0,
                'settle': 5.0,
            },
            'max_velocity_scaling_factor': 0.1,
            'max_acceleration_scaling_factor': 0.1,
            'controller_path_tolerance_rad': 0.05,
            'controller_goal_tolerance_rad': 0.01,
            # Repeated runtime exploration observed up to 0.010403 rad of
            # MuJoCo gravity-loaded steady-state error. Keep 0.015 rad as an
            # explicit simulation-only threshold; velocity and the one-second
            # duration gates remain unchanged.
            'settle_position_tolerance_rad': 0.015,
            'settle_velocity_tolerance_rad_s': 0.01,
            'settle_duration_sec': 1.0,
            'planning_attempts': 1,
        },
        'orchestration_timeouts_sec': orchestration_timeouts,
        'feedback_buffer': {
            'capacity': 512,
            'max_sample_distance_ns': 80_000_000,
            'clock_reset_threshold_ns': 500_000_000,
            'startup_state_timeout_sec': 90.0,
            'startup_planning_scene_timeout_sec': 90.0,
        },
        'dataset_manifest': {
            'run_id': run_id,
            'git': {'commit': commit, 'dirty': True},
            'source_hashes': {
                'urdf_sha256': _sha256(
                    REPOSITORY_ROOT
                    / 'ros2_ws/src/cleany_description/urdf/'
                    'cleany_control.urdf.xacro'
                ),
                'mjcf_sha256': _sha256(
                    REPOSITORY_ROOT
                    / 'ros2_ws/src/cleany_mujoco_sim/scenes/'
                    'handeye.xml.in'
                ),
                'pose_manifest_sha256': hashlib.sha256(
                    pose_payload
                ).hexdigest(),
            },
            'software_versions': {
                'ros_distro': 'humble',
                'moveit': '2.5.9',
                'opencv': '4.5.4',
                'mujoco': '3.4.0',
                'mujoco_ros2_control': '0.0.3',
                'vendor': {
                    'mujoco_ros2_control_camera_core': '0.0.3',
                    'opencv-contrib': '4.5.4',
                },
            },
            'target': {
                'board_svg_sha256': (
                    '63284e29c1e359d90018c2138c6017879057b99811e7fa72a8'
                    'fcc0b924d87d10'
                ),
                'board_pdf_sha256': (
                    '614c2447b55101e0147851ce6a11a61a4013f0e34f15a6e439'
                    '9aea764c213959'
                ),
                'size_provenance': 'simulation_manifest_exact_geometry',
            },
            'timing': {
                'simulation_timestep_s': 0.002,
                'controller_update_rate_hz': 50.0,
                'image_rate_hz': 10.0,
                'joint_state_rate_hz': 50.0,
            },
            'calibration_parameters': {
                'motion': {
                    'velocity_scaling': 0.1,
                    'acceleration_scaling': 0.1,
                },
                'settle': {
                    'position_rad': 0.015,
                    'velocity_rad_s': 0.01,
                    'duration_sec': 1.0,
                },
                'safety_profile': {
                    'id': 'mujoco_e2e_measured_clearance_v1',
                    'required_collision_margin_m': 0.10,
                    'observed_collision_clearance_m': (
                        OBSERVED_TARGET_CLEARANCE_M
                    ),
                },
                'orchestration_timeouts_sec': orchestration_timeouts,
            },
            'random_seed': 20260810,
        },
    }


def _log_tail(path: Path, lines: int = 220) -> str:
    try:
        return '\n'.join(
            path.read_text(encoding='utf-8', errors='replace').splitlines()[
                -lines:
            ]
        )
    except OSError as error:
        return f'<cannot read log: {error}>'


def _stop_process_group(process: subprocess.Popen[bytes]) -> None:
    if process.poll() is not None:
        return
    for sig, timeout_sec in (
        (signal.SIGINT, 8.0),
        (signal.SIGTERM, 4.0),
        (signal.SIGKILL, 2.0),
    ):
        try:
            os.killpg(process.pid, sig)
        except ProcessLookupError:
            return
        try:
            process.wait(timeout=timeout_sec)
            return
        except subprocess.TimeoutExpired:
            continue


def test_single_pose_launch_records_feedback_timed_mujoco_sample() -> None:
    run_id = f'e2e_{uuid4().hex[:10]}'
    domain_id = 120 + (os.getpid() % 80)
    with tempfile.TemporaryDirectory(
        prefix='cleany_single_pose_e2e_'
    ) as temporary:
        root = Path(temporary)
        artifact_root = root / 'artifacts'
        request_path = root / 'single_pose_request.json'
        request_path.write_text(
            json.dumps(_config(artifact_root, run_id), indent=2),
            encoding='utf-8',
        )
        log_path = root / 'launch.log'
        environment = os.environ.copy()
        environment['ROS_DOMAIN_ID'] = str(domain_id)
        environment['ROS_HOME'] = str(root / 'ros_home')
        environment['RCUTILS_COLORIZED_OUTPUT'] = '0'
        (root / 'ros_home').mkdir()

        with log_path.open('wb') as log_stream:
            process = subprocess.Popen(
                [
                    'ros2',
                    'launch',
                    'cleany_handeye_calibration',
                    'single_pose_mujoco.launch.py',
                    f'request_file:={request_path}',
                    'headless:=true',
                ],
                cwd=root,
                env=environment,
                stdout=log_stream,
                stderr=subprocess.STDOUT,
                start_new_session=True,
            )
        run_directory = artifact_root / run_id
        samples_path = run_directory / 'samples.jsonl'
        journal_path = run_directory / 'orchestration.jsonl'
        deadline = time.monotonic() + 130.0
        try:
            while time.monotonic() < deadline:
                return_code = process.poll()
                if return_code is not None:
                    pytest.fail(
                        'single-pose launch exited before producing a sample '
                        f'(code {return_code})\n{_log_tail(log_path)}'
                    )
                log_tail = _log_tail(log_path)
                if 'single-pose calibration startup failed:' in log_tail:
                    pytest.fail(
                        'single-pose orchestrator failed during startup\n'
                        + log_tail
                    )
                if samples_path.is_file() and samples_path.stat().st_size:
                    break
                if journal_path.is_file():
                    rows = [
                        json.loads(line)
                        for line in journal_path.read_text().splitlines()
                        if line
                    ]
                    if rows and rows[-1]['status'] == 'failed':
                        pytest.fail(
                            'single-pose stage failed: '
                            f'{rows[-1]}\n{_log_tail(log_path)}'
                        )
                time.sleep(0.1)
            else:
                pytest.fail(
                    'timed out waiting for the single-pose sample\n'
                    + _log_tail(log_path)
                )

            samples = [
                json.loads(line)
                for line in samples_path.read_text().splitlines()
                if line
            ]
            assert len(samples) == 1
            sample = samples[0]
            assert sample['sample_id'] == 'sample_001'
            assert sample['pose_id'] == 'calibration_001'
            assert sample['split'] == 'calibration'
            assert sample['image_stamp_ns'] > 0
            assert (
                sample['joint_state_before_stamp_ns']
                <= sample['image_stamp_ns']
                <= sample['joint_state_after_stamp_ns']
            )
            assert (
                sample['joint_state_before_stamp_ns']
                < sample['joint_state_after_stamp_ns']
            )
            assert len(sample['joint_names']) == 12
            positions = dict(
                zip(
                    sample['joint_names'],
                    sample['joint_positions_rad'],
                    strict=True,
                )
            )
            for name, expected in zip(
                LEFT_JOINTS, EXPECTED_RESOLVED, strict=True
            ):
                assert positions[name] == pytest.approx(expected, abs=0.015)
            for name in RIGHT_JOINTS:
                assert positions[name] == pytest.approx(0.0, abs=0.01)
            assert sample['base_to_gripper']['parent_frame'] == 'base_link'
            assert (
                sample['base_to_gripper']['child_frame']
                == 'left_gripper_frame'
            )
            assert (
                sample['camera_to_target']['parent_frame']
                == 'left_wrist_rgb_optical_frame'
            )
            assert (
                sample['camera_to_target']['child_frame']
                == 'charuco_target'
            )
            assert len(
                sample['target_detection']['corner_ids']
            ) >= 16
            assert set(
                sample['target_detection']['covered_quadrants']
            ) == {'top_left', 'top_right', 'bottom_left', 'bottom_right'}
            assert sample['pnp']['ambiguous'] is False
            assert sample['pnp']['reprojection_rmse_px'] < 2.0
            image_path = run_directory / sample['image_path']
            assert image_path.is_file()
            assert image_path.read_bytes().startswith(b'\x89PNG\r\n\x1a\n')

            journal = [
                json.loads(line)
                for line in journal_path.read_text().splitlines()
                if line
            ]
            expected_stages = [
                'resolve_position_ik',
                'validate_resolved_pose',
                'plan',
                'execute',
                'wait_settled',
                'acquire_image',
                'detect_target',
                'compute_feedback_fk',
                'record_sample',
            ]
            assert [row['stage'] for row in journal] == [
                stage for stage in expected_stages for _ in range(2)
            ]
            assert [row['status'] for row in journal] == [
                status
                for _ in expected_stages
                for status in ('started', 'succeeded')
            ]
        finally:
            _stop_process_group(process)
