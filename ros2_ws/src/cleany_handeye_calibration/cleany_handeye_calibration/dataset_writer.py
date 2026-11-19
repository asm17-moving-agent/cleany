"""Recoverable hand-eye dataset storage under a configurable artifact root.

Images are committed before a journal entry, and a journal entry is committed
before the atomically replaced ``samples.jsonl`` file.  Consequently a crash
can leave an orphan image or a replayable journal, but never a committed row
that names a missing image.  Opening the writer replays journals, verifies all
hashes, removes unreferenced images, and preserves every previously committed
sample.
"""

from __future__ import annotations

from collections.abc import Callable, Mapping, Sequence
from dataclasses import dataclass
import fcntl
import hashlib
import json
import math
import os
from pathlib import Path, PurePosixPath
import re
import subprocess
import threading
from typing import Any
from uuid import uuid4

import cv2
import numpy as np

from cleany_handeye_calibration.camera_acquisition import (
    CameraContract,
    CameraFramePair,
    CameraInfoFrame,
    DEFAULT_CAMERA_CONTRACT,
    validate_camera_pair,
)
from cleany_handeye_calibration.schema import (
    CAMERA_CALIBRATION_HASH_SCHEMA,
    CORNER_POINT_ORDERING,
    CalibrationSampleRecord,
    camera_calibration_sha256,
    sample_record_from_mapping,
    sample_record_to_mapping,
)
from cleany_handeye_calibration.target_detector import (
    DICTIONARY_NAME,
    MARKER_LENGTH_M,
    SQUARE_LENGTH_M,
    SQUARES_X,
    SQUARES_Y,
)


DATASET_MANIFEST_SCHEMA_VERSION = 1
DATASET_MANIFEST_NAME = 'manifest.yaml'
SAMPLES_FILE_NAME = 'samples.jsonl'
_RUN_ID_PATTERN = re.compile(r'^[A-Za-z0-9][A-Za-z0-9._-]*$')
_SHA256_PATTERN = re.compile(r'^[0-9a-f]{64}$')
_GIT_COMMIT_PATTERN = re.compile(r'^[0-9a-fA-F]{40,64}$')


class DatasetError(RuntimeError):
    """Base class for a dataset storage failure."""


class DatasetCorruptionError(DatasetError):
    """A committed manifest, sample, journal, or image failed validation."""


class DuplicateSampleError(DatasetError):
    """A sample ID is already committed in this run."""


def _text(value: str, *, field_name: str) -> str:
    if not isinstance(value, str) or not value or value != value.strip():
        raise ValueError(f'{field_name} must be a non-empty trimmed string')
    return value


def _positive_finite(value: float, *, field_name: str) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError) as error:
        raise ValueError(f'{field_name} must be numeric') from error
    if not math.isfinite(result) or result <= 0.0:
        raise ValueError(f'{field_name} must be finite and positive')
    return result


def _sha256(value: str, *, field_name: str) -> str:
    if not isinstance(value, str) or not _SHA256_PATTERN.fullmatch(value):
        raise ValueError(f'{field_name} must be a lowercase SHA-256 digest')
    return value


def sha256_bytes(payload: bytes) -> str:
    return hashlib.sha256(payload).hexdigest()


def sha256_file(path: str | os.PathLike[str]) -> str:
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def _canonical_json_bytes(value: Any) -> bytes:
    return json.dumps(
        value,
        sort_keys=True,
        separators=(',', ':'),
        ensure_ascii=True,
        allow_nan=False,
    ).encode('ascii')


def _pretty_json_bytes(value: Any) -> bytes:
    return (
        json.dumps(
            value,
            sort_keys=True,
            indent=2,
            ensure_ascii=True,
            allow_nan=False,
        )
        + '\n'
    ).encode('ascii')


def mapping_sha256(value: Mapping[str, Any]) -> str:
    if not isinstance(value, Mapping):
        raise ValueError('value must be a mapping')
    return sha256_bytes(_canonical_json_bytes(value))


def _json_object_copy(
    value: Mapping[str, Any],
    *,
    field_name: str,
) -> dict[str, Any]:
    if not isinstance(value, Mapping):
        raise ValueError(f'{field_name} must be a mapping')
    try:
        copied = json.loads(_canonical_json_bytes(value).decode('ascii'))
    except (TypeError, ValueError) as error:
        raise ValueError(
            f'{field_name} must contain only finite JSON values'
        ) from error
    if not isinstance(copied, dict):
        raise ValueError(f'{field_name} must be a JSON object')
    return copied


@dataclass(frozen=True, slots=True)
class GitProvenance:
    commit: str
    dirty: bool

    def __post_init__(self) -> None:
        commit = _text(self.commit, field_name='git commit')
        if not _GIT_COMMIT_PATTERN.fullmatch(commit):
            raise ValueError('git commit must be a 40-64 digit hexadecimal ID')
        if not isinstance(self.dirty, bool):
            raise ValueError('git dirty must be a bool')
        object.__setattr__(self, 'commit', commit.lower())


def read_git_provenance(
    repository_root: str | os.PathLike[str],
) -> GitProvenance:
    """Capture the current commit and full tracked/untracked dirty state."""

    root = Path(repository_root).resolve(strict=True)
    commit = subprocess.run(
        ['git', '-C', str(root), 'rev-parse', 'HEAD'],
        check=True,
        capture_output=True,
        text=True,
    ).stdout.strip()
    status = subprocess.run(
        [
            'git',
            '-C',
            str(root),
            'status',
            '--porcelain=v1',
            '--untracked-files=all',
        ],
        check=True,
        capture_output=True,
        text=True,
    ).stdout
    return GitProvenance(commit=commit, dirty=bool(status))


@dataclass(frozen=True, slots=True)
class SourceArtifactHashes:
    urdf_sha256: str
    mjcf_sha256: str
    pose_manifest_sha256: str

    def __post_init__(self) -> None:
        for field_name in (
            'urdf_sha256',
            'mjcf_sha256',
            'pose_manifest_sha256',
        ):
            object.__setattr__(
                self,
                field_name,
                _sha256(getattr(self, field_name), field_name=field_name),
            )


@dataclass(frozen=True, slots=True)
class SoftwareVersions:
    ros_distro: str
    moveit: str
    opencv: str
    mujoco: str
    mujoco_ros2_control: str
    vendor_versions: Mapping[str, str]

    def __post_init__(self) -> None:
        for field_name in (
            'ros_distro',
            'moveit',
            'opencv',
            'mujoco',
            'mujoco_ros2_control',
        ):
            object.__setattr__(
                self,
                field_name,
                _text(getattr(self, field_name), field_name=field_name),
            )
        if not isinstance(self.vendor_versions, Mapping):
            raise ValueError('vendor_versions must be a mapping')
        normalized: dict[str, str] = {}
        for name, version in sorted(self.vendor_versions.items()):
            normalized[_text(name, field_name='vendor package')] = _text(
                version,
                field_name=f'version for {name}',
            )
        if not normalized:
            raise ValueError('vendor_versions must not be empty')
        object.__setattr__(self, 'vendor_versions', normalized)


@dataclass(frozen=True, slots=True)
class TargetDatasetContract:
    board_svg_sha256: str
    board_pdf_sha256: str
    size_provenance: str
    square_length_m_used: float = SQUARE_LENGTH_M
    marker_length_m_used: float = MARKER_LENGTH_M
    squares_x: int = SQUARES_X
    squares_y: int = SQUARES_Y
    dictionary: str = DICTIONARY_NAME
    legacy_pattern: bool = False
    point_ordering: str = CORNER_POINT_ORDERING

    def __post_init__(self) -> None:
        object.__setattr__(
            self,
            'board_svg_sha256',
            _sha256(
                self.board_svg_sha256,
                field_name='board_svg_sha256',
            ),
        )
        object.__setattr__(
            self,
            'board_pdf_sha256',
            _sha256(
                self.board_pdf_sha256,
                field_name='board_pdf_sha256',
            ),
        )
        object.__setattr__(
            self,
            'size_provenance',
            _text(self.size_provenance, field_name='size_provenance'),
        )
        if self.squares_x != SQUARES_X or self.squares_y != SQUARES_Y:
            raise ValueError('target must contain 7 x 5 squares')
        square = _positive_finite(
            self.square_length_m_used,
            field_name='square_length_m_used',
        )
        marker = _positive_finite(
            self.marker_length_m_used,
            field_name='marker_length_m_used',
        )
        if marker >= square:
            raise ValueError(
                'marker length must be smaller than square length'
            )
        if not math.isclose(
            square,
            SQUARE_LENGTH_M,
            rel_tol=0.0,
            abs_tol=1.0e-12,
        ):
            raise ValueError(
                'simulation target square length must be exactly 0.030 m'
            )
        if not math.isclose(
            marker,
            MARKER_LENGTH_M,
            rel_tol=0.0,
            abs_tol=1.0e-12,
        ):
            raise ValueError(
                'simulation target marker length must be exactly 0.015 m'
            )
        if self.dictionary != DICTIONARY_NAME:
            raise ValueError('target dictionary must be DICT_5X5_100')
        if self.legacy_pattern is not False:
            raise ValueError('target legacy_pattern must be false')
        if self.point_ordering != CORNER_POINT_ORDERING:
            raise ValueError('unsupported target point ordering')
        object.__setattr__(self, 'square_length_m_used', square)
        object.__setattr__(self, 'marker_length_m_used', marker)


@dataclass(frozen=True, slots=True)
class CaptureTiming:
    simulation_timestep_s: float
    controller_update_rate_hz: float
    image_rate_hz: float
    joint_state_rate_hz: float

    def __post_init__(self) -> None:
        for field_name in (
            'simulation_timestep_s',
            'controller_update_rate_hz',
            'image_rate_hz',
            'joint_state_rate_hz',
        ):
            object.__setattr__(
                self,
                field_name,
                _positive_finite(
                    getattr(self, field_name),
                    field_name=field_name,
                ),
            )


@dataclass(frozen=True, slots=True)
class DatasetManifestV1:
    """Required run provenance; no value is discovered or guessed silently."""

    run_id: str
    git: GitProvenance
    source_hashes: SourceArtifactHashes
    software: SoftwareVersions
    camera: CameraContract
    camera_vertical_fov_degrees: float
    target: TargetDatasetContract
    timing: CaptureTiming
    calibration_parameters: Mapping[str, Any]
    random_seed: int

    def __post_init__(self) -> None:
        run_id = _text(self.run_id, field_name='run_id')
        if not _RUN_ID_PATTERN.fullmatch(run_id):
            raise ValueError(
                'run_id must contain only letters, digits, dot, underscore, '
                'or hyphen'
            )
        if not isinstance(self.git, GitProvenance):
            raise ValueError('git must be GitProvenance')
        if not isinstance(self.source_hashes, SourceArtifactHashes):
            raise ValueError('source_hashes must be SourceArtifactHashes')
        if not isinstance(self.software, SoftwareVersions):
            raise ValueError('software must be SoftwareVersions')
        if not isinstance(self.camera, CameraContract):
            raise ValueError('camera must be CameraContract')
        if self.camera != DEFAULT_CAMERA_CONTRACT:
            raise ValueError(
                'camera must match the fixed wrist-camera contract'
            )
        fov = _positive_finite(
            self.camera_vertical_fov_degrees,
            field_name='camera_vertical_fov_degrees',
        )
        if not math.isclose(fov, 93.0, rel_tol=0.0, abs_tol=1.0e-12):
            raise ValueError('camera vertical FOV must be 93 degrees')
        focal_length_px = self.camera.height / (
            2.0 * math.tan(math.radians(fov) / 2.0)
        )
        if not math.isclose(
            self.camera.k[4],
            focal_length_px,
            rel_tol=0.0,
            abs_tol=1.0e-6,
        ):
            raise ValueError(
                'camera fy must match height and vertical FOV'
            )
        if not isinstance(self.target, TargetDatasetContract):
            raise ValueError('target must be TargetDatasetContract')
        if not isinstance(self.timing, CaptureTiming):
            raise ValueError('timing must be CaptureTiming')
        parameters = _json_object_copy(
            self.calibration_parameters,
            field_name='calibration_parameters',
        )
        if not parameters:
            raise ValueError('calibration_parameters must not be empty')
        if (
            isinstance(self.random_seed, bool)
            or not isinstance(self.random_seed, int)
            or self.random_seed < 0
        ):
            raise ValueError('random_seed must be a non-negative integer')
        object.__setattr__(self, 'run_id', run_id)
        object.__setattr__(self, 'camera_vertical_fov_degrees', fov)
        object.__setattr__(self, 'calibration_parameters', parameters)


def _manifest_camera_info(manifest: DatasetManifestV1) -> CameraInfoFrame:
    camera = manifest.camera
    return CameraInfoFrame(
        stamp_ns=1,
        frame_id=camera.frame_id,
        width=camera.width,
        height=camera.height,
        distortion_model=camera.distortion_model,
        d=camera.d,
        k=camera.k,
        r=camera.r,
        p=camera.p,
    )


def manifest_to_mapping(manifest: DatasetManifestV1) -> dict[str, Any]:
    if not isinstance(manifest, DatasetManifestV1):
        raise ValueError('manifest must be DatasetManifestV1')
    camera = manifest.camera
    target = manifest.target
    computed_focal_length_px = camera.height / (
        2.0
        * math.tan(
            math.radians(manifest.camera_vertical_fov_degrees) / 2.0
        )
    )
    body: dict[str, Any] = {
        'schema_version': DATASET_MANIFEST_SCHEMA_VERSION,
        'run_id': manifest.run_id,
        'git': {
            'commit': manifest.git.commit,
            'dirty': manifest.git.dirty,
        },
        'source_hashes': {
            'urdf_sha256': manifest.source_hashes.urdf_sha256,
            'mjcf_sha256': manifest.source_hashes.mjcf_sha256,
            'pose_manifest_sha256': (
                manifest.source_hashes.pose_manifest_sha256
            ),
        },
        'software_versions': {
            'ros_distro': manifest.software.ros_distro,
            'moveit': manifest.software.moveit,
            'opencv': manifest.software.opencv,
            'mujoco': manifest.software.mujoco,
            'mujoco_ros2_control': (
                manifest.software.mujoco_ros2_control
            ),
            'vendor': dict(manifest.software.vendor_versions),
        },
        'camera': {
            'frame_id': camera.frame_id,
            'encoding': camera.encoding,
            'width': camera.width,
            'height': camera.height,
            'vertical_fov_degrees': (
                manifest.camera_vertical_fov_degrees
            ),
            'focal_length_formula': (
                'fy_px = height_px / '
                '(2 * tan(vertical_fov_rad / 2))'
            ),
            'computed_focal_length_px': computed_focal_length_px,
            'distortion_model': camera.distortion_model,
            'K': list(camera.k),
            'D': list(camera.d),
            'R': list(camera.r),
            'P': list(camera.p),
            'calibration_hash_schema': (
                CAMERA_CALIBRATION_HASH_SCHEMA
            ),
            'calibration_sha256': camera_calibration_sha256(
                _manifest_camera_info(manifest)
            ),
        },
        'target': {
            'type': 'charuco',
            'squares_x': target.squares_x,
            'squares_y': target.squares_y,
            'square_length_m_used': target.square_length_m_used,
            'marker_length_m_used': target.marker_length_m_used,
            'dictionary': target.dictionary,
            'legacy_pattern': target.legacy_pattern,
            'point_ordering': target.point_ordering,
            'size_provenance': target.size_provenance,
            'board_svg_sha256': target.board_svg_sha256,
            'board_pdf_sha256': target.board_pdf_sha256,
        },
        'timing': {
            'simulation_timestep_s': (
                manifest.timing.simulation_timestep_s
            ),
            'controller_update_rate_hz': (
                manifest.timing.controller_update_rate_hz
            ),
            'image_rate_hz': manifest.timing.image_rate_hz,
            'joint_state_rate_hz': manifest.timing.joint_state_rate_hz,
        },
        'calibration': {
            'parameters': dict(manifest.calibration_parameters),
            'random_seed': manifest.random_seed,
        },
    }
    body['manifest_sha256'] = mapping_sha256(body)
    return body


@dataclass(frozen=True, slots=True)
class StoredCalibrationSample:
    record: CalibrationSampleRecord
    image_sha256: str
    source_image_sha256: str
    record_sha256: str


@dataclass(frozen=True, slots=True)
class ArchivedAttemptImage:
    sequence: int
    pose_id: str
    attempt: int
    image_stamp_ns: int
    image_path: str
    png_sha256: str
    source_image_sha256: str


class _DatasetLock:
    def __init__(self, path: Path, thread_lock: threading.RLock) -> None:
        self._path = path
        self._thread_lock = thread_lock
        self._stream = None

    def __enter__(self) -> _DatasetLock:
        self._thread_lock.acquire()
        try:
            self._stream = self._path.open('a+b')
            fcntl.flock(self._stream.fileno(), fcntl.LOCK_EX)
            return self
        except BaseException:
            self._thread_lock.release()
            raise

    def __exit__(self, exc_type, exc_value, traceback) -> None:
        assert self._stream is not None
        try:
            fcntl.flock(self._stream.fileno(), fcntl.LOCK_UN)
            self._stream.close()
        finally:
            self._thread_lock.release()


class DatasetWriter:
    """Atomic writer and recovery reader for one calibration run."""

    def __init__(
        self,
        *,
        artifact_root: str | os.PathLike[str],
        manifest: DatasetManifestV1,
        fault_hook: Callable[[str], None] | None = None,
    ) -> None:
        if not isinstance(manifest, DatasetManifestV1):
            raise ValueError('manifest must be DatasetManifestV1')
        root = Path(artifact_root).expanduser()
        if root.exists() and not root.is_dir():
            raise ValueError('artifact_root must be a directory')
        self._artifact_root = root.resolve()
        self._manifest = manifest
        self._run_dir = self._artifact_root / manifest.run_id
        self._images_dir = self._run_dir / 'images'
        self._attempt_images_dir = self._run_dir / 'attempt_images'
        self._journal_dir = self._run_dir / '.journal'
        self._samples_path = self._run_dir / SAMPLES_FILE_NAME
        self._manifest_path = self._run_dir / DATASET_MANIFEST_NAME
        self._fault_hook = fault_hook
        self._thread_lock = threading.RLock()

        self._artifact_root.mkdir(parents=True, exist_ok=True)
        if self._run_dir.is_symlink():
            raise ValueError('run directory must not be a symbolic link')
        self._run_dir.mkdir(exist_ok=True)
        if self._run_dir.resolve() != self._run_dir:
            raise ValueError('run directory must stay within artifact_root')
        if (
            self._images_dir.is_symlink()
            or self._attempt_images_dir.is_symlink()
            or self._journal_dir.is_symlink()
        ):
            raise ValueError('dataset subdirectories must not be symlinks')
        self._images_dir.mkdir(exist_ok=True)
        self._attempt_images_dir.mkdir(exist_ok=True)
        self._journal_dir.mkdir(parents=True, exist_ok=True)
        self._lock = _DatasetLock(
            self._run_dir / '.writer.lock',
            self._thread_lock,
        )
        with self._lock:
            self._initialize_manifest_locked()
            self._recover_locked()

    @property
    def artifact_root(self) -> Path:
        return self._artifact_root

    @property
    def run_directory(self) -> Path:
        return self._run_dir

    @property
    def manifest_path(self) -> Path:
        return self._manifest_path

    @property
    def samples_path(self) -> Path:
        return self._samples_path

    @property
    def attempt_images_directory(self) -> Path:
        return self._attempt_images_dir

    def _invoke_fault_hook(self, stage: str) -> None:
        if self._fault_hook is not None:
            self._fault_hook(stage)

    @staticmethod
    def _fsync_directory(path: Path) -> None:
        descriptor = os.open(path, os.O_RDONLY | os.O_DIRECTORY)
        try:
            os.fsync(descriptor)
        finally:
            os.close(descriptor)

    @classmethod
    def _atomic_write(cls, path: Path, payload: bytes) -> None:
        temporary = path.parent / (
            f'.{path.name}.tmp-{os.getpid()}-{uuid4().hex}'
        )
        descriptor = os.open(
            temporary,
            os.O_WRONLY | os.O_CREAT | os.O_EXCL,
            0o640,
        )
        try:
            view = memoryview(payload)
            while view:
                written = os.write(descriptor, view)
                if written <= 0:
                    raise OSError('short write while committing artifact')
                view = view[written:]
            os.fsync(descriptor)
        except BaseException:
            os.close(descriptor)
            temporary.unlink(missing_ok=True)
            raise
        else:
            os.close(descriptor)
        os.replace(temporary, path)
        cls._fsync_directory(path.parent)

    @staticmethod
    def _load_json(payload: bytes, *, artifact_name: str) -> Any:
        def reject_constant(value: str) -> None:
            raise ValueError(f'non-finite JSON constant {value}')

        try:
            return json.loads(
                payload.decode('utf-8'),
                parse_constant=reject_constant,
            )
        except (UnicodeDecodeError, json.JSONDecodeError, ValueError) as error:
            raise DatasetCorruptionError(
                f'{artifact_name} is not strict JSON: {error}'
            ) from error

    def _initialize_manifest_locked(self) -> None:
        expected = manifest_to_mapping(self._manifest)
        if not self._manifest_path.exists():
            self._atomic_write(
                self._manifest_path,
                _pretty_json_bytes(expected),
            )
            return
        actual = self._load_json(
            self._manifest_path.read_bytes(),
            artifact_name=DATASET_MANIFEST_NAME,
        )
        if not isinstance(actual, dict):
            raise DatasetCorruptionError('manifest must contain an object')
        stored_hash = actual.get('manifest_sha256')
        body = dict(actual)
        body.pop('manifest_sha256', None)
        if stored_hash != mapping_sha256(body):
            raise DatasetCorruptionError('manifest SHA-256 does not match')
        if actual != expected:
            raise DatasetError(
                'existing run manifest differs from requested provenance'
            )

    def _image_path_from_mapping(self, mapping: Mapping[str, Any]) -> Path:
        image_path = mapping.get('image_path')
        if not isinstance(image_path, str):
            raise DatasetCorruptionError('sample image_path must be text')
        relative = PurePosixPath(image_path)
        if (
            relative.is_absolute()
            or '..' in relative.parts
            or len(relative.parts) != 2
            or relative.parts[0] != 'images'
        ):
            raise DatasetCorruptionError('sample image_path is unsafe')
        path = self._run_dir.joinpath(*relative.parts)
        if path.parent != self._images_dir:
            raise DatasetCorruptionError('sample image_path escapes images')
        return path

    def _validate_stored_mapping(
        self,
        mapping: Mapping[str, Any],
        *,
        artifact_name: str,
    ) -> StoredCalibrationSample:
        if not isinstance(mapping, Mapping):
            raise DatasetCorruptionError(
                f'{artifact_name} sample must be an object'
            )
        try:
            image_sha = _sha256(
                mapping['image_sha256'],
                field_name='image_sha256',
            )
            source_sha = _sha256(
                mapping['source_image_sha256'],
                field_name='source_image_sha256',
            )
            record_sha = _sha256(
                mapping['record_sha256'],
                field_name='record_sha256',
            )
            hash_body = dict(mapping)
            hash_body.pop('record_sha256')
            if record_sha != mapping_sha256(hash_body):
                raise ValueError('record SHA-256 does not match')
            record = sample_record_from_mapping(mapping)
            expected_camera_hash = manifest_to_mapping(self._manifest)[
                'camera'
            ]['calibration_sha256']
            if mapping['camera_calibration_sha256'] != expected_camera_hash:
                raise ValueError(
                    'sample camera calibration differs from manifest'
                )
            image_path = self._image_path_from_mapping(mapping)
            if image_path.is_symlink():
                raise ValueError('dataset image must not be a symbolic link')
            if not image_path.is_file():
                raise ValueError(f'image is missing: {record.image_path}')
            if sha256_file(image_path) != image_sha:
                raise ValueError('image SHA-256 does not match')
        except (KeyError, ValueError) as error:
            raise DatasetCorruptionError(
                f'{artifact_name} is invalid: {error}'
            ) from error
        return StoredCalibrationSample(
            record=record,
            image_sha256=image_sha,
            source_image_sha256=source_sha,
            record_sha256=record_sha,
        )

    def _read_sample_mappings_locked(self) -> list[dict[str, Any]]:
        if not self._samples_path.exists():
            return []
        payload = self._samples_path.read_bytes()
        if payload and not payload.endswith(b'\n'):
            raise DatasetCorruptionError(
                'samples.jsonl must end with a complete newline'
            )
        records: list[dict[str, Any]] = []
        sample_ids: set[str] = set()
        for line_number, line in enumerate(payload.splitlines(), start=1):
            if not line:
                raise DatasetCorruptionError(
                    f'samples.jsonl line {line_number} is blank'
                )
            mapping = self._load_json(
                line,
                artifact_name=f'samples.jsonl line {line_number}',
            )
            stored = self._validate_stored_mapping(
                mapping,
                artifact_name=f'samples.jsonl line {line_number}',
            )
            sample_id = stored.record.sample.sample_id
            if sample_id in sample_ids:
                raise DatasetCorruptionError(
                    f'duplicate committed sample_id: {sample_id}'
                )
            sample_ids.add(sample_id)
            records.append(dict(mapping))
        return records

    def _rewrite_samples_locked(
        self,
        mappings: Sequence[Mapping[str, Any]],
    ) -> None:
        payload = b''.join(
            _canonical_json_bytes(mapping) + b'\n' for mapping in mappings
        )
        self._atomic_write(self._samples_path, payload)

    def _journal_paths(self) -> tuple[Path, ...]:
        return tuple(sorted(self._journal_dir.glob('*.json')))

    def _recover_locked(self) -> None:
        for directory in (
            self._run_dir,
            self._images_dir,
            self._journal_dir,
        ):
            for temporary in directory.glob('.*.tmp-*'):
                if temporary.is_file():
                    temporary.unlink()

        mappings = self._read_sample_mappings_locked()
        by_id = {
            mapping['sample_id']: mapping
            for mapping in mappings
        }
        for journal_path in self._journal_paths():
            journal = self._load_json(
                journal_path.read_bytes(),
                artifact_name=journal_path.name,
            )
            stored = self._validate_stored_mapping(
                journal,
                artifact_name=journal_path.name,
            )
            sample_id = stored.record.sample.sample_id
            committed = by_id.get(sample_id)
            if committed is None:
                mappings.append(dict(journal))
                by_id[sample_id] = dict(journal)
                self._rewrite_samples_locked(mappings)
            elif committed != journal:
                raise DatasetCorruptionError(
                    f'journal conflicts with committed sample {sample_id}'
                )
            journal_path.unlink()
            self._fsync_directory(self._journal_dir)

        referenced_images = {
            self._image_path_from_mapping(mapping)
            for mapping in mappings
        }
        for image_path in self._images_dir.glob('*.png'):
            if image_path.is_symlink():
                raise DatasetCorruptionError(
                    'dataset images must not be symbolic links'
                )
            if image_path not in referenced_images:
                image_path.unlink()
        if not self._samples_path.exists():
            self._rewrite_samples_locked(mappings)
        self._fsync_directory(self._images_dir)

    @staticmethod
    def _encode_rgb_png(pair: CameraFramePair) -> bytes:
        image = pair.image
        pixels = np.frombuffer(image.data, dtype=np.uint8).reshape(
            image.height,
            image.width,
            3,
        )
        if image.encoding == 'rgb8':
            pixels = cv2.cvtColor(pixels, cv2.COLOR_RGB2BGR)
        elif image.encoding != 'bgr8':
            raise ValueError(
                f'unsupported PNG source encoding: {image.encoding!r}'
            )
        success, encoded = cv2.imencode(
            '.png',
            np.ascontiguousarray(pixels),
            [cv2.IMWRITE_PNG_COMPRESSION, 3],
        )
        if not success:
            raise DatasetError('OpenCV failed to encode camera image as PNG')
        return bytes(encoded)

    def append_sample(
        self,
        record: CalibrationSampleRecord,
        pair: CameraFramePair,
    ) -> StoredCalibrationSample:
        """Commit one image and row using the recoverable three-stage order."""

        if not isinstance(record, CalibrationSampleRecord):
            raise ValueError('record must be a CalibrationSampleRecord')
        if not isinstance(pair, CameraFramePair):
            raise ValueError('pair must be a CameraFramePair')
        validation = validate_camera_pair(
            pair.image,
            pair.camera_info,
            contract=self._manifest.camera,
        )
        if validation.pair is None:
            assert validation.rejection is not None
            raise ValueError(
                'camera pair violates the dataset contract: '
                f'{validation.rejection.reason.value}'
            )
        if pair.stamp_ns != record.image_stamp_ns:
            raise ValueError('camera pair stamp must equal record image stamp')
        if pair.camera_info != record.camera_info:
            raise ValueError(
                'camera pair CameraInfo must equal the recorded calibration'
            )

        encoded_image = self._encode_rgb_png(pair)
        image_sha = sha256_bytes(encoded_image)
        source_sha = sha256_bytes(pair.image.data)
        mapping = sample_record_to_mapping(record)
        mapping['image_sha256'] = image_sha
        mapping['source_image_sha256'] = source_sha
        mapping['record_sha256'] = mapping_sha256(mapping)
        stored = StoredCalibrationSample(
            record=record,
            image_sha256=image_sha,
            source_image_sha256=source_sha,
            record_sha256=mapping['record_sha256'],
        )

        image_path = self._image_path_from_mapping(mapping)
        journal_path = self._journal_dir / (
            f'{record.sample.sample_id}.json'
        )
        with self._lock:
            mappings = self._read_sample_mappings_locked()
            if any(
                existing['sample_id'] == record.sample.sample_id
                for existing in mappings
            ):
                raise DuplicateSampleError(
                    f'sample_id is already committed: '
                    f'{record.sample.sample_id}'
                )
            if journal_path.exists():
                raise DatasetError(
                    f'unrecovered journal exists for '
                    f'{record.sample.sample_id}'
                )

            if image_path.is_symlink():
                raise DatasetCorruptionError(
                    'orphan image must not be a symbolic link'
                )
            if image_path.exists():
                if sha256_file(image_path) != image_sha:
                    raise DatasetCorruptionError(
                        'existing orphan image differs from new sample image'
                    )
            else:
                self._atomic_write(image_path, encoded_image)
            self._invoke_fault_hook('after_image_commit')

            self._atomic_write(
                journal_path,
                _canonical_json_bytes(mapping) + b'\n',
            )
            self._invoke_fault_hook('after_journal_commit')

            mappings.append(mapping)
            self._rewrite_samples_locked(mappings)
            self._invoke_fault_hook('after_samples_commit')

            journal_path.unlink()
            self._fsync_directory(self._journal_dir)
        return stored

    def archive_attempt_image(
        self,
        *,
        pose_id: str,
        attempt: int,
        pair: CameraFramePair,
    ) -> ArchivedAttemptImage:
        """Durably preserve every acquired post-settle frame before PnP."""

        pose = _text(pose_id, field_name='pose_id')
        if not _RUN_ID_PATTERN.fullmatch(pose):
            raise ValueError('pose_id contains unsafe filename characters')
        if isinstance(attempt, bool) or not isinstance(attempt, int):
            raise ValueError('attempt must be a positive integer')
        if attempt <= 0:
            raise ValueError('attempt must be a positive integer')
        if not isinstance(pair, CameraFramePair):
            raise ValueError('pair must be a CameraFramePair')
        validation = validate_camera_pair(
            pair.image,
            pair.camera_info,
            contract=self._manifest.camera,
        )
        if validation.pair is None:
            assert validation.rejection is not None
            raise ValueError(
                'camera pair violates the attempt archive contract: '
                f'{validation.rejection.reason.value}'
            )

        encoded = self._encode_rgb_png(pair)
        png_sha = sha256_bytes(encoded)
        source_sha = sha256_bytes(pair.image.data)
        with self._lock:
            sequence = 1
            for path in self._attempt_images_dir.iterdir():
                if path.is_symlink():
                    raise DatasetCorruptionError(
                        'attempt image artifacts must not be symlinks'
                    )
                prefix = path.name.split('_', 1)[0]
                if prefix.isdigit():
                    sequence = max(sequence, int(prefix) + 1)
            basename = (
                f'{sequence:06d}_{pose}_attempt_{attempt:02d}_'
                f'stamp_{pair.stamp_ns}'
            )
            image_path = self._attempt_images_dir / f'{basename}.png'
            metadata_path = self._attempt_images_dir / f'{basename}.json'
            relative_image_path = image_path.relative_to(
                self._run_dir
            ).as_posix()
            metadata = {
                'schema_version': 1,
                'sequence': sequence,
                'pose_id': pose,
                'attempt': attempt,
                'image_stamp_ns': pair.stamp_ns,
                'camera_frame_id': pair.image.frame_id,
                'encoding': pair.image.encoding,
                'width': pair.image.width,
                'height': pair.image.height,
                'camera_calibration_sha256': (
                    camera_calibration_sha256(pair.camera_info)
                ),
                'image_path': relative_image_path,
                'png_sha256': png_sha,
                'source_image_sha256': source_sha,
            }
            self._atomic_write(image_path, encoded)
            self._atomic_write(
                metadata_path,
                _pretty_json_bytes(metadata),
            )
        return ArchivedAttemptImage(
            sequence=sequence,
            pose_id=pose,
            attempt=attempt,
            image_stamp_ns=pair.stamp_ns,
            image_path=relative_image_path,
            png_sha256=png_sha,
            source_image_sha256=source_sha,
        )

    def read_samples(self) -> tuple[StoredCalibrationSample, ...]:
        """Verify and return every committed row in append order."""

        with self._lock:
            self._recover_locked()
            return tuple(
                self._validate_stored_mapping(
                    mapping,
                    artifact_name=SAMPLES_FILE_NAME,
                )
                for mapping in self._read_sample_mappings_locked()
            )


__all__ = [
    'DATASET_MANIFEST_SCHEMA_VERSION',
    'CaptureTiming',
    'ArchivedAttemptImage',
    'DatasetCorruptionError',
    'DatasetError',
    'DatasetManifestV1',
    'DatasetWriter',
    'DuplicateSampleError',
    'GitProvenance',
    'SoftwareVersions',
    'SourceArtifactHashes',
    'StoredCalibrationSample',
    'TargetDatasetContract',
    'manifest_to_mapping',
    'mapping_sha256',
    'read_git_provenance',
    'sha256_bytes',
    'sha256_file',
]
