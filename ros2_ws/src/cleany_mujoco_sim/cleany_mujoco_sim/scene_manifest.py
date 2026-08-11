from __future__ import annotations

import argparse
from dataclasses import dataclass
import hashlib
import math
from pathlib import Path
import sys
from typing import Any, Mapping, Sequence

import yaml

from cleany_mujoco_sim.scene_loader import _package_share


SCHEMA_VERSION = 'cleany.handeye_scene/v1'
EXPECTED_BOARD = {
    'squares_x': 7,
    'squares_y': 5,
    'dictionary': 'DICT_5X5_100',
    'legacy_pattern': False,
}


class SceneManifestError(ValueError):
    """Raised when a hand-eye scene manifest is incomplete or inconsistent."""


class PhysicalMeasurementRequiredError(SceneManifestError):
    """Raised when a physical run has no traceable board measurement."""


@dataclass(frozen=True)
class PrintableAsset:
    format: str
    path: Path
    sha256: str
    media_width_m: float
    media_height_m: float


@dataclass(frozen=True)
class HandEyeSceneManifest:
    path: Path
    data: Mapping[str, Any]
    printable_assets: tuple[PrintableAsset, ...]


def _mapping(value: Any, label: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise SceneManifestError(f'{label} must be a mapping')
    return value


def _positive_number(value: Any, label: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise SceneManifestError(f'{label} must be a number')
    result = float(value)
    if not math.isfinite(result) or result <= 0.0:
        raise SceneManifestError(f'{label} must be positive and finite')
    return result


def _package_uri_path(uri: Any) -> Path:
    if not isinstance(uri, str) or not uri.startswith('package://'):
        raise SceneManifestError(f'asset URI must be package://, got {uri!r}')
    remainder = uri.removeprefix('package://')
    package_name, separator, relative_path = remainder.partition('/')
    if not separator or not relative_path:
        raise SceneManifestError(f'invalid package URI: {uri}')
    return _package_share(package_name) / relative_path


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def load_handeye_scene_manifest(path: Path) -> HandEyeSceneManifest:
    manifest_path = path.expanduser().resolve()
    try:
        raw = yaml.safe_load(manifest_path.read_text(encoding='utf-8'))
    except (OSError, yaml.YAMLError) as error:
        raise SceneManifestError(
            f'cannot read scene manifest {manifest_path}: {error}'
        ) from error
    data = _mapping(raw, 'manifest')
    if data.get('schema_version') != SCHEMA_VERSION:
        raise SceneManifestError(
            f'unsupported schema_version: {data.get("schema_version")!r}'
        )

    target = _mapping(data.get('target'), 'target')
    board = _mapping(target.get('board'), 'target.board')
    for key, expected in EXPECTED_BOARD.items():
        if board.get(key) != expected:
            raise SceneManifestError(
                f'target.board.{key} must be {expected!r}, '
                f'got {board.get(key)!r}'
            )
    marker_ids = board.get('marker_ids')
    if marker_ids != list(range(17)):
        raise SceneManifestError('target.board.marker_ids must be 0..16')

    nominal = _mapping(
        board.get('simulation_nominal'),
        'target.board.simulation_nominal',
    )
    if nominal.get('status') != 'nominal_model':
        raise SceneManifestError(
            'simulation_nominal.status must be nominal_model'
        )
    square_length = _positive_number(
        nominal.get('square_length_m'),
        'simulation_nominal.square_length_m',
    )
    marker_length = _positive_number(
        nominal.get('marker_length_m'),
        'simulation_nominal.marker_length_m',
    )
    if not math.isclose(square_length, 0.030, abs_tol=1e-12):
        raise SceneManifestError('simulation square length must be 0.030 m')
    if not math.isclose(marker_length, 0.015, abs_tol=1e-12):
        raise SceneManifestError('simulation marker length must be 0.015 m')
    if not math.isclose(
        _positive_number(nominal.get('board_width_m'), 'board_width_m'),
        7 * square_length,
        abs_tol=1e-12,
    ):
        raise SceneManifestError('nominal board width does not match 7 squares')
    if not math.isclose(
        _positive_number(nominal.get('board_height_m'), 'board_height_m'),
        5 * square_length,
        abs_tol=1e-12,
    ):
        raise SceneManifestError('nominal board height does not match 5 squares')

    assets = _mapping(target.get('printable_assets'), 'printable_assets')
    records: list[PrintableAsset] = []
    for asset_format in ('svg', 'pdf'):
        record = _mapping(assets.get(asset_format), f'assets.{asset_format}')
        asset_path = _package_uri_path(record.get('uri'))
        expected_hash = record.get('sha256')
        if not isinstance(expected_hash, str) or len(expected_hash) != 64:
            raise SceneManifestError(f'{asset_format} sha256 is invalid')
        if not asset_path.is_file():
            raise SceneManifestError(f'{asset_format} asset is missing: {asset_path}')
        actual_hash = _sha256(asset_path)
        if actual_hash != expected_hash:
            raise SceneManifestError(
                f'{asset_format} sha256 mismatch: expected {expected_hash}, '
                f'got {actual_hash}'
            )
        records.append(
            PrintableAsset(
                format=asset_format,
                path=asset_path,
                sha256=actual_hash,
                media_width_m=_positive_number(
                    record.get('media_width_m'),
                    f'{asset_format}.media_width_m',
                ),
                media_height_m=_positive_number(
                    record.get('media_height_m'),
                    f'{asset_format}.media_height_m',
                ),
            )
        )

    _validate_pose_metadata(target)
    return HandEyeSceneManifest(
        path=manifest_path,
        data=data,
        printable_assets=tuple(records),
    )


def _validate_pose_metadata(target: Mapping[str, Any]) -> None:
    ground_truth = _mapping(target.get('ground_truth_pose'), 'ground_truth_pose')
    if ground_truth.get('semantics') != 'base_T_target':
        raise SceneManifestError('ground-truth semantics must be base_T_target')
    if ground_truth.get('evaluation_only') is not True:
        raise SceneManifestError('ground truth must be evaluation_only')
    if ground_truth.get('allowed_for_pnp_or_solver_input') is not False:
        raise SceneManifestError('ground truth cannot be PnP/solver input')
    translation = ground_truth.get('translation_m')
    quaternion = ground_truth.get('quaternion_xyzw')
    if not _finite_vector(translation, 3):
        raise SceneManifestError('ground-truth translation must be finite xyz')
    if not _finite_vector(quaternion, 4):
        raise SceneManifestError('ground-truth quaternion must be finite xyzw')
    norm = math.sqrt(sum(float(value) ** 2 for value in quaternion))
    if not math.isclose(norm, 1.0, abs_tol=1e-9):
        raise SceneManifestError('ground-truth quaternion must be normalized')


def _finite_vector(value: Any, length: int) -> bool:
    return (
        isinstance(value, Sequence)
        and not isinstance(value, (str, bytes))
        and len(value) == length
        and all(
            not isinstance(item, bool)
            and isinstance(item, (int, float))
            and math.isfinite(float(item))
            for item in value
        )
    )


def preflight_manifest(
    manifest: HandEyeSceneManifest,
    *,
    profile: str,
) -> None:
    if profile == 'simulation':
        return
    if profile != 'physical':
        raise SceneManifestError(f'unknown preflight profile: {profile}')

    board = _mapping(manifest.data['target'], 'target')['board']
    measurement = _mapping(board, 'target.board')['physical_measurement']
    measurement = _mapping(measurement, 'physical_measurement')
    if measurement.get('status') != 'measured':
        raise PhysicalMeasurementRequiredError(
            'physical target has not been measured; keep collection disabled '
            'until square/marker lengths and measurement provenance are recorded'
        )
    _positive_number(
        measurement.get('square_length_m'),
        'physical_measurement.square_length_m',
    )
    _positive_number(
        measurement.get('marker_length_m'),
        'physical_measurement.marker_length_m',
    )
    for field in (
        'measured_at',
        'measured_by',
        'measurement_tool',
        'evidence_ref',
    ):
        value = measurement.get(field)
        if not isinstance(value, str) or not value.strip():
            raise PhysicalMeasurementRequiredError(
                f'physical_measurement.{field} is required'
            )


def default_manifest_path() -> Path:
    return _package_share('cleany_mujoco_sim') / 'config' / 'handeye_scene.yaml'


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description='Validate Cleany hand-eye scene assets and measurements.'
    )
    parser.add_argument(
        '--manifest',
        type=Path,
        default=default_manifest_path(),
    )
    parser.add_argument(
        '--profile',
        choices=('simulation', 'physical'),
        default='simulation',
    )
    arguments = parser.parse_args(argv)
    try:
        manifest = load_handeye_scene_manifest(arguments.manifest)
        preflight_manifest(manifest, profile=arguments.profile)
    except SceneManifestError as error:
        print(f'hand-eye scene preflight failed: {error}', file=sys.stderr)
        return 2
    print(
        f'hand-eye scene preflight passed for {arguments.profile}: '
        f'{manifest.path}'
    )
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
