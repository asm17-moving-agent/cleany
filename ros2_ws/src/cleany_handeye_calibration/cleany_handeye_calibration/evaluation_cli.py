"""Command-line entry point for offline candidate evaluation artifacts."""

from __future__ import annotations

import argparse
from pathlib import Path

from cleany_handeye_calibration.evaluation_artifacts import (
    load_evaluation_config,
    load_ground_truth,
    sha256_file,
    validate_dataset_manifest,
    write_evaluation_artifacts,
)
from cleany_handeye_calibration.experiment_evaluation import (
    load_sample_records,
    run_solver_experiment,
)
from cleany_handeye_calibration.offline_fk import UrdfOfflineFk
from cleany_handeye_calibration.timestamp_sensitivity import (
    evaluate_timestamp_sensitivity,
    load_continuous_trajectory_log,
)


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            'Evaluate five hand-eye methods and timestamp sensitivity '
            'without modifying robot calibration files.'
        )
    )
    parser.add_argument('--samples', required=True)
    parser.add_argument('--continuous-log', required=True)
    parser.add_argument('--urdf', required=True)
    parser.add_argument('--ground-truth', required=True)
    parser.add_argument('--config', required=True)
    parser.add_argument('--output-directory', required=True)
    return parser


def run(args: argparse.Namespace) -> tuple[Path, ...]:
    samples_path = Path(args.samples).expanduser().resolve(strict=True)
    continuous_path = Path(args.continuous_log).expanduser().resolve(
        strict=True
    )
    urdf_path = Path(args.urdf).expanduser().resolve(strict=True)
    truth_path = Path(args.ground_truth).expanduser().resolve(strict=True)
    config_path = Path(args.config).expanduser().resolve(strict=True)
    config = load_evaluation_config(config_path)
    urdf_sha256 = sha256_file(urdf_path)
    dataset_manifest_path = samples_path.parent / 'manifest.yaml'
    validate_dataset_manifest(
        dataset_manifest_path,
        urdf_sha256=urdf_sha256,
    )
    records = load_sample_records(samples_path)
    trajectory = load_continuous_trajectory_log(continuous_path)
    ground_truth = load_ground_truth(truth_path)
    fk_port = UrdfOfflineFk(urdf_path)
    solver = run_solver_experiment(
        records,
        fk_port,
        config.solver,
        ground_truth=ground_truth,
    )
    timestamp = evaluate_timestamp_sensitivity(
        trajectory,
        fk_port,
        config.timestamp,
        solver.selection,
        ground_truth=ground_truth,
    )
    return write_evaluation_artifacts(
        args.output_directory,
        solver,
        timestamp,
        input_hashes={
            'samples_jsonl': sha256_file(samples_path),
            'continuous_log_jsonl': sha256_file(continuous_path),
            'urdf': urdf_sha256,
            'dataset_manifest': sha256_file(dataset_manifest_path),
            'ground_truth': sha256_file(truth_path),
            'evaluation_config': sha256_file(config_path),
        },
    )


def main(argv=None) -> None:
    try:
        paths = run(_parser().parse_args(argv))
    except Exception as error:
        print(f'hand-eye evaluation failed: {type(error).__name__}: {error}')
        raise SystemExit(2) from error
    for path in paths:
        print(path)


if __name__ == '__main__':
    main()
