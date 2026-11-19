"""Command-line preflight for immutable calibration pose manifests."""

from __future__ import annotations

import argparse
import json
from typing import Sequence

from cleany_handeye_calibration.models import SampleSplit
from cleany_handeye_calibration.pose_manifest import (
    load_pose_manifest,
    preflight_pose_manifest,
)


def main(argv: Sequence[str] | None = None) -> None:
    parser = argparse.ArgumentParser(
        description='Preflight one materialized Cleany hand-eye pose manifest.'
    )
    parser.add_argument('manifest')
    arguments = parser.parse_args(argv)
    manifest = load_pose_manifest(arguments.manifest)
    validation = preflight_pose_manifest(manifest)
    print(
        json.dumps(
            {
                'schema_version': manifest.schema_version,
                'calibration_pose_count': len(
                    manifest.poses_for_split(SampleSplit.CALIBRATION)
                ),
                'held_out_pose_count': len(
                    manifest.poses_for_split(SampleSplit.HELD_OUT)
                ),
                'random_seed': manifest.generator.random_seed,
                'attempts_used': manifest.generator.attempts_used,
                'maximum_axis_parallelism': (
                    validation.computed_diversity.maximum_axis_parallelism
                ),
                'rotation_covariance_log_det': (
                    validation.computed_diversity.rotation_covariance_log_det
                ),
                'rotation_covariance_rank': (
                    validation.computed_diversity.rotation_covariance_rank
                ),
                'nonparallel_axis_pose_ids': (
                    validation.computed_diversity.nonparallel_axis_pose_ids
                ),
            },
            allow_nan=False,
            indent=2,
            sort_keys=True,
        )
    )


if __name__ == '__main__':
    main()
