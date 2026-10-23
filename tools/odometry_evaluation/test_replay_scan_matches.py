import json
import math
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import numpy as np

import replay_scan_matches as replay
from lidar_scan_matcher import ScanMatchResult


def match_result():
    return ScanMatchResult(
        delta_x_m=1.0, delta_y_m=2.0, delta_yaw_rad=-math.pi + .01,
        delta_yaw_deg=math.degrees(-math.pi + .01), rmse_m=.01, rmse_mm=10,
        inlier_ratio=.8, num_inliers=40, num_total_points=50, converged=True,
        iterations=3, condition_number=1.0,
    )


class SpyMatcher:
    instances = []

    def __init__(self, **kwargs):
        self.kwargs = kwargs
        self.guesses = []
        self.instances.append(self)

    def match(self, ref, target, initial_guess):
        self.guesses.append(initial_guess)
        return match_result()


class ReplayTests(unittest.TestCase):
    def test_se2_guess_config_and_null_record(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            np.savez(root / "scans.npz", start=np.zeros((20, 2)),
                     destination=np.ones((20, 2)), returned=np.ones((20, 2)) * 2)
            record = {"axis": "x", "repetition": 1, "result": {
                "raw_scan_file": "scans.npz",
                "extrinsic_calibration_applied": {"yaw_deg": 30, "lever_arm_x_m": .2, "lever_arm_y_m": -.1},
                "evaluation_settings": {"icp_min_inlier_ratio": .3, "icp_convergence": {"max_period": 4}},
                "outbound": {"is_valid": True, "odom_reported": {"dx_m": 1, "dy_m": 0, "dyaw_deg": 90},
                             "raw_lidar_icp": {"delta_x_m": 0, "delta_y_m": 0, "delta_yaw_rad": math.pi - .01}},
                "inbound": {"is_valid": False, "odom_reported": {"dx_m": 1, "dy_m": 0, "dyaw_deg": 0},
                            "raw_lidar_icp": {"delta_x_m": 0, "delta_y_m": 0, "delta_yaw_rad": 0}},
                "raw_loop_icp": {"is_valid": True, "delta_x_m": 0, "delta_y_m": 0, "delta_yaw_rad": 0},
            }}
            summary = root / "summary.json"
            summary.write_text(json.dumps({"records": [{"result": None}, record]}))
            SpyMatcher.instances.clear()
            before = summary.read_bytes()
            with patch.object(replay, "LidarScanMatcher", SpyMatcher), patch.object(replay, "ROOT", root):
                replay.replay_summary(summary)
                rows = replay.replay_summary(
                    summary, min_inlier_ratio=.55, max_iterations=12,
                    convergence_config={"max_period": 7})
            self.assertEqual(SpyMatcher.instances[0].kwargs["min_inlier_ratio"], .3)
            self.assertEqual(SpyMatcher.instances[0].kwargs["convergence_config"], {"max_period": 4})
            spy = SpyMatcher.instances[1]
            self.assertEqual(len(rows), 3)
            self.assertEqual(spy.kwargs["min_inlier_ratio"], .55)
            self.assertEqual(spy.kwargs["convergence_config"], {"max_period": 7})
            self.assertEqual(spy.kwargs["max_iterations"], 12)
            self.assertTrue(rows[2]["previously_valid"])
            self.assertFalse(rows[1]["previously_valid"])
            self.assertEqual(rows[0]["applied_min_inlier_ratio"], .55)
            self.assertEqual(rows[0]["applied_max_iterations"], 12)
            self.assertEqual(summary.read_bytes(), before)
            # Loop composition: (1,0,90deg) then (1,0,0) => (1,1,90deg).
            expected = replay.PlanarExtrinsic(30, .2, -.1).base_to_lidar(1, 1, math.pi / 2)
            for actual, value in zip(spy.guesses[2], expected):
                self.assertAlmostEqual(actual, value)
            self.assertAlmostEqual(rows[0]["yaw_change_rad"], .02)

    def test_report_counts_and_existing_output_refusal(self):
        rows = [
            {"previously_valid": True, "newly_valid": False, "translation_change_m": 1, "yaw_change_rad": .2},
            {"previously_valid": False, "newly_valid": True, "translation_change_m": None, "yaw_change_rad": None},
            {"previously_valid": False, "newly_valid": False, "translation_change_m": None, "yaw_change_rad": None},
        ]
        report = replay.build_report(rows)
        self.assertEqual(report["counts"], {"previously_valid": 1, "newly_valid": 1,
                                            "still_invalid": 1, "regressed": 1, "improved": 1})
        with tempfile.TemporaryDirectory() as tmp:
            target = Path(tmp) / "report.json"
            target.write_text("{}")
            from evaluation_results import write_data_json
            with self.assertRaises(FileExistsError):
                write_data_json(target, report)
            self.assertEqual(target.read_text(), "{}")

    def test_cli_publishes_fresh_report_and_refuses_existing_output(self):
        with tempfile.TemporaryDirectory() as tmp:
            target = Path(tmp) / "report.json"
            source = Path(tmp) / "summary.json"
            row = {"previously_valid": False, "newly_valid": True,
                   "translation_change_m": .001, "yaw_change_rad": .0001}
            argv = [str(source), "--min-inlier-ratio", ".3", "--max-iterations", "33",
                    "--output", str(target)]
            with patch.object(replay, "replay_summary", return_value=[row]) as replayed:
                self.assertEqual(replay.main(argv), 0)
                replayed.assert_called_once_with(
                    source, min_inlier_ratio=.3, max_iterations=33)
                before = target.read_bytes()
                self.assertEqual(json.loads(before)["counts"]["newly_valid"], 1)
                with self.assertRaises(FileExistsError):
                    replay.main(argv)
                self.assertEqual(target.read_bytes(), before)


if __name__ == "__main__":
    unittest.main()
