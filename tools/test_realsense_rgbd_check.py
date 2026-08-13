from __future__ import annotations

import importlib.util
from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace
import unittest


def _load_module() -> ModuleType:
    path = Path(__file__).with_name('realsense_rgbd_check.py')
    spec = importlib.util.spec_from_file_location('realsense_rgbd_check', path)
    assert spec is not None
    assert spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


checker = _load_module()


def _message(
    stamp_ns: int,
    *,
    encoding: str = '',
    data: bytes = b'',
) -> SimpleNamespace:
    return SimpleNamespace(
        header=SimpleNamespace(
            stamp=SimpleNamespace(
                sec=stamp_ns // 1_000_000_000,
                nanosec=stamp_ns % 1_000_000_000,
            ),
            frame_id='camera_color_optical_frame',
        ),
        width=2,
        height=2,
        encoding=encoding,
        data=data,
        k=[100.0, 0.0, 1.0, 0.0, 100.0, 1.0, 0.0, 0.0, 1.0],
    )


class RgbdCheckTest(unittest.TestCase):
    def test_monitor_requires_one_exact_stamp_for_all_topics(self) -> None:
        monitor = checker.RgbdMonitor(depth_scale_m=0.001)
        messages = {
            'color': _message(10, encoding='rgb8', data=bytes(12)),
            'color_info': _message(10),
            'depth': _message(
                10,
                encoding='16UC1',
                data=b'\x01\x00' * 4,
            ),
            'depth_info': _message(10),
        }

        for kind, message in messages.items():
            monitor.add(kind, message, now=1.0)

        self.assertEqual(monitor.complete_count, 1)
        self.assertEqual(monitor.invalid_count, 0)
        self.assertEqual(monitor.minimum_valid_depth_ratio, 1.0)

    def test_mismatched_stamp_does_not_complete_bundle(self) -> None:
        monitor = checker.RgbdMonitor(depth_scale_m=0.001)
        for index, kind in enumerate(checker.TOPIC_KINDS):
            monitor.add(kind, _message(index + 1), now=1.0)

        self.assertEqual(monitor.complete_count, 0)
        self.assertEqual(len(monitor.pending), 4)

    def test_intrinsics_mismatch_is_invalid(self) -> None:
        monitor = checker.RgbdMonitor(depth_scale_m=0.001)
        messages = {
            'color': _message(10, encoding='rgb8', data=bytes(12)),
            'color_info': _message(10),
            'depth': _message(
                10,
                encoding='16UC1',
                data=b'\x01\x00' * 4,
            ),
            'depth_info': _message(10),
        }
        messages['depth_info'].k[0] = 101.0

        for kind, message in messages.items():
            monitor.add(kind, message, now=1.0)

        self.assertEqual(monitor.complete_count, 0)
        self.assertEqual(monitor.invalid_count, 1)
        self.assertIn('intrinsics', monitor.errors[0])

    def test_report_detects_trailing_stream_outage(self) -> None:
        monitor = checker.RgbdMonitor(depth_scale_m=0.001)
        monitor.first_complete_monotonic = 1.0
        monitor.last_complete_monotonic = 2.0
        monitor.first_stamp_ns = 1_000_000_000
        monitor.last_stamp_ns = 2_000_000_000
        monitor.complete_count = 10
        monitor.minimum_valid_depth_ratio = 0.5
        monitor.topic_counts = {kind: 10 for kind in checker.TOPIC_KINDS}

        report = monitor.report(
            requested_duration_seconds=10.0,
            elapsed_seconds=10.0,
            minimum_rate_hz=5.0,
            maximum_gap_seconds=2.0,
            started_at_monotonic=0.0,
        )

        self.assertEqual(report['trailing_gap_seconds'], 8.0)
        self.assertFalse(report['checks']['maximum_gap_met'])
        self.assertFalse(report['passed'])


if __name__ == '__main__':
    unittest.main()
