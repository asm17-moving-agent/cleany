"""Check recording estimates across bag splits, recorder changes and SSH gaps."""

import importlib.util
import tempfile
import unittest
from contextlib import nullcontext
from pathlib import Path
from unittest.mock import patch


source = Path(__file__).resolve().parents[2] / "tools/validation/floor19_panel.py"
spec = importlib.util.spec_from_file_location("floor19_panel", source)
panel = importlib.util.module_from_spec(spec)
spec.loader.exec_module(panel)


class RecordingMetricsTests(unittest.TestCase):
    def test_recent_images_can_span_numeric_splits_while_odometry_continues(self):
        with tempfile.TemporaryDirectory() as tmp:
            bag = Path(tmp) / "recording"
            bag.mkdir()
            now = 30_000_000_000
            topics = list(panel.SENSOR_TOPICS) + ["/wheel/odom"]
            for index, ids in ((8, [4]), (9, [0, 1]), (10, [2, 3, 5])):
                with panel.closing(panel.sqlite3.connect(bag / f"recording_{index}.db3")) as db:
                    db.executescript("CREATE TABLE topics(id INTEGER, name TEXT); "
                                     "CREATE TABLE messages(topic_id INTEGER, timestamp INTEGER);")
                    db.executemany("INSERT INTO topics VALUES (?, ?)", enumerate(topics))
                    db.executemany("INSERT INTO messages VALUES (?, ?)", [(i, now-1) for i in ids])
                    db.execute("INSERT INTO messages VALUES (4, ?)", (now-9_000_000_000,))
                    db.commit()
            self.assertEqual(panel.missing_sensors(bag, now), ["LiDAR"])
            self.assertEqual(panel.missing_sensors(bag, now+10_000_000_000),
                             list(panel.SENSOR_TOPICS.values()))

    def test_only_files_in_the_active_bag_count(self):
        with tempfile.TemporaryDirectory() as tmp:
            run = Path(tmp)
            bag = run / "recording"
            bag.mkdir()
            (bag / "part0.db3").write_bytes(b"x" * 100)
            (bag / "part1.db3").write_bytes(b"x" * 30)
            (run / "checkpoint").write_bytes(b"x" * 200)
            units = ("ActiveState=failed\nMainPID=0\n\nId=cleany-floor19-record.service\nActiveState=active\nMainPID=42\n"
                     "InvocationID=active\nExecMainStartTimestampMonotonic=13000000")
            with patch.object(panel, "RUNS", run), patch.object(panel, "command", return_value=units), \
                    patch.object(panel, "recorder_directory", return_value=bag), \
                    patch.object(panel.time, "monotonic", return_value=100.0):
                result = panel.recording_status()
            self.assertEqual(result["bag"]["size"], 130)
            self.assertEqual(result["bag"]["seconds"], 87)
            self.assertEqual(result["disk"], str(bag))

    def test_rate_needs_recent_samples_from_one_uninterrupted_recorder(self):
        rate = panel.RecordingRate()
        self.assertIsNone(rate.update({"id": "a", "size": 100}, 0))
        self.assertEqual(rate.update({"id": "a", "size": 260}, 16), 10)
        self.assertIsNone(rate.update({"id": "b", "size": 500}, 17))
        self.assertIsNone(rate.update({"id": "b", "size": 500}, 47))
        self.assertIsNone(rate.update(None, 49))
        self.assertIsNone(rate.update({"id": "b", "size": 800}, 60))
        self.assertIsNone(rate.update({"id": "b", "size": 750}, 90))
        self.assertIsNone(rate.update({"id": "b", "size": 1050}, 160))

    def test_estimate_excludes_reserve_and_handles_full_disk(self):
        self.assertEqual(panel.estimate_label(11 * panel.GIB, panel.GIB / 600), "약 10분")
        self.assertEqual(panel.estimate_label(9 * panel.GIB, None), "0분")
        self.assertEqual(panel.estimate_label(12 * panel.GIB, None), "속도 측정 중")


class BagResetTests(unittest.TestCase):
    def setUp(self):
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        self.run_dir = Path(tmp.name) / "session"
        self.run_dir.mkdir()
        self.bag = self.run_dir / "recording-vslam"
        self.bag.mkdir()
        for i in range(2):
            (self.bag / f"recording-vslam_{i}.db3").write_text("old")
        self.checkpoint = self.run_dir / "checkpoints" / "map.pbstream"
        self.checkpoint.parent.mkdir()
        self.checkpoint.write_text("keep")
        self.other = self.run_dir / "archived-bag"
        self.other.mkdir()
        self.unit = "cleany-floor19-record-vslam.service"
        self.status = {"unit": self.unit, "path": str(self.bag), "id": "old"}
        self.operations = []

    def test_paused_status_does_not_report_sensor_loss(self):
        self.status["mode"] = "paused"
        with patch.object(panel, "unit_state", return_value={"ActiveState": "active", "InvocationID": "map"}), \
                patch.object(panel, "recording_status", return_value={"recorder_count": 1, "bag": self.status}), \
                patch.object(panel, "missing_sensors") as inspect:
            self.assertEqual(panel.remote("status", "")["bag"]["mode"], "paused")
            inspect.assert_not_called()

    def test_stop_saves_bag_and_latches_manual_stop_independently_of_mapper(self):
        (self.run_dir / "session").touch()
        with patch.object(panel, "RUNS", self.run_dir.parent), \
                patch.object(panel, "confirmed_bag", return_value=self.status), \
                patch.object(panel, "command") as ctl:
            ctl.side_effect = lambda *a, **kw: (self.bag / "metadata.yaml").write_text("metadata")
            self.assertEqual(panel.control_bag("stop-bag", "old"), {"path": str(self.bag)})
            ctl.assert_called_once_with("systemctl", "--user", "stop", self.unit, timeout=110)
            self.assertTrue((panel.RUNS / ".floor19-recording-stopped").exists())
            self.assertTrue((self.bag / "recording-vslam_0.db3").exists())

    def control(self, _ctl, _user, operation, unit, **kwargs):
        self.assertEqual(unit, self.unit)
        self.operations.append(operation)
        if operation == "stop":
            (self.bag / "metadata.yaml").touch()
        elif operation == "start":
            self.bag.mkdir()
            self.status["id"] = "new"
            with panel.closing(panel.sqlite3.connect(self.bag / "recording-vslam_0.db3")) as db:
                db.executescript("create table messages(id integer); insert into messages values(1);")

    def execute(self, token="old"):
        with patch.object(panel, "recording_status", return_value={"recorder_count": 1, "bag": self.status}), \
                patch.object(panel, "pinned_unit", return_value=nullcontext()), \
                patch.object(panel, "restart_mapper", side_effect=lambda *_: self.operations.append("map")), \
                patch.object(panel, "command", side_effect=self.control):
            return panel.reset_bag(self.run_dir, "mapper", token)

    def test_replaces_all_splits_after_new_bag_records(self):
        self.assertEqual(self.execute(), {"path": str(self.bag)})
        self.assertEqual(self.operations, ["map", "stop", "start"])
        self.assertEqual([p.name for p in self.bag.iterdir()], ["recording-vslam_0.db3"])
        self.assertEqual(set(self.run_dir.iterdir()), {self.bag, self.checkpoint.parent, self.other})
        self.assertEqual(self.checkpoint.read_text(), "keep")

    def test_failed_replacement_preserves_all_previous_splits(self):
        with patch.object(panel, "wait_for_new_bag", side_effect=RuntimeError("no data")):
            with self.assertRaisesRegex(RuntimeError, "기존 bag 보관 위치"):
                self.execute()
        previous, = self.run_dir.glob("recording-vslam-before-reset-*")
        self.assertEqual([f.read_text() for f in sorted(previous.glob("*.db3"))], ["old", "old"])
        self.assertTrue(self.checkpoint.is_file())

    def test_stale_confirmation_or_foreign_files_prevent_changes(self):
        with self.assertRaisesRegex(RuntimeError, "세션이 바뀌"):
            self.execute("different")
        link = self.bag / "recording-vslam_2.db3"
        link.symlink_to(self.checkpoint)
        with self.assertRaisesRegex(RuntimeError, "bag 외의 파일"):
            self.execute()
        self.assertEqual(self.operations, [])
        self.assertTrue(link.is_symlink())
