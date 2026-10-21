import os
import signal
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import evaluation_stack as stack


def config(root):
    base = root / "base.yaml"
    base.write_text("mode: hardware\n")
    return stack.validate_runtime_config({
        "schema_version": 1, "startup_timeout_s": 1.0, "discovery_s": 0.01,
        "shutdown_timeout_s": 1.0,
        "agent": {"serial_port": "/dev/fake-agent", "baudrate": 115200},
        "base": {"config": "base.yaml"},
        "lidar": {"serial_port": "/dev/fake-lidar", "baudrate": 115200,
                  "frame_id": "laser", "inverted": False, "angle_compensate": True,
                  "scan_mode": "Standard", "node_name": "lidar",
                  "restart_stale": True, "restart_node_names": ["known_lidar"]},
    }, root)


class Probe:
    def __init__(self, ready=False):
        self.ready = ready
        self.calls = 0

    def spin(self, _seconds):
        self.calls += 1

    def snapshot(self):
        ready = self.ready or self.calls > 1
        return {"ready": ready, "errors": [], "odom_fresh": ready,
                "scan_fresh": ready, "wheel_fresh": ready, "enable_available": False,
                "publishers": {"cmd_vel": 0, "wheel_command": 0, "wheel_state": int(ready),
                               "odom": int(ready), "scan": int(ready)}}


class Proc:
    next_pid = 9000

    def __init__(self, *args, **kwargs):
        self.args = args[0]
        self.pid = Proc.next_pid
        Proc.next_pid += 1
        self.returncode = None

    def poll(self):
        return self.returncode

    def wait(self, timeout=None):
        self.returncode = 0
        return 0


class EvaluationStackTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.root = Path(self.tmp.name)
        self.cfg = config(self.root)
        self.spawn = patch.object(stack.subprocess, "Popen", Proc)
        self.spawn.start()
        self.addCleanup(self.spawn.stop)
        # These tests must never signal real host processes, even for fake PID reuse.
        for name in ("kill", "killpg"):
            patcher = patch.object(stack.os, name)
            patcher.start()
            self.addCleanup(patcher.stop)

    def tearDown(self):
        self.tmp.cleanup()

    def test_commands_are_argument_lists_with_hardware_settings(self):
        commands = stack.describe_commands(self.cfg, self.root)
        self.assertTrue(all(isinstance(cmd, list) for cmd in commands.values()))
        self.assertIn("mock:=false", commands["base"])
        self.assertIn("--dev", commands["agent"])
        self.assertEqual(commands["lidar"][:4], ["ros2", "run", "rplidar_ros", "rplidar_node"])
        self.assertIn("__node:=lidar", commands["lidar"])
        self.assertIn("scan_mode:=Standard", commands["lidar"])
        self.assertIn("angle_compensate:=true", commands["lidar"])

    def test_missing_values_and_bool_numeric_rejected(self):
        raw = {"schema_version": 1, "startup_timeout_s": True, "discovery_s": 1,
               "shutdown_timeout_s": 1}
        with self.assertRaises(ValueError):
            stack.validate_runtime_config(raw, self.root)

    def test_all_missing_launches_all_and_waits(self):
        with patch.object(stack, "_proc_entries", return_value=[]), \
             patch.object(stack.Path, "exists", return_value=True):
            obj = stack.EvaluationStack(self.cfg, self.root, self.root / "logs", Probe())
            result = obj.start()
        self.assertEqual(len(result["started"]), 3)
        self.assertTrue(result["snapshot"]["scan_fresh"])
        obj.close()

    def test_healthy_reuse_launches_nothing(self):
        probe = Probe(ready=True)
        probe.snapshot = lambda: {"ready": True, "errors": [], "odom_fresh": True, "scan_fresh": True,
                                  "wheel_fresh": True, "enable_available": True,
                                  "publishers": {"cmd_vel": 0, "wheel_command": 1, "wheel_state": 1,
                                                 "odom": 1, "scan": 1}}
        with patch.object(stack, "_proc_entries", return_value=[(21, "micro_ros_agent", "--dev\0/dev/fake-agent"),
                                                               (22, "rplidar_node", "--ros-args\0-r\0__node:=known_lidar\0-p\0serial_port:=/dev/fake-lidar")]):
            obj = stack.EvaluationStack(self.cfg, self.root, self.root / "logs", probe)
            self.assertEqual(obj.start()["started"], [])

    def test_partial_base_refuses_duplicate(self):
        probe = Probe()
        original = probe.snapshot
        probe.snapshot = lambda: {**original(), "ready": False, "odom_fresh": False,
                                  "scan_fresh": False, "wheel_fresh": False,
                                  "publishers": {**original()["publishers"], "wheel_command": 1}}
        with patch.object(stack, "_proc_entries", return_value=[]), patch.object(stack.Path, "exists", return_value=True):
            obj = stack.EvaluationStack(self.cfg, self.root, self.root / "logs", probe)
            with self.assertRaisesRegex(RuntimeError, "timed out"):
                obj.start()

    def test_missing_configured_device_fails_before_launch(self):
        with patch.object(stack, "_proc_entries", return_value=[]), patch.object(Path, "exists", return_value=False):
            obj = stack.EvaluationStack(self.cfg, self.root, self.root / "logs", Probe())
            with self.assertRaisesRegex(RuntimeError, "device missing"):
                obj.start()
        self.assertEqual(obj.processes, [])

    def test_cancellation_closes_owned_children(self):
        with patch.object(stack, "_proc_entries", return_value=[]), patch.object(Path, "exists", return_value=True), \
             patch.object(stack.os, "killpg") as killpg, patch.object(stack, "_pid_identity", return_value="fake"), \
             patch.object(stack, "_group_owned", side_effect=[True, False] * 3):
            obj = stack.EvaluationStack(self.cfg, self.root, self.root / "logs", Probe(),
                                        cancelled=lambda: bool(obj.processes))
            with self.assertRaisesRegex(RuntimeError, "cancelled"):
                obj.start()
        self.assertTrue(killpg.called)

    def test_timeout_reports_failure(self):
        class Never(Probe):
            def snapshot(self):
                return {"ready": False, "errors": ["no data"], "odom_fresh": False,
                        "scan_fresh": False, "wheel_fresh": False, "enable_available": False,
                        "publishers": {"cmd_vel": 0, "wheel_command": 0, "wheel_state": 0, "odom": 0, "scan": 0}}
        with patch.object(stack, "_proc_entries", return_value=[]), patch.object(Path, "exists", return_value=True), \
             patch.object(stack.time, "monotonic", side_effect=[0, 0, 2, 2, 2, 2, 2]):
            obj = stack.EvaluationStack(self.cfg, self.root, self.root / "logs", Never())
            with self.assertRaisesRegex(RuntimeError, "timed out"):
                obj.start()

    def test_exact_process_tokens(self):
        self.assertTrue(stack.EvaluationStack._serial_match("--dev\0/dev/fake-lidar", "/dev/fake-lidar"))
        self.assertTrue(stack.EvaluationStack._serial_match("serial_port:=/dev/fake-lidar", "/dev/fake-lidar"))
        self.assertTrue(stack.EvaluationStack._serial_match("--dev=/dev/fake-lidar", "/dev/fake-lidar"))
        self.assertFalse(stack.EvaluationStack._serial_match("/dev/fake-lidar-extra", "/dev/fake-lidar"))

    def test_stale_known_lidar_restarts_exact_pid(self):
        probe = Probe()
        snapshot_calls = 0
        def stale_then_fresh():
            nonlocal snapshot_calls
            snapshot_calls += 1
            fresh = snapshot_calls > 1
            return {"ready": fresh, "errors": [], "odom_fresh": fresh, "scan_fresh": fresh,
                    "wheel_fresh": fresh, "enable_available": True,
                    "publishers": {"cmd_vel": 0, "wheel_command": 1, "wheel_state": 1,
                                   "odom": 1, "scan": int(fresh)}}
        probe.snapshot = stale_then_fresh
        records = [(31, "rplidar_node", "--ros-args\0-r\0__node:=known_lidar\0-p\0serial_port:=/dev/fake-lidar")]
        with patch.object(stack, "_proc_entries", return_value=records), \
             patch.object(stack.Path, "exists", return_value=True), \
             patch.object(stack, "_pid_identity", return_value="identity"), \
             patch.object(stack.EvaluationStack, "_stop_pid") as stop:
            obj = stack.EvaluationStack(self.cfg, self.root, self.root / "logs", probe)
            result = obj.start()
        stop.assert_called_once_with(31, "identity")
        self.assertEqual(len(result["started"]), 1)
        self.assertEqual(result["restarted_lidar_pids"], [31])

    def test_unknown_or_prefix_node_stale_refused(self):
        for node in ("other", "known_lidar_extra"):
            with self.subTest(node=node):
                records = [(31, "rplidar_node", f"--ros-args\0-r\0__node:={node}\0-p\0serial_port:=/dev/fake-lidar")]
                with patch.object(stack, "_proc_entries", return_value=records), \
                     patch.object(stack.Path, "exists", return_value=True), \
                     patch.object(stack.EvaluationStack, "_stop_pid") as stop, \
                     patch.object(stack.subprocess, "Popen") as spawn:
                    obj = stack.EvaluationStack(self.cfg, self.root, self.root / "logs", Probe())
                    with self.assertRaisesRegex(RuntimeError, "not an identifiable"):
                        obj.start()
                stop.assert_not_called()
                spawn.assert_not_called()

    def test_early_owned_process_exit_fails(self):
        class Exited(Proc):
            def __init__(self, *args, **kwargs):
                super().__init__(*args, **kwargs)
                self.returncode = 7
        with patch.object(stack.subprocess, "Popen", Exited), \
             patch.object(stack, "_proc_entries", return_value=[]), \
             patch.object(stack.Path, "exists", return_value=True), \
             patch.object(stack, "_pid_identity", return_value="fake"):
            obj = stack.EvaluationStack(self.cfg, self.root, self.root / "logs", Probe())
            with self.assertRaisesRegex(RuntimeError, "exited early"):
                obj.start()

    def test_stale_scan_without_identifiable_process_refused(self):
        probe = Probe()
        probe.snapshot = lambda: {"ready": False, "errors": [], "odom_fresh": False,
                                  "scan_fresh": False, "wheel_fresh": False, "enable_available": False,
                                  "publishers": {"cmd_vel": 0, "wheel_command": 0, "wheel_state": 0,
                                                 "odom": 0, "scan": 1}}
        with patch.object(stack, "_proc_entries", return_value=[]), patch.object(stack.Path, "exists", return_value=True), patch.object(stack.subprocess, "Popen") as spawn:
            obj = stack.EvaluationStack(self.cfg, self.root, self.root / "logs", probe)
            with self.assertRaisesRegex(RuntimeError, "no identifiable"):
                obj.start()
        spawn.assert_not_called()

    def test_agent_only_graph_starts_missing_driver_without_duplicate_agent(self):
        probe = Probe()
        original = probe.snapshot
        probe.snapshot = lambda: {
            **original(), "publishers": {**original()["publishers"], "wheel_state": 1}
        }
        with patch.object(stack, "_proc_entries", return_value=[]), \
             patch.object(stack.Path, "exists", return_value=True):
            obj = stack.EvaluationStack(self.cfg, self.root, self.root / "logs", probe)
            obj.start()
        self.assertEqual(len(obj.processes), 2)
        self.assertEqual(obj.processes[0][0].args[2], "cleany_base_driver")

    def test_pid_reuse_does_not_authorize_group_signals(self):
        with patch.object(stack, "_pid_identity", return_value="new-start-time"):
            self.assertFalse(stack._group_owned(123, "old-start-time"))

    def test_cleanup_escalates_only_owned_group(self):
        process = Proc(["fake"])
        obj = stack.EvaluationStack(self.cfg, self.root, self.root / "logs", Probe())
        obj.processes = [(process, process.pid, "original")]
        with patch.object(stack, "_group_owned", return_value=True), \
             patch.object(stack.time, "monotonic", side_effect=range(100)), \
             patch.object(stack.os, "killpg") as killpg:
            obj.close()
        self.assertEqual(killpg.call_args_list, [
            unittest.mock.call(process.pid, signal.SIGINT),
            unittest.mock.call(process.pid, signal.SIGTERM),
            unittest.mock.call(process.pid, signal.SIGKILL),
        ])


if __name__ == "__main__":
    unittest.main()
