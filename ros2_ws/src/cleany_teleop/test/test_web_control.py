from types import SimpleNamespace
import unittest

from cleany_teleop.web_control import Control


class WebControlTests(unittest.TestCase):
    def setUp(self):
        self.now = 100.0
        self.sent = []
        self.control = Control(self.sent.append, lambda: self.now)
        self.control.subscriber_count = 1
        self.sequence = 0
        self.feed()

    def feed(self, enabled=False, accepted=0, boot_id=123):
        self.sequence += 1
        self.control.feedback(
            SimpleNamespace(
                protocol_version=2,
                boot_id=boot_id,
                sequence=self.sequence,
                timestamp_us=1_000_000 + int((self.now - 100) * 1e6),
                last_command_sequence=accepted,
                watchdog_ms=250,
                max_velocity_rad_s=10,
                enabled=enabled,
                fault_bits=0,
                command_age_ms=0,
                target_rad_s=[0.0] * 4,
                commanded_rad_s=[0.0] * 4,
                velocity_rad_s=[0.0] * 4,
                pwm_percent=[0] * 4,
            )
        )

    def press(self):
        self.control.request(
            dict(
                client="phone",
                sequence=1,
                action="press",
                direction="forward",
                speed=0.5,
            )
        )

    def moving(self):
        self.press()
        self.now += 0.02
        self.feed(accepted=self.sent[-1].sequence)
        self.control.tick()
        self.assertEqual(self.sent[-1].mode, Control.ENABLE)
        self.now += 0.02
        self.feed(enabled=True, accepted=self.sent[-1].sequence)
        self.control.tick()
        self.assertEqual(self.sent[-1].mode, Control.VELOCITY)

    def test_session_speed_ceiling_rejects_motion_before_enable(self):
        self.control.MAX_WEB_VELOCITY = 1.0
        with self.assertRaises(ValueError):
            self.control.request(
                dict(
                    client="phone",
                    sequence=1,
                    action="press",
                    direction="forward",
                    speed=1.1,
                )
            )
        self.assertFalse(self.sent)
        self.assertEqual(self.control.status()["web_max_velocity"], 1.0)

    def test_stop_invalidates_late_requests(self):
        self.moving()
        self.control.request(dict(client="phone", sequence=3, action="stop"))
        with self.assertRaises(ValueError):
            self.control.request(dict(client="phone", sequence=2, action="heartbeat"))
        self.assertEqual(self.sent[-1].mode, Control.STOP)
        self.assertIsNone(self.control.owner)

    def test_expired_lease_cannot_be_revived_by_heartbeat(self):
        self.moving()
        self.now += 0.26
        with self.assertRaises(ValueError):
            self.control.request(dict(client="phone", sequence=2, action="heartbeat"))
        self.assertEqual(self.sent[-1].mode, Control.STOP)

    def test_stale_feedback_and_reboot_stop(self):
        self.moving()
        self.now += 0.11
        self.control.tick()
        self.assertEqual(self.sent[-1].mode, Control.STOP)
        self.now += 0.01
        self.feed(boot_id=124)
        self.assertIsNone(self.control.owner)
        self.assertEqual(self.sent[-1].mode, Control.STOP)

    def test_enable_retry_keeps_deadline_and_never_moves_without_ack(self):
        self.press()
        self.now += 0.02
        self.feed(accepted=self.sent[-1].sequence)
        self.control.tick()
        original = self.sent[-1]
        for _ in range(6):
            self.now += 0.02
            self.feed()
            self.control.tick()
        enables = [c for c in self.sent if c.mode == Control.ENABLE]
        self.assertTrue(
            all(
                c.sequence == original.sequence
                and c.valid_until_us == original.valid_until_us
                for c in enables
            )
        )
        self.assertFalse(any(c.mode == Control.VELOCITY for c in self.sent))
        self.assertEqual(self.sent[-1].mode, Control.STOP)

    def test_second_phone_cannot_take_over_active_control(self):
        self.moving()
        with self.assertRaises(ValueError):
            self.control.request(
                dict(
                    client="second",
                    sequence=1,
                    action="press",
                    direction="right",
                    speed=0.5,
                )
            )
        self.assertEqual(self.control.owner, "phone")
        self.control.request(dict(client="second", sequence=2, action="stop"))
        self.assertEqual(self.sent[-1].mode, Control.STOP)

    def test_unchanged_held_input_runs_beyond_ten_seconds_then_expires(self):
        self.moving()
        baseline = len(self.sent)
        for seq in range(2, 602):
            self.now += 0.07
            self.feed(enabled=True)
            self.control.request(
                dict(
                    client="phone",
                    sequence=seq,
                    action="update",
                    vector=[1, 0, 0],
                    speed=0.5,
                )
            )
            self.control.tick()
        self.assertGreater(self.now - self.control.started_at, 40)
        self.assertEqual(self.control.phase, "moving")
        self.assertTrue(all(c.mode == Control.VELOCITY for c in self.sent[baseline:]))
        self.now += 0.26
        self.feed(enabled=True)
        self.control.tick()
        self.assertEqual(self.sent[-1].mode, Control.STOP)
        self.assertIsNone(self.control.owner)

    def test_competing_ros_publisher_and_invalid_speed_rejected(self):
        self.control.publisher_count = 2
        with self.assertRaises(ValueError):
            self.press()
        self.assertFalse(self.sent)
        self.control.publisher_count = 1
        with self.assertRaises(ValueError):
            self.control.request(
                dict(
                    client="phone",
                    sequence=2,
                    action="press",
                    direction="forward",
                    speed=float("nan"),
                )
            )
        self.assertFalse(self.sent)

    def test_direction_updates_do_not_stop_or_repeat_enable(self):
        self.moving()
        baseline = len(self.sent)
        started_at = self.control.started_at
        for sequence, direction in [(2, "left"), (3, "forward"), (4, "cw")]:
            self.now += 0.01
            self.feed(enabled=True)
            self.control.request(
                dict(
                    client="phone",
                    sequence=sequence,
                    action="update",
                    direction=direction,
                    speed=1,
                )
            )
            self.control.tick()
        self.assertTrue(all(c.mode == Control.VELOCITY for c in self.sent[baseline:]))
        self.assertEqual(self.sent[-1].velocity_rad_s, (1.0, -1.0, 1.0, -1.0))
        self.assertEqual(self.control.started_at, started_at)

    def test_expired_or_stopped_session_cannot_resume_through_update(self):
        self.moving()
        self.now += 0.26
        self.feed(enabled=True)
        with self.assertRaises(ValueError):
            self.control.request(
                dict(
                    client="phone",
                    sequence=2,
                    action="update",
                    direction="left",
                    speed=1,
                )
            )
        self.assertEqual(self.sent[-1].mode, Control.STOP)
        with self.assertRaises(ValueError):
            self.control.request(
                dict(
                    client="phone",
                    sequence=3,
                    action="update",
                    direction="left",
                    speed=1,
                )
            )

    def test_fresh_press_after_expiry_repeats_stop_enable_ack_and_uses_latest_target(
        self,
    ):
        self.moving()
        self.now += 0.26
        self.feed(enabled=True)
        self.control.tick()
        self.assertEqual(self.sent[-1].mode, Control.STOP)
        self.now += 0.02
        self.feed(accepted=self.sent[-1].sequence)
        self.control.tick()
        self.assertEqual(self.control.phase, "idle")
        self.control.request(
            dict(
                client="phone", sequence=2, action="press", vector=[0, 1, 0], speed=0.5
            )
        )
        self.assertEqual(self.sent[-1].mode, Control.STOP)
        self.now += 0.02
        self.feed(accepted=self.sent[-1].sequence)
        self.control.tick()
        self.assertEqual(self.sent[-1].mode, Control.ENABLE)
        self.now += 0.02
        self.feed(enabled=True, accepted=self.sent[-1].sequence)
        self.control.tick()
        self.assertEqual(self.sent[-1].mode, Control.VELOCITY)
        self.assertEqual(self.sent[-1].velocity_rad_s, (-0.5, 0.5, 0.5, -0.5))

    def test_other_phone_cannot_update_direction(self):
        self.moving()
        with self.assertRaises(ValueError):
            self.control.request(
                dict(
                    client="second",
                    sequence=1,
                    action="update",
                    direction="left",
                    speed=1,
                )
            )
        self.assertEqual(self.control.targets, (0.5,) * 4)

    def test_joystick_mixing_zero_and_live_updates_keep_session(self):
        self.moving()
        baseline = len(self.sent)
        started = self.control.started_at
        for seq, vector, expected in [
            (2, [1, 1, 0], (0.0, 2.0, 2.0, 0.0)),
            (3, [0, 0, 0], (0.0,) * 4),
            (4, [0, 0, -0.5], (1.0, -1.0, 1.0, -1.0)),
        ]:
            self.now += 0.02
            self.feed(enabled=True)
            self.control.request(
                dict(
                    client="phone",
                    sequence=seq,
                    action="update",
                    vector=vector,
                    speed=2,
                )
            )
            self.control.tick()
            self.assertEqual(self.sent[-1].velocity_rad_s, expected)
        self.assertTrue(all(c.mode == Control.VELOCITY for c in self.sent[baseline:]))
        self.assertEqual(self.control.started_at, started)

    def test_invalid_joystick_vectors_never_publish_motion(self):
        for seq, vector in enumerate(
            ([1, 0], [2, 0, 0], [True, 0, 0], [float("nan"), 0, 0], "forward"), 1
        ):
            with self.assertRaises(ValueError):
                self.control.request(
                    dict(
                        client="phone",
                        sequence=seq,
                        action="press",
                        vector=vector,
                        speed=1,
                    )
                )
        self.assertFalse(self.sent)

    def test_ten_rad_s_accepts_mixed_motion_but_respects_both_limits(self):
        self.moving()
        self.control.request(
            dict(
                client="phone", sequence=2, action="update", vector=[1, 1, 1], speed=10
            )
        )
        self.control.tick()
        self.assertEqual(max(map(abs, self.sent[-1].velocity_rad_s)), 10)
        targets = self.control.targets
        with self.assertRaises(ValueError):
            self.control.request(
                dict(
                    client="phone",
                    sequence=3,
                    action="update",
                    vector=[1, 0, 0],
                    speed=10.1,
                )
            )
        self.assertEqual(self.control.targets, targets)
        self.control.state.max_velocity_rad_s = 3
        with self.assertRaises(ValueError):
            self.control.request(
                dict(
                    client="phone",
                    sequence=4,
                    action="update",
                    vector=[1, 0, 0],
                    speed=10,
                )
            )
        self.assertEqual(self.control.status()["web_max_velocity"], 3)
        self.control.request(
            dict(client="phone", sequence=5, action="update", vector=[1, 0, 0], speed=3)
        )
        self.control.tick()
        self.assertEqual(self.sent[-1].velocity_rad_s, (3.0,) * 4)

    def test_reset_pause_stops_and_rejects_new_motion_until_remote_restart(self):
        self.moving()
        self.control.request(dict(client="reset", sequence=1, action="pause"))
        self.assertEqual(self.sent[-1].mode, self.control.STOP)
        self.assertTrue(self.control.paused)
        self.assertFalse(self.control.status()["ready"])
        self.now += 0.02
        self.feed(enabled=False, accepted=self.sent[-1].sequence)
        self.control.tick()
        self.assertTrue(self.control.status()["stop_confirmed"])
        before = len(self.sent)
        with self.assertRaises(ValueError):
            self.control.request(
                dict(
                    client="phone",
                    sequence=3,
                    action="press",
                    vector=[1, 0, 0],
                    speed=10,
                )
            )
        self.assertEqual(len(self.sent), before)
        self.control.request(dict(client="phone", sequence=4, action="stop"))
        self.assertEqual(self.sent[-1].mode, self.control.STOP)

    def test_http_acceptance_is_not_esp_stop_confirmation(self):
        self.moving()
        self.control.request(dict(client="phone", sequence=2, action="stop"))
        self.assertFalse(self.control.status()["stop_confirmed"])
        self.assertIsNone(self.control.status()["stop_ack_ms"])
        self.now += 0.02
        self.feed(enabled=False, accepted=self.sent[-1].sequence)
        self.control.tick()
        self.assertTrue(self.control.status()["stop_confirmed"])
        self.assertAlmostEqual(self.control.status()["stop_ack_ms"], 20)

    def test_status_separates_target_and_feedback_and_advertises_limits(self):
        self.moving()
        status = self.control.status()
        self.assertEqual(status["target"], [0.5] * 4)
        self.assertEqual(status["velocity"], [0.0] * 4)
        self.assertEqual(status["web_max_velocity"], 10.0)
        self.assertEqual(status["mcu_max_velocity"], 10.0)

    def test_fault_reason_survives_later_user_stop_and_state_polling(self):
        self.moving()
        self.now += 0.11
        self.control.tick()
        reason = self.control.status()["last_fault_reason"]
        self.assertIn("100 ms", reason)
        self.control.request(
            dict(client="phone", sequence=3, action="stop", reason="입력 해제")
        )
        status = self.control.status()
        self.assertEqual(status["last_fault_reason"], reason)
        self.assertIn("입력 해제", status["last_stop_reason"])
        event = [
            event
            for event in status["events"]
            if event["kind"] == "stop" and event["source"] == "safety"
        ][-1]
        self.assertGreater(event["feedback_age_ms"], 100)
        self.assertEqual(event["phase"], "moving")

    def test_reordered_update_is_identified_without_shortening_lease_or_stopping(self):
        self.moving()
        self.now += 0.07
        self.feed(enabled=True)
        self.control.request(
            dict(
                client="phone", sequence=4, action="update", vector=[0, 1, 0], speed=0.5
            )
        )
        lease = self.control.lease_until
        with self.assertRaisesRegex(ValueError, "이전 요청"):
            self.control.request(
                dict(
                    client="phone",
                    sequence=3,
                    action="update",
                    vector=[1, 0, 0],
                    speed=0.5,
                )
            )
        self.assertEqual(self.control.phase, "moving")
        self.assertEqual(self.control.lease_until, lease)
        self.assertEqual(self.control.targets, (-0.5, 0.5, 0.5, -0.5))


if __name__ == "__main__":
    unittest.main()
