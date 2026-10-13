"""Network worker uses bounded ROS RPCs; ROS callbacks never perform socket I/O."""

import json
import threading
import time
from math import isfinite

import rclpy
import websocket
from rclpy.node import Node

from cleany_interfaces.srv import CancelMission, GetRuntimeSnapshot, OfferMission
from cleany_control_bridge.session import BridgeSession


class ControlBridge(Node):
    def __init__(self) -> None:
        super().__init__("control_bridge")
        self.declare_parameter("url", "ws://127.0.0.1:8080/api/robots/cleany-01/gateway/ws")
        self.declare_parameter("journal_path", "~/.local/state/cleany/control-bridge.db")
        self.declare_parameter("rpc_timeout", 3.0)
        self.declare_parameter("heartbeat_interval", 1.0)
        self.declare_parameter("reconnect_initial", 1.0)
        self.declare_parameter("reconnect_max", 30.0)
        self.url = self.get_parameter("url").value
        self.rpc_timeout = float(self.get_parameter("rpc_timeout").value)
        self.interval = float(self.get_parameter("heartbeat_interval").value)
        self.reconnect_initial = float(self.get_parameter("reconnect_initial").value)
        self.reconnect_max = float(self.get_parameter("reconnect_max").value)
        if any(not isfinite(v) or v <= 0 for v in (
            self.rpc_timeout, self.interval, self.reconnect_initial, self.reconnect_max,
        )) or self.reconnect_max < self.reconnect_initial:
            raise ValueError("bridge timing parameters must be positive")
        self.session = BridgeSession(self.get_parameter("journal_path").value)
        self.offer_client = self.create_client(OfferMission, "mission/offer")
        self.cancel_client = self.create_client(CancelMission, "mission/cancel")
        self.snapshot_client = self.create_client(GetRuntimeSnapshot, "mission/snapshot")
        self.stop_event = threading.Event()
        self.socket = None
        self.synced = False
        self.snapshot_event_id = ""
        self.snapshot_sent_at = 0.0
        self.deferred_commands = {}
        self.worker = threading.Thread(target=self._run, name="control-gateway", daemon=True)
        self.worker.start()

    def _rpc(self, client, request):
        if not client.wait_for_service(timeout_sec=self.rpc_timeout):
            raise TimeoutError("mission runtime service unavailable")
        future = client.call_async(request)
        done = threading.Event()
        future.add_done_callback(lambda _: done.set())
        if not done.wait(self.rpc_timeout):
            # Delivery/acceptance is uncertain. Reconnect and reconcile; never invent rejection.
            raise TimeoutError("mission runtime response unavailable")
        return future.result()

    def _observe(self):
        response = self._rpc(self.snapshot_client, GetRuntimeSnapshot.Request())
        self.session.observe(json.loads(response.snapshot_json))

    def _command(self, event):
        self.session.validate_command(event)
        kind = event["event_type"]
        if kind == "ack":
            if event["payload"]["ack_event_id"] == self.snapshot_event_id:
                if event["payload"]["disposition"] == "conflict":
                    raise RuntimeError("Backend snapshot conflict; reconciliation required")
                self.synced = True
            self.session.acknowledge(event["payload"]["ack_event_id"])
            return None
        if kind == "sync.request":
            self.interval = event["payload"]["heartbeat_interval_seconds"]
            self._observe()
            return self.session.sync_event()
        if self.session.command_seen(event["event_id"]):
            return self.session.command_done(event)
        if not self.synced:
            if len(self.deferred_commands) >= 8:
                raise RuntimeError("too many commands before snapshot acknowledgement")
            self.deferred_commands[event["event_id"]] = event
            return None
        mid = event["mission_id"]
        if kind == "mission.offer":
            payload = event["payload"]
            response = self._rpc(self.offer_client, OfferMission.Request(
                mission_id=mid, mission_type="clean_desk", target_kind="SEAT",
                target_id=payload["target_id"], requested_by=payload["requested_by"],
            ))
            if not response.accepted:
                self.session.enqueue("mission.rejected", mid, {
                    "message": response.reason,
                    "retryable": response.reason in (
                        "BUSY", "NOT_STOPPED", "NAV2_NOT_READY", "ODOMETRY_STALE",
                        "LOCALIZATION_STALE", "LOCALIZATION_UNAVAILABLE",
                    ),
                })
            elif response.report_json:
                self.session.result(json.loads(response.report_json),
                                    self.session.snapshot["execution_profile"])
            else:
                self.session.accepted(mid)
        else:
            response = self._rpc(self.cancel_client, CancelMission.Request(mission_id=mid))
            if not response.accepted:
                raise RuntimeError("cancel not acknowledged by runtime: " + response.reason)
        self._observe()
        return self.session.command_done(event)

    def _send_snapshot(self, event):
        self.synced = False
        self.snapshot_event_id = event["event_id"]
        self.snapshot_sent_at = time.monotonic()
        self.socket.send(json.dumps(event))

    def _run(self):
        retry = self.reconnect_initial
        while not self.stop_event.is_set():
            try:
                self._observe()
                self.socket = websocket.create_connection(self.url, timeout=self.rpc_timeout)
                self.socket.settimeout(.2)
                self.deferred_commands.clear()
                self._send_snapshot(self.session.sync_event())
                retry = self.reconnect_initial
                next_heartbeat = 0.0
                while not self.stop_event.is_set():
                    if time.monotonic() >= next_heartbeat:
                        self._observe()
                        if self.synced:
                            for event in self.session.pending():
                                self.socket.send(json.dumps(event))
                            # Apply phase evidence first, then reconcile terminal availability.
                            self._send_snapshot(self.session.sync_event())
                        elif time.monotonic() - self.snapshot_sent_at > self.rpc_timeout:
                            raise TimeoutError("Backend snapshot acknowledgement unavailable")
                        self.socket.send(json.dumps(self.session.heartbeat()))
                        next_heartbeat = time.monotonic() + self.interval
                    try:
                        raw = self.socket.recv()
                    except websocket.WebSocketTimeoutException:
                        continue
                    if not raw:
                        break
                    reply = self._command(json.loads(raw))
                    if reply:
                        if reply["event_type"] == "robot.snapshot":
                            self._send_snapshot(reply)
                        else:
                            self.socket.send(json.dumps(reply))
                    if self.synced:
                        for event_id in list(self.deferred_commands):
                            command = self.deferred_commands.pop(event_id)
                            reply = self._command(command)
                            if reply:
                                self.socket.send(json.dumps(reply))
            except Exception as exc:
                if not self.stop_event.is_set():
                    self.get_logger().warning(f"Gateway reconnect: {exc}")
            finally:
                if self.socket:
                    self.socket.close()
                    self.socket = None
            self.stop_event.wait(retry)
            retry = min(retry * 2, self.reconnect_max)

    def destroy_node(self):
        self.stop_event.set()
        if self.socket:
            self.socket.close()
        self.worker.join(timeout=2 * self.rpc_timeout + 1)
        if not self.worker.is_alive():
            self.session.close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ControlBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
