"""Nonblocking System Supervisor lease client for the Mission Runtime."""

import time

from cleany_interfaces.msg import SystemCapabilities, SystemHeartbeat, SystemState
from cleany_interfaces.srv import CheckPermission
from rclpy.qos import DurabilityPolicy, QoSProfile

from cleany_mission_manager.core.supervision import Authorization

OPERABLE = ("OPERATIONAL", "DEGRADED")


class RosSupervision:
    """Acquire, renew and release one Supervisor lease without blocking the tick.

    Subscriptions:
        system/heartbeat (cleany_interfaces/msg/SystemHeartbeat): Liveness and epoch.
        system/state (cleany_interfaces/msg/SystemState): Operating state and
            revocations. QoS: reliable, transient-local depth 1.
        system/capabilities (cleany_interfaces/msg/SystemCapabilities): Availability
            used for the admission precheck. QoS: reliable, transient-local depth 1.

    Services:
        system/check_permission (cleany_interfaces/srv/CheckPermission): Client.

    Notes:
        Retained snapshots never grant authority: the lease deadline counts from
        request dispatch on the local monotonic clock and a stale heartbeat or a
        new epoch invalidates it. A late reply cannot revive an expired lease.
    """

    def __init__(self, node, capabilities: list[str], *, heartbeat_timeout: float,
                 request_timeout: float) -> None:
        if not capabilities or any(not name.strip() for name in capabilities):
            raise ValueError("supervision_capabilities must be a non-empty list")
        self.capabilities = sorted(set(capabilities))
        self.heartbeat_timeout, self.request_timeout = heartbeat_timeout, request_timeout
        self.client = node.create_client(CheckPermission, "system/check_permission")
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        node.create_subscription(SystemHeartbeat, "system/heartbeat", self._heartbeat, 1)
        node.create_subscription(SystemState, "system/state", self._state, latched)
        node.create_subscription(SystemCapabilities, "system/capabilities", self._capabilities, latched)
        self.epoch, self.heartbeat_at, self.operating = "", float("-inf"), ""
        self.available_capabilities: set[str] = set()
        self.mission_id = ""
        self.lease_id, self.version, self.lease_epoch = "", 0, ""
        self.deadline = self.renew_at = 0.0
        self.pending = None
        self.pending_sent = 0.0
        self.denied = ""

    def _heartbeat(self, message) -> None:
        if message.epoch != self.epoch:
            # A Supervisor restart invalidates every lease of the previous epoch.
            self.epoch, self.operating = message.epoch, ""
            self.available_capabilities = set()
            self.deadline = 0.0
        self.heartbeat_at = time.monotonic()

    def _state(self, message) -> None:
        if message.epoch != self.epoch:
            return
        self.operating = message.operating_state
        for permission in message.permissions:
            if permission.lease_id == self.lease_id and self.lease_id and not permission.granted:
                self.deadline = 0.0
                self.denied = permission.reason or "lease revoked"

    def _capabilities(self, message) -> None:
        if message.epoch == self.epoch:
            self.available_capabilities = {c.name for c in message.capabilities if c.status == "AVAILABLE"}

    def _alive(self) -> bool:
        return bool(self.epoch) and time.monotonic() - self.heartbeat_at < self.heartbeat_timeout

    def available(self) -> tuple[bool, str]:
        if not self._alive():
            return False, "SUPERVISOR_UNAVAILABLE"
        if self.operating not in OPERABLE:
            return False, "SYSTEM_" + (self.operating or "UNKNOWN")
        missing = [name for name in self.capabilities if name not in self.available_capabilities]
        if missing:
            return False, "CAPABILITY_UNAVAILABLE:" + ",".join(missing)
        if self.mission_id:
            return False, "LEASE_ACTIVE"
        return True, ""

    def acquire(self, mission_id: str) -> None:
        self.mission_id, self.denied = mission_id, ""
        self.lease_id, self.version, self.lease_epoch, self.deadline = "", 0, "", 0.0
        self._send(acquire=True)

    def _send(self, *, acquire=False, release=False) -> None:
        if not self.client.service_is_ready() or not self.epoch:
            return
        request = CheckPermission.Request(
            epoch=self.epoch, mission_id=self.mission_id, capabilities=self.capabilities,
            lease_id="" if acquire else self.lease_id, version=0 if acquire else self.version,
            release=release)
        sent_at, identity = time.monotonic(), (self.mission_id, self.lease_id, self.version)
        future = self.client.call_async(request)
        if release:
            return  # Authority is already gone locally; the reply is informational.
        self.pending, self.pending_sent = future, sent_at
        future.add_done_callback(lambda done: self._reply(done, sent_at, identity, acquire))

    def _reply(self, future, sent_at: float, identity: tuple, acquire: bool) -> None:
        if future is not self.pending:
            return
        self.pending = None
        if identity != (self.mission_id, self.lease_id, self.version):
            return
        try:
            response = future.result()
        except Exception as exc:
            self.denied, self.deadline = f"permission service failed: {exc}", 0.0
            return
        if not response.granted or response.epoch != self.epoch:
            self.denied, self.deadline = response.reason or "permission denied", 0.0
            return
        if not acquire and time.monotonic() >= self.deadline:
            return  # Expired locally while the renewal was in flight.
        self.lease_id, self.version, self.lease_epoch = response.lease_id, response.version, response.epoch
        # Network delay can only shorten the lifetime.
        self.deadline = sent_at + response.valid_for_s
        self.renew_at = sent_at + response.valid_for_s / 3

    def _valid(self) -> bool:
        return (bool(self.lease_id) and self.lease_epoch == self.epoch and self._alive()
                and time.monotonic() < self.deadline)

    def authorization(self) -> tuple[Authorization, str]:
        if self._valid():
            return Authorization.GRANTED, ""
        if self.denied:
            return Authorization.DENIED, self.denied
        if self.lease_id:
            return Authorization.DENIED, "lease expired or Supervisor unavailable"
        if not self.mission_id:
            return Authorization.DENIED, "no lease requested"
        return Authorization.PENDING, "waiting for Supervisor lease"

    def maintain(self) -> None:
        now = time.monotonic()
        if self.pending is not None and now - self.pending_sent > self.request_timeout:
            self.client.remove_pending_request(self.pending)
            self.pending = None
        if not self.mission_id or self.pending is not None or self.denied:
            return
        if not self.lease_id:
            self._send(acquire=True)  # The service was not ready at admission.
        elif self._valid() and now >= self.renew_at:
            self._send()

    def release(self) -> None:
        if self._valid():
            self._send(release=True)
        self._clear()

    def abandon(self) -> None:
        self._clear()

    def _clear(self) -> None:
        self.mission_id, self.lease_id, self.version, self.lease_epoch = "", "", 0, ""
        self.deadline, self.pending, self.denied = 0.0, None, ""
