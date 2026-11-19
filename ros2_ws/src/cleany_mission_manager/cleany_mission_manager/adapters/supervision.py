"""Mission Runtime을 위한 비블로킹 System Supervisor 임대 client입니다."""

import time

from cleany_interfaces.msg import SystemCapabilities, SystemHeartbeat, SystemState
from cleany_interfaces.srv import CheckPermission
from rclpy.qos import DurabilityPolicy, QoSProfile

from cleany_mission_manager.core.supervision import Authorization

OPERABLE = ("OPERATIONAL", "DEGRADED")


class RosSupervision:
    """tick을 막지 않고 Supervisor 임대 하나를 취득, 갱신하고 반환합니다.

    Subscriptions:
        system/heartbeat (cleany_interfaces/msg/SystemHeartbeat): 생존 상태와 epoch.
        system/state (cleany_interfaces/msg/SystemState): 운영 상태와 권한 회수.
            QoS는 reliable, transient_local, depth 1입니다.
        system/capabilities (cleany_interfaces/msg/SystemCapabilities):
            수락 사전 검사에 사용하는 가용성. QoS는 reliable, transient_local, depth 1입니다.

    Services:
        system/check_permission (cleany_interfaces/srv/CheckPermission): Client.

    Notes:
        보관된 snapshot 자체는 권한을 부여하지 않습니다. 임대 마감은 로컬 단조 시계의
        요청 전송 시점부터 계산하며 오래된 heartbeat나 새 epoch는 임대를 무효화합니다.
        늦은 응답은 만료된 임대를 되살리지 못합니다.
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
            # Supervisor가 재시작하면 이전 epoch의 모든 임대가 무효화됩니다.
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
            return  # 로컬에서는 이미 권한이 사라졌으므로 응답은 정보 확인용입니다.
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
            return  # 갱신 요청이 진행되는 동안 로컬에서 만료되었습니다.
        self.lease_id, self.version, self.lease_epoch = response.lease_id, response.version, response.epoch
        # 네트워크 지연은 유효 시간을 줄일 수만 있습니다.
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
            self._send(acquire=True)  # 수락 시점에 서비스가 준비되지 않았습니다.
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
