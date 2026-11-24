from uuid import uuid4

from .models import Availability, Lease, OperatingState, PermissionDecision


class MissionPermissionManager:
    def __init__(self, epoch: str, config: dict):
        self.epoch, self.config = epoch, config
        self.leases: dict[str, Lease] = {}
        self.version = 0

    def check(self, *, epoch: str, mission_id: str, capabilities: list[str], lease_id: str,
              version: int, release: bool, states: dict, operating: OperatingState,
              now: float) -> PermissionDecision:
        def denied(reason):
            return PermissionDecision(False, self.epoch, reason=reason)
        if epoch != self.epoch:
            return denied('supervisor epoch changed; revalidate mission before reacquiring')
        existing = self.leases.get(lease_id)
        if existing and (existing.mission_id != mission_id or existing.version != version):
            return denied('lease identity or version mismatch')
        if release:
            if existing and existing.granted:
                # Mission이 완료 또는 정지를 확인한 후에만 반환이 유효합니다.
                existing.granted, existing.stopped, existing.ended_at = False, True, now
                existing.reason = 'released after mission completion'
            return denied('released' if existing else 'unknown lease')
        if not mission_id or not capabilities or any(cap not in states for cap in capabilities):
            return denied('mission identity and known nonempty capability set required')
        requested = tuple(sorted(set(capabilities)))
        if existing and requested != existing.capabilities:
            return denied('renewal cannot change required capabilities')
        if lease_id and (not existing or not existing.granted or now >= existing.expires_at):
            return denied('lease expired or revoked; explicit mission revalidation required')
        if operating not in (OperatingState.OPERATIONAL, OperatingState.DEGRADED):
            return denied('operation is ' + operating.value)
        if any(states[cap] != Availability.AVAILABLE for cap in requested):
            return denied('required capability unavailable or unknown')
        if not existing:
            if any(lease.mission_id == mission_id and not lease.stopped for lease in self.leases.values()):
                return denied('previous permission needs confirmed mission stop/completion')
            self.version += 1
            existing = Lease(mission_id, uuid4().hex, self.version, requested, now)
            self.leases[existing.lease_id] = existing
        existing.expires_at = now + self.config['lease_s']
        return PermissionDecision(True, self.epoch, existing.lease_id, existing.version,
                                  self.config['lease_s'])

    def update(self, states, operating, now: float, stop_port, faults) -> None:
        for lease in self.leases.values():
            if lease.granted:
                reason = ('lease expired' if now >= lease.expires_at else
                          'operation locked' if operating not in (
                              OperatingState.OPERATIONAL, OperatingState.DEGRADED) else
                          'required capability lost' if any(
                              states[cap] != Availability.AVAILABLE for cap in lease.capabilities) else '')
                if reason:
                    lease.granted, lease.reason = False, reason
            if lease.granted or lease.stopped:
                continue
            if lease.stop_requested_at is None:
                lease.stop_requested_at = now
            if not lease.stop_dispatched:
                # 재시작 직후처럼 정지 서버가 아직 발견되지 않을 수 있으므로
                # 즉시 실패하지 않고 정지 마감 시각 안에서 재시도합니다.
                try:
                    stop_port.request(lease, self.epoch)
                    lease.stop_dispatched = True
                except Exception as exc:
                    if now - lease.stop_requested_at >= self.config['stop_timeout_s']:
                        faults.escalate_stop(lease.lease_id, now, str(exc))
                    continue
            try:
                status = stop_port.poll(lease.lease_id)
            except Exception:
                status = 'FAILED'
            lease.stop_accepted |= status in ('ACCEPTED', 'STOPPED')
            if status == 'STOPPED':
                lease.stopped, lease.ended_at = True, now
                faults.confirm_stop(lease.lease_id, now)
            elif status == 'FAILED' or now - lease.stop_requested_at >= self.config['stop_timeout_s']:
                faults.escalate_stop(lease.lease_id, now, 'mission stop failed or timed out')

    def persisted(self) -> list[dict]:
        """허가 여부와 무관하게 미확인 임대는 재시작 후 다시 정지해야 합니다."""
        return [{'mission_id': lease.mission_id, 'lease_id': lease.lease_id,
                 'capabilities': list(lease.capabilities)}
                for lease in self.leases.values() if not lease.stopped]

    def restore(self, entries: list[dict]) -> None:
        for entry in entries:
            # 버전 0은 발급하지 않으므로 복구된 임대를 갱신할 수 없습니다.
            self.leases[entry['lease_id']] = Lease(
                entry['mission_id'], entry['lease_id'], 0, tuple(entry.get('capabilities', ())), 0., granted=False,
                reason='restored unconfirmed lease from previous supervisor process', restored=True)

    def prune(self, now: float, retention_s: float) -> list[str]:
        removed = [lease_id for lease_id, lease in self.leases.items()
                   if lease.stopped and lease.ended_at is not None and now - lease.ended_at >= retention_s]
        for lease_id in removed:
            del self.leases[lease_id]
        return removed
