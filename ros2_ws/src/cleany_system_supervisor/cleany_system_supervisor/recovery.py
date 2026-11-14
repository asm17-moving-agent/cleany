"""충돌하는 reset을 막도록 직렬화한 범위 제한 컴포넌트 소유 복구입니다."""
from uuid import uuid4

from .models import RecoveryJob, RecoveryState

TERMINAL = (RecoveryState.SUCCEEDED, RecoveryState.FAILED, RecoveryState.CANCELED)


class UnsupportedRecovery(ValueError):
    """이 fault의 복구를 구현한 소유자가 없으며 재시도 가능한 실패가 아닙니다."""


class RecoveryCoordinator:
    def __init__(self, config: dict, port):
        self.config, self.port = config, port
        self.jobs: dict[str, RecoveryJob] = {}

    def request(self, fault, now: float) -> RecoveryJob:
        if not fault.active:
            raise ValueError('fault is not active')
        if not fault.recovery_policy:
            raise UnsupportedRecovery(f'{fault.code} has no component-owned recovery; operator action required')
        # 수동 요청을 포함해 하나의 fault 구간에 하나의 제한된 예산을 적용합니다.
        for job in self.jobs.values():
            if job.fault_code == fault.code and job.started_at >= fault.first_seen:
                return job
        policy = self.config['recovery_policies'][fault.recovery_policy]
        job = RecoveryJob(uuid4().hex, fault.code, fault.source, policy.copy(), started_at=now)
        self.jobs[job.job_id] = job
        return job

    def cancel(self, job: RecoveryJob, now: float) -> None:
        job.cancel_requested = True
        if not job.resource_held:
            job.state, job.detail = RecoveryState.CANCELED, 'canceled before dispatch'
        elif job.cancel_at is None:
            job.cancel_at = now
            try:
                self.port.cancel(job)
            except Exception:
                pass  # 전송 유실 중에도 종료 결과까지 리소스를 점유합니다.

    def _retry_or_fail(self, job, now, detail):
        job.resource_held, job.verify_after, job.cancel_at = False, None, None
        job.detail = detail
        if job.cancel_requested:
            job.state = RecoveryState.CANCELED
        elif (job.attempt >= job.policy['max_attempts'] or
              now - job.started_at >= job.policy['total_timeout_s']):
            job.state = RecoveryState.FAILED
        else:
            job.state = RecoveryState.IDLE

    def update(self, faults, health, now: float, allow_dispatch: bool, allow_continue: bool = True) -> None:
        self._dispatch(faults, health, now, allow_dispatch, allow_continue)
        for job in self.jobs.values():
            if job.state in TERMINAL and not job.resource_held and job.finished_at is None:
                job.finished_at = now

    def prune(self, faults, now: float, retention_s: float) -> list[RecoveryJob]:
        """예산을 유지하도록 fault 구간이 끝난 뒤에만 완료 작업을 잊습니다."""
        removed = []
        for job in list(self.jobs.values()):
            fault = faults.get(job.fault_code)
            episode_over = fault is None or not fault.active or fault.first_seen > job.started_at
            if (job.finished_at is not None and episode_over and not job.resource_held
                    and now - job.finished_at >= retention_s):
                removed.append(self.jobs.pop(job.job_id))
        return removed

    def _dispatch(self, faults, health, now, allow_dispatch, allow_continue) -> None:
        for fault in faults.values():
            if (fault.active and fault.recovery_policy and
                    self.config['recovery_policies'][fault.recovery_policy]['automatic']):
                self.request(fault, now)
        for job in self.jobs.values():
            if not job.resource_held:
                if job.state == RecoveryState.IDLE and now - job.started_at >= job.policy['total_timeout_s']:
                    job.state, job.detail = RecoveryState.FAILED, 'total timeout while queued'
                continue
            try:
                result = self.port.poll(job)
            except Exception:
                result = 'UNKNOWN'
            if job.cancel_at is not None:
                if result in ('SUCCEEDED', 'FAILED', 'CANCELED'):
                    self._retry_or_fail(job, now, 'component completed after cancel/timeout')
                elif now - job.cancel_at >= job.policy['cancel_timeout_s']:
                    # 이전 작업이 아직 실행 중일 수 있으면 다른 reset을 보내지 않습니다.
                    job.state, job.detail = RecoveryState.FAILED, 'cancel unconfirmed; resource quarantined'
                continue
            if job.verify_after is not None:
                if health.healthy(job.source, now, after=job.verify_after):
                    job.state, job.resource_held = RecoveryState.SUCCEEDED, False
                    job.detail = 'component result verified with fresh stable health'
                elif (now - job.verify_after >= job.policy['verify_timeout_s'] or
                      now - job.started_at >= job.policy['total_timeout_s']):
                    self._retry_or_fail(job, now, 'post-recovery health verification timed out')
                continue
            if result == 'SUCCEEDED':
                job.verify_after = now
            elif result in ('FAILED', 'CANCELED'):
                self._retry_or_fail(job, now, 'component recovery failed')
            elif (now - job.attempt_at >= job.policy['attempt_timeout_s'] or
                  now - job.started_at >= job.policy['total_timeout_s']):
                self.cancel(job, now)
                job.cancel_requested = False  # 시간 초과는 재시도할 수 있지만 명시적 사용자 취소는 재시도하지 않습니다.
            elif not allow_continue:
                self.cancel(job, now)
        if any(job.resource_held for job in self.jobs.values()) or not allow_dispatch:
            return
        pending = [job for job in self.jobs.values() if job.state == RecoveryState.IDLE]
        priorities = {'CRITICAL': 0, 'ERROR': 1, 'WARNING': 2}
        pending.sort(key=lambda job: (priorities[faults[job.fault_code].severity], job.started_at))
        for job in pending[:1]:
            if health.healthy(job.source, now):
                job.state, job.detail = RecoveryState.SUCCEEDED, 'health recovered before dispatch'
                continue
            job.attempt += 1
            job.state, job.resource_held, job.attempt_at = RecoveryState.RUNNING, True, now
            try:
                self.port.start(job)
            except Exception as exc:
                self._retry_or_fail(job, now, str(exc))
