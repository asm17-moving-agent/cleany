"""Bounded component-owned recovery, serialized to prevent conflicting resets."""
from uuid import uuid4

from .models import RecoveryJob, RecoveryState


class RecoveryCoordinator:
    def __init__(self, config: dict, port):
        self.config, self.port = config, port
        self.jobs: dict[str, RecoveryJob] = {}

    def request(self, fault, now: float) -> RecoveryJob:
        if not fault.active or not fault.recovery_policy:
            raise ValueError('active fault has no component recovery policy')
        # A single fault episode gets one bounded budget, including manual requests.
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
                pass  # Keep resource held until terminal result, including on transport loss.

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
                    # Never dispatch another reset while the old operation might still run.
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
                job.cancel_requested = False  # Timeout may retry, explicit user cancel may not.
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
